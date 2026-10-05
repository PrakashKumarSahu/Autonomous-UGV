#!/usr/bin/env python3
"""
YOLO26 Real-Time Instance Segmentation & Hazard Mapping Node.

Model: YOLO26n-seg (Ultralytics, NMS-free, COCO-80 classes). The weights file
(`yolo26n-seg.pt`, ~6.4 MB) is downloaded automatically by Ultralytics on first
use into the current working directory / Ultralytics cache.

Subscribes:
  - input_image_topic   (sensor_msgs/Image, default /camera/image_raw)
Publishes:
  - output_mask_topic    (sensor_msgs/Image, mono8: 0=safe, 128=caution, 255=lethal)
  - output_overlay_topic (sensor_msgs/Image, bgr8 visualization for RViz)

Hazard classification (configurable via parameters, see config/perception.yaml):
  - class name contains any `lethal_classes` keyword  → 255 (red overlay)
  - class name contains any `caution_classes` keyword → 128 (amber overlay)
  - any other detected class                          → `unknown_class_cost`
If the model cannot be loaded, a Canny-edge heuristic fallback is used so the
downstream terrain node still receives a mask.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import numpy as np
import cv2
from cv_bridge import CvBridge
import torch

from sensor_msgs.msg import Image


DEFAULT_LETHAL = [
    'person', 'car', 'truck', 'bus', 'motorcycle', 'bicycle', 'dog', 'horse',
    'cow', 'sheep', 'bench', 'chair', 'fire hydrant', 'stop sign',
    'rock', 'stone', 'boulder', 'tree', 'trunk', 'log', 'ditch', 'pothole', 'obstacle',
]
DEFAULT_CAUTION = ['potted plant', 'bush', 'shrub', 'vegetation', 'backpack', 'suitcase']

LETHAL_COLOR = np.array([0, 0, 255], dtype=np.float32)     # red   (BGR)
CAUTION_COLOR = np.array([0, 165, 255], dtype=np.float32)  # amber (BGR)


class YoloHazardSegmentationNode(Node):
    def __init__(self):
        super().__init__('yolo_seg_node')

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter('input_image_topic', '/camera/image_raw')
        self.declare_parameter('output_mask_topic', '/perception/hazard_mask')
        self.declare_parameter('output_overlay_topic', '/perception/yolo/overlay')
        self.declare_parameter('model_name', 'yolo26n-seg.pt')
        self.declare_parameter('confidence_threshold', 0.35)
        self.declare_parameter('device', 'cuda:0' if torch.cuda.is_available() else 'cpu')
        self.declare_parameter('enable_fp16', True)
        self.declare_parameter('lethal_classes', DEFAULT_LETHAL)
        self.declare_parameter('caution_classes', DEFAULT_CAUTION)
        self.declare_parameter('unknown_class_cost', 128)

        self.input_image_topic = self.get_parameter('input_image_topic').value
        self.output_mask_topic = self.get_parameter('output_mask_topic').value
        self.output_overlay_topic = self.get_parameter('output_overlay_topic').value
        self.model_name = self.get_parameter('model_name').value
        self.conf_thresh = float(self.get_parameter('confidence_threshold').value)
        self.device_str = self.get_parameter('device').value
        if self.device_str != 'cpu' and not torch.cuda.is_available():
            self.get_logger().warn(f'CUDA unavailable — falling back from {self.device_str} to cpu')
            self.device_str = 'cpu'
        self.enable_fp16 = bool(self.get_parameter('enable_fp16').value)
        self.lethal_classes = [c.lower() for c in self.get_parameter('lethal_classes').value]
        self.caution_classes = [c.lower() for c in self.get_parameter('caution_classes').value]
        self.unknown_cost = int(np.clip(self.get_parameter('unknown_class_cost').value, 0, 255))

        self.bridge = CvBridge()

        # ── Model ───────────────────────────────────────────────────────────
        self.get_logger().info(f'Loading YOLO model: {self.model_name} on {self.device_str}...')
        try:
            from ultralytics import YOLO
            self.model = YOLO(self.model_name)
            self.model.to(self.device_str)
            # FP16 is applied on the underlying torch model. This avoids the
            # deprecated predict(half=True) argument and works on all 8.x releases.
            if self.enable_fp16 and self.device_str != 'cpu':
                self.model.model.half()
            self.yolo_available = True
            self.get_logger().info(
                f'YOLO segmentation model loaded ({len(self.model.names)} classes, '
                f'fp16={self.enable_fp16 and self.device_str != "cpu"}).'
            )
        except Exception as e:
            self.get_logger().warn(f'Could not load YOLO model: {e}. Using edge-based fallback detector.')
            self.yolo_available = False

        # ── I/O ─────────────────────────────────────────────────────────────
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.image_sub = self.create_subscription(
            Image, self.input_image_topic, self.image_callback, sensor_qos)
        self.mask_pub = self.create_publisher(Image, self.output_mask_topic, 10)
        self.overlay_pub = self.create_publisher(Image, self.output_overlay_topic, 10)

    # ── Helpers ─────────────────────────────────────────────────────────────

    def _class_cost(self, class_name: str) -> int:
        name = class_name.lower()
        if any(k in name for k in self.lethal_classes):
            return 255
        if any(k in name for k in self.caution_classes):
            return 128
        return self.unknown_cost

    @staticmethod
    def _blend(overlay: np.ndarray, region: np.ndarray, color: np.ndarray, alpha: float) -> None:
        overlay[region] = (overlay[region] * (1.0 - alpha) + color * alpha).astype(np.uint8)

    def fallback_hazard_detection(self, cv_image: np.ndarray) -> np.ndarray:
        """Canny-edge heuristic used only when the YOLO model is unavailable."""
        h, w = cv_image.shape[:2]
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        hazard_mask = np.zeros((h, w), dtype=np.uint8)
        top = int(h * 0.35)  # ground plane is in the lower 65% of the image
        edges = cv2.Canny(gray[top:, :], 80, 180)
        dilated = cv2.dilate(edges, cv2.getStructuringElement(cv2.MORPH_RECT, (9, 9)))
        hazard_mask[top:, :] = np.where(dilated > 0, 255, 0).astype(np.uint8)
        return hazard_mask

    # ── Callback ────────────────────────────────────────────────────────────

    def image_callback(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Image conversion failed: {e}')
            return

        h, w = cv_image.shape[:2]
        hazard_mask = np.zeros((h, w), dtype=np.uint8)
        overlay = cv_image.copy()

        if self.yolo_available:
            try:
                results = self.model.predict(
                    source=cv_image,
                    conf=self.conf_thresh,
                    device=self.device_str,
                    verbose=False
                )
                res = results[0] if results else None
                if res is not None and res.masks is not None:
                    # Masks may be uint8/float16/float32 depending on version —
                    # cast to float32 (cv2.resize does not support float16).
                    masks_data = res.masks.data.cpu().numpy().astype(np.float32)
                    classes = res.boxes.cls.cpu().numpy().astype(int)
                    for i, cls_id in enumerate(classes):
                        region = cv2.resize(masks_data[i], (w, h),
                                            interpolation=cv2.INTER_NEAREST) > 0.5
                        cost = self._class_cost(res.names.get(cls_id, str(cls_id)))
                        hazard_mask[region] = np.maximum(hazard_mask[region], cost)
                        if cost >= 255:
                            self._blend(overlay, region, LETHAL_COLOR, 0.5)
                        elif cost > 0:
                            self._blend(overlay, region, CAUTION_COLOR, 0.3)
            except Exception as e:
                self.get_logger().error(f'YOLO inference error: {e}', throttle_duration_sec=5.0)
                hazard_mask.fill(0)
        else:
            hazard_mask = self.fallback_hazard_detection(cv_image)
            self._blend(overlay, hazard_mask == 255, LETHAL_COLOR, 0.5)

        mask_msg = self.bridge.cv2_to_imgmsg(hazard_mask, encoding='mono8')
        mask_msg.header = msg.header
        self.mask_pub.publish(mask_msg)

        overlay_msg = self.bridge.cv2_to_imgmsg(overlay, encoding='bgr8')
        overlay_msg.header = msg.header
        self.overlay_pub.publish(overlay_msg)


def main(args=None):
    rclpy.init(args=args)
    node = YoloHazardSegmentationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
