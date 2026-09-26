#!/usr/bin/env python3
"""
YOLOv8/YOLO11 Real-Time Instance Segmentation & Hazard Mapping Node.
Subscribes:
  - /camera/image_raw (sensor_msgs/msg/Image)
Publishes:
  - /perception/hazard_mask (sensor_msgs/msg/Image, mono8: 0=Safe, 128=Caution, 255=Hazard)
  - /perception/yolo/overlay (sensor_msgs/msg/Image, bgr8 visualization)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import numpy as np
import cv2
from cv_bridge import CvBridge
import torch

from sensor_msgs.msg import Image


class YoloHazardSegmentationNode(Node):
    def __init__(self):
        super().__init__('yolo_seg_node')

        # Parameters
        self.declare_parameter('input_image_topic', '/camera/image_raw')
        self.declare_parameter('output_mask_topic', '/perception/hazard_mask')
        self.declare_parameter('output_overlay_topic', '/perception/yolo/overlay')
        self.declare_parameter('model_name', 'yolov8n-seg.pt')
        self.declare_parameter('confidence_threshold', 0.35)
        self.declare_parameter('device', 'cuda:0' if torch.cuda.is_available() else 'cpu')
        self.declare_parameter('enable_fp16', True)

        self.input_image_topic = self.get_parameter('input_image_topic').value
        self.output_mask_topic = self.get_parameter('output_mask_topic').value
        self.output_overlay_topic = self.get_parameter('output_overlay_topic').value
        self.model_name = self.get_parameter('model_name').value
        self.conf_thresh = float(self.get_parameter('confidence_threshold').value)
        self.device_str = self.get_parameter('device').value
        self.enable_fp16 = bool(self.get_parameter('enable_fp16').value)

        self.bridge = CvBridge()

        # Initialize YOLO
        self.get_logger().info(f'Loading YOLO model: {self.model_name} on device: {self.device_str}...')
        try:
            from ultralytics import YOLO
            self.model = YOLO(self.model_name)
            self.model.to(self.device_str)
            self.yolo_available = True
            self.get_logger().info('YOLO segmentation model loaded successfully.')
        except Exception as e:
            self.get_logger().warn(f'Could not load YOLO model: {e}. Running in robust fallback hazard detector mode.')
            self.yolo_available = False

        # QoS Profiles
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscriber
        self.image_sub = self.create_subscription(
            Image,
            self.input_image_topic,
            self.image_callback,
            sensor_qos
        )

        # Publishers
        self.mask_pub = self.create_publisher(Image, self.output_mask_topic, 10)
        self.overlay_pub = self.create_publisher(Image, self.output_overlay_topic, 10)

        # Classes that represent hazards in off-road outdoor scenarios
        self.hazard_keywords = {
            'rock', 'stone', 'boulder', 'tree', 'trunk', 'log', 'ditch', 'pothole',
            'car', 'truck', 'bus', 'person', 'dog', 'bench', 'chair', 'obstacle'
        }

    def fallback_hazard_detection(self, cv_image: np.ndarray) -> np.ndarray:
        """Heuristic obstacle/contrast detection when ML model is warming up."""
        h, w = cv_image.shape[:2]
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        
        # Binary mask initialized to 0 (traversable)
        hazard_mask = np.zeros((h, w), dtype=np.uint8)

        # Detect salient edge clusters in lower 65% of the image (ground plane)
        lower_region = gray[int(h * 0.35):, :]
        edges = cv2.Canny(lower_region, 80, 180)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (9, 9))
        dilated = cv2.dilate(edges, kernel)
        
        # Place detected obstacle edges as lethal hazards (255)
        hazard_mask[int(h * 0.35):, :] = np.where(dilated > 0, 255, 0).astype(np.uint8)
        return hazard_mask

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
                    half=self.enable_fp16 and ('cuda' in self.device_str),
                    verbose=False
                )

                if results and len(results) > 0 and results[0].masks is not None:
                    res = results[0]
                    masks_data = res.masks.data.cpu().numpy()
                    classes = res.boxes.cls.cpu().numpy().astype(int)
                    names = res.names

                    for i, cls_id in enumerate(classes):
                        class_name = names.get(cls_id, str(cls_id)).lower()
                        mask_resized = cv2.resize(masks_data[i], (w, h), interpolation=cv2.INTER_NEAREST)

                        # Determine if this class is a hazard or caution
                        is_hazard = any(k in class_name for k in self.hazard_keywords) or (class_name not in {'road', 'trail', 'grass'})
                        
                        if is_hazard:
                            hazard_mask[mask_resized > 0.5] = 255
                            # Red overlay for lethal hazards
                            overlay[mask_resized > 0.5] = (
                                overlay[mask_resized > 0.5] * 0.5 + np.array([0, 0, 255]) * 0.5
                            ).astype(np.uint8)
                        else:
                            # Green overlay for traversable regions
                            overlay[mask_resized > 0.5] = (
                                overlay[mask_resized > 0.5] * 0.7 + np.array([0, 255, 0]) * 0.3
                            ).astype(np.uint8)

                else:
                    # No masks detected, run fallback heuristic
                    hazard_mask = self.fallback_hazard_detection(cv_image)

            except Exception as e:
                self.get_logger().error(f'YOLO inference error: {e}')
                hazard_mask = self.fallback_hazard_detection(cv_image)
        else:
            hazard_mask = self.fallback_hazard_detection(cv_image)

        # Publish hazard mask (mono8)
        mask_msg = self.bridge.cv2_to_imgmsg(hazard_mask, encoding='mono8')
        mask_msg.header = msg.header
        self.mask_pub.publish(mask_msg)

        # Publish visualization overlay (bgr8)
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
