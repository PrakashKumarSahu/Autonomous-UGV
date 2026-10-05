#!/usr/bin/env python3
"""
Depth Anything V2 Metric — Monocular Depth Estimation Node.

Uses the official Depth Anything V2 Metric Indoor model from HuggingFace
(depth-anything/Depth-Anything-V2-Metric-Indoor-Small-hf) which outputs
depth directly in metres — no scale conversion or disparity heuristics needed.

Publishes two topics:
  output_depth_topic  (32FC1, metres) — for Nav2, RTAB-Map, point cloud
  output_color_topic  (bgr8 TURBO)   — for RViz visualization (no black image)

Modes:
  camera_type:=monocular → this IS the depth source for the full pipeline
  enable_depth_viz:=true → this runs alongside Gazebo real depth for comparison

Model: depth-anything/Depth-Anything-V2-Metric-Indoor-Small-hf
  Download: cached by HuggingFace transformers on first run (~300MB)
  GPU: RTX 4050 runs inference at ~15 FPS (frames are skipped if GPU busy)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import numpy as np
import cv2
from cv_bridge import CvBridge
import torch

from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2


# Depth Anything V2 Metric Indoor model — outputs metric depth in metres
METRIC_MODEL = 'depth-anything/Depth-Anything-V2-Metric-Indoor-Small-hf'


class DepthEstimationNode(Node):
    def __init__(self):
        super().__init__('depth_node')

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter('input_image_topic',  '/camera/image_raw')
        self.declare_parameter('camera_info_topic',  '/camera/camera_info')
        self.declare_parameter('output_depth_topic', '/perception/depth/image_raw')
        self.declare_parameter('output_color_topic', '/perception/depth_ai/colorized')
        self.declare_parameter('output_points_topic', '/perception/depth/points')
        self.declare_parameter('depth_frame_id',     'camera_link_optical')
        self.declare_parameter('device', 'cuda:0' if torch.cuda.is_available() else 'cpu')
        self.declare_parameter('min_depth',          0.2)
        self.declare_parameter('max_depth',          10.0)
        self.declare_parameter('colorize_max_depth', 10.0)
        self.declare_parameter('publish_pointcloud', False)
        self.declare_parameter('model_id',           METRIC_MODEL)

        self.input_image_topic  = self.get_parameter('input_image_topic').value
        self.camera_info_topic  = self.get_parameter('camera_info_topic').value
        self.output_depth_topic = self.get_parameter('output_depth_topic').value
        self.output_color_topic = self.get_parameter('output_color_topic').value
        self.output_points_topic = self.get_parameter('output_points_topic').value
        self.depth_frame_id     = self.get_parameter('depth_frame_id').value
        self.device_str         = self.get_parameter('device').value
        self.min_depth          = float(self.get_parameter('min_depth').value)
        self.max_depth          = float(self.get_parameter('max_depth').value)
        self.colorize_max_depth = float(self.get_parameter('colorize_max_depth').value)
        self.publish_pointcloud = bool(self.get_parameter('publish_pointcloud').value)
        self.model_id           = self.get_parameter('model_id').value

        self.bridge = CvBridge()
        self.device = torch.device(self.device_str if torch.cuda.is_available() else 'cpu')
        self.get_logger().info(f'[DepthNode] device={self.device}  model={self.model_id}')

        # Camera intrinsics (updated from live CameraInfo)
        self.fx, self.fy = 450.0, 450.0
        self.cx, self.cy = 320.0, 240.0
        self.camera_info_received = False
        self.latest_camera_info   = None

        # ── QoS ─────────────────────────────────────────────────────────────
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=1
        )
        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST, depth=10
        )

        # ── Subscribers ──────────────────────────────────────────────────────
        self.image_sub = self.create_subscription(Image,      self.input_image_topic,  self.image_callback,       sensor_qos)
        self.info_sub  = self.create_subscription(CameraInfo, self.camera_info_topic,  self.camera_info_callback, sensor_qos)

        # ── Publishers ───────────────────────────────────────────────────────
        self.depth_pub = self.create_publisher(Image, self.output_depth_topic, pub_qos)
        self.color_pub = self.create_publisher(Image, self.output_color_topic, pub_qos)
        # Matching camera_info topic — always the /camera_info sibling of the depth topic
        # e.g. /perception/depth/image_raw → /perception/depth/camera_info
        _depth_ns = self.output_depth_topic.rsplit('/', 1)[0]
        depth_info_topic = _depth_ns + '/camera_info'
        self.depth_info_pub = self.create_publisher(CameraInfo, depth_info_topic, pub_qos)
        if self.publish_pointcloud:
            self.points_pub = self.create_publisher(PointCloud2, self.output_points_topic, pub_qos)

        # ── Load Depth Anything V2 Metric model ──────────────────────────────
        self.depth_pipe   = None
        self.is_metric    = False
        self.is_processing = False
        self._load_model()

        self.get_logger().info(
            f'[DepthNode] Ready — '
            f'{"metric depth (metres)" if self.is_metric else "geometric fallback"}\n'
            f'  raw 32FC1 → {self.output_depth_topic}\n'
            f'  TURBO bgr8 → {self.output_color_topic}'
        )

    def _load_model(self):
        """Load Depth Anything V2 Metric model from HuggingFace cache."""
        try:
            from transformers import pipeline as hf_pipeline
            # Map the `device` parameter ('cpu' | 'cuda' | 'cuda:N') to an HF device index.
            if self.device.type == 'cuda':
                device_id = self.device.index if self.device.index is not None else 0
            else:
                device_id = -1
            self.get_logger().info(f'Loading {self.model_id} (cached in ~/.cache/huggingface/)...')
            self.depth_pipe = hf_pipeline(
                'depth-estimation',
                model=self.model_id,
                device=device_id
            )
            self.is_metric = True
            self.get_logger().info(f'Depth Anything V2 Metric loaded on {"GPU" if device_id >= 0 else "CPU"}')
        except Exception as e:
            self.get_logger().warn(
                f'Could not load Depth Anything model: {e}\n'
                f'  → Using geometric fallback (no GPU required).\n'
                f'  Install: pip install transformers torch torchvision pillow'
            )

    # ── Callbacks ────────────────────────────────────────────────────────────

    def camera_info_callback(self, msg: CameraInfo):
        self.latest_camera_info = msg
        if not self.camera_info_received and len(msg.k) >= 9 and msg.k[0] > 0:
            self.fx, self.fy = float(msg.k[0]), float(msg.k[4])
            self.cx, self.cy = float(msg.k[2]), float(msg.k[5])
            self.camera_info_received = True
            self.get_logger().info(f'Camera intrinsics: fx={self.fx:.1f} fy={self.fy:.1f} cx={self.cx:.1f} cy={self.cy:.1f}')

    def image_callback(self, msg: Image):
        if self.is_processing:
            return  # skip frame — GPU still busy with previous
        self.is_processing = True
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            depth_map = self._estimate_depth(cv_image)

            # ── Publish raw 32FC1 ────────────────────────────────────────────
            depth_msg = self.bridge.cv2_to_imgmsg(depth_map, encoding='32FC1')
            depth_msg.header           = msg.header
            depth_msg.header.frame_id  = self.depth_frame_id
            self.depth_pub.publish(depth_msg)

            # ── Publish TURBO colorized bgr8 ─────────────────────────────────
            colorized = self._colorize(depth_map)
            color_msg = self.bridge.cv2_to_imgmsg(colorized, encoding='bgr8')
            color_msg.header = depth_msg.header
            self.color_pub.publish(color_msg)

            # ── Publish matching CameraInfo ───────────────────────────────────
            info_out = self._make_camera_info(depth_msg)
            self.depth_info_pub.publish(info_out)

            # ── Optional: point cloud ─────────────────────────────────────────
            if self.publish_pointcloud:
                self._publish_cloud(depth_map, msg.header, self.depth_frame_id)
        except Exception as e:
            self.get_logger().error(f'image_callback error: {e}')
        finally:
            self.is_processing = False

    # ── Depth Estimation ─────────────────────────────────────────────────────

    def _estimate_depth(self, cv_image: np.ndarray) -> np.ndarray:
        """
        Estimate depth in metres from RGB image.

        With Depth Anything V2 Metric model: output IS in metres directly.
        Fallback: geometric horizon-based estimation from camera height.
        """
        h, w = cv_image.shape[:2]

        if self.depth_pipe is not None:
            try:
                from PIL import Image as PILImage
                rgb     = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
                pil_img = PILImage.fromarray(rgb)
                out     = self.depth_pipe(pil_img)
                depth   = np.array(out['predicted_depth'], dtype=np.float32)

                if depth.shape != (h, w):
                    depth = cv2.resize(depth, (w, h), interpolation=cv2.INTER_LINEAR)

                if self.is_metric:
                    # Metric model: output already in metres
                    depth = np.clip(depth, self.min_depth, self.max_depth)
                else:
                    # Relative disparity model: convert via EMA-smoothed range
                    d_min, d_max = float(depth.min()), float(depth.max())
                    if d_max > d_min:
                        norm  = (depth - d_min) / (d_max - d_min)
                        depth = self.min_depth + (1.0 - norm) * (self.max_depth - self.min_depth)
                    else:
                        depth = np.full((h, w), 2.5, dtype=np.float32)
                    depth = np.clip(depth, self.min_depth, self.max_depth)

                return depth.astype(np.float32)
            except Exception as e:
                self.get_logger().error(f'Depth inference failed: {e}', throttle_duration_sec=10.0)

        # ── Geometric fallback ────────────────────────────────────────────────
        # Used only when the DA V2 Metric model fails to load.
        # Formula: depth = (fy * camera_height) / (pixel_distance_from_horizon)
        # camera_link_optical height above ground:
        #   wheel_radius(0.1) + base_height/2(0.1) + mast(0.4323) + optical_z(0.0125) ≈ 0.64m
        gray      = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        y_coords  = np.arange(h, dtype=np.float32)[:, None]
        dy        = np.maximum(y_coords - h * 0.42, 1.0)
        depth     = np.repeat((self.fy * 0.64) / dy, w, axis=1)
        edges     = cv2.Canny(gray, 50, 150).astype(np.float32) / 255.0
        depth    -= cv2.GaussianBlur(edges, (15, 15), 0) * 1.5
        return np.clip(depth, self.min_depth, self.max_depth).astype(np.float32)

    # ── Colorization ─────────────────────────────────────────────────────────

    def _colorize(self, depth_f: np.ndarray) -> np.ndarray:
        """
        Convert 32FC1 depth (metres) to bgr8 TURBO colormap for RViz.
        Black = invalid / out-of-range. Dark blue = near. Red = far.
        """
        invalid = ~np.isfinite(depth_f) | (depth_f <= 0.0)
        clean   = np.where(invalid, 0.0, depth_f)
        norm    = (np.clip(clean, 0.0, self.colorize_max_depth) / self.colorize_max_depth * 255).astype(np.uint8)
        colored = cv2.applyColorMap(norm, cv2.COLORMAP_TURBO)
        colored[invalid] = (0, 0, 0)
        return colored

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _make_camera_info(self, depth_msg: Image) -> CameraInfo:
        info = CameraInfo()
        info.header = depth_msg.header
        info.height = depth_msg.height
        info.width  = depth_msg.width
        if self.latest_camera_info is not None:
            info.distortion_model = self.latest_camera_info.distortion_model
            info.d = self.latest_camera_info.d
            info.k = self.latest_camera_info.k
            info.r = self.latest_camera_info.r
            info.p = self.latest_camera_info.p
        else:
            info.distortion_model = 'plumb_bob'
            info.d = [0.0]*5
            info.k = [self.fx, 0.0, self.cx, 0.0, self.fy, self.cy, 0.0, 0.0, 1.0]
            info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
            info.p = [self.fx, 0.0, self.cx, 0.0, 0.0, self.fy, self.cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return info

    def _publish_cloud(self, depth_map: np.ndarray, header: Header, frame_id: str):
        h, w   = depth_map.shape
        step   = 4
        sd     = depth_map[::step, ::step]
        u      = np.arange(0, w, step, dtype=np.float32)
        v      = np.arange(0, h, step, dtype=np.float32)
        uu, vv = np.meshgrid(u, v)
        valid  = (sd >= self.min_depth) & (sd <= self.max_depth)
        x      = (uu - self.cx) * sd / self.fx
        y      = (vv - self.cy) * sd / self.fy
        pts    = np.stack([x[valid], y[valid], sd[valid]], axis=-1)
        hdr    = Header(); hdr.stamp = header.stamp; hdr.frame_id = frame_id
        self.points_pub.publish(pc2.create_cloud_xyz32(hdr, pts))


def main(args=None):
    rclpy.init(args=args)
    node = DepthEstimationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
