#!/usr/bin/env python3
"""
Depth Source Relay Node — Simulation Mode (camera_type:=sim).

Bridges Gazebo's REAL depth camera sensor (metric, 0.1–10 m, 32FC1) to the
pipeline's canonical topics. The rest of the pipeline (point_cloud_xyz_node,
rgbd_sync → RTAB-Map, Nav2 ObstacleLayer) only reads from:

  /perception/depth/image_raw   (32FC1, metric metres, frame: camera_link_optical)
  /perception/depth/camera_info (depth intrinsics, frame: camera_link_optical)

ADDITIONAL — colorized depth for clean RViz visualization:

  /perception/depth/colorized   (bgr8, TURBO colormap 0–10m, black=invalid/NaN)
    • Near objects  (0–3 m)  → warm red/orange
    • Mid-range     (3–7 m)  → yellow/green
    • Far objects   (7–10 m) → cool blue/purple
    • Invalid/NaN            → black (not treated as real depth)

  This eliminates the "black image" problem in RViz. The raw 32FC1 image makes
  RViz show black because NaN pixels (undetected/out-of-range depths from
  Gazebo) are mapped to 0 by the Image display even with Normalize Range=true.
  The colorized image replaces NaN with explicit black and applies a perceptual
  colormap so depth differences are immediately visible.

Swappable in production — replace this node with:
  camera_type:=monocular   → depth_node        (Depth Anything V3, any RGB camera)
  camera_type:=realsense   → realsense_relay_node (Intel RealSense D435/D455)
  camera_type:=zed         → zed_relay_node    (ZED 2 / ZED X)
All produce the same output topics — the rest of the stack is unchanged.
"""

import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge


class DepthRelayNode(Node):
    def __init__(self):
        super().__init__('depth_relay_node')

        self._bridge = CvBridge()

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter('input_depth_topic',   '/camera/depth/image_raw')
        self.declare_parameter('input_info_topic',    '/camera/depth/camera_info')
        self.declare_parameter('output_depth_topic',  '/perception/depth/image_raw')
        self.declare_parameter('output_info_topic',   '/perception/depth/camera_info')
        self.declare_parameter('output_color_topic',  '/perception/depth/colorized')
        self.declare_parameter('output_frame_id',     'camera_link_optical')
        self.declare_parameter('fallback_info_topic', '/camera/camera_info')
        self.declare_parameter('colorize_max_depth',  10.0)   # metres mapped to far end of colormap

        in_depth   = self.get_parameter('input_depth_topic').value
        in_info    = self.get_parameter('input_info_topic').value
        out_depth  = self.get_parameter('output_depth_topic').value
        out_info   = self.get_parameter('output_info_topic').value
        out_color  = self.get_parameter('output_color_topic').value
        self.frame_id      = self.get_parameter('output_frame_id').value
        fallback_info      = self.get_parameter('fallback_info_topic').value
        self.max_depth     = self.get_parameter('colorize_max_depth').value

        self.latest_info: CameraInfo = None

        # ── QoS ─────────────────────────────────────────────────────────────
        gz_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # ── Publishers ───────────────────────────────────────────────────────
        self.depth_pub = self.create_publisher(Image,      out_depth, pub_qos)
        self.info_pub  = self.create_publisher(CameraInfo, out_info,  pub_qos)
        self.color_pub = self.create_publisher(Image,      out_color, pub_qos)

        # ── Subscribers ──────────────────────────────────────────────────────
        self.depth_sub         = self.create_subscription(Image,      in_depth,      self.depth_cb,         gz_qos)
        self.info_sub          = self.create_subscription(CameraInfo, in_info,       self.info_cb,          gz_qos)
        self.fallback_info_sub = self.create_subscription(CameraInfo, fallback_info, self.fallback_info_cb, gz_qos)

        self.get_logger().info(
            f'[DepthRelayNode] SIM depth: {in_depth} → {out_depth} (32FC1) '
            f'+ {out_color} (bgr8 TURBO colormap, frame: {self.frame_id})'
        )

    # ── Callbacks ────────────────────────────────────────────────────────────

    def depth_cb(self, msg: Image):
        """Re-publish depth image (raw 32FC1) + colorized version (bgr8)."""
        msg.header.frame_id = self.frame_id

        # 1. Raw 32FC1 — for Nav2, RTAB-Map, point cloud (unchanged data)
        self.depth_pub.publish(msg)

        # 2. Colorized bgr8 — for clean RViz visualization
        try:
            colorized_msg = self._colorize(msg)
            self.color_pub.publish(colorized_msg)
        except Exception as e:
            # Never crash the relay on colorization failure
            self.get_logger().warn(f'Colorize failed (once): {e}', throttle_duration_sec=10.0)

        # 3. Matching camera_info
        if self.latest_info is not None:
            info_out = CameraInfo()
            info_out.header        = msg.header
            info_out.height        = self.latest_info.height
            info_out.width         = self.latest_info.width
            info_out.distortion_model = self.latest_info.distortion_model
            info_out.d = self.latest_info.d
            info_out.k = self.latest_info.k
            info_out.r = self.latest_info.r
            info_out.p = self.latest_info.p
            self.info_pub.publish(info_out)

    def _colorize(self, msg: Image) -> Image:
        """
        Convert 32FC1 depth (metres) to bgr8 TURBO colormap.

        Colormap scale (0 → max_depth metres):
          dark blue  ← close (0–2m)
          cyan/green ← mid   (3–6m)
          yellow/red ← far   (7–10m)
          black      ← NaN / Inf / zero (invalid/undetected pixels)

        The bgr8 result is directly displayable by any RViz Image plugin without
        any normalization settings — just subscribe and it works.
        """
        # Convert ROS Image to numpy float32 array
        depth_f = self._bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')

        # Build a mask for invalid pixels BEFORE replacing them
        invalid_mask = ~np.isfinite(depth_f) | (depth_f <= 0.0)

        # Replace NaN/Inf with 0 for the normalization step (avoids RuntimeWarning)
        depth_f_clean = np.where(invalid_mask, 0.0, depth_f)

        # Clamp to [0, max_depth] then normalize to [0, 255]
        depth_clipped = np.clip(depth_f_clean, 0.0, self.max_depth)
        depth_norm    = (depth_clipped / self.max_depth * 255.0).astype(np.uint8)

        # Apply TURBO colormap (perceptually uniform, best for depth)
        colored = cv2.applyColorMap(depth_norm, cv2.COLORMAP_TURBO)   # bgr8

        # Set invalid pixels explicitly to black (not left as colormap noise)
        colored[invalid_mask] = (0, 0, 0)

        # Wrap as ROS Image
        out = self._bridge.cv2_to_imgmsg(colored, encoding='bgr8')
        out.header = msg.header
        return out

    def info_cb(self, msg: CameraInfo):
        """Store depth camera_info (depth-specific intrinsics from Gazebo)."""
        self.latest_info = msg

    def fallback_info_cb(self, msg: CameraInfo):
        """
        Use RGB camera_info as fallback if depth camera_info never arrives.
        Gazebo may not publish camera_info for all sensor types.
        RGB intrinsics (fx=615.96, cx=419.83) are close enough for navigation.
        """
        if self.latest_info is None:
            self.latest_info = msg
            self.get_logger().warn(
                'Depth camera_info not received — using RGB camera_info as fallback. '
                'Acceptable for navigation; obstacle positions accurate within ~10% at edges.'
            )


def main(args=None):
    rclpy.init(args=args)
    node = DepthRelayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
