#!/usr/bin/env python3
"""
ZED Stereo Camera Depth Relay Node — Hardware Mode (camera_type:=zed).

Bridges ZED 2 / ZED X depth output to the UGV pipeline's canonical topics.
ZED publishes 32FC1 depth in metres natively — only frame_id correction needed.

Prerequisites:
  Install ZED SDK + zed-ros2-wrapper: https://github.com/stereolabs/zed-ros2-wrapper
  ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2

Supports outdoor/long-range navigation (up to 20 m) with neural depth enhancement.

Canonical depth topic contract (all camera_type sources must publish these):
  /perception/depth/image_raw   (32FC1, metres)        — Nav2, RTAB-Map, point cloud
  /perception/depth/camera_info (CameraInfo)           — intrinsics for point cloud
  /perception/depth/colorized   (bgr8, TURBO colormap) — RViz RealDepth panel
"""

import rclpy
import numpy as np
import cv2
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo


class ZEDRelayNode(Node):
    def __init__(self):
        super().__init__('zed_relay_node')

        self.declare_parameter('input_depth_topic',  '/zed/zed_node/depth/depth_registered')
        self.declare_parameter('input_info_topic',   '/zed/zed_node/depth/camera_info')
        self.declare_parameter('output_depth_topic', '/perception/depth/image_raw')
        self.declare_parameter('output_info_topic',  '/perception/depth/camera_info')
        self.declare_parameter('output_color_topic', '/perception/depth/colorized')
        self.declare_parameter('output_frame_id',    'camera_link_optical')
        self.declare_parameter('colorize_max_depth', 10.0)

        in_depth        = self.get_parameter('input_depth_topic').value
        in_info         = self.get_parameter('input_info_topic').value
        out_depth       = self.get_parameter('output_depth_topic').value
        out_info        = self.get_parameter('output_info_topic').value
        out_color       = self.get_parameter('output_color_topic').value
        self.frame_id   = self.get_parameter('output_frame_id').value
        self.colorize_max = float(self.get_parameter('colorize_max_depth').value)

        self.bridge = CvBridge()
        self.latest_info: CameraInfo = None

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=1
        )
        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST, depth=10
        )

        self.depth_pub = self.create_publisher(Image,      out_depth, pub_qos)
        self.info_pub  = self.create_publisher(CameraInfo, out_info,  pub_qos)
        self.color_pub = self.create_publisher(Image,      out_color, pub_qos)
        self.depth_sub = self.create_subscription(Image,      in_depth, self.depth_cb, sensor_qos)
        self.info_sub  = self.create_subscription(CameraInfo, in_info,  self.info_cb,  sensor_qos)

        self.get_logger().info(
            f'[ZEDRelayNode] {in_depth} (32FC1 m) → {out_depth}'
            f' + {out_color} (bgr8 TURBO)'
        )

    def depth_cb(self, msg: Image):
        # ZED already publishes 32FC1 in metres — only frame correction needed
        msg.header.frame_id = self.frame_id
        self.depth_pub.publish(msg)

        # Colorized TURBO output — required for RViz RealDepth panel
        try:
            depth_f   = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
            colorized = self._colorize(depth_f)
            color_msg = self.bridge.cv2_to_imgmsg(colorized, encoding='bgr8')
            color_msg.header = msg.header
            self.color_pub.publish(color_msg)
        except Exception as e:
            self.get_logger().warn(f'Colorize failed: {e}', throttle_duration_sec=10.0)

        if self.latest_info is not None:
            info_out = CameraInfo()
            info_out.header           = msg.header
            info_out.height           = self.latest_info.height
            info_out.width            = self.latest_info.width
            info_out.distortion_model = self.latest_info.distortion_model
            info_out.d = self.latest_info.d
            info_out.k = self.latest_info.k
            info_out.r = self.latest_info.r
            info_out.p = self.latest_info.p
            self.info_pub.publish(info_out)

    def _colorize(self, depth_m: np.ndarray) -> np.ndarray:
        """Convert 32FC1 depth (metres) to bgr8 TURBO colormap. Black = invalid."""
        invalid = ~np.isfinite(depth_m) | (depth_m <= 0.0)
        clean   = np.where(invalid, 0.0, depth_m)
        norm    = (np.clip(clean, 0.0, self.colorize_max) / self.colorize_max * 255).astype(np.uint8)
        colored = cv2.applyColorMap(norm, cv2.COLORMAP_TURBO)  # dark blue=near, red=far
        colored[invalid] = (0, 0, 0)
        return colored

    def info_cb(self, msg: CameraInfo):
        self.latest_info = msg


def main(args=None):
    rclpy.init(args=args)
    node = ZEDRelayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
