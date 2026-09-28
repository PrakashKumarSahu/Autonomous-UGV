#!/usr/bin/env python3
"""
Intel RealSense Depth Relay Node — Hardware Mode (camera_type:=realsense).

Bridges Intel RealSense D435/D455/D457 depth output to the UGV pipeline's
canonical depth topics.

Prerequisites:
  sudo apt install ros-jazzy-realsense2-camera
  ros2 launch realsense2_camera rs_launch.py

Default RealSense depth topic: 16UC1 in millimetres.
This node converts to 32FC1 in metres, re-frames to camera_link_optical,
and publishes on /perception/depth/image_raw so the rest of the pipeline
(SLAM, Nav2, GridMap) works identically to sim mode.

Configurable parameters allow adapting to different RealSense launch configs.
"""

import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo


class RealSenseRelayNode(Node):
    def __init__(self):
        super().__init__('realsense_relay_node')

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter('input_depth_topic',  '/camera/camera/depth/image_rect_raw')
        self.declare_parameter('input_info_topic',   '/camera/camera/depth/camera_info')
        self.declare_parameter('output_depth_topic', '/perception/depth/image_raw')
        self.declare_parameter('output_info_topic',  '/perception/depth/camera_info')
        self.declare_parameter('output_frame_id',    'camera_link_optical')
        self.declare_parameter('min_depth_m', 0.1)
        self.declare_parameter('max_depth_m', 10.0)

        in_depth  = self.get_parameter('input_depth_topic').value
        in_info   = self.get_parameter('input_info_topic').value
        out_depth = self.get_parameter('output_depth_topic').value
        out_info  = self.get_parameter('output_info_topic').value
        self.frame_id  = self.get_parameter('output_frame_id').value
        self.min_depth = float(self.get_parameter('min_depth_m').value)
        self.max_depth = float(self.get_parameter('max_depth_m').value)

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
        self.depth_sub = self.create_subscription(Image,      in_depth, self.depth_cb, sensor_qos)
        self.info_sub  = self.create_subscription(CameraInfo, in_info,  self.info_cb,  sensor_qos)

        self.get_logger().info(
            f'[RealSenseRelayNode] {in_depth} (16UC1 mm) → {out_depth} (32FC1 m)'
        )

    def depth_cb(self, msg: Image):
        try:
            # RealSense D435: 16UC1 depth in millimetres
            depth_mm = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
            depth_m  = depth_mm.astype(np.float32) / 1000.0

            # Clip to valid sensor range and NaN-fill invalid pixels (0 mm = no return)
            invalid = (depth_m < self.min_depth) | (depth_m > self.max_depth)
            depth_m[invalid] = float('nan')

            out = self.bridge.cv2_to_imgmsg(depth_m, encoding='32FC1')
            out.header          = msg.header
            out.header.frame_id = self.frame_id
            self.depth_pub.publish(out)

            # Publish camera_info aligned to this depth stamp
            if self.latest_info is not None:
                info_out = CameraInfo()
                info_out.header        = out.header
                info_out.height        = self.latest_info.height
                info_out.width         = self.latest_info.width
                info_out.distortion_model = self.latest_info.distortion_model
                info_out.d = self.latest_info.d
                info_out.k = self.latest_info.k
                info_out.r = self.latest_info.r
                info_out.p = self.latest_info.p
                self.info_pub.publish(info_out)
        except Exception as e:
            self.get_logger().error(f'depth_cb: {e}')

    def info_cb(self, msg: CameraInfo):
        self.latest_info = msg


def main(args=None):
    rclpy.init(args=args)
    node = RealSenseRelayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
