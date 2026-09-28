#!/usr/bin/env python3
"""
ZED Stereo Camera Depth Relay Node — Hardware Mode (camera_type:=zed).

Bridges ZED 2 / ZED X depth output to the UGV pipeline's canonical topics.
ZED publishes 32FC1 depth in metres natively — only frame_id correction needed.

Prerequisites:
  Install ZED SDK + zed-ros2-wrapper: https://github.com/stereolabs/zed-ros2-wrapper
  ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2

Supports outdoor/long-range navigation (up to 20 m) with neural depth enhancement.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo


class ZEDRelayNode(Node):
    def __init__(self):
        super().__init__('zed_relay_node')

        self.declare_parameter('input_depth_topic',  '/zed/zed_node/depth/depth_registered')
        self.declare_parameter('input_info_topic',   '/zed/zed_node/depth/camera_info')
        self.declare_parameter('output_depth_topic', '/perception/depth/image_raw')
        self.declare_parameter('output_info_topic',  '/perception/depth/camera_info')
        self.declare_parameter('output_frame_id',    'camera_link_optical')

        in_depth  = self.get_parameter('input_depth_topic').value
        in_info   = self.get_parameter('input_info_topic').value
        out_depth = self.get_parameter('output_depth_topic').value
        out_info  = self.get_parameter('output_info_topic').value
        self.frame_id = self.get_parameter('output_frame_id').value

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
            f'[ZEDRelayNode] {in_depth} (32FC1 m) → {out_depth}'
        )

    def depth_cb(self, msg: Image):
        # ZED already publishes 32FC1 in metres — only frame correction needed
        msg.header.frame_id = self.frame_id
        self.depth_pub.publish(msg)

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
