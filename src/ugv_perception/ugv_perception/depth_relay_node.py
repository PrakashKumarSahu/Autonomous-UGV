#!/usr/bin/env python3
"""
Depth Source Relay Node — Simulation Mode (camera_type:=sim).

Bridges Gazebo's REAL depth camera sensor (metric, 0.1–10 m, 32FC1) to the
pipeline's canonical topics. The rest of the pipeline (point_cloud_xyz_node,
rgbd_sync → RTAB-Map, Nav2 ObstacleLayer) only reads from:

  /perception/depth/image_raw   (32FC1, metric metres, frame: camera_link_optical)
  /perception/depth/camera_info (depth intrinsics, frame: camera_link_optical)

Swappable in production — replace this node with:
  camera_type:=monocular   → depth_node        (Depth Anything V3, any RGB camera)
  camera_type:=realsense   → realsense_relay_node (Intel RealSense D435/D455)
  camera_type:=zed         → zed_relay_node    (ZED 2 / ZED X)
All produce the same output topics — the rest of the stack is unchanged.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo


class DepthRelayNode(Node):
    def __init__(self):
        super().__init__('depth_relay_node')

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter('input_depth_topic',  '/camera/depth/image_raw')
        self.declare_parameter('input_info_topic',   '/camera/depth/camera_info')
        self.declare_parameter('output_depth_topic', '/perception/depth/image_raw')
        self.declare_parameter('output_info_topic',  '/perception/depth/camera_info')
        self.declare_parameter('output_frame_id',    'camera_link_optical')
        # Fallback: use RGB camera_info if depth camera_info is unavailable
        self.declare_parameter('fallback_info_topic', '/camera/camera_info')

        in_depth   = self.get_parameter('input_depth_topic').value
        in_info    = self.get_parameter('input_info_topic').value
        out_depth  = self.get_parameter('output_depth_topic').value
        out_info   = self.get_parameter('output_info_topic').value
        self.frame_id     = self.get_parameter('output_frame_id').value
        fallback_info     = self.get_parameter('fallback_info_topic').value

        self.latest_info: CameraInfo = None

        # ── QoS ─────────────────────────────────────────────────────────────
        # Gazebo bridge publishes BEST_EFFORT (fire-and-forget).
        gz_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        # Downstream nodes (Nav2, RTAB-Map) require RELIABLE.
        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # ── Publishers ───────────────────────────────────────────────────────
        self.depth_pub = self.create_publisher(Image,      out_depth, pub_qos)
        self.info_pub  = self.create_publisher(CameraInfo, out_info,  pub_qos)

        # ── Subscribers ──────────────────────────────────────────────────────
        self.depth_sub        = self.create_subscription(Image,      in_depth,      self.depth_cb,        gz_qos)
        self.info_sub         = self.create_subscription(CameraInfo, in_info,       self.info_cb,         gz_qos)
        self.fallback_info_sub = self.create_subscription(CameraInfo, fallback_info, self.fallback_info_cb, gz_qos)

        self.get_logger().info(
            f'[DepthRelayNode] SIM depth: {in_depth} → {out_depth} '
            f'(frame_id: {self.frame_id})'
        )
        self.get_logger().info(
            '  Swap via camera_type launch arg: monocular | realsense | zed'
        )

    # ── Callbacks ────────────────────────────────────────────────────────────

    def depth_cb(self, msg: Image):
        """Re-publish depth image with canonical frame_id."""
        msg.header.frame_id = self.frame_id
        self.depth_pub.publish(msg)

        # Piggy-back: re-publish last known camera_info matched to this stamp
        if self.latest_info is not None:
            info_out = CameraInfo()
            info_out.header        = msg.header   # same stamp + canonical frame
            info_out.height        = self.latest_info.height
            info_out.width         = self.latest_info.width
            info_out.distortion_model = self.latest_info.distortion_model
            info_out.d = self.latest_info.d
            info_out.k = self.latest_info.k
            info_out.r = self.latest_info.r
            info_out.p = self.latest_info.p
            self.info_pub.publish(info_out)

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
                'This is acceptable for navigation; obstacle positions will be accurate '
                'within ~10% at the edges of the depth image.'
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
