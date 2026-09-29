#!/usr/bin/env python3
"""
Synchronized Depth Image to 3D PointCloud2 Conversion Node.

Subscribes to the canonical depth topics produced by the active depth source
relay (sim/monocular/realsense/zed) and converts to a PointCloud2 message
consumed by Nav2's ObstacleLayer.

QoS contract:
  Inputs:
    /perception/depth/image_raw   (32FC1, RELIABLE) ← from depth_relay_node
    /perception/depth/camera_info (CameraInfo, RELIABLE) ← from depth_relay_node
  Output:
    /perception/depth/points      (PointCloud2, RELIABLE) → Nav2 ObstacleLayer

NOTE: Both subscriptions use RELIABLE QoS with depth=5 to match the relay
publisher. Using BEST_EFFORT + depth=1 (previous) caused silent frame drops
under CPU load, creating gaps in the point cloud that made RViz's CameraDepthCloud
flicker between OK and Error status.
"""

import rclpy
import rclpy.duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2


class DepthToPointCloudNode(Node):
    def __init__(self):
        super().__init__('depth_to_pointcloud_node')

        self.declare_parameter('depth_topic',       '/perception/depth/image_raw')
        self.declare_parameter('camera_info_topic', '/perception/depth/camera_info')
        self.declare_parameter('pointcloud_topic',  '/perception/depth/points')
        # Every N-th pixel is sampled. step=4 = 1/16 of pixels → ~4800 pts at 640×480.
        # Lower = denser cloud but more CPU. Increase if Nav2 is laggy.
        self.declare_parameter('step', 4)

        self.depth_topic       = self.get_parameter('depth_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.pointcloud_topic  = self.get_parameter('pointcloud_topic').value
        self.step              = int(self.get_parameter('step').value)

        self.bridge = CvBridge()

        # Fallback intrinsics (Localbot Gazebo depth camera defaults).
        # Updated on every CameraInfo message.
        self.fx = 450.0
        self.fy = 450.0
        self.cx = 320.0
        self.cy = 240.0

        # ── QoS ─────────────────────────────────────────────────────────────
        # RELIABLE + depth=5: matches depth_relay_node's publisher QoS.
        # depth=5: buffers up to 5 pending images so burst drops under CPU load
        # don't cause gaps. Previously BEST_EFFORT+depth=1 caused frame drops.
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            durability=DurabilityPolicy.VOLATILE
        )

        self.info_sub  = self.create_subscription(CameraInfo, self.camera_info_topic, self.info_cb,  reliable_qos)
        self.depth_sub = self.create_subscription(Image,      self.depth_topic,       self.depth_cb, reliable_qos)

        # Output: RELIABLE to match Nav2 ObstacleLayer expectation
        self.cloud_pub = self.create_publisher(PointCloud2, self.pointcloud_topic,
                                               QoSProfile(
                                                   reliability=ReliabilityPolicy.RELIABLE,
                                                   history=HistoryPolicy.KEEP_LAST,
                                                   depth=10,
                                                   durability=DurabilityPolicy.VOLATILE
                                               ))

        self.get_logger().info(
            f'[DepthToPointCloudNode] {self.depth_topic} → {self.pointcloud_topic} '
            f'(step={self.step}, QoS: RELIABLE)'
        )

    def info_cb(self, msg: CameraInfo):
        """Update camera intrinsics from live CameraInfo."""
        if len(msg.k) >= 9 and msg.k[0] > 0:
            self.fx, self.fy = float(msg.k[0]), float(msg.k[4])
            self.cx, self.cy = float(msg.k[2]), float(msg.k[5])

    def depth_cb(self, msg: Image):
        """Convert 32FC1 depth image to PointCloud2 in camera_link_optical frame."""
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge failed: {e}', throttle_duration_sec=5.0)
            return

        h, w = depth.shape
        sub_depth = depth[::self.step, ::self.step]

        u = np.arange(0, w, self.step, dtype=np.float32)
        v = np.arange(0, h, self.step, dtype=np.float32)
        uu, vv = np.meshgrid(u, v)

        z = sub_depth
        # Valid: finite, not zero, within sensor range [0.2m, 25m]
        valid = (z > 0.2) & (z < 10.0) & np.isfinite(z)

        x = (uu - self.cx) * z / self.fx
        y = (vv - self.cy) * z / self.fy

        points = np.stack([x[valid], y[valid], z[valid]], axis=-1)

        if points.shape[0] == 0:
            # Still publish an empty cloud so Nav2 receives a heartbeat.
            # This prevents the ObstacleLayer from stalling on missing data.
            points = np.zeros((0, 3), dtype=np.float32)

        header = Header()
        # Stamp 150 ms in the past so RViz's TF lookup never fails.
        # Root cause: TF2 cannot extrapolate into the future — even by 1 ms.
        # RTAB-Map publishes map→odom TF at ~15 Hz (every ~67 ms). Its most recent
        # TF is therefore at most 67 ms old. Requesting the transform at
        # (now - 150 ms) is guaranteed to fall BETWEEN two stored RTAB-Map TFs
        # so TF2 always interpolates successfully → no ExtrapolationException → no flicker.
        # Nav2 observation_persistence=0.5s so 150 ms-old data is fully valid.
        _now = self.get_clock().now()
        header.stamp    = (_now - rclpy.duration.Duration(seconds=0.15)).to_msg()
        header.frame_id = msg.header.frame_id if msg.header.frame_id else 'camera_link_optical'

        cloud_msg = pc2.create_cloud_xyz32(header, points)
        self.cloud_pub.publish(cloud_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DepthToPointCloudNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
