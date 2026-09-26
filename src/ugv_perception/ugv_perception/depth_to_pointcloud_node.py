#!/usr/bin/env python3
"""
Synchronized Depth Image to 3D PointCloud2 Conversion Node.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2


class DepthToPointCloudNode(Node):
    def __init__(self):
        super().__init__('depth_to_pointcloud_node')

        self.declare_parameter('depth_topic', '/perception/depth/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('pointcloud_topic', '/perception/depth/points')
        self.declare_parameter('step', 4)

        self.depth_topic = self.get_parameter('depth_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.pointcloud_topic = self.get_parameter('pointcloud_topic').value
        self.step = int(self.get_parameter('step').value)

        self.bridge = CvBridge()
        self.fx = 450.0
        self.fy = 450.0
        self.cx = 320.0
        self.cy = 240.0

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)

        self.info_sub = self.create_subscription(CameraInfo, self.camera_info_topic, self.info_cb, qos)
        self.depth_sub = self.create_subscription(Image, self.depth_topic, self.depth_cb, qos)
        self.cloud_pub = self.create_publisher(PointCloud2, self.pointcloud_topic, 10)

    def info_cb(self, msg: CameraInfo):
        if len(msg.k) >= 9 and msg.k[0] > 0:
            self.fx, self.fy = float(msg.k[0]), float(msg.k[4])
            self.cx, self.cy = float(msg.k[2]), float(msg.k[5])

    def depth_cb(self, msg: Image):
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        except Exception as e:
            return

        h, w = depth.shape
        sub_depth = depth[::self.step, ::self.step]

        u = np.arange(0, w, self.step, dtype=np.float32)
        v = np.arange(0, h, self.step, dtype=np.float32)
        uu, vv = np.meshgrid(u, v)

        z = sub_depth
        valid = (z > 0.2) & (z < 25.0) & (~np.isnan(z)) & (~np.isinf(z))

        x = (uu - self.cx) * z / self.fx
        y = (vv - self.cy) * z / self.fy

        points = np.stack([x[valid], y[valid], z[valid]], axis=-1)

        header = Header()
        header.stamp = msg.header.stamp
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
