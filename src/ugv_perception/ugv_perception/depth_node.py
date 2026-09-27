#!/usr/bin/env python3
"""
Modular Monocular Depth Estimation Node for Autonomous UGV.
Subscribes:
  - /camera/image_raw (sensor_msgs/msg/Image)
  - /camera/camera_info (sensor_msgs/msg/CameraInfo)
Publishes:
  - /perception/depth/image_raw (sensor_msgs/msg/Image, 32FC1 metric depth)
  - /perception/depth/points (sensor_msgs/msg/PointCloud2)
"""

import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import numpy as np
import cv2
from cv_bridge import CvBridge
import torch

from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2


class DepthEstimationNode(Node):
    def __init__(self):
        super().__init__('depth_node')

        # Parameters
        self.declare_parameter('input_image_topic', '/camera/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('output_depth_topic', '/perception/depth/image_raw')
        self.declare_parameter('output_points_topic', '/perception/depth/points')
        self.declare_parameter('depth_frame_id', 'camera_link_optical')
        self.declare_parameter('device', 'cuda:0' if torch.cuda.is_available() else 'cpu')
        self.declare_parameter('min_depth', 0.2)
        self.declare_parameter('max_depth', 25.0)
        self.declare_parameter('publish_pointcloud', False)

        self.input_image_topic = self.get_parameter('input_image_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.output_depth_topic = self.get_parameter('output_depth_topic').value
        self.output_points_topic = self.get_parameter('output_points_topic').value
        self.depth_frame_id = self.get_parameter('depth_frame_id').value
        self.device_str = self.get_parameter('device').value
        self.min_depth = float(self.get_parameter('min_depth').value)
        self.max_depth = float(self.get_parameter('max_depth').value)
        self.publish_pointcloud = bool(self.get_parameter('publish_pointcloud').value)

        self.bridge = CvBridge()
        self.device = torch.device(self.device_str if torch.cuda.is_available() else 'cpu')
        self.get_logger().info(f'Initializing Depth Node on device: {self.device}')

        # Camera intrinsics storage
        self.fx = 450.0
        self.fy = 450.0
        self.cx = 320.0
        self.cy = 240.0
        self.camera_info_received = False
        self.running_d_min = None
        self.running_d_max = None

        # QoS Profiles
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            self.input_image_topic,
            self.image_callback,
            sensor_qos
        )

        self.info_sub = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_callback,
            sensor_qos
        )

        # Publishers
        self.depth_pub = self.create_publisher(Image, self.output_depth_topic, 10)
        self.depth_info_pub = self.create_publisher(CameraInfo, '/perception/depth/camera_info', 10)
        if self.publish_pointcloud:
            self.points_pub = self.create_publisher(PointCloud2, self.output_points_topic, 10)

        self.is_processing = False
        self.latest_camera_info = None

        self.depth_pipe = None
        try:
            from transformers import pipeline
            device_id = 0 if torch.cuda.is_available() else -1
            self.get_logger().info('Loading official Depth Anything V2 neural network model...')
            self.depth_pipe = pipeline('depth-estimation', model='depth-anything/Depth-Anything-V2-Small-hf', device=device_id)
            self.get_logger().info('Depth Anything V2 loaded successfully on GPU!')
        except Exception as e:
            self.get_logger().warn(f'Could not load Depth Anything V2 model: {e}. Using geometric fallback.')

        self.get_logger().info('Depth Node successfully started and ready for frames.')

    def camera_info_callback(self, msg: CameraInfo):
        self.latest_camera_info = msg
        if not self.camera_info_received:
            if len(msg.k) >= 9 and msg.k[0] > 0:
                self.fx = float(msg.k[0])
                self.fy = float(msg.k[4])
                self.cx = float(msg.k[2])
                self.cy = float(msg.k[5])
                self.camera_info_received = True
                self.get_logger().info(f'Camera intrinsics calibrated: fx={self.fx:.1f}, fy={self.fy:.1f}, cx={self.cx:.1f}, cy={self.cy:.1f}')

    def estimate_depth(self, cv_image: np.ndarray) -> np.ndarray:
        """
        Estimate metric depth from RGB using official Depth Anything V2 model.
        """
        h, w = cv_image.shape[:2]

        if self.depth_pipe is not None:
            try:
                from PIL import Image as PILImage
                rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
                pil_img = PILImage.fromarray(rgb)
                out = self.depth_pipe(pil_img)
                raw_depth = np.array(out['predicted_depth'], dtype=np.float32)

                # Resize to original image resolution if needed
                if raw_depth.shape != (h, w):
                    raw_depth = cv2.resize(raw_depth, (w, h), interpolation=cv2.INTER_LINEAR)

                # Depth Anything relative disparity conversion to metric depth with EMA smoothing
                inst_min, inst_max = float(raw_depth.min()), float(raw_depth.max())
                alpha = 0.25
                if self.running_d_min is None:
                    self.running_d_min = inst_min
                    self.running_d_max = inst_max
                else:
                    self.running_d_min = (1.0 - alpha) * self.running_d_min + alpha * inst_min
                    self.running_d_max = (1.0 - alpha) * self.running_d_max + alpha * inst_max

                d_min, d_max = self.running_d_min, self.running_d_max
                if d_max > d_min:
                    norm = (raw_depth - d_min) / (d_max - d_min)
                    norm = np.clip(norm, 0.0, 1.0)
                    # Disparity: high value is close, low value is far
                    metric_depth = self.min_depth + (1.0 - norm) * (self.max_depth - self.min_depth)
                else:
                    metric_depth = np.full((h, w), 2.5, dtype=np.float32)

                return np.clip(metric_depth, self.min_depth, self.max_depth).astype(np.float32)
            except Exception as e:
                self.get_logger().error(f'Depth Anything inference failed: {e}. Using geometric fallback.')

        # Geometric depth estimation fallback
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        y_coords = np.arange(h, dtype=np.float32)[:, None]
        horizon_y = h * 0.42
        dy = np.maximum(y_coords - horizon_y, 1.0)
        cam_height = 0.43
        base_depth = (self.fy * cam_height) / dy
        base_depth = np.repeat(base_depth, w, axis=1)

        edges = cv2.Canny(gray, 50, 150).astype(np.float32) / 255.0
        blurred_edges = cv2.GaussianBlur(edges, (15, 15), 0)
        depth_map = base_depth - (blurred_edges * 1.5)
        return np.clip(depth_map, self.min_depth, self.max_depth).astype(np.float32)

    def image_callback(self, msg: Image):
        if self.is_processing:
            # Skip frame to preserve real-time synchronization and avoid queue latency
            return

        self.is_processing = True
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            depth_map = self.estimate_depth(cv_image)

            # Publish 32FC1 depth image with exact matching timestamp.
            # Always use the canonical ROS optical frame (camera_link_optical), NOT the
            # Gazebo-injected frame_id (tugbot/camera_front/color). Using the Gazebo
            # frame_id here breaks point_cloud_xyz_node synchronisation because the
            # depth CameraInfo would carry a mismatching frame_id.
            depth_msg = self.bridge.cv2_to_imgmsg(depth_map, encoding='32FC1')
            depth_msg.header = msg.header
            depth_msg.header.frame_id = self.depth_frame_id  # always 'camera_link_optical'
            self.depth_pub.publish(depth_msg)

            # Publish matching camera info for depth_image_proc synchronization.
            # frame_id must match depth image frame_id exactly.
            depth_info = CameraInfo()
            depth_info.header = depth_msg.header  # already has canonical frame_id
            depth_info.height = depth_msg.height
            depth_info.width = depth_msg.width
            if self.latest_camera_info is not None:
                depth_info.distortion_model = self.latest_camera_info.distortion_model
                depth_info.d = self.latest_camera_info.d
                depth_info.k = self.latest_camera_info.k
                depth_info.r = self.latest_camera_info.r
                depth_info.p = self.latest_camera_info.p
            else:
                depth_info.distortion_model = 'plumb_bob'
                depth_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
                depth_info.k = [self.fx, 0.0, self.cx, 0.0, self.fy, self.cy, 0.0, 0.0, 1.0]
                depth_info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
                depth_info.p = [self.fx, 0.0, self.cx, 0.0, 0.0, self.fy, self.cy, 0.0, 0.0, 0.0, 1.0, 0.0]
            self.depth_info_pub.publish(depth_info)

            # Publish Point Cloud if enabled (uses canonical frame_id)
            if self.publish_pointcloud:
                self.publish_cloud(depth_map, msg.header, self.depth_frame_id)
        except Exception as e:
            self.get_logger().error(f'Failed in image_callback: {e}')
        finally:
            self.is_processing = False

    def publish_cloud(self, depth_map: np.ndarray, header: Header, frame_id: str):
        h, w = depth_map.shape
        # Downsample factor for real-time 30Hz throughput
        step = 4
        sub_depth = depth_map[::step, ::step]

        u = np.arange(0, w, step, dtype=np.float32)
        v = np.arange(0, h, step, dtype=np.float32)
        uu, vv = np.meshgrid(u, v)

        # Back-project to 3D in camera optical frame
        z = sub_depth
        x = (uu - self.cx) * z / self.fx
        y = (vv - self.cy) * z / self.fy

        # Filter valid points
        valid = (z >= self.min_depth) & (z <= self.max_depth)
        points = np.stack([x[valid], y[valid], z[valid]], axis=-1)

        # Generate PointCloud2 message
        cloud_header = Header()
        cloud_header.stamp = header.stamp
        cloud_header.frame_id = frame_id

        cloud_msg = pc2.create_cloud_xyz32(cloud_header, points)
        self.points_pub.publish(cloud_msg)


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
