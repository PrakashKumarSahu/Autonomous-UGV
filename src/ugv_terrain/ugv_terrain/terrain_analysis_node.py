#!/usr/bin/env python3
"""
2.5D Multi-Layer Terrain Traversability Mapping Node.
Subscribes:
  - /perception/depth/points (sensor_msgs/msg/PointCloud2)
  - /perception/hazard_mask (sensor_msgs/msg/Image)
Publishes:
  - /terrain/traversability_grid (nav_msgs/msg/OccupancyGrid)
"""

import math
import numpy as np
import cv2
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import PointCloud2, Image
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose, Point, Quaternion
import sensor_msgs_py.point_cloud2 as pc2

import tf2_ros


class TerrainAnalysisNode(Node):
    def __init__(self):
        super().__init__('terrain_analysis_node')

        # Parameters
        self.declare_parameter('pointcloud_topic', '/perception/depth/points')
        self.declare_parameter('hazard_mask_topic', '/perception/hazard_mask')
        self.declare_parameter('output_grid_topic', '/terrain/traversability_grid')
        self.declare_parameter('map_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('grid_resolution', 0.10)
        self.declare_parameter('grid_size_x', 12.0)
        self.declare_parameter('grid_size_y', 12.0)
        self.declare_parameter('max_step_height', 0.22)
        self.declare_parameter('max_slope_angle', 25.0)
        self.declare_parameter('slope_weight', 1.0)
        self.declare_parameter('roughness_weight', 1.2)
        self.declare_parameter('hazard_weight', 2.0)

        self.pointcloud_topic = self.get_parameter('pointcloud_topic').value
        self.hazard_mask_topic = self.get_parameter('hazard_mask_topic').value
        self.output_grid_topic = self.get_parameter('output_grid_topic').value
        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.resolution = float(self.get_parameter('grid_resolution').value)
        self.size_x = float(self.get_parameter('grid_size_x').value)
        self.size_y = float(self.get_parameter('grid_size_y').value)
        self.max_step = float(self.get_parameter('max_step_height').value)
        self.max_slope = float(self.get_parameter('max_slope_angle').value)
        self.slope_weight = float(self.get_parameter('slope_weight').value)
        self.roughness_weight = float(self.get_parameter('roughness_weight').value)
        self.hazard_weight = float(self.get_parameter('hazard_weight').value)

        self.num_cells_x = int(self.size_x / self.resolution)
        self.num_cells_y = int(self.size_y / self.resolution)

        self.bridge = CvBridge()
        self.latest_hazard_mask = None

        # TF Buffer & Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # QoS
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscribers
        self.cloud_sub = self.create_subscription(
            PointCloud2,
            self.pointcloud_topic,
            self.cloud_callback,
            sensor_qos
        )

        self.mask_sub = self.create_subscription(
            Image,
            self.hazard_mask_topic,
            self.mask_callback,
            sensor_qos
        )

        # Publisher
        self.grid_pub = self.create_publisher(OccupancyGrid, self.output_grid_topic, 10)
        self.get_logger().info('Terrain Traversability Analysis Node active.')

    def mask_callback(self, msg: Image):
        try:
            self.latest_hazard_mask = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        except Exception:
            pass

    def cloud_callback(self, msg: PointCloud2):
        # Read XYZ coordinates from point cloud
        gen = pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        points_list = []
        for p in gen:
            points_list.append([p[0], p[1], p[2]])

        if not points_list:
            return

        points = np.array(points_list, dtype=np.float32)

        # In camera optical frame: Z is forward, X is right, Y is down
        # Filter for points ahead of camera
        forward_mask = (points[:, 2] > 0.3) & (points[:, 2] < self.size_x / 2.0)
        pts = points[forward_mask]
        if len(pts) == 0:
            return

        # Robot coordinate mapping:
        # robot X = camera Z (forward)
        # robot Y = -camera X (left)
        # robot Z = -camera Y (up)
        rx = pts[:, 2]
        ry = -pts[:, 0]
        rz = -pts[:, 1]

        # Allocate rolling 2D grid centered on robot
        # Robot is at center (size_x/2, size_y/2)
        half_x = self.size_x / 2.0
        half_y = self.size_y / 2.0

        grid_x = np.floor((rx + half_x) / self.resolution).astype(np.int32)
        grid_y = np.floor((ry + half_y) / self.resolution).astype(np.int32)

        valid_idx = (grid_x >= 0) & (grid_x < self.num_cells_x) & \
                    (grid_y >= 0) & (grid_y < self.num_cells_y)

        gx = grid_x[valid_idx]
        gy = grid_y[valid_idx]
        gz = rz[valid_idx]

        # Compute min and max height per cell using bincount or 2D accumulation
        # Initialize grid layers
        min_elev = np.full((self.num_cells_y, self.num_cells_x), np.inf, dtype=np.float32)
        max_elev = np.full((self.num_cells_y, self.num_cells_x), -np.inf, dtype=np.float32)
        counts = np.zeros((self.num_cells_y, self.num_cells_x), dtype=np.int32)

        for i in range(len(gx)):
            xi = gx[i]
            yi = gy[i]
            zi = gz[i]
            if zi < min_elev[yi, xi]:
                min_elev[yi, xi] = zi
            if zi > max_elev[yi, xi]:
                max_elev[yi, xi] = zi
            counts[yi, xi] += 1

        observed = counts > 0
        diff_elev = np.zeros_like(min_elev)
        diff_elev[observed] = max_elev[observed] - min_elev[observed]

        # Compute costmap values:
        # Default space is -1 (unknown) for unobserved areas
        costmap = np.full((self.num_cells_y, self.num_cells_x), -1, dtype=np.int8)

        # Step height cost (rocks, drops, physical obstacles)
        step_cost = np.clip((diff_elev / self.max_step) * 100.0, 0, 100).astype(np.int8)
        costmap[observed] = step_cost[observed]

        # Clear robot footprint: robot is guaranteed free at its current position
        center_x = self.num_cells_x // 2
        center_y = self.num_cells_y // 2
        footprint_cells = int(0.6 / self.resolution)
        costmap[center_y - footprint_cells:center_y + footprint_cells + 1,
                center_x - footprint_cells:center_x + footprint_cells + 1] = 0

        # Integrate YOLO hazard mask if available via geometric back-projection
        if self.latest_hazard_mask is not None:
            mh, mw = self.latest_hazard_mask.shape[:2]
            fx, fy, cx, cy = 616.0, 616.0, float(mw) / 2.0, float(mh) / 2.0
            
            valid_pts = pts[valid_idx]
            cam_z = valid_pts[:, 2]
            safe_z = np.maximum(cam_z, 0.1)
            u = np.round((valid_pts[:, 0] / safe_z) * fx + cx).astype(np.int32)
            v = np.round((valid_pts[:, 1] / safe_z) * fy + cy).astype(np.int32)
            
            in_img = (u >= 0) & (u < mw) & (v >= 0) & (v < mh)
            if np.any(in_img):
                u_in = u[in_img]
                v_in = v[in_img]
                gx_in = gx[in_img]
                gy_in = gy[in_img]
                hazard_hits = self.latest_hazard_mask[v_in, u_in] > 128
                if np.any(hazard_hits):
                    costmap[gy_in[hazard_hits], gx_in[hazard_hits]] = 100

        # Build and publish OccupancyGrid
        grid_msg = OccupancyGrid()
        grid_msg.header.stamp = msg.header.stamp
        grid_msg.header.frame_id = self.base_frame

        meta = MapMetaData()
        meta.map_load_time = msg.header.stamp
        meta.resolution = self.resolution
        meta.width = self.num_cells_x
        meta.height = self.num_cells_y

        # Origin is at (-half_x, -half_y) relative to base_frame
        origin_pose = Pose()
        origin_pose.position.x = -half_x
        origin_pose.position.y = -half_y
        origin_pose.position.z = 0.0
        origin_pose.orientation.w = 1.0
        meta.origin = origin_pose
        grid_msg.info = meta

        # Flatten into 1D row-major array
        grid_msg.data = costmap.flatten().tolist()
        self.grid_pub.publish(grid_msg)


def main(args=None):
    rclpy.init(args=args)
    node = TerrainAnalysisNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
