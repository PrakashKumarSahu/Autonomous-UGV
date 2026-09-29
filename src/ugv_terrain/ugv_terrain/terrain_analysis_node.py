#!/usr/bin/env python3
"""
2.5D Multi-Layer Terrain Traversability Mapping Node.
Subscribes:
  - /perception/depth/points (sensor_msgs/msg/PointCloud2)
  - /perception/hazard_mask (sensor_msgs/msg/Image)
  - /perception/depth/camera_info (sensor_msgs/msg/CameraInfo) — live intrinsics
Publishes:
  - /terrain/traversability_grid (nav_msgs/msg/OccupancyGrid)
"""

import numpy as np
import cv2
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import PointCloud2, Image, CameraInfo
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose, Point, Quaternion
import sensor_msgs_py.point_cloud2 as pc2


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

        # Camera intrinsics — populated from /perception/depth/camera_info at runtime.
        # Default values are the Gazebo depth camera intrinsics (848×480, FOV~87°).
        # These are overwritten by the first camera_info message received, so ANY
        # depth source (RealSense, ZED, Depth Anything V3) self-calibrates automatically.
        self.fx = 421.62   # pixels — updated from camera_info K[0]
        self.fy = 421.62   # pixels — updated from camera_info K[4]
        self.cx = 422.29   # pixels — updated from camera_info K[2]
        self.cy = 236.57   # pixels — updated from camera_info K[5]

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

        # Subscribe to camera_info to get real intrinsics (fx,fy,cx,cy) dynamically.
        # Fixes: was hardcoded to fx=616.0 which is wrong for Gazebo depth cam (fx≈421).
        # Uses Reliable QoS since depth_relay_node publishes camera_info as Reliable.
        reliable_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=1)
        self.info_sub = self.create_subscription(
            CameraInfo,
            '/perception/depth/camera_info',
            self._camera_info_cb,
            reliable_qos
        )

        # Publisher
        self.grid_pub = self.create_publisher(OccupancyGrid, self.output_grid_topic, 10)
        self.get_logger().info('Terrain Traversability Analysis Node active.')

    def _camera_info_cb(self, msg) -> None:
        """Update camera intrinsics from live camera_info (auto-calibrates to any depth source)."""
        k = msg.k  # row-major 3×3 intrinsic matrix
        self.fx = k[0]
        self.fy = k[4]
        self.cx = k[2]
        self.cy = k[5]

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

        # Compute min and max height per cell — vectorized with numpy ufuncs.
        # np.minimum.at / maximum.at / add.at are ~10-100x faster than Python for-loops
        # on typical depth camera point clouds (50k+ points at step=4).
        # Initialize grid layers
        min_elev = np.full((self.num_cells_y, self.num_cells_x), np.inf, dtype=np.float32)
        max_elev = np.full((self.num_cells_y, self.num_cells_x), -np.inf, dtype=np.float32)
        counts = np.zeros((self.num_cells_y, self.num_cells_x), dtype=np.int32)

        flat_idx = gy * self.num_cells_x + gx
        np.minimum.at(min_elev.ravel(), flat_idx, gz)
        np.maximum.at(max_elev.ravel(), flat_idx, gz)
        np.add.at(counts.ravel(), flat_idx, 1)

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

        # Integrate YOLO hazard mask if available via geometric back-projection.
        # Uses live camera intrinsics (self.fx/fy/cx/cy) from /perception/depth/camera_info.
        # Previously hardcoded fx=616.0 which was wrong for the Gazebo depth cam (fx≈421).
        if self.latest_hazard_mask is not None:
            mh, mw = self.latest_hazard_mask.shape[:2]
            fx, fy, cx, cy = self.fx, self.fy, self.cx, self.cy
            
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
