#!/usr/bin/env python3
"""
2.5D Terrain Traversability Mapping Node.

Builds a robot-centric rolling OccupancyGrid from the camera point cloud and the
YOLO26 hazard mask.

Subscribes:
  - pointcloud_topic   (sensor_msgs/PointCloud2, camera_link_optical frame)
  - hazard_mask_topic  (sensor_msgs/Image mono8: 0 safe, 128 caution, 255 lethal)
  - camera_info_topic  (sensor_msgs/CameraInfo) — live intrinsics for mask projection
Publishes:
  - output_grid_topic  (nav_msgs/OccupancyGrid, frame = base_frame)
        -1 unknown, 0 free … 100 lethal

Per-cell cost layers (max of all):
  1. Step     — elevation range inside a cell vs. max_step_height (rocks, edges)
  2. Height   — cell top/bottom vs. ground (z=0 in base_frame), with a tolerance
                that grows with distance by tan(max_slope_angle). Flags positive
                obstacles and negative ones (ditches, holes) while letting gentle
                slopes through.
  3. Semantic — points whose image projection hits a lethal YOLO hazard pixel.

Points are transformed camera → base_frame with the real TF (static, cached), so
ground lies at z≈0. If TF is not yet available, the URDF default mount is used.
"""

import math

import numpy as np
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.time import Time

from sensor_msgs.msg import PointCloud2, Image, CameraInfo
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose
import sensor_msgs_py.point_cloud2 as pc2
from tf2_ros import Buffer, TransformListener


# camera_link_optical → base_footprint for the default Localbot URDF
# (optical axes: Z fwd, X right, Y down; mount ≈ x 0.055, y 0.033, z 0.645).
_DEFAULT_R = np.array([[0.0, 0.0, 1.0],
                       [-1.0, 0.0, 0.0],
                       [0.0, -1.0, 0.0]], dtype=np.float32)
_DEFAULT_T = np.array([0.0553, 0.0325, 0.6448], dtype=np.float32)


def _quat_to_matrix(x, y, z, w) -> np.ndarray:
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ], dtype=np.float32)


class TerrainAnalysisNode(Node):
    def __init__(self):
        super().__init__('terrain_analysis_node')

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter('pointcloud_topic', '/perception/depth/points')
        self.declare_parameter('hazard_mask_topic', '/perception/hazard_mask')
        self.declare_parameter('camera_info_topic', '/perception/depth/camera_info')
        self.declare_parameter('output_grid_topic', '/terrain/traversability_grid')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('grid_resolution', 0.10)
        self.declare_parameter('grid_size_x', 12.0)
        self.declare_parameter('grid_size_y', 12.0)
        self.declare_parameter('min_range', 0.3)
        self.declare_parameter('max_step_height', 0.22)
        self.declare_parameter('max_slope_angle', 25.0)
        self.declare_parameter('footprint_clear_radius', 0.6)

        gp = lambda n: self.get_parameter(n).value  # noqa: E731
        self.pointcloud_topic = gp('pointcloud_topic')
        self.hazard_mask_topic = gp('hazard_mask_topic')
        self.camera_info_topic = gp('camera_info_topic')
        self.output_grid_topic = gp('output_grid_topic')
        self.base_frame = gp('base_frame')
        self.resolution = float(gp('grid_resolution'))
        self.size_x = float(gp('grid_size_x'))
        self.size_y = float(gp('grid_size_y'))
        self.min_range = float(gp('min_range'))
        self.max_step = float(gp('max_step_height'))
        self.slope_tan = math.tan(math.radians(float(gp('max_slope_angle'))))
        self.clear_radius = float(gp('footprint_clear_radius'))

        self.num_cells_x = int(self.size_x / self.resolution)
        self.num_cells_y = int(self.size_y / self.resolution)

        self.bridge = CvBridge()
        self.latest_hazard_mask = None

        # Intrinsics — overwritten by the first camera_info (any depth source).
        self.fx, self.fy, self.cx, self.cy = 450.0, 450.0, 320.0, 240.0

        # Camera → base transform (static; cached after first successful lookup).
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.cam_R, self.cam_T = _DEFAULT_R, _DEFAULT_T
        self.tf_cached = False

        # ── I/O ─────────────────────────────────────────────────────────────
        sensor_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                history=HistoryPolicy.KEEP_LAST, depth=1)
        reliable_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                                  history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(PointCloud2, self.pointcloud_topic, self.cloud_callback, sensor_qos)
        self.create_subscription(Image, self.hazard_mask_topic, self.mask_callback, sensor_qos)
        self.create_subscription(CameraInfo, self.camera_info_topic, self._camera_info_cb, reliable_qos)
        self.grid_pub = self.create_publisher(OccupancyGrid, self.output_grid_topic, 10)
        self.get_logger().info('Terrain traversability node active.')

    # ── Callbacks ───────────────────────────────────────────────────────────

    def _camera_info_cb(self, msg: CameraInfo) -> None:
        if msg.k[0] > 0:
            self.fx, self.fy, self.cx, self.cy = msg.k[0], msg.k[4], msg.k[2], msg.k[5]

    def mask_callback(self, msg: Image):
        try:
            self.latest_hazard_mask = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        except Exception:
            pass

    def _update_camera_tf(self, frame_id: str) -> None:
        if self.tf_cached or not frame_id:
            return
        try:
            tf = self.tf_buffer.lookup_transform(self.base_frame, frame_id, Time())
        except Exception:
            return  # keep URDF default until TF is available
        q, t = tf.transform.rotation, tf.transform.translation
        self.cam_R = _quat_to_matrix(q.x, q.y, q.z, q.w)
        self.cam_T = np.array([t.x, t.y, t.z], dtype=np.float32)
        self.tf_cached = True
        self.get_logger().info(
            f'Using TF {frame_id} → {self.base_frame}: t=({t.x:.3f}, {t.y:.3f}, {t.z:.3f})')

    def cloud_callback(self, msg: PointCloud2):
        self._update_camera_tf(msg.header.frame_id)

        points = pc2.read_points_numpy(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        if points.size == 0:
            return
        points = points.astype(np.float32, copy=False)

        # Keep points in front of the camera within the grid half-size (optical Z = forward).
        pts = points[(points[:, 2] > self.min_range) & (points[:, 2] < self.size_x / 2.0)]
        if len(pts) == 0:
            return

        # Optical frame → base frame (ground ≈ z 0).
        base_pts = pts @ self.cam_R.T + self.cam_T
        rx, ry, rz = base_pts[:, 0], base_pts[:, 1], base_pts[:, 2]

        half_x, half_y = self.size_x / 2.0, self.size_y / 2.0
        grid_x = np.floor((rx + half_x) / self.resolution).astype(np.int32)
        grid_y = np.floor((ry + half_y) / self.resolution).astype(np.int32)
        valid = ((grid_x >= 0) & (grid_x < self.num_cells_x) &
                 (grid_y >= 0) & (grid_y < self.num_cells_y))
        gx, gy, gz = grid_x[valid], grid_y[valid], rz[valid]

        shape = (self.num_cells_y, self.num_cells_x)
        min_elev = np.full(shape, np.inf, dtype=np.float32)
        max_elev = np.full(shape, -np.inf, dtype=np.float32)
        counts = np.zeros(shape, dtype=np.int32)
        flat_idx = gy * self.num_cells_x + gx
        np.minimum.at(min_elev.ravel(), flat_idx, gz)
        np.maximum.at(max_elev.ravel(), flat_idx, gz)
        np.add.at(counts.ravel(), flat_idx, 1)
        observed = counts > 0

        cost = np.zeros(shape, dtype=np.float32)

        # 1. Step layer — intra-cell elevation range.
        step = np.where(observed, max_elev - min_elev, 0.0)
        cost = np.maximum(cost, np.clip(step / self.max_step * 100.0, 0.0, 100.0))

        # 2. Height layer — deviation from ground, slope-tolerant with distance.
        cx_idx = np.arange(self.num_cells_x, dtype=np.float32)
        cy_idx = np.arange(self.num_cells_y, dtype=np.float32)
        cell_x = (cx_idx + 0.5) * self.resolution - half_x
        cell_y = (cy_idx + 0.5) * self.resolution - half_y
        dist = np.hypot(cell_x[None, :], cell_y[:, None])
        tolerance = self.max_step + dist * self.slope_tan
        too_high = observed & (max_elev > tolerance)
        too_low = observed & (min_elev < -tolerance)
        cost[too_high | too_low] = 100.0

        costmap = np.full(shape, -1, dtype=np.int8)
        costmap[observed] = cost[observed].astype(np.int8)

        # 3. Semantic layer — project points into the hazard mask.
        if self.latest_hazard_mask is not None:
            mh, mw = self.latest_hazard_mask.shape[:2]
            cam_pts = pts[valid]
            z = np.maximum(cam_pts[:, 2], 0.1)
            u = np.round(cam_pts[:, 0] / z * self.fx + self.cx).astype(np.int32)
            v = np.round(cam_pts[:, 1] / z * self.fy + self.cy).astype(np.int32)
            in_img = (u >= 0) & (u < mw) & (v >= 0) & (v < mh)
            if np.any(in_img):
                hits = self.latest_hazard_mask[v[in_img], u[in_img]] > 128
                if np.any(hits):
                    costmap[gy[in_img][hits], gx[in_img][hits]] = 100

        # Robot footprint is always free.
        ccx, ccy = self.num_cells_x // 2, self.num_cells_y // 2
        r = int(self.clear_radius / self.resolution)
        costmap[ccy - r:ccy + r + 1, ccx - r:ccx + r + 1] = 0

        self._publish(costmap, msg.header.stamp, half_x, half_y)

    def _publish(self, costmap: np.ndarray, stamp, half_x: float, half_y: float) -> None:
        grid = OccupancyGrid()
        grid.header.stamp = stamp
        grid.header.frame_id = self.base_frame
        meta = MapMetaData()
        meta.map_load_time = stamp
        meta.resolution = self.resolution
        meta.width = self.num_cells_x
        meta.height = self.num_cells_y
        origin = Pose()
        origin.position.x = -half_x
        origin.position.y = -half_y
        origin.orientation.w = 1.0
        meta.origin = origin
        grid.info = meta
        grid.data = costmap.ravel().tolist()
        self.grid_pub.publish(grid)


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
