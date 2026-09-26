# Autonomous UGV: System Architecture & Subsystem Specification

## 1. Overview
This autonomous mobile robot stack is engineered for outdoor unstructured environments where GPS signals are degraded, spoofed, or denied. It integrates monocular vision AI, 2.5D elevation and hazard mapping, visual SLAM, sensor fusion, and non-holonomic motion planning on ROS 2 Jazzy.

## 2. Subsystem Data Flow

```
[Camera & IMU] ──> [ugv_perception] ──> [ugv_terrain] ──> [Nav2 Costmaps]
        │                   │                                    │
        └───> [RTAB-Map] ───┴─> [robot_localization EKF] ───> [Nav2 Planners] ──> [/cmd_vel]
```

### 2.1 Sensor Layer
- **Monocular RGB Camera**: Publishes 640x480 images at 30 Hz on `/camera/image_raw` with calibrated intrinsics on `/camera/camera_info`.
- **6-Axis IMU**: Publishes angular velocity and linear acceleration at 50 Hz on `/imu/data`.
- **Wheel Encoders**: Publishes wheel odometry at 50 Hz on `/diff_drive_controller/odom`.

### 2.2 Perception AI (`ugv_perception`)
- **Depth Estimation**: Infers relative/metric depth (`32FC1`) using Depth Anything architecture and derives a 3D point cloud (`sensor_msgs/PointCloud2`).
- **Hazard Segmentation**: Computes instance segmentation masks using YOLO11/YOLOv8 and formats a pixel-aligned hazard mask (`0`=safe, `128`=caution, `255`=lethal hazard).

### 2.3 2.5D Terrain Analysis (`ugv_terrain`)
- Evaluates elevation variance, step obstacles, and slope angle.
- Integrates the semantic hazard mask and publishes `/terrain/traversability_grid` (`nav_msgs/OccupancyGrid`) at 10-15 Hz.

### 2.4 Localization & Fusion (`ugv_bringup`, RTAB-Map, EKF)
- **RTAB-Map**: Synchronizes RGB and Depth to compute 6-DOF visual odometry (`/rtabmap/odom`).
- **robot_localization**: Extended Kalman Filter fuses visual odometry with wheel odometry and 50 Hz IMU, broadcasting `odom -> base_footprint` TF.

### 2.5 Path Planning & Collision Avoidance (Nav2)
- **Global Planner**: `SmacPlannerHybrid` plans kinematically viable paths around boulders, trees, and steep slopes.
- **Local Controller**: `RegulatedPurePursuitController` dynamically regulates vehicle speed based on curvature and obstacle proximity.
