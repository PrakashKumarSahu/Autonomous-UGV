# Autonomous-UGV
### Vision-Based Autonomous Navigation for Outdoor UGV (BEL Problem Statement 26126)

A modular, camera-primary autonomous mobile robot stack designed for unstructured outdoor terrains where GPS is unreliable, degraded, or denied. Built on **ROS 2 Jazzy Jalisco**, **Ubuntu 24.04 LTS**, and **Gazebo Harmonic**.

---

## 1. System Architecture

```
[Camera & IMU] ──> [ugv_perception] ──> [ugv_terrain] ──> [Nav2 Costmaps]
        │                   │                                    │
        └───> [RTAB-Map] ───┴─> [robot_localization EKF] ───> [Nav2 Planners] ──> [/cmd_vel]
```

- **Perception AI (`ugv_perception`)**: Depth Anything V2/V3 metric depth and point cloud generation + YOLO11/YOLOv8 instance segmentation classifying traversable paths from natural hazards (boulders, trees, ditches).
- **2.5D Terrain Mapping (`ugv_terrain`)**: Rolling multi-layer elevation, slope gradient, and roughness grid fused with semantic hazard masks into `nav_msgs/OccupancyGrid`.
- **Visual SLAM & Localization (`ugv_bringup`, RTAB-Map, EKF)**: Virtual RGB-D visual odometry and loop-closure SLAM fused with 50 Hz IMU and wheel encoders via `robot_localization` EKF.
- **Path Planning & Reactive Avoidance (Nav2)**: Smac Hybrid-A* global path planner + Regulated Pure Pursuit local controller executing adaptive curvature and proximity speed regulation.
- **Actuation (`ugv_control`)**: Standard `ros2_control` differential drive controller.
- **Simulation Environment (`ugv_sim`)**: Gazebo Harmonic (`gz-sim 8.11`) with realistic lighting, physics, and unstructured outdoor obstacle terrain.

---

## 2. Package Organization

```
Autonomous-UGV/
├── .gitignore
├── README.md
├── docs/
│   ├── system_architecture.md
│   ├── tf_tree.md
│   └── parameter_guide.md
└── src/
    ├── ugv_bringup/       # Master launch orchestrator, RViz dashboard, and YAML configs
    ├── ugv_perception/    # Depth Anything and YOLOv8/YOLO11 perception nodes
    ├── ugv_terrain/       # 2.5D multi-layer elevation and hazard traversability mapper
    ├── ugv_description/   # Robot URDF/Xacro, sensor frames, Gazebo plugins
    ├── ugv_sim/           # Gazebo Harmonic outdoor world and ros_gz_bridge configuration
    └── ugv_control/       # ros2_control diff-drive controller parameters
```

---

## 3. Quickstart & Build Instructions

### 3.1 Build the Stack
This repository acts directly as your self-contained ROS 2 Workspace:

```bash
cd /home/ros2/Autonomous-UGV   # Or active worktree directory
colcon build --symlink-install
source install/setup.bash
```

### 3.2 Launch Options

#### Option A: Full System Simulation (Gazebo Harmonic + Autonomy Stack + RViz)
```bash
ros2 launch ugv_bringup ugv_complete.launch.py mode:=sim
```

#### Option B: Physical Hardware Deployment
```bash
ros2 launch ugv_bringup ugv_complete.launch.py mode:=hw
```

#### Option C: Modular Subsystem Bringup
```bash
# 1. Launch Gazebo Harmonic simulation only
ros2 launch ugv_sim sim.launch.py

# 2. Launch AI Perception (Depth + YOLO Segmentation)
ros2 launch ugv_bringup perception.launch.py

# 3. Launch 2.5D Terrain Traversability Analysis
ros2 launch ugv_terrain terrain_mapping.launch.py

# 4. Launch Visual SLAM & EKF State Estimation
ros2 launch ugv_bringup localization.launch.py

# 5. Launch Nav2 Autonomous Navigation
ros2 launch ugv_bringup navigation.launch.py
```

---

## 4. Key Topic Interfaces

| Topic | Type | Description |
| :--- | :--- | :--- |
| `/camera/image_raw` | `sensor_msgs/msg/Image` | Monocular RGB camera stream |
| `/camera/camera_info` | `sensor_msgs/msg/CameraInfo` | Camera intrinsic calibration parameters |
| `/perception/depth/image_raw` | `sensor_msgs/msg/Image` (`32FC1`) | Metric monocular depth map |
| `/perception/depth/points` | `sensor_msgs/msg/PointCloud2` | 3D dense point cloud |
| `/perception/hazard_mask` | `sensor_msgs/msg/Image` (`mono8`) | Segmented hazard mask (`0`=safe, `255`=hazard) |
| `/odometry/filtered` | `nav_msgs/msg/Odometry` | Fused drift-free 6-DOF odometry |
| `/terrain/traversability_grid` | `nav_msgs/msg/OccupancyGrid` | 2.5D slope + hazard traversability costmap |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Velocity command to wheel actuators |
