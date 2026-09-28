# Autonomous UGV — Visual SLAM + Camera-Only Navigation

A fully autonomous Unmanned Ground Vehicle (UGV) stack built on **ROS 2 Jazzy** and **Gazebo Harmonic**, using a **camera-only** perception pipeline (no lidar). The system performs real-time Visual SLAM, object detection, terrain traversability analysis, and autonomous point-to-point navigation in a warehouse environment.

---

## Table of Contents

- [System Overview](#system-overview)
- [Architecture](#architecture)
- [Package Structure](#package-structure)
- [Prerequisites](#prerequisites)
- [Installation & Build](#installation--build)
- [Running the System](#running-the-system)
- [Launch Arguments](#launch-arguments)
- [Sensor Data Flow](#sensor-data-flow)
- [Topic Reference](#topic-reference)
- [TF Frame Tree](#tf-frame-tree)
- [Configuration Files](#configuration-files)
- [Production Depth Source Swap](#production-depth-source-swap)
- [Troubleshooting](#troubleshooting)
- [Known Limitations](#known-limitations)

---

## System Overview

```
┌─────────────────────────────────────────────────────────────────┐
│                    Autonomous UGV Stack                         │
│                                                                 │
│  Gazebo Harmonic (Warehouse Sim)                                │
│    ├── RGB Camera    → /camera/image_raw      (15 Hz)          │
│    ├── Depth Camera  → /camera/depth/image_raw (15 Hz, 32FC1)  │
│    ├── IMU           → /imu/data              (30 Hz)          │
│    └── DiffDrive Odom→ /odometry/filtered     (varies)         │
│                                                                 │
│  Perception Layer                                               │
│    ├── depth_relay_node  → /perception/depth/image_raw         │
│    ├── point_cloud_xyz   → /perception/depth/points            │
│    └── yolo_seg_node     → /perception/hazard_mask             │
│                                                                 │
│  SLAM (RTAB-Map RGB-D)                                          │
│    ├── rgbd_sync         → /rtabmap/rgbd_image                 │
│    └── rtabmap           → /map (OccupancyGrid, builds live)   │
│                                                                 │
│  Navigation (Nav2)                                              │
│    ├── SmacPlanner2D     → global path (/plan)                 │
│    └── RegulatedPurePursuit → /cmd_vel → Gazebo DiffDrive      │
└─────────────────────────────────────────────────────────────────┘
```

**Key design decisions:**
- **No lidar** — depth from camera only (Gazebo metric depth sensor in sim; RealSense/ZED/Depth Anything V3 in production)
- **Visual SLAM** (RTAB-Map RGB-D) — builds 2D occupancy map as robot explores
- **Unknown space navigation** — robot navigates into unmapped areas, building map on the go
- **Source of truth = Gazebo** — RViz visualizes data from Gazebo, not its own world

---

## Architecture

```
                        ┌──────────────┐
                        │   Gazebo     │
                        │  (Harmonic)  │
                        └──────┬───────┘
                               │  ros_gz_bridge
              ┌────────────────┼────────────────┐
              ▼                ▼                 ▼
       /camera/image_raw  /camera/depth/    /odometry/filtered
       /camera/camera_info  image_raw       /tf (odom→base)
                               │
                        ┌──────▼──────┐
                        │ depth_relay │  ← sim mode
                        │    node     │  (+ QoS adapter)
                        └──────┬──────┘
                               │ /perception/depth/image_raw (Reliable)
              ┌────────────────┼────────────────┐
              ▼                                 ▼
    ┌──────────────────┐              ┌────────────────────┐
    │   rgbd_sync      │              │  point_cloud_xyz   │
    │  (RGB + Depth)   │              │  /perception/depth │
    └────────┬─────────┘              │  /points (PC2)     │
             │ /rtabmap/rgbd_image    └────────┬───────────┘
    ┌────────▼─────────┐                      │
    │    RTAB-Map      │              ┌────────▼──────────┐
    │   Visual SLAM    │              │   Nav2 Local      │
    │  /map → odom→TF  │──────────────│  ObstacleLayer    │
    └────────┬─────────┘              └────────────────────┘
             │ /map
    ┌────────▼─────────┐
    │  Nav2 Planner    │
    │  (SmacPlanner2D) │
    └────────┬─────────┘
             │ /plan
    ┌────────▼─────────┐
    │  Nav2 Controller │
    │  (RegPurePursuit)│
    └────────┬─────────┘
             │ /cmd_vel → Gazebo DiffDrive
    ┌────────▼─────────┐
    │   YOLO Seg Node  │
    │  /camera/image_raw│
    └──────────────────┘
      /perception/hazard_mask
      /perception/yolo/overlay
```

---

## Package Structure

```
src/
├── ugv_bringup/          # Master launch files and configuration
│   ├── launch/
│   │   ├── ugv_complete.launch.py   ← MAIN ENTRY POINT
│   │   ├── localization.launch.py   ← RTAB-Map SLAM + rgbd_sync
│   │   ├── navigation.launch.py     ← Nav2 (planner, controller, BT)
│   │   ├── perception.launch.py     ← Depth relay, YOLO, point cloud
│   │   └── rviz.launch.py           ← RViz2 visualization
│   └── config/
│       ├── nav2_params.yaml         ← Nav2 tuning (RPP + SmacPlanner)
│       ├── rtabmap.yaml             ← RTAB-Map SLAM parameters
│       └── ekf.yaml                 ← robot_localization EKF (hw mode)
│
├── ugv_sim/              # Gazebo simulation environment
│   ├── launch/sim.launch.py         ← Gazebo + bridge + RSP + joint relay
│   ├── config/gazebo_bridge.yaml    ← GZ↔ROS topic bridging rules
│   ├── models/tugbot/model.sdf      ← Tugbot robot SDF (cameras at 15Hz)
│   └── worlds/tugbot_warehouse.sdf  ← Warehouse world (ogre2 renderer)
│
├── ugv_perception/       # AI perception stack (Python nodes)
│   └── ugv_perception/
│       ├── depth_relay_node.py      ← Gazebo depth → pipeline (sim)
│       ├── realsense_relay_node.py  ← RealSense D435/D455 relay (hw)
│       ├── zed_relay_node.py        ← ZED 2/ZED X relay (hw)
│       ├── depth_node.py            ← Depth Anything V3 (monocular)
│       ├── yolo_seg_node.py         ← YOLOv8 hazard segmentation
│       ├── hazard_mask_node.py      ← Semantic hazard mask builder
│       └── depth_to_pointcloud_node.py ← (utility, not used in main launch)
│
├── ugv_terrain/          # Terrain traversability analysis
│   ├── launch/terrain_mapping.launch.py
│   └── ugv_terrain/terrain_analysis_node.py
│
├── ugv_description/      # Robot URDF model and TF
│   ├── urdf/ugv_base.urdf.xacro     ← Robot kinematic tree
│   └── launch/robot_state_publisher.launch.py
│
└── ugv_control/          # Hardware ros2_control spawner (hw mode only)
    └── launch/control.launch.py     ← diff_drive_controller spawner
```

---

## Prerequisites

| Dependency | Version | Notes |
|---|---|---|
| ROS 2 | Jazzy (Ubuntu 24.04) | Required |
| Gazebo | Harmonic (gz-sim 8) | Required for simulation |
| `ros_gz_bridge` | Latest | GZ↔ROS topic bridge |
| `rtabmap_ros` | ≥0.21 | Visual SLAM |
| `nav2` | Jazzy | Autonomous navigation |
| `depth_image_proc` | Jazzy | Depth→PointCloud |
| `robot_localization` | Jazzy | EKF (hw mode only) |
| `ultralytics` | ≥8.0 | YOLO (Python pip) |
| Python | 3.10+ | |
| CUDA (optional) | ≥11.8 | Accelerates YOLO |

Install ROS 2 dependencies:
```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-rtabmap-ros \
  ros-jazzy-nav2-bringup \
  ros-jazzy-depth-image-proc \
  ros-jazzy-robot-localization \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-twist-mux

# Python perception dependencies
pip install ultralytics opencv-python torch torchvision
```

---

## Installation & Build

```bash
# Clone the repository
git clone https://github.com/PrakashKumarSahu/Autonomous-UGV.git
cd Autonomous-UGV

# Install ROS 2 dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build all packages
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source the workspace
source install/setup.bash
```

> **Note:** Add `source ~/Autonomous-UGV/install/setup.bash` to your `~/.bashrc` to auto-source on terminal open.

---

## Running the System

### Simulation Mode (default)

```bash
source install/setup.bash
ros2 launch ugv_bringup ugv_complete.launch.py mode:=sim
```

Gazebo and RViz open automatically. Wait ~30–60 seconds for the warehouse world to fully load before setting navigation goals.

### Send a Navigation Goal

In RViz: use the **"Nav2 Goal"** tool (green arrow icon in toolbar) to click a goal position on the map.

Or from terminal:
```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 5.0, y: 2.0, z: 0.0}, orientation: {w: 1.0}}}}"
```

### Verify Pipeline is Running

```bash
# All these should be non-zero Hz
ros2 topic hz /camera/image_raw          # ~5–15 Hz (RGB camera)
ros2 topic hz /camera/depth/image_raw    # ~5–15 Hz (Depth from Gazebo)
ros2 topic hz /perception/depth/image_raw # ~5–15 Hz (relay output)
ros2 topic hz /perception/depth/points   # ~5–15 Hz (PointCloud for Nav2)
ros2 topic hz /rtabmap/rgbd_image        # ~1–3 Hz (SLAM input)
ros2 topic hz /map                       # ~0.5–1 Hz (SLAM map output)
```

---

## Launch Arguments

| Argument | Default | Description |
|---|---|---|
| `mode` | `sim` | `sim` = Gazebo simulation \| `hw` = real robot |
| `use_sim_time` | `true` | Use Gazebo `/clock` topic |
| `enable_rviz` | `true` | Launch RViz2 dashboard |
| `enable_rtabmap` | `true` | Enable RTAB-Map Visual SLAM |
| `camera_type` | `sim` | Depth source (see below) |
| `enable_depth_viz` | `false` | Show Depth Anything V3 overlay in RViz |

### `camera_type` Options

| Value | Depth Source | Use Case |
|---|---|---|
| `sim` | Gazebo depth_camera sensor (metric, 0.1–10 m) | Default simulation |
| `monocular` | Depth Anything V3 (monocular estimation) | Any RGB-only camera |
| `realsense` | Intel RealSense D435/D455 | Indoor production |
| `zed` | Stereolabs ZED 2/ZED X | Outdoor, long-range |

All sources publish to **identical topics** (`/perception/depth/image_raw`, `/perception/depth/camera_info`) — the rest of the pipeline is hardware-agnostic.

---

## Sensor Data Flow

```
SIM MODE (mode:=sim, camera_type:=sim)
────────────────────────────────────────────────────────────────────
Gazebo depth_camera sensor (ogre2, 15 Hz, 32FC1, 0.1–10 m)
  │  GZ topic: /world/world_demo/model/tugbot/link/camera_front/
  │            sensor/depth/depth_image   ← NOT "image" (Harmonic quirk)
  ▼
ros_gz_bridge  → /camera/depth/image_raw  (BestEffort, 32FC1)
  ▼
depth_relay_node  (BestEffort subscriber, Reliable publisher)
  │  Fixes: frame_id → camera_link_optical
  │  Fixes: stamp sync to depth image header
  ▼
/perception/depth/image_raw  (Reliable, 32FC1, metric metres)
/perception/depth/camera_info (Reliable)

HW MODE (mode:=hw, camera_type:=realsense)
────────────────────────────────────────────────────────────────────
RealSense D435 driver → /camera/aligned_depth_to_color/image_raw (16UC1, mm)
  ▼
realsense_relay_node  (converts 16UC1 mm → 32FC1 metres)
  ▼
/perception/depth/image_raw  (same canonical topic)
```

---

## Topic Reference

| Topic | Type | Publisher | Subscribers | QoS |
|---|---|---|---|---|
| `/camera/image_raw` | `sensor_msgs/Image` | Gazebo bridge | YOLO, rgbd_sync | BestEffort |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | Gazebo bridge | depth_relay (fallback), rgbd_sync | BestEffort |
| `/camera/depth/image_raw` | `sensor_msgs/Image` | Gazebo bridge | depth_relay_node | BestEffort |
| `/camera/depth/camera_info` | `sensor_msgs/CameraInfo` | Gazebo bridge | depth_relay_node | BestEffort |
| `/perception/depth/image_raw` | `sensor_msgs/Image` | depth_relay_node | rgbd_sync, point_cloud_xyz | Reliable |
| `/perception/depth/camera_info` | `sensor_msgs/CameraInfo` | depth_relay_node | rgbd_sync, point_cloud_xyz | Reliable |
| `/perception/depth/points` | `sensor_msgs/PointCloud2` | point_cloud_xyz_node | Nav2 ObstacleLayer, terrain_analysis | Reliable |
| `/perception/hazard_mask` | `sensor_msgs/Image` | yolo_seg_node | terrain_analysis_node | Reliable |
| `/perception/yolo/overlay` | `sensor_msgs/Image` | yolo_seg_node | RViz | Reliable |
| `/perception/depth_ai/image_raw` | `sensor_msgs/Image` | depth_node (optional) | RViz only | Reliable |
| `/rtabmap/rgbd_image` | `rtabmap_msgs/RGBDImage` | rgbd_sync | rtabmap SLAM | Reliable |
| `/map` | `nav_msgs/OccupancyGrid` | rtabmap | Nav2 global planner | Transient Local |
| `/terrain/traversability_grid` | `nav_msgs/OccupancyGrid` | terrain_analysis_node | RViz | Reliable |
| `/odometry/filtered` | `nav_msgs/Odometry` | Gazebo bridge (sim) / EKF (hw) | Nav2, RTAB-Map | Reliable |
| `/tf` | `tf2_msgs/TFMessage` | Gazebo bridge + RSP + rtabmap | All | Reliable |
| `/cmd_vel` | `geometry_msgs/Twist` | Nav2 controller | Gazebo bridge → DiffDrive | Reliable |
| `/plan` | `nav_msgs/Path` | Nav2 planner | RViz | Reliable |
| `/imu/data` | `sensor_msgs/Imu` | Gazebo bridge | EKF (hw mode only) | BestEffort |

---

## TF Frame Tree

```
map
 └── odom                    ← published by RTAB-Map (map correction)
      └── base_footprint     ← published by Gazebo bridge (DiffDrive odom)
           └── base_link
                ├── left_wheel_link
                ├── right_wheel_link
                ├── imu_link
                └── camera_link
                     └── camera_link_optical   ← all depth/RGB images use this frame
                          └── tugbot/camera_front/color  (Gazebo alias)
```

> **Joint name mismatch fix:** Gazebo SDF uses `wheel_left_joint`/`wheel_right_joint`; URDF uses `left_wheel_joint`/`right_wheel_joint`. The `joint_state_relay.py` node transparently renames them so `robot_state_publisher` can publish correct wheel TFs.

---

## Configuration Files

### `nav2_params.yaml` — Key Parameters

| Parameter | Value | Notes |
|---|---|---|
| `desired_linear_vel` | 0.6 m/s | Robot cruise speed |
| `rotate_to_heading_min_angle` | 0.85 rad (~49°) | Rotates in-place before translating if heading error > this |
| `obstacle_range` | 6.0 m | Local costmap marks obstacles within this range |
| `inflation_radius` | 0.55 m | Obstacle inflation for robot footprint (robot_radius=0.30 m) |
| `allow_unknown` | true | Global planner navigates through unexplored space |
| `xy_goal_tolerance` | 0.25 m | Acceptable goal position error |

### `rtabmap.yaml` — Key Parameters

| Parameter | Value | Notes |
|---|---|---|
| `wait_for_transform` | 1.0 s | Startup tolerance for TF availability |
| `tf_tolerance` | 0.5 s | TF lookup tolerance during operation |
| `Grid/RangeMax` | 10.0 m | Matches Gazebo depth sensor max range |
| `Grid/CellSize` | 0.10 m | Map resolution |
| `Vis/FeatureType` | 8 (FAST/ORB) | Feature extractor for visual odometry |
| `Reg/Force3DoF` | true | 2D SLAM (planar warehouse environment) |

---

## Production Depth Source Swap

The depth pipeline is fully swappable without modifying any code. All four sources publish to **identical canonical topics**:

```bash
# Simulation (Gazebo real depth sensor — default)
ros2 launch ugv_bringup ugv_complete.launch.py mode:=sim

# Hardware — monocular camera only (Depth Anything V3)
# Requires: pip install depth-anything-v3
ros2 launch ugv_bringup ugv_complete.launch.py mode:=hw camera_type:=monocular

# Hardware — Intel RealSense D435/D455
# Requires: ros-jazzy-realsense2-camera
ros2 launch ugv_bringup ugv_complete.launch.py mode:=hw camera_type:=realsense

# Hardware — Stereolabs ZED 2 or ZED X
# Requires: ZED SDK + ros-jazzy-zed-ros2-wrapper
ros2 launch ugv_bringup ugv_complete.launch.py mode:=hw camera_type:=zed

# Compare AI depth vs real depth in RViz (sim only, costs extra GPU)
ros2 launch ugv_bringup ugv_complete.launch.py mode:=sim enable_depth_viz:=true
```

---

## Troubleshooting

### "No map received" in RViz SLAMMap

**Check in order:**
```bash
# 1. Is Gazebo fully loaded? (wait 45–60s after launch)
ros2 topic hz /clock              # Must be non-zero (~40 Hz)

# 2. Is the depth camera working?
ros2 topic hz /camera/depth/image_raw    # Must be ~5–15 Hz

# 3. Is the depth relay running?
ros2 topic hz /perception/depth/image_raw  # Must match above

# 4. Is rgbd_sync producing output?
ros2 topic hz /rtabmap/rgbd_image   # Must be ~1–3 Hz

# 5. Is RTAB-Map producing a map?
ros2 topic hz /map                  # Should be ~0.5–1 Hz
```

**Common causes:**
- Gazebo not fully loaded yet (wait longer before checking)
- Old Gazebo process still running: `pkill -f "gz sim"`

### Robot not moving to Nav2 goal

```bash
# Check TF chain is complete
ros2 run tf2_tools view_frames  # Should show: map→odom→base_footprint

# Check if Nav2 is active
ros2 node list | grep nav2

# Check if SLAM has published any map
ros2 topic echo /map --once
```

### "Detected jump back in time" warnings

**Normal** — these appear for 2–3 seconds when Gazebo first starts or after a restart as the sim clock initializes. They do not affect functionality.

### YOLO fails to load / no hazard mask

YOLO runs in a graceful fallback mode if the model fails to load. Check:
```bash
pip install ultralytics       # Install if missing
ros2 topic echo /perception/yolo/overlay  # Check output
```

### Camera feeds not appearing in RViz

Camera topics use `BestEffort` QoS from Gazebo bridge. Make sure RViz Image displays are set to **Best Effort** reliability (already configured in the provided `.rviz` file).

---

## Known Limitations

1. **TF clock jump warnings at startup** — Normal Gazebo sim clock initialization behavior. Clears within 3 seconds.

2. **SLAM requires robot movement** — RTAB-Map builds the map as the robot moves. Set a Nav2 goal to start mapping. The initial area around the spawn point will be mapped first.

3. **Camera-only depth** — Without lidar, the depth camera has a limited 80° FOV and 10m range. Objects behind and beside the robot are not seen until the robot rotates.

4. **Depth Anything V3 (monocular) scale** — In `camera_type:=monocular` mode, depth is relative (not absolute metric). Scale calibration is required for accurate obstacle distances in hardware mode.

5. **Warehouse world load time** — The warehouse SDF with all collision meshes takes 30–60 seconds to fully load in Gazebo. Navigation goals set before world loads may fail.

6. **Navigation goal behavior** — If no path exists to the goal (fully blocked), Nav2 will attempt spin/backup recovery behaviors then report failure. The robot stops at its last position; it does not return to start.

---

## Repository

**GitHub:** https://github.com/PrakashKumarSahu/Autonomous-UGV.git  
**Branch:** `main`  
**ROS 2:** Jazzy | **Gazebo:** Harmonic | **Ubuntu:** 24.04
