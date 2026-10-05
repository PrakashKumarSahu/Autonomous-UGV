# Autonomous UGV — Vision-Based Outdoor Navigation

[![ROS 2 Jazzy](https://img.shields.io/badge/ROS_2-Jazzy-22314E?logo=ros)](https://docs.ros.org/en/jazzy/)
[![Gazebo Harmonic](https://img.shields.io/badge/Gazebo-Harmonic-FF6F00?logo=gazebo)](https://gazebosim.org/docs/harmonic/)
[![YOLO26](https://img.shields.io/badge/YOLO-26n--seg-00FFFF?logo=ultralytics)](https://github.com/ultralytics/ultralytics)
[![Depth Anything V2](https://img.shields.io/badge/Depth_Anything-V2_Metric-FF69B4?logo=huggingface)](https://huggingface.co/depth-anything/Depth-Anything-V2-Metric-Indoor-Small-hf)
[![License: Apache-2.0](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](LICENSE)

An end-to-end autonomous ground vehicle navigation stack for **GPS-denied outdoor environments** using **a single RGB-D camera as the primary sensor**. Built on **ROS 2 Jazzy** and **Gazebo Harmonic (gz-sim 8)**, the system features:

- **Perception AI:** Real-time instance segmentation with **YOLO26n-seg** (Ultralytics, NMS-free, FP16) and metric depth estimation with **Depth Anything V2 Metric**.
- **Visual SLAM:** 3-DoF planar SLAM with **RTAB-Map** using RGB-D odometry and loop-closure to build a persistent 2D occupancy grid (`/map`) and publish `map → odom` corrections.
- **2.5D Terrain Traversability:** A rolling, slope-tolerant height-deviation and step-analysis grid (`/terrain/traversability_grid`) with YOLO hazard back-projection.
- **Autonomous Navigation:** **Nav2** with a 2D SmacPlanner (`GridBased`) and Regulated Pure Pursuit (`RPP`) path tracking, dynamic obstacle inflation, and recovery behaviors.
- **Full RViz2 Dashboard:** A 17-display control center displaying height-colored point clouds (AxisColor Z), live SLAM, local/global costmaps, path execution, odometry trails, and camera/hazard overlays.

> **No LiDAR. No GPS. Pure vision-based autonomy from Point A to Point B.**

---

## Table of Contents

- [System Architecture](#system-architecture)
- [Outdoor Demonstration World](#outdoor-demonstration-world)
- [Package Structure](#package-structure)
- [Hardware & Environment Requirements](#hardware--environment-requirements)
- [Installation & Build](#installation--build)
- [Quick Start](#quick-start)
- [Launch Arguments](#launch-arguments)
- [Core Modules](#core-modules)
  - [1. ugv_sim — Simulation & Gazebo Bridge](#1-ugv_sim--simulation--gazebo-bridge)
  - [2. ugv_description — URDF & TF Tree](#2-ugv_description--urdf--tf-tree)
  - [3. ugv_perception — YOLO26 & Depth Pipeline](#3-ugv_perception--yolo26--depth-pipeline)
  - [4. ugv_terrain — Traversability Analysis](#4-ugv_terrain--traversability-analysis)
  - [5. ugv_bringup — Orchestration, SLAM & Nav2](#5-ugv_bringup--orchestration-slam--nav2)
  - [6. ugv_control — Hardware ros2_control Scaffold](#6-ugv_control--hardware-ros2_control-scaffold)
- [Topic and QoS Contracts](#topic-and-qos-contracts)
- [Configuration Reference](#configuration-reference)
- [Author & Maintenance](#author--maintenance)

---

## System Architecture

```
                                  ┌─────────────────────────────┐
                                  │   Gazebo Harmonic / HW      │
                                  │  (Camera, IMU, Diff-Drive)  │
                                  └──────────────┬──────────────┘
                                                 │ GZ / ROS Bridge
                  ┌──────────────────────────────┼──────────────────────────────┐
                  │                              │                              │
                  ▼                              ▼                              ▼
      /camera/image_raw (bgr8)      /camera/depth/image_raw (32FC1)    /odometry/filtered & /tf
                  │                              │                              │
                  │                              ▼                              │
                  │                    ┌──────────────────┐                     │
                  │                    │ depth_relay_node │                     │
                  │                    └─────────┬────────┘                     │
                  │                              │                              │
                  │             /perception/depth/image_raw (RELIABLE)          │
                  │             /perception/depth/camera_info (RELIABLE)        │
                  │                              │                              │
        ┌─────────┴─────────┐          ┌─────────┴─────────┐                    │
        ▼                   ▼          ▼                   ▼                    │
┌──────────────┐     ┌─────────────┐ ┌─────────────┐ ┌──────────────┐           │
│yolo_seg_node │     │ depth_node  │ │ depth_to_   │ │  rtabmap_    │           │
│  (YOLO26n)   │     │ (DA V2-M)   │ │ pointcloud  │ │  sync/slam   │◄──────────┘
└───────┬──────┘     └──────┬──────┘ └──────┬──────┘ └──────┬───────┘
        │ /perception/      │ /perception/  │ /perception/  │ /map
        │ hazard_mask       │ depth_ai/*    │ depth/points  │ map→odom TF
        │                   │ (RViz viz)    │               │
        └──────────────┬────┴───────────────┘               ▼
                       ▼                       ┌────────────────────────┐
             ┌───────────────────┐             │       Nav2 Stack       │
             │ terrain_analysis_ │             │ Smac2D Planner         │
             │       node        │             │ Regulated Pure Pursuit │
             └─────────┬─────────┘             │ Costmaps & Recovery    │
                       │                       └────────────┬───────────┘
                       ▼                                    │ /cmd_vel
             /terrain/traversability_grid                   ▼
             (Rolling OccupancyGrid, RViz)           Gazebo DiffDrive
```

---

## Outdoor Demonstration World

The simulation environment (`ugv_sim/worlds/ugv_test_arena.sdf`) is an open 500m × 500m natural outdoor field built with native SDFormat primitives (zero external Gazebo Fuel or mesh dependencies).

```
                     [Goal B: Gate & Cyan Pad (0, +12.0)]
                                    ▲
                         ▲       [Tree 10]       ▲
                      [Tree 09]      │        [Tree 08]
                         │           ▼
                   [Rock Wall A/B/C: (0, +7.5) - Chokepoint]
                         │
                 [Stream Bed]   [Step Hill]   [Waypoint Cone 2]
                         │
                     [Tree 05]   [Bush 03]    [Fallen Log]
                         │
                   [Waypoint Cone 1]  [Rock 04]  [Ditch Ridge]
                                    │
                                 [Mound]
                                    ▲
                    [Start A: Gate & Green Pad (0, -6.5)]
                     (Localbot spawns facing North +Y)
```

Key environmental features:
- **Navigation Course:** Direct North-South corridor (18.5 meters from Point A to Point B).
- **Obstacle Clusters:** 12 trees with trunks and spherical foliage, 14 rock formations, fallen timber, natural brush, and a rock chokepoint at $Y \approx +7.5$ requiring dynamic path replanning.
- **Terrain Elevation:** Uneven earth mounds, a sunken ditch with approach banks, a stepped terrace, and orange route cones at $(0, -1.0)$, $(2.0, +3.5)$, and $(-1.0, +9.5)$.
- **Harsh Visuals & Lighting:** Ambient sky fill, directional sunlight with shadows disabled for rendering stability, and textured dirt trail tiles.

---

## Package Structure

```
Autonomous-UGV/
├── docs/                        # Architecture guides and reference material
├── src/
│   ├── ugv_bringup/             # Master orchestrator, Nav2 & RTAB-Map configurations
│   │   ├── config/              # nav2_params.yaml, rtabmap.yaml, ekf.yaml
│   │   ├── launch/              # ugv_complete, localization, navigation, perception, rviz
│   │   └── rviz/                # ugv_autonomy.rviz (17-display dashboard)
│   ├── ugv_sim/                 # Simulation environment and ROS-GZ Bridge
│   │   ├── models/localbot/     # SDF robot definition, sensors, and diff-drive plugin
│   │   ├── scripts/             # joint_state_relay.py (QoS bridge)
│   │   └── worlds/              # ugv_test_arena.sdf, empty_world.sdf
│   ├── ugv_description/         # URDF/Xacro descriptions and TF publishers
│   │   └── urdf/                # ugv_base.urdf.xacro, camera.urdf.xacro, imu.urdf.xacro
│   ├── ugv_perception/          # AI vision and depth processing stack
│   │   ├── config/              # perception.yaml (YOLO26 & depth tunables)
│   │   └── ugv_perception/      # yolo_seg_node, depth_node, depth_relay_node, pointcloud
│   ├── ugv_terrain/             # 2.5D elevation and traversability mapping
│   │   ├── config/              # terrain.yaml
│   │   └── ugv_terrain/         # terrain_analysis_node.py
│   └── ugv_control/             # ros2_control hardware interface scaffold
└── README.md
```

---

## Hardware & Environment Requirements

| Requirement | Minimum / Tested Version | Notes |
|---|---|---|
| **Operating System** | Ubuntu 24.04 LTS (Noble Numbat) | Native Linux environment recommended |
| **ROS 2 Distribution** | ROS 2 Jazzy Jalisco | Base desktop installation |
| **Simulator** | Gazebo Harmonic (`gz-sim 8`) | Integrated via `ros_gz_sim` & `ros_gz_bridge` |
| **Python** | Python 3.12 | Standard distribution runtime |
| **GPU / Acceleration** | NVIDIA RTX 4050 Laptop GPU (or higher) | CUDA 12+ / PyTorch with CUDA support |
| **PyTorch & Ultralytics** | `torch >= 2.4`, `ultralytics >= 8.4.173` | YOLO26 requires Ultralytics 8.4.170+ |
| **NumPy** | `numpy == 1.26.4` (1.x branch) | Required for ROS 2 Jazzy `cv_bridge` compatibility |

---

## Installation & Build

### 1. Install ROS 2 Jazzy & System Dependencies

```bash
sudo apt update && sudo apt install -y \
  ros-jazzy-desktop \
  ros-jazzy-nav2-bringup \
  ros-jazzy-nav2-msgs \
  ros-jazzy-nav2-smac-planner \
  ros-jazzy-nav2-regulated-pure-pursuit-controller \
  ros-jazzy-rtabmap-ros \
  ros-jazzy-rtabmap-sync \
  ros-jazzy-rtabmap-slam \
  ros-jazzy-robot-localization \
  ros-jazzy-ros-gz-sim \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-ros-gz-image \
  ros-jazzy-cv-bridge \
  ros-jazzy-sensor-msgs-py \
  ros-jazzy-tf2-ros \
  ros-jazzy-xacro \
  python3-pip
```

### 2. Install Python AI Libraries

Ensure `numpy` is maintained on the 1.x branch to prevent C-API ABI mismatches with system `cv_bridge`:

```bash
pip install --break-system-packages \
  "numpy<2" \
  "ultralytics>=8.4.173" \
  "transformers" \
  "pillow" \
  "opencv-python"
```

Verify GPU acceleration and model readiness:
```bash
python3 -c "import torch, ultralytics; print(f'CUDA available: {torch.cuda.is_available()}, Ultralytics: {ultralytics.__version__}')"
```

### 3. Clone and Build the Workspace

```bash
cd ~
git clone https://github.com/PrakashKumarSahu/Autonomous-UGV.git
cd Autonomous-UGV

source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

---

## Quick Start

### 1. Single-Command Launch (Simulation Mode)

Launch the full stack — Gazebo physics, Localbot spawn, ROS-GZ Bridge, YOLO26 segmentation, Depth relay, RTAB-Map SLAM, Nav2, and RViz:

```bash
ros2 launch ugv_bringup ugv_complete.launch.py
```

### 2. Autonomous Navigation (Point A to Point B)

Once the world and RViz load:
1. In RViz, the robot starts at **Point A** (`(0.0, -6.5)` facing North).
2. Click the **Nav2 Goal** tool on the top toolbar (or press `g`).
3. Click and drag at **Point B** (`(0.0, 12.0)` facing North).
4. Nav2 computes a global path via SmacPlanner2D avoiding trees, rocks, and the stream chokepoint, and drives Localbot to the goal gate via Regulated Pure Pursuit.

Alternatively, send the goal via CLI:
```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  '{pose:{header:{frame_id:"map"},pose:{position:{x:0.0,y:12.0,z:0.0},orientation:{w:1.0}}}}'
```

---

## Launch Arguments

Override parameters directly on the CLI:

| Argument | Default | Options | Description |
|---|---|---|---|
| `mode` | `sim` | `sim`, `hw` | Simulation (Gazebo Harmonic) or real hardware execution. |
| `use_sim_time` | `true` | `true`, `false` | Synchronize against Gazebo `/clock` (`false` in hardware mode). |
| `camera_type` | `sim` | `sim`, `monocular`, `realsense`, `zed` | Depth source abstraction. All publish identical canonical topics. |
| `enable_depth_viz` | `false` | `true`, `false` | Runs Depth Anything V2 Metric alongside Gazebo for side-by-side RViz AI comparison (~2GB VRAM). |
| `enable_rtabmap` | `true` | `true`, `false` | Runs RTAB-Map SLAM (publishes `/map` and `map → odom` transform). |
| `enable_rviz` | `true` | `true`, `false` | Launches pre-configured RViz2 dashboard. |
| `world` | `ugv_test_arena.sdf` | Path to SDF | Path to Gazebo world file. |
| `world_name` | `world_demo` | String | Must match the `<world name="...">` attribute inside the SDF. |
| `robot_name` | `localbot` | String | Entity name in Gazebo. |
| `spawn_x`, `spawn_y` | `0.0`, `-6.5` | Floats | Spawn coordinates in meters (Point A). |
| `spawn_yaw` | `1.5708` | Float (rad) | Initial heading (`1.5708` = North / +Y). |

**Examples:**
```bash
# Run with Monocular AI Depth instead of Gazebo sensor
ros2 launch ugv_bringup ugv_complete.launch.py camera_type:=monocular

# Run in an empty flat world
ros2 launch ugv_bringup ugv_complete.launch.py \
  world:=$(ros2 pkg prefix ugv_sim)/share/ugv_sim/worlds/empty_world.sdf \
  world_name:=empty spawn_x:=0.0 spawn_y:=0.0 spawn_yaw:=0.0
```

---

## Core Modules

### 1. `ugv_sim` — Simulation & Gazebo Bridge

- **Localbot Robot Model:** Native differential-drive robot defined in `models/localbot/model.sdf`. Wheel separation $0.45\text{ m}$, wheel radius $0.10\text{ m}$, mass $15\text{ kg}$, forward-mounted camera at $Z = 0.645\text{ m}$.
- **Sensors:**
  - RGB Camera: 640×480 @ 30 Hz ($80^\circ\text{ H-FOV}$).
  - Depth Camera: 640×480 @ 30 Hz ($0.3\text{ m} - 10.0\text{ m}$ range, 32FC1).
  - IMU: 6-DoF sensor running at 50 Hz.
- **Dynamic Parameter Bridge:** `sim.launch.py` dynamically generates a temporary `ros_gz_bridge` YAML configuration on launch, parameterizing topics to any `world_name` or `robot_name` and safely cleaning up on exit.
- **Joint State Relay:** `scripts/joint_state_relay.py` bridges `/joint_states` from Gazebo (BestEffort QoS) to `/joint_states_urdf` (Reliable QoS) to feed `robot_state_publisher` without TF dropouts.

### 2. `ugv_description` — URDF & TF Tree

Provides the kinematic model of the robot:
- `base_footprint` — Ground projection reference frame.
- `base_link` — Center of chassis mass ($Z = 0.20\text{ m}$).
- `left_wheel_link` / `right_wheel_link` — Driven traction wheels (aligned rotation axis).
- `camera_link` & `camera_link_optical` — Physical housing and standard optical coordinate frame ($Z\text{-forward}$, $X\text{-right}$, $Y\text{-down}$).

### 3. `ugv_perception` — YOLO26 & Depth Pipeline

Centralized in `config/perception.yaml`:
- **`yolo_seg_node`:** Executes `yolo26n-seg.pt` (latest YOLO26 NMS-free architecture, ~6.4MB) with FP16 tensor core acceleration.
  - Subscribes to `/camera/image_raw`.
  - Classifies obstacles into lethal (`255`), caution (`128`), and safe (`0`).
  - Publishes `/perception/hazard_mask` (mono8) and `/perception/yolo/overlay` (bgr8).
  - Fallback: Canny-edge ground-plane saliency detector if GPU or weights are unavailable.
- **`depth_relay_node`:** Canonical relay for Gazebo depth data (`32FC1` metric meters) to `/perception/depth/image_raw` with Reliable QoS and TURBO colorization to `/perception/depth/colorized`.
- **`depth_node`:** Monocular metric depth estimation using `Depth-Anything-V2-Metric-Indoor-Small-hf` directly outputting metric depth.
- **`depth_to_pointcloud_node`:** High-throughput depth-to-cloud projector with vectorized numpy operations, configurable downsampling step, and stamped TF buffer offsets.

### 4. `ugv_terrain` — Traversability Analysis

`ugv_terrain/terrain_analysis_node.py` builds a rolling 2.5D grid in the robot's local frame (`base_footprint`):
- Uses TF2 transforms to resolve camera mount position ($Z \approx 0.645\text{ m}$) relative to ground level.
- Computes cell step height: $\Delta Z = Z_{max} - Z_{min}$ per 10cm grid cell.
- Evaluates slope tolerance: obstacles and drops beyond $Z = \pm(step_{max} + d \cdot \tan(\theta_{slope}))$ are tagged as obstacles.
- Geometric back-projection: projects 3D surface points into the live YOLO26 hazard mask, fusing semantic risk directly into the terrain cost.
- Publishes `/terrain/traversability_grid` (OccupancyGrid) for terrain analysis in RViz.

### 5. `ugv_bringup` — Orchestration, SLAM & Nav2

- **Localization:** `rtabmap_sync/rgbd_sync` packages color, depth, and intrinsics into synchronized `RGBDImage` messages. `rtabmap_slam/rtabmap` runs 3-DoF planar visual SLAM with loop closure detection, publishing `/map` and the `map → odom` transform.
- **Navigation:**
  - `planner_server`: SmacPlanner2D searching the 2D costmap with a 0.75m obstacle inflation margin.
  - `controller_server`: Regulated Pure Pursuit (`RPP`) tracking paths at 20 Hz with collision checking and velocity scaling.
  - `behavior_server`: Spin, backup, and wait recovery behaviors.

### 6. `ugv_control` — Hardware ros2_control Scaffold

Provides controller configurations (`diff_drive_controller.yaml`) and spawner scripts for physical deployment with hardware motor drives.

---

## Topic and QoS Contracts

| Topic | Message Type | Reliability | Publisher | Consumers |
|---|---|---|---|---|
| `/camera/image_raw` | `sensor_msgs/Image` (bgr8) | Best Effort | `ros_gz_bridge` | `yolo_seg_node`, `rgbd_sync` |
| `/camera/depth/image_raw` | `sensor_msgs/Image` (32FC1) | Best Effort | `ros_gz_bridge` | `depth_relay_node` |
| `/perception/depth/image_raw` | `sensor_msgs/Image` (32FC1) | Reliable | `depth_relay_node` | `depth_to_pointcloud_node`, `rgbd_sync` |
| `/perception/depth/camera_info` | `sensor_msgs/CameraInfo` | Reliable | `depth_relay_node` | `depth_to_pointcloud_node`, `terrain_analysis_node` |
| `/perception/depth/colorized` | `sensor_msgs/Image` (bgr8) | Reliable | `depth_relay_node` | RViz (`RealDepth` display) |
| `/perception/depth/points` | `sensor_msgs/PointCloud2` | Reliable | `depth_to_pointcloud_node` | `nav2_costmap_2d`, `terrain_analysis_node` |
| `/perception/hazard_mask` | `sensor_msgs/Image` (mono8) | Reliable | `yolo_seg_node` | `terrain_analysis_node` |
| `/perception/yolo/overlay` | `sensor_msgs/Image` (bgr8) | Reliable | `yolo_seg_node` | RViz (`YOLOHazardOverlay` display) |
| `/terrain/traversability_grid` | `nav_msgs/OccupancyGrid` | Reliable | `terrain_analysis_node` | RViz (`TerrainGrid` display) |
| `/map` | `nav_msgs/OccupancyGrid` | Transient Local | `rtabmap` | `nav2_costmap_2d` (Global), RViz |
| `/odometry/filtered` | `nav_msgs/Odometry` | Best Effort | `ros_gz_bridge` | `rtabmap`, Nav2 Controller |
| `/cmd_vel` | `geometry_msgs/Twist` | Reliable | Nav2 Controller | `ros_gz_bridge` → DiffDrive |

---

## Configuration Reference

Key configuration files and their tuning roles:

| Configuration File | Path | Key Parameters |
|---|---|---|
| **Perception Config** | `src/ugv_perception/config/perception.yaml` | `model_name: yolo26n-seg.pt`, `confidence_threshold: 0.35`, `enable_fp16: true`, `step: 4`, `stamp_offset: 0.15` |
| **Terrain Config** | `src/ugv_terrain/config/terrain.yaml` | `grid_resolution: 0.10`, `max_step_height: 0.22`, `max_slope_angle: 25.0`, `footprint_clear_radius: 0.6` |
| **Nav2 Parameters** | `src/ugv_bringup/config/nav2_params.yaml` | `desired_linear_vel: 0.4`, `inflation_radius: 0.75`, `movement_time_allowance: 8.0`, `use_collision_detection: true` |
| **RTAB-Map Config** | `src/ugv_bringup/config/rtabmap.yaml` | `RGBD/LinearUpdate: 0.0`, `Reg/Force3DoF: true`, `Grid/RangeMax: 10.0`, `Grid/CellSize: 0.10` |

---

## Author & Maintenance

- **Author & Maintainer:** Prakash Kumar Sahu
- **Email:** [jaysahu0201@gmail.com](mailto:jaysahu0201@gmail.com)
- **GitHub Repository:** [PrakashKumarSahu/Autonomous-UGV](https://github.com/PrakashKumarSahu/Autonomous-UGV)
- **License:** Apache License 2.0
