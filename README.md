# Autonomous UGV — Vision-Based Outdoor Navigation

[![ROS 2 Jazzy](https://img.shields.io/badge/ROS_2-Jazzy-22314E?logo=ros)](https://docs.ros.org/en/jazzy/)
[![Gazebo Harmonic](https://img.shields.io/badge/Gazebo-Harmonic-FF6F00?logo=gazebo)](https://gazebosim.org/docs/harmonic/)
[![YOLO26](https://img.shields.io/badge/YOLO-26n--seg-00FFFF?logo=ultralytics)](https://github.com/ultralytics/ultralytics)
[![Depth Anything V3 TRT](https://img.shields.io/badge/Depth_Anything-V3_TensorRT-FF6F00?logo=nvidia)](https://github.com/ika-rwth-aachen/ros2-depth-anything-v3-trt)
[![License: Apache-2.0](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](LICENSE)

An end-to-end autonomous ground vehicle navigation stack for **GPS-denied outdoor environments** using **a single RGB camera as the primary sensor**. Built on **ROS 2 Jazzy** and **Gazebo Harmonic (gz-sim 8)**, the system features:

- **Perception AI:** Real-time instance segmentation with **YOLO26n-seg** (Ultralytics, NMS-free, FP16) and TensorRT-accelerated metric depth with **Depth Anything V3** (ika-rwth-aachen).
- **Visual SLAM:** 3-DoF planar SLAM with **RTAB-Map** using RGB-D odometry and loop-closure to build a persistent 2D occupancy grid and publish `map → odom` corrections.
- **2.5D Terrain Traversability:** A rolling, slope-tolerant height-deviation and step-analysis grid (`/terrain/traversability_grid`) with YOLO hazard back-projection.
- **Autonomous Navigation:** **Nav2** with Smac2D planner and Regulated Pure Pursuit controller, dynamic obstacle inflation, and recovery behaviors.
- **Full RViz2 Dashboard:** 17-display panel showing height-colored point clouds (AxisColor Z), live SLAM, costmaps, path execution, odometry trails, camera, and hazard overlays.

> **No LiDAR. No GPS. Pure vision-based autonomy from Point A to Point B.**

---

## Table of Contents

- [System Architecture](#system-architecture)
- [Package Structure](#package-structure)
- [Hardware & Environment Requirements](#hardware--environment-requirements)
- [Installation & Build](#installation--build)
- [Quick Start](#quick-start)
- [Depth Anything V3 TRT Setup (GPU Mode)](#depth-anything-v3-trt-setup-gpu-mode)
- [Launch Arguments](#launch-arguments)
- [Core Modules](#core-modules)
  - [1. ugv_sim — Simulation & Gazebo Bridge](#1-ugv_sim--simulation--gazebo-bridge)
  - [2. ugv_description — URDF & TF Tree](#2-ugv_description--urdf--tf-tree)
  - [3. ugv_perception — YOLO26 & Depth Pipeline](#3-ugv_perception--yolo26--depth-pipeline)
  - [4. ugv_terrain — Traversability Analysis](#4-ugv_terrain--traversability-analysis)
  - [5. ugv_bringup — Orchestration, SLAM & Nav2](#5-ugv_bringup--orchestration-slam--nav2)
  - [6. ugv_control — Hardware ros2_control Scaffold](#6-ugv_control--hardware-ros2_control-scaffold)
- [Depth Sources — Swappable Architecture](#depth-sources--swappable-architecture)
- [Topic and QoS Contracts](#topic-and-qos-contracts)
- [Configuration Reference](#configuration-reference)
- [Troubleshooting](#troubleshooting)
- [Author & Maintenance](#author--maintenance)

---

## System Architecture

```
                                  ┌──────────────────────────────────┐
                                  │   Gazebo Harmonic / Physical HW  │
                                  │  (Camera, IMU, Differential Drive)│
                                  └──────────────┬───────────────────┘
                                                 │ GZ↔ROS Bridge
              ┌──────────────────────────────────┼────────────────────────────────┐
              │                                  │                                │
              ▼                                  ▼                                ▼
  /camera/image_raw (bgr8)         /camera/depth/image_raw           /odometry/filtered
              │                      (32FC1 — sim mode)
              │                                  │
              │          ┌─────────────── Depth Source (camera_type) ───────────────┐
              │          │ sim       → depth_relay_node    (Gazebo depth sensor)    │
              │          │ da_v3_trt → depth_anything_v3   (TRT inference node)     │
              │          │                + da3_relay_node (canonical bridge)       │
              │          │ realsense → realsense_relay_node                         │
              │          │ zed       → zed_relay_node                               │
              │          └─────────────────────────┬────────────────────────────────┘
              │                                    │
              │             /perception/depth/image_raw  (32FC1, RELIABLE)
              │             /perception/depth/camera_info (RELIABLE)
              │             /perception/depth/colorized   (bgr8 TURBO, RViz)
              │                                    │
    ┌─────────┴──────────┐             ┌───────────┴────────────┐    ┌───────────────────┐
    ▼                    ▼             ▼                        ▼    ▼                   │
┌─────────────┐  ┌──────────────┐  ┌───────────────┐  ┌────────────────┐               │
│yolo_seg_node│  │ da3_relay_   │  │depth_to_point │  │rtabmap_sync /  │◄──────────────┘
│ (YOLO26n-  │  │ node (relay  │  │cloud_node     │  │rtabmap_slam    │
│  seg, FP16)│  │  colorized)  │  │(→/perc/depth  │  │(Visual SLAM,   │
└──────┬──────┘  └──────────────┘  │  /points)     │  │/map, map→odom) │
       │                           └───────┬────────┘  └──────┬─────────┘
       │ /perception/                      │                   │ /map
       │ hazard_mask                       │ PointCloud2       │ TF
       │ /perception/yolo/overlay          │                   │
       └─────────────┬─────────────────────┘                   │
                     ▼                              ┌───────────▼─────────────┐
           ┌──────────────────────┐                │       Nav2 Stack        │
           │  terrain_analysis_   │                │  SmacPlanner2D          │
           │       node           │                │  Regulated Pure Pursuit │
           │(height map, slope,   │                │  Costmaps + Recovery    │
           │ YOLO fusion)         │                └───────────┬─────────────┘
           └──────────┬───────────┘                            │ /cmd_vel → robot
                      │ /terrain/traversability_grid           │
                      └────────────────────────────────────────┘
                                  RViz2 Dashboard
```

---

## Package Structure

```
Autonomous-UGV/
├── src/
│   ├── ugv_bringup/          # Top-level orchestration: launch, Nav2, SLAM, RViz configs
│   │   ├── config/
│   │   │   ├── da3_ugv.param.yaml    # DA V3 TRT parameters for UGV
│   │   │   ├── ekf.yaml              # Robot_Localization EKF (hw mode)
│   │   │   ├── nav2_params.yaml      # Nav2 planner, controller, costmaps
│   │   │   └── rtabmap.yaml          # RTAB-Map visual SLAM config
│   │   └── launch/
│   │       ├── ugv_complete.launch.py  # ← MAIN ENTRY POINT
│   │       ├── perception.launch.py
│   │       ├── localization.launch.py
│   │       ├── navigation.launch.py
│   │       └── rviz.launch.py
│   ├── ugv_description/      # URDF xacro, robot model, TF tree
│   ├── ugv_perception/       # Perception nodes (YOLO26, depth relay, DA V3 bridge)
│   │   ├── config/
│   │   │   └── perception.yaml       # All perception node parameters
│   │   └── ugv_perception/
│   │       ├── da3_relay_node.py     # Bridges DA V3 TRT output → canonical /perception/depth/*
│   │       ├── depth_relay_node.py   # Sim: Gazebo depth → canonical topics
│   │       ├── realsense_relay_node.py
│   │       ├── zed_relay_node.py
│   │       ├── depth_to_pointcloud_node.py
│   │       └── yolo_seg_node.py      # YOLO26n-seg, FP16, configurable hazard classes
│   ├── ugv_sim/              # Gazebo world, robot model SDF, GZ↔ROS bridge
│   ├── ugv_terrain/          # 2.5D traversability analysis
│   └── ugv_control/          # ros2_control scaffold (hw mode future)
├── third_party/
│   ├── ros2-depth-anything-v3-trt/   # Submodule: ika-rwth-aachen DA V3 TRT C++ node
│   │   └── depth_anything_v3/        # ROS 2 package (builds only when TRT available)
│   └── yolo_ros/                     # COLCON_IGNORE'd (not used)
└── scripts/
    └── setup_da3_trt.sh     # TRT + ONNX model setup helper
```

---

## Hardware & Environment Requirements

| Component | Required | Notes |
|-----------|----------|-------|
| OS | Ubuntu 24.04 | Other distros untested |
| ROS 2 | Jazzy (desktop-full) | `apt install ros-jazzy-desktop-full` |
| Gazebo | Harmonic (gz-sim 8) | Installed via ROS repos |
| GPU | NVIDIA (any modern) | CUDA 12+ for YOLO26 FP16; TRT for DA V3 |
| CUDA | 12.x or 13.x | Via `torch`; full toolkit only needed for DA V3 TRT |
| RAM | ≥ 16 GB | SLAM + Nav2 + Gazebo + YOLO in parallel |
| Python | 3.10+ | System Python from ROS 2 Jazzy |

### Python Packages
```bash
pip install "numpy<2" --break-system-packages   # numpy 1.x — required by ROS Jazzy cv_bridge
pip install ultralytics                          # YOLO26n-seg
pip install torch torchvision --index-url https://download.pytorch.org/whl/cu124  # CUDA 12.4
# OR for CUDA 13: torch is auto-bundled with the ubuntu pip package
```

### ROS 2 Packages (apt)
```bash
sudo apt install -y \
  ros-jazzy-nav2-bringup ros-jazzy-nav2-smac-planner \
  ros-jazzy-nav2-regulated-pure-pursuit-controller \
  ros-jazzy-rtabmap-ros ros-jazzy-robot-localization \
  ros-jazzy-ros-gz-bridge ros-jazzy-ros-gz-sim \
  ros-jazzy-cv-bridge ros-jazzy-image-transport \
  ros-jazzy-image-transport-plugins \
  ros-jazzy-sensor-msgs-py \
  ros-jazzy-message-filters
```

---

## Installation & Build

```bash
# 1. Clone the repository with submodules
git clone --recurse-submodules https://github.com/PrakashKumarSahu/Autonomous-UGV.git
cd Autonomous-UGV

# 2. Install ROS 2 and Python dependencies (see above)

# 3. Build all packages
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install

# 4. Source the workspace
source install/setup.bash
```

> [!NOTE]
> The `depth_anything_v3` package (DA V3 TRT C++ node) will be found by colcon but its C++ compilation is **automatically skipped** when TensorRT/CUDA toolkit is not installed — you get a CMake WARNING but no build failure. All other packages build and run normally with `camera_type:=sim`.

---

## Quick Start

**Simulation with Gazebo depth sensor (default):**
```bash
source install/setup.bash
ros2 launch ugv_bringup ugv_complete.launch.py mode:=sim
```

**Simulation with Depth Anything V3 TRT (real GPU depth, requires TRT setup):**
```bash
# First time: set up TRT and download model (~15 min)
bash scripts/setup_da3_trt.sh

# Then launch
ros2 launch ugv_bringup ugv_complete.launch.py mode:=sim camera_type:=da_v3_trt
```

**Physical robot with RealSense:**
```bash
ros2 launch ugv_bringup ugv_complete.launch.py \
  mode:=hw camera_type:=realsense use_sim_time:=false
```

**Send a navigation goal** (after the system is running and SLAM has initialized):
```bash
ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped \
  '{header: {frame_id: map}, pose: {position: {x: 5.0, y: 5.0}, orientation: {w: 1.0}}}'
```
Or use the **Nav2 Goal** tool in RViz2 (2D Nav Goal button).

---

## Depth Anything V3 TRT Setup (GPU Mode)

Depth Anything V3 TRT replaces the camera_type:=sim Gazebo depth sensor with **real-time monocular metric depth estimation** using a TensorRT-accelerated model. This enables outdoor navigation **without a depth camera** (RGB only).

### Prerequisites
- NVIDIA GPU with CUDA 12+ drivers
- CUDA Toolkit (`nvcc` available)
- TensorRT SDK 10.x

### Setup (automated)
```bash
bash scripts/setup_da3_trt.sh
```

This script will:
1. Verify your GPU and CUDA installation
2. Install TensorRT (via pip or prompt for manual SDK download)
3. Download the DA V3 ONNX model (`DA3METRIC-SMALL.fp16.onnx`) from HuggingFace
4. Build the `depth_anything_v3` ROS 2 C++ package
5. Print launch instructions

### Manual Setup
```bash
# 1. Install CUDA Toolkit
sudo apt install cuda-toolkit-12-6   # match your driver version

# 2. Install TensorRT
pip install tensorrt --break-system-packages
# OR: download full SDK from https://developer.nvidia.com/tensorrt

# 3. Download ONNX model
mkdir -p third_party/ros2-depth-anything-v3-trt/depth_anything_v3/models/
wget -O third_party/ros2-depth-anything-v3-trt/depth_anything_v3/models/DA3METRIC-SMALL.fp16.onnx \
  https://huggingface.co/TillBeemelmanns/Depth-Anything-V3-ONNX/resolve/main/DA3METRIC-SMALL.fp16.onnx

# 4. Build the TRT package
colcon build --packages-select depth_anything_v3 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

> [!IMPORTANT]
> On the **first launch** with `camera_type:=da_v3_trt`, TensorRT compiles the ONNX model into an optimized engine file. This takes **2–5 minutes** and is cached. Subsequent launches load the engine directly.

### How it integrates
```
/camera/image_raw ──► depth_anything_v3 (C++ TRT node)
                           │ 32FC1 depth, FP16 inference, ~15 FPS (RTX 4050)
                           ▼
/depth_anything_v3/output/depth_image ──► da3_relay_node (Python bridge)
                                               │ TURBO colorize + camera_info forward
                                               ▼
                        /perception/depth/image_raw  (canonical, RELIABLE)
                        /perception/depth/camera_info (canonical, RELIABLE)
                        /perception/depth/colorized   (bgr8 TURBO, RViz)
                                               │
                                               ▼
                        depth_to_pointcloud_node → /perception/depth/points → Nav2
```

The `da3_ugv.param.yaml` in `ugv_bringup/config/` configures the TRT node for UGV use (model path, sky handling, depth range, etc.).

---

## Launch Arguments

| Argument | Default | Options | Description |
|----------|---------|---------|-------------|
| `mode` | `sim` | `sim`, `hw` | Simulation or physical robot |
| `camera_type` | `sim` | `sim`, `da_v3_trt`, `realsense`, `zed` | Depth source |
| `world` | `ugv_test_arena.sdf` | Any `.sdf` path | Gazebo world file |
| `world_name` | `ugv_test_arena` | String | SDF world name tag |
| `robot_name` | `localbot` | String | Robot entity name in Gazebo |
| `spawn_x/y/z` | `0.0 / 0.0 / 0.2` | Float | Robot spawn position |
| `spawn_yaw` | `1.5708` | Float (rad) | Robot initial heading |
| `enable_rtabmap` | `true` | `true/false` | Enable visual SLAM |
| `enable_rviz` | `true` | `true/false` | Launch RViz2 |
| `rviz_config` | `ugv_autonomy.rviz` | Path | RViz config file |
| `nav2_params_file` | `nav2_params.yaml` | Path | Nav2 parameter file |
| `use_sim_time` | `true` | `true/false` | Use Gazebo `/clock` |
| `perception_config` | `perception.yaml` | Path | Perception parameters YAML |

---

## Core Modules

### 1. ugv_sim — Simulation & Gazebo Bridge

**Package:** `ugv_sim` (ament_cmake)  
**World:** `ugv_test_arena.sdf` — a 500×500 m open outdoor terrain with:
- Undulating hills, boulders, trees (static single-link models)
- Waypoint markers, log obstacles, stream beds
- Directional sun lighting (no shadows — required to avoid Gazebo ogre2/depth crash)

**Key node:** `joint_state_relay.py` — bridges wheel joint states from Gazebo (BEST_EFFORT) to RTAB-Map (RELIABLE) on topic `/joint_states_urdf`.

**Bridge:** A dynamic YAML bridge config is generated at launch time by `sim.launch.py` and torn down cleanly via `atexit` when the launch is interrupted.

> [!CAUTION]
> Gazebo Harmonic + ogre2 renderer + shadows + depth_camera = **guaranteed crash**. All lights in `ugv_test_arena.sdf` have `cast_shadows: 0` and the scene has `<shadows>0</shadows>`. Never enable shadows.

### 2. ugv_description — URDF & TF Tree

**Package:** `ugv_description` (ament_cmake)  
**Robot:** `localbot` — a differential-drive UGV with:
- Chassis, 4 wheels (mecanum geometry)
- Forward-facing RGB-D camera (RGB at 640×480 30Hz, depth at 640×480 30Hz)
- IMU (Gazebo plugin)

**TF Tree:**
```
map
 └── odom          (RTAB-Map SLAM output)
      └── base_footprint   (EKF-fused odometry or Gazebo odom)
           └── base_link
                └── camera_link
                     └── camera_link_optical   (depth/image frame, Z-forward)
```

### 3. ugv_perception — YOLO26 & Depth Pipeline

**Package:** `ugv_perception` (ament_python)

#### Nodes

| Node | Executable | Purpose |
|------|-----------|---------|
| `depth_relay_node` | `depth_relay_node` | `camera_type:=sim` — Gazebo depth → canonical topics |
| `da3_relay_node` | `da3_relay_node` | `camera_type:=da_v3_trt` — DA V3 TRT output → canonical topics |
| `realsense_relay_node` | `realsense_relay_node` | `camera_type:=realsense` |
| `zed_relay_node` | `zed_relay_node` | `camera_type:=zed` |
| `depth_to_pointcloud_node` | `depth_to_pointcloud_node` | Depth image → PointCloud2 (feeds Nav2 + terrain) |
| `yolo_seg_node` | `yolo_seg_node` | YOLO26n-seg instance segmentation, FP16 |

#### YOLO26n-seg Details

- **Model:** `yolo26n-seg.pt` (auto-downloaded by Ultralytics on first run, ~10MB)
- **FP16:** `model.model.half()` — avoids deprecated `half=True` predict kwarg
- **Hazard classes:** Configurable via `lethal_classes` and `caution_classes` in `perception.yaml`
- **Output:** `/perception/hazard_mask` (mono8, cost values 0/128/255) and `/perception/yolo/overlay` (bgr8 annotated)

#### Depth Anything V3 TRT (da3_relay_node)

The `da3_relay_node` bridges the **ika-rwth-aachen** `depth_anything_v3` C++ TRT node to the canonical UGV depth contract:

- Subscribes: `/depth_anything_v3/output/depth_image` (32FC1, from TRT node)
- Subscribes: `/camera/camera_info` (for forwarding)
- Publishes: `/perception/depth/image_raw` (32FC1, RELIABLE)
- Publishes: `/perception/depth/camera_info` (RELIABLE)
- Publishes: `/perception/depth/colorized` (bgr8 TURBO, for RViz)

**DA V3 TRT depth postprocessing pipeline** (in the C++ node):
1. TensorRT inference on the ONNX model → raw depth tensor
2. Focal-length scaling: `depth *= (fx + fy) / 2 / 300.0` (aligns to actual camera intrinsics)
3. Sky pixel detection (separate model output) → fills with `sky_depth_cap` value
4. Bilinear upscale to original image resolution
5. Publishes 32FC1 depth + optional PointCloud2

**Configuration:** `ugv_bringup/config/da3_ugv.param.yaml`

#### Canonical Depth Topic Contract

Every depth source publishes **identical** topics; downstream nodes subscribe to these, never to source-specific topics:

| Topic | Encoding | QoS | Frame | Description |
|-------|----------|-----|-------|-------------|
| `/perception/depth/image_raw` | `32FC1` (metres) | RELIABLE | `camera_link_optical` | Metric depth map |
| `/perception/depth/camera_info` | CameraInfo | RELIABLE | `camera_link_optical` | Camera intrinsics |
| `/perception/depth/colorized` | `bgr8` (TURBO) | RELIABLE | `camera_link_optical` | RViz visualization |
| `/perception/depth/points` | PointCloud2 | RELIABLE | `camera_link_optical` | Nav2 obstacle cloud |

### 4. ugv_terrain — Traversability Analysis

**Package:** `ugv_terrain` (ament_python)

`terrain_analysis_node.py` builds a rolling 2.5D traversability map:
- Subscribes to `/perception/depth/points` (PointCloud2)
- Uses TF2 to transform points from `camera_link_optical` to the world frame
- Computes height deviation and slope on a 2D grid
- Back-projects YOLO hazard mask detections onto the grid
- Publishes `/terrain/traversability_grid` (nav_msgs/OccupancyGrid)

**Configuration:** `ugv_terrain/config/terrain.yaml`

### 5. ugv_bringup — Orchestration, SLAM & Nav2

**Package:** `ugv_bringup` (ament_cmake)

#### Launch files

| File | Purpose |
|------|---------|
| `ugv_complete.launch.py` | **Master launch** — starts all 6 modules in order |
| `perception.launch.py` | Depth source selection + YOLO26 + point cloud |
| `localization.launch.py` | RTAB-Map visual SLAM + optional EKF |
| `navigation.launch.py` | Nav2 lifecycle manager + all Nav2 nodes |
| `rviz.launch.py` | RViz2 with `ugv_autonomy.rviz` |

#### Key configuration files

| File | Purpose |
|------|---------|
| `config/nav2_params.yaml` | Nav2: Smac2D, RPP, costmaps, inflation 0.75m |
| `config/rtabmap.yaml` | RTAB-Map: RGB-D sync, SLAM params |
| `config/ekf.yaml` | EKF: wheel odom + IMU (hw mode) |
| `config/da3_ugv.param.yaml` | DA V3 TRT: model, sky, depth range |

#### Nav2 Key Design Values

| Parameter | Value | Rationale |
|-----------|-------|-----------|
| `inflation_radius` | 0.75m | Robot radius (0.30m) + 0.45m safety margin |
| `desired_linear_vel` | 0.4 m/s | Slower = more reaction time for visual obstacles |
| `movement_time_allowance` | 8.0s | Fast stuck detection; robot must move 0.30m in 8s |
| `transform_tolerance` | 1.0s | Matches 1Hz RTAB-Map TF publish rate |
| `use_collision_detection` | true | RPP stops and replans before hitting mapped obstacles |
| `stamp_offset` (cloud) | 0.15s | Cloud stamped 150ms in past — ensures TF interpolation |

### 6. ugv_control — Hardware ros2_control Scaffold

**Package:** `ugv_control` (ament_cmake)

Stub for future hardware integration. Contains a `diff_drive_controller.yaml` configuration. **Never launched in sim mode.** For physical robot deployment, this package provides the `ros2_control` hardware interface for direct motor commanding.

---

## Depth Sources — Swappable Architecture

All depth sources implement the **same output contract** (`/perception/depth/*`). Switching depth sources requires only changing `camera_type`:

```bash
# Default: Gazebo simulated depth sensor
ros2 launch ugv_bringup ugv_complete.launch.py camera_type:=sim

# Real GPU monocular depth (Depth Anything V3 TensorRT)
ros2 launch ugv_bringup ugv_complete.launch.py camera_type:=da_v3_trt

# Intel RealSense D435/D455 (physical robot)
ros2 launch ugv_bringup ugv_complete.launch.py camera_type:=realsense mode:=hw use_sim_time:=false

# Stereolabs ZED 2 / ZED X (physical robot, outdoor)
ros2 launch ugv_bringup ugv_complete.launch.py camera_type:=zed mode:=hw use_sim_time:=false
```

> [!TIP]
> When running `camera_type:=da_v3_trt` in sim mode, the UGV camera feed (`/camera/image_raw`) is from Gazebo, but depth is computed by the DA V3 TensorRT model — this is useful for **testing the full AI perception pipeline in simulation** without a real depth sensor.

---

## Topic and QoS Contracts

### Perception Topics

| Topic | Type | QoS | Producer | Consumer |
|-------|------|-----|----------|----------|
| `/camera/image_raw` | Image (bgr8) | BE | Gazebo bridge / camera driver | YOLO, DA V3, RTAB-Map |
| `/camera/camera_info` | CameraInfo | BE | Gazebo bridge | depth nodes, RTAB-Map |
| `/perception/depth/image_raw` | Image (32FC1) | RELIABLE | Depth relay node | depth_to_pointcloud, RTAB-Map |
| `/perception/depth/camera_info` | CameraInfo | RELIABLE | Depth relay node | depth_to_pointcloud, RTAB-Map |
| `/perception/depth/colorized` | Image (bgr8) | RELIABLE | Depth relay node | RViz |
| `/perception/depth/points` | PointCloud2 | RELIABLE | depth_to_pointcloud_node | Nav2 costmap, terrain |
| `/perception/hazard_mask` | Image (mono8) | RELIABLE | yolo_seg_node | terrain_analysis |
| `/perception/yolo/overlay` | Image (bgr8) | RELIABLE | yolo_seg_node | RViz |
| `/depth_anything_v3/output/depth_image` | Image (32FC1) | RELIABLE | depth_anything_v3 C++ node | da3_relay_node |

### Navigation & Localization Topics

| Topic | Type | Producer | Consumer |
|-------|------|----------|----------|
| `/map` | OccupancyGrid | RTAB-Map | Nav2 global costmap, RViz |
| `/odometry/filtered` | Odometry | EKF / RTAB-Map | Nav2, RViz |
| `/tf` | TFMessage | RTAB-Map, joint_state_relay | All nodes |
| `/cmd_vel` | Twist | Nav2 RPP controller | Gazebo bridge / motor driver |
| `/goal_pose` | PoseStamped | RViz Nav2Goal / user | Nav2 BT Navigator |

### YOLO Hazard Mask Values

| Value | Meaning |
|-------|---------|
| `0` | Traversable (no detection) |
| `128` | Caution (soft obstacle from `caution_classes`) |
| `255` | Lethal hazard (from `lethal_classes`) |

---

## Configuration Reference

### perception.yaml (`src/ugv_perception/config/`)

Loaded by `perception.launch.py`. Top-level keys are node names. Key sections:

```yaml
da3_relay_node:               # camera_type:=da_v3_trt bridge node
  ros__parameters:
    da3_depth_topic: /depth_anything_v3/output/depth_image
    colorize_max_depth: 15.0  # metres — outdoor scenes are deeper than 10m

depth_to_pointcloud_node:
  ros__parameters:
    step: 4                   # ~19k points at 640×480
    max_depth: 15.0           # raised for outdoor DA V3 TRT use
    stamp_offset: 0.15        # 150ms past — ensures TF interpolation

yolo_seg_node:
  ros__parameters:
    model_name: yolo26n-seg.pt   # auto-downloaded by Ultralytics
    enable_fp16: true
```

### da3_ugv.param.yaml (`src/ugv_bringup/config/`)

Parameters for the `depth_anything_v3` C++ TRT node when `camera_type:=da_v3_trt`:

```yaml
depth_anything_v3:
  ros__parameters:
    onnx_path: "models/DA3METRIC-SMALL.fp16.onnx"  # relative to package share
    precision: "fp16"
    sky_threshold: 0.3      # sky pixel detection aggressiveness
    sky_depth_cap: 20.0     # metres — sky pixels get this depth
    point_cloud_downsample_factor: 100  # disable internal cloud; use depth_to_pointcloud
```

### nav2_params.yaml (`src/ugv_bringup/config/`)

Full Nav2 configuration. Notable tuning:
- `inflation_radius: 0.75` — wide clearance for camera-based obstacle detection
- `transform_tolerance: 1.0` — accommodates 1Hz RTAB-Map TF updates
- `use_collision_detection: true` — RPP stops before hitting mapped obstacles

---

## Troubleshooting

### Build fails with "TensorRT not found"
Normal if TRT is not installed. The `depth_anything_v3` package skips C++ compilation and prints a CMake WARNING (not an error). All other packages build fine. Run `scripts/setup_da3_trt.sh` to install TRT when ready.

### Gazebo crashes immediately
Ensure shadows are disabled. Check `ugv_test_arena.sdf`: all lights must have `cast_shadows: 0` and scene must have `<shadows>0</shadows>`.

### `/perception/depth/points` not visible in RViz
- Verify `depth_to_pointcloud_node` is running: `ros2 node list | grep depth_to`
- Check fixed frame is `map` or `odom` in RViz
- Confirm TF exists: `ros2 run tf2_ros tf2_echo map camera_link_optical`

### "CameraCloud status: ERROR" flashing in terminal
This is the `depth_to_pointcloud_node` logging a TF timeout when the `map → odom → base_footprint → camera_link_optical` chain is not yet complete (SLAM hasn't initialized). Normal during the first 5–10 seconds. Clears once RTAB-Map publishes its first map frame.

### YOLO model download timeout
```bash
python3 -c "from ultralytics import YOLO; YOLO('yolo26n-seg.pt')"
```
Downloads on first run (~10MB). Check internet access.

### Navigation goal rejected immediately
RTAB-Map needs to have published at least one `/map` frame. Wait for the terminal to show `[rtabmap] map updated` before sending goals.

### DA V3 TRT: Engine compilation stuck
On first launch, TRT engine compilation is CPU+GPU-intensive. Normal to take 2–5 minutes. Do NOT interrupt. The `.engine` file is cached in the models directory after compilation.

---

## Author & Maintenance

| Field | Value |
|-------|-------|
| Author | Prakash Kumar Sahu |
| Email | jaysahu0201@gmail.com |
| GitHub | [PrakashKumarSahu/Autonomous-UGV](https://github.com/PrakashKumarSahu/Autonomous-UGV) |
| ROS 2 | Jazzy |
| Gazebo | Harmonic (gz-sim 8) |

### Third-party components

| Component | License | Source |
|-----------|---------|--------|
| YOLO26n-seg | AGPL-3.0 | [Ultralytics](https://github.com/ultralytics/ultralytics) |
| Depth Anything V3 TRT | Apache-2.0 | [ika-rwth-aachen/ros2-depth-anything-v3-trt](https://github.com/ika-rwth-aachen/ros2-depth-anything-v3-trt) |
| RTAB-Map | BSD-3 | [introlab/rtabmap](https://github.com/introlab/rtabmap) |
| Nav2 | Apache-2.0 | [ros-navigation/navigation2](https://github.com/ros-navigation/navigation2) |
