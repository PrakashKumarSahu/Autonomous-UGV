# Autonomous UGV — ROS 2 Jazzy + Gazebo Harmonic

An autonomous ground vehicle stack built on **ROS 2 Jazzy** and **Gazebo Harmonic**. It combines real-time instance segmentation (YOLO26n-seg), depth estimation (Gazebo sensor, Depth Anything V3 TRT, RealSense, or ZED), 2.5D terrain traversability mapping, visual SLAM (RTAB-Map), and autonomous navigation (Nav2 with SmacPlanner2D + Regulated Pure Pursuit).

---

## Table of Contents

1. [System Overview](#1-system-overview)
2. [Architecture](#2-architecture)
3. [Package Structure](#3-package-structure)
4. [Prerequisites](#4-prerequisites)
5. [Installation and Build](#5-installation-and-build)
6. [Running the System](#6-running-the-system)
7. [Depth Source Modes](#7-depth-source-modes)
8. [Depth Anything V3 TRT — Docker Setup](#8-depth-anything-v3-trt--docker-setup)
9. [Configuration Reference](#9-configuration-reference)
10. [Topic Contract](#10-topic-contract)
11. [TF Tree](#11-tf-tree)
12. [RViz Dashboard](#12-rviz-dashboard)
13. [Hardware Mode](#13-hardware-mode)
14. [Modifying the System](#14-modifying-the-system)
15. [Troubleshooting](#15-troubleshooting)

---

## 1. System Overview

The UGV is a differential-drive wheeled robot ("Localbot") that:

- **Perceives** its surroundings via a front-mounted RGB-D camera and YOLO26n-seg instance segmentation
- **Maps** terrain traversability using a 2.5D elevation + semantic occupancy grid
- **Localizes** via visual SLAM (RTAB-Map) building a 2D occupancy map in real time
- **Navigates** autonomously to goal poses using Nav2 (SmacPlanner2D + Regulated Pure Pursuit)
- **Simulates** in Gazebo Harmonic with a custom obstacle-course world (`ugv_test_arena.sdf`)

### Robot Specifications (Localbot)

| Property | Value |
|---|---|
| Chassis | 0.60 × 0.40 × 0.20 m |
| Drive | Differential drive, 2 × 0.10 m wheels (0.45 m separation) |
| Camera | RGB-D (640×480, 30 Hz, 80° FOV, 0.3–10 m, `camera_link_optical` frame) |
| IMU | 50 Hz, Gaussian noise σ=0.005 rad/s |
| Max speed | 1.5 m/s linear, 2.0 rad/s angular |

---

## 2. Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│  ugv_complete.launch.py — Single entry point                        │
│                                                                      │
│  ┌─────────────┐  ┌──────────────────────────────────────────────┐  │
│  │  ugv_sim    │  │  ugv_perception                              │  │
│  │  (sim mode) │  │                                              │  │
│  │  Gazebo     │  │  [ONE depth source, camera_type arg]         │  │
│  │  Harmonic   │  │  sim        → depth_relay_node               │  │
│  │  localbot   │  │  da_v3_trt  → da3_relay_node (+ Docker)     │  │
│  │  model.sdf  │  │  realsense  → realsense_relay_node           │  │
│  │  world SDF  │  │  zed        → zed_relay_node                 │  │
│  │             │  │                                              │  │
│  │  ros_gz     │  │  [Always on]                                 │  │
│  │  bridge     │  │  depth_to_pointcloud_node                    │  │
│  │             │  │  yolo_seg_node (YOLO26n-seg)                 │  │
│  └──────┬──────┘  └───────────────────────┬──────────────────────┘  │
│         │                                  │                          │
│  /camera/image_raw                 /perception/depth/*               │
│  /camera/depth/*                   /perception/depth/points          │
│  /odometry/filtered                /perception/hazard_mask           │
│  /joint_states → relay             /perception/yolo/overlay          │
│  /tf  /clock                                │                         │
│         │                                  │                          │
│  ┌──────▼──────────────────────────────────▼──────────────────────┐  │
│  │  ugv_terrain                                                   │  │
│  │  terrain_analysis_node                                         │  │
│  │  → /terrain/traversability_grid (OccupancyGrid, base_frame)   │  │
│  └────────────────────────────────────────────────────────────────┘  │
│                                                                      │
│  ┌─────────────────────────────────────────────────────────────────┐ │
│  │  ugv_bringup — localization.launch.py                          │ │
│  │  rgbd_sync      /camera/image_raw + /perception/depth/*        │ │
│  │                 → /rtabmap/rgbd_image                          │ │
│  │  rtabmap         RGBDImage + /odometry/filtered                │ │
│  │                 → /map (OccupancyGrid) + map→odom TF           │ │
│  └─────────────────────────────────────────────────────────────────┘ │
│                                                                      │
│  ┌─────────────────────────────────────────────────────────────────┐ │
│  │  ugv_bringup — navigation.launch.py (Nav2)                     │ │
│  │  controller_server   (Regulated Pure Pursuit, 20 Hz)           │ │
│  │  planner_server      (SmacPlanner2D)                           │ │
│  │  behavior_server     (Spin / BackUp / Wait)                    │ │
│  │  bt_navigator        (NavigateToPose / NavigateThroughPoses)   │ │
│  │  lifecycle_manager                                             │ │
│  │  Local costmap:  /perception/depth/points → ObstacleLayer      │ │
│  │  Global costmap: /map → StaticLayer                            │ │
│  └─────────────────────────────────────────────────────────────────┘ │
│                                                                      │
│  ┌──────────────────────────┐                                        │
│  │  RViz2 (optional)        │                                        │
│  │  ugv_autonomy.rviz       │                                        │
│  └──────────────────────────┘                                        │
└─────────────────────────────────────────────────────────────────────┘
```

---

## 3. Package Structure

```
Autonomous-UGV/
├── src/
│   ├── ugv_bringup/          # Top-level launch + all configuration files
│   │   ├── launch/
│   │   │   ├── ugv_complete.launch.py   # ← Single system entry point
│   │   │   ├── perception.launch.py     # Depth source + YOLO + point cloud
│   │   │   ├── localization.launch.py   # RTAB-Map SLAM + EKF (hw)
│   │   │   ├── navigation.launch.py     # Nav2 stack
│   │   │   └── rviz.launch.py           # RViz2 dashboard
│   │   ├── config/
│   │   │   ├── nav2_params.yaml         # Nav2: RPP, SmacPlanner, costmaps
│   │   │   ├── rtabmap.yaml             # RTAB-Map SLAM parameters
│   │   │   ├── ekf.yaml                 # EKF sensor fusion (hw mode only)
│   │   │   └── da3_ugv.param.yaml       # DA V3 TRT node parameters
│   │   └── rviz/
│   │       └── ugv_autonomy.rviz        # RViz panel layout
│   │
│   ├── ugv_description/      # Robot URDF model (for robot_state_publisher / TF only)
│   │   ├── urdf/
│   │   │   ├── ugv.urdf.xacro           # Root: includes base + sensors
│   │   │   ├── ugv_base.urdf.xacro      # Chassis, wheels, casters (joints + kinematics)
│   │   │   └── sensors/
│   │   │       ├── camera.urdf.xacro    # camera_link + camera_link_optical frames
│   │   │       └── imu.urdf.xacro       # imu_link frame
│   │   └── launch/
│   │       └── robot_state_publisher.launch.py
│   │
│   ├── ugv_perception/       # AI perception nodes (Python)
│   │   ├── ugv_perception/
│   │   │   ├── depth_relay_node.py      # camera_type:=sim — Gazebo depth bridge
│   │   │   ├── da3_relay_node.py        # camera_type:=da_v3_trt — DA V3 TRT bridge
│   │   │   ├── realsense_relay_node.py  # camera_type:=realsense — RealSense bridge
│   │   │   ├── zed_relay_node.py        # camera_type:=zed — ZED bridge
│   │   │   ├── depth_to_pointcloud_node.py  # 32FC1 depth → PointCloud2
│   │   │   └── yolo_seg_node.py         # YOLO26n-seg hazard instance segmentation
│   │   └── config/
│   │       └── perception.yaml          # Parameters for all perception nodes
│   │
│   ├── ugv_terrain/          # Terrain traversability mapping (Python)
│   │   ├── ugv_terrain/
│   │   │   └── terrain_analysis_node.py # 2.5D OccupancyGrid from point cloud + YOLO
│   │   ├── config/
│   │   │   └── terrain.yaml
│   │   └── launch/
│   │       └── terrain_mapping.launch.py
│   │
│   ├── ugv_sim/              # Gazebo simulation (C++ package, install-only)
│   │   ├── launch/
│   │   │   └── sim.launch.py            # Gazebo + RSP + bridge + spawn
│   │   ├── models/
│   │   │   └── localbot/
│   │   │       ├── model.config
│   │   │       └── model.sdf            # ← Actual robot loaded by Gazebo
│   │   ├── worlds/
│   │   │   ├── ugv_test_arena.sdf       # Default: obstacle course
│   │   │   └── empty_world.sdf          # Empty flat ground
│   │   ├── scripts/
│   │   │   └── joint_state_relay.py     # QoS bridge: BestEffort → Reliable
│   │   └── config/
│   │       └── gazebo_bridge.yaml       # Reference (bridge is generated dynamically)
│   │
│   └── ugv_control/          # ros2_control scaffolding (NOT active in default sim mode)
│       ├── config/
│       │   └── diff_drive_controller.yaml   # For hw mode with ros2_control
│       └── launch/
│           └── control.launch.py            # For hw mode with ros2_control
│
├── docker/                   # Docker-based DA V3 TRT integration
│   ├── da3_trt.compose.yml   # Main compose file for DA V3 TRT container
│   ├── da3_docker.param.yaml # DA V3 TRT parameters (mounted into container)
│   ├── .env                  # Default env vars (ROS_DOMAIN_ID, image tag)
│   └── README.md             # Docker setup guide
│
├── third_party/
│   ├── ros2-depth-anything-v3-trt/   # DA V3 TRT C++ ROS 2 package (submodule)
│   │   └── depth_anything_v3/        # Colcon-buildable package
│   └── yolo_ros/                     # (reference, not used by perception nodes)
│
└── scripts/
    ├── setup_da3_docker.sh   # One-time DA V3 Docker setup
    └── setup_da3_trt.sh      # Native host TRT build (advanced)
```

> **Note on `ugv_control`:** The default simulation uses Gazebo's built-in DiffDrive plugin (in `localbot/model.sdf`), not `ros2_control`. `ugv_control` is scaffolding for future hardware mode with `ros2_control`-based motor drivers. It is not launched by `ugv_complete.launch.py`.

---

## 4. Prerequisites

### System Requirements

| Requirement | Version |
|---|---|
| OS | Ubuntu 24.04 LTS |
| ROS 2 | Jazzy Jalisco |
| Gazebo | Harmonic (via `ros-jazzy-ros-gz-*`) |
| Python | 3.12 |
| GPU | NVIDIA (CUDA 12+) — required for YOLO26 GPU inference |

### ROS 2 Packages

```bash
sudo apt install -y \
  ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge ros-jazzy-ros-gz-image \
  ros-jazzy-nav2-bringup ros-jazzy-nav2-smac-planner \
  ros-jazzy-nav2-regulated-pure-pursuit-controller \
  ros-jazzy-rtabmap-ros \
  ros-jazzy-robot-state-publisher ros-jazzy-joint-state-publisher \
  ros-jazzy-robot-localization \
  ros-jazzy-tf2-ros ros-jazzy-tf2-tools \
  ros-jazzy-sensor-msgs-py \
  ros-jazzy-cv-bridge ros-jazzy-image-transport
```

### Python Dependencies

```bash
# Core
pip install "numpy==1.26.4"   # MUST be 1.x — ROS Jazzy cv_bridge compiled against numpy 1.x

# YOLO26n-seg
pip install ultralytics torch torchvision

# Simulation
pip install pyyaml
```

> **Important:** Do not upgrade numpy to 2.x. `cv_bridge` will crash with: `TypeError: bytes expected, not memoryview`.

---

## 5. Installation and Build

```bash
# Clone with submodules
git clone --recurse-submodules https://github.com/PrakashKumarSahu/Autonomous-UGV.git
cd Autonomous-UGV

# Source ROS 2
source /opt/ros/jazzy/setup.bash

# Build
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

> **First YOLO run:** `yolo26n-seg.pt` (~6.4 MB) is downloaded automatically by Ultralytics on first launch. Subsequent runs use the cache. Pre-download with: `python3 -c "from ultralytics import YOLO; YOLO('yolo26n-seg.pt')"`

> **Third-party depth package:** `third_party/ros2-depth-anything-v3-trt/` contains the DA V3 TRT C++ node. It is colcon-discoverable but skips gracefully without TensorRT installed. To use it, see [Section 8](#8-depth-anything-v3-trt--docker-setup).

---

## 6. Running the System

### Simulation Mode (default)

```bash
source install/setup.bash

# Full system — Gazebo + all perception + SLAM + Nav2 + RViz
ros2 launch ugv_bringup ugv_complete.launch.py

# With a custom world
ros2 launch ugv_bringup ugv_complete.launch.py \
  world:=/path/to/my_world.sdf \
  world_name:=my_world \
  spawn_x:=0.0 spawn_y:=0.0 spawn_yaw:=0.0
```

### Launch Arguments

| Argument | Default | Description |
|---|---|---|
| `mode` | `sim` | `sim` (Gazebo) or `hw` (real robot) |
| `use_sim_time` | `true` | Use Gazebo `/clock` for all nodes |
| `camera_type` | `sim` | Depth source: `sim` / `da_v3_trt` / `realsense` / `zed` |
| `world` | `ugv_test_arena.sdf` | Path to Gazebo SDF world file |
| `world_name` | `world_demo` | World name attribute in SDF (must match `<world name="...">`) |
| `robot_name` | `localbot` | Robot entity name in Gazebo |
| `spawn_x/y/z` | `0.0 / -6.5 / 0.01` | Robot spawn position (metres) |
| `spawn_yaw` | `1.5708` | Robot spawn yaw (radians): 1.5708=north, 0=east |
| `enable_rtabmap` | `true` | Enable RTAB-Map SLAM |
| `enable_rviz` | `true` | Launch RViz2 dashboard |
| `nav2_params_file` | `config/nav2_params.yaml` | Override Nav2 config without rebuilding |

### Send a Navigation Goal

```bash
# After launch: wait for "Lifecycle manager navigation is active" in terminal, then:
# Option 1 — RViz2: click "2D Nav Goal" button and click on the map
# Option 2 — CLI:
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 3.0, y: 2.0, z: 0.0}, orientation: {w: 1.0}}}}"
```

---

## 7. Depth Source Modes

All four depth sources produce an identical output contract. Only **one** source runs at a time, selected by the `camera_type` launch argument:

### Canonical Output Topics

| Topic | Type | QoS | Description |
|---|---|---|---|
| `/perception/depth/image_raw` | `sensor_msgs/Image` (32FC1) | RELIABLE | Metric depth in metres |
| `/perception/depth/camera_info` | `sensor_msgs/CameraInfo` | RELIABLE | Camera intrinsics |
| `/perception/depth/colorized` | `sensor_msgs/Image` (bgr8) | RELIABLE | TURBO colormap (RViz) |
| `/perception/depth/points` | `sensor_msgs/PointCloud2` | RELIABLE | 3D point cloud → Nav2 |

### Source Selection

| `camera_type` | Node | Input | Use Case |
|---|---|---|---|
| `sim` | `depth_relay_node` | `/camera/depth/image_raw` (Gazebo 32FC1) | Simulation |
| `da_v3_trt` | `da3_relay_node` | `/depth_anything_v3/output/depth_image` (Docker container) | Monocular camera, outdoor |
| `realsense` | `realsense_relay_node` | `/camera/camera/depth/image_rect_raw` (16UC1 mm) | Intel RealSense D435/D455 |
| `zed` | `zed_relay_node` | `/zed/zed_node/depth/depth_registered` (32FC1 m) | ZED 2 / ZED X |

### Always-On Nodes (all modes)

| Node | Input → Output | Purpose |
|---|---|---|
| `depth_to_pointcloud_node` | `/perception/depth/image_raw` → `/perception/depth/points` | Feeds Nav2 ObstacleLayer |
| `yolo_seg_node` | `/camera/image_raw` → `/perception/hazard_mask` + `/perception/yolo/overlay` | Instance segmentation |

---

## 8. Depth Anything V3 TRT — Docker Setup

Depth Anything V3 TRT requires **TensorRT + CUDA** to build. The recommended approach uses a prebuilt Docker image that includes all dependencies, so no host-side TRT installation is needed.

### Prerequisites

```bash
# 1. Add user to docker group (one-time)
sudo usermod -aG docker $USER
newgrp docker   # or log out/in

# 2. Install NVIDIA Container Toolkit (one-time)
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
  sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
  sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
sudo apt update && sudo apt install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

### One-Time Setup

```bash
# Downloads image, ONNX model, builds the ROS 2 package inside container
scripts/setup_da3_docker.sh
```

The script:
1. Validates GPU access and Docker permissions
2. Pulls `tillbeemelmanns/ros2-depth-anything-v3:latest-dev` (~17 GB, includes TRT 10 + CUDA 13 + ROS 2 Jazzy)
3. Downloads `DA3METRIC-LARGE.onnx` (~700 MB) from HuggingFace
4. Builds `depth_anything_v3` inside the container using `colcon` → cached in Docker volume `ugv_da3_build`
5. Verifies the `depth_anything_v3_main` binary exists in the volume

### Running with DA V3 TRT

```bash
# Terminal 1 — DA V3 TRT container (GPU inference)
docker compose -f docker/da3_trt.compose.yml up

# Terminal 2 — UGV stack (bridged via da3_relay_node)
ros2 launch ugv_bringup ugv_complete.launch.py camera_type:=da_v3_trt
```

> **First launch:** TRT compiles the ONNX model to a `.engine` file (~3–8 min on RTX 4050). The engine is cached next to the ONNX file. Subsequent launches start in <10 s.

### Docker Configuration Files

| File | Purpose |
|---|---|
| [`docker/da3_trt.compose.yml`](docker/da3_trt.compose.yml) | Service: nvidia runtime, `--network host`, build volume, model mount, launch command |
| [`docker/da3_docker.param.yaml`](docker/da3_docker.param.yaml) | DA V3 TRT node params mounted read-only into container |
| [`docker/.env`](docker/.env) | Defaults: `ROS_DOMAIN_ID=0`, `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`, image tag |
| [`docker/README.md`](docker/README.md) | Extended Docker architecture docs |

### How It Works

```
┌─────────────────────────────────┐    --network host    ┌───────────────────────────────┐
│  HOST (ROS 2 Jazzy)             │◄────────────────────►│  DOCKER CONTAINER             │
│                                 │                       │  tillbeemelmanns/ros2-da3-v3  │
│  da3_relay_node                 │                       │  (TRT 10.13 + CUDA 13)        │
│    ← /depth_anything_v3/output/ │◄──── DDS FastDDS ────│                               │
│         depth_image (32FC1)     │                       │  depth_anything_v3_main       │
│    → /perception/depth/*        │                       │    ← /camera/image_raw        │
│         (RELIABLE QoS)          │                       │    ← /camera/camera_info      │
│                                 │                       │    → /depth_anything_v3/      │
│  depth_to_pointcloud_node       │                       │         output/depth_image    │
│    → /perception/depth/points   │                       │                               │
└─────────────────────────────────┘                       └───────────────────────────────┘
```

---

## 9. Configuration Reference

### Key Configuration Files

| File | Controls |
|---|---|
| `src/ugv_bringup/config/nav2_params.yaml` | Nav2: speed, lookahead, inflation radius, costmap layers, progress checker |
| `src/ugv_bringup/config/rtabmap.yaml` | SLAM: keyframe thresholds, 2D grid map, visual features |
| `src/ugv_bringup/config/ekf.yaml` | EKF sensor fusion (hw mode only) |
| `src/ugv_bringup/config/da3_ugv.param.yaml` | DA V3 TRT: model path, sky cap, precision |
| `src/ugv_perception/config/perception.yaml` | All perception node parameters (topics, thresholds, YOLO classes) |
| `src/ugv_terrain/config/terrain.yaml` | Terrain grid: resolution, step/slope thresholds |
| `src/ugv_sim/models/localbot/model.sdf` | Robot physics, sensors, DiffDrive plugin |
| `src/ugv_sim/worlds/ugv_test_arena.sdf` | Default simulation world (obstacles, terrain) |

### Important Parameter Decisions

#### Nav2 (`nav2_params.yaml`)

| Parameter | Value | Rationale |
|---|---|---|
| `desired_linear_vel` | 0.4 m/s | More reaction time vs. detected obstacles |
| `inflation_radius` | 0.75 m | Robot (r=0.32 m) + 0.43 m safety margin |
| `use_collision_detection` | `true` | RPP pre-checks path against costmap |
| `movement_time_allowance` | 8.0 s | Detect stuck state in 8 s |
| `observation_persistence` | 0.5 s | Stale obstacles clear quickly on turns |
| `transform_tolerance` | 1.0 s | Matches RTAB-Map ~1 Hz TF publish rate |

#### RTAB-Map (`rtabmap.yaml`)

| Parameter | Value | Rationale |
|---|---|---|
| `RGBD/LinearUpdate` | 0.0 | First keyframe on first frame (no movement needed) |
| `RGBD/AngularUpdate` | 0.0 | Idem |
| `map_always_update` | `true` | Republish `/map` every iteration, not just on loop closure |
| `Reg/Force3DoF` | `true` | 2D SLAM (ground UGV, no 3D needed) |

#### YOLO26n-seg (`perception.yaml`)

| Parameter | Value | Description |
|---|---|---|
| `model_name` | `yolo26n-seg.pt` | Auto-downloaded on first run |
| `confidence_threshold` | 0.35 | Minimum detection confidence |
| `device` | `cuda:0` | GPU inference (falls back to CPU if unavailable) |
| `enable_fp16` | `true` | Half-precision for speed |
| `lethal_classes` | `[person, car, truck, …]` | → mask value 255 (red overlay, obstacle) |
| `caution_classes` | `[vegetation, bush, …]` | → mask value 128 (amber overlay) |

---

## 10. Topic Contract

### Core Topics

| Topic | Type | Publisher | Consumers |
|---|---|---|---|
| `/camera/image_raw` | Image (bgr8, BE) | Gazebo bridge / camera driver | YOLO, RTAB-Map rgbd_sync, DA V3 TRT |
| `/camera/camera_info` | CameraInfo (BE) | Gazebo bridge / camera driver | RTAB-Map rgbd_sync, DA V3 TRT relay |
| `/camera/depth/image_raw` | Image (32FC1, BE) | Gazebo bridge (sim mode only) | depth_relay_node |
| `/perception/depth/image_raw` | Image (32FC1, RL) | Depth source relay | RTAB-Map rgbd_sync, depth_to_pointcloud |
| `/perception/depth/camera_info` | CameraInfo (RL) | Depth source relay | RTAB-Map rgbd_sync, depth_to_pointcloud |
| `/perception/depth/colorized` | Image (bgr8, RL) | Depth source relay | RViz2 |
| `/perception/depth/points` | PointCloud2 (RL) | depth_to_pointcloud_node | Nav2 ObstacleLayer |
| `/perception/hazard_mask` | Image (mono8, RL) | yolo_seg_node | terrain_analysis_node |
| `/perception/yolo/overlay` | Image (bgr8, RL) | yolo_seg_node | RViz2 |
| `/terrain/traversability_grid` | OccupancyGrid (RL) | terrain_analysis_node | RViz2 |
| `/odometry/filtered` | Odometry (BE) | Gazebo bridge (DiffDrive) | RTAB-Map, Nav2 |
| `/tf` | TFMessage (BE) | Gazebo bridge (odom→base_footprint) | TF2 tree |
| `/tf_static` | TFMessage (transient) | robot_state_publisher | TF2 tree |
| `/joint_states_urdf` | JointState (RL) | joint_state_relay | robot_state_publisher |
| `/map` | OccupancyGrid | RTAB-Map | Nav2 GlobalCostmap StaticLayer |
| `/rtabmap/rgbd_image` | RGBDImage | rgbd_sync | rtabmap SLAM node |
| `/cmd_vel` | Twist | Nav2 controller_server | Gazebo bridge → DiffDrive |
| `/clock` | Clock (BE) | Gazebo bridge | All nodes (use_sim_time) |

**QoS abbreviations:** BE = BestEffort (volatile), RL = Reliable (volatile)

---

## 11. TF Tree

```
map
 └── odom                    ← RTAB-Map publishes (map→odom TF)
      └── base_footprint     ← Gazebo DiffDrive (odom→base_footprint TF, via ros_gz_bridge)
           └── base_link     ← robot_state_publisher (URDF static joint)
                ├── camera_link        ← RSP (base_camera_joint: xyz=0.0553 0 0.4323)
                │    └── camera_link_optical  ← RSP (optical frame: Z-fwd, X-right, Y-down)
                ├── imu_link           ← RSP (base_imu_joint)
                ├── left_wheel_link    ← RSP (dynamic from /joint_states_urdf)
                ├── right_wheel_link   ← RSP (dynamic from /joint_states_urdf)
                ├── front_caster_wheel_link  ← RSP (static)
                └── rear_caster_wheel_link   ← RSP (static)
```

### Simulation QoS Bridge Chain

```
Gazebo DiffDrive plugin
  → /model/localbot/tf  (gz Pose_V)
  → ros_gz_bridge (BestEffort)
  → /tf  (TFMessage, BestEffort)
  → TF2 buffer (works with BestEffort)

Gazebo JointStatePublisher plugin
  → /world/world_demo/model/localbot/joint_state  (gz Model, BestEffort)
  → ros_gz_bridge → /joint_states  (BestEffort)
  → joint_state_relay.py  (QoS bridge)
  → /joint_states_urdf  (Reliable)
  → robot_state_publisher  (needs Reliable)
```

---

## 12. RViz Dashboard

The default [`rviz/ugv_autonomy.rviz`](src/ugv_bringup/rviz/ugv_autonomy.rviz) shows:

| Panel | Topic | Notes |
|---|---|---|
| Robot Model | TF + URDF | Localbot with wheels |
| TF tree | `/tf`, `/tf_static` | Full transform tree |
| Map | `/map` | RTAB-Map 2D occupancy |
| Global costmap | `/global_costmap/costmap` | Inflated SLAM map |
| Local costmap | `/local_costmap/costmap` | Rolling 8×8 m window |
| Nav2 path | `/plan` | Global planned path |
| YOLO overlay | `/perception/yolo/overlay` | Instance segmentation |
| Depth colorized | `/perception/depth/colorized` | TURBO depth map |
| Point cloud | `/perception/depth/points` | 3D obstacle cloud |
| Traversability grid | `/terrain/traversability_grid` | 2.5D cost map |
| Odometry | `/odometry/filtered` | Wheel odometry arrow |

---

## 13. Hardware Mode

```bash
# With RealSense
ros2 launch realsense2_camera rs_launch.py &
ros2 launch ugv_bringup ugv_complete.launch.py \
  mode:=hw \
  use_sim_time:=false \
  camera_type:=realsense

# With ZED
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2 &
ros2 launch ugv_bringup ugv_complete.launch.py \
  mode:=hw \
  use_sim_time:=false \
  camera_type:=zed

# With DA V3 TRT (monocular camera)
# 1. Start Docker container (see Section 8)
docker compose -f docker/da3_trt.compose.yml up &
# 2. Start UGV stack
ros2 launch ugv_bringup ugv_complete.launch.py \
  mode:=hw \
  use_sim_time:=false \
  camera_type:=da_v3_trt
```

**Hardware mode differences from sim:**
- `sim.launch.py` is skipped (no Gazebo)
- `robot_state_publisher` is started directly by `ugv_complete.launch.py`
- EKF (`ekf_filter_node`) is started for sensor fusion
- `use_sim_time:=false` — all nodes use wall clock

---

## 14. Modifying the System

### Change Speed Limits

Edit `src/ugv_bringup/config/nav2_params.yaml`:

```yaml
FollowPath:
  desired_linear_vel: 0.4      # m/s — increase for faster navigation
  rotate_to_heading_angular_vel: 0.8  # rad/s
```

### Change Inflation Radius

```yaml
inflation_layer:
  inflation_radius: 0.75   # metres — decrease if path planning fails in tight spaces
```

### Add a New World

1. Create an SDF world file: `src/ugv_sim/worlds/my_world.sdf`
2. Ensure `<world name="my_world">` attribute matches `world_name` launch argument
3. Do **not** include the robot — it is spawned separately
4. Launch with: `ros2 launch ugv_bringup ugv_complete.launch.py world:=/path/to/my_world.sdf world_name:=my_world`

### Override Perception Config

```bash
ros2 launch ugv_bringup ugv_complete.launch.py \
  perception_config:=/path/to/my_perception.yaml
```

### Add Custom YOLO Classes

Edit `src/ugv_perception/config/perception.yaml`:

```yaml
yolo_seg_node:
  ros__parameters:
    lethal_classes: [person, car, ...]   # substring match against COCO-80 class names
    caution_classes: [vegetation, ...]
```

### Disable RTAB-Map (use pre-built map)

```bash
ros2 launch ugv_bringup ugv_complete.launch.py enable_rtabmap:=false
# Provide /map separately (e.g., map_server node)
```

---

## 15. Troubleshooting

### Robot appears frozen in RViz (wheels always at 0°)

**Cause:** `robot_state_publisher` is receiving `/joint_states` (BestEffort) instead of `/joint_states_urdf` (Reliable).

**Fix:** Verify `joint_state_relay` is running: `ros2 node list | grep joint_state`

### `/map` never published ("No map received")

**Cause:** RTAB-Map is waiting for camera + depth synchronization.

**Fix:**
```bash
# Check rgbd_sync is receiving both topics
ros2 topic hz /camera/image_raw
ros2 topic hz /perception/depth/image_raw

# Check RTAB-Map is active
ros2 node list | grep rtabmap
```

### Nav2 "TF timeout" or "ExtrapolationException"

**Cause:** RTAB-Map publishes `map→odom` TF at ~1 Hz. Nav2 needs this TF.

**Fix:** Already handled by `transform_tolerance: 1.0` in `nav2_params.yaml`. If still failing:
```bash
ros2 run tf2_tools view_frames   # visualize full TF tree as PDF
```

### YOLO model not found

**Cause:** `yolo26n-seg.pt` is not downloaded yet.

**Fix:**
```bash
python3 -c "from ultralytics import YOLO; YOLO('yolo26n-seg.pt')"
# Downloads to ~/.config/Ultralytics/ or current directory
```

### DA V3 TRT container: "model file does not exist"

**Cause:** The node resolves `onnx_path` relative to `get_package_share_directory("depth_anything_v3")`. The models directory must be mounted at exactly `/da3_install/depth_anything_v3/share/depth_anything_v3/models/`.

**Fix:** Verify the volume mount in `docker/da3_trt.compose.yml` matches the correct path. Re-run `scripts/setup_da3_docker.sh` if the build volume is missing.

### Gazebo sensors not publishing

**Cause:** `world_name` launch argument doesn't match the SDF `<world name="...">` attribute.

**Fix:** Check the SDF: `grep 'world name' src/ugv_sim/worlds/ugv_test_arena.sdf` → should return `world_demo`. Launch with: `world_name:=world_demo` (default).

### RViz depth image shows black

**This is expected for raw 32FC1.** Subscribe to `/perception/depth/colorized` instead of `/perception/depth/image_raw` in RViz for a visible colored depth image.

### "colcon build" fails for `depth_anything_v3`

**Expected when TRT is not installed.** The CMakeLists.txt issues a warning and skips the build:
```
CMake Warning: TensorRT not found — depth_anything_v3 will NOT be built.
```
Use the Docker approach instead (see Section 8).

---

## License

Apache 2.0 — see individual package `package.xml` files.

## Maintainer

**Prakash Kumar Sahu** — jaysahu0201@gmail.com
