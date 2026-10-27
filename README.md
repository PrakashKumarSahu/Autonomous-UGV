<div align="center">

# 🤖 Autonomous UGV

### ROS 2 Jazzy · Gazebo Harmonic · YOLO26n-seg · Depth Anything V3 · RTAB-Map · Nav2

[![ROS 2](https://img.shields.io/badge/ROS_2-Jazzy-blue?logo=ros&logoColor=white)](https://docs.ros.org/en/jazzy/)
[![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic-orange?logo=gazebo)](https://gazebosim.org/)
[![Python](https://img.shields.io/badge/Python-3.12-3776AB?logo=python&logoColor=white)](https://www.python.org/)
[![License](https://img.shields.io/badge/License-Apache_2.0-green.svg)](LICENSE)

A production-quality autonomous ground vehicle stack for **GPS-denied outdoor navigation** using a single RGB-D camera. Combines real-time AI perception, visual SLAM, terrain traversability mapping, and reactive autonomous navigation — all running in Gazebo Harmonic simulation with a seamless path to real hardware.

[**Quick Start**](#-quick-start) · [**Architecture**](#-architecture) · [**Configuration**](#-configuration-reference) · [**Troubleshooting**](#-troubleshooting) · [**GitHub**](#-github-repository)

</div>

---

## ⚡ Quick Start

> [!IMPORTANT]
> Requires Ubuntu 24.04, ROS 2 Jazzy, and an NVIDIA GPU. See [Prerequisites](#-prerequisites) for full details.

```bash
# 1. Clone (with submodules)
git clone --recurse-submodules https://github.com/PrakashKumarSahu/Autonomous-UGV.git
cd Autonomous-UGV

# 2. Install dependencies & build
sudo apt install -y ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge ros-jazzy-nav2-bringup \
  ros-jazzy-rtabmap-ros ros-jazzy-robot-state-publisher ros-jazzy-cv-bridge \
  ros-jazzy-sensor-msgs-py ros-jazzy-tf2-tools ros-jazzy-robot-localization
pip install "numpy==1.26.4" ultralytics torch torchvision pyyaml
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

# 3. Launch
ros2 launch ugv_bringup ugv_complete.launch.py
```

After a few seconds you'll see Gazebo, RViz, and the full pipeline active. Send a navigation goal via the **"2D Nav Goal"** button in RViz.

---

## 📖 Table of Contents

1. [What is this?](#-what-is-this)
2. [Architecture](#-architecture)
3. [Package Structure](#-package-structure)
4. [Prerequisites](#-prerequisites)
5. [Installation & Build](#-installation--build)
6. [Running the System](#-running-the-system)
7. [Depth Source Modes](#-depth-source-modes)
8. [Depth Anything V3 TRT — Docker](#-depth-anything-v3-trt--docker)
9. [Configuration Reference](#-configuration-reference)
10. [Topic Contract](#-topic-contract)
11. [TF Tree](#-tf-tree)
12. [RViz Dashboard](#-rviz-dashboard)
13. [Hardware Mode](#-hardware-mode)
14. [Modifying the System](#-modifying-the-system)
15. [Troubleshooting](#-troubleshooting)
16. [GitHub Repository](#-github-repository)

---

## 🧠 What is this?

The **Autonomous UGV** is a complete robotics software stack for a differential-drive ground vehicle ("Localbot") capable of autonomous navigation in complex indoor/outdoor environments — **without GPS**.

### What it does

| Capability | Technology | Description |
|---|---|---|
| **Perception** | YOLO26n-seg | Real-time instance segmentation → hazard mask (lethal / caution / safe) |
| **Depth Estimation** | Gazebo sim / DA V3 TRT / RealSense / ZED | Metric depth → 3D point cloud for obstacle avoidance |
| **SLAM** | RTAB-Map | RGB-D visual SLAM → 2D occupancy map + map→odom transform |
| **Terrain Analysis** | Custom 2.5D grid | Elevation + semantic layers → traversability cost grid |
| **Navigation** | Nav2 (RPP + SmacPlanner2D) | Global path planning + local reactive control |
| **Simulation** | Gazebo Harmonic | Physics, sensors, DiffDrive — `ugv_test_arena.sdf` obstacle course |

### Robot Specifications — Localbot

| Property | Value |
|---|---|
| Chassis dimensions | 0.60 × 0.40 × 0.20 m |
| Drive type | Differential drive |
| Wheel radius / separation | 0.10 m / 0.45 m |
| Max linear / angular speed | 1.5 m/s / 2.0 rad/s |
| Camera | RGB-D · 640×480 · 30 Hz · 80° FOV · 0.3–10 m range |
| IMU | 50 Hz · σ = 0.005 rad/s |

---

## 🏗️ Architecture

### System Pipeline

```
╔══════════════════════════════════════════════════════════════════════════╗
║  ros2 launch ugv_bringup ugv_complete.launch.py                          ║
╠═══════════════════════════╦══════════════════════════════════════════════╣
║  SIMULATION (ugv_sim)     ║  PERCEPTION (ugv_perception)                 ║
║  ─────────────────────    ║  ────────────────────────────────────────    ║
║  Gazebo Harmonic          ║  ┌─ camera_type arg (pick ONE) ──────────┐   ║
║  └─ localbot/model.sdf   ║  │  sim        → depth_relay_node        │   ║
║       DiffDrive plugin    ║  │  da_v3_trt  → da3_relay_node + Docker │   ║
║       RGB-D camera        ║  │  realsense  → realsense_relay_node    │   ║
║       IMU                 ║  │  zed        → zed_relay_node          │   ║
║  ros_gz_bridge            ║  └────────────────────────────────────────┘   ║
║       /camera/*           ║  ┌─ always on ──────────────────────────┐   ║
║       /odometry/filtered  ║  │  depth_to_pointcloud_node            │   ║
║       /joint_states       ║  │  yolo_seg_node (YOLO26n-seg)         │   ║
║       /tf  /clock         ║  └────────────────────────────────────────┘   ║
╠═══════════════════════════╩════════════╦═════════════════════════════════╣
║  TERRAIN (ugv_terrain)                 ║  Outputs                         ║
║  ──────────────────────                ║  /perception/depth/image_raw     ║
║  terrain_analysis_node                 ║  /perception/depth/points        ║
║  depth points + hazard mask            ║  /perception/hazard_mask         ║
║  → /terrain/traversability_grid        ║  /terrain/traversability_grid    ║
╠════════════════════════════════════════╩═════════════════════════════════╣
║  LOCALIZATION (ugv_bringup/localization.launch.py)                        ║
║  rgbd_sync → /rtabmap/rgbd_image                                          ║
║  rtabmap   → /map (OccupancyGrid) + map→odom TF at ~1 Hz                  ║
╠══════════════════════════════════════════════════════════════════════════╣
║  NAVIGATION (ugv_bringup/navigation.launch.py)  — Nav2                    ║
║  bt_navigator → planner_server (SmacPlanner2D) → /plan                    ║
║             → controller_server (RPP 20 Hz)    → /cmd_vel                 ║
║  Local costmap : ObstacleLayer ← /perception/depth/points                 ║
║  Global costmap: StaticLayer   ← /map                                     ║
║  Recovery: behavior_server (Spin → BackUp → Wait)                         ║
╚══════════════════════════════════════════════════════════════════════════╝
```

### DA V3 TRT Docker Integration

```
HOST (ROS 2 Jazzy)                         DOCKER CONTAINER
────────────────────────────────    ───────────────────────────────────────
da3_relay_node                     tillbeemelmanns/ros2-depth-anything-v3
  ← /depth_anything_v3/output/  ◄── DDS FastDDS (--network host) ──►  depth_anything_v3_main
       depth_image (32FC1)           ← /camera/image_raw
  → /perception/depth/*              ← /camera/camera_info
                                     → /depth_anything_v3/output/depth_image
depth_to_pointcloud_node
  → /perception/depth/points
```

---

## 📦 Package Structure

```
Autonomous-UGV/
├── src/
│   ├── ugv_bringup/              # 🔧 Top-level orchestrator — launch + all configs
│   │   ├── launch/
│   │   │   ├── ugv_complete.launch.py   ← SINGLE ENTRY POINT
│   │   │   ├── perception.launch.py     # Depth source selector + YOLO + point cloud
│   │   │   ├── localization.launch.py   # RTAB-Map SLAM + EKF (hw only)
│   │   │   ├── navigation.launch.py     # Full Nav2 stack
│   │   │   └── rviz.launch.py
│   │   ├── config/
│   │   │   ├── nav2_params.yaml         # ← All Nav2 tuning lives here
│   │   │   ├── rtabmap.yaml             # ← All SLAM tuning lives here
│   │   │   ├── ekf.yaml                 # EKF (hardware mode)
│   │   │   └── da3_ugv.param.yaml       # DA V3 TRT node params
│   │   └── rviz/ugv_autonomy.rviz
│   │
│   ├── ugv_description/          # 🤖 URDF — kinematics + TF only (NOT loaded by Gazebo)
│   │   └── urdf/
│   │       ├── ugv.urdf.xacro
│   │       ├── ugv_base.urdf.xacro      # Chassis, wheels, casters
│   │       └── sensors/
│   │           ├── camera.urdf.xacro    # camera_link + camera_link_optical
│   │           └── imu.urdf.xacro
│   │
│   ├── ugv_perception/           # 👁️ AI perception nodes (Python)
│   │   ├── ugv_perception/
│   │   │   ├── depth_relay_node.py      # sim — Gazebo depth bridge
│   │   │   ├── da3_relay_node.py        # da_v3_trt — DA V3 TRT bridge
│   │   │   ├── realsense_relay_node.py  # realsense — 16UC1 mm → 32FC1 m
│   │   │   ├── zed_relay_node.py        # zed — frame correction + colorize
│   │   │   ├── depth_to_pointcloud_node.py  # 32FC1 → PointCloud2
│   │   │   └── yolo_seg_node.py         # YOLO26n-seg → hazard mask
│   │   └── config/perception.yaml
│   │
│   ├── ugv_terrain/              # 🗺️ 2.5D traversability mapping (Python)
│   │   ├── ugv_terrain/terrain_analysis_node.py
│   │   └── config/terrain.yaml
│   │
│   ├── ugv_sim/                  # 🌍 Gazebo simulation
│   │   ├── launch/sim.launch.py         # Gazebo + RSP + bridge (dynamic YAML) + spawn
│   │   ├── models/localbot/model.sdf    # ← Actual robot Gazebo loads (DiffDrive here)
│   │   ├── worlds/
│   │   │   ├── ugv_test_arena.sdf       # Default obstacle course
│   │   │   └── empty_world.sdf
│   │   └── scripts/joint_state_relay.py # QoS bridge: BestEffort→Reliable
│   │
│   └── ugv_control/              # ⚙️ ros2_control scaffold (NOT active in sim — future hw)
│       ├── config/diff_drive_controller.yaml
│       └── launch/control.launch.py
│
├── docker/                       # 🐳 Depth Anything V3 TRT container
│   ├── da3_trt.compose.yml       # Docker Compose — nvidia runtime, host network
│   ├── da3_docker.param.yaml     # Node params (mounted read-only)
│   ├── .env                      # ROS_DOMAIN_ID, RMW_IMPLEMENTATION defaults
│   └── README.md
│
├── third_party/
│   ├── ros2-depth-anything-v3-trt/  # DA V3 C++ ROS 2 package (git submodule)
│   └── yolo_ros/                    # Reference only (not used by perception nodes)
│
└── scripts/
    ├── setup_da3_docker.sh       # ← Run this once to set up DA V3 Docker
    └── setup_da3_trt.sh          # Native host TRT build (advanced)
```

> [!NOTE]
> **Why does `ugv_description` not have Gazebo plugins?**
> Gazebo loads `localbot/model.sdf` directly — not the URDF. The URDF is used **only** by `robot_state_publisher` to broadcast TF transforms. All simulation physics, sensors, and DiffDrive are declared inside `model.sdf`.

> [!NOTE]
> **Why is `ugv_control` not launched?**
> The simulation uses Gazebo's native DiffDrive plugin. `ugv_control` is scaffolding for future hardware deployments using `ros2_control`-based motor drivers.

---

## 💻 Prerequisites

### Hardware Requirements

| Component | Minimum | Recommended |
|---|---|---|
| GPU | NVIDIA GTX 1060 (6 GB VRAM) | NVIDIA RTX 4050+ (8 GB VRAM) |
| RAM | 8 GB | 16 GB |
| Disk | 30 GB free | 60 GB (DA V3 Docker image = 17 GB) |
| OS | Ubuntu 24.04 LTS | Ubuntu 24.04 LTS |
| NVIDIA Driver | 520+ | 595+ |

### Software Requirements

| Software | Version | Notes |
|---|---|---|
| ROS 2 | Jazzy Jalisco | [Installation guide](https://docs.ros.org/en/jazzy/Installation.html) |
| Gazebo | Harmonic | Installed via `ros-jazzy-ros-gz-*` |
| Python | 3.12 | Ships with Ubuntu 24.04 |
| CUDA | 12.0+ | Required for YOLO GPU inference |
| Docker | 24.0+ | Required only for DA V3 TRT mode |

### ROS 2 Packages

```bash
sudo apt install -y \
  ros-jazzy-ros-gz-sim \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-ros-gz-image \
  ros-jazzy-nav2-bringup \
  ros-jazzy-nav2-smac-planner \
  ros-jazzy-nav2-regulated-pure-pursuit-controller \
  ros-jazzy-rtabmap-ros \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-joint-state-publisher \
  ros-jazzy-robot-localization \
  ros-jazzy-tf2-ros \
  ros-jazzy-tf2-tools \
  ros-jazzy-sensor-msgs-py \
  ros-jazzy-cv-bridge \
  ros-jazzy-image-transport
```

### Python Dependencies

```bash
pip install "numpy==1.26.4" ultralytics torch torchvision pyyaml
```

> [!WARNING]
> **Do NOT install numpy 2.x.** ROS 2 Jazzy's `cv_bridge` was compiled against numpy 1.x. Upgrading will cause: `TypeError: bytes expected, not memoryview` — crashing all perception nodes.

---

## 🔧 Installation & Build

### Step 1 — Clone the repository

```bash
# --recurse-submodules is required for third_party/ros2-depth-anything-v3-trt
git clone --recurse-submodules https://github.com/PrakashKumarSahu/Autonomous-UGV.git
cd Autonomous-UGV
```

> [!TIP]
> If you forgot `--recurse-submodules`, run: `git submodule update --init --recursive`

### Step 2 — Install dependencies

```bash
# ROS 2 packages (see Prerequisites section above for full list)
sudo apt install -y ros-jazzy-ros-gz-sim ros-jazzy-nav2-bringup ros-jazzy-rtabmap-ros \
  ros-jazzy-robot-state-publisher ros-jazzy-cv-bridge ros-jazzy-sensor-msgs-py \
  ros-jazzy-tf2-tools ros-jazzy-robot-localization ros-jazzy-ros-gz-bridge \
  ros-jazzy-ros-gz-image ros-jazzy-nav2-smac-planner \
  ros-jazzy-nav2-regulated-pure-pursuit-controller ros-jazzy-joint-state-publisher \
  ros-jazzy-tf2-ros ros-jazzy-image-transport

# Python packages
pip install "numpy==1.26.4" ultralytics torch torchvision pyyaml
```

### Step 3 — Build

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

> [!NOTE]
> You will see a CMake warning: `TensorRT not found — depth_anything_v3 will NOT be built.`  
> This is **expected** unless you're using DA V3 TRT mode. The rest of the system builds normally.

### Step 4 — Pre-download YOLO model (optional but recommended)

```bash
python3 -c "from ultralytics import YOLO; YOLO('yolo26n-seg.pt')"
```

`yolo26n-seg.pt` (~6.4 MB) downloads automatically on first launch. This pre-downloads it so the first launch isn't delayed.

---

## 🚀 Running the System

### Default: Simulation Mode

```bash
source install/setup.bash
ros2 launch ugv_bringup ugv_complete.launch.py
```

This starts everything: **Gazebo** → **perception pipeline** → **RTAB-Map SLAM** → **Nav2** → **RViz**.

Wait for this message before sending goals:
```
[lifecycle_manager_navigation] Lifecycle manager navigation is active
```

### Send a Navigation Goal

```bash
# Option A — RViz2 (recommended):  click "2D Nav Goal" and click a point on the map

# Option B — CLI:
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 3.0, y: 2.0, z: 0.0}, orientation: {w: 1.0}}}}"
```

### Common Launch Variants

```bash
# Custom world
ros2 launch ugv_bringup ugv_complete.launch.py \
  world:=$(pwd)/src/ugv_sim/worlds/empty_world.sdf \
  world_name:=empty

# Custom spawn pose
ros2 launch ugv_bringup ugv_complete.launch.py \
  spawn_x:=1.0 spawn_y:=0.0 spawn_yaw:=0.0

# DA V3 TRT depth (after Docker setup — see Section 8)
ros2 launch ugv_bringup ugv_complete.launch.py camera_type:=da_v3_trt

# Override Nav2 params without rebuilding
ros2 launch ugv_bringup ugv_complete.launch.py \
  nav2_params_file:=/path/to/my_nav2_params.yaml

# No RViz (headless)
ros2 launch ugv_bringup ugv_complete.launch.py enable_rviz:=false
```

### Launch Arguments Reference

| Argument | Default | Options / Description |
|---|---|---|
| `mode` | `sim` | `sim` — Gazebo simulation · `hw` — real robot |
| `use_sim_time` | `true` | `true` in sim, `false` on hardware |
| `camera_type` | `sim` | `sim` · `da_v3_trt` · `realsense` · `zed` |
| `world` | `ugv_test_arena.sdf` | Absolute path to any Gazebo SDF world |
| `world_name` | `world_demo` | Must match `<world name="...">` inside the SDF |
| `robot_name` | `localbot` | Gazebo entity name |
| `spawn_x` | `0.0` | Spawn X position (m) |
| `spawn_y` | `-6.5` | Spawn Y position (m) |
| `spawn_z` | `0.01` | Spawn Z position (m) |
| `spawn_yaw` | `1.5708` | Spawn yaw (rad) — π/2 = facing north |
| `enable_rtabmap` | `true` | Set `false` to skip SLAM (use a pre-built map) |
| `enable_rviz` | `true` | Set `false` for headless operation |
| `nav2_params_file` | `config/nav2_params.yaml` | Override Nav2 config path |

---

## 🔭 Depth Source Modes

The depth pipeline uses a **swappable relay architecture**: exactly one relay node runs at a time (selected by `camera_type`), and all four produce the same canonical output topics. The rest of the stack (SLAM, Nav2, terrain) never changes.

### Depth Source Comparison

| `camera_type` | Hardware Required | Setup Complexity | Accuracy | Use Case |
|---|---|---|---|---|
| `sim` | None (Gazebo) | ⭐ None | Ground truth | Development & testing |
| `da_v3_trt` | Any RGB camera + NVIDIA GPU | ⭐⭐⭐ Docker | Very high (monocular) | Outdoor with single camera |
| `realsense` | Intel RealSense D435/D455 | ⭐⭐ Driver install | High (structured light) | Indoor/outdoor, \<5 m |
| `zed` | Stereolabs ZED 2 / ZED X | ⭐⭐⭐ SDK install | High (stereo) | Outdoor, \<20 m |

### Canonical Output Contract (all modes publish these)

| Topic | Type | QoS | Consumers |
|---|---|---|---|
| `/perception/depth/image_raw` | `Image` (32FC1, metres) | RELIABLE | RTAB-Map, depth_to_pointcloud |
| `/perception/depth/camera_info` | `CameraInfo` | RELIABLE | RTAB-Map, depth_to_pointcloud |
| `/perception/depth/colorized` | `Image` (bgr8, TURBO) | RELIABLE | RViz2 visualization |
| `/perception/depth/points` | `PointCloud2` | RELIABLE | Nav2 ObstacleLayer |

### Nodes Always Running (independent of `camera_type`)

| Node | Input → Output | Purpose |
|---|---|---|
| `depth_to_pointcloud_node` | `/perception/depth/image_raw` → `/perception/depth/points` | Feeds Nav2 local costmap |
| `yolo_seg_node` | `/camera/image_raw` → `/perception/hazard_mask` + `/perception/yolo/overlay` | Semantic hazard detection |

---

## 🐳 Depth Anything V3 TRT — Docker

Depth Anything V3 TRT requires CUDA + TensorRT to compile. The **recommended approach** runs it inside a prebuilt Docker container — no host-side TRT installation needed.

> [!IMPORTANT]
> The Docker image is **~17 GB**. Ensure you have at least 25 GB free disk space before proceeding.

### Prerequisites (one-time system setup)

**Step 1 — Add yourself to the docker group:**

```bash
sudo usermod -aG docker $USER
newgrp docker   # apply without logout
```

**Step 2 — Install NVIDIA Container Toolkit:**

```bash
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey \
  | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg

curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list \
  | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' \
  | sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

sudo apt update && sudo apt install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

**Step 3 — Verify GPU access:**

```bash
docker run --rm --gpus all nvidia/cuda:12.0-base-ubuntu22.04 nvidia-smi
```

### One-Time DA V3 Setup

```bash
bash scripts/setup_da3_docker.sh
```

This script automatically:
1. Checks GPU access and Docker permissions
2. Pulls `tillbeemelmanns/ros2-depth-anything-v3:latest-dev` (~17 GB — TRT 10.13 + CUDA 13 + ROS 2 Jazzy)
3. Downloads `DA3METRIC-LARGE.onnx` (~700 MB) from HuggingFace
4. Builds the `depth_anything_v3` C++ package inside the container (colcon, ~40 s)
5. Caches the build in Docker volume `ugv_da3_build` (survives container restarts)
6. Verifies the `depth_anything_v3_main` binary exists

### Running with DA V3 TRT

```bash
# Terminal 1 — start the Docker container (GPU inference)
docker compose -f docker/da3_trt.compose.yml up

# Terminal 2 — start the UGV stack
ros2 launch ugv_bringup ugv_complete.launch.py camera_type:=da_v3_trt
```

> [!WARNING]
> **First launch**: TensorRT compiles the ONNX model to an optimized `.engine` file. This takes **3–8 minutes** on an RTX 4050. The engine is cached alongside the ONNX file and reused on every subsequent launch (startup in <10 s).

### Docker Configuration Files

| File | Purpose |
|---|---|
| [`docker/da3_trt.compose.yml`](docker/da3_trt.compose.yml) | NVIDIA runtime, `--network host`, build volume mount, launch command |
| [`docker/da3_docker.param.yaml`](docker/da3_docker.param.yaml) | DA V3 node params — `onnx_path`, `precision`, `sky_threshold` |
| [`docker/.env`](docker/.env) | Env defaults: `ROS_DOMAIN_ID=0`, `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` |
| [`docker/README.md`](docker/README.md) | Extended Docker architecture documentation |

> [!NOTE]
> **How `onnx_path` is resolved:** The C++ node prepends `get_package_share_directory("depth_anything_v3")` to the value. The compose file mounts the models directory to `/da3_install/depth_anything_v3/share/depth_anything_v3/models/` — exactly where the node looks.

---

## ⚙️ Configuration Reference

### Configuration Files Map

| File | Controls | When to Edit |
|---|---|---|
| `src/ugv_bringup/config/nav2_params.yaml` | Speed, lookahead, inflation, costmap, recovery | Nav2 tuning |
| `src/ugv_bringup/config/rtabmap.yaml` | Keyframes, grid map, visual features, 2D SLAM | SLAM tuning |
| `src/ugv_bringup/config/ekf.yaml` | Sensor fusion (hw mode) | Hardware deployment |
| `src/ugv_bringup/config/da3_ugv.param.yaml` | DA V3 model path, sky cap, FP16 | DA V3 TRT config |
| `src/ugv_perception/config/perception.yaml` | YOLO model, YOLO classes, relay topics | Perception tuning |
| `src/ugv_terrain/config/terrain.yaml` | Grid resolution, step/slope thresholds | Terrain tuning |
| `src/ugv_sim/models/localbot/model.sdf` | Robot physics, sensor specs, DiffDrive | Robot hardware changes |
| `src/ugv_sim/worlds/ugv_test_arena.sdf` | World obstacles, terrain, lighting | Simulation environment |

### Key Nav2 Parameters (`nav2_params.yaml`)

| Parameter | Value | Why This Value |
|---|---|---|
| `desired_linear_vel` | 0.4 m/s | Slower = more time to react to obstacles (was 0.5) |
| `inflation_radius` | 0.75 m | Robot (r=0.32 m) + 0.43 m safety buffer |
| `use_collision_detection` | `true` | RPP stops if planned path hits costmap obstacle |
| `movement_time_allowance` | 8.0 s | Triggers stuck recovery after 8 s of no progress |
| `observation_persistence` | 0.5 s | Clears stale cloud obstacles quickly on rotation |
| `transform_tolerance` | 1.0 s | Matches RTAB-Map ~1 Hz TF publish rate |
| `failure_tolerance` | 0.5 s | Fails fast on TF/controller errors → faster recovery |

### Key RTAB-Map Parameters (`rtabmap.yaml`)

| Parameter | Value | Why This Value |
|---|---|---|
| `RGBD/LinearUpdate` | `"0.0"` | First keyframe created immediately (no movement required) |
| `RGBD/AngularUpdate` | `"0.0"` | Same — prevents "No map received" on startup |
| `Reg/Force3DoF` | `"true"` | 2D planar SLAM — correct for a ground UGV |
| `Optimizer/Slam2D` | `"true"` | Graph optimization in 2D only |
| `Grid/RangeMax` | `"10.0"` | Matches Gazebo depth sensor far clip |
| `wait_for_transform` | `1.0` | Prevents frame drops during Gazebo startup |
| `GridGlobal/FootprintRadius` | `"0.55"` | Footprint inflation for grid map erosion |

### YOLO26n-seg Parameters (`perception.yaml`)

| Parameter | Default | Description |
|---|---|---|
| `model_name` | `yolo26n-seg.pt` | Auto-downloaded on first run by Ultralytics |
| `confidence_threshold` | `0.35` | Minimum detection confidence |
| `device` | `cuda:0` | GPU inference (auto-falls back to CPU) |
| `enable_fp16` | `true` | Half-precision for ~2× speed |
| `lethal_classes` | `[person, car, truck, ...]` | Mask value = 255 (red, obstacle) |
| `caution_classes` | `[vegetation, bush, ...]` | Mask value = 128 (amber, slow-down) |

---

## 📡 Topic Contract

### Core Topics

| Topic | Type | Publisher | QoS | Consumers |
|---|---|---|---|---|
| `/camera/image_raw` | `Image` (bgr8) | Gazebo bridge | BE | YOLO, RTAB-Map, DA V3 |
| `/camera/camera_info` | `CameraInfo` | Gazebo bridge | BE | RTAB-Map, DA V3 relay |
| `/camera/depth/image_raw` | `Image` (32FC1) | Gazebo bridge | BE | `depth_relay_node` (sim only) |
| `/perception/depth/image_raw` | `Image` (32FC1) | Active relay | RL | RTAB-Map, depth_to_pointcloud |
| `/perception/depth/camera_info` | `CameraInfo` | Active relay | RL | RTAB-Map, depth_to_pointcloud |
| `/perception/depth/colorized` | `Image` (bgr8) | Active relay | RL | RViz2 |
| `/perception/depth/points` | `PointCloud2` | depth_to_pointcloud | RL | Nav2 ObstacleLayer |
| `/perception/hazard_mask` | `Image` (mono8) | yolo_seg_node | RL | terrain_analysis_node |
| `/perception/yolo/overlay` | `Image` (bgr8) | yolo_seg_node | RL | RViz2 |
| `/terrain/traversability_grid` | `OccupancyGrid` | terrain_analysis | RL | RViz2 |
| `/odometry/filtered` | `Odometry` | Gazebo bridge | BE | RTAB-Map, Nav2 |
| `/tf` | `TFMessage` | Gazebo bridge + RSP | BE | TF2 |
| `/joint_states_urdf` | `JointState` | joint_state_relay | RL | robot_state_publisher |
| `/map` | `OccupancyGrid` | RTAB-Map | RL/transient | Nav2 GlobalCostmap |
| `/cmd_vel` | `Twist` | Nav2 controller | RL | Gazebo bridge → DiffDrive |
| `/clock` | `Clock` | Gazebo bridge | BE | All nodes (sim time) |

**QoS:** BE = BestEffort volatile · RL = Reliable volatile

---

## 🌐 TF Tree

```
map
 └── odom                     ← RTAB-Map publishes map→odom at ~1 Hz
      └── base_footprint       ← Gazebo DiffDrive → ros_gz_bridge (BestEffort)
           └── base_link        ← robot_state_publisher (URDF static joint, z=0.2 m)
                ├── camera_link          (x=0.0553 m, z=0.4323 m from base_link)
                │    └── camera_link_optical   (rpy: -π/2, 0, -π/2 — Z forward, X right)
                ├── imu_link
                ├── left_wheel_link      (dynamic from /joint_states_urdf)
                ├── right_wheel_link     (dynamic from /joint_states_urdf)
                ├── front_caster_wheel_link  (static)
                └── rear_caster_wheel_link   (static)
```

### Simulation QoS Bridge Chain

```
Gazebo DiffDrive → /model/localbot/odometry + /model/localbot/tf
  → ros_gz_bridge (BestEffort) → /odometry/filtered + /tf

Gazebo JointStatePublisher → /world/world_demo/model/localbot/joint_state
  → ros_gz_bridge (BestEffort) → /joint_states
  → joint_state_relay.py (QoS bridge)
  → /joint_states_urdf (Reliable)
  → robot_state_publisher
```

> [!NOTE]
> `joint_state_relay.py` is required because `robot_state_publisher` expects RELIABLE QoS but Gazebo publishes BEST_EFFORT. The relay bridges this QoS mismatch.

---

## 📊 RViz Dashboard

The default `rviz/ugv_autonomy.rviz` shows:

| Panel | Topic | Notes |
|---|---|---|
| Robot Model | TF + URDF | Localbot with animated wheels |
| TF Tree | `/tf`, `/tf_static` | Full transform visualization |
| SLAM Map | `/map` | RTAB-Map 2D occupancy grid |
| Global Costmap | `/global_costmap/costmap` | Inflated SLAM map for planning |
| Local Costmap | `/local_costmap/costmap` | Rolling 8×8 m obstacle window |
| Planned Path | `/plan` | SmacPlanner2D output |
| YOLO Overlay | `/perception/yolo/overlay` | Detected instances colored |
| Depth (colorized) | `/perception/depth/colorized` | TURBO colormap — subscribe this, not raw |
| Point Cloud | `/perception/depth/points` | 3D obstacle cloud |
| Traversability | `/terrain/traversability_grid` | 2.5D cost grid |
| Odometry | `/odometry/filtered` | Wheel odometry arrow |

> [!TIP]
> For depth visualization in RViz, always subscribe to `/perception/depth/colorized` (bgr8 TURBO map). The raw `/perception/depth/image_raw` (32FC1) will appear mostly black in RViz because NaN pixels map to zero.

---

## 🔌 Hardware Mode

Switch from simulation to a real robot by changing two arguments:

```bash
# With Intel RealSense D435/D455
sudo apt install ros-jazzy-realsense2-camera
ros2 launch realsense2_camera rs_launch.py &

ros2 launch ugv_bringup ugv_complete.launch.py \
  mode:=hw \
  use_sim_time:=false \
  camera_type:=realsense
```

```bash
# With Stereolabs ZED 2 / ZED X
# Install ZED SDK: https://www.stereolabs.com/developers/release
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2 &

ros2 launch ugv_bringup ugv_complete.launch.py \
  mode:=hw \
  use_sim_time:=false \
  camera_type:=zed
```

```bash
# With monocular camera + DA V3 TRT (see Docker section first)
docker compose -f docker/da3_trt.compose.yml up &

ros2 launch ugv_bringup ugv_complete.launch.py \
  mode:=hw \
  use_sim_time:=false \
  camera_type:=da_v3_trt
```

**What changes in `hw` mode vs `sim`:**
- `sim.launch.py` is skipped — no Gazebo
- `robot_state_publisher` is started directly
- `ekf_filter_node` is started for IMU + odometry fusion
- All nodes use wall clock (`use_sim_time:=false`)

---

## 🔨 Modifying the System

### Change Navigation Speed

```yaml
# src/ugv_bringup/config/nav2_params.yaml
FollowPath:
  desired_linear_vel: 0.6              # increase (max safe ~0.8 m/s)
  rotate_to_heading_angular_vel: 1.2   # increase proportionally
```

### Change Safety Inflation

```yaml
# nav2_params.yaml — both local and global costmap
inflation_layer:
  inflation_radius: 0.55   # reduce if Nav2 can't find paths in tight spaces
```

### Add / Change YOLO Hazard Classes

```yaml
# src/ugv_perception/config/perception.yaml
yolo_seg_node:
  ros__parameters:
    lethal_classes: [person, car, truck, bicycle, dog, rock, boulder, tree, trunk]
    caution_classes: [vegetation, bush, shrub, potted plant]
    unknown_class_cost: 128   # 0=ignore, 128=caution, 255=lethal
```

Class matching is substring-based against COCO-80 class names (case-insensitive).

### Add a Custom World

```bash
# 1. Create your SDF
cp src/ugv_sim/worlds/empty_world.sdf src/ugv_sim/worlds/my_world.sdf
# Edit my_world.sdf — note the <world name="my_world"> attribute

# 2. Launch with it
ros2 launch ugv_bringup ugv_complete.launch.py \
  world:=$(pwd)/src/ugv_sim/worlds/my_world.sdf \
  world_name:=my_world
```

> [!WARNING]
> The `world_name` argument **must exactly match** the `<world name="...">` attribute inside the SDF file. Mismatch causes Gazebo to fail silently and sensors to not publish.

### Disable SLAM (pre-built map)

```bash
ros2 launch ugv_bringup ugv_complete.launch.py enable_rtabmap:=false
# Then provide /map via: ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=my_map.yaml
```

### Tune the Point Cloud (Nav2 obstacle density)

```yaml
# src/ugv_perception/config/perception.yaml
depth_to_pointcloud_node:
  ros__parameters:
    step: 4       # sample every 4th pixel → ~4800 pts/frame at 640×480
                  # lower = denser cloud (more CPU); higher = fewer points (faster)
    min_depth: 0.2   # metres — discard points closer than this
    max_depth: 10.0  # metres — discard points farther than this
    stamp_offset: 0.15  # seconds — back-stamps cloud to avoid TF ExtrapolationException
```

---

## 🐛 Troubleshooting

### Issue: Robot appears stationary in RViz (wheels don't move)

**Cause:** `robot_state_publisher` is not receiving joint states — typically because `joint_state_relay.py` is not running, or the Gazebo bridge topic name is wrong.

**Fix:**
```bash
ros2 node list | grep joint_state_relay   # should appear
ros2 topic hz /joint_states               # should be ~30 Hz
ros2 topic hz /joint_states_urdf          # should be ~30 Hz
```

---

### Issue: `/map` never published — "No map received" in RViz

**Cause:** RTAB-Map's `rgbd_sync` is not getting synchronized RGB + depth frames.

**Fix:**
```bash
ros2 topic hz /camera/image_raw              # should be ~30 Hz
ros2 topic hz /perception/depth/image_raw   # should be ~30 Hz

# Check rgbd_sync is outputting
ros2 topic hz /rtabmap/rgbd_image           # should be ~5-15 Hz
```

If `/perception/depth/image_raw` is missing, the active depth relay is not running. Check `camera_type` argument and the appropriate relay node is alive.

---

### Issue: Nav2 "TF timeout" or `ExtrapolationException`

**Cause:** TF lookup for `map→base_footprint` fails because RTAB-Map publishes `map→odom` at ~1 Hz and TF2 cannot extrapolate forward.

**Fix:** Already handled by `transform_tolerance: 1.0` in `nav2_params.yaml`. If still failing:
```bash
ros2 run tf2_tools view_frames  # generates frames.pdf — check map→odom chain exists
ros2 topic hz /tf               # Gazebo odom→base_footprint should be ~50 Hz
```

---

### Issue: YOLO node logs "Could not load YOLO model"

**Cause:** `yolo26n-seg.pt` not found in YOLO's model search path.

**Fix:**
```bash
# Pre-download the model
python3 -c "from ultralytics import YOLO; YOLO('yolo26n-seg.pt')"

# The node falls back to Canny-edge detection automatically — not a crash.
# Navigation still works but semantic hazard classes are unavailable.
```

---

### Issue: DA V3 Docker — "model file does not exist"

**Cause:** The C++ node resolves `onnx_path` relative to its ROS 2 package share directory inside the container. The compose file must mount the models directory to exactly the right path.

**Fix:**
```bash
# Verify the volume mount in docker/da3_trt.compose.yml shows:
#   <host_models_dir>:/da3_install/depth_anything_v3/share/depth_anything_v3/models/

# If the build volume is missing, re-run setup:
bash scripts/setup_da3_docker.sh
```

---

### Issue: RViz depth image appears completely black

**Cause:** You're subscribing to `/perception/depth/image_raw` (32FC1). RViz maps NaN depth pixels to 0 (black) and the rest often appears black without manual normalization.

**Fix:** Subscribe to `/perception/depth/colorized` (bgr8) — this is the TURBO-colormap version with NaN pixels shown as explicit black and real depths as blue→red gradient.

---

### Issue: Gazebo sensors not publishing — no `/camera/image_raw`

**Cause:** Usually a `world_name` mismatch, or Gazebo is still loading.

**Fix:**
```bash
# Check the world name in the SDF
grep 'world name' src/ugv_sim/worlds/ugv_test_arena.sdf
# Expected: <world name="world_demo">

# Launch with matching argument (default is correct):
ros2 launch ugv_bringup ugv_complete.launch.py world_name:=world_demo

# Check bridge is running
ros2 node list | grep ros_gz_bridge
ros2 topic list | grep camera
```

---

### Issue: `cv_bridge` crash — `TypeError: bytes expected, not memoryview`

**Cause:** numpy 2.x installed. `cv_bridge` in ROS 2 Jazzy requires numpy 1.x.

**Fix:**
```bash
pip install "numpy==1.26.4"
# Then restart all ROS 2 nodes
```

---

## 🐙 GitHub Repository

**Repository:** [https://github.com/PrakashKumarSahu/Autonomous-UGV](https://github.com/PrakashKumarSahu/Autonomous-UGV)  
**Branch:** `main`  
**Author:** Prakash Kumar Sahu &lt;jaysahu0201@gmail.com&gt;

### Clone

```bash
# Required: --recurse-submodules for third_party/ros2-depth-anything-v3-trt
git clone --recurse-submodules https://github.com/PrakashKumarSahu/Autonomous-UGV.git

# Fix a missing submodule after cloning without the flag:
git submodule update --init --recursive
```

### Branching Strategy

| Branch | Purpose |
|---|---|
| `main` | Stable, tested — all features land here |
| `feature/<name>` | New capability development |
| `fix/<name>` | Bug fix before merging to main |

> [!CAUTION]
> **Never force-push `main`.** This is a shared branch. Use `git push origin main` only (no `--force`).

### Commit Convention

This project uses [Conventional Commits](https://www.conventionalcommits.org/):

```
<type>(<scope>): <short imperative summary>

<optional body — explain WHY, not WHAT>
```

| Type | Use for |
|---|---|
| `feat` | New feature or capability |
| `fix` | Bug fix |
| `docs` | Documentation only |
| `refactor` | Code restructuring (no behavior change) |
| `chore` | Build scripts, CI, package metadata |
| `perf` | Performance improvement |

**Real examples from this project:**
```
feat(depth): Docker-based DA V3 TRT integration — verified GPU inference
fix(audit): dead URDF plugins, stale docstrings, wrong email; rewrite README
docs(readme): comprehensive improvement — badges, quick-start, better structure
feat(ugv): YOLO26 upgrade, perception/terrain modularization, bugfixes & README overhaul
```

### Commit History

```
32cf221  docs(readme): add GitHub Repository section
58e2bed  fix(audit): dead URDF plugins, stale docstrings, wrong email; rewrite README
a8f5545  feat(depth): Docker-based DA V3 TRT integration — verified GPU inference
2647043  feat(depth): DA V3 TRT integration; remove DA V2; colcon restructure & README
331f3e6  feat(ugv): YOLO26 upgrade, perception/terrain modularization, bugfixes & README
14a6804  rviz: full panel rewrite + world enhancements
3d123ca  world: fix Gazebo crash — single-link models + shadows off
```

### Local Development Workflow

```bash
# 1. Fork and clone
git clone --recurse-submodules https://github.com/PrakashKumarSahu/Autonomous-UGV.git
cd Autonomous-UGV

# 2. Set author (required before committing)
git config user.name "PrakashKumarSahu"
git config user.email "jaysahu0201@gmail.com"

# 3. Create a branch
git checkout -b fix/my-bugfix

# 4. Make changes, build, verify
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch ugv_bringup ugv_complete.launch.py   # verify it works

# 5. Commit
git add <files>
git commit -m "fix(scope): describe what was fixed and why"

# 6. Push
git push origin fix/my-bugfix
```

### Submodule Management

| Path | Upstream | Purpose |
|---|---|---|
| `third_party/ros2-depth-anything-v3-trt` | `github.com/ika-rwth-aachen/ros2-depth-anything-v3-trt` | DA V3 C++ ROS 2 node |
| `third_party/yolo_ros` | upstream yolo_ros | Reference (not used by perception nodes) |

```bash
# Update DA V3 submodule to latest upstream
git submodule update --remote third_party/ros2-depth-anything-v3-trt
git add third_party/ros2-depth-anything-v3-trt
git commit -m "chore: update DA V3 TRT submodule to latest upstream"
git push origin main
```

### Push Authentication

```bash
# Option 1 — GitHub CLI (easiest)
gh auth login
git push origin main

# Option 2 — Personal Access Token
# Create at: https://github.com/settings/tokens  (scope: repo)
git remote set-url origin https://<TOKEN>@github.com/PrakashKumarSahu/Autonomous-UGV.git
git push origin main

# Option 3 — SSH key
# Add your public key at: https://github.com/settings/keys
git remote set-url origin git@github.com:PrakashKumarSahu/Autonomous-UGV.git
git push origin main
```

---

## 📄 License

Apache 2.0 — see individual package `package.xml` files for per-package licenses.

---

<div align="center">

Made with ❤️ by **[Prakash Kumar Sahu](https://github.com/PrakashKumarSahu)** · jaysahu0201@gmail.com

</div>
