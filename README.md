# Autonomous UGV

A fully visual, camera-only Autonomous Unmanned Ground Vehicle built on **ROS 2 Jazzy** and **Gazebo Harmonic**. The robot uses a single RGB-D camera for all perception: real-time depth sensing, YOLO-based hazard detection, visual SLAM, and reactive navigation via Nav2.

> **No LiDAR. No GPS. Pure vision.**

---

## Table of Contents

- [System Overview](#system-overview)
- [Architecture](#architecture)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Quick Start](#quick-start)
- [Launch Arguments](#launch-arguments)
- [Modules](#modules)
  - [ugv\_sim — Simulation Environment](#ugv_sim--simulation-environment)
  - [ugv\_description — Robot Model & TF](#ugv_description--robot-model--tf)
  - [ugv\_perception — AI Perception Stack](#ugv_perception--ai-perception-stack)
  - [ugv\_terrain — Traversability Mapping](#ugv_terrain--traversability-mapping)
  - [ugv\_bringup — SLAM, Nav2 & Orchestration](#ugv_bringup--slam-nav2--orchestration)
  - [ugv\_control — Hardware ros2\_control (Future)](#ugv_control--hardware-ros2_control-future)
- [Configuration Reference](#configuration-reference)
- [RViz Dashboard](#rviz-dashboard)
- [Worlds](#worlds)
- [Camera / Depth Source Switching](#camera--depth-source-switching)
- [AI Depth Visualization](#ai-depth-visualization)
- [Key Topics](#key-topics)
- [TF Tree](#tf-tree)
- [Known Limitations](#known-limitations)
- [Developer Guide](#developer-guide)

---

## System Overview

The UGV autonomously navigates in unknown indoor/outdoor environments using only a forward-facing RGB-D camera. The full software stack:

1. **Simulation** — Gazebo Harmonic spawns the robot and provides ground-truth sensor data
2. **Perception** — converts raw sensor data to a canonical `/perception/depth/*` interface
3. **SLAM** — RTAB-Map builds a live 2D occupancy map from camera data
4. **Navigation** — Nav2 plans globally on the SLAM map and drives via Regulated Pure Pursuit
5. **Terrain Analysis** — a rolling traversability grid fuses depth + YOLO hazard masks

Everything launches from a single command:

```bash
ros2 launch ugv_bringup ugv_complete.launch.py
```

---

## Architecture

```
┌──────────────────────────────────────────────────────────────────┐
│                    ugv_complete.launch.py                        │
│  ┌──────────┐  ┌────────────┐  ┌──────────────┐  ┌──────────┐  │
│  │ ugv_sim  │  │ugv_percep. │  │  ugv_terrain │  │ugv_bringup│  │
│  │ Gazebo   │  │  AI Stack  │  │ Traversability│  │SLAM+Nav2 │  │
│  └────┬─────┘  └─────┬──────┘  └──────┬───────┘  └────┬─────┘  │
│       │              │                │               │         │
└───────┼──────────────┼────────────────┼───────────────┼─────────┘
        │              │                │               │
    /camera/*    /perception/*    /terrain/*      /map, /tf
    /tf, /odom   /depth/*                         /cmd_vel
```

### Data Flow

```
Gazebo
  ├── /camera/image_raw        (RGB, BestEffort)
  ├── /camera/depth/image_raw  (32FC1 metres, BestEffort)
  ├── /camera/camera_info      (BestEffort)
  ├── /odometry/filtered       (wheel odom, BestEffort)
  └── /tf  (odom→base_footprint)
         │
         ▼
[depth_relay_node]  (camera_type:=sim)
  ├── /perception/depth/image_raw   (32FC1, RELIABLE) ──► RTAB-Map, point cloud
  ├── /perception/depth/camera_info (RELIABLE)        ──► RTAB-Map, terrain
  └── /perception/depth/colorized   (bgr8 TURBO)      ──► RViz RealDepth panel
         │
         ▼
[depth_to_pointcloud_node]
  └── /perception/depth/points  (PointCloud2, RELIABLE) ──► Nav2 ObstacleLayer
         │
[yolo_seg_node]
  ├── /perception/hazard_mask   (mono8: 0=safe,128=caution,255=lethal)
  └── /perception/yolo/overlay  (bgr8 visualization)
         │
[terrain_analysis_node]
  └── /terrain/traversability_grid  (OccupancyGrid)
         │
[rtabmap / rgbd_sync]
  ├── /map  (OccupancyGrid, SLAM map)
  └── /tf   (map→odom correction)
         │
[Nav2 stack]
  └── /cmd_vel ──► Gazebo DiffDrive
```

---

## Prerequisites

| Requirement | Version | Notes |
|---|---|---|
| Ubuntu | 24.04 LTS | Required for Gazebo Harmonic |
| ROS 2 | Jazzy | `ros-jazzy-desktop` |
| Gazebo | Harmonic | `gz-harmonic` |
| Python | 3.12 | Ships with Ubuntu 24.04 |
| NVIDIA GPU | Any CUDA 12+ | Optional; enables YOLO FP16 + DA V2 Metric |

### Required ROS 2 packages

```bash
sudo apt install -y \
  ros-jazzy-nav2-bringup \
  ros-jazzy-nav2-msgs \
  ros-jazzy-rtabmap-ros \
  ros-jazzy-robot-localization \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-ros-gz-sim \
  ros-jazzy-cv-bridge \
  ros-jazzy-sensor-msgs-py \
  ros-jazzy-xacro
```

### Required Python packages

```bash
pip install torch torchvision --extra-index-url https://download.pytorch.org/whl/cu124
pip install transformers ultralytics pillow opencv-python-headless
```

> The Depth Anything V2 Metric model (~300 MB) downloads automatically from HuggingFace on first run when `enable_depth_viz:=true`.

---

## Installation

```bash
# 1. Clone
git clone https://github.com/PrakashKumarSahu/Autonomous-UGV.git
cd Autonomous-UGV

# 2. Build
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install

# 3. Source
source install/setup.bash
```

> **Tip:** Add `source ~/Autonomous-UGV/install/setup.bash` to your `~/.bashrc`.

---

## Quick Start

### Default simulation (recommended)

```bash
ros2 launch ugv_bringup ugv_complete.launch.py
```

Launches Gazebo with `ugv_test_arena.sdf`, spawns the robot, starts all perception/SLAM/Nav2 nodes, and opens RViz.

### Set a navigation goal

In RViz, click **2D Nav Goal** (toolbar), then click + drag on the map to set the robot's target pose. Nav2 plans a collision-free path and drives to the goal.

### Custom world

```bash
ros2 launch ugv_bringup ugv_complete.launch.py \
  world:=/path/to/my_world.sdf \
  world_name:=my_world \
  spawn_x:=1.0 spawn_y:=2.0 spawn_yaw:=0.0
```

> `world_name` must match the `<world name="...">` attribute in your SDF file.

### Empty world (no obstacles)

```bash
ros2 launch ugv_bringup ugv_complete.launch.py \
  world:=$(ros2 pkg prefix ugv_sim)/share/ugv_sim/worlds/empty_world.sdf \
  world_name:=empty_world
```

---

## Launch Arguments

All arguments can be passed as `key:=value` on the command line.

| Argument | Default | Description |
|---|---|---|
| `mode` | `sim` | `sim` — Gazebo simulation. `hw` — Real hardware (skips Gazebo). |
| `use_sim_time` | `true` | Use `/clock` from Gazebo. Set `false` in `hw` mode. |
| `world` | `ugv_test_arena.sdf` | Full path to a Gazebo SDF world file. The world must NOT include the robot (it is spawned separately). |
| `world_name` | `world_demo` | Must match the `<world name="...">` attribute in the SDF. Used to construct GZ sensor topic paths. |
| `robot_name` | `localbot` | Gazebo entity name for the spawned robot. Also determines cmd_vel topic path. |
| `spawn_x/y/z` | `0.0 / -6.5 / 0.01` | Robot spawn position in world frame (metres). |
| `spawn_yaw` | `1.5708` | Robot spawn orientation (radians). `1.5708` = facing +Y (north). |
| `camera_type` | `sim` | Depth source. See [Camera / Depth Source Switching](#camera--depth-source-switching). |
| `enable_depth_viz` | `false` | Run Depth Anything V2 Metric as a side-by-side RViz AI depth overlay. |
| `enable_rtabmap` | `true` | Enable RTAB-Map visual SLAM. |
| `enable_rviz` | `true` | Launch RViz2 dashboard. |
| `rviz_config` | `ugv_autonomy.rviz` | Path to a custom RViz `.rviz` config file. |
| `nav2_params_file` | `nav2_params.yaml` | Path to custom Nav2 YAML. Override without rebuilding. |

---

## Modules

### `ugv_sim` — Simulation Environment

**Package type:** `ament_cmake`
**Launch:** `sim.launch.py` (included by `ugv_complete.launch.py`)

Responsibilities:
- Starts **Gazebo Harmonic** with the given world SDF
- Spawns **Localbot** from `models/localbot/model.sdf` (independent of the world file — zero fuel dependency)
- Dynamically generates the **ros\_gz\_bridge YAML** at runtime, parameterized on `world_name` and `robot_name`
- Starts **robot\_state\_publisher** (URDF → TF)
- Bridges wheel joint states with **joint\_state\_relay** (QoS fix: Gazebo BestEffort → RSP Reliable)

#### Localbot model

| Property | Value |
|---|---|
| Chassis | 0.60 × 0.40 × 0.20 m |
| Drive | Differential drive, wheel separation 0.45 m, radius 0.10 m |
| Camera | RGB-D camera at 0.43 m above chassis top (0.64 m total), FOV ~87° |
| RGB sensor | 848 × 480, 15 FPS |
| Depth sensor | 848 × 480, 15 FPS, range 0.3–10 m |
| IMU | 6-DoF (accel + gyro), 200 Hz |

#### Built-in worlds

| World | File | Description |
|---|---|---|
| Test arena | `ugv_test_arena.sdf` | Enclosed rectangular arena with walls, pillars, and obstacles |
| Empty | `empty_world.sdf` | Flat ground plane; bring your own SDF obstacles |

#### Customizing the bridge

The `gazebo_bridge.yaml` in `config/` is a **reference file** kept for documentation. The actual bridge configuration is generated dynamically at launch time from `_make_bridge_yaml()` in `sim.launch.py`. To add new bridged topics, edit that function.

---

### `ugv_description` — Robot Model & TF

**Package type:** `ament_cmake`
**Launch:** `robot_state_publisher.launch.py`

Provides the robot URDF definition and publishes the TF tree.

#### Frame tree

```
map (SLAM global frame)
└── odom (wheel odometry frame)
    └── base_footprint (2D ground projection)
        └── base_link (chassis center)
            ├── left_wheel_link / right_wheel_link
            ├── front_caster_wheel_link / rear_caster_wheel_link
            ├── imu_link
            └── camera_link
                └── camera_link_optical  ← all sensor data in this frame
```

#### URDF vs. model.sdf

The URDF (`ugv_base.urdf.xacro`) is used **only** by robot\_state\_publisher for TF and visualization. The Gazebo physics simulation uses `model.sdf` exclusively (spawned with `-file` flag). The `<gazebo>` tags inside the URDF are NOT processed in this setup.

---

### `ugv_perception` — AI Perception Stack

**Package type:** `ament_python`
**Launch:** `perception.launch.py` (included by `ugv_complete.launch.py`)

#### Canonical depth topic contract

All `camera_type` depth sources are **interchangeable** — they all publish the same three topics with the same QoS:

| Topic | Type | QoS | Consumer |
|---|---|---|---|
| `/perception/depth/image_raw` | `Image` (32FC1, metres) | RELIABLE | RTAB-Map, point cloud node |
| `/perception/depth/camera_info` | `CameraInfo` | RELIABLE | RTAB-Map, point cloud node, terrain node |
| `/perception/depth/colorized` | `Image` (bgr8 TURBO) | RELIABLE | RViz `RealDepth` panel |

Swapping depth sources requires only changing `camera_type` — nothing else in the pipeline changes.

#### Nodes

**`depth_relay_node`** (`camera_type:=sim`)
Bridges Gazebo's raw depth camera (BestEffort) to the canonical RELIABLE topics. Converts 32FC1 → TURBO bgr8 colormap for RViz. Handles missing depth camera\_info via fallback to RGB camera\_info.

**`depth_node`** (`camera_type:=monocular`)
Runs Depth Anything V2 Metric Indoor (HuggingFace `depth-anything/Depth-Anything-V2-Metric-Indoor-Small-hf`) on any RGB image. Publishes metric depth in metres. Falls back to a geometric horizon estimate if the model cannot load.

**`realsense_relay_node`** (`camera_type:=realsense`)
Converts RealSense 16UC1 depth (mm) → 32FC1 (m), clips to valid range, adds TURBO colorization.

**`zed_relay_node`** (`camera_type:=zed`)
Re-frames ZED 32FC1 depth (already in metres) and adds TURBO colorization.

**`depth_to_pointcloud_node`** (always active)
Converts `/perception/depth/image_raw` + `/perception/depth/camera_info` → `PointCloud2` for Nav2 ObstacleLayer. Uses every 4th pixel (configurable `step` parameter). Max depth capped at 10 m to match sensor specification and Nav2 obstacle range.

**`yolo_seg_node`** (always active)
Runs YOLO11n-seg on `/camera/image_raw`. Classifies detected segments:
- **Lethal (255):** class name contains a hazard keyword (`rock`, `stone`, `boulder`, `tree`, `car`, `person`, etc.)
- **Caution (128):** detected object with no keyword match (amber overlay in RViz)
- **Safe (0):** undetected / background regions

> Uses COCO-80 classes by default. COCO does not include `road/trail/grass`, so classification is purely keyword-based. To use outdoor-specific classes, train or fine-tune with a custom dataset.

#### Depth Anything V2 Metric — AI visualization

When `enable_depth_viz:=true`, an additional `depth_node` instance runs as a visualization overlay:

```bash
ros2 launch ugv_bringup ugv_complete.launch.py enable_depth_viz:=true
```

Publishes to `/perception/depth_ai/colorized` (shown in RViz **AIDepth** panel). Does NOT feed Nav2 or SLAM — purely visual. Requires ~2 GB VRAM.

---

### `ugv_terrain` — Traversability Mapping

**Package type:** `ament_python`
**Launch:** `terrain_mapping.launch.py`

**`terrain_analysis_node`:**
- Subscribes to `/perception/depth/points` and `/perception/hazard_mask`
- Auto-calibrates camera intrinsics from `/perception/depth/camera_info`
- Projects 3D points into a 12 × 12 m robot-centric grid at 0.1 m resolution
- Computes step height cost per cell (max − min elevation within cell)
- Burns YOLO lethal hazard pixels via geometric back-projection
- Publishes `/terrain/traversability_grid` (OccupancyGrid, frame: `base_footprint`)

> The traversability grid is currently for RViz visualization. Wiring it into Nav2 as a costmap layer is a planned improvement.

---

### `ugv_bringup` — SLAM, Nav2 & Orchestration

**Package type:** `ament_cmake`

Central package containing all launch files, configs, and the RViz config.

#### `localization.launch.py`

**`rgbd_sync`:** Synchronizes RGB + depth + camera\_info into `/rtabmap/rgbd_image` using approximate time sync (100 ms tolerance).

**`rtabmap`:** Visual SLAM — builds `/map`, publishes `map→odom` TF. Configured for planar 2D. Deletes its database on restart for a fresh map each run. EKF (`ekf_node`) runs **only** in `mode:=hw`.

#### `navigation.launch.py`

| Node | Role |
|---|---|
| `controller_server` | Regulated Pure Pursuit at 20 Hz |
| `planner_server` | SmacPlanner2D global path planner |
| `behavior_server` | Spin / BackUp / Wait recovery behaviors |
| `bt_navigator` | Behavior Tree goal orchestration |
| `lifecycle_manager_navigation` | Manages node lifecycles |

The robot declares itself stuck if it fails to move 0.30 m in 8 seconds, then attempts recovery (Spin → BackUp → Wait → abort goal).

#### `ugv_complete.launch.py`

Single entry point for the entire system. Launches all sub-modules in dependency order.

---

### `ugv_control` — Hardware ros2\_control (Future)

Contains ros2\_control spawner configuration for `joint_state_broadcaster` and `diff_drive_controller`. **Not launched** in the current system. Scaffold for future hardware integration when a physical robot with ros2\_control hardware interfaces is available.

---

## Configuration Reference

### `nav2_params.yaml`

`src/ugv_bringup/config/nav2_params.yaml`

| Parameter | Value | Effect |
|---|---|---|
| `movement_time_allowance` | 8.0 s | Declares stuck if <0.30 m progress in 8 s |
| `required_movement_radius` | 0.30 m | Minimum progress distance |
| `failure_tolerance` | 0.5 s | Controller error tolerance before recovery |
| `desired_linear_vel` | 0.4 m/s | Cruising speed |
| `use_collision_detection` | `true` | RPP stops if path will hit costmap obstacle |
| `inflation_radius` | 0.75 m | Safety margin around all obstacles |
| `transform_tolerance` | 1.0 s | Allows 1 s TF latency (RTAB-Map ~1 Hz) |
| `allow_unknown` | `true` | Planner can route through unmapped space |

### `rtabmap.yaml`

`src/ugv_bringup/config/rtabmap.yaml`

| Parameter | Value | Effect |
|---|---|---|
| `RGBD/LinearUpdate` | `0.0` | Keyframe on every frame (map published immediately) |
| `Grid/RangeMax` | `10.0` | Matches depth sensor max range |
| `Grid/CellSize` | `0.10` | 10 cm map resolution |
| `Reg/Force3DoF` | `true` | Planar SLAM |
| `wait_for_transform` | `1.0` | Generous TF wait during startup |

### `ekf.yaml` (hardware mode only)

`src/ugv_bringup/config/ekf.yaml`

Fuses wheel odometry + IMU yaw rate in hardware mode. Not active in simulation.

---

## RViz Dashboard

Pre-configured dashboard (`ugv_autonomy.rviz`) — Fixed Frame: `map`:

| Panel | Topic | Purpose |
|---|---|---|
| **Map** | `/map` | SLAM occupancy grid |
| **GlobalCostmap** | `/global_costmap/costmap` | Global planning space |
| **LocalCostmap** | `/local_costmap/costmap` | Local obstacle avoidance |
| **Path** | `/plan` | Planned global path |
| **RealDepth** | `/perception/depth/colorized` | Live colorized depth |
| **AIDepth** | `/perception/depth_ai/colorized` | DA V2 Metric overlay (if enabled) |
| **YOLO** | `/perception/yolo/overlay` | YOLO11 segmentation |
| **PointCloud** | `/perception/depth/points` | 3D obstacle cloud |
| **Traversability** | `/terrain/traversability_grid` | Rolling terrain cost grid |

---

## Worlds

### World file requirements

1. Must NOT contain the robot model (robot is spawned separately)
2. The `<world name="...">` attribute must match the `world_name` launch arg
3. Encode as UTF-8

### Using Gazebo Fuel worlds

```bash
# Download a Fuel world:
gz fuel download --url "https://fuel.gazebosim.org/1.0/OpenRobotics/worlds/Warehouse" -t world

# Launch with it:
ros2 launch ugv_bringup ugv_complete.launch.py \
  world:=~/.gz/fuel/fuel.gazebosim.org/openrobotics/worlds/warehouse/1/warehouse.sdf \
  world_name:=warehouse \
  spawn_x:=0.0 spawn_y:=0.0
```

---

## Camera / Depth Source Switching

### `camera_type:=sim` (default)

Gazebo built-in depth camera. Metric depth, 0.3–10 m.

### `camera_type:=monocular`

Depth Anything V2 Metric Indoor on any RGB camera. Requires GPU.

```bash
ros2 launch ugv_bringup ugv_complete.launch.py mode:=hw camera_type:=monocular use_sim_time:=false
```

Model auto-downloads (~300 MB) and caches in `~/.cache/huggingface/`.

### `camera_type:=realsense`

Requires `realsense2_camera` and Intel D435/D455.

```bash
ros2 launch realsense2_camera rs_launch.py &
ros2 launch ugv_bringup ugv_complete.launch.py mode:=hw camera_type:=realsense use_sim_time:=false
```

### `camera_type:=zed`

Requires ZED SDK and `zed-ros2-wrapper`.

```bash
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2 &
ros2 launch ugv_bringup ugv_complete.launch.py mode:=hw camera_type:=zed use_sim_time:=false
```

---

## AI Depth Visualization

Run Depth Anything V2 Metric alongside Gazebo for visual comparison:

```bash
ros2 launch ugv_bringup ugv_complete.launch.py enable_depth_viz:=true
```

Publishes AI depth to `/perception/depth_ai/colorized` (RViz **AIDepth** panel). Does NOT feed navigation. Requires ~2 GB VRAM.

---

## Key Topics

| Topic | Type | Publisher | Consumers |
|---|---|---|---|
| `/camera/image_raw` | `Image` (bgr8) | ros_gz_bridge | YOLO, RTAB-Map, depth\_node |
| `/camera/depth/image_raw` | `Image` (32FC1) | ros_gz_bridge | depth\_relay |
| `/odometry/filtered` | `Odometry` | ros_gz_bridge | RTAB-Map, Nav2 |
| `/perception/depth/image_raw` | `Image` (32FC1, RELIABLE) | depth relay | depth\_to\_pointcloud, RTAB-Map |
| `/perception/depth/camera_info` | `CameraInfo` (RELIABLE) | depth relay | depth\_to\_pointcloud, terrain |
| `/perception/depth/colorized` | `Image` (bgr8, RELIABLE) | depth relay | RViz RealDepth |
| `/perception/depth/points` | `PointCloud2` (RELIABLE) | depth\_to\_pointcloud | Nav2 ObstacleLayer |
| `/perception/hazard_mask` | `Image` (mono8) | yolo\_seg | terrain\_analysis |
| `/perception/yolo/overlay` | `Image` (bgr8) | yolo\_seg | RViz |
| `/terrain/traversability_grid` | `OccupancyGrid` | terrain\_analysis | RViz |
| `/map` | `OccupancyGrid` | RTAB-Map | Nav2 global\_costmap |
| `/cmd_vel` | `Twist` | Nav2 | Gazebo DiffDrive |

---

## TF Tree

```
map  ←────────────── RTAB-Map (map→odom correction)
│
└── odom  ←──────── Gazebo DiffDrive (continuous wheel odometry)
    │
    └── base_footprint
        │
        └── base_link
            ├── left_wheel_link
            ├── right_wheel_link
            ├── front_caster_wheel_link
            ├── rear_caster_wheel_link
            ├── imu_link
            └── camera_link
                └── camera_link_optical   ← all sensor data in this frame
```

---

## Known Limitations

1. **Traversability grid not in Nav2 costmap.** Computed at high frequency but not wired as a costmap layer. Currently visualization-only.

2. **YOLO uses COCO-80 classes.** No outdoor terrain classes in COCO. Hazard detection is keyword-based. Fine-tune on a domain-specific dataset for real field use.

3. **30 cm depth blind spot.** Gazebo depth camera near-clip is 0.3 m. Objects closer than 30 cm are invisible. The 0.75 m inflation radius mitigates this in practice.

4. **Monocular SLAM drift.** No stereo or IMU fusion in sim mode. Long featureless corridors cause drift; loop closure re-aligns.

5. **Camera URDF/SDF 15 mm offset.** Depth and RGB physical origins differ by 15 mm but share one URDF frame. Below the 10 cm map cell resolution — negligible impact.

6. **Hardware mode is a scaffold.** `ugv_control` (ros2\_control) is not fully wired. Perception and SLAM work in `mode:=hw`; motor control requires additional setup.

---

## Developer Guide

### Building a single package

```bash
colcon build --symlink-install --packages-select ugv_perception
source install/setup.bash
```

### Adding a new world

1. Create your SDF file (no robot model inside)
2. Set `<world name="your_world_name">` in the SDF
3. Launch with `world:=/path/to/world.sdf world_name:=your_world_name`

No code changes or rebuild needed.

### Adding a new depth source

1. Create `src/ugv_perception/ugv_perception/my_sensor_relay_node.py`
2. Publish to the three canonical topics (RELIABLE QoS):
   - `/perception/depth/image_raw` (32FC1, metres)
   - `/perception/depth/camera_info` (CameraInfo)
   - `/perception/depth/colorized` (bgr8 TURBO)
3. Add entry point in `src/ugv_perception/setup.py`
4. Add a conditional `Node(...)` block in `perception.launch.py`
5. Document the new `camera_type` value in `ugv_complete.launch.py`

### Modifying Nav2 behavior

Edit `src/ugv_bringup/config/nav2_params.yaml`. No rebuild needed.

- **Speed:** `desired_linear_vel` (controller) + `max_linear_velocity` (URDF DiffDrive)
- **Safety margin:** `inflation_radius`
- **Stuck detection:** `movement_time_allowance` / `required_movement_radius`
- **Override at launch:** `nav2_params_file:=/path/to/custom.yaml`

### Modifying RTAB-Map

Edit `src/ugv_bringup/config/rtabmap.yaml`. No rebuild needed.

To keep the map across restarts, set `delete_db_on_start: false` in `localization.launch.py`.

### Integrating traversability grid into Nav2

Add to `nav2_params.yaml` local\_costmap → plugins:

```yaml
plugins: ["obstacle_layer", "traversability_layer", "inflation_layer"]
traversability_layer:
  plugin: "nav2_costmap_2d::StaticLayer"
  map_topic: /terrain/traversability_grid
  subscribe_to_updates: true
```

(Also requires changing the terrain node's output frame from `base_footprint` to `odom`.)

---

## License

MIT License — see [LICENSE](LICENSE) for details.
