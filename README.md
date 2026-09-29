# Autonomous UGV — Complete System Documentation

> **Maintainer reference** — sufficient for a new developer to understand, build, run, debug, modify, and extend the entire project without external assistance.

---

## Table of Contents

1. [Project Overview](#1-project-overview)
2. [Repository Structure](#2-repository-structure)
3. [System Architecture](#3-system-architecture)
4. [Data Flow](#4-data-flow)
5. [TF Frame Tree](#5-tf-frame-tree)
6. [Topics, Services & Actions](#6-topics-services--actions)
7. [Packages & Nodes](#7-packages--nodes)
8. [Configuration Files](#8-configuration-files)
9. [Dependencies & Versions](#9-dependencies--versions)
10. [Build Instructions](#10-build-instructions)
11. [Launch Commands](#11-launch-commands)
12. [Simulation Setup](#12-simulation-setup)
13. [Depth Source Swapping (sim → production)](#13-depth-source-swapping-sim--production)
14. [Tuning & Parameters](#14-tuning--parameters)
15. [Troubleshooting & Common Failures](#15-troubleshooting--common-failures)
16. [Debugging Commands](#16-debugging-commands)
17. [How Components Interact](#17-how-components-interact)
18. [How to Safely Modify the System](#18-how-to-safely-modify-the-system)
19. [Known Limitations](#19-known-limitations)
20. [Bug History & Fixes](#20-bug-history--fixes)

---

## 1. Project Overview

A fully visual, camera-only Autonomous UGV (Unmanned Ground Vehicle) system built on ROS 2 Jazzy + Gazebo Harmonic. The robot navigates autonomously in a warehouse environment using:

- **Visual SLAM** (RTAB-Map) to build a 2D occupancy map from an RGB-D camera
- **Nav2** (Smac2D planner + Regulated Pure Pursuit controller) for path planning and execution
- **YOLO** instance segmentation for hazard detection
- **Depth camera** for real-time obstacle avoidance via Nav2 ObstacleLayer
- **No lidar** — 100% camera-based perception

**Design principle**: The Gazebo depth sensor is ground truth in simulation. In production, the depth source is swappable (RealSense, ZED, Depth Anything V3) without changing any other code.

---

## 2. Repository Structure

```
Autonomous-UGV/
├── src/
│   ├── ugv_bringup/          # Top-level launch, configs, RViz
│   │   ├── launch/
│   │   │   ├── ugv_complete.launch.py   # SINGLE ENTRY POINT — launches everything
│   │   │   ├── sim.launch.py            # (via ugv_sim) Gazebo + RSP + bridge
│   │   │   ├── perception.launch.py     # Depth relay + YOLO + point cloud
│   │   │   ├── localization.launch.py   # RTAB-Map SLAM + rgbd_sync + optional EKF
│   │   │   ├── navigation.launch.py     # Nav2 (controller, planner, BT, lifecycle)
│   │   │   └── rviz.launch.py           # RViz2 dashboard
│   │   ├── config/
│   │   │   ├── nav2_params.yaml         # All Nav2 parameters (costmaps, planner, controller)
│   │   │   ├── rtabmap.yaml             # RTAB-Map SLAM parameters
│   │   │   └── ekf.yaml                 # EKF (hw mode only; not used in sim)
│   │   └── rviz/
│   │       └── ugv_autonomy.rviz        # RViz config (fixed frame, displays, QoS)
│   │
│   ├── ugv_sim/              # Gazebo simulation
│   │   ├── launch/sim.launch.py         # Gazebo + RSP + bridge + joint relay
│   │   ├── config/gazebo_bridge.yaml    # ROS↔Gazebo topic mappings + QoS
│   │   ├── models/tugbot/
│   │   │   ├── model.config             # REQUIRED for model:// URI resolution
│   │   │   ├── model.sdf                # LOCAL modified Tugbot SDF (fixed light joint)
│   │   │   └── meshes/                  # Tugbot visual meshes
│   │   ├── worlds/tugbot_warehouse.sdf  # Warehouse world (uses model://tugbot)
│   │   └── scripts/joint_state_relay.py # Renames SDF joint names → URDF joint names
│   │
│   ├── ugv_description/      # Robot URDF model
│   │   ├── launch/robot_state_publisher.launch.py
│   │   └── urdf/
│   │       ├── ugv.urdf.xacro           # Top-level includes ugv_base + sensors
│   │       ├── ugv_base.urdf.xacro      # Base chassis, wheels, diff-drive plugin
│   │       └── sensors/
│   │           ├── camera.urdf.xacro    # Camera link + camera_link_optical
│   │           └── imu.urdf.xacro       # IMU link
│   │
│   ├── ugv_perception/       # Perception AI stack
│   │   └── ugv_perception/
│   │       ├── depth_relay_node.py      # BestEffort→Reliable QoS bridge for Gazebo depth
│   │       ├── depth_node.py            # Depth Anything V3 monocular depth
│   │       ├── depth_to_pointcloud_node.py  # (not launched; depth_image_proc used instead)
│   │       ├── hazard_mask_node.py      # Hazard mask fusion
│   │       ├── yolo_seg_node.py         # YOLOv8/YOLO11 instance segmentation
│   │       ├── realsense_relay_node.py  # RealSense D435/D455 depth relay
│   │       └── zed_relay_node.py        # ZED 2/ZED X depth relay
│   │
│   ├── ugv_terrain/          # 2.5D terrain traversability
│   │   └── ugv_terrain/
│   │       └── terrain_analysis_node.py # Fuses depth + YOLO into height map
│   │
│   └── ugv_control/          # (hw mode) ros2_control diff-drive
│       └── config/diff_drive_controller.yaml
│
├── README.md                 # This document
└── install/                  # colcon build output (not committed)
```

> [!IMPORTANT]
> The Tugbot model (`src/ugv_sim/models/tugbot/`) is loaded by Gazebo via `model://tugbot` using `GZ_SIM_RESOURCE_PATH`. **Never change** the `<uri>` in `tugbot_warehouse.sdf` back to the Fuel URL — it would bypass all local model fixes.

---

## 3. System Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                         GAZEBO HARMONIC                             │
│  ┌─────────────┐  ┌──────────────┐  ┌──────────────────────────┐   │
│  │  Warehouse  │  │  Tugbot SDF  │  │   GZ→ROS Bridge          │   │
│  │  World SDF  │  │  (local)     │  │   (ros_gz_bridge)        │   │
│  │             │  │  DiffDrive   │──│─→ /odometry/filtered     │   │
│  │             │  │  RGB Camera  │──│─→ /camera/image_raw      │   │
│  │             │  │  Depth Cam   │──│─→ /camera/depth/image_raw│   │
│  │             │  │  IMU         │──│─→ /imu/data              │   │
│  └─────────────┘  └──────────────┘  └──────────────────────────┘   │
└─────────────────────────────────────────────────────────────────────┘
                              │
              ┌───────────────┼────────────────────┐
              │               │                    │
    ┌─────────▼────┐ ┌────────▼───────┐  ┌────────▼──────────┐
    │  SLAM Stack  │ │ Perception AI  │  │  Nav2 Stack        │
    │              │ │                │  │                    │
    │ depth_relay  │ │ depth_relay    │  │ Controller Server  │
    │    ↓         │ │    ↓           │  │  (RPP controller)  │
    │ rgbd_sync    │ │ point_cloud_   │  │   ↓ /cmd_vel       │
    │    ↓         │ │ xyz_node       │  │ Planner Server     │
    │ rtabmap      │ │    ↓           │  │  (Smac2D)          │
    │  → /map      │ │ /perception/   │  │ BT Navigator       │
    │  → map→odom  │ │ depth/points   │  │ Lifecycle Manager  │
    │    TF        │ │                │  │                    │
    └──────────────┘ │ yolo_seg_node  │  │ Local Costmap      │
                     │  → /hazard_mask│  │  (ObstacleLayer)   │
                     │  → /yolo/      │  │ Global Costmap     │
                     │    overlay     │  │  (StaticLayer)     │
                     └────────────────┘  └────────────────────┘
```

---

## 4. Data Flow

### Sensor → Navigation pipeline

```
Gazebo depth_camera
  └──[GZ bridge, BestEffort]──→ /camera/depth/image_raw (32FC1)
       └──[depth_relay_node]──→ /perception/depth/image_raw (Reliable)
            ├──[point_cloud_xyz_node]──→ /perception/depth/points
            │    └──[Nav2 ObstacleLayer]──→ local_costmap obstacle cells
            └──[rgbd_sync]──┐
                            │ (synced with /camera/image_raw + /camera/camera_info)
                            ↓
                      /rtabmap/rgbd_image
                            │
                      [rtabmap SLAM]
                            ├──→ /map (OccupancyGrid, Transient Local, ~1Hz)
                            │    └──[Nav2 StaticLayer]──→ global_costmap walls
                            └──→ map→odom TF (~1Hz)

Gazebo DiffDrive
  └──[GZ bridge]──→ /odometry/filtered (nav_msgs/Odometry, ~100Hz)
                    └──[RTAB-Map odom input + Nav2 controller]

Nav2 BT Navigator
  └──→ /plan (Path)
       └──[RPP controller]──→ /cmd_vel (Twist)
            └──[GZ bridge]──→ /model/tugbot/cmd_vel → Gazebo DiffDrive
```

### Joint state flow (sim mode)

```
Gazebo JointStatePublisher
  └──[GZ bridge, BestEffort]──→ /joint_states
       └──[joint_state_relay.py]──→ /joint_states_urdf  (ONLY wheel joints)
            └──[robot_state_publisher]──→ /tf (base_footprint→base_link→camera_link...)
```

> [!WARNING]
> Gazebo publishes `wheel_left_joint` / `wheel_right_joint` (SDF names) but the URDF defines `left_wheel_joint` / `right_wheel_joint`. The relay node translates names AND filters out non-URDF joints (gripper, warnign_light). Without filtering, RSP logs "unknown joint" at 50 Hz, crashing the TF tree and causing RViz flickering.

---

## 5. TF Frame Tree

```
map
 └──[RTAB-Map, ~1 Hz]──→ odom
      └──[Gazebo DiffDrive bridge, ~20 Hz]──→ base_footprint
           └──[RSP, static]──→ base_link
                ├──[RSP, static]──→ left_wheel_link
                ├──[RSP, static]──→ right_wheel_link
                ├──[RSP, static]──→ front_caster_wheel_link
                ├──[RSP, static]──→ rear_caster_wheel_link
                └──[RSP, static]──→ camera_link
                      └──[RSP, static]──→ camera_link_optical
```

| Transform | Publisher | Rate | QoS |
|-----------|-----------|------|-----|
| `map → odom` | rtabmap | ~1 Hz | Reliable |
| `odom → base_footprint` | ros_gz_bridge | ~20 Hz | Reliable |
| `base_footprint → base_link` | robot_state_publisher | on joint update | Reliable |
| `base_link → camera_link` | robot_state_publisher | static | Transient Local |

> [!NOTE]
> The RViz **Fixed Frame** is set to `odom` (not `map`). `odom` is always available immediately; `map` only exists after RTAB-Map publishes its first keyframe. Once SLAM is running, the viewer can switch Fixed Frame to `map` to see the full SLAM coordinate system.

---

## 6. Topics, Services & Actions

### Core topics

| Topic | Type | Publisher | Subscriber(s) | QoS |
|-------|------|-----------|---------------|-----|
| `/camera/image_raw` | `sensor_msgs/Image` | ros_gz_bridge | rgbd_sync, yolo_seg_node | BestEffort |
| `/camera/depth/image_raw` | `sensor_msgs/Image` (32FC1) | ros_gz_bridge | depth_relay_node | BestEffort |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | ros_gz_bridge | rgbd_sync | BestEffort |
| `/perception/depth/image_raw` | `sensor_msgs/Image` (32FC1) | depth_relay_node | rgbd_sync, point_cloud_xyz_node | Reliable |
| `/perception/depth/camera_info` | `sensor_msgs/CameraInfo` | depth_relay_node | point_cloud_xyz_node, rtabmap | Reliable |
| `/perception/depth/points` | `sensor_msgs/PointCloud2` | point_cloud_xyz_node | Nav2 ObstacleLayer, terrain_analysis | Reliable |
| `/perception/yolo/overlay` | `sensor_msgs/Image` | yolo_seg_node | RViz | Reliable |
| `/perception/hazard_mask` | `sensor_msgs/Image` | yolo_seg_node | terrain_analysis_node | Reliable |
| `/rtabmap/rgbd_image` | `rtabmap_msgs/RGBDImage` | rgbd_sync | rtabmap | Reliable |
| `/map` | `nav_msgs/OccupancyGrid` | rtabmap | Nav2 StaticLayer, RViz | Reliable + TransientLocal |
| `/odometry/filtered` | `nav_msgs/Odometry` | ros_gz_bridge | rtabmap, Nav2 controller | Reliable |
| `/cmd_vel` | `geometry_msgs/Twist` | Nav2 controller | ros_gz_bridge | Reliable |
| `/joint_states` | `sensor_msgs/JointState` | ros_gz_bridge | joint_state_relay | BestEffort |
| `/joint_states_urdf` | `sensor_msgs/JointState` | joint_state_relay | robot_state_publisher | Reliable |
| `/tf` | `tf2_msgs/TFMessage` | ros_gz_bridge, rtabmap, RSP | all | Reliable |
| `/tf_static` | `tf2_msgs/TFMessage` | robot_state_publisher | all | Reliable + TransientLocal |
| `/plan` | `nav_msgs/Path` | Nav2 planner | RPP controller, RViz | Reliable |
| `/local_costmap/costmap` | `nav_msgs/OccupancyGrid` | Nav2 | RViz | Reliable + Volatile |
| `/global_costmap/costmap` | `nav_msgs/OccupancyGrid` | Nav2 | RViz | Reliable + Volatile |

### Nav2 actions (send goals via RViz or CLI)

| Action | Type | Server |
|--------|------|--------|
| `/navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | bt_navigator |
| `/navigate_through_poses` | `nav2_msgs/action/NavigateThroughPoses` | bt_navigator |
| `/compute_path_to_pose` | `nav2_msgs/action/ComputePathToPose` | planner_server |
| `/follow_path` | `nav2_msgs/action/FollowPath` | controller_server |

---

## 7. Packages & Nodes

### `ugv_sim` — Simulation infrastructure

| Node | Executable | Purpose |
|------|-----------|---------|
| `ros_gz_bridge` | `parameter_bridge` | Bridges GZ↔ROS topics (config: `gazebo_bridge.yaml`) |
| `joint_state_relay` | `joint_state_relay.py` | Renames SDF wheel joint names to URDF names; drops non-URDF joints |

**Gazebo model loading**: `GZ_SIM_RESOURCE_PATH` is set in `sim.launch.py` to `install/ugv_sim/share/ugv_sim/models:~/.gz/fuel/...`. The world SDF uses `<uri>model://tugbot</uri>` which resolves to the local modified `model.sdf`.

### `ugv_description` — Robot model

| Node | Purpose |
|------|---------|
| `robot_state_publisher` | Parses URDF, publishes TF for all fixed joints and rotating wheel joints |

RSP reads joint states from `/joint_states_urdf` (renamed from Gazebo's `/joint_states`).

### `ugv_perception` — Perception AI

| Node | Input | Output | Active when |
|------|-------|--------|-------------|
| `depth_relay_node` | `/camera/depth/image_raw` (BestEffort 32FC1) | `/perception/depth/image_raw` (Reliable) | `camera_type:=sim` |
| `depth_node` (depth_anything) | `/camera/image_raw` | `/perception/depth/image_raw` | `camera_type:=monocular` |
| `realsense_relay_node` | `/camera/camera/depth/image_rect_raw` | `/perception/depth/image_raw` | `camera_type:=realsense` |
| `zed_relay_node` | `/zed/zed_node/depth/depth_registered` | `/perception/depth/image_raw` | `camera_type:=zed` |
| `point_cloud_xyz_node` | `/perception/depth/image_raw` + camera_info | `/perception/depth/points` | always |
| `yolo_seg_node` | `/camera/image_raw` | `/perception/hazard_mask`, `/perception/yolo/overlay` | always |
| `depth_viz_node` | `/camera/image_raw` | `/perception/depth_ai/image_raw` | `enable_depth_viz:=true` |

> [!NOTE]
> Exactly ONE depth source node is active at a time (selected by `camera_type`). All four relay nodes are launched but only the matching one passes the `IfCondition` check. The shared nodes (`point_cloud_xyz_node`, `yolo_seg_node`) always run.

### `ugv_bringup` — SLAM & Navigation

| Node | Package | Purpose |
|------|---------|---------|
| `rgbd_sync` | `rtabmap_sync` | Synchronizes RGB + depth + camera_info → RGBDImage |
| `rtabmap` | `rtabmap_slam` | Visual SLAM: builds `/map`, publishes `map→odom` TF |
| `controller_server` | `nav2_controller` | RPP path following controller + local costmap |
| `planner_server` | `nav2_planner` | Smac2D global path planner + global costmap |
| `behaviors` | `nav2_behaviors` | Recovery behaviors (spin, backup, wait) |
| `bt_navigator` | `nav2_bt_navigator` | Behavior tree navigation orchestrator |
| `lifecycle_manager_navigation` | `nav2_lifecycle_manager` | Manages Nav2 node lifecycle |

### `ugv_terrain` — Terrain analysis

| Node | Input | Output |
|------|-------|--------|
| `terrain_analysis_node` | `/perception/depth/points`, `/perception/hazard_mask`, `/perception/depth/camera_info` | Height map + traversability grid |

---

## 8. Configuration Files

### `src/ugv_bringup/config/nav2_params.yaml`

Key parameters and their rationale:

```yaml
controller_server:
  failure_tolerance: 1.0          # 0.3 caused aborts from brief TF hiccups (sim clock)
  
  progress_checker:
    required_movement_radius: 0.15 # must move 15cm within movement_time_allowance
    movement_time_allowance: 25.0  # 25s before declaring stuck (allows replanning pauses)

  FollowPath:  # Regulated Pure Pursuit Controller
    desired_linear_vel: 0.5        # m/s — safe for warehouse
    transform_tolerance: 1.0       # must match costmap tolerance; RTAB-Map updates map@1Hz
    use_collision_detection: false  # rely on costmap; RPP collision check caused phantom stops
    use_rotate_to_heading: true    # rotate in place before path following

local_costmap:
  obstacle_layer:
    observation_persistence: 0.5  # 1.0 kept stale obstacles too long during turns
    obstacle_range: 6.0            # mark obstacles up to 6m
    raytrace_range: 7.0            # clear free space up to 7m

global_costmap:
  static_layer:
    map_subscribe_transient_local: true  # receives /map even after publisher starts
```

### `src/ugv_bringup/config/rtabmap.yaml`

```yaml
rtabmap:
  RGBD/LinearUpdate: "0.0"         # keyframe on every frame (not only when moving)
  RGBD/AngularUpdate: "0.0"        # same — ensures /map published immediately at startup
  Rtabmap/CreateIntermediateNodes: "true"  # more frequent map updates
  Reg/Force3DoF: "true"            # 2D SLAM for flat ground
  Grid/FromDepth: "true"           # build 2D map from depth camera
  Grid/CellSize: "0.10"            # 10cm resolution, matches global_costmap
  Vis/MinInliers: "15"             # minimum visual feature matches for keyframe
```

**Key launch-time params** (set in `localization.launch.py`, override yaml):
```
map_always_update: True   → republishes /map at ~1Hz even without new keyframes
delete_db_on_start: True  → fresh map each launch (set False for map reuse)
```

### `src/ugv_sim/config/gazebo_bridge.yaml`

Maps Gazebo topics to ROS topics. **Critical QoS rule**: All GZ→ROS bridges use BestEffort. Any ROS node subscribing directly to Gazebo-bridged topics must use BestEffort QoS or it will receive nothing.

| GZ topic | ROS topic | Direction |
|----------|-----------|-----------|
| `/world/world_demo/model/tugbot/link/camera_front/sensor/color/image` | `/camera/image_raw` | GZ→ROS |
| `/world/world_demo/model/tugbot/link/camera_front/sensor/depth/depth_image` | `/camera/depth/image_raw` | GZ→ROS |
| `/model/tugbot/odometry` | `/odometry/filtered` | GZ→ROS |
| `/model/tugbot/tf` | `/tf` | GZ→ROS |
| `/model/tugbot/cmd_vel` | `/cmd_vel` | ROS→GZ |
| `/world/world_demo/model/tugbot/joint_state` | `/joint_states` | GZ→ROS |

> [!CAUTION]
> Depth camera GZ topic is `depth_image` NOT `image`. Gazebo Harmonic depth cameras publish on `.../sensor/depth/depth_image`. The wrong topic `sensor/depth/image` is a silent failure (bridge connects to ghost topic, zero bytes received, SLAM never gets depth data).

### `src/ugv_bringup/rviz/ugv_autonomy.rviz`

Fixed Frame: `odom` (not `map`). Required QoS for image displays:

| Display | Topic | QoS |
|---------|-------|-----|
| RGBCamera | `/camera/image_raw` | BestEffort (Gazebo bridge) |
| RealDepth | `/perception/depth/image_raw` | Reliable (depth_relay_node) |
| YOLOHazardOverlay | `/perception/yolo/overlay` | Reliable (yolo_seg_node) |
| AIDepth | `/perception/depth_ai/image_raw` | Reliable (depth_node) |
| SLAMMap | `/map` | Reliable + Transient Local |
| LocalCostmap | `/local_costmap/costmap` | Reliable + Volatile |
| GlobalCostmap | `/global_costmap/costmap` | Reliable + Volatile |

---

## 9. Dependencies & Versions

| Dependency | Version | Purpose |
|------------|---------|---------|
| Ubuntu | 24.04 LTS | Base OS |
| ROS 2 | Jazzy Jalisco | Middleware |
| Gazebo | Harmonic (gz-sim 8) | Simulation |
| ros_gz_bridge | Jazzy | GZ↔ROS bridge |
| rtabmap_ros | ≥0.21 | Visual SLAM |
| nav2 | Jazzy | Navigation |
| depth_image_proc | Jazzy | Depth→PointCloud |
| ultralytics | ≥8.0 (YOLO11) | Object detection |
| robot_localization | Jazzy | EKF (hw mode only) |
| OpenCV | ≥4.8 | Image processing |

**GPU**: NVIDIA GPU required for YOLO. Without GPU, `yolo_seg_node` falls back to Canny edge detection. RTAB-Map and depth relay work on CPU.

---

## 10. Build Instructions

```bash
# Prerequisites: ROS 2 Jazzy sourced, Gazebo Harmonic installed

cd ~/Autonomous-UGV
source /opt/ros/jazzy/setup.bash

# Install ROS dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build all packages
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source the workspace
source install/setup.bash
```

> [!TIP]
> For faster iteration on a single package: `colcon build --packages-select ugv_bringup --cmake-args -DCMAKE_BUILD_TYPE=Release`

---

## 11. Launch Commands

### Simulation (primary mode)

```bash
source install/setup.bash
ros2 launch ugv_bringup ugv_complete.launch.py mode:=sim
```

**All launch arguments:**

| Argument | Default | Description |
|----------|---------|-------------|
| `mode` | `sim` | `sim` (Gazebo) or `hw` (real robot) |
| `use_sim_time` | `true` | Use `/clock` from Gazebo |
| `enable_rviz` | `true` | Launch RViz2 dashboard |
| `enable_rtabmap` | `true` | Enable SLAM |
| `camera_type` | `sim` | Depth source: `sim`, `monocular`, `realsense`, `zed` |
| `enable_depth_viz` | `false` | Run Depth Anything V3 as RViz overlay (GPU intensive) |

### Example variants

```bash
# Without RViz (headless / SSH)
ros2 launch ugv_bringup ugv_complete.launch.py mode:=sim enable_rviz:=false

# With Depth Anything V3 visualization overlay
ros2 launch ugv_bringup ugv_complete.launch.py mode:=sim enable_depth_viz:=true

# Hardware mode with RealSense
ros2 launch ugv_bringup ugv_complete.launch.py mode:=hw camera_type:=realsense

# Hardware mode with ZED
ros2 launch ugv_bringup ugv_complete.launch.py mode:=hw camera_type:=zed
```

### Sending navigation goals

```bash
# Via CLI (map-frame coordinates)
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  '{pose: {header: {frame_id: "map"}, pose: {position: {x: 5.0, y: -2.0, z: 0.0},
   orientation: {w: 1.0}}}}'

# Via RViz: use "Nav2 Goal" tool (2D goal pose button in toolbar)
```

---

## 12. Simulation Setup

### Startup sequence

The warehouse world + Tugbot take **30–60 seconds** to fully load. Expected startup timeline:

```
t=0s   → Gazebo starts, sim clock begins at 0
t=3s   → "Detected jump back in time" warnings (normal — sim clock sync)
t=10s  → Gazebo physics engine initialized
t=30s  → ogre2 renderer ready, cameras start publishing
t=35s  → /camera/image_raw appears, /camera/depth/image_raw appears
t=36s  → depth_relay_node bridges depth to /perception/depth/image_raw
t=37s  → rgbd_sync synchronizes first RGB-D pair
t=38s  → rtabmap creates first keyframe → /map published → Nav2 global costmap ready
t=60s  → System fully operational, Nav2 green in RViz
```

Do not test topics or RViz displays before 60 seconds — most will show "No data."

### Models loaded from Fuel (internet required first run)

The world SDF uses Fuel URIs for all models EXCEPT Tugbot. These are cached in `~/.gz/fuel/` on first run:
- Warehouse: `https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models/Warehouse`
- Charging station: `https://fuel.ignitionrobotics.org/1.0/MovAi/models/Tugbot-charging-station`
- Various carts, shelves

**Tugbot is loaded locally** via `model://tugbot` → `install/ugv_sim/share/ugv_sim/models/tugbot/`.

### Gazebo resource path

Set in `sim.launch.py`:
```python
GZ_SIM_RESOURCE_PATH = install/ugv_sim/share/ugv_sim/models : ~/.gz/fuel/...
```

The install path comes first so local models take priority over Fuel cache.

---

## 13. Depth Source Swapping (sim → production)

All depth sources publish to the same topic `/perception/depth/image_raw` with identical format (32FC1, metres, frame_id=`camera_link_optical`). Nothing else in the pipeline changes.

```
camera_type:=sim        → depth_relay_node (Gazebo depth_camera → Reliable bridge)
camera_type:=monocular  → depth_node (Depth Anything V3 from RGB camera)
camera_type:=realsense  → realsense_relay_node (RealSense D435/D455)
camera_type:=zed        → zed_relay_node (ZED 2 / ZED X)
```

**Production deployment steps:**
1. Connect camera hardware
2. Launch camera driver (e.g., `ros2 launch realsense2_camera rs_launch.py`)
3. Launch UGV stack: `ros2 launch ugv_bringup ugv_complete.launch.py mode:=hw camera_type:=realsense`
4. Verify `/perception/depth/image_raw` is publishing: `ros2 topic hz /perception/depth/image_raw`

> [!NOTE]
> `camera_type:=monocular` (Depth Anything V3) produces relative depth without metric scale. For metric-accurate obstacle avoidance at production, use `realsense` or `zed` which provide true metric depth.

---

## 14. Tuning & Parameters

### RTAB-Map SLAM

**Reduce memory usage** (for long sessions):
```yaml
Mem/RehearsalSimilarity: "0.60"  # merge similar nodes (lower = more aggressive)
Mem/MemoryThr: "300"              # max working memory nodes (add to yaml)
```

**More aggressive mapping** (if map is sparse):
```yaml
Vis/MinInliers: "10"   # lower = more keyframes (noisy), higher = fewer (sparser)
Grid/CellSize: "0.05"  # 5cm resolution (doubles memory usage)
```

**Reuse map across launches** (for persistent localization):
In `localization.launch.py`, set:
```python
'delete_db_on_start': False,  # keep previous map
```
Remove `arguments=['-d']` from rtabmap node.

### Nav2

**Robot is too cautious** (stops near obstacles that aren't real):
- Reduce `inflation_radius` in both costmaps (currently 0.55/0.60m)
- Increase `cost_scaling_factor` to make inflation steeper but shorter

**Robot doesn't reach goal** (path blocked):
- Check if `robot_radius` (0.30m) + `inflation_radius` (0.55m) = 0.85m is realistic
- The TugBot is ~0.5m wide; robot_radius=0.30m is correct
- If shelves are narrow, reduce inflation_radius to 0.40m

**Navigation is too slow**:
- Increase `desired_linear_vel` (currently 0.5 m/s, max DiffDrive allows 1.5 m/s)
- Increase `rotate_to_heading_angular_vel` (currently 0.8 rad/s)

---

## 15. Troubleshooting & Common Failures

### SLAM map not appearing in RViz

**Symptom**: SLAMMap display shows "No map received" or "Status: Warn"

**Diagnosis**:
```bash
ros2 topic echo /map --no-arr --once
ros2 topic info /map --verbose   # check Durability: TRANSIENT_LOCAL
```

**Causes & fixes**:
1. System < 60s old — wait
2. `/camera/depth/image_raw` not publishing → Gazebo not fully loaded yet
3. `/rtabmap/rgbd_image` not publishing → rgbd_sync not syncing → check QoS
4. `map_always_update` not set → `ros2 param get /rtabmap map_always_update` should be `True`
5. RViz map display using wrong QoS → ensure `Durability: Transient Local, Reliability: Reliable`

### Robot stops navigating mid-path

**Symptom**: Nav2 sends a goal, robot moves for a few seconds, then stops. RViz shows goal as "failed."

**Diagnosis**:
```bash
ros2 topic echo /diagnostics 2>/dev/null | grep -A3 "controller\|progress"
ros2 topic hz /map              # should be ~1 Hz
ros2 topic hz /odometry/filtered  # should be ~100 Hz
```

**Causes & fixes**:
1. TF failure during Gazebo sim-clock jump → `failure_tolerance: 1.0` handles this (current config)
2. Progress checker timeout → robot genuinely stuck; use recovery behaviors or change goal
3. Global costmap blocked → SLAM map has wrong obstacle at goal → clear map with `ros2 service call /rtabmap/reset std_srvs/srv/Empty`
4. Local costmap phantom obstacle → restart nav stack; point cloud data was noisy

### RViz robot model flickers white

**Symptom**: TugBot model blinks between colored and white repeatedly.

**Diagnosis**:
```bash
ros2 topic hz /joint_states_urdf   # should be ~50 Hz
ros2 topic echo /joint_states_urdf --no-arr --once | grep name  # should show only left/right wheel joints
```

**Root cause**: RSP receives unknown joint names → TF invalidated → RViz loses transforms.
**Fix status**: Already fixed — `joint_state_relay.py` filters to wheel joints only.

If flickering returns after model changes: check that `model://tugbot` is used (not Fuel URL), and local `model.sdf` `warnign_light_joint` is `type="fixed"`.

### Tugbot light is spinning

**Symptom**: Warning light on top of TugBot spins rapidly in Gazebo.

**Root cause**: `warnign_light_joint` was `revolute` with `JointController` at 10 rad/s. Fixed to `type="fixed"`.

**Verification**:
```bash
grep "warnign_light_joint" install/ugv_sim/share/ugv_sim/models/tugbot/model.sdf
# Must show: type="fixed"
grep "uri" install/ugv_sim/share/ugv_sim/worlds/tugbot_warehouse.sdf | grep tugbot
# Must show: model://tugbot (NOT fuel.ignitionrobotics.org URL)
```

**If light still spins**: The world is using the Fuel-cached model. Verify `tugbot_warehouse.sdf` has `<uri>model://tugbot</uri>`, rebuild, and relaunch.

### Depth camera shows black image in RViz

**Symptom**: RealDepth panel shows all-black image.

**Cause**: 32FC1 depth images contain float32 metre values. RViz `Image` display must have `Normalize Range: true`, `Min: 0`, `Max: 10`.

**Verify in RViz**: Click the RealDepth display → expand properties → check Min Value=0, Max Value=10, Normalize Range=true.

### "No Image" for RGB camera in RViz

**Cause 1**: Gazebo not fully loaded yet (< 60s). Wait.
**Cause 2**: QoS mismatch — `/camera/image_raw` is BestEffort but RViz configured Reliable.
**Fix**: RViz RGBCamera display must use `Reliability Policy: Best Effort`.

### Nav2 not finding a path

**Symptom**: Goal sent, immediately fails with "PLANNING_FAILED."

**Diagnosis**:
```bash
ros2 topic echo /global_costmap/costmap --no-arr --once | grep -E "width|height"
# width/height=0 means costmap not initialized
ros2 topic echo /map --no-arr --once  # check SLAM map exists
```

**Cause**: Global costmap's static layer hasn't received `/map` yet. Wait for SLAM map, or check RTAB-Map is running.

---

## 16. Debugging Commands

```bash
# After launch: source install/setup.bash

# Check all expected topics are alive
ros2 topic list | grep -E "camera|depth|map|yolo|rtabmap|odom|cmd_vel|costmap|joint"

# Check topic rates (run in separate terminal)
ros2 topic hz /camera/image_raw              # expect ~15 Hz
ros2 topic hz /camera/depth/image_raw        # expect ~15 Hz
ros2 topic hz /perception/depth/image_raw    # expect ~15 Hz
ros2 topic hz /perception/depth/points       # expect ~15 Hz
ros2 topic hz /rtabmap/rgbd_image            # expect ~5-10 Hz
ros2 topic hz /map                           # expect ~1 Hz (Transient Local — may show 0)
ros2 topic hz /odometry/filtered             # expect ~100 Hz
ros2 topic hz /cmd_vel                       # non-zero when navigating

# Check single message on map (respects Transient Local, shows cached message)
ros2 topic echo /map --no-arr --once

# Verify QoS policies match publisher↔subscriber
ros2 topic info /map --verbose
ros2 topic info /camera/image_raw --verbose
ros2 topic info /perception/depth/image_raw --verbose

# Check TF tree
ros2 run tf2_tools view_frames  # generates frames.pdf

# Check specific transform
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_footprint

# Check Nav2 lifecycle state
ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server
ros2 lifecycle get /bt_navigator

# Check RTAB-Map params
ros2 param get /rtabmap map_always_update  # should be True
ros2 param get /rtabmap RGBD/LinearUpdate  # should be 0.0

# Check joint states (should only have 2 wheel joints)
ros2 topic echo /joint_states_urdf --no-arr --once

# Reset SLAM map (resets pose graph and rebuilds from scratch)
ros2 service call /rtabmap/reset std_srvs/srv/Empty

# Manual navigation goal via CLI
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  '{pose: {header: {frame_id: "map"}, pose: {position: {x: 5.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}'

# Check Gazebo model URI (verify local model is used, not Fuel URL)
grep "<uri>" install/ugv_sim/share/ugv_sim/worlds/tugbot_warehouse.sdf | grep tugbot
# Expected: <uri>model://tugbot</uri>

# Kill everything cleanly
pkill -9 -f "gz sim|ros2 launch|parameter_bridge|rtabmap|rviz2|yolo_seg|depth_relay"
```

---

## 17. How Components Interact

### Startup dependency chain

```
Gazebo → bridge → /camera/image_raw, /camera/depth/image_raw
                              ↓
depth_relay_node → /perception/depth/image_raw
                              ↓
rgbd_sync (syncs with /camera/camera_info) → /rtabmap/rgbd_image
                              ↓
rtabmap → /map (TransientLocal) AND map→odom TF
                              ↓
Nav2 static_layer (global_costmap) receives /map
Nav2 lifecycle_manager activates all Nav2 nodes
                              ↓
Nav2 ready for goals
```

**Bottleneck**: The entire Nav2 stack is ready only after `/map` publishes. With `map_always_update: True` and `RGBD/LinearUpdate: "0.0"`, RTAB-Map publishes within seconds of cameras coming online.

### cmd_vel flow and latency

```
Nav2 RPP Controller (20 Hz loop)
  → /cmd_vel (Reliable)
  → ros_gz_bridge (ROS→GZ)
  → /model/tugbot/cmd_vel (Gazebo topic)
  → Tugbot DiffDrive plugin (physics step, default 1ms)
  → robot moves
  → Gazebo odom published
  → ros_gz_bridge (GZ→ROS)
  → /odometry/filtered (Reliable, ~20 Hz)
  → Nav2 controller reads new robot pose
```

**Total latency**: ~50ms (controller loop 50ms + bridge latency <5ms + Gazebo step). At 0.5 m/s, the robot travels ~2.5cm per control cycle.

### How SLAM and Nav2 share the map

RTAB-Map publishes `/map` with `Transient Local` QoS. Nav2's `StaticLayer` subscribes with `map_subscribe_transient_local: true`. Late-joining (Nav2 starting after RTAB-Map) always receives the latest cached map.

RTAB-Map also publishes `map→odom` TF. Nav2 uses this TF to transform the global costmap (in `map` frame) into the `odom` frame for the controller. Updates at ~1 Hz — this is why `transform_tolerance: 1.0` is required in the controller (not 0.5).

---

## 18. How to Safely Modify the System

### Adding a new sensor

1. Add sensor to `ugv_base.urdf.xacro` (URDF link + joint)
2. Add sensor to Tugbot `model.sdf` (Gazebo sensor definition)
3. Add bridge entry to `gazebo_bridge.yaml`
4. Add sensor topic to `perception.launch.py` or a new launch file

### Changing the SLAM algorithm

RTAB-Map is launched via `localization.launch.py`. To replace with a different SLAM:
1. Remove `rgbd_sync_node` and `rtabmap_slam_node`
2. Add new SLAM node
3. Ensure it publishes `map→odom` TF and `/map` (OccupancyGrid)
4. Keep all Nav2 configuration unchanged

### Adding a Nav2 recovery behavior

In `nav2_params.yaml`, add to:
```yaml
behaviors:
  behavior_plugins: ["spin", "backup", "wait", "your_behavior"]
  your_behavior:
    plugin: "nav2_behaviors::YourBehavior"
```

Also add to lifecycle_nodes in `navigation.launch.py` if it's a separate node.

### Changing the world

1. Replace/modify `src/ugv_sim/worlds/tugbot_warehouse.sdf`
2. Update spawn pose for Tugbot (`<pose>` inside `<include>`)
3. If adding dynamic models, add bridge entries in `gazebo_bridge.yaml`
4. Rebuild: `colcon build --packages-select ugv_sim`

### Modifying the Tugbot model

**Always edit `src/ugv_sim/models/tugbot/model.sdf`**, then rebuild. Never edit the Fuel cache.

After editing:
```bash
colcon build --packages-select ugv_sim
# Verify change propagated:
grep "your_change" install/ugv_sim/share/ugv_sim/models/tugbot/model.sdf
```

> [!CAUTION]
> Never change `<uri>model://tugbot</uri>` in the world SDF back to the Fuel URL. The Fuel URL bypasses all local model modifications.

---

## 19. Known Limitations

| Limitation | Impact | Workaround |
|------------|--------|------------|
| Single front-facing camera | Can't see obstacles to sides/rear | Reduce speed near walls; larger inflation radius |
| SLAM requires texture | Featureless white walls confuse RTAB-Map | Minimum `Vis/MinInliers: "15"` avoids false matches |
| map→odom TF at ~1 Hz | Global costmap stale between updates | `transform_tolerance: 1.0` in all Nav2 components |
| Depth Anything V3 is relative | No metric scale for obstacle distances | Use `camera_type:=realsense` or `zed` in production |
| Gazebo startup takes 30-60s | Camera data unavailable early | Wait before testing; progress_checker handles this |
| No 3D obstacle handling | Can only avoid obstacles at camera height | `min_obstacle_height: 0.05`, `max_obstacle_height: 2.5` approximate this |
| YOLO falls back to Canny | Without GPU, no semantic hazard detection | YOLO fallback still provides basic edge-based detection |

---

## 20. Bug History & Fixes

| Commit | Bug | Root Cause | Fix |
|--------|-----|-----------|-----|
| `57e33de` | SLAM never gets depth data | Gazebo depth topic is `depth_image` not `image`; bridge connected to ghost topic | Fixed GZ topic name in `gazebo_bridge.yaml` |
| `1119abf` | rgbd_sync receives no images | `qos=0` (Reliable) but Gazebo bridge publishes BestEffort; incompatible in DDS | Changed to `qos=1` (SENSOR_DATA/BestEffort) |
| `ce3f06c` | YOLO hazard back-projection wrong | Hardcoded `fx=616.0` in terrain_analysis; Gazebo cam has `fx≈421.62` | Subscribe to `/perception/depth/camera_info` for live intrinsics |
| `ce3f06c` | YOLO fallback never called | `fallback_hazard_detection()` existed but `else` branch filled zeros | Call fallback in else branch |
| `e326bfb` | RViz fixed frame wrong | `Fixed Frame: map` but `map` doesn't exist until RTAB-Map builds it | Changed to `Fixed Frame: odom` |
| `e326bfb` | YOLO overlay not shown | RViz subscribed BestEffort but yolo_seg_node publishes Reliable | Fixed RViz QoS to Reliable |
| `219b04e` | Tugbot light spins fast | `warnign_light_joint` revolute at 10 rad/s via JointController | Changed to `type="fixed"`, removed JointController plugin |
| `219b04e` | RViz robot blinks white | joint_state_relay forwarded non-URDF joints (gripper, light) to RSP; RSP "unknown joint" errors crashed TF at 50Hz | Filter relay to wheel joints only |
| `3f856dd` | SLAM map "No map received" | RTAB-Map only publishes `/map` after robot moves (needs keyframe); RViz shows blank | Set `RGBD/LinearUpdate: "0.0"`, `map_always_update: True` |
| `(current)` | **All local model.sdf fixes ignored** | World SDF used Fuel URL for Tugbot, bypassing local model directory | Changed URI to `model://tugbot`, added `model.config` |
| `(current)` | **Robot stops navigating mid-path** | `failure_tolerance: 0.3` caused abort on any 0.3s TF hiccup (sim clock); RPP collision detection caused phantom stops | `failure_tolerance: 1.0`, `use_collision_detection: false`, `transform_tolerance: 1.0` |
| `(current)` | Stale obstacles block path during turns | `observation_persistence: 1.0` kept obstacles 1s after camera moved away | Reduced to `0.5` |
