# Autonomous UGV — System Documentation

> **Long-term maintenance reference** — sufficient for a new developer to understand, build, run, debug, modify, and extend the entire project without external assistance.

---

## Table of Contents

**Quick Reference**
1. [Build & Launch](#1-build--launch)
2. [Send a Navigation Goal](#2-send-a-navigation-goal)

**Architecture**
3. [Module Map](#3-module-map)
4. [Data Flow](#4-data-flow)
5. [TF Frame Tree](#5-tf-frame-tree)
6. [Topic Reference](#6-topic-reference)

**Module Documentation**
7. [Module: Simulation (ugv_sim)](#7-module-simulation-ugv_sim)
8. [Module: Robot Description (ugv_description)](#8-module-robot-description-ugv_description)
9. [Module: Perception (ugv_perception)](#9-module-perception-ugv_perception)
10. [Module: SLAM (localization.launch.py)](#10-module-slam-localizationlaunchpy)
11. [Module: Navigation (navigation.launch.py)](#11-module-navigation-navigationlaunchpy)
12. [Module: Terrain Analysis (ugv_terrain)](#12-module-terrain-analysis-ugv_terrain)
13. [Module: Visualization (ugv_bringup rviz)](#13-module-visualization-ugv_bringup-rviz)

**Operations**
14. [Configuration Reference](#14-configuration-reference)
15. [Depth Source Swapping (sim → production)](#15-depth-source-swapping-sim--production)
16. [Troubleshooting](#16-troubleshooting)
17. [Debugging Commands](#17-debugging-commands)
18. [How to Safely Modify Each Module](#18-how-to-safely-modify-each-module)
19. [Known Limitations](#19-known-limitations)
20. [Bug & Change History](#20-bug--change-history)

---

## 1. Build & Launch

```bash
cd ~/Autonomous-UGV
source /opt/ros/jazzy/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

# Launch everything (Gazebo + perception + SLAM + Nav2 + RViz)
ros2 launch ugv_bringup ugv_complete.launch.py mode:=sim
```

**Startup time**: Gazebo + renderer take **30–60 s** to initialize. Cameras appear at ~t=35s, SLAM map at ~t=40s, Nav2 ready at ~t=60s.

**All launch arguments:**

| Argument | Default | Description |
|----------|---------|-------------|
| `mode` | `sim` | `sim` (Gazebo) \| `hw` (real robot) |
| `camera_type` | `sim` | Depth source: `sim` \| `monocular` \| `realsense` \| `zed` |
| `enable_rviz` | `true` | Launch RViz2 visualization |
| `enable_rtabmap` | `true` | Enable RTAB-Map SLAM |
| `enable_depth_viz` | `false` | Run Depth Anything V3 as RViz overlay |
| `use_sim_time` | `true` | Use Gazebo `/clock` |

---

## 2. Send a Navigation Goal

```bash
# Via RViz: use the "Nav2 Goal" (2D pose estimate) tool in the toolbar.

# Via CLI:
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  '{pose: {header: {frame_id: "map"},
    pose: {position: {x: 5.0, y: -2.0, z: 0.0}, orientation: {w: 1.0}}}}'

# Reset SLAM map:
ros2 service call /rtabmap/reset std_srvs/srv/Empty
```

---

## 3. Module Map

Each module has a **clearly defined interface**: the topics it provides and requires. Modules are independently maintainable — to work on any module, read only its section below.

```
┌─────────────────────────────────────────────────────────────────────────┐
│                          SIMULATION MODULE                              │
│  ugv_sim package                                                        │
│  ┌──────────────┐  ┌─────────────────┐  ┌────────────────────────────┐ │
│  │ Gazebo World │  │  Localbot Model │  │   ros_gz_bridge            │ │
│  │ (warehouse)  │  │  (model.sdf)    │  │   gazebo_bridge.yaml       │ │
│  │ Fuel models  │  │  DiffDrive      │──│─→ /camera/* /imu/* /tf     │ │
│  │ (Warehouse,  │  │  RGB Camera     │──│─→ /camera/image_raw        │ │
│  │  shelves...) │  │  Depth Camera   │──│─→ /camera/depth/image_raw  │ │
│  └──────────────┘  │  IMU            │──│─→ /odometry/filtered       │ │
│                    │  JointState     │──│─→ /joint_states            │ │
│                    └─────────────────┘  │                            │ │
│                                         │  ← /cmd_vel (ROS→GZ)      │ │
│                                         └────────────────────────────┘ │
│                    ┌─────────────────────────────────────────────────┐  │
│                    │  joint_state_relay.py                           │  │
│                    │  /joint_states (BestEffort) → /joint_states_urdf│  │
│                    │  QoS bridge only — no name remapping (Localbot) │  │
│                    └─────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────────────┘
         │                    │                          │
         ▼                    ▼                          ▼
┌─────────────────┐  ┌──────────────────────┐  ┌──────────────────────────┐
│ ROBOT DESCRIPTION│  │   PERCEPTION MODULE  │  │      SLAM MODULE         │
│  ugv_description │  │   ugv_perception     │  │  localization.launch.py  │
│                  │  │                      │  │                          │
│  URDF + RSP      │  │  depth_relay_node    │  │  rgbd_sync               │
│  → /tf (static)  │  │  → /perception/depth │  │  → /rtabmap/rgbd_image   │
│  → /tf_static    │  │                      │  │                          │
│                  │  │  point_cloud_xyz_node│  │  rtabmap SLAM            │
│  Requires:       │  │  → /perception/depth │  │  → /map (Transient Local)│
│  /joint_states   │  │    /points           │  │  → map→odom TF (~1 Hz)   │
│  _urdf           │  │                      │  │                          │
│                  │  │  yolo_seg_node       │  │  Requires:               │
│                  │  │  → /perception/yolo  │  │  /rtabmap/rgbd_image     │
│                  │  │    /overlay          │  │  /odometry/filtered      │
│                  │  │  → /perception/      │  │                          │
│                  │  │    hazard_mask       │  │                          │
└─────────────────┘  └──────────────────────┘  └──────────────────────────┘
                              │                          │
                              ▼                          ▼
                    ┌──────────────────────────────────────────────────────┐
                    │              NAVIGATION MODULE                        │
                    │           navigation.launch.py                        │
                    │                                                        │
                    │  Local Costmap (ObstacleLayer on /perception/depth/   │
                    │    points + InflationLayer)                           │
                    │  Global Costmap (StaticLayer on /map)                │
                    │  Smac2D Planner → /plan                              │
                    │  RPP Controller → /cmd_vel                           │
                    │  BT Navigator (lifecycle orchestrator)               │
                    └──────────────────────────────────────────────────────┘
```

---

## 4. Data Flow

### Sensor → Navigation (complete pipeline)

```
Gazebo depth_camera (30 Hz, 32FC1 metres)
  └──[GZ bridge, BestEffort]──→ /camera/depth/image_raw
       ├──[depth_relay_node]──→ /perception/depth/image_raw (Reliable)
       │    ├──[point_cloud_xyz_node]──→ /perception/depth/points
       │    │    └──[Nav2 ObstacleLayer]──→ local_costmap obstacle cells
       │    └──[rgbd_sync]──┐
       │                    ├── (+ /camera/image_raw + /camera/camera_info)
       │                    ▼
       │              /rtabmap/rgbd_image
       │                    │
       │              [rtabmap SLAM]
       │                    ├──→ /map (Transient Local, ~1 Hz)
       │                    │    └──[Nav2 StaticLayer]──→ global_costmap walls
       │                    └──→ map→odom TF
       │
       └──[Nav2 Controller (RPP)]──→ /cmd_vel (20 Hz)
                                         │
Gazebo DiffDrive ←─────────────────────[GZ bridge]
  └──→ /odometry/filtered (20 Hz) ──→ Nav2 controller + RTAB-Map

/cmd_vel ──→ Gazebo DiffDrive ──→ robot moves ──→ new odometry ──→ (loop)
```

### Joint state flow (Localbot)

```
Gazebo JointStatePublisher
  → /joint_states (BestEffort) [only: left_wheel_joint, right_wheel_joint]
  → [joint_state_relay] QoS bridge (BestEffort → Reliable)
  → /joint_states_urdf
  → [robot_state_publisher] publishes wheel TF to /tf
```

> [!NOTE]
> With Localbot, joint names from Gazebo already match URDF names. The relay does **only QoS bridging** — no name remapping. This is a significant simplification vs. the TugBot which required `wheel_left_joint` → `left_wheel_joint` translation.

---

## 5. TF Frame Tree

```
map
 └──[rtabmap, ~1 Hz]──→ odom
      └──[GZ bridge DiffDrive, ~20 Hz]──→ base_footprint
           └──[RSP static]──→ base_link
                ├──[RSP static + joint_states_urdf]──→ left_wheel_link
                ├──[RSP static + joint_states_urdf]──→ right_wheel_link
                ├──[RSP static]──→ front_caster_wheel_link
                ├──[RSP static]──→ rear_caster_wheel_link
                ├──[RSP static]──→ imu_link
                └──[RSP static]──→ camera_link
                      └──[RSP static]──→ camera_link_optical
```

> [!IMPORTANT]
> RViz Fixed Frame is set to `odom`. The `odom` frame exists immediately from the first Gazebo physics step. `map` only appears after RTAB-Map builds its first keyframe (~40s after launch). Switching Fixed Frame to `map` gives the SLAM coordinate system view.

---

## 6. Topic Reference

| Topic | Type | Publisher | QoS | Consumer(s) |
|-------|------|-----------|-----|------------|
| `/camera/image_raw` | Image | GZ bridge | BestEffort | rgbd_sync, yolo_seg |
| `/camera/camera_info` | CameraInfo | GZ bridge | BestEffort | rgbd_sync |
| `/camera/depth/image_raw` | Image (32FC1) | GZ bridge | BestEffort | depth_relay_node |
| `/camera/depth/camera_info` | CameraInfo | GZ bridge | BestEffort | depth_relay_node |
| `/perception/depth/image_raw` | Image (32FC1) | depth_relay_node | Reliable | rgbd_sync, point_cloud |
| `/perception/depth/camera_info` | CameraInfo | depth_relay_node | Reliable | point_cloud, rtabmap |
| `/perception/depth/points` | PointCloud2 | point_cloud_xyz | Reliable | Nav2 ObstacleLayer |
| `/perception/yolo/overlay` | Image | yolo_seg_node | Reliable | RViz |
| `/perception/hazard_mask` | Image | yolo_seg_node | Reliable | terrain_analysis |
| `/rtabmap/rgbd_image` | RGBDImage | rgbd_sync | Reliable | rtabmap |
| `/map` | OccupancyGrid | rtabmap | Reliable + TransientLocal | Nav2 StaticLayer, RViz |
| `/odometry/filtered` | Odometry | GZ bridge | Reliable | rtabmap, Nav2 |
| `/cmd_vel` | Twist | Nav2 controller | Reliable | GZ bridge → Localbot |
| `/joint_states` | JointState | GZ bridge | BestEffort | joint_state_relay |
| `/joint_states_urdf` | JointState | joint_state_relay | Reliable | robot_state_publisher |
| `/tf` | TFMessage | GZ bridge + rtabmap + RSP | Reliable | all |
| `/tf_static` | TFMessage | RSP | TransientLocal | all |
| `/plan` | Path | Nav2 planner | Reliable | controller, RViz |
| `/local_costmap/costmap` | OccupancyGrid | Nav2 | Reliable + Volatile | RViz |
| `/global_costmap/costmap` | OccupancyGrid | Nav2 | Reliable + Volatile | RViz |

> [!CAUTION]
> **QoS mismatch = silent data loss in DDS.** Reliable publisher + BestEffort subscriber = 0 bytes received. Always check QoS with `ros2 topic info <topic> --verbose` when debugging missing data.

---

## 7. Module: Simulation (ugv_sim)

**What it provides**: Physical simulation of the robot and warehouse environment, sensor data, wheel odometry, TF, cmd_vel actuation.

**Output topics**: `/camera/image_raw`, `/camera/camera_info`, `/camera/depth/image_raw`, `/camera/depth/camera_info`, `/imu/data`, `/odometry/filtered`, `/tf` (odom→base_footprint), `/joint_states`, `/clock`

**Input topics**: `/cmd_vel`

### Localbot Robot Model

`src/ugv_sim/models/localbot/` — the project-native robot. **Zero Fuel/internet dependency.**

| Component | Value | Matches URDF? |
|-----------|-------|---------------|
| Chassis | 0.6 × 0.4 × 0.2 m | ✅ exact |
| Wheel radius | 0.1 m | ✅ exact (TugBot was 0.195 m) |
| Wheel separation | 0.45 m | ✅ exact (TugBot was 0.5605 m) |
| Left wheel joint | `left_wheel_joint` | ✅ exact (TugBot was `wheel_left_joint`) |
| Right wheel joint | `right_wheel_joint` | ✅ exact (TugBot was `wheel_right_joint`) |
| Camera position | 0.0553, 0, 0.4323 from base_link | ✅ exact |
| Camera link name | `camera_link` | ✅ exact |
| Sensors | RGB cam + depth cam + IMU | ✅ only used sensors |

**Files:**

| File | Purpose |
|------|---------|
| `models/localbot/model.config` | Required for `model://localbot` URI resolution |
| `models/localbot/model.sdf` | Complete robot SDF (geometry + sensors + plugins) |
| `worlds/tugbot_warehouse.sdf` | World: warehouse + Localbot spawn |
| `config/gazebo_bridge.yaml` | ROS↔GZ topic mappings + QoS |
| `scripts/joint_state_relay.py` | QoS bridge for joint states (BestEffort→Reliable) |
| `launch/sim.launch.py` | Launches: Gazebo + RSP + bridge + joint relay |

### Spawning in a Different World

To use Localbot in any other world:
```xml
<!-- In your world SDF -->
<include>
  <uri>model://localbot</uri>
  <name>localbot</name>
  <pose>0 0 0.01 0 0 0</pose>
</include>
```

Make sure `GZ_SIM_RESOURCE_PATH` includes the `ugv_sim` install models directory:
```bash
export GZ_SIM_RESOURCE_PATH=$(ros2 pkg prefix ugv_sim)/share/ugv_sim/models:$GZ_SIM_RESOURCE_PATH
```

Or simply launch via `sim.launch.py` (sets this automatically).

### How GZ_SIM_RESOURCE_PATH works

Set in `sim.launch.py`:
```
install/ugv_sim/share/ugv_sim/models : ~/.gz/fuel/...
```
Local install path comes **first** so Localbot takes precedence over any Fuel cache. The `model://localbot` URI resolves to:
```
install/ugv_sim/share/ugv_sim/models/localbot/model.sdf
```

> [!CAUTION]
> **Never replace `<uri>model://localbot</uri>` with a Fuel URL.** Fuel URLs bypass GZ_SIM_RESOURCE_PATH entirely — all local SDF changes become invisible to Gazebo.

---

## 8. Module: Robot Description (ugv_description)

**What it provides**: URDF robot model (for TF + RViz visualization), TF static transforms.

**Output topics**: `/tf` (static transforms for all fixed links), `/tf_static`

**Input topics**: `/joint_states_urdf` (from joint_state_relay)

**Files:**

| File | Purpose |
|------|---------|
| `urdf/ugv.urdf.xacro` | Top-level URDF: includes base + sensors |
| `urdf/ugv_base.urdf.xacro` | Chassis, wheels, DiffDrive plugin (sim: not executed) |
| `urdf/sensors/camera.urdf.xacro` | camera_link + camera_link_optical |
| `urdf/sensors/imu.urdf.xacro` | imu_link |

### URDF Link/Joint Tree

```
base_footprint  (virtual)
└── base_link   (chassis, 0.6×0.4×0.2m navy-blue box)
    ├── left_wheel_link    ← left_wheel_joint (continuous)
    ├── right_wheel_link   ← right_wheel_joint (continuous)
    ├── front_caster_wheel_link  ← front_caster_wheel_joint (fixed)
    ├── rear_caster_wheel_link   ← rear_caster_wheel_joint (fixed)
    ├── camera_link              ← base_camera_joint (fixed, at 0.0553,0,0.4323)
    │   └── camera_link_optical  ← camera_optical_joint (fixed, -π/2 optical rotation)
    └── imu_link                 ← base_imu_joint (fixed, at 0,0,0.11)
```

> [!NOTE]
> The `ugv_base.urdf.xacro` also defines a DiffDrive Gazebo plugin — this is **not used in simulation**. The Gazebo simulation uses the DiffDrive plugin defined in Localbot's `model.sdf`. The URDF plugin would only activate if the URDF model were spawned directly into Gazebo (not done in this project).

---

## 9. Module: Perception (ugv_perception)

**What it provides**: Reliable depth images, 3D point cloud for Nav2, YOLO hazard masks, AI depth overlay.

**Output topics**: `/perception/depth/image_raw`, `/perception/depth/camera_info`, `/perception/depth/points`, `/perception/yolo/overlay`, `/perception/hazard_mask`, (optional) `/perception/depth_ai/image_raw`

**Input topics**: `/camera/depth/image_raw`, `/camera/depth/camera_info`, `/camera/image_raw`, `/camera/camera_info`

**Launch**: `perception.launch.py` (included via `ugv_complete.launch.py`)

### Nodes

| Node | Active when | Input | Output |
|------|-------------|-------|--------|
| `depth_relay_node` | `camera_type:=sim` | `/camera/depth/*` (BestEffort) | `/perception/depth/*` (Reliable) |
| `depth_node` (Depth Anything V3) | `camera_type:=monocular` | `/camera/image_raw` | `/perception/depth/image_raw` |
| `realsense_relay_node` | `camera_type:=realsense` | `/camera/camera/depth/*` | `/perception/depth/*` |
| `zed_relay_node` | `camera_type:=zed` | `/zed/zed_node/depth/*` | `/perception/depth/*` |
| `point_cloud_xyz_node` | always | `/perception/depth/*` | `/perception/depth/points` |
| `yolo_seg_node` | always | `/camera/image_raw` | `/perception/yolo/overlay`, `/perception/hazard_mask` |
| `depth_viz_node` | `enable_depth_viz:=true` | `/camera/image_raw` | `/perception/depth_ai/image_raw` |

**Exactly ONE depth source** is active at a time (selected by `camera_type` launch arg).

### Key design: depth_relay_node

The relay exists to solve a QoS mismatch:
- Gazebo bridge publishes BestEffort (all sensor topics)
- RTAB-Map and Nav2 nodes need Reliable to guarantee delivery
- relay node: subscribes BestEffort, republishes Reliable + correct frame_id

For production: the same relay pattern exists for RealSense, ZED, and Depth Anything V3.

---

## 10. Module: SLAM (localization.launch.py)

**What it provides**: 2D occupancy map, `map→odom` TF.

**Output topics**: `/map` (OccupancyGrid, Reliable + TransientLocal), `/tf` (map→odom at ~1 Hz)

**Input topics**: `/rtabmap/rgbd_image`, `/odometry/filtered`

**Launch**: `localization.launch.py` (mode:=sim uses RTAB-Map only; mode:=hw also enables EKF)

### Nodes

| Node | Package | Purpose |
|------|---------|---------|
| `rgbd_sync` | rtabmap_sync | Sync RGB + depth + camera_info → RGBDImage |
| `rtabmap` | rtabmap_slam | Visual SLAM: map building + map→odom TF |
| `ekf_filter_node` | robot_localization | Sensor fusion (hw mode only) |

### Key parameters (in `config/rtabmap.yaml`)

```yaml
RGBD/LinearUpdate: "0.0"   # keyframe every frame (not just when moving)
RGBD/AngularUpdate: "0.0"  # same — /map published immediately at startup
map_always_update: True     # republish /map at ~1 Hz even without new keyframes
Grid/CellSize: "0.10"      # 10cm resolution (matches global_costmap)
Grid/FromDepth: "true"     # build 2D map from depth camera (no lidar)
Reg/Force3DoF: "true"      # 2D SLAM for flat ground UGV
```

**Map persistence**: `delete_db_on_start: True` in launch = fresh map each run. To reuse a saved map: set `False` and remove `arguments=['-d']`.

---

## 11. Module: Navigation (navigation.launch.py)

**What it provides**: Path planning, path execution, recovery behaviors.

**Output topics**: `/cmd_vel`, `/plan`, `/local_costmap/costmap`, `/global_costmap/costmap`

**Input topics**: `/map`, `/odometry/filtered`, `/perception/depth/points`, `/tf`

### Nav2 Stack

| Component | Plugin | Purpose |
|-----------|--------|---------|
| Controller Server | RegulatedPurePursuitController | Path following (local) |
| Planner Server | SmacPlanner2D | Global path planning |
| Behavior Server | Spin, Backup, Wait | Recovery behaviors |
| BT Navigator | NavigateToPose | Goal orchestration |
| Lifecycle Manager | - | Manages Nav2 node lifecycle |

### Key parameters (in `config/nav2_params.yaml`)

```yaml
controller_server:
  failure_tolerance: 1.0       # 0.3 caused abort on brief TF hiccups
  movement_time_allowance: 25.0  # generous: allows replanning pauses
  
  FollowPath:  # RPP
    transform_tolerance: 1.0   # must match RTAB-Map's ~1Hz TF update rate
    use_collision_detection: false  # costmap handles this; RPP checker caused phantom stops
    desired_linear_vel: 0.5    # m/s, safe for warehouse

local_costmap:
  obstacle_layer:
    observation_persistence: 0.5  # clears stale obstacles faster during turns
    obstacle_range: 6.0
    raytrace_range: 7.0
  inflation_layer:
    inflation_radius: 0.55      # tune if narrow passages are blocked
```

---

## 12. Module: Terrain Analysis (ugv_terrain)

**What it provides**: 2.5D height map + traversability grid from depth + YOLO fusion.

**Output**: Grid map (traversability score per cell)

**Input topics**: `/perception/depth/points`, `/perception/hazard_mask`, `/perception/depth/camera_info`

**Node**: `terrain_analysis_node.py`
- Subscribes to live camera_info for intrinsics (fx, fy) — no hardcoded values
- Uses vectorized NumPy operations for efficient projection
- YOLO hazard mask fused with terrain height for enhanced obstacle classification

---

## 13. Module: Visualization (ugv_bringup rviz)

**Launch**: `rviz.launch.py` (included via `ugv_complete.launch.py`)

**Config**: `rviz/ugv_autonomy.rviz`

| Display | Topic | QoS |
|---------|-------|-----|
| RGBCamera | `/camera/image_raw` | BestEffort |
| RealDepth | `/perception/depth/image_raw` | Reliable |
| YOLOHazardOverlay | `/perception/yolo/overlay` | Reliable |
| AIDepth | `/perception/depth_ai/image_raw` | Reliable |
| SLAMMap | `/map` | Reliable + Transient Local |
| LocalCostmap | `/local_costmap/costmap` | Reliable + Volatile |
| GlobalCostmap | `/global_costmap/costmap` | Reliable + Volatile |
| Robot model | URDF via RSP | — |
| Point cloud | `/perception/depth/points` | Reliable |

**Fixed Frame**: `odom` (always available; switch to `map` for SLAM coordinate view)

---

## 14. Configuration Reference

### `src/ugv_bringup/config/nav2_params.yaml`

Full Nav2 stack configuration. Key values with reasoning:

| Parameter | Value | Why |
|-----------|-------|-----|
| `failure_tolerance` | 1.0 | Sim-clock jumps + 1Hz map→odom TF cause brief failures |
| `movement_time_allowance` | 25.0s | Allow time for replanning pauses |
| `transform_tolerance` (RPP) | 1.0 | Matches RTAB-Map's 1Hz TF update interval |
| `use_collision_detection` | false | Costmap handles this; RPP checker caused phantom stops |
| `observation_persistence` | 0.5s | Front camera can't see sides; clear stale obstacles fast |
| `inflation_radius` (local) | 0.55m | ~2× robot_radius (0.30m); reduce if narrow passage issues |

### `src/ugv_bringup/config/rtabmap.yaml`

RTAB-Map SLAM parameters. Key values:

| Parameter | Value | Why |
|-----------|-------|-----|
| `RGBD/LinearUpdate` | 0.0 | Keyframe on every frame → map at startup without motion |
| `RGBD/AngularUpdate` | 0.0 | Same |
| `map_always_update` | True | Republish `/map` at ~1 Hz even between keyframes |
| `Grid/CellSize` | 0.10 | 10cm resolution = matches global_costmap resolution |
| `wait_for_transform` | 1.0 | Prevents TF lookup failure during Gazebo startup |

### `src/ugv_sim/config/gazebo_bridge.yaml`

ROS↔Gazebo topic bridge. All GZ→ROS bridges use BestEffort internally.

> [!CAUTION]
> Depth camera GZ topic suffix is `depth_image` NOT `image`. Using `image` creates a ghost topic — zero bytes received, SLAM never gets depth data, map never appears.

---

## 15. Depth Source Swapping (sim → production)

All depth sources publish to the same standardized interface:

```
/perception/depth/image_raw    (32FC1, metres, frame_id=camera_link_optical)
/perception/depth/camera_info  (matching CameraInfo)
```

Nothing downstream (Nav2, RTAB-Map, terrain analysis) changes when the source changes.

```bash
# Simulation (default)
ros2 launch ugv_bringup ugv_complete.launch.py mode:=sim camera_type:=sim

# Hardware + RealSense D435/D455
ros2 launch realsense2_camera rs_launch.py  # in separate terminal
ros2 launch ugv_bringup ugv_complete.launch.py mode:=hw camera_type:=realsense

# Hardware + ZED 2/ZED X  
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2  # in separate terminal
ros2 launch ugv_bringup ugv_complete.launch.py mode:=hw camera_type:=zed

# Hardware + Depth Anything V3 (relative depth, any RGB camera)
ros2 launch ugv_bringup ugv_complete.launch.py mode:=hw camera_type:=monocular
```

---

## 16. Troubleshooting

### SLAM map not appearing ("No map received")

1. Wait ≥ 60s — cameras take 35s, RTAB-Map needs a few more seconds
2. `ros2 topic hz /camera/depth/image_raw` — must be ~30 Hz
3. `ros2 topic hz /rtabmap/rgbd_image` — must be ≥ 5 Hz (sync succeeds)
4. `ros2 topic echo /map --no-arr --once` — must return data
5. If still nothing: `ros2 service call /rtabmap/reset std_srvs/srv/Empty` then wait

### Robot stops navigating mid-path

1. `ros2 topic hz /odometry/filtered` — must be ~20 Hz
2. `ros2 topic hz /map` — must be ~1 Hz
3. `ros2 run tf2_ros tf2_echo map odom` — must succeed
4. If Nav2 aborts: check progress checker — robot may be genuinely stuck (obstacle)
5. Try a different goal or `ros2 service call /rtabmap/reset` to rebuild map

### Localbot not appearing in Gazebo (invisible or missing)

1. `ls install/ugv_sim/share/ugv_sim/models/localbot/` — must show `model.config model.sdf`
2. If missing: rebuild with `colcon build --packages-select ugv_sim`
3. Check `GZ_SIM_RESOURCE_PATH` contains the install models dir (done automatically by sim.launch.py)

### RViz shows wrong robot model (TugBot mesh instead of Localbot box)

Rebuild `ugv_description`: `colcon build --packages-select ugv_description`
The mesh was replaced with a box visual in ugv_base.urdf.xacro.

### Wheels not turning / robot not moving

1. `ros2 topic hz /cmd_vel` — must be non-zero when navigating
2. `ros2 topic echo /cmd_vel --once` — check values are non-zero
3. `ros2 topic info /cmd_vel --verbose` — check bridge direction is ROS→GZ

### Camera/depth not visible in RViz

- `/camera/image_raw` display: **must be BestEffort** (Gazebo bridge QoS)
- `/perception/depth/image_raw` display: **must be Reliable** (relay node QoS)
- Depth image looks all black: set RViz `Normalize Range: true`, `Max: 10`

---

## 17. Debugging Commands

```bash
source install/setup.bash

# ── Topic health ──────────────────────────────────────────────────────────────
ros2 topic list | grep -E "camera|depth|map|odom|cmd_vel|joint|perception"
ros2 topic hz /camera/image_raw                # expect 30 Hz
ros2 topic hz /camera/depth/image_raw          # expect 30 Hz
ros2 topic hz /perception/depth/image_raw      # expect 30 Hz
ros2 topic hz /perception/depth/points         # expect 30 Hz
ros2 topic hz /rtabmap/rgbd_image             # expect 5-15 Hz
ros2 topic hz /map                             # ~1 Hz (TransientLocal — may show 0)
ros2 topic hz /odometry/filtered               # expect 20 Hz
ros2 topic hz /joint_states_urdf               # expect 50 Hz

# ── QoS diagnosis ─────────────────────────────────────────────────────────────
ros2 topic info /map --verbose                 # check Transient Local + Reliable
ros2 topic info /camera/image_raw --verbose    # check BestEffort
ros2 topic info /perception/depth/image_raw --verbose  # check Reliable

# ── TF diagnosis ──────────────────────────────────────────────────────────────
ros2 run tf2_tools view_frames                 # generates frames.pdf
ros2 run tf2_ros tf2_echo map odom             # SLAM transform
ros2 run tf2_ros tf2_echo odom base_footprint  # DiffDrive transform

# ── Nav2 state ────────────────────────────────────────────────────────────────
ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server
ros2 lifecycle get /bt_navigator

# ── SLAM state ────────────────────────────────────────────────────────────────
ros2 param get /rtabmap map_always_update      # must be True
ros2 param get /rtabmap RGBD/LinearUpdate      # must be 0.0
ros2 service call /rtabmap/reset std_srvs/srv/Empty  # reset map

# ── Joint states ──────────────────────────────────────────────────────────────
ros2 topic echo /joint_states --no-arr --once  # Gazebo output: left/right_wheel_joint
ros2 topic echo /joint_states_urdf --no-arr --once  # relay output: same names (Localbot)

# ── Localbot model verification ───────────────────────────────────────────────
ls install/ugv_sim/share/ugv_sim/models/localbot/
grep "model name" install/ugv_sim/share/ugv_sim/models/localbot/model.sdf
grep "uri" install/ugv_sim/share/ugv_sim/worlds/tugbot_warehouse.sdf | grep localbot

# ── Navigation goal ───────────────────────────────────────────────────────────
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  '{pose: {header: {frame_id: "map"}, pose: {position: {x: 5.0, y: -2.0, z: 0.0}, orientation: {w: 1.0}}}}'

# ── Clean kill ────────────────────────────────────────────────────────────────
pkill -9 -f "gz sim|ros2 launch|parameter_bridge|rtabmap|rviz2|yolo_seg|depth_relay"
```

---

## 18. How to Safely Modify Each Module

### Modify Localbot (geometry, sensors, physics)

1. Edit `src/ugv_sim/models/localbot/model.sdf`
2. If adding/removing sensors: also update `src/ugv_sim/config/gazebo_bridge.yaml`
3. If changing link names: also update `src/ugv_description/urdf/` and bridge config
4. Build: `colcon build --packages-select ugv_sim`
5. Verify: `grep "<your_change>" install/ugv_sim/share/ugv_sim/models/localbot/model.sdf`

### Modify the world (add/remove obstacles, change robot spawn)

1. Edit `src/ugv_sim/worlds/tugbot_warehouse.sdf`
2. Spawn pose is `<pose>x y z r p y</pose>` inside the localbot `<include>` block
3. To add new Fuel models: add `<include><uri>https://...URL...</uri></include>`
4. Build: `colcon build --packages-select ugv_sim`

### Add a new sensor to Localbot

1. Add link + joint + sensor to `src/ugv_sim/models/localbot/model.sdf`
2. Add link + joint to `src/ugv_description/urdf/sensors/` (new xacro file)
3. Include the new xacro in `urdf/ugv.urdf.xacro`
4. Add bridge entry in `src/ugv_sim/config/gazebo_bridge.yaml`
5. Add relay node in `src/ugv_bringup/launch/perception.launch.py` if QoS bridging needed

### Modify SLAM behavior

Edit `src/ugv_bringup/config/rtabmap.yaml` and/or params in `localization.launch.py`.

To reuse a saved map: in `localization.launch.py`, set `'delete_db_on_start': False` and remove `arguments=['-d']`.

### Modify Nav2 behavior (speed, safety margins, planner)

Edit `src/ugv_bringup/config/nav2_params.yaml`.

Key sections: `controller_server` (RPP params), `local_costmap` (obstacle sensing), `global_costmap` (map-based planning).

### Swap the planner

In `nav2_params.yaml`, change:
```yaml
planner_server:
  planner_plugins: ["GridBased"]
  GridBased:
    plugin: "nav2_smac_planner::SmacPlanner2D"  # change this
```

Available options: `SmacPlanner2D`, `SmacPlannerHybrid`, `NavfnPlanner`, `ThetaStarPlanner`.

### Add a production camera source

1. Create a relay node in `src/ugv_perception/ugv_perception/` (copy `realsense_relay_node.py` as template)
2. Add `<exec_depend>` in `src/ugv_perception/package.xml`
3. Add entry_point in `src/ugv_perception/setup.py`
4. Add node to `src/ugv_bringup/launch/perception.launch.py` with `IfCondition`
5. Document the new `camera_type` value in `ugv_complete.launch.py`

---

## 19. Known Limitations

| Limitation | Impact | Workaround |
|------------|--------|------------|
| Single front-facing camera | Can't sense obstacles to sides/rear | Reduce speed; larger `inflation_radius` |
| SLAM requires texture | Featureless white walls confuse RTAB-Map | `Vis/MinInliers: "15"` prevents false matches |
| map→odom TF at ~1 Hz | Controller needs `transform_tolerance: 1.0` | Already configured |
| Depth Anything V3 = relative depth | No metric scale for obstacle distance | Use `realsense` or `zed` in production |
| Gazebo startup ~60s | Cameras unavailable for first minute | Wait before testing; expected behavior |
| Caster wheels (ball joint) | No wheel visual in Gazebo for casters (SDF ball joint shows as point) | Spheres still provide correct physics |

---

## 20. Bug & Change History

| Commit | Bug | Root Cause | Fix |
|--------|-----|-----------|-----|
| `57e33de` | SLAM never gets depth | Depth GZ topic is `depth_image` not `image` | Fixed topic name in bridge config |
| `1119abf` | rgbd_sync gets no images | `qos=0` (Reliable) vs BestEffort bridge | Changed to `qos=1` (SENSOR_DATA) |
| `ce3f06c` | Wrong back-projection in terrain | Hardcoded `fx=616.0`; actual Gazebo cam `fx≈421.6` | Subscribe to live camera_info |
| `e326bfb` | RViz fixed frame wrong | `map` frame doesn't exist until SLAM builds | Changed to `odom` |
| `219b04e` | Tugbot light spinning fast | `warnign_light_joint` revolute at 10 rad/s | Changed to `type="fixed"`, removed JointController |
| `219b04e` | RViz robot flickers white | Relay passed non-URDF joints to RSP → "unknown joint" at 50 Hz → TF crash | Filter relay to wheel joints only |
| `3f856dd` | `/map` never published | RTAB-Map waits for 0.1m movement before first keyframe | `RGBD/LinearUpdate: "0.0"` + `map_always_update: True` |
| `72d669b` | All model.sdf fixes ignored | World SDF used Fuel URL, bypassing local model dir | Changed URI to `model://tugbot` + added `model.config` |
| `72d669b` | Robot stops mid-navigation | `failure_tolerance: 0.3` aborted on brief TF hiccups | Increased to 1.0; disabled RPP collision detection |
| **`current`** | **URDF↔SDF wheel physics mismatch** | TugBot wheel_radius=0.195m but URDF says 0.1m; separation also wrong | **Localbot uses URDF-exact params** |
| **`current`** | **Fuel/internet dependency for robot** | World loaded MovAI TugBot from `fuel.ignitionrobotics.org` | **Created Localbot: zero Fuel dependency** |
| **`current`** | **Joint name mismatch (wheel_left vs left_wheel)** | TugBot SDF used reversed naming convention | **Localbot uses URDF-matching names; relay simplified** |
| **`current`** | **TugBot alias link in URDF** | `tugbot/camera_front/color` link matched TugBot sensor frame_id | **Removed alias; Localbot sensor uses `camera_link_optical` directly** |
| **`current`** | **External mesh dependencies** | URDF used `tugbot_simp.dae` and `wheel.dae` meshes | **Replaced with primitive geometry (box/cylinder); zero mesh dependencies** |
