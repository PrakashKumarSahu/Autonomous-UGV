# Autonomous UGV — Complete System Documentation

> **Long-term maintenance reference** — one document for understanding, building, running, debugging, extending, and deploying the entire project.

---

## Table of Contents

**Quick Reference**
1. [Build & Launch](#1-build--launch)
2. [All Launch Arguments](#2-all-launch-arguments)
3. [Common Recipes](#3-common-recipes)
4. [Send Navigation Goals](#4-send-navigation-goals)

**Architecture**
5. [Module Overview](#5-module-overview)
6. [Data Flow Diagram](#6-data-flow-diagram)
7. [TF Frame Tree](#7-tf-frame-tree)
8. [Topic Reference](#8-topic-reference)

**Module Documentation** *(each module is independently maintainable)*
9. [Simulation (ugv\_sim)](#9-simulation-ugv_sim)
10. [Robot Description (ugv\_description)](#10-robot-description-ugv_description)
11. [Perception (ugv\_perception)](#11-perception-ugv_perception)
12. [SLAM (localization.launch.py)](#12-slam-localizationlaunchpy)
13. [Navigation (navigation.launch.py)](#13-navigation-navigationlaunchpy)
14. [Terrain Analysis (ugv\_terrain)](#14-terrain-analysis-ugv_terrain)

**Operations**
15. [Using a Custom World](#15-using-a-custom-world)
16. [Depth Source Swapping](#16-depth-source-swapping)
17. [Configuration Reference](#17-configuration-reference)
18. [Standalone Module Launch](#18-standalone-module-launch)
19. [Troubleshooting](#19-troubleshooting)
20. [Debugging Commands](#20-debugging-commands)
21. [How to Modify Each Module Safely](#21-how-to-modify-each-module-safely)
22. [Known Limitations](#22-known-limitations)
23. [Change History](#23-change-history)

---

## 1. Build & Launch

```bash
cd ~/Autonomous-UGV
source /opt/ros/jazzy/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

# Default: Gazebo + test arena + all modules
ros2 launch ugv_bringup ugv_complete.launch.py
```

**Startup timeline** (Gazebo Harmonic with NVIDIA GPU):

| Time | What becomes ready |
|------|-------------------|
| 0–5 s | Physics starts |
| ~8 s | Localbot spawns (3s delay ensures physics is ready) |
| ~15 s | Camera/depth topics active, bridge running |
| ~30 s | RTAB-Map first keyframe → `/map` published |
| ~45 s | Nav2 lifecycle active, goals accepted |

---

## 2. All Launch Arguments

Run `ros2 launch ugv_bringup ugv_complete.launch.py --show-args` to see the full list.

| Argument | Default | Description |
|----------|---------|-------------|
| `mode` | `sim` | `sim` (Gazebo) or `hw` (real robot) |
| `use_sim_time` | `true` | Use `/clock` from Gazebo |
| **World** | | |
| `world` | `ugv_test_arena.sdf` | Path to any Gazebo SDF world file |
| `world_name` | `world_demo` | Must match `<world name="...">` inside the SDF |
| **Robot** | | |
| `robot_name` | `localbot` | Entity name in Gazebo, used in all GZ topic paths |
| `spawn_x` | `0.0` | Spawn X position (metres, East) |
| `spawn_y` | `-6.5` | Spawn Y position (metres, North) |
| `spawn_z` | `0.01` | Spawn Z position (metres above ground) |
| `spawn_yaw` | `1.5708` | Spawn yaw angle (rad): 1.5708=N, 0=E, 3.14=S |
| **Perception** | | |
| `camera_type` | `sim` | Depth source: `sim`\|`monocular`\|`realsense`\|`zed` |
| `enable_depth_viz` | `false` | Run Depth Anything V3 overlay in RViz (GPU) |
| **SLAM** | | |
| `enable_rtabmap` | `true` | Enable RTAB-Map SLAM |
| **Navigation** | | |
| `nav2_params_file` | `nav2_params.yaml` | Path to Nav2 YAML (override without rebuild) |
| **Visualization** | | |
| `enable_rviz` | `true` | Launch RViz2 |
| `rviz_config` | `ugv_autonomy.rviz` | Path to RViz config file |

---

## 3. Common Recipes

```bash
# ── Default simulation (everything) ──────────────────────────────────────────
ros2 launch ugv_bringup ugv_complete.launch.py

# ── Custom world + spawn position ────────────────────────────────────────────
ros2 launch ugv_bringup ugv_complete.launch.py \
  world:=$(ros2 pkg prefix ugv_sim)/share/ugv_sim/worlds/empty_world.sdf \
  world_name:=empty \
  spawn_x:=0.0  spawn_y:=0.0  spawn_z:=0.01  spawn_yaw:=0.0

# ── Completely custom world (any SDF file) ────────────────────────────────────
ros2 launch ugv_bringup ugv_complete.launch.py \
  world:=/path/to/my_world.sdf \
  world_name:=my_world \
  spawn_x:=5.0  spawn_y:=2.0  spawn_yaw:=3.14

# ── Hardware + RealSense ──────────────────────────────────────────────────────
ros2 launch realsense2_camera rs_launch.py       # terminal 1
ros2 launch ugv_bringup ugv_complete.launch.py \
  mode:=hw  camera_type:=realsense  use_sim_time:=false

# ── Hardware + ZED ────────────────────────────────────────────────────────────
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2   # terminal 1
ros2 launch ugv_bringup ugv_complete.launch.py \
  mode:=hw  camera_type:=zed  use_sim_time:=false

# ── Custom Nav2 config (no rebuild needed) ────────────────────────────────────
ros2 launch ugv_bringup ugv_complete.launch.py \
  nav2_params_file:=/path/to/my_nav2_params.yaml

# ── Headless (no RViz — useful for remote/CI) ─────────────────────────────────
ros2 launch ugv_bringup ugv_complete.launch.py enable_rviz:=false

# ── SLAM debug only (no Nav2) ─────────────────────────────────────────────────
ros2 launch ugv_bringup ugv_complete.launch.py \
  enable_rtabmap:=true  # Launch just sim+perception+SLAM+RViz, skip Nav2
  # (Nav2 is always included via ugv_complete.launch.py; for SLAM-only testing
  #  launch modules individually — see Section 18)

# ── Reset map ─────────────────────────────────────────────────────────────────
ros2 service call /rtabmap/reset std_srvs/srv/Empty
```

---

## 4. Send Navigation Goals

```bash
# Via RViz: use the "Nav2 Goal" tool in the toolbar (click + drag for heading)

# Via CLI:
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  '{pose: {header: {frame_id: "map"},
    pose: {position: {x: 0.0, y: 6.0, z: 0.0},
           orientation: {w: 1.0}}}}'

# Test arena suggested goals (after SLAM builds):
# North through all obstacles: x=0,  y=6
# NW corridor:                 x=-7, y=5
# NE side (near L-wall):       x=7,  y=5
# Arena center (zigzag zone):  x=0,  y=0
```

---

## 5. Module Overview

Each module has a single clean interface — it provides a set of ROS topics and requires a set of ROS topics. You can work on any module by reading only its section.

```
┌──────────────────────────────────────────────────────────────────────────┐
│  SIMULATION MODULE  (ugv_sim)                                            │
│                                                                          │
│  ugv_test_arena.sdf   empty_world.sdf   <any custom SDF>                │
│    ↓ world (no robot)                                                    │
│  Gazebo Harmonic ←────────── GZ_SIM_RESOURCE_PATH                       │
│    ↓                                                                     │
│  ros_gz_sim create                   ← sim.launch.py spawns Localbot    │
│    ↓ spawns model://localbot at (spawn_x, spawn_y, spawn_yaw)           │
│                                                                          │
│  Dynamic GZ Bridge  (OpaqueFunction → temp YAML, parametric)            │
│    GZ /world/{world_name}/model/{robot_name}/...  →  ROS /camera/*      │
│                                                   →  /imu/data          │
│                                                   →  /odometry/filtered │
│                                                   →  /tf                │
│                                                   →  /joint_states      │
│    ROS /cmd_vel  →  GZ /model/{robot_name}/cmd_vel                      │
│                                                                          │
│  joint_state_relay.py   (QoS bridge: BestEffort→Reliable)               │
│    /joint_states (BE) → /joint_states_urdf (Reliable)                   │
└──────────────────────────────────────────────────────────────────────────┘
         │                   │                           │
         ▼                   ▼                           ▼
┌──────────────┐  ┌────────────────────┐  ┌────────────────────────────────┐
│  DESCRIPTION │  │    PERCEPTION      │  │           SLAM                 │
│  ugv_desc..  │  │    ugv_perception  │  │   localization.launch.py       │
│              │  │                    │  │                                │
│  RSP: URDF   │  │  ONE depth source: │  │  rgbd_sync: RGB+depth→RGBD    │
│  → /tf_static│  │  sim/mono/RS/zed   │  │  rtabmap:  RGBD→/map           │
│  → /tf (whl) │  │  → /perception/   │  │            → map→odom TF       │
│              │  │    depth/*         │  │                                │
│  Needs:      │  │                    │  │  Needs: /rtabmap/rgbd_image    │
│  /joint_     │  │  point_cloud_xyz   │  │         /odometry/filtered     │
│  states_urdf │  │  → /perception/   │  │                                │
│              │  │    depth/points    │  │  (EKF in hw mode)              │
│              │  │                    │  │                                │
│              │  │  yolo_seg_node     │  │                                │
│              │  │  → /perception/   │  │                                │
│              │  │    yolo/overlay    │  │                                │
│              │  │  → /perception/   │  │                                │
│              │  │    hazard_mask     │  │                                │
└──────────────┘  └────────────────────┘  └────────────────────────────────┘
                              │                           │
                              ▼                           ▼
                  ┌─────────────────────────────────────────────────────────┐
                  │                  NAVIGATION  (Nav2)                     │
                  │           navigation.launch.py                          │
                  │                                                         │
                  │  Local costmap  (ObstacleLayer + InflationLayer)        │
                  │    ← /perception/depth/points (camera point cloud)      │
                  │  Global costmap (StaticLayer)                           │
                  │    ← /map (from RTAB-Map)                               │
                  │                                                         │
                  │  Smac2D Planner → /plan                                 │
                  │  RPP Controller → /cmd_vel                              │
                  │  BT Navigator  ← /navigate_to_pose action               │
                  └─────────────────────────────────────────────────────────┘
```

---

## 6. Data Flow Diagram

```
Gazebo depth_camera (30 Hz, 32FC1 metres)
  └──[GZ bridge, BestEffort]──→ /camera/depth/image_raw
       ├──[depth_relay_node (sim)]──→ /perception/depth/image_raw (Reliable)
       │    ├──[point_cloud_xyz_node]──→ /perception/depth/points
       │    │    └──[Nav2 ObstacleLayer]──→ local costmap obstacle cells
       │    └──[rgbd_sync]
       │         + /camera/image_raw
       │         + /camera/camera_info
       │         ↓
       │    /rtabmap/rgbd_image
       │         └──[rtabmap SLAM]──→ /map (Transient Local, ~1 Hz)
       │                           └──→ map→odom TF (~1 Hz)
       │
       └──[Nav2 RPP Controller]──→ /cmd_vel (20 Hz)
                                         │
              Gazebo DiffDrive ←────────[GZ bridge]
                └──→ /odometry/filtered (20 Hz)
                └──→ /tf (odom→base_footprint, 20 Hz)
                └──→ /joint_states (BestEffort, 50 Hz)
                          └──[joint_state_relay]──→ /joint_states_urdf (Reliable)
                                    └──[RSP]──→ /tf_static + wheel TF
```

---

## 7. TF Frame Tree

```
map          (published by rtabmap at ~1 Hz when SLAM builds first keyframe)
 └── odom    (published by GZ bridge DiffDrive at ~20 Hz — available immediately)
      └── base_footprint  (virtual ground projection)
           └── base_link  (chassis center)
                ├── left_wheel_link   ← left_wheel_joint (continuous, from joint_states_urdf)
                ├── right_wheel_link  ← right_wheel_joint (continuous)
                ├── front_caster_wheel_link  ← fixed
                ├── rear_caster_wheel_link   ← fixed
                ├── imu_link                 ← fixed
                └── camera_link              ← fixed (at 0.0553, 0, 0.4323 from base_link)
                      └── camera_link_optical ← fixed (-π/2 optical rotation)
```

> [!IMPORTANT]
> RViz Fixed Frame should be `odom` (always available from t=0). Switching to `map` gives the SLAM coordinate view but `map` only exists after RTAB-Map builds its first keyframe (~30s).

---

## 8. Topic Reference

| Topic | Type | QoS | Publisher → Consumer |
|-------|------|-----|---------------------|
| `/camera/image_raw` | Image | BestEffort | GZ bridge → rgbd_sync, yolo |
| `/camera/camera_info` | CameraInfo | BestEffort | GZ bridge → rgbd_sync |
| `/camera/depth/image_raw` | Image (32FC1) | BestEffort | GZ bridge → depth_relay |
| `/camera/depth/camera_info` | CameraInfo | BestEffort | GZ bridge → depth_relay |
| `/perception/depth/image_raw` | Image (32FC1) | Reliable | depth_relay → rgbd_sync, pt cloud |
| `/perception/depth/camera_info` | CameraInfo | Reliable | depth_relay → pt cloud |
| `/perception/depth/points` | PointCloud2 | Reliable | pt cloud → Nav2 ObstacleLayer |
| `/perception/yolo/overlay` | Image | Reliable | yolo → RViz |
| `/perception/hazard_mask` | Image | Reliable | yolo → terrain |
| `/rtabmap/rgbd_image` | RGBDImage | Reliable | rgbd_sync → rtabmap |
| `/map` | OccupancyGrid | Reliable+TL | rtabmap → Nav2 StaticLayer, RViz |
| `/odometry/filtered` | Odometry | Reliable | GZ bridge → rtabmap, Nav2 |
| `/cmd_vel` | Twist | Reliable | Nav2 RPP → GZ bridge → Localbot |
| `/joint_states` | JointState | BestEffort | GZ bridge → joint_relay |
| `/joint_states_urdf` | JointState | Reliable | joint_relay → RSP |
| `/tf` | TFMessage | Reliable | GZ+rtabmap+RSP → all |
| `/tf_static` | TFMessage | Reliable+TL | RSP → all |
| `/plan` | Path | Reliable | Nav2 planner → controller, RViz |
| `/local_costmap/costmap` | OccupancyGrid | Reliable | Nav2 → RViz |
| `/global_costmap/costmap` | OccupancyGrid | Reliable | Nav2 → RViz |

> [!CAUTION]
> **QoS mismatch = silent data loss.** Reliable publisher + BestEffort subscriber = zero bytes received in DDS. Always verify with `ros2 topic info <topic> --verbose` when debugging missing data.

---

## 9. Simulation (ugv\_sim)

**Package**: `ugv_sim`  
**Provides**: Sensor data, odometry, TF, clock  
**Requires** (from ROS): `/cmd_vel`

### Key files

| File | Purpose |
|------|---------|
| `worlds/ugv_test_arena.sdf` | Default obstacle course (zero Fuel dependencies) |
| `worlds/empty_world.sdf` | Minimal flat world — template for custom worlds |
| `models/localbot/model.sdf` | Complete robot SDF (cameras, IMU, DiffDrive) |
| `models/localbot/model.config` | Required for `model://localbot` URI resolution |
| `launch/sim.launch.py` | Gazebo + dynamic bridge + spawner + RSP + relay |
| `scripts/joint_state_relay.py` | QoS bridge: BestEffort → Reliable for joint states |

### How the dynamic bridge works

`sim.launch.py` uses `OpaqueFunction` to run Python code at launch runtime (when `LaunchConfiguration` values are resolved to strings). It calls `_make_bridge_yaml(world_name, robot_name)` which builds the complete bridge config as a Python dict, writes it to a temp YAML file, and passes the path to `ros_gz_bridge`. This means **any world name and any robot name work without changing any config file**.

### Localbot robot specs

| Property | Value | Matches URDF? |
|----------|-------|--------------|
| Wheel radius | 0.1 m | ✅ (TugBot was 0.195 m) |
| Wheel separation | 0.45 m | ✅ (TugBot was 0.5605 m) |
| Left wheel joint | `left_wheel_joint` | ✅ (TugBot had reversed naming) |
| Right wheel joint | `right_wheel_joint` | ✅ |
| Camera position | `0.0553, 0, 0.4323` from base_link | ✅ |
| Camera link name | `camera_link` | ✅ |
| Sensors | RGB + depth_camera + IMU | ✅ only used sensors |

### GZ_SIM_RESOURCE_PATH

Set in `sim.launch.py` to: `install/ugv_sim/share/ugv_sim/models : ~/.gz/fuel/...`

Local install directory comes **first**. `model://localbot` resolves to `install/ugv_sim/share/ugv_sim/models/localbot/model.sdf`. **Never use a Fuel URL** for the robot — it bypasses local changes.

---

## 10. Robot Description (ugv\_description)

**Package**: `ugv_description`  
**Provides**: `/tf_static` (fixed joints), `/tf` (wheel joints via RSP)  
**Requires**: `/joint_states_urdf` (from joint_state_relay)

### URDF structure

```
ugv.urdf.xacro                    ← top-level (includes base + sensors)
├── ugv_base.urdf.xacro           ← chassis, wheels (box/cylinder geometry — no mesh)
├── sensors/camera.urdf.xacro     ← camera_link, camera_link_optical
└── sensors/imu.urdf.xacro        ← imu_link
```

**No external mesh dependencies** — all geometry is primitive SDF/URDF shapes (box, cylinder). The TugBot `.dae` meshes were removed when Localbot was created.

### URDF link tree

```
base_footprint → base_link (navy box, 0.6×0.4×0.2m)
  ├── left_wheel_link  (orange cylinder, r=0.1m)
  ├── right_wheel_link (orange cylinder, r=0.1m)
  ├── front_caster_wheel_link
  ├── rear_caster_wheel_link
  ├── camera_link (grey box at 0.0553, 0, 0.4323 from base_link)
  │   └── camera_link_optical (optical frame, −π/2 rotation)
  └── imu_link
```

---

## 11. Perception (ugv\_perception)

**Package**: `ugv_perception`  
**Launch**: `ugv_bringup/launch/perception.launch.py`  
**Provides**: Standardised depth topics, point cloud, YOLO overlay  
**Requires**: `/camera/image_raw`, `/camera/camera_info`, `/camera/depth/*`

### Depth source selection

Exactly **one** depth source is active at a time (selected by `camera_type` launch arg). All publish to the same standardised output interface:

| `camera_type` | Node | Input | Notes |
|---------------|------|-------|-------|
| `sim` (default) | `depth_relay_node` | `/camera/depth/*` (BestEffort) | Gazebo real depth sensor |
| `monocular` | `depth_node` | `/camera/image_raw` | Depth Anything V3 (relative depth) |
| `realsense` | `realsense_relay_node` | `/camera/camera/depth/*` | Metric depth, prerequisite: `rs_launch.py` |
| `zed` | `zed_relay_node` | `/zed/zed_node/depth/*` | Metric depth, prerequisite: `zed_camera.launch.py` |

**Standardised output** (same for all sources):
```
/perception/depth/image_raw    (32FC1, metres, frame: camera_link_optical)
/perception/depth/camera_info  (matching CameraInfo)
```

Nothing downstream (Nav2, RTAB-Map, terrain) changes when the source changes.

### Shared nodes (always active)

| Node | Input | Output |
|------|-------|--------|
| `point_cloud_xyz_node` | `/perception/depth/*` | `/perception/depth/points` (PointCloud2) |
| `yolo_seg_node` | `/camera/image_raw` | `/perception/yolo/overlay`, `/perception/hazard_mask` |
| `depth_viz_node` | `/camera/image_raw` | `/perception/depth_ai/image_raw` (if `enable_depth_viz:=true`) |

---

## 12. SLAM (localization.launch.py)

**Launch**: `ugv_bringup/launch/localization.launch.py`  
**Provides**: `/map` (OccupancyGrid), map→odom TF  
**Requires**: `/camera/image_raw`, `/camera/camera_info`, `/perception/depth/image_raw`, `/odometry/filtered`

### Nodes

| Node | Package | Purpose |
|------|---------|---------|
| `rgbd_sync` | `rtabmap_sync` | Synchronises RGB + depth + CameraInfo → RGBDImage |
| `rtabmap` | `rtabmap_slam` | Visual SLAM: builds map, publishes map→odom TF |
| `ekf_filter_node` | `robot_localization` | Sensor fusion (hw mode only, `mode:=hw`) |

### Key parameters (in `config/rtabmap.yaml`)

| Parameter | Value | Reason |
|-----------|-------|--------|
| `RGBD/LinearUpdate` | `"0.0"` | Keyframe on every frame → /map at startup without movement |
| `RGBD/AngularUpdate` | `"0.0"` | Same |
| `map_always_update` | `True` | Republish /map at ~1Hz between keyframes |
| `Grid/CellSize` | `"0.10"` | 10cm resolution = matches global_costmap |
| `Grid/FromDepth` | `"true"` | Build map from depth camera (no lidar) |
| `Reg/Force3DoF` | `"true"` | 2D SLAM for flat-ground UGV |
| `wait_for_transform` | `1.0` | Prevents TF lookup failure during Gazebo startup |

**Map reset**: `ros2 service call /rtabmap/reset std_srvs/srv/Empty`  
**Reuse saved map**: In `localization.launch.py`, set `'delete_db_on_start': False` and remove `arguments=['-d']`.

---

## 13. Navigation (navigation.launch.py)

**Launch**: `ugv_bringup/launch/navigation.launch.py`  
**Provides**: `/cmd_vel`, `/plan`, costmaps  
**Requires**: `/map`, `/odometry/filtered`, `/perception/depth/points`, `/tf`

### Nav2 stack

| Node | Plugin | Purpose |
|------|--------|---------|
| `controller_server` | `RegulatedPurePursuitController` | Path following |
| `planner_server` | `SmacPlanner2D` | Global path planning |
| `behaviors` | Spin, Backup, Wait | Recovery behaviors |
| `bt_navigator` | NavigateToPose BT | Goal orchestration |

### Key parameters (in `config/nav2_params.yaml`)

| Parameter | Value | Reason |
|-----------|-------|--------|
| `failure_tolerance` | 1.0 | Sim-clock jumps + 1Hz map→odom TF cause brief failures |
| `movement_time_allowance` | 25.0 s | Allows replanning pauses |
| `transform_tolerance` (RPP) | 1.0 | Matches RTAB-Map's 1Hz TF update rate |
| `use_collision_detection` | `false` | Costmap handles this; RPP collision checker caused phantom stops |
| `observation_persistence` | 0.5 s | Front-only camera: clear stale obstacles fast during turns |
| `inflation_radius` | 0.55 m | ~2× robot radius; tune if narrow passages are blocked |

**Custom Nav2 config** (no rebuild needed):
```bash
ros2 launch ugv_bringup ugv_complete.launch.py nav2_params_file:=/path/to/my_params.yaml
```

---

## 14. Terrain Analysis (ugv\_terrain)

**Package**: `ugv_terrain`  
**Provides**: 2.5D traversability grid map  
**Requires**: `/perception/depth/points`, `/perception/hazard_mask`, `/perception/depth/camera_info`

Fuses depth-based height estimation with YOLO hazard masks. Camera intrinsics (fx, fy) are read from live `/camera_info` — not hardcoded.

---

## 15. Using a Custom World

The simulation is fully decoupled from any specific world file. The robot is **spawned separately** via `ros_gz_sim create`, not embedded in the world SDF.

### Requirements for a custom world SDF

1. **No robot model** — do NOT include `<include>` for the robot
2. **World-level system plugins** — must have Physics, UserCommands, SceneBroadcaster, Imu, Sensors
3. **Sensors plugin must use ogre2** — required for depth_camera
4. **`start_paused: false`** — robot sensors need to publish immediately
5. **Any world name** — set `world_name:=` launch arg to match `<world name="...">`

### Step-by-step

```bash
# 1. Start from the empty world template
cp $(ros2 pkg prefix ugv_sim)/share/ugv_sim/worlds/empty_world.sdf ~/my_world.sdf

# 2. Edit ~/my_world.sdf:
#    - Change <world name="empty"> to <world name="my_world">
#    - Add obstacles using the template in the file
#    - Do NOT add robot include

# 3. Launch
ros2 launch ugv_bringup ugv_complete.launch.py \
  world:=~/my_world.sdf \
  world_name:=my_world \
  spawn_x:=0.0  spawn_y:=0.0  spawn_z:=0.01  spawn_yaw:=0.0
```

### Built-in worlds

| World SDF | World Name | Description |
|-----------|-----------|-------------|
| `ugv_test_arena.sdf` | `world_demo` | 20×16m arena with 5-row obstacle course |
| `empty_world.sdf` | `empty` | Flat ground only — template for custom worlds |

### Adding obstacles to the test arena

Obstacles are inline SDF primitives. To add one, insert a `<model>` block inside `ugv_test_arena.sdf`:

```xml
<model name="my_obstacle">
  <static>true</static>
  <pose>X Y Z 0 0 0</pose>   <!-- Z = half of object height for boxes -->
  <link name="link">
    <visual name="v">
      <geometry><box><size>W D H</size></box></geometry>
      <material><ambient>R G B 1</ambient><diffuse>R G B 1</diffuse></material>
    </visual>
    <collision name="c">
      <geometry><box><size>W D H</size></box></geometry>
    </collision>
  </link>
</model>
```

Then rebuild: `colcon build --packages-select ugv_sim`

---

## 16. Depth Source Swapping

All depth sources publish to the same standardised interface — nothing downstream changes:

```bash
# Simulation (Gazebo real depth sensor)
ros2 launch ugv_bringup ugv_complete.launch.py camera_type:=sim

# Hardware: Depth Anything V3 (monocular, any RGB camera)
ros2 launch ugv_bringup ugv_complete.launch.py \
  mode:=hw  camera_type:=monocular  use_sim_time:=false

# Hardware: Intel RealSense D435/D455
ros2 launch realsense2_camera rs_launch.py          # start camera driver first
ros2 launch ugv_bringup ugv_complete.launch.py \
  mode:=hw  camera_type:=realsense  use_sim_time:=false

# Hardware: Stereolabs ZED 2 / ZED X
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2   # driver first
ros2 launch ugv_bringup ugv_complete.launch.py \
  mode:=hw  camera_type:=zed  use_sim_time:=false
```

> [!NOTE]
> Depth Anything V3 produces **relative depth** (no metric scale). Adequate for obstacle avoidance at UGV speeds. For metric accuracy (needed for precise SLAM), use RealSense or ZED.

---

## 17. Configuration Reference

### `config/rtabmap.yaml`

RTAB-Map SLAM configuration. See [Section 12](#12-slam-localizationlaunchpy) for full parameter table.

### `config/nav2_params.yaml`

Nav2 full stack configuration. Key sections:
- `controller_server.ros__parameters.FollowPath` — RPP controller settings
- `local_costmap.ros__parameters` — obstacle sensing radius, inflation
- `global_costmap.ros__parameters` — map resolution (must match RTAB-Map)
- `planner_server` — path planning algorithm and plugin

### `config/ekf.yaml`

Robot localization EKF (only used in `mode:=hw`). Fuses `/odometry/filtered` + `/imu/data`.

---

## 18. Standalone Module Launch

Each module can be launched independently for development/testing:

```bash
# Simulation only (no perception/SLAM/Nav2)
ros2 launch ugv_sim sim.launch.py

# Simulation with custom world
ros2 launch ugv_sim sim.launch.py \
  world:=~/my_world.sdf  world_name:=my_world \
  spawn_x:=0  spawn_y:=0  spawn_yaw:=0

# Robot description only (URDF + TF)
ros2 launch ugv_description robot_state_publisher.launch.py

# Perception only (requires camera topics to be active)
ros2 launch ugv_bringup perception.launch.py camera_type:=sim

# SLAM only (requires /camera/* and /odometry/filtered)
ros2 launch ugv_bringup localization.launch.py

# Navigation only (requires /map and /odometry/filtered and /tf)
ros2 launch ugv_bringup navigation.launch.py

# RViz only
ros2 launch ugv_bringup rviz.launch.py
```

---

## 19. Troubleshooting

### SLAM map not appearing

1. Wait ≥ 45 s — Gazebo starts camera at ~15s, RTAB-Map needs a few more seconds
2. `ros2 topic hz /camera/depth/image_raw` → must be ~30 Hz
3. `ros2 topic hz /rtabmap/rgbd_image` → must be ≥ 5 Hz
4. `ros2 topic echo /map --no-arr --once` → must print data
5. Force reset: `ros2 service call /rtabmap/reset std_srvs/srv/Empty`

### Gazebo crashes on launch

The old `tugbot_warehouse.sdf` downloads 8+ Fuel models via HTTP — a timeout causes abort. Solution: use `ugv_test_arena.sdf` (default) which has zero Fuel dependencies.

### Robot not appearing in Gazebo

The robot is now spawned 3 seconds after Gazebo starts via `ros_gz_sim create`. If it doesn't appear:
1. Check `ls install/ugv_sim/share/ugv_sim/models/localbot/` — must have `model.sdf`
2. If missing: `colcon build --packages-select ugv_sim`
3. Check the spawn_node output for errors (topic creation timeout means Gazebo wasn't ready)

### Robot stops mid-navigation

1. `ros2 topic hz /odometry/filtered` → must be ~20 Hz
2. `ros2 run tf2_ros tf2_echo map odom` → must succeed
3. Check goal reachability — robot may be genuinely blocked
4. `ros2 topic echo /cmd_vel --once` → check non-zero values are published

### Custom world: sensors not working

Verify your world SDF has:
```xml
<plugin name='ignition::gazebo::systems::Sensors' filename='ignition-gazebo-sensors-system'>
  <render_engine>ogre2</render_engine>   <!-- ogre v1 does NOT support depth_camera -->
</plugin>
<plugin name='ignition::gazebo::systems::Imu' filename='ignition-gazebo-imu-system'/>
```
And verify `world_name:=` matches `<world name="...">` in your SDF.

### RViz robot model flickers white

Caused by `robot_state_publisher` receiving unknown joint names → TF breaks → RViz shows white mesh for 1 frame.  
Fix: `joint_state_relay.py` filters to URDF wheel joints only. If adding joints, also add them to `URDF_WHEEL_JOINTS` in the relay.

### Wrong camera frame in depth images

Check `ros2 topic echo /camera/depth/image_raw --no-arr --once | grep frame_id`. Must be `camera_link_optical`. If not, verify the bridge is running and `world_name`/`robot_name` match the actual world/model names in Gazebo.

---

## 20. Debugging Commands

```bash
source install/setup.bash

# ── Topic health ──────────────────────────────────────────────────────────────
ros2 topic hz /camera/image_raw          # expect 30 Hz
ros2 topic hz /camera/depth/image_raw    # expect 30 Hz
ros2 topic hz /perception/depth/points  # expect 30 Hz
ros2 topic hz /rtabmap/rgbd_image        # expect 5-15 Hz
ros2 topic hz /odometry/filtered         # expect 20 Hz
ros2 topic hz /map                       # expect ~1 Hz (TL — may show 0 Hz)
ros2 topic hz /joint_states_urdf         # expect 50 Hz

# ── QoS diagnosis ─────────────────────────────────────────────────────────────
ros2 topic info /map --verbose                        # check Transient Local
ros2 topic info /camera/image_raw --verbose           # check BestEffort
ros2 topic info /perception/depth/image_raw --verbose # check Reliable

# ── TF tree ───────────────────────────────────────────────────────────────────
ros2 run tf2_tools view_frames        # generates frames.pdf
ros2 run tf2_ros tf2_echo map odom    # SLAM transform
ros2 run tf2_ros tf2_echo odom base_footprint  # wheel odometry

# ── Nav2 state ────────────────────────────────────────────────────────────────
ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server
ros2 lifecycle get /bt_navigator

# ── SLAM ──────────────────────────────────────────────────────────────────────
ros2 param get /rtabmap map_always_update      # must be True
ros2 param get /rtabmap RGBD/LinearUpdate      # must be 0.0
ros2 service call /rtabmap/reset std_srvs/srv/Empty

# ── Joint states ──────────────────────────────────────────────────────────────
ros2 topic echo /joint_states --no-arr --once      # from GZ bridge (BestEffort)
ros2 topic echo /joint_states_urdf --no-arr --once # from relay (Reliable)

# ── World / spawn verification ────────────────────────────────────────────────
ros2 topic list | grep world             # should show GZ bridge topics with world_name
ros2 topic list | grep localbot          # should show model-level topics

# ── Dynamic bridge config verification ────────────────────────────────────────
ls /tmp/ugv_*_gz_bridge.yaml             # temp file generated by sim.launch.py
cat /tmp/ugv_*_gz_bridge.yaml            # verify world_name and robot_name are correct

# ── Goal sending ──────────────────────────────────────────────────────────────
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  '{pose: {header: {frame_id: "map"}, pose: {position: {x: 0.0, y: 6.0, z: 0.0}, orientation: {w: 1.0}}}}'

# ── Clean kill ────────────────────────────────────────────────────────────────
pkill -9 -f "gz sim|ros2 launch|parameter_bridge|rtabmap|rviz2|yolo_seg|depth_relay"
```

---

## 21. How to Modify Each Module Safely

### Add a new world

1. Copy `src/ugv_sim/worlds/empty_world.sdf` as your template
2. Change `<world name="empty">` to `<world name="your_name">`
3. Add obstacles inside the world (copy the template shown in the file)
4. **Do NOT add the robot** — it is spawned via launch args
5. Rebuild: `colcon build --packages-select ugv_sim`
6. Launch: `ros2 launch ugv_bringup ugv_complete.launch.py world:=... world_name:=your_name`

### Modify Localbot geometry or sensors

1. Edit `src/ugv_sim/models/localbot/model.sdf`
2. If adding a sensor link: also add the link/joint to `src/ugv_description/urdf/sensors/`
3. Bridge config is generated dynamically — no YAML editing needed for same-named links
4. If changing link names: update `_make_bridge_yaml()` in `sim.launch.py`
5. Rebuild: `colcon build --packages-select ugv_sim ugv_description`

### Swap the Nav2 planner

In `nav2_params.yaml` change:
```yaml
planner_server:
  planner_plugins: ["GridBased"]
  GridBased:
    plugin: "nav2_smac_planner::SmacPlanner2D"  # swap here
```
Options: `SmacPlanner2D`, `SmacPlannerHybrid`, `NavfnPlanner`, `ThetaStarPlanner`

### Change SLAM algorithm

Replace RTAB-Map with another SLAM by modifying `localization.launch.py`. The contract is: publish `/map` (Reliable + TransientLocal OccupancyGrid) and `map→odom` TF.

### Add a new depth source

1. Create relay node in `src/ugv_perception/ugv_perception/mynew_relay_node.py`
2. Add entry_point in `src/ugv_perception/setup.py`
3. Add `<exec_depend>` in `src/ugv_perception/package.xml`
4. Add conditional node to `perception.launch.py` with `EqualsSubstitution(camera_type, 'mynew')`
5. Rebuild: `colcon build --packages-select ugv_perception ugv_bringup`

### Use external Nav2 config

```bash
# No rebuild needed — just pass the file path
ros2 launch ugv_bringup ugv_complete.launch.py nav2_params_file:=/path/to/my_params.yaml
```

### Use external RViz config

```bash
ros2 launch ugv_bringup ugv_complete.launch.py rviz_config:=/path/to/my_config.rviz
```

---

## 22. Known Limitations

| Limitation | Impact | Workaround |
|------------|--------|------------|
| Front-facing camera only | No side/rear obstacle detection | Larger `inflation_radius`; lower speed |
| SLAM at ~1 Hz | RPP needs `transform_tolerance: 1.0` | Already configured |
| Depth Anything V3 = relative depth | No metric scale | Use `realsense` or `zed` in production |
| Gazebo startup ~15s | Camera unavailable first 15s | Wait; expected behavior |
| `world_name` must match world SDF `name` attribute | Sensors silent if mismatch | Bridge config shows correct expected topics in `/tmp/ugv_*_gz_bridge.yaml` |
| Teleop GUI widget hardcodes `/model/localbot/cmd_vel` | GUI teleop breaks if `robot_name!=localbot` | Edit Teleop plugin topic in world SDF or use ROS teleop: `ros2 run teleop_twist_keyboard teleop_twist_keyboard` |

---

## 23. Change History

| Commit | Fix | Root Cause |
|--------|-----|-----------|
| `57e33de` | SLAM never receives depth | GZ depth topic is `depth_image` not `image` |
| `ce3f06c` | Wrong terrain back-projection | Hardcoded `fx=616` vs actual Gazebo cam `fx≈421` |
| `e326bfb` | RViz fixed frame wrong | `map` frame not available until SLAM builds |
| `219b04e` | Tugbot warning light spinning | `warnign_light_joint` revolute at 10 rad/s → fixed type |
| `219b04e` | RViz robot flickering white | Relay passed non-URDF joints → RSP "unknown joint" → TF crash |
| `3f856dd` | `/map` never published | RTAB-Map waited for 0.1m movement → `RGBD/LinearUpdate: "0.0"` |
| `72d669b` | All model.sdf changes ignored | World used Fuel URL, bypassing local model dir |
| `72d669b` | Robot stops mid-navigation | `failure_tolerance: 0.3` aborted on brief TF hiccups |
| `e002329` | **URDF↔SDF wheel mismatch** | TugBot `wheel_radius=0.195m` but URDF says 0.1m |
| `e002329` | **Fuel/internet dependency for robot** | Created **Localbot** — zero Fuel dependency |
| `e002329` | **Joint name mismatch** | TugBot reversed names → relay simplified to QoS-only |
| `2370418` | **Gazebo crash on launch** | Old world downloaded 8+ Fuel models via HTTP |
| `2370418` | **Sensors/IMU at model level** | World-level systems duplicated → sensor instability |
| **current** | **World not modular** | Bridge hardcoded `world_demo`/`localbot` → **OpaqueFunction + dynamic YAML** |
| **current** | **Robot baked into world SDF** | Can't use any world → **ros_gz_sim create spawner** |
| **current** | **No spawn position control** | → **spawn_x/y/z/yaw args exposed at top level** |
| **current** | **Nav2 config not runtime-swappable** | → **nav2_params_file arg** |
| **current** | **No empty world template** | → **empty_world.sdf with template comments** |
