# Autonomous UGV: System Architecture, Configuration & Operations Guide

## 1. Executive Summary

This document provides a comprehensive operational and architectural reference for the **Autonomous Unmanned Ground Vehicle (UGV)** software stack. The system is deployed on **ROS 2 Jazzy** running under **Ubuntu 24.04 LTS** and simulated in **Gazebo Harmonic (GZ Sim 8.11)** with **NVIDIA CUDA hardware acceleration**.

The autonomy stack integrates:
- **Neural Perception**: Zero-shot monocular metric depth estimation (Depth Anything V2) and semantic hazard detection (YOLOv8-seg).
- **Terrain Traversability Mapping**: Real-time 2.5D elevation, slope, and step-height traversability analysis (`ugv_terrain`).
- **State Estimation & SLAM**: Dual-mode localization featuring direct simulation TF/odometry bridging and RTAB-Map 2D graph SLAM with loop closure.
- **Nav2 Autonomy Engine**: Global SmacPlanner2D path planning with Regulated Pure Pursuit path tracking and multi-layer costmaps.

---

## 2. Git Repository Branches & Worktree Topology

The project repository utilizes Git worktrees to maintain isolated development, planning, and debugging environments across concurrent features.

```
/home/ros2/Autonomous-UGV                                                             [main]
/home/ros2/.gemini/antigravity/worktrees/Autonomous-UGV/plan_modular_implementation   [plan_modular_implementation]
/home/ros2/.gemini/antigravity/worktrees/Autonomous-UGV/debug_pipeline_nav2_rotation  [debug_pipeline_nav2_rotation] (ACTIVE PRODUCTION)
```

### Branch Profiles

| Branch Name | Worktree Path | Commit Hash | Purpose & Status |
| :--- | :--- | :--- | :--- |
| `debug_pipeline_nav2_rotation` | `.../worktrees/Autonomous-UGV/debug_pipeline_nav2_rotation` | `78244fc` | **Active Verified Branch**: Contains all verified fixes for physical motor torque, symmetrical wheel joint axes, Gazebo TF bridge, RTAB-Map ground filtering, RViz costmap rendering, and Nav2 waypoint navigation. |
| `plan_modular_implementation` | `.../worktrees/Autonomous-UGV/plan_modular_implementation` | `258ec94` | **Architectural Staging**: Modular refactoring workspace merging main pipeline updates with structured node wrappers. |
| `main` | `/home/ros2/Autonomous-UGV` | `df61bd9` | **Repository Root**: Stable baseline branch prior to rotation deadlock and traction resolution. |

### Upstreaming Workflow

To synchronize verified fixes from `debug_pipeline_nav2_rotation` back into `main`:
```bash
cd /home/ros2/Autonomous-UGV
git merge debug_pipeline_nav2_rotation --ff-only
# Or standard merge if fast-forward is not linear:
# git merge debug_pipeline_nav2_rotation -m "merge: integrate verified nav2 rotation and pipeline sync fixes"
```

---

## 3. System Architecture & Inter-System Interaction

### High-Level Architectural Flow

```mermaid
flowchart TD
    subgraph SimOrHW["Hardware / Simulation Layer"]
        GZ["Gazebo Harmonic / Real UGV"]
        DIFF["DiffDrive Controller & Motor Actuators"]
        CAM["Front RGB-D Camera (Color + Depth)"]
        IMU["6-DOF IMU Sensor"]
    end

    subgraph Perception["Perception Pipeline (ugv_perception)"]
        DEPTH["Depth Node (Depth Anything V2)"]
        YOLO["Hazard Node (YOLOv8-Seg)"]
        PC["PointCloud XYZ Generator"]
    end

    subgraph Terrain["Terrain Analysis (ugv_terrain)"]
        ELEV["Elevation Grid & Slope Filter"]
        TRAV["Traversability Grid Publisher"]
    end

    subgraph SLAM_EKF["State Estimation & SLAM (ugv_bringup)"]
        BRIDGE["ros_gz_bridge (TF & Odom)"]
        RTAB["RTAB-Map SLAM (2D Planar Mode)"]
        EKF["robot_localization EKF (HW Mode)"]
    end

    subgraph Nav2["Nav2 Navigation Engine (ugv_bringup)"]
        BT["BT Navigator Engine"]
        SMAC["SmacPlanner2D (Global Path)"]
        RPP["Regulated Pure Pursuit Controller"]
        GCOST["Global Costmap (Map + Inscribed Footprint)"]
        LCOST["Local Costmap (Traversability Grid)"]
    end

    CAM -->|/camera/image_raw| DEPTH
    CAM -->|/camera/image_raw| YOLO
    DEPTH -->|/perception/depth/image_raw| PC
    DEPTH -->|/perception/depth/camera_info| PC
    DEPTH -->|RGBD Sync| RTAB
    PC -->|/perception/depth/points| ELEV
    YOLO -->|/perception/hazard_mask| ELEV
    ELEV -->|/terrain/traversability_grid| LCOST

    GZ -->|/world/world_demo/clock| BRIDGE
    DIFF -->|/model/tugbot/odometry| BRIDGE
    DIFF -->|/model/tugbot/tf| BRIDGE

    BRIDGE -->|/odometry/filtered| RTAB
    BRIDGE -->|/odometry/filtered| RPP
    BRIDGE -->|/tf (odom -> base_footprint)| Nav2
    RTAB -->|/map OccupancyGrid| GCOST
    RTAB -->|/tf (map -> odom)| Nav2

    BT --> SMAC
    BT --> RPP
    SMAC -->|Path| RPP
    RPP -->|/cmd_vel| DIFF
```

### Complete Coordinate Frame Tree (TF2)

The coordinate transformations are unified and continuous:

```mermaid
flowchart LR
    map["map (Global Fixed Frame)"] -->|RTAB-Map SLAM (1-2 Hz)| odom["odom (Continuous Odometry Frame)"]
    odom -->|Gazebo Bridge / EKF (20-50 Hz)| base_footprint["base_footprint (Ground Projection)"]
    base_footprint -->|URDF Static TF| base_link["base_link (Chassis Origin)"]
    base_link -->|robot_state_publisher| left_wheel["left_wheel_link"]
    base_link -->|robot_state_publisher| right_wheel["right_wheel_link"]
    base_link -->|robot_state_publisher| front_caster["front_caster_wheel_link"]
    base_link -->|robot_state_publisher| rear_caster["rear_caster_wheel_link"]
    base_link -->|robot_state_publisher| camera_link["camera_link"]
    camera_link -->|URDF Optical TF| camera_opt["camera_link_optical"]
    base_link -->|robot_state_publisher| imu_link["imu_link"]
```

---

## 4. Key Topic Communication & Synchronization Matrix

| Topic Name | Message Type | Rate | Source Node | Target Subsystems | Synchronization & QoS |
| :--- | :--- | :--- | :--- | :--- | :--- |
| `/clock` | `rosgraph_msgs/msg/Clock` | Sim Rate (~1 kHz) | `ros_gz_bridge` | All Nodes | System-wide simulation time clock (`use_sim_time: true`). |
| `/tf` | `tf2_msgs/msg/TFMessage` | 20-50 Hz | `ros_gz_bridge`, `rtabmap`, `robot_state_publisher` | All TF Listeners | Volatile, Reliable, Queue 100. |
| `/odometry/filtered` | `nav_msgs/msg/Odometry` | 20.0 Hz | `ros_gz_bridge` (Sim) / `ekf_node` (HW) | `controller_server`, `rtabmap` | Continuous SE(2) odometry. |
| `/camera/image_raw` | `sensor_msgs/msg/Image` | 30.0 Hz | `ros_gz_bridge` (Sim) / Camera HW | `depth_node`, `yolo_seg_node`, `rgbd_sync` | SensorData QoS (Best Effort). |
| `/camera/camera_info`| `sensor_msgs/msg/CameraInfo` | 30.0 Hz | `ros_gz_bridge` (Sim) / Camera HW | `depth_node`, `rgbd_sync` | SensorData QoS (Best Effort). |
| `/perception/depth/image_raw` | `sensor_msgs/msg/Image` | ~15-20 Hz | `depth_node` | `point_cloud_xyz_node`, `rgbd_sync` | 32FC1 metric depth in meters. |
| `/perception/depth/camera_info`| `sensor_msgs/msg/CameraInfo`| ~15-20 Hz | `depth_node` | `point_cloud_xyz_node` | Synchronized header and intrinsics with depth image. |
| `/perception/depth/points` | `sensor_msgs/msg/PointCloud2` | ~15 Hz | `point_cloud_xyz_node` | `terrain_analysis_node` | 3D organized Cartesian points in `camera_link_optical`. |
| `/terrain/traversability_grid` | `nav_msgs/msg/OccupancyGrid` | 10.0 Hz | `terrain_analysis_node` | `local_costmap` | Local $12\times 12\text{ m}$ grid with footprint clearing. |
| `/map` | `nav_msgs/msg/OccupancyGrid` | 1.0 Hz | `rtabmap` | `global_costmap` | Transient Local, Reliable QoS, 2D SLAM ground truth. |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 20.0 Hz | `controller_server` | `ros_gz_bridge` / Motor HW | Bounded linear ($[-0.6, 1.2]\text{ m/s}$) and angular ($[-1.8, 1.8]\text{ rad/s}$). |

---

## 5. Detailed Package Configurations & Critical Fixes

### Package 1: `ugv_sim` (Simulation & Dynamics Engine)

#### Root Causes Fixed:
1. **Inverted Right Wheel Axis**:
   - `wheel_right_joint` had `<xyz>0 0 -1</xyz>` while `wheel_left_joint` had `<xyz>0 0 1</xyz>`. Any forward speed command drove the two wheels in opposite directions (spinning in place), while turning commands drove both wheels forward!
   - *Fix*: Aligned both drive joints to `<xyz>0 0 1</xyz>` in `src/ugv_sim/models/tugbot/model.sdf` and `~/.gz/fuel/.../tugbot/1/model.sdf`.
2. **Drive Motor Stall vs Caster Ground Friction**:
   - The Tugbot chassis weighs $46.2\text{ kg}$. The original model specified an effort limit of only $9.6\text{ N}\cdot\text{m}$ with caster friction $\mu = 1.16$ and joint damping $10.0$. Motors stalled immediately against ground friction.
   - *Fix*: Raised drive wheel joint effort to $150.0\text{ N}\cdot\text{m}$ (joint damping $0.1$, friction $0.0$) and decreased caster friction to $\mu = 0.01$ (damping $0.1$).
3. **Gazebo Harmonic Qt Quick Graphics Glitches**:
   - NVIDIA proprietary drivers produce severe horizontal/diagonal striping artifacts when Qt Quick uses threaded rendering.
   - *Fix*: Added `QSG_RENDER_LOOP=basic`, `QT_X11_NO_MITSHM=1`, `__NV_PRIME_RENDER_OFFLOAD=1`, and `__GLX_VENDOR_LIBRARY_NAME=nvidia` in `sim.launch.py`.

#### Core Configuration Files:
- [model.sdf](file:///home/ros2/.gemini/antigravity/worktrees/Autonomous-UGV/debug_pipeline_nav2_rotation/src/ugv_sim/models/tugbot/model.sdf#L895-L907):
  ```xml
  <plugin filename="ignition-gazebo-diff-drive-system" name="ignition::gazebo::systems::DiffDrive">
      <left_joint>wheel_left_joint</left_joint>
      <right_joint>wheel_right_joint</right_joint>
      <wheel_separation>0.5605</wheel_separation>
      <wheel_radius>0.195</wheel_radius>
      <odom_publish_frequency>20</odom_publish_frequency>
      <max_linear_acceleration>5.0</max_linear_acceleration>
      <max_angular_acceleration>5.0</max_angular_acceleration>
      <frame_id>odom</frame_id>
      <child_frame_id>base_footprint</child_frame_id>
  </plugin>
  ```
- [gazebo_bridge.yaml](file:///home/ros2/.gemini/antigravity/worktrees/Autonomous-UGV/debug_pipeline_nav2_rotation/src/ugv_sim/config/gazebo_bridge.yaml#L19-L36):
  Directly bridges simulation clock (`/world/world_demo/clock`), odometry (`/model/tugbot/odometry`), and TF (`/model/tugbot/tf`) to eliminate clock deadlocks.

---

### Package 2: `ugv_perception` (Neural Metric Depth & Hazards)

#### Root Causes Fixed:
1. **Monocular Depth Scale Jumps**:
   - Depth Anything V2 estimates relative disparity. Per-frame min/max normalization caused depth breathing and scale jumping when the camera faced flat corridors.
   - *Fix*: Implemented an Exponential Moving Average (EMA, $\alpha=0.25$) on running disparity bounds, producing temporally stable metric depth.
2. **Synchronization Drop in `depth_image_proc`**:
   - When CameraInfo was delayed, `point_cloud_xyz_node` dropped pairs.
   - *Fix*: `depth_node.py` now publishes synchronized `CameraInfo` matching the exact timestamp and frame ID of every generated depth frame.

---

### Package 3: `ugv_terrain` (2.5D Traversability Grid)

#### Root Causes Fixed:
1. **RViz Blinding White Glare**:
   - `terrain_analysis_node.py` initialized unobserved cells to `0` (free/white). Because the local analysis window is $12\times 12\text{ m}$, this rendered a massive white opaque plane blinding the operator.
   - *Fix*: Initialized unobserved cells to `-1` (unknown, rendered transparent in RViz) and configured RViz displays to `Color Scheme: costmap` with `Draw Behind: true`.
2. **Immediate Footprint Collision Clearance**:
   - Clears a $0.40\text{ m}$ radius circle under the robot center to ensure the local planner never aborts on its own footprint.

---

### Package 4: `ugv_bringup` (Localization & RTAB-Map SLAM)

#### Root Causes Fixed:
1. **Corridor Visual Odometry Jump & Relocalization Deadlock**:
   - `RGBD/NeighborLinkRefining: true` and `RGBD/ProximityBySpace: true` attempted visual alignment in symmetrical warehouse corridors, triggering false loop closures that shifted the map by 15 meters.
   - *Fix*: Disabled neighbor link refining and proximity by space. SLAM uses continuous odometry from `/odometry/filtered` for graph constraints.
2. **False Ground Obstacle Projection**:
   - Default `Grid/NormalsSegmentation: true` treated slight depth curvature on flat warehouse floors as steep obstacle walls.
   - *Fix*: Set `Grid/NormalsSegmentation: false` with height threshold `Grid/MaxGroundHeight: 0.12`, `Grid/NoiseFilteringRadius: 0.15`, and `GridGlobal/FootprintRadius: 0.55`.

---

### Package 5: `ugv_bringup` (Nav2 Autonomous Navigation)

#### Root Causes Fixed:
1. **"Start Occupied" Abort on Re-planning**:
   - When sending sequential goals or angled turns, costmap inflation layers inflated small residual floor noise under the robot radius.
   - *Fix*: Added `footprint_clearing_enabled: true` to both `terrain_layer` (local costmap) and `static_layer` (global costmap).
2. **Controller Deadlock on Turn-in-Place**:
   - Pure Pursuit controller stalled when orientation heading differed by more than $45^\circ$.
   - *Fix*: Configured `use_rotate_to_heading: true` with `rotate_to_heading_min_angle: 0.85` ($48.7^\circ$) and velocity-scaled lookahead distance ($[0.45, 1.6]\text{ m}$).

---

## 6. How to Run the System

### Prerequisites & Build

Source ROS 2 Jazzy and build all workspace packages:
```bash
cd /home/ros2/.gemini/antigravity/worktrees/Autonomous-UGV/debug_pipeline_nav2_rotation
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### Launch Mode 1: Full Simulation Autonomy (Gazebo Harmonic + AI + Nav2 + RViz)

```bash
ros2 launch ugv_bringup ugv_complete.launch.py mode:=sim
```

**What this launches:**
1. Gazebo Harmonic with `tugbot_warehouse.sdf` and corrected physics.
2. `ros_gz_bridge` bridging clock, TF, odometry, sensors, and velocity commands.
3. Depth Anything V2 neural depth estimation and YOLOv8 segmentation on CUDA GPU.
4. 2.5D Elevation & Traversability mapping (`/terrain/traversability_grid`).
5. RTAB-Map 2D SLAM publishing global `/map` and `map -> odom` transform.
6. Full Nav2 stack (SmacPlanner2D, Regulated Pure Pursuit, Behaviors, BT Navigator).
7. RViz2 preconfigured dashboard with costmap coloring and traversability layers.

### Launch Mode 2: Physical Hardware Autonomy (`mode:=hw`)

When deploying on the physical ground vehicle:
```bash
ros2 launch ugv_bringup ugv_complete.launch.py mode:=hw
```
- In `mode:=hw`, Gazebo Harmonic and simulation bridges are disabled.
- `robot_localization` EKF node is automatically activated to fuse physical wheel encoders (`/diff_drive_controller/odom`) with the onboard 6-axis IMU (`/imu/data`).

### Optional Launch Arguments

| Parameter | Type | Default | Description |
| :--- | :--- | :--- | :--- |
| `mode` | `string` | `sim` | `sim` for Gazebo Harmonic, `hw` for physical robot hardware. |
| `use_sim_time` | `bool` | `true` | Synchronize all nodes with `/clock`. |
| `enable_rviz` | `bool` | `true` | Launch RViz2 operator dashboard. |
| `enable_rtabmap`| `bool` | `true` | Enable RTAB-Map SLAM for mapping and loop closure. |

---

## 7. How to Send Goals & Verification

### Option A: Via RViz2 GUI
Click the **Nav2 Goal** tool in the RViz toolbar, click on any open aisle in the warehouse, and drag the arrow in the desired heading direction.

### Option B: Automated Multi-Waypoint Verification Suite
Run the verified automated test script which tests straight traversal and multi-angle sequential waypoints:
```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 /home/ros2/.gemini/antigravity/brain/438b36ec-9aef-43cc-9c42-b567696b147b/scratch/verify_multi_goal.py
```

**Expected Output:**
```
[INIT] Waiting for Action Server...
[INIT] Waiting for map -> base_footprint transform...
[START] Position: x=1.649, y=0.124, yaw=27.5°

[GOAL] Sending: x=2.54, y=0.58, yaw=27.5°
[RESULT] Status: 4 (SUCCEEDED=4), FinalPose: (2.24, 0.43), DistToGoal: 0.336m, MaxLinV: 0.60m/s

[GOAL] Sending: x=2.83, y=1.24, yaw=53.4°
[RESULT] Status: 4 (SUCCEEDED=4), FinalPose: (2.68, 0.92), DistToGoal: 0.352m, MaxLinV: 0.60m/s

[GOAL] Sending: x=3.58, y=1.37, yaw=26.6°
[RESULT] Status: 4 (SUCCEEDED=4), FinalPose: (3.32, 1.29), DistToGoal: 0.267m, MaxLinV: 0.54m/s

=======================================================
Goal 1 (Straight):   PASS (dist=0.34m, max_v=0.60m/s)
Goal 2 (Turn Left):  PASS (dist=0.35m, max_v=0.60m/s)
Goal 3 (Turn Right): PASS (dist=0.27m, max_v=0.54m/s)
=======================================================
```

---

## 8. Troubleshooting & Diagnostics Reference

| Symptom | Root Cause | Verified Resolution |
| :--- | :--- | :--- |
| **Robot spins on spot instead of driving** | Inverted joint axis on right wheel (`xyz 0 0 -1`) or drive torque stall ($<10\text{ N}\cdot\text{m}$). | Symmetrical joint axis (`xyz 0 0 1`), effort raised to $150.0\text{ N}\cdot\text{m}$, caster friction $\mu = 0.01$. |
| **Gazebo GUI shows striped/glitched rendering** | NVIDIA driver incompatible with Qt Quick threaded render loops. | Add `QSG_RENDER_LOOP=basic` to launch environment. Do **not** use software rendering. |
| **RViz shows blinding white glare** | `terrain_analysis_node` initialized unobserved cells to `0` (free space) instead of `-1` (unknown). | Set unobserved initialization to `np.full(..., -1)` and configure display `Draw Behind: true`. |
| **Nav2 reports "Start occupied"** | RTAB-Map monocular depth noise placed cells with lethal cost on floor under the robot footprint. | Configure `footprint_clearing_enabled: true` in costmaps and set `Grid/NormalsSegmentation: false` with `GridGlobal/FootprintRadius: 0.55`. |
| **EKF hangs on "Waiting for clock to start..."** | `wait_until_started()` called before node executor spins in ROS 2 Jazzy `ekf_node`. | Bridge `/model/tugbot/tf` to `/tf` directly in simulation mode (`mode:=sim`); use EKF only for physical hardware (`mode:=hw`). |
| **Duplicate node warnings / Nav2 actions inactive** | Previous background tasks or orphaned processes left running. | Clean processes with `pkill -9 -f "nav2_\|ros2\|rtabmap\|gz"` before launching. |
