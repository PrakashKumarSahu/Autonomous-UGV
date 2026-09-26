# Parameter Tuning Guide for Outdoor Unstructured Terrain

## 1. Nav2 Regulated Pure Pursuit Controller
- `desired_linear_vel`: Nominal forward cruise speed ($0.8\text{ m/s}$).
- `lookahead_dist`: Nominal lookahead distance ($1.2\text{ m}$).
- `use_regulated_linear_velocity_scaling`: Set to `true` to ensure the UGV slows down during sharp turns on loose dirt.
- `use_cost_regulated_linear_velocity_scaling`: Set to `true` to slow the vehicle down when approaching rock and ditch boundaries.

## 2. Nav2 Smac Hybrid-A* Planner
- `minimum_turning_radius`: Set to physical chassis turn limit (e.g. $0.4\text{ m}$).
- `non_straight_penalty`: Penalty weight for turns ($1.2$) ensuring the vehicle prefers straight segments across open terrain.
- `cost_penalty`: Multiplier for traveling near hazard borders ($2.0$).

## 3. 2.5D Terrain Analysis (`ugv_terrain`)
- `max_step_height`: Vertical threshold to classify a ground ledge/rock as an impassable obstacle ($0.22\text{ m}$).
- `max_slope_angle`: Critical slope incline ($25.0^\circ$). Slopes steeper than this limit receive maximum lethal cost.
- `grid_resolution`: Resolution of the rolling traversability costmap ($0.10\text{ m}$).
