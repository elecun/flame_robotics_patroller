# Navigation Stack for Ackermann Mobile Robot

This directory contains the global and local planner modules for a mobile robot with an Ackermann steering structure and an independent Z-axis telescopic mast.
The robot configuration assumes a width of 1.0m and a wheelbase of 1.5m.

## Overview
The navigation stack is divided into three primary layers:
1. **Work Planner**: Manages high-level tasks, passing down 3D target sequences `(Latitude, Longitude, Z)`.
2. **Global Planner**: Interpolates and sequentially steps through the 3D waypoints provided by the Work Planner.
3. **Local Planner**: Computes steering, base speed, and mast Z-speed to safely navigate towards the target `(Lat, Lon, Z)` while locally avoiding obstacles detected by Ouster VLP-16 LiDAR.

All planners use abstract base classes defined in `planner_base.py`.

## Module Structure

### Work Planner (`work_planner.py`)
- **`WorkPlanner`**: Accepts a sequence of tasks and delegates them to the Global Planner. 

### Abstract Base Classes (`planner_base.py`)
- `BaseGlobalPlanner`: Enforces the 3D `get_next_target(current_lat, current_lon, current_z)` interface.
- `BaseLocalPlanner`: Enforces the 3D `compute_commands(...)` interface, which now outputs `(target_steering, target_speed, target_z_speed)`.

### Global Planners (`global_planners.py`)
- **`WaypointFollower`**: Targets the next waypoint in the 3D list.
- **`LookaheadPathTracker`**: Searches for a lookahead point along the path.

### Local Planners (`local_planners.py`)
Multiple advanced local planners are implemented, all now supporting Z-axis P-control alongside X/Y obstacle avoidance:
1. **`AckermannDWA`** (Dynamic Window Approach): Samples velocities and steering angles.
2. **`ElasticBandPlanner`** (Elastic Band): Deformable path tracking.
3. **`ShootingMPCPlanner`** (Model Predictive Control): Forward-shooting sequence generation.
4. **`VFHPursuitPlanner`** (VFH + Pure Pursuit): Polar histogram obstacle avoidance.
5. **`ReactiveAckermannPlanner`** (APF): Baseline Artificial Potential Field.
6. **`BlindPursuitPlanner`**: Baseline Pure Pursuit without obstacle avoidance.

### Utilities (`utils.py`)
Helper functions for coordinate conversions.

## Simulation
A standalone simulation environment is provided in `simulation.py`. The simulation animates the robot traversing the path and visually indicates the Z-axis height using color mapping.

### Running the Simulation
Dependencies:
- `numpy`
- `matplotlib`

Run the script and select the desired planner using the `--planner` argument (`dwa`, `eb`, `mpc`, `vfh`, `apf`, `blind`):

```bash
python3 experimental/drive/simulation.py --planner dwa
```