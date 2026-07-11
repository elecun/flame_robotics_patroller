import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import argparse

from work_planner import WorkPlanner
from global_planners import LookaheadPathTracker, WaypointFollower
from local_planners import ReactiveAckermannPlanner, BlindPursuitPlanner, AckermannDWA, ElasticBandPlanner, ShootingMPCPlanner, VFHPursuitPlanner
from utils import meters_to_latlon, latlon_to_meters

# Simulation settings
DT = 0.1 # time step (seconds)
WHEELBASE = 1.5
ROBOT_WIDTH = 1.0
MAX_STEERING = 0.5
CRUISE_SPEED = 2.5
MAX_MAST_SPEED = 0.5

# Reference point for lat/lon conversion
LAT_REF, LON_REF = 37.5, 127.0

def simulate_lidar(robot_x, robot_y, robot_heading, obstacles):
    local_pts = []
    cos_h, sin_h = math.cos(robot_heading), math.sin(robot_heading)
    for ox, oy in obstacles:
        dx, dy = ox - robot_x, oy - robot_y
        lx, ly = dx * cos_h + dy * sin_h, -dx * sin_h + dy * cos_h
        if math.sqrt(lx**2 + ly**2) < 15.0:
            local_pts.append([lx, ly])
    return np.array(local_pts)

def main():
    parser = argparse.ArgumentParser(description="Ackermann 3D Navigation Simulation")
    parser.add_argument('--planner', type=str, default='dwa', choices=['apf', 'blind', 'dwa', 'eb', 'mpc', 'vfh'])
    args = parser.parse_args()

    # 1. Define Waypoints in local meters (x, y, z)
    local_waypoints = [(0, 0, 0.0), (20, 0, 1.0), (30, 10, 2.0), (30, 30, 1.5), (10, 30, 0.5), (0, 20, 0.0)]
    global_waypoints = [(*meters_to_latlon(LAT_REF, LON_REF, x, y), z) for x, y, z in local_waypoints]
    
    obstacles_local = [(10, -1), (10, 0), (10, 1), (30, 20), (29, 20), (20, 30)]
    
    # 2. Initialize Planners
    global_planner = WaypointFollower([], threshold_distance=3.0)
    work_planner = WorkPlanner(global_waypoints, global_planner)
    
    planners = {
        'apf': ReactiveAckermannPlanner(WHEELBASE, MAX_STEERING, CRUISE_SPEED, MAX_MAST_SPEED),
        'blind': BlindPursuitPlanner(WHEELBASE, MAX_STEERING, CRUISE_SPEED, MAX_MAST_SPEED),
        'dwa': AckermannDWA(WHEELBASE, ROBOT_WIDTH, MAX_STEERING, CRUISE_SPEED, MAX_MAST_SPEED),
        'eb': ElasticBandPlanner(WHEELBASE, ROBOT_WIDTH, MAX_STEERING, CRUISE_SPEED, MAX_MAST_SPEED),
        'mpc': ShootingMPCPlanner(WHEELBASE, ROBOT_WIDTH, MAX_STEERING, CRUISE_SPEED, MAX_MAST_SPEED),
        'vfh': VFHPursuitPlanner(WHEELBASE, ROBOT_WIDTH, MAX_STEERING, CRUISE_SPEED, MAX_MAST_SPEED)
    }
    local_planner = planners[args.planner]
    
    # 3. Initialize Robot State
    rx, ry, rz = 0.0, 0.0, 0.0
    r_heading, r_speed, r_steering, r_z_speed = 0.0, 0.0, 0.0, 0.0
    
    path_x, path_y, path_z = [], [], []
    
    fig, ax = plt.subplots(figsize=(8, 8))
    
    def update(frame):
        nonlocal rx, ry, rz, r_heading, r_speed, r_steering, r_z_speed
        
        current_lat, current_lon = meters_to_latlon(LAT_REF, LON_REF, rx, ry)
        lidar_data = simulate_lidar(rx, ry, r_heading, obstacles_local)
        
        # A. Work Planner -> Global Planner
        target_lat, target_lon, target_z = work_planner.step(current_lat, current_lon, rz)
        
        # B. Local Planner
        t_steer, t_speed, t_z_speed = local_planner.compute_commands(
            target_lat, target_lon, target_z,
            current_lat, current_lon, rz,
            r_heading, r_speed, r_steering, r_z_speed,
            lidar_data
        )
        
        # C. Dynamics
        r_steering, r_speed, r_z_speed = t_steer, t_speed, t_z_speed
        rx += r_speed * math.cos(r_heading) * DT
        ry += r_speed * math.sin(r_heading) * DT
        rz += r_z_speed * DT
        r_heading += (r_speed / WHEELBASE) * math.tan(r_steering) * DT
        
        path_x.append(rx)
        path_y.append(ry)
        path_z.append(rz)
        
        # D. Plotting
        ax.clear()
        ax.set_xlim(-5, 40)
        ax.set_ylim(-5, 40)
        ax.set_aspect('equal')
        ax.set_title(f"3D Navigation Simulation (Planner: {args.planner.upper()})\nCurrent Z: {rz:.2f}m / Target Z: {target_z:.2f}m")
        
        wx = [p[0] for p in local_waypoints]
        wy = [p[1] for p in local_waypoints]
        ax.plot(wx, wy, 'g--', label="Global Path")
        
        # Visualize waypoint Z-height by label
        for i, (x, y, z) in enumerate(local_waypoints):
            ax.scatter(x, y, c='g')
            ax.text(x, y+1, f"Z={z:.1f}m", color='green', fontsize=8)
            
        ox = [p[0] for p in obstacles_local]
        oy = [p[1] for p in obstacles_local]
        ax.scatter(ox, oy, c='r', marker='x', label="Obstacles")
        
        ax.plot(path_x, path_y, 'b-', label="Robot Path")
        ax.arrow(rx, ry, 2.0*math.cos(r_heading), 2.0*math.sin(r_heading), head_width=1.0, head_length=1.0, fc='b', ec='b')
        
        # Draw a circle representing the robot with color mapping to Z height
        circle = plt.Circle((rx, ry), 1.0, color=plt.cm.viridis(rz / 2.0), alpha=0.5)
        ax.add_patch(circle)
        
        tx, ty = latlon_to_meters(LAT_REF, LON_REF, target_lat, target_lon)
        ax.scatter([tx], [ty], c='m', marker='*', s=100, label="Next Target")
        
        ax.legend(loc='upper right')
        
    ani = animation.FuncAnimation(fig, update, frames=300, interval=50, repeat=False)
    plt.show()

if __name__ == '__main__':
    main()
