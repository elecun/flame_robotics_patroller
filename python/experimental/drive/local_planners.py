import math
import numpy as np
from typing import Tuple
from .planner_base import BaseLocalPlanner
from .utils import latlon_to_meters

def compute_z_speed(target_z: float, current_z: float, max_speed: float = 0.5) -> float:
    """Simple P-controller for the Z-axis mast."""
    error = target_z - current_z
    kp = 1.0
    v_z = kp * error
    return max(min(v_z, max_speed), -max_speed)

class BlindPursuitPlanner(BaseLocalPlanner):
    def __init__(self, wheelbase: float = 1.5, max_steering: float = 0.5, cruise_speed: float = 2.5, max_mast_speed: float = 0.5):
        self.wheelbase = wheelbase
        self.max_steering = max_steering
        self.cruise_speed = cruise_speed
        self.max_mast_speed = max_mast_speed

    def compute_commands(self, target_lat, target_lon, target_z, current_lat, current_lon, current_z, current_heading, current_speed, current_steering_angle, current_z_speed, lidar_data) -> Tuple[float, float, float]:
        dx, dy = latlon_to_meters(current_lat, current_lon, target_lat, target_lon)
        cos_h, sin_h = math.cos(current_heading), math.sin(current_heading)
        lx, ly = dx * cos_h + dy * sin_h, -dx * sin_h + dy * cos_h
        
        L_sq = lx**2 + ly**2
        steering = 0.0
        speed = 0.0
        if L_sq > 0:
            kappa = 2 * ly / L_sq
            steering = max(min(math.atan(kappa * self.wheelbase), self.max_steering), -self.max_steering)
            if not (lx < 0.1 and math.sqrt(L_sq) < 1.0):
                speed = self.cruise_speed * max(0.2, 1.0 - abs(steering) / self.max_steering)
                
        z_speed = compute_z_speed(target_z, current_z, self.max_mast_speed)
        return steering, speed, z_speed

class ReactiveAckermannPlanner(BaseLocalPlanner):
    def __init__(self, wheelbase=1.5, max_steering=0.5, cruise_speed=2.5, max_mast_speed=0.5, attract_gain=1.0, repulse_gain=2.0, influence_radius=5.0):
        self.wheelbase, self.max_steering, self.cruise_speed, self.max_mast_speed = wheelbase, max_steering, cruise_speed, max_mast_speed
        self.attract_gain, self.repulse_gain, self.influence_radius = attract_gain, repulse_gain, influence_radius

    def compute_commands(self, target_lat, target_lon, target_z, current_lat, current_lon, current_z, current_heading, current_speed, current_steering_angle, current_z_speed, lidar_data) -> Tuple[float, float, float]:
        dx, dy = latlon_to_meters(current_lat, current_lon, target_lat, target_lon)
        cos_h, sin_h = math.cos(current_heading), math.sin(current_heading)
        lx, ly = dx * cos_h + dy * sin_h, -dx * sin_h + dy * cos_h
        
        dist_to_target = math.sqrt(lx**2 + ly**2)
        f_att_x = self.attract_gain * (lx / dist_to_target) if dist_to_target > 0 else 0.0
        f_att_y = self.attract_gain * (ly / dist_to_target) if dist_to_target > 0 else 0.0
            
        f_rep_x, f_rep_y = 0.0, 0.0
        if lidar_data is not None and len(lidar_data) > 0:
            for pt in lidar_data:
                dist = math.sqrt(pt[0]**2 + pt[1]**2)
                if 0.1 < dist < self.influence_radius:
                    force_mag = self.repulse_gain * (1.0/dist - 1.0/self.influence_radius) / (dist**2)
                    f_rep_x -= force_mag * (pt[0]/dist)
                    f_rep_y -= force_mag * (pt[1]/dist)
                    
        target_v_x, target_v_y = f_att_x + f_rep_x, f_att_y + f_rep_y
        L_sq = target_v_x**2 + target_v_y**2
        steering = 0.0 if L_sq == 0 or target_v_x < 0 else math.atan(2 * target_v_y / L_sq * self.wheelbase)
        steering = max(min(steering, self.max_steering), -self.max_steering)
        speed = 0.0 if dist_to_target < 1.0 else self.cruise_speed * max(0.1, 1.0 - (abs(steering)/self.max_steering)*0.5)
        
        z_speed = compute_z_speed(target_z, current_z, self.max_mast_speed)
        return steering, speed, z_speed

class AckermannDWA(BaseLocalPlanner):
    def __init__(self, wheelbase=1.5, robot_width=1.0, max_steering=0.5, max_speed=2.5, max_mast_speed=0.5):
        self.wheelbase, self.robot_width = wheelbase, robot_width
        self.max_steering, self.max_speed, self.max_mast_speed = max_steering, max_speed, max_mast_speed
        self.dt = 0.2
        self.predict_time = 2.0

    def compute_commands(self, target_lat, target_lon, target_z, current_lat, current_lon, current_z, current_heading, current_speed, current_steering_angle, current_z_speed, lidar_data) -> Tuple[float, float, float]:
        dx, dy = latlon_to_meters(current_lat, current_lon, target_lat, target_lon)
        cos_h, sin_h = math.cos(current_heading), math.sin(current_heading)
        lx, ly = dx * cos_h + dy * sin_h, -dx * sin_h + dy * cos_h
        target_angle = math.atan2(ly, lx)
        
        v_samples = np.linspace(0.0, self.max_speed, 6)
        steer_samples = np.linspace(-self.max_steering, self.max_steering, 11)
        best_cost, best_v, best_steer = float('inf'), 0.0, 0.0
        for v in v_samples:
            for steer in steer_samples:
                cost = self._eval_trajectory(v, steer, target_angle, lidar_data)
                if cost < best_cost:
                    best_cost, best_v, best_steer = cost, v, steer
                    
        z_speed = compute_z_speed(target_z, current_z, self.max_mast_speed)
        return best_steer, best_v, z_speed
        
    def _eval_trajectory(self, v, steer, target_angle, lidar_data):
        x, y, theta = 0.0, 0.0, 0.0
        min_dist = float('inf')
        steps = int(self.predict_time / self.dt)
        for _ in range(steps):
            x += v * math.cos(theta) * self.dt
            y += v * math.sin(theta) * self.dt
            theta += (v / self.wheelbase) * math.tan(steer) * self.dt
            if lidar_data is not None and len(lidar_data) > 0:
                dists = np.sqrt((lidar_data[:,0] - x)**2 + (lidar_data[:,1] - y)**2)
                min_step_dist = np.min(dists)
                if min_step_dist < min_dist: min_dist = min_step_dist
                    
        heading_cost = abs(math.atan2(math.sin(target_angle - theta), math.cos(target_angle - theta)))
        clearance_cost = 10000.0 if min_dist < (self.robot_width/2.0 + 0.3) else 1.0/min_dist
        velocity_cost = self.max_speed - v
        return 1.0 * heading_cost + 2.0 * clearance_cost + 0.5 * velocity_cost

class ElasticBandPlanner(BaseLocalPlanner):
    def __init__(self, wheelbase=1.5, robot_width=1.0, max_steering=0.5, cruise_speed=2.5, max_mast_speed=0.5):
        self.wheelbase, self.max_steering, self.cruise_speed, self.max_mast_speed = wheelbase, max_steering, cruise_speed, max_mast_speed
        self.robot_width = robot_width

    def compute_commands(self, target_lat, target_lon, target_z, current_lat, current_lon, current_z, current_heading, current_speed, current_steering_angle, current_z_speed, lidar_data) -> Tuple[float, float, float]:
        dx, dy = latlon_to_meters(current_lat, current_lon, target_lat, target_lon)
        cos_h, sin_h = math.cos(current_heading), math.sin(current_heading)
        lx, ly = dx * cos_h + dy * sin_h, -dx * sin_h + dy * cos_h
        
        dist = math.sqrt(lx**2 + ly**2)
        z_speed = compute_z_speed(target_z, current_z, self.max_mast_speed)
        
        if dist < 0.5: return 0.0, 0.0, z_speed
        
        band = [np.array([lx * (i/5.0), ly * (i/5.0)]) for i in range(1, 6)]
        if lidar_data is not None and len(lidar_data) > 0:
            for pt in band:
                for obs in lidar_data:
                    d = np.linalg.norm(pt - obs)
                    if d < (self.robot_width/2.0 + 1.0):
                        pt += (pt - obs) / (d**3) * 0.5
        
        target_pt = band[0]
        L_sq = target_pt[0]**2 + target_pt[1]**2
        steering = max(min(math.atan(2*target_pt[1]/L_sq * self.wheelbase), self.max_steering), -self.max_steering) if L_sq > 0 and target_pt[0] > 0 else 0.0
        speed = self.cruise_speed * max(0.2, 1.0 - abs(steering)/self.max_steering)
        
        return steering, speed, z_speed

class ShootingMPCPlanner(BaseLocalPlanner):
    def __init__(self, wheelbase=1.5, robot_width=1.0, max_steering=0.5, cruise_speed=2.5, max_mast_speed=0.5):
        self.wheelbase, self.robot_width = wheelbase, robot_width
        self.max_steering, self.cruise_speed, self.max_mast_speed = max_steering, cruise_speed, max_mast_speed
        self.horizon = 10
        self.dt = 0.2

    def compute_commands(self, target_lat, target_lon, target_z, current_lat, current_lon, current_z, current_heading, current_speed, current_steering_angle, current_z_speed, lidar_data) -> Tuple[float, float, float]:
        dx, dy = latlon_to_meters(current_lat, current_lon, target_lat, target_lon)
        cos_h, sin_h = math.cos(current_heading), math.sin(current_heading)
        lx, ly = dx * cos_h + dy * sin_h, -dx * sin_h + dy * cos_h
        
        v_seqs = np.random.uniform(0, self.cruise_speed, (50, self.horizon))
        s_seqs = np.random.uniform(-self.max_steering, self.max_steering, (50, self.horizon))
        best_cost, best_cmd = float('inf'), (0.0, 0.0)
        
        for i in range(50):
            cost, x, y, theta = 0.0, 0.0, 0.0, 0.0
            for j in range(self.horizon):
                v, s = v_seqs[i, j], s_seqs[i, j]
                x += v * math.cos(theta) * self.dt
                y += v * math.sin(theta) * self.dt
                theta += (v / self.wheelbase) * math.tan(s) * self.dt
                cost += math.sqrt((x - lx)**2 + (y - ly)**2) * 0.1
                if lidar_data is not None and len(lidar_data) > 0:
                    dists = np.sqrt((lidar_data[:,0] - x)**2 + (lidar_data[:,1] - y)**2)
                    if np.min(dists) < (self.robot_width/2.0 + 0.3): cost += 1000.0
            if cost < best_cost:
                best_cost, best_cmd = cost, (s_seqs[i, 0], v_seqs[i, 0])
                
        z_speed = compute_z_speed(target_z, current_z, self.max_mast_speed)
        return best_cmd[0], best_cmd[1], z_speed

class VFHPursuitPlanner(BaseLocalPlanner):
    def __init__(self, wheelbase=1.5, robot_width=1.0, max_steering=0.5, cruise_speed=2.5, max_mast_speed=0.5):
        self.wheelbase, self.robot_width = wheelbase, robot_width
        self.max_steering, self.cruise_speed, self.max_mast_speed = max_steering, cruise_speed, max_mast_speed
        self.num_sectors = 36

    def compute_commands(self, target_lat, target_lon, target_z, current_lat, current_lon, current_z, current_heading, current_speed, current_steering_angle, current_z_speed, lidar_data) -> Tuple[float, float, float]:
        dx, dy = latlon_to_meters(current_lat, current_lon, target_lat, target_lon)
        cos_h, sin_h = math.cos(current_heading), math.sin(current_heading)
        lx, ly = dx * cos_h + dy * sin_h, -dx * sin_h + dy * cos_h
        target_angle = math.atan2(ly, lx)
        
        histogram = np.zeros(self.num_sectors)
        if lidar_data is not None and len(lidar_data) > 0:
            for pt in lidar_data:
                dist = math.sqrt(pt[0]**2 + pt[1]**2)
                if dist < 5.0 and dist > 0.1:
                    angle = math.atan2(pt[1], pt[0])
                    sector = int((angle + math.pi) / (2*math.pi) * self.num_sectors) % self.num_sectors
                    enlargement = math.asin(min(1.0, (self.robot_width/2.0 + 0.2) / dist))
                    sec_span = int(enlargement / (2*math.pi) * self.num_sectors) + 1
                    for s in range(sector - sec_span, sector + sec_span + 1):
                        histogram[s % self.num_sectors] += 1.0 / dist
                        
        best_angle, min_diff = target_angle, float('inf')
        for i in range(self.num_sectors):
            if histogram[i] < 1.5:
                angle = i * (2*math.pi) / self.num_sectors - math.pi
                diff = abs(math.atan2(math.sin(target_angle - angle), math.cos(target_angle - angle)))
                if diff < min_diff: min_diff, best_angle = diff, angle
                    
        vx, vy = math.cos(best_angle) * 2.0, math.sin(best_angle) * 2.0
        L_sq = vx**2 + vy**2
        steering = max(min(math.atan(2*vy/L_sq * self.wheelbase), self.max_steering), -self.max_steering) if L_sq > 0 and vx > 0 else 0.0
        
        z_speed = compute_z_speed(target_z, current_z, self.max_mast_speed)
        return steering, self.cruise_speed * max(0.2, 1.0 - abs(steering)/self.max_steering), z_speed
