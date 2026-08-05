"""
Ackermann Dynamic Window Approach (DWA) Local Planner implementation.
Customized from PythonRobotics DWA to Ackermann kinematics and Oriented Bounding Box collision checking.
"""
import math
import numpy as np
from typing import List, Dict, Tuple, Optional
from core.plugin.path_planner.robot_config import RobotConfig
from core.plugin.path_planner.base_local_planner import BaseLocalPlanner


class AckermannDWALocalPlanner(BaseLocalPlanner):
    """
    Dynamic Window Approach (DWA) Local Planner adapted for Ackermann Mobile Robots.
    Control Space: (v, delta) -> Forward velocity v, Front wheel steering angle delta.
    """

    def __init__(self, config: RobotConfig):
        super().__init__(config)
        self.last_delta: float = 0.0

    def compute_velocity_commands(
        self,
        current_pose: Dict[str, float],
        current_vel: float,
        local_path: List[Dict[str, float]],
        obstacle_points: List[Tuple[float, float]]
    ) -> Tuple[float, float]:
        """
        Compute optimal (v, delta) control command pair using Ackermann DWA evaluation.
        """
        if not local_path:
            return 0.0, 0.0

        # Calculate Dynamic Window boundaries for Control Space (v, delta)
        dw = self._calc_dynamic_window(current_vel, self.last_delta)

        best_v = 0.0
        best_delta = 0.0
        best_traj = None
        min_cost = float("inf")

        # Discretize control space search grid
        v_samples = np.linspace(dw[0], dw[1], self.config.v_samples)
        delta_samples = np.linspace(dw[2], dw[3], self.config.steer_samples)

        # Extract target reference point from global local_path
        target_point = self._get_target_reference_point(current_pose, local_path)

        for v in v_samples:
            for delta in delta_samples:
                # 1. Predict trajectory over prediction horizon
                trajectory = self._predict_trajectory(current_pose, v, delta)

                # 1.5. Evaluate Corridor Boundary Constraint (Disallow crossing specified corridor boundary)
                corridor_cost = self._calc_corridor_boundary_cost(trajectory, local_path)
                if math.isinf(corridor_cost):
                    continue  # Trajectory violates corridor boundary

                # 2. Evaluate Collision & Obstacle Cost
                obs_cost, min_dist = self._calc_obstacle_cost(trajectory, obstacle_points)
                if math.isinf(obs_cost):
                    continue  # Trajectory collides with footprint

                # 3. Evaluate Path Distance Cost (Heading alignment / Goal tracking)
                path_dist_cost = self._calc_path_distance_cost(trajectory, target_point)

                # 4. Evaluate Velocity Tracking Cost
                vel_cost = abs(target_point["v_ref"] - v)

                # 5. Evaluate Steering Smoothness Cost
                steer_smooth_cost = abs(delta - self.last_delta)

                # Total Cost Sum
                cost = (
                    self.config.path_distance_weight * path_dist_cost
                    + self.config.obstacle_weight * obs_cost
                    + self.config.velocity_weight * vel_cost
                    + self.config.steer_smoothness_weight * steer_smooth_cost
                    + 5.0 * corridor_cost
                )

                if cost < min_cost:
                    min_cost = cost
                    best_v = v
                    best_delta = delta
                    best_traj = trajectory

        if best_traj is not None:
            self.best_local_path = [{"x": p[0], "y": p[1], "heading": p[2]} for p in best_traj]
        else:
            self.best_local_path = []

        # Update last steering angle state
        self.last_delta = best_delta
        return float(best_v), float(best_delta)

    def _calc_dynamic_window(self, current_v: float, current_delta: float) -> Tuple[float, float, float, float]:
        """
        Calculate Search Dynamic Window [v_min, v_max, delta_min, delta_max] based on kinematic limits.
        """
        # Dynamic window based on vehicle specs
        Vs = [self.config.min_velocity, self.config.max_velocity, self.config.min_steer_angle, self.config.max_steer_angle]

        # Dynamic window based on motor acceleration / steering rate limits
        Vd = [
            current_v - self.config.max_accel * self.config.dt,
            current_v + self.config.max_accel * self.config.dt,
            current_delta - self.config.max_steer_rate * self.config.dt,
            current_delta + self.config.max_steer_rate * self.config.dt
        ]

        dw = [
            max(Vs[0], Vd[0]),
            min(Vs[1], Vd[1]),
            max(Vs[2], Vd[2]),
            min(Vs[3], Vd[3])
        ]
        return dw[0], dw[1], dw[2], dw[3]

    def _predict_trajectory(self, current_pose: Dict[str, float], v: float, delta: float) -> List[Tuple[float, float, float]]:
        """
        Simulate vehicle trajectory over prediction horizon using Ackermann Kinematic Model:
        x_{t+1} = x_t + v * cos(theta) * dt
        y_{t+1} = y_t + v * sin(theta) * dt
        theta_{t+1} = theta_t + (v / L) * tan(delta) * dt
        """
        trajectory = []
        x = current_pose["x"]
        y = current_pose["y"]
        theta = current_pose["heading"]
        dt = self.config.dt
        L = self.config.wheelbase

        time_steps = int(self.config.predict_time / dt)
        for _ in range(time_steps):
            x += v * math.cos(theta) * dt
            y += v * math.sin(theta) * dt
            theta += (v / L) * math.tan(delta) * dt
            # Normalize theta to [-pi, pi]
            theta = math.atan2(math.sin(theta), math.cos(theta))
            trajectory.append((x, y, theta))

        return trajectory

    def _calc_obstacle_cost(
        self,
        trajectory: List[Tuple[float, float, float]],
        obstacle_points: List[Tuple[float, float]]
    ) -> Tuple[float, float]:
        """
        Evaluate Oriented Bounding Box (OBB) footprint collision and obstacle clearance cost.
        """
        if not obstacle_points:
            return 0.0, float("inf")

        min_dist = float("inf")
        half_l = self.config.length / 2.0 + self.config.inflation_radius
        half_w = self.config.width / 2.0 + self.config.inflation_radius

        for tx, ty, ttheta in trajectory:
            cos_t = math.cos(ttheta)
            sin_t = math.sin(ttheta)

            for ox, oy in obstacle_points:
                # Transform obstacle into robot local OBB frame
                dx = ox - tx
                dy = oy - ty
                local_x = dx * cos_t + dy * sin_t
                local_y = -dx * sin_t + dy * cos_t

                # Check inside inflated oriented bounding box
                if abs(local_x) <= half_l and abs(local_y) <= half_w:
                    return float("inf"), 0.0  # Collision detected

                dist = math.hypot(dx, dy)
                if dist < min_dist:
                    min_dist = dist

        # Inverse distance obstacle cost
        obs_cost = 1.0 / (min_dist + 1e-6)
        return obs_cost, min_dist

    def _calc_corridor_boundary_cost(
        self,
        trajectory: List[Tuple[float, float, float]],
        local_path: List[Dict[str, float]]
    ) -> float:
        """
        Evaluate corridor boundary constraint:
        Distance from trajectory points to local path center line must not exceed corridor_boundary / 2.0.
        Returns float("inf") if trajectory crosses corridor boundary limit.
        """
        if not local_path or len(local_path) < 2:
            return 0.0

        max_lateral_dev = 0.0
        for tx, ty, _ in trajectory:
            # Find closest line segment in local_path to (tx, ty)
            min_dist_to_segment = float("inf")
            allowed_half_w = 1.25  # default 2.5m / 2

            for i in range(len(local_path) - 1):
                p1 = local_path[i]
                p2 = local_path[i + 1]
                half_w = p1.get("corridor_boundary", 2.5) / 2.0

                # Perpendicular distance from point (tx, ty) to segment p1-p2
                x1, y1 = p1["x"], p1["y"]
                x2, y2 = p2["x"], p2["y"]
                dx = x2 - x1
                dy = y2 - y1
                l2 = dx * dx + dy * dy
                if l2 == 0:
                    dist = math.hypot(tx - x1, ty - y1)
                else:
                    t = max(0.0, min(1.0, ((tx - x1) * dx + (ty - y1) * dy) / l2))
                    proj_x = x1 + t * dx
                    proj_y = y1 + t * dy
                    dist = math.hypot(tx - proj_x, ty - proj_y)

                if dist < min_dist_to_segment:
                    min_dist_to_segment = dist
                    allowed_half_w = half_w

            # Strict Boundary Check (Robot body width buffer included: 0.5m half-width)
            robot_half_width = self.config.width / 2.0
            if (min_dist_to_segment + robot_half_width) > allowed_half_w:
                return float("inf")  # Disallowed: crosses corridor boundary

            if min_dist_to_segment > max_lateral_dev:
                max_lateral_dev = min_dist_to_segment

        return max_lateral_dev

    def _calc_path_distance_cost(
        self,
        trajectory: List[Tuple[float, float, float]],
        target_point: Dict[str, float]
    ) -> float:
        """
        Calculate distance between trajectory endpoint and target global path reference point.
        """
        last_x, last_y, last_theta = trajectory[-1]
        dx = target_point["x"] - last_x
        dy = target_point["y"] - last_y
        dist_cost = math.hypot(dx, dy)

        # Heading difference cost
        angle_diff = abs(math.atan2(math.sin(target_point["heading"] - last_theta), math.cos(target_point["heading"] - last_theta)))
        return dist_cost + 0.5 * angle_diff

    def _get_target_reference_point(
        self,
        current_pose: Dict[str, float],
        local_path: List[Dict[str, float]]
    ) -> Dict[str, float]:
        """
        Find lookahead target point along local_path.
        """
        rx, ry = current_pose["x"], current_pose["y"]
        lookahead_dist = 1.5

        for pt in local_path:
            dist = math.hypot(pt["x"] - rx, pt["y"] - ry)
            if dist >= lookahead_dist:
                return pt

        return local_path[-1]
