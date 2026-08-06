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
        Vectorized with NumPy for performance.
        """
        if not obstacle_points:
            return 0.0, float("inf")

        half_l = self.config.length / 2.0 + self.config.inflation_radius
        half_w = self.config.width / 2.0 + self.config.inflation_radius

        # Convert to numpy arrays
        traj_arr = np.array(trajectory)  # (T, 3) -> x, y, theta
        obs_arr = np.array(obstacle_points)  # (N, 2) -> ox, oy

        tx = traj_arr[:, 0]  # (T,)
        ty = traj_arr[:, 1]  # (T,)
        ttheta = traj_arr[:, 2]  # (T,)
        ox = obs_arr[:, 0]  # (N,)
        oy = obs_arr[:, 1]  # (N,)

        # Broadcast: dx[t, n], dy[t, n]
        dx = ox[None, :] - tx[:, None]  # (T, N)
        dy = oy[None, :] - ty[:, None]  # (T, N)

        # Transform to local OBB frame for each trajectory point
        cos_t = np.cos(ttheta)[:, None]  # (T, 1)
        sin_t = np.sin(ttheta)[:, None]  # (T, 1)
        local_x = dx * cos_t + dy * sin_t   # (T, N)
        local_y = -dx * sin_t + dy * cos_t  # (T, N)

        # Check collision: inside inflated OBB
        inside = (np.abs(local_x) <= half_l) & (np.abs(local_y) <= half_w)
        if inside.any():
            return float("inf"), 0.0

        # Min distance
        r = np.hypot(dx, dy)
        min_r = float(np.min(r))
        obs_cost = 1.0 / (min_r + 1e-6)
        return obs_cost, min_r

    def _calc_corridor_boundary_cost(
        self,
        trajectory: List[Tuple[float, float, float]],
        local_path: List[Dict[str, float]]
    ) -> float:
        """
        Evaluate corridor boundary constraint (vectorized with NumPy).
        Distance from trajectory points to local path center line must not exceed corridor_boundary / 2.0.
        Returns float("inf") if trajectory crosses corridor boundary limit.
        """
        if not local_path or len(local_path) < 2:
            return 0.0

        n_seg = len(local_path) - 1
        traj_arr = np.array([(t[0], t[1]) for t in trajectory])  # (T, 2)
        T = len(traj_arr)

        # Build segment arrays
        seg_x1 = np.array([local_path[i]["x"] for i in range(n_seg)])
        seg_y1 = np.array([local_path[i]["y"] for i in range(n_seg)])
        seg_x2 = np.array([local_path[i + 1]["x"] for i in range(n_seg)])
        seg_y2 = np.array([local_path[i + 1]["y"] for i in range(n_seg)])
        seg_half_w = np.array([local_path[i].get("corridor_boundary", 2.5) / 2.0 for i in range(n_seg)])

        seg_dx = seg_x2 - seg_x1  # (S,)
        seg_dy = seg_y2 - seg_y1  # (S,)
        seg_l2 = seg_dx * seg_dx + seg_dy * seg_dy  # (S,)
        seg_l2 = np.maximum(seg_l2, 1e-12)  # avoid division by zero

        robot_half_width = self.config.width / 2.0
        max_lateral_dev = 0.0

        for ti in range(T):
            px, py = traj_arr[ti, 0], traj_arr[ti, 1]
            # Vectorized point-to-segment distance for all segments
            t_param = np.clip(((px - seg_x1) * seg_dx + (py - seg_y1) * seg_dy) / seg_l2, 0.0, 1.0)
            proj_x = seg_x1 + t_param * seg_dx
            proj_y = seg_y1 + t_param * seg_dy
            dists = np.hypot(px - proj_x, py - proj_y)  # (S,)

            nearest_seg_idx = np.argmin(dists)
            min_dist = dists[nearest_seg_idx]
            allowed_half_w = seg_half_w[nearest_seg_idx]

            if (min_dist + robot_half_width) > allowed_half_w:
                return float("inf")

            if min_dist > max_lateral_dev:
                max_lateral_dev = min_dist

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
