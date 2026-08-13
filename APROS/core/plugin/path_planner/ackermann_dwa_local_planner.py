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
        Compute optimal (v, delta) control command pair using Batch-Vectorized Ackermann DWA evaluation.
        Evaluates all trajectory candidates in parallel using NumPy SIMD matrix operations.
        """
        if not local_path:
            return 0.0, 0.0

        # Calculate Dynamic Window boundaries for Control Space (v, delta)
        dw = self._calc_dynamic_window(current_vel, self.last_delta)

        # Discretize control space search grid
        v_samples = np.linspace(dw[0], dw[1], self.config.v_samples)
        delta_samples = np.linspace(dw[2], dw[3], self.config.steer_samples)
        V_grid, D_grid = np.meshgrid(v_samples, delta_samples, indexing='ij')
        V_flat = V_grid.ravel()      # Shape (K,) where K = v_samples * steer_samples
        D_flat = D_grid.ravel()      # Shape (K,)
        K = len(V_flat)

        # 1. Batch Predict Trajectories for ALL candidates simultaneously
        # Trajectories shape: (K, T, 3) where columns are (x, y, theta)
        dt = self.config.dt
        L = self.config.wheelbase
        time_steps = int(self.config.predict_time / dt)

        rx, ry = current_pose["x"], current_pose["y"]
        th_robot = current_pose["heading"]
        cos_r = math.cos(th_robot)
        sin_r = math.sin(th_robot)

        # Transform local_path into Robot Local Sensor Frame (x0=0, y0=0, th0=0)
        local_path_robot = []
        for pt in local_path:
            dx = pt["x"] - rx
            dy = pt["y"] - ry
            lx = dx * cos_r + dy * sin_r
            ly = -dx * sin_r + dy * cos_r
            lh = pt["heading"] - th_robot
            lh = math.atan2(math.sin(lh), math.cos(lh))
            local_path_robot.append({
                "x": lx, "y": ly, "heading": lh,
                "v_ref": pt.get("v_ref", 1.0),
                "corridor_boundary": pt.get("corridor_boundary", 2.5)
            })

        # 1. Batch Predict Trajectories in Robot Local Sensor Frame starting at (0.0, 0.0, 0.0)
        dt = self.config.dt
        L = self.config.wheelbase
        time_steps = int(self.config.predict_time / dt)

        trajs = np.zeros((K, time_steps, 3), dtype=np.float64)
        x_curr = np.zeros(K, dtype=np.float64)
        y_curr = np.zeros(K, dtype=np.float64)
        th_curr = np.zeros(K, dtype=np.float64)

        tan_D = np.tan(D_flat)
        v_L = V_flat / L

        for t in range(time_steps):
            x_curr += V_flat * np.cos(th_curr) * dt
            y_curr += V_flat * np.sin(th_curr) * dt
            th_curr += v_L * tan_D * dt
            th_curr = np.arctan2(np.sin(th_curr), np.cos(th_curr))
            trajs[:, t, 0] = x_curr
            trajs[:, t, 1] = y_curr
            trajs[:, t, 2] = th_curr

        # 2. Batch Corridor Boundary Cost Evaluation (in Robot Local Frame)
        n_seg = len(local_path_robot) - 1
        corridor_costs = np.zeros(K, dtype=np.float64)

        if n_seg >= 1:
            seg_x1 = np.array([local_path_robot[i]["x"] for i in range(n_seg)])
            seg_y1 = np.array([local_path_robot[i]["y"] for i in range(n_seg)])
            seg_x2 = np.array([local_path_robot[i + 1]["x"] for i in range(n_seg)])
            seg_y2 = np.array([local_path_robot[i + 1]["y"] for i in range(n_seg)])
            seg_half_w = np.array([local_path_robot[i].get("corridor_boundary", 2.5) / 2.0 for i in range(n_seg)])

            seg_dx = seg_x2 - seg_x1
            seg_dy = seg_y2 - seg_y1
            seg_l2 = np.maximum(seg_dx * seg_dx + seg_dy * seg_dy, 1e-12)
            robot_half_w = self.config.width / 2.0

            all_pts_x = trajs[:, :, 0].ravel()
            all_pts_y = trajs[:, :, 1].ravel()

            t_params = np.clip(((all_pts_x[:, None] - seg_x1[None, :]) * seg_dx[None, :] +
                                (all_pts_y[:, None] - seg_y1[None, :]) * seg_dy[None, :]) / seg_l2[None, :], 0.0, 1.0)
            proj_x = seg_x1[None, :] + t_params * seg_dx[None, :]
            proj_y = seg_y1[None, :] + t_params * seg_dy[None, :]
            dists = np.hypot(all_pts_x[:, None] - proj_x, all_pts_y[:, None] - proj_y)

            nearest_seg = np.argmin(dists, axis=1)
            min_dists = dists[np.arange(len(all_pts_x)), nearest_seg]
            allowed_w = seg_half_w[nearest_seg]

            min_dists_kt = min_dists.reshape(K, time_steps)
            allowed_w_kt = allowed_w.reshape(K, time_steps)

            violates_corridor = (min_dists_kt + robot_half_w) > allowed_w_kt
            invalid_corridor = violates_corridor.any(axis=1)
            corridor_costs = np.max(min_dists_kt, axis=1)
            corridor_costs[invalid_corridor] = float("inf")

        # 3. Batch Obstacle Clearance & OBB Footprint Collision Evaluation (VLP-16 Local Points)
        obs_costs = np.zeros(K, dtype=np.float64)
        if obstacle_points:
            obs_arr = np.asarray(obstacle_points)  # (N, 2) in local robot frame
            # Filter obstacle points within lookahead horizon (lookahead_distance + length / 2 + inflation margin)
            max_lookahead_range = getattr(self.config, "lookahead_distance", 3.0) + self.config.length / 2.0 + self.config.inflation_radius + 1.0
            obs_dists = np.hypot(obs_arr[:, 0], obs_arr[:, 1])
            in_lookahead_range = obs_dists <= max_lookahead_range
            if in_lookahead_range.any():
                obs_arr = obs_arr[in_lookahead_range]
            half_l = self.config.length / 2.0 + self.config.inflation_radius
            half_w = self.config.width / 2.0 + self.config.inflation_radius

            flat_trajs = trajs.reshape(-1, 3)
            tx = flat_trajs[:, 0, None]
            ty = flat_trajs[:, 1, None]
            th = flat_trajs[:, 2, None]

            ox = obs_arr[None, :, 0]
            oy = obs_arr[None, :, 1]

            dx = ox - tx
            dy = oy - ty

            cos_th = np.cos(th)
            sin_th = np.sin(th)
            local_x = dx * cos_th + dy * sin_th
            local_y = -dx * sin_th + dy * cos_th

            inside = (np.abs(local_x) <= half_l) & (np.abs(local_y) <= half_w)
            collides_kt = inside.any(axis=1).reshape(K, time_steps)
            collides_k = collides_kt.any(axis=1)

            r_all = np.hypot(dx, dy).reshape(K, time_steps, -1)
            min_r_per_k = np.min(r_all, axis=(1, 2))

            obs_costs = 1.0 / (min_r_per_k + 1e-6)
            obs_costs[collides_k] = float("inf")

        # 4. Target Reference & Velocity & Smoothness Cost Evaluation (in Local Robot Frame)
        target_point = self._get_target_reference_point({"x": 0.0, "y": 0.0, "heading": 0.0}, local_path_robot)
        last_pts = trajs[:, -1, :]  # (K, 3)

        dx_g = target_point["x"] - last_pts[:, 0]
        dy_g = target_point["y"] - last_pts[:, 1]
        dist_costs = np.hypot(dx_g, dy_g)

        angle_diffs = np.abs(np.arctan2(np.sin(target_point["heading"] - last_pts[:, 2]),
                                         np.cos(target_point["heading"] - last_pts[:, 2])))
        path_dist_costs = dist_costs + 0.5 * angle_diffs

        vel_costs = np.abs(target_point["v_ref"] - V_flat)
        steer_smooth_costs = np.abs(D_flat - self.last_delta)

        total_costs = (
            self.config.path_distance_weight * path_dist_costs
            + self.config.obstacle_weight * obs_costs
            + self.config.velocity_weight * vel_costs
            + self.config.steer_smoothness_weight * steer_smooth_costs
            + getattr(self.config, "corridor_weight", 5.0) * corridor_costs
        )

        min_idx = np.argmin(total_costs)
        min_cost = total_costs[min_idx]

        if math.isinf(min_cost):
            self.best_local_path = []
            self.last_delta = 0.0
            return 0.0, 0.0

        best_v = max(0.0, float(V_flat[min_idx]))
        best_delta = float(D_flat[min_idx])
        best_traj = trajs[min_idx]

        # Transform best local trajectory back to global relative space for visualization
        best_global_traj = []
        for p in best_traj:
            lx, ly, lh = float(p[0]), float(p[1]), float(p[2])
            gx = rx + lx * cos_r - ly * sin_r
            gy = ry + lx * sin_r + ly * cos_r
            gh = lh + th_robot
            best_global_traj.append({"x": gx, "y": gy, "heading": gh})

        self.best_local_path = best_global_traj
        self.last_delta = best_delta

        return best_v, best_delta

    def _calc_dynamic_window(self, current_v: float, current_delta: float) -> Tuple[float, float, float, float]:
        """
        Calculate Search Dynamic Window [v_min, v_max, delta_min, delta_max] based on kinematic limits.
        """
        # Dynamic window based on vehicle specs (Forward-only drive assumption: min_velocity >= 0.0)
        min_v_spec = max(0.0, self.config.min_velocity)
        Vs = [min_v_spec, self.config.max_velocity, self.config.min_steer_angle, self.config.max_steer_angle]

        # Dynamic window based on motor acceleration / steering rate limits
        Vd = [
            current_v - self.config.max_accel * self.config.dt,
            current_v + self.config.max_accel * self.config.dt,
            current_delta - self.config.max_steer_rate * self.config.dt,
            current_delta + self.config.max_steer_rate * self.config.dt
        ]

        dw = [
            max(Vs[0], Vd[0], 0.0),
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
        Find lookahead target point along local_path using configured lookahead_distance.
        """
        rx, ry = current_pose["x"], current_pose["y"]
        lookahead_dist = getattr(self.config, "lookahead_distance", 3.0)

        for pt in local_path:
            dist = math.hypot(pt["x"] - rx, pt["y"] - ry)
            if dist >= lookahead_dist:
                return pt

        return local_path[-1]
