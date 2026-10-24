"""Model Predictive Path Integral (MPPI) local planner for Ackermann robots."""
import math
from typing import Dict, List, Optional, Tuple

import numpy as np

from core.plugin.path_planner.base_local_planner import BaseLocalPlanner


class MPPILocalPlanner(BaseLocalPlanner):
    """Sample Ackermann control sequences and optimize them with MPPI weights.

    Reflected from ``MPPIControllerForAPROS`` in ``mppi_pathtracking.ipynb``:
    - All internal MPPI state, heading, steering noise, steering angle limits,
      steering rate limits, and cost calculations operate strictly in **radians (`rad`)**.
    - ``steering_output_unit = "rad"`` (returns `[v_ms, steer_rad]` to ``DriveExecutor``).
    - Robot steering angle range is clamped to ``[-30.0 deg, +30.0 deg]`` (or configured
      ``[min_steer_angle, max_steer_angle]`` in radians).
    - Maximum steering angular velocity is rate-limited at each rollout step
      (default ``15.0 deg/s`` = ``math.radians(15.0) rad/s``).
    - Steering polarity follows ISO 8855 / ROS convention:
      ``+`` is Left turn (CCW), ``-`` is Right turn (CW).
    - ``best_local_path`` is a global-frame ribbon centerline rendered in Viser.
    """
    steering_output_unit = "rad"

    def __init__(
        self,
        config,
        samples: int = 96,
        horizon: int = 36,
        iterations: int = 2,
        temperature: float = 1.0,
        steering_noise_deg: Optional[float] = None,
        steering_noise_rad: float = 0.16,
        velocity_noise: float = 0.08,
        initial_velocity_kmh: float = 2.0,
        obstacle_limit: int = 192,
        obstacle_range_margin: float = 1.0,
        path_distance_weight: float = 1.4,
        heading_weight: float = 0.8,
        clearance_weight: float = 2.0,
        collision_cost: float = 1e4,
        velocity_weight: float = 1.2,
        steering_weight: float = 0.08,
        max_steer_rate_deg_s: Optional[float] = None,
        max_steer_rate_rad_s: Optional[float] = None,
        steering_change_weight: float = 0.02,
        terminal_distance_weight: float = 2.0,
    ):
        super().__init__(config)
        self.samples = max(16, int(samples))
        self.horizon = max(8, int(horizon))
        self.iterations = max(1, int(iterations))
        self.temperature = max(1e-3, float(temperature))

        # Internal steering noise in radians (if steering_noise_deg is provided in config, convert to rad)
        if steering_noise_deg is not None:
            self.steering_noise_rad = math.radians(float(steering_noise_deg))
            self.steering_noise_deg = float(steering_noise_deg)
        else:
            self.steering_noise_rad = float(steering_noise_rad)
            self.steering_noise_deg = math.degrees(self.steering_noise_rad)

        self.velocity_noise = float(velocity_noise)
        self.obstacle_limit = max(1, int(obstacle_limit))
        self.obstacle_range_margin = float(obstacle_range_margin)
        self.path_distance_weight = float(path_distance_weight)
        self.heading_weight = float(heading_weight)
        self.clearance_weight = float(clearance_weight)
        self.collision_cost = float(collision_cost)
        self.velocity_weight = float(velocity_weight)
        self.steering_weight = float(steering_weight)

        # Internal max steering rate in rad/s (default: 15.0 deg/s = 0.261799 rad/s)
        if max_steer_rate_rad_s is not None:
            self.max_steer_rate_rad_s = float(max_steer_rate_rad_s)
        elif max_steer_rate_deg_s is not None:
            self.max_steer_rate_rad_s = math.radians(float(max_steer_rate_deg_s))
        elif hasattr(config, "max_steer_rate") and config.max_steer_rate is not None:
            self.max_steer_rate_rad_s = float(config.max_steer_rate)
        else:
            self.max_steer_rate_rad_s = math.radians(15.0)
        self.max_steer_rate_deg_s = math.degrees(self.max_steer_rate_rad_s)

        self.steering_change_weight = float(steering_change_weight)
        if self.max_steer_rate_rad_s <= 0.0 or self.steering_change_weight < 0.0:
            raise ValueError("MPPI steering rate must be positive and change weight nonnegative")

        self.terminal_distance_weight = float(terminal_distance_weight)
        self.best_local_path: List[Dict[str, float]] = []

        self.initial_velocity_kmh = float(initial_velocity_kmh)
        init_v_ms = min(float(config.max_velocity), self.initial_velocity_kmh / 3.6)
        self._nominal_v = np.full(self.horizon, init_v_ms, dtype=np.float64)
        self._nominal_delta_rad = np.zeros(self.horizon, dtype=np.float64)
        self._current_steer_rad = 0.0
        self._rng = np.random.default_rng()

    def reset_state(self):
        """Reset warm-start nominal velocity and steering sequences."""
        init_v_ms = min(float(self.config.max_velocity), self.initial_velocity_kmh / 3.6)
        self._nominal_v = np.full(self.horizon, init_v_ms, dtype=np.float64)
        self._nominal_delta_rad = np.zeros(self.horizon, dtype=np.float64)
        self._current_steer_rad = 0.0
        self.best_local_path = []

    def compute_velocity_commands(
        self,
        current_pose: Dict[str, float],
        current_vel: float,
        local_path: List[Dict[str, float]],
        obstacle_points: List[Tuple[float, float]],
    ) -> Tuple[float, float]:
        """Compute optimal (target_v_ms, target_steer_rad) using MPPIControllerForAPROS logic in radians."""
        if not local_path:
            self.best_local_path = []
            return 0.0, 0.0

        cfg = self.config
        dt = float(cfg.dt)
        k = self.samples
        horizon = self.horizon
        rx, ry, heading = float(current_pose["x"]), float(current_pose["y"]), float(current_pose["heading"])
        max_lookahead = float(getattr(cfg, "lookahead_distance", 3.0))
        max_distance = max_lookahead + float(cfg.length) / 2.0
        rollout_dt = max(dt, max_lookahead / (max(float(cfg.max_velocity), 0.1) * horizon))
        control_dt = float(current_pose.get("control_dt", dt))

        # Current steering angle in radians (prefer steering_rad, fallback to steering_deg converted to rad)
        if "steering_rad" in current_pose:
            current_steer_rad = float(current_pose["steering_rad"])
        elif "steering_deg" in current_pose:
            current_steer_rad = math.radians(float(current_pose["steering_deg"]))
        else:
            current_steer_rad = self._current_steer_rad

        # Transform local reference path into robot body frame (+X forward, +Y left)
        path = np.asarray(
            [[p["x"], p["y"], p.get("heading", 0.0), p.get("v_ref", cfg.max_velocity)] for p in local_path],
            dtype=np.float64,
        )
        dx, dy = path[:, 0] - rx, path[:, 1] - ry
        c, s = math.cos(heading), math.sin(heading)
        px = dx * c + dy * s
        py = -dx * s + dy * c
        local_headings = np.arctan2(np.sin(path[:, 2] - heading), np.cos(path[:, 2] - heading))
        target_idx = int(np.argmin(np.abs(np.hypot(px, py) - max_lookahead)))
        target_speed = float(np.clip(path[target_idx, 3], cfg.min_velocity, cfg.max_velocity))

        # Limit obstacle evaluation to the local horizon and cap point count for predictable runtime.
        obs = np.asarray(obstacle_points, dtype=np.float64).reshape(-1, 2) if obstacle_points else np.empty((0, 2))
        if len(obs):
            # LiDAR points arrive in the robot/sensor frame.
            obs_local = obs[np.hypot(obs[:, 0], obs[:, 1]) <= max_distance + self.obstacle_range_margin]
            if len(obs_local) > self.obstacle_limit:
                obs_local = obs_local[::max(1, len(obs_local) // self.obstacle_limit)][:self.obstacle_limit]
        else:
            obs_local = obs

        # Steering angle bounds in radians (strictly bounded within [-30 deg, +30 deg])
        hard_limit_rad = math.radians(30.0)
        steering_min_rad = max(-hard_limit_rad, float(cfg.min_steer_angle))
        steering_max_rad = min(hard_limit_rad, float(cfg.max_steer_angle))
        current_steer_rad = float(np.clip(current_steer_rad, steering_min_rad, steering_max_rad))

        nominal_v = self._nominal_v.copy()
        nominal_delta_rad = self._nominal_delta_rad.copy()
        trajectories = None
        costs = None
        noise_v = None

        for _ in range(self.iterations):
            noise_v = self._rng.normal(0.0, self.velocity_noise, (k, horizon))
            noise_delta_rad = self._rng.normal(0.0, self.steering_noise_rad, (k, horizon))
            velocities = np.clip(nominal_v[None, :] + noise_v, cfg.min_velocity, cfg.max_velocity)
            deltas_rad = np.clip(
                nominal_delta_rad[None, :] + noise_delta_rad,
                steering_min_rad,
                steering_max_rad,
            )
            previous_delta_rad = np.full(k, current_steer_rad, dtype=np.float64)
            costs = np.zeros(k, dtype=np.float64)
            x = np.zeros(k, dtype=np.float64)
            y = np.zeros(k, dtype=np.float64)
            theta = np.zeros(k, dtype=np.float64)
            trajectories = np.zeros((k, horizon, 3), dtype=np.float64)

            for t in range(horizon):
                v = velocities[:, t]
                step_dt = control_dt if t == 0 else rollout_dt
                max_delta_change_rad = self.max_steer_rate_rad_s * step_dt
                delta_rad = np.clip(
                    deltas_rad[:, t],
                    previous_delta_rad - max_delta_change_rad,
                    previous_delta_rad + max_delta_change_rad,
                )
                delta_rad = np.clip(delta_rad, steering_min_rad, steering_max_rad)
                deltas_rad[:, t] = delta_rad
                delta_change_rad = delta_rad - previous_delta_rad
                previous_delta_rad = delta_rad

                x += v * np.cos(theta) * rollout_dt
                y += v * np.sin(theta) * rollout_dt
                theta += v / max(1e-6, cfg.wheelbase) * np.tan(delta_rad) * rollout_dt
                theta = np.arctan2(np.sin(theta), np.cos(theta))
                trajectories[:, t, :] = np.column_stack((x, y, theta))

                # Track the closest reference point in body frame
                nearest = np.argmin((x[:, None] - px[None, :]) ** 2 + (y[:, None] - py[None, :]) ** 2, axis=1)
                path_error = np.hypot(x - px[nearest], y - py[nearest])
                heading_error = np.abs(
                    np.arctan2(np.sin(theta - local_headings[nearest]), np.cos(theta - local_headings[nearest]))
                )
                costs += self.path_distance_weight * path_error + self.heading_weight * heading_error

                if len(obs_local):
                    ox = obs_local[None, :, 0] - x[:, None]
                    oy = obs_local[None, :, 1] - y[:, None]
                    oxr = ox * np.cos(theta[:, None]) + oy * np.sin(theta[:, None])
                    oyr = -ox * np.sin(theta[:, None]) + oy * np.cos(theta[:, None])
                    half_l = cfg.length / 2.0 + cfg.inflation_radius
                    half_w = cfg.width / 2.0 + cfg.inflation_radius
                    inside = (np.abs(oxr) <= half_l) & (np.abs(oyr) <= half_w)
                    collision = inside.any(axis=1)
                    clearance = np.sqrt(ox * ox + oy * oy).min(axis=1)
                    costs += self.clearance_weight / np.maximum(clearance - half_w, 0.05)
                    costs += collision.astype(np.float64) * self.collision_cost

                costs += self.velocity_weight * np.abs(v - target_speed)
                costs += self.steering_weight * (delta_rad ** 2)
                # Convert rad change to deg equivalent scale so steering_change_weight behaves identically to MPPIControllerForAPROS
                delta_change_deg = np.rad2deg(delta_change_rad)
                costs += self.steering_change_weight * (delta_change_deg ** 2)

            costs += self.terminal_distance_weight * np.hypot(
                trajectories[:, -1, 0] - px[target_idx],
                trajectories[:, -1, 1] - py[target_idx],
            )

            baseline = float(np.min(costs))
            weights = np.exp(np.clip(-(costs - baseline) / self.temperature, -60.0, 0.0))
            weights /= max(float(weights.sum()), 1e-12)
            nominal_v = np.clip(
                nominal_v + np.sum(weights[:, None] * noise_v, axis=0),
                cfg.min_velocity,
                cfg.max_velocity,
            )
            # A weighted average of rate-limited candidates remains rate-limited
            nominal_delta_rad = np.clip(
                np.sum(weights[:, None] * deltas_rad, axis=0),
                steering_min_rad,
                steering_max_rad,
            )

        best_idx = int(np.argmin(costs))
        if costs[best_idx] >= self.collision_cost:
            self.best_local_path = []
            return 0.0, 0.0

        # Roll out the smooth optimal nominal sequence in global frame for Viser visualization
        global_path = [{"x": float(rx), "y": float(ry), "heading": float(heading)}]
        ox, oy, otheta = 0.0, 0.0, 0.0
        for t in range(horizon):
            ov = float(nominal_v[t])
            odelta_rad = float(nominal_delta_rad[t])
            ox += ov * math.cos(otheta) * rollout_dt
            oy += ov * math.sin(otheta) * rollout_dt
            otheta += ov / max(1e-6, cfg.wheelbase) * math.tan(odelta_rad) * rollout_dt
            gx = rx + ox * c - oy * s
            gy = ry + ox * s + oy * c
            gh = heading + otheta
            global_path.append({"x": float(gx), "y": float(gy), "heading": float(gh)})
            if math.hypot(ox, oy) >= max_lookahead:
                break
        self.best_local_path = global_path

        self._nominal_v[:-1] = nominal_v[1:]
        self._nominal_v[-1] = nominal_v[-1]
        self._nominal_delta_rad[:-1] = nominal_delta_rad[1:]
        self._nominal_delta_rad[-1] = nominal_delta_rad[-1]

        cmd_v_ms = float(nominal_v[0])
        cmd_steer_rad = float(np.clip(nominal_delta_rad[0], steering_min_rad, steering_max_rad))
        self._current_steer_rad = cmd_steer_rad
        return cmd_v_ms, cmd_steer_rad
