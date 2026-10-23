"""Model Predictive Path Integral (MPPI) local planner for Ackermann robots."""
import math
from typing import Dict, List, Tuple

import numpy as np

from core.plugin.path_planner.base_local_planner import BaseLocalPlanner


class MPPILocalPlanner(BaseLocalPlanner):
    """Sample Ackermann control sequences and optimize them with MPPI weights.

    ``best_local_path`` is a global-frame ribbon centerline for visualization.
    Obstacle points and path points use the global route frame, as in DWA.
    """

    def __init__(self, config, samples: int = 96, horizon: int = 36,
                 iterations: int = 2, temperature: float = 1.0,
                 steering_noise: float = 0.16, velocity_noise: float = 0.22,
                 initial_velocity_mps: float = 0.6, obstacle_limit: int = 192,
                 obstacle_range_margin: float = 1.0,
                 path_distance_weight: float = 1.4, heading_weight: float = 0.25,
                 clearance_weight: float = 2.0, collision_cost: float = 1e4,
                 velocity_weight: float = 0.12, steering_weight: float = 0.08,
                 terminal_distance_weight: float = 2.0):
        super().__init__(config)
        self.samples = max(16, int(samples))
        self.horizon = max(8, int(horizon))
        self.iterations = max(1, int(iterations))
        self.temperature = max(1e-3, float(temperature))
        self.steering_noise = float(steering_noise)
        self.velocity_noise = float(velocity_noise)
        self.obstacle_limit = max(1, int(obstacle_limit))
        self.obstacle_range_margin = float(obstacle_range_margin)
        self.path_distance_weight = float(path_distance_weight)
        self.heading_weight = float(heading_weight)
        self.clearance_weight = float(clearance_weight)
        self.collision_cost = float(collision_cost)
        self.velocity_weight = float(velocity_weight)
        self.steering_weight = float(steering_weight)
        self.terminal_distance_weight = float(terminal_distance_weight)
        self.best_local_path: List[Dict[str, float]] = []
        self._nominal_v = np.full(self.horizon, min(config.max_velocity, initial_velocity_mps), dtype=np.float64)
        self._nominal_delta = np.zeros(self.horizon, dtype=np.float64)
        self._rng = np.random.default_rng()

    def compute_velocity_commands(self, current_pose: Dict[str, float], current_vel: float,
                                  local_path: List[Dict[str, float]],
                                  obstacle_points: List[Tuple[float, float]]) -> Tuple[float, float]:
        if not local_path:
            self.best_local_path = []
            return 0.0, 0.0

        cfg = self.config
        dt = float(cfg.dt)
        k = self.samples
        horizon = self.horizon
        rx, ry, heading = current_pose["x"], current_pose["y"], current_pose["heading"]
        max_lookahead = float(getattr(cfg, "lookahead_distance", 3.0))
        max_distance = max_lookahead + float(cfg.length) / 2.0
        rollout_dt = max(dt, max_lookahead / (max(float(cfg.max_velocity), 0.1) * horizon))

        path = np.asarray([[p["x"], p["y"], p.get("heading", 0.0),
                            p.get("v_ref", cfg.max_velocity)] for p in local_path], dtype=np.float64)
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
            # LiDAR points arrive in the robot/sensor frame (same convention as DWA).
            obs_local = obs[np.hypot(obs[:, 0], obs[:, 1]) <= max_distance + self.obstacle_range_margin]
            if len(obs_local) > self.obstacle_limit:
                obs_local = obs_local[::max(1, len(obs_local) // self.obstacle_limit)][:self.obstacle_limit]
        else:
            obs_local = obs

        nominal_v = self._nominal_v.copy()
        nominal_delta = self._nominal_delta.copy()
        trajectories = None
        costs = None
        noise_v = noise_delta = None
        for _ in range(self.iterations):
            noise_v = self._rng.normal(0.0, self.velocity_noise, (k, horizon))
            noise_delta = self._rng.normal(0.0, self.steering_noise, (k, horizon))
            velocities = np.clip(nominal_v[None, :] + noise_v, cfg.min_velocity, cfg.max_velocity)
            # Include a small set of recovery samples around the nominal sequence.
            steering_min = float(cfg.min_steer_angle)
            steering_max = float(cfg.max_steer_angle)
            deltas = np.clip(nominal_delta[None, :] + noise_delta, steering_min, steering_max)
            costs = np.zeros(k, dtype=np.float64)
            x = np.zeros(k); y = np.zeros(k); theta = np.zeros(k)
            trajectories = np.zeros((k, horizon, 3), dtype=np.float64)
            for t in range(horizon):
                v = velocities[:, t]
                delta = deltas[:, t]
                x += v * np.cos(theta) * rollout_dt
                y += v * np.sin(theta) * rollout_dt
                theta += v / max(1e-6, cfg.wheelbase) * np.tan(delta) * rollout_dt
                theta = np.arctan2(np.sin(theta), np.cos(theta))
                trajectories[:, t, :] = np.column_stack((x, y, theta))

                # Track the closest forward reference point; its width defines the safe corridor.
                nearest = np.argmin((x[:, None] - px[None, :]) ** 2 + (y[:, None] - py[None, :]) ** 2, axis=1)
                path_error = np.hypot(x - px[nearest], y - py[nearest])
                heading_error = np.abs(np.arctan2(np.sin(theta - local_headings[nearest]),
                                                  np.cos(theta - local_headings[nearest])))
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
                costs += self.steering_weight * (delta ** 2)
            costs += self.terminal_distance_weight * np.hypot(trajectories[:, -1, 0] - px[target_idx],
                                    trajectories[:, -1, 1] - py[target_idx])

            baseline = float(np.min(costs))
            weights = np.exp(np.clip(-(costs - baseline) / self.temperature, -60.0, 0.0))
            weights /= max(float(weights.sum()), 1e-12)
            nominal_v = np.clip(nominal_v + np.sum(weights[:, None] * noise_v, axis=0),
                                cfg.min_velocity, cfg.max_velocity)
            nominal_delta = np.clip(nominal_delta + np.sum(weights[:, None] * noise_delta, axis=0),
                                    steering_min, steering_max)

        best_idx = int(np.argmin(costs))
        if costs[best_idx] >= self.collision_cost:
            self.best_local_path = []
            return 0.0, 0.0
        best = trajectories[best_idx]
        # Return the best collision-checked rollout transformed into the route's global frame.
        global_path = [{"x": float(rx), "y": float(ry), "heading": float(heading)}]
        for x, y, theta in best:
            gx = rx + x * c - y * s
            gy = ry + x * s + y * c
            gh = heading + theta
            global_path.append({"x": float(gx), "y": float(gy), "heading": float(gh)})
            if math.hypot(x, y) >= max_lookahead:
                break
        self.best_local_path = global_path

        self._nominal_v[:-1] = nominal_v[1:]
        self._nominal_v[-1] = nominal_v[-1]
        self._nominal_delta[:-1] = nominal_delta[1:]
        self._nominal_delta[-1] = nominal_delta[-1]
        return float(nominal_v[0]), float(nominal_delta[0])
