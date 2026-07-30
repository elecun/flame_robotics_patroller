"""
LocalPlanner module for APROS plugin system (core/plugin/local_planner.py).
Provides abstract base class for local trajectory & velocity control algorithms,
and concrete Dynamic Window Approach (DWA) implementation.
"""

from abc import abstractmethod
from typing import Tuple, List, Optional, Dict, Any, Union
import math
import numpy as np
from core.plugin.base import BasePlugin
from util.logger.console import ConsoleLogger

logger = ConsoleLogger.get_logger()


class BaseLocalPlanner(BasePlugin):
    """
    Abstract Base Class for Local Motion Planners.
    Calculates target linear velocity (v) and steering angle (or angular velocity w)
    to steer the mobile robot toward the next target waypoint while avoiding local obstacles.
    """

    def __init__(self, name: str = "local_planner"):
        super().__init__(name)
        self.target_waypoint: Optional[Tuple[float, float]] = None

    def initialize(self, config: dict) -> bool:
        """Initialize local planner parameters."""
        return True

    def set_target_waypoint(self, waypoint: Tuple[float, float]):
        """Set next target waypoint (x, y) in world coordinates."""
        self.target_waypoint = (float(waypoint[0]), float(waypoint[1]))

    def process(self, data: dict) -> dict:
        """
        Process local planning tick.
        Expected input data:
          - state: tuple (x, y, yaw, v, w) or dict containing robot state
          - goal: tuple (gx, gy)
          - obstacles: list or array of obstacle (x, y) coordinates
        Returns dict containing calculated velocity and steering output:
          - speed: linear velocity [m/s or km/h]
          - steer_angle: steering angle [deg]
        """
        state = data.get("state", (0.0, 0.0, 0.0, 0.0, 0.0))
        goal = data.get("goal", self.target_waypoint)
        obstacles = data.get("obstacles", np.empty((0, 2)))

        if goal is None:
            return {"speed": 0.0, "steer_angle": 0.0, "v": 0.0, "w": 0.0}

        speed, steer_angle, v, w = self.compute_control(state, goal, obstacles)
        return {
            "speed": speed,           # km/h
            "steer_angle": steer_angle, # degrees
            "v": v,                   # m/s
            "w": w                    # rad/s
        }

    @abstractmethod
    def compute_control(
        self,
        state: Tuple[float, float, float, float, float],
        goal: Tuple[float, float],
        obstacles: Union[List[Tuple[float, float]], np.ndarray]
    ) -> Tuple[float, float, float, float]:
        """
        Abstract control computation method.
        
        :param state: Current robot state (x [m], y [m], yaw [rad], v [m/s], w [rad/s])
        :param goal: Target goal waypoint (gx [m], gy [m])
        :param obstacles: Array of local obstacle coordinates [[ox0, oy0], [ox1, oy1], ...]
        :return: Tuple (speed_kmh, steer_angle_deg, v_ms, w_rads)
                 - speed_kmh: Mobile drive linear speed command in km/h
                 - steer_angle_deg: Steering angle command in degrees
                 - v_ms: Linear velocity in m/s
                 - w_rads: Angular velocity in rad/s
        """
        pass


class DWAPlanner(BaseLocalPlanner):
    """
    Dynamic Window Approach (DWA) Local Path Planner.
    Calculates optimal (v, w) trajectory sampling based on PythonRobotics DWA algorithm,
    and converts outputs to linear speed (km/h) and steering angle (deg) for Ackermann/Unicycle drive.
    """

    def __init__(
        self,
        name: str = "dwa_planner",
        max_speed: float = 1.38,       # max linear speed [m/s] (~5.0 km/h)
        min_speed: float = -0.5,      # min linear speed [m/s]
        max_yaw_rate: float = 0.4886,  # max steering/yaw rate [rad/s] (~28 deg)
        max_accel: float = 0.5,       # max linear acceleration [m/s^2]
        max_delta_yaw_rate: float = 1.0, # max angular acceleration [rad/s^2]
        v_resolution: float = 0.05,   # velocity sampling resolution [m/s]
        w_resolution: float = 0.02,   # yaw rate sampling resolution [rad/s]
        dt: float = 0.1,              # simulation time step [s]
        predict_time: float = 2.0,    # trajectory prediction time [s]
        to_goal_cost_gain: float = 0.15,
        speed_cost_gain: float = 1.0,
        obstacle_cost_gain: float = 1.0,
        robot_stuck_flag_cons: float = 0.001,
        robot_radius: float = 0.6,    # robot safety radius [m]
        wheelbase: float = 1.15       # Ackermann wheelbase [m]
    ):
        super().__init__(name)
        self.max_speed = float(max_speed)
        self.min_speed = float(min_speed)
        self.max_yaw_rate = float(max_yaw_rate)
        self.max_accel = float(max_accel)
        self.max_delta_yaw_rate = float(max_delta_yaw_rate)
        self.v_resolution = float(v_resolution)
        self.w_resolution = float(w_resolution)
        self.dt = float(dt)
        self.predict_time = float(predict_time)
        self.to_goal_cost_gain = float(to_goal_cost_gain)
        self.speed_cost_gain = float(speed_cost_gain)
        self.obstacle_cost_gain = float(obstacle_cost_gain)
        self.robot_stuck_flag_cons = float(robot_stuck_flag_cons)
        self.robot_radius = float(robot_radius)
        self.wheelbase = float(wheelbase)

    def initialize(self, config: dict) -> bool:
        """Initialize DWA Planner parameters from config dict."""
        self.max_speed = float(config.get("max_speed", self.max_speed))
        self.min_speed = float(config.get("min_speed", self.min_speed))
        self.max_yaw_rate = float(config.get("max_yaw_rate", self.max_yaw_rate))
        self.max_accel = float(config.get("max_accel", self.max_accel))
        self.max_delta_yaw_rate = float(config.get("max_delta_yaw_rate", self.max_delta_yaw_rate))
        self.predict_time = float(config.get("predict_time", self.predict_time))
        self.robot_radius = float(config.get("robot_radius", self.robot_radius))
        self.wheelbase = float(config.get("wheelbase", self.wheelbase))
        logger.info(f"[{self.name}] DWA Planner initialized (max_speed: {self.max_speed * 3.6:.1f}km/h, max_yaw_rate: {math.degrees(self.max_yaw_rate):.1f}deg)")
        return True

    def compute_control(
        self,
        state: Tuple[float, float, float, float, float],
        goal: Tuple[float, float],
        obstacles: Union[List[Tuple[float, float]], np.ndarray]
    ) -> Tuple[float, float, float, float]:
        """
        Compute optimal DWA control output (v, w) and convert to speed (km/h) and steer angle (deg).
        
        :param state: (x, y, yaw, v, w)
        :param goal: (gx, gy)
        :param obstacles: Obstacle array [[ox, oy], ...]
        :return: (speed_kmh, steer_angle_deg, v_best, w_best)
        """
        x, y, yaw, v, w = state
        gx, gy = goal

        # Convert obstacles to numpy array
        ob = np.asarray(obstacles)
        if ob.ndim == 1 or ob.size == 0:
            ob = np.empty((0, 2))

        # Calculate Dynamic Window [v_min, v_max, w_min, w_max]
        dw = self._calc_dynamic_window(v, w)

        # Evaluate trajectories inside Dynamic Window
        best_v, best_w, best_traj = self._calc_control_and_trajectory(x, y, yaw, v, w, dw, gx, gy, ob)

        # Convert outputs:
        # 1. Linear velocity v (m/s) -> speed (km/h)
        speed_kmh = best_v * 3.6

        # 2. Angular velocity w / Steering conversion for Ackermann Kinematics
        # delta = arctan(L * w / v) if v != 0, or direct angle mapping
        if abs(best_v) > 0.05:
            steer_angle_rad = math.atan2(self.wheelbase * best_w, best_v)
        else:
            steer_angle_rad = best_w  # Fallback to direct yaw rate angle

        steer_angle_deg = math.degrees(steer_angle_rad)
        # Clamp steering angle to max limits (e.g. +-28 deg)
        max_steer_deg = math.degrees(self.max_yaw_rate)
        steer_angle_deg = max(-max_steer_deg, min(max_steer_deg, steer_angle_deg))

        return speed_kmh, steer_angle_deg, best_v, best_w

    def _calc_dynamic_window(self, v: float, w: float) -> Tuple[float, float, float, float]:
        """Calculate dynamic window based on robot specifications and current velocity."""
        # Specification limits
        Vs = [self.min_speed, self.max_speed, -self.max_yaw_rate, self.max_yaw_rate]

        # Acceleration limits
        Vd = [
            v - self.max_accel * self.dt,
            v + self.max_accel * self.dt,
            w - self.max_delta_yaw_rate * self.dt,
            w + self.max_delta_yaw_rate * self.dt
        ]

        # Intersection of specification and acceleration limits
        dw = [
            max(Vs[0], Vd[0]),
            min(Vs[1], Vd[1]),
            max(Vs[2], Vd[2]),
            min(Vs[3], Vd[3])
        ]
        return dw[0], dw[1], dw[2], dw[3]

    def _calc_control_and_trajectory(
        self,
        x: float, y: float, yaw: float, v: float, w: float,
        dw: Tuple[float, float, float, float],
        gx: float, gy: float,
        ob: np.ndarray
    ) -> Tuple[float, float, np.ndarray]:
        """Search best control (v, w) trajectory within dynamic window."""
        dw_vmin, dw_vmax, dw_wmin, dw_wmax = dw
        x_init = np.array([x, y, yaw, v, w])

        min_cost = float("inf")
        best_v = 0.0
        best_w = 0.0
        best_traj = np.array([x_init])

        # Grid search over velocity and yaw rate ranges
        v_samples = np.arange(dw_vmin, dw_vmax + 1e-5, self.v_resolution)
        w_samples = np.arange(dw_wmin, dw_wmax + 1e-5, self.w_resolution)

        for sample_v in v_samples:
            for sample_w in w_samples:
                traj = self._predict_trajectory(x_init, sample_v, sample_w)

                # Calculate evaluation costs
                to_goal_cost = self.to_goal_cost_gain * self._calc_to_goal_cost(traj, gx, gy)
                speed_cost = self.speed_cost_gain * (self.max_speed - traj[-1, 3])
                ob_cost = self.obstacle_cost_gain * self._calc_obstacle_cost(traj, ob)

                final_cost = to_goal_cost + speed_cost + ob_cost

                if min_cost >= final_cost:
                    min_cost = final_cost
                    best_v = sample_v
                    best_w = sample_w
                    best_traj = traj

        return best_v, best_w, best_traj

    def _predict_trajectory(self, x_init: np.ndarray, v: float, w: float) -> np.ndarray:
        """Predict robot trajectory over predict_time duration."""
        x = np.array(x_init)
        traj = np.array(x)
        time = 0.0
        while time <= self.predict_time:
            x = self._motion_step(x, v, w, self.dt)
            traj = np.vstack((traj, x))
            time += self.dt
        return traj

    def _motion_step(self, x: np.ndarray, v: float, w: float, dt: float) -> np.ndarray:
        """Unicycle/Ackermann motion model step update."""
        x[2] += w * dt
        x[0] += v * math.cos(x[2]) * dt
        x[1] += v * math.sin(x[2]) * dt
        x[3] = v
        x[4] = w
        return x

    def _calc_to_goal_cost(self, traj: np.ndarray, gx: float, gy: float) -> float:
        """Calculate heading/distance cost to target goal waypoint."""
        dx = gx - traj[-1, 0]
        dy = gy - traj[-1, 1]
        error_angle = math.atan2(dy, dx)
        cost_angle = error_angle - traj[-1, 2]
        cost = abs(math.atan2(math.sin(cost_angle), math.cos(cost_angle)))
        return cost

    def _calc_obstacle_cost(self, traj: np.ndarray, ob: np.ndarray) -> float:
        """Calculate clearance cost to nearest obstacle."""
        if ob.size == 0:
            return 0.0

        ox = ob[:, 0]
        oy = ob[:, 1]
        dx = traj[:, 0, None] - ox[None, :]
        dy = traj[:, 1, None] - oy[None, :]
        r = np.hypot(dx, dy)

        if np.array(r <= self.robot_radius).any():
            return float("inf")

        min_r = np.min(r)
        return 1.0 / min_r if min_r > 0 else float("inf")
