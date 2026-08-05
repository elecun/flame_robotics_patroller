"""
Abstract Base Class interface for Local Path Planners.
"""
from abc import ABC, abstractmethod
from typing import List, Dict, Tuple
from core.plugin.path_planner.robot_config import RobotConfig


class BaseLocalPlanner(ABC):
    """
    Abstract Base Class for Local Path Planners in Ackermann Mobile Robots.
    Computes real-time velocity (v) and steering angle (delta) control commands
    based on current pose, reference global path, and obstacle observations.
    """

    def __init__(self, config: RobotConfig):
        """
        Initialize Local Planner with robot configuration.

        Args:
            config (RobotConfig): Robot physical specifications and kinematic limits.
        """
        self.config = config

    @abstractmethod
    def compute_velocity_commands(
        self,
        current_pose: Dict[str, float],
        current_vel: float,
        local_path: List[Dict[str, float]],
        obstacle_points: List[Tuple[float, float]]
    ) -> Tuple[float, float]:
        """
        Compute control commands (v, delta) for Ackermann vehicle.

        Args:
            current_pose (Dict[str, float]): Current robot pose {'x': float, 'y': float, 'heading': float}
            current_vel (float): Current forward linear velocity (m/s)
            local_path (List[Dict[str, float]]): Target reference path points [{'x', 'y', 'heading', 'curvature', 'v_ref'}]
            obstacle_points (List[Tuple[float, float]]): List of obstacle (x, y) coordinates in global frame

        Returns:
            Tuple[float, float]: Control command pair (v, delta):
                - v: Forward linear velocity in m/s
                - delta: Front wheel steering angle in radians [-max_steer_angle, +max_steer_angle]
        """
        pass
