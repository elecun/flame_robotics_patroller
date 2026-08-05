"""
Abstract Base Class interface for Global Path Planners.
"""
from abc import ABC, abstractmethod
from typing import List, Dict, Tuple, Any
from core.plugin.path_planner.robot_config import RobotConfig


class BaseGlobalPlanner(ABC):
    """
    Abstract Base Class for Global Path Planners in Ackermann Mobile Robots.
    Standardizes global sub-path generation from raw waypoints.
    """

    def __init__(self, config: RobotConfig):
        """
        Initialize Global Planner with robot configuration.
        
        Args:
            config (RobotConfig): Robot physical specifications and kinematic limits.
        """
        self.config = config

    @abstractmethod
    def plan(self, waypoints: List[Tuple[float, float]]) -> List[Dict[str, float]]:
        """
        Interpolate raw waypoints into a smooth, continuous global sub-path with heading,
        curvature, and reference velocity profile.

        Args:
            waypoints (List[Tuple[float, float]]): List of 2D waypoints in ENU coordinate frame [(x0, y0), (x1, y1), ...]

        Returns:
            List[Dict[str, float]]: List of path point dicts containing:
                - 'x': float - X position in meters
                - 'y': float - Y position in meters
                - 'heading': float - Heading angle theta in radians [-pi, pi]
                - 'curvature': float - Path curvature kappa (1/m)
                - 'v_ref': float - Reference velocity in m/s
        """
        pass
