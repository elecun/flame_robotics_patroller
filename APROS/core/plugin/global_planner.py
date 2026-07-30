"""
GlobalPlanner module for APROS plugin system (core/plugin/global_planner.py).
Defines abstract and concrete global path planner plugins.
"""

from abc import abstractmethod
from typing import List, Tuple, Optional, Any
import numpy as np
from core.plugin.base import BasePlugin
from util.logger.console import ConsoleLogger

logger = ConsoleLogger.get_logger()


class BaseGlobalPlanner(BasePlugin):
    """
    Abstract Base Class for Global Path Planner Plugins.
    Provides standardized interface for global route generation from start pose to target goal pose.
    """

    def __init__(self, name: str = "global_planner"):
        super().__init__(name)
        self.start_pose: Optional[Tuple[float, float, float]] = None  # (x, y, yaw)
        self.goal_pose: Optional[Tuple[float, float, float]] = None   # (x, y, yaw)
        self.global_path: List[Tuple[float, float]] = []              # [(x0, y0), (x1, y1), ...]

    def initialize(self, config: dict) -> bool:
        """Initialize global planner configuration."""
        return True

    def process(self, data: dict) -> dict:
        """
        Process request for global path planning.
        Expected data keys: 'start', 'goal', 'obstacles_x', 'obstacles_y'
        """
        start = data.get("start")
        goal = data.get("goal")
        ox = data.get("obstacles_x", [])
        oy = data.get("obstacles_y", [])

        if start is not None and goal is not None:
            path = self.make_plan(start, goal, ox, oy)
            return {"path": path, "success": len(path) > 0}
        return {"path": [], "success": False}

    @abstractmethod
    def make_plan(
        self,
        start: Tuple[float, float],
        goal: Tuple[float, float],
        ox: Optional[List[float]] = None,
        oy: Optional[List[float]] = None
    ) -> List[Tuple[float, float]]:
        """
        Abstract method to generate global path from start to goal coordinates.
        :param start: Starting (x, y) world position
        :param goal: Goal (x, y) world position
        :param ox: Obstacle x-coordinates
        :param oy: Obstacle y-coordinates
        :return: List of (x, y) waypoint tuples representing global path
        """
        pass
