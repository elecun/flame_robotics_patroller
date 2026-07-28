"""
Global Path Planner Abstract Interface Class for APROS Plugins (core/plugin/global_path_planner.py).
Defines standardized interfaces for global route and motion planning algorithms
(e.g., A*, Dijkstra, RRT*, Hybrid A*, PRM, Breadth-First Search).
"""

from abc import abstractmethod
from typing import List, Tuple, Optional, Dict, Any, Union
import numpy as np
from core.plugin.base import BasePlugin


class BaseGlobalPathPlanner(BasePlugin):
    """
    Abstract Base Class for Global Path Planning Plugins.
    Inherits from BasePlugin to provide ZPipe context sharing and modular lifecycle management.
    """

    def __init__(self, name: str = "global_path_planner"):
        super().__init__(name)
        self.map_grid: Optional[np.ndarray] = None  # 2D Grid map matrix (0: Free, 1: Obstacle)
        self.resolution: float = 0.1  # Grid resolution (meters per pixel/cell)
        self.origin: Tuple[float, float] = (0.0, 0.0)  # Map origin world coordinates (x_min, y_min)
        self.last_path: Optional[np.ndarray] = None  # Planned path array of shape (N, 2) [x, y] or (N, 3) [x, y, yaw]

    def initialize(self, config: dict) -> bool:
        """
        Initialize planner parameters from config dictionary.
        """
        self.resolution = float(config.get("resolution", 0.1))
        orig = config.get("origin", (0.0, 0.0))
        self.origin = (float(orig[0]), float(orig[1]))
        return True

    def set_map(self, map_grid: np.ndarray, resolution: float = 0.1, origin: Tuple[float, float] = (0.0, 0.0)):
        """
        Set occupancy grid map matrix and spatial parameters.
        :param map_grid: 2D numpy array where 0 = Free space, 1 (or 100) = Occupied obstacle
        :param resolution: Map resolution in meters/cell
        :param origin: World coordinates of map cell (0, 0)
        """
        self.map_grid = map_grid
        self.resolution = float(resolution)
        self.origin = (float(origin[0]), float(origin[1]))

    @abstractmethod
    def plan(
        self,
        start: Tuple[float, float],
        goal: Tuple[float, float],
        ox: Optional[List[float]] = None,
        oy: Optional[List[float]] = None
    ) -> Tuple[Optional[List[float]], Optional[List[float]]]:
        """
        Abstract method to compute global path from start to goal coordinates.
        
        :param start: Start pose (x, y) in world coordinates (meters)
        :param goal: Goal pose (x, y) in world coordinates (meters)
        :param ox: List of obstacle x-coordinates (optional)
        :param oy: List of obstacle y-coordinates (optional)
        :return: Tuple (rx, ry) representing planned waypoint x and y lists in world coordinates.
                 Returns (None, None) if path generation fails or no path exists.
        """
        pass

    def process(self, data: dict) -> dict:
        """
        BasePlugin process interface implementation.
        Expects data dict containing 'start' and 'goal' keys.
        Returns data dict with 'path_x' and 'path_y'.
        """
        start = data.get("start")
        goal = data.get("goal")
        ox = data.get("ox")
        oy = data.get("oy")

        if not start or not goal:
            return {"status": "error", "message": "Missing 'start' or 'goal' in process input data."}

        rx, ry = self.plan(start=start, goal=goal, ox=ox, oy=oy)

        if rx is not None and ry is not None:
            self.last_path = np.column_stack([rx, ry])
            return {
                "status": "success",
                "path_x": rx,
                "path_y": ry,
                "num_waypoints": len(rx)
            }
        else:
            return {
                "status": "failed",
                "message": "Path planning failed.",
                "path_x": None,
                "path_y": None
            }
