"""
CostMap module for APROS plugin system (core/plugin/cost_map.py).
Manages 2D Occupancy Grid Costmap for local and global motion planning.
"""

from typing import Tuple, List, Optional, Union
import numpy as np
from core.plugin.base import BasePlugin
from util.logger.console import ConsoleLogger

logger = ConsoleLogger.get_logger()


class CostMap(BasePlugin):
    """
    2D Occupancy CostMap plugin.
    Stores and updates 2D grid cell costs (0: Free, 100/255: Occupied, 1~99: Inflation cost).
    """

    def __init__(
        self,
        name: str = "cost_map",
        width: float = 50.0,
        height: float = 50.0,
        resolution: float = 0.1,
        origin: Tuple[float, float] = (-25.0, -25.0),
        inflation_radius: float = 0.5
    ):
        super().__init__(name)
        self.width = float(width)  # Width in meters
        self.height = float(height)  # Height in meters
        self.resolution = float(resolution)  # Grid resolution (m/cell)
        self.origin = (float(origin[0]), float(origin[1]))  # (x_min, y_min)
        self.inflation_radius = float(inflation_radius)

        # Calculate grid dimensions
        self.nx = int(np.round(self.width / self.resolution))
        self.ny = int(np.round(self.height / self.resolution))

        # 2D Grid map: 0 (free) to 100 (occupied)
        self.grid = np.zeros((self.ny, self.nx), dtype=np.uint8)

    def initialize(self, config: dict) -> bool:
        """Initialize CostMap from config dictionary."""
        self.width = float(config.get("width", self.width))
        self.height = float(config.get("height", self.height))
        self.resolution = float(config.get("resolution", self.resolution))
        orig = config.get("origin", self.origin)
        self.origin = (float(orig[0]), float(orig[1]))
        self.inflation_radius = float(config.get("inflation_radius", self.inflation_radius))

        self.nx = int(np.round(self.width / self.resolution))
        self.ny = int(np.round(self.height / self.resolution))
        self.grid = np.zeros((self.ny, self.nx), dtype=np.uint8)
        logger.info(f"[{self.name}] Initialized CostMap grid ({self.nx}x{self.ny}, res: {self.resolution}m)")
        return True

    def process(self, data: dict) -> dict:
        """
        Process incoming sensor/map data.
        Expected keys in data: 'obstacles_x', 'obstacles_y' or 'point_cloud'
        """
        if "obstacles_x" in data and "obstacles_y" in data:
            self.update_obstacles(data["obstacles_x"], data["obstacles_y"])
        return {"grid": self.grid, "resolution": self.resolution, "origin": self.origin}

    def update_obstacles(self, ox: Union[List[float], np.ndarray], oy: Union[List[float], np.ndarray]):
        """Clear grid and insert obstacle coordinates."""
        self.grid.fill(0)
        ox = np.asarray(ox)
        oy = np.asarray(oy)

        # Convert world coordinates to grid indices
        ix = np.round((ox - self.origin[0]) / self.resolution).astype(int)
        iy = np.round((oy - self.origin[1]) / self.resolution).astype(int)

        # Filter valid indices within grid bounds
        valid = (ix >= 0) & (ix < self.nx) & (iy >= 0) & (iy < self.ny)
        self.grid[iy[valid], ix[valid]] = 100

        if self.inflation_radius > 0.0:
            self._inflate_obstacles()

    def _inflate_obstacles(self):
        """Apply obstacle inflation based on inflation_radius."""
        cell_radius = int(np.ceil(self.inflation_radius / self.resolution))
        occupied_y, occupied_x = np.where(self.grid == 100)

        for cy, cx in zip(occupied_y, occupied_x):
            min_x = max(0, cx - cell_radius)
            max_x = min(self.nx, cx + cell_radius + 1)
            min_y = max(0, cy - cell_radius)
            max_y = min(self.ny, cy + cell_radius + 1)

            for y in range(min_y, max_y):
                for x in range(min_x, max_x):
                    dist = math.hypot(x - cx, y - cy) * self.resolution
                    if dist <= self.inflation_radius and self.grid[y, x] < 100:
                        cost = int(100 * (1.0 - dist / self.inflation_radius))
                        self.grid[y, x] = max(self.grid[y, x], cost)

    def world_to_grid(self, gx: float, gy: float) -> Tuple[int, int]:
        """Convert world coordinates (x, y) to grid indices (ix, iy)."""
        ix = int(np.round((gx - self.origin[0]) / self.resolution))
        iy = int(np.round((gy - self.origin[1]) / self.resolution))
        return ix, iy

    def grid_to_world(self, ix: int, iy: int) -> Tuple[float, float]:
        """Convert grid indices (ix, iy) to world coordinates (x, y)."""
        wx = ix * self.resolution + self.origin[0]
        wy = iy * self.resolution + self.origin[1]
        return wx, wy

    def is_occupied(self, x: float, y: float) -> bool:
        """Check if world position (x, y) is occupied by obstacle."""
        ix, iy = self.world_to_grid(x, y)
        if 0 <= ix < self.nx and 0 <= iy < self.ny:
            return self.grid[iy, ix] >= 50
        return True  # Out of bounds treated as occupied
