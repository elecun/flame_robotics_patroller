"""
Base Ground Removal Abstract Class Module (core/plugin/ground_removal/base_ground_removal.py).
Defines standard interface for ground removal algorithms on LiDAR point cloud data.

Coordinate Frame Standard:
  - X axis: Forward (robot heading direction)
  - Y axis: Left (perpendicular to heading)
  - Z axis: Up (vertical height direction)
"""

from abc import ABC, abstractmethod
import numpy as np


class BaseGroundRemoval(ABC):
    """
    Abstract base class for LiDAR ground removal algorithms.
    
    All point clouds passed to `remove_ground` MUST follow the standard robot frame coordinate system:
      - +X : Forward
      - +Y : Left
      - +Z : Up
    """

    def __init__(self, name: str = "base_ground_removal"):
        """
        Initialize the base ground removal module.

        :param name: Unique algorithm identifier name.
        """
        self.name = name

    @abstractmethod
    def remove_ground(self, points: np.ndarray) -> np.ndarray:
        """
        Filter and remove ground points from the input point cloud.

        :param points: Point cloud array of shape (N, 3) or (N, C) with (x, y, z, ...).
                       Coordinate convention: X=forward, Y=left, Z=up.
        :return: Filtered point cloud array (non-ground points) of shape (M, C).
        """
        pass
