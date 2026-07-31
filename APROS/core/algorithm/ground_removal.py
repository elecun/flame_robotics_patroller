"""
Ground Removal Algorithm Module for VLP-16 Point Cloud Data.
Filters out point cloud data within a virtual ground plane (+30mm / -30mm tolerance around robot ground plane)
taking into account the vehicle tilt_x (pitch angle in degrees) from the Baumer Incline Sensor.
"""

import math
import numpy as np
from typing import Optional


class GroundRemovalFilter:
    """
    Ground removal filter for VLP-16 LiDAR point cloud processing.
    """

    def __init__(
        self,
        vlp16_offset_x: float = 1.027,
        vlp16_offset_y: float = 0.0,
        vlp16_offset_z: float = 0.320,
        vlp16_pitch_deg: float = 15.0,
        ground_safe_m: float = 0.030
    ):
        """
        :param vlp16_offset_x: LiDAR X-offset relative to robot base origin (m)
        :param vlp16_offset_y: LiDAR Y-offset relative to robot base origin (m)
        :param vlp16_offset_z: LiDAR Z-offset relative to robot base origin (m)
        :param vlp16_pitch_deg: LiDAR downward tilt pitch angle in degrees (default: 15.0)
        :param ground_safe_m: Ground safe threshold height in meters (default: 0.030m = +30mm)
        """
        self.vlp16_offset_x = float(vlp16_offset_x)
        self.vlp16_offset_y = float(vlp16_offset_y)
        self.vlp16_offset_z = float(vlp16_offset_z)
        self.vlp16_pitch_deg = float(vlp16_pitch_deg)
        self.ground_safe_m = float(ground_safe_m)

    def remove_ground_points(self, points: np.ndarray, tilt_x_deg: float = 0.0) -> np.ndarray:
        """
        Keeps only point cloud data located ABOVE the ground_safe threshold height (z_robot > ground_safe_m).

        :param points: Point cloud array of shape (N, 3) or (N, 4) in VLP-16 local sensor frame.
        :param tilt_x_deg: Vehicle pitch tilt X angle in degrees from Baumer Incline Sensor.
        :return: Filtered point cloud array containing only points above ground_safe height.
        """
        if points is None or len(points) == 0:
            return np.empty((0, 4 if points is not None and points.shape[1] >= 4 else 3), dtype=np.float32)

        # 1. Combined pitch angle (Sensor installation pitch + Vehicle tilt X)
        total_pitch_rad = math.radians(self.vlp16_pitch_deg + tilt_x_deg)

        # Rotation around Y-axis for local sensor -> robot base orientation transform
        x_s = points[:, 0]
        z_s = points[:, 2]

        cos_p = math.cos(total_pitch_rad)
        sin_p = math.sin(total_pitch_rad)

        # Compute point Z-coordinate in robot ground-relative frame
        z_robot = -x_s * sin_p + z_s * cos_p + self.vlp16_offset_z

        # 2. Keep only points located ABOVE ground_safe threshold height (z_robot > ground_safe_m)
        is_above_ground = z_robot > self.ground_safe_m

        return points[is_above_ground]
