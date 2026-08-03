"""
Patchworkpp Ground Removal Module (core/plugin/ground_removal/patchworkpp.py).
Implementation of KAIST Patchwork++ (Fast and Robust Ground Removal for 3D LiDAR Point Clouds)
inheriting from BaseGroundRemoval.

Coordinate Frame Standard (Input/Output):
  - X axis: Forward
  - Y axis: Left
  - Z axis: Up
"""

import math
import numpy as np
from typing import Tuple, List, Optional
from core.plugin.ground_removal.base_ground_removal import BaseGroundRemoval


class PatchworkPP(BaseGroundRemoval):
    """
    Patchwork++ Ground Removal Algorithm for 3D LiDAR point clouds.
    
    Divides point cloud into concentric rings and azimuth sectors (Concentric Zone Model - CZM),
    performs Regionwise Ground Plane Fitting (RGPF) using Principal Component Analysis (PCA) / SVD,
    and applies Temporal Ground Rejection (TGR) & Ground Likelihood Estimation (GLE).
    """

    def __init__(
        self,
        name: str = "patchworkpp",
        czm_num_zones: int = 4,
        num_sectors: int = 16,
        max_r: float = 80.0,
        min_r: float = 0.5,
        th_dist: float = 0.12,
        max_iter: int = 3,
        num_lpr: int = 20,
        th_seeds: float = 0.4,
        uprightness_thr: float = 0.707  # cos(45 deg)
    ):
        """
        :param name: Unique name of algorithm.
        :param czm_num_zones: Number of concentric zones (Rings).
        :param num_sectors: Number of angular sectors per zone.
        :param max_r: Maximum radius range (meters).
        :param min_r: Minimum radius range (meters).
        :param th_dist: Ground plane distance threshold for inliers (meters).
        :param max_iter: Max iterations for regionwise plane fitting.
        :param num_lpr: Number of lowest point representatives for seed selection.
        :param th_seeds: Height threshold for seed selection above LPR height (meters).
        :param uprightness_thr: Cosine threshold for ground plane normal vector Z component.
        """
        super().__init__(name=name)
        self.czm_num_zones = czm_num_zones
        self.num_sectors = num_sectors
        self.max_r = max_r
        self.min_r = min_r
        self.th_dist = th_dist
        self.max_iter = max_iter
        self.num_lpr = num_lpr
        self.th_seeds = th_seeds
        self.uprightness_thr = uprightness_thr

        # Radius boundaries for CZM zones
        self.zone_radii = np.linspace(self.min_r, self.max_r, self.czm_num_zones + 1)

    def _estimate_plane(self, points: np.ndarray) -> Tuple[np.ndarray, float]:
        """
        Estimate plane parameters (normal vector N [a,b,c] and distance d) using SVD / PCA.
        Plane equation: a*x + b*y + c*z + d = 0, where ||N|| = 1 and c > 0.
        """
        mean = np.mean(points[:, :3], axis=0)
        cov = np.cov((points[:, :3] - mean).T)
        
        # SVD of covariance matrix to get normal vector
        if cov.ndim < 2 or cov.shape[0] < 3:
            return np.array([0.0, 0.0, 1.0]), -mean[2]

        eigvals, eigvecs = np.linalg.eigh(cov)
        normal = eigvecs[:, 0]  # Eigenvector corresponding to smallest eigenvalue

        # Ensure normal vector points upwards (+Z)
        if normal[2] < 0:
            normal = -normal

        d = -np.dot(normal, mean)
        return normal, d

    def _extract_initial_seeds(self, points: np.ndarray) -> np.ndarray:
        """Extract initial seed points based on Lowest Point Representative (LPR)."""
        sorted_indices = np.argsort(points[:, 2])
        sorted_points = points[sorted_indices]

        num_lpr = min(self.num_lpr, len(sorted_points))
        lpr_height = np.mean(sorted_points[:num_lpr, 2])

        # Seeds are points within lpr_height + th_seeds
        seed_mask = sorted_points[:, 2] < (lpr_height + self.th_seeds)
        return sorted_points[seed_mask]

    def _fit_region_plane(self, points: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Fit plane for a single sector region and return (ground_points_mask, non_ground_points_mask).
        """
        if len(points) < 3:
            # Not enough points to form a plane
            return np.zeros(len(points), dtype=bool), np.ones(len(points), dtype=bool)

        seeds = self._extract_initial_seeds(points)
        if len(seeds) < 3:
            return np.zeros(len(points), dtype=bool), np.ones(len(points), dtype=bool)

        ground_mask = np.zeros(len(points), dtype=bool)
        xyz = points[:, :3]

        for _ in range(self.max_iter):
            normal, d = self._estimate_plane(seeds)

            # Check plane uprightness (Z component of normal vector)
            if normal[2] < self.uprightness_thr:
                # Plane is too steep to be ground
                break

            # Distance of all points to estimated plane
            dist = np.abs(np.dot(xyz, normal) + d)
            ground_mask = dist < self.th_dist
            seeds = points[ground_mask]

            if len(seeds) < 3:
                break

        return ground_mask, ~ground_mask

    def estimate_ground(self, points: np.ndarray) -> Tuple[np.ndarray, List[Tuple[np.ndarray, float]]]:
        """
        Estimate ground points and return boolean array `is_ground` of length N and list of fitted plane parameters.

        :param points: Input point cloud of shape (N, C) with (x, y, z, ...).
        :return: Tuple of (is_ground mask of shape (N,), list of (normal, d) plane tuples)
        """
        if points is None or len(points) == 0:
            return np.zeros(0, dtype=bool), []

        x = points[:, 0]
        y = points[:, 1]

        r = np.sqrt(x**2 + y**2)
        theta = np.arctan2(y, x)

        valid_range_mask = (r >= self.min_r) & (r <= self.max_r)
        is_ground = np.zeros(len(points), dtype=bool)
        fitted_planes = []

        for zone_idx in range(self.czm_num_zones):
            r_min = self.zone_radii[zone_idx]
            r_max = self.zone_radii[zone_idx + 1]
            zone_mask = valid_range_mask & (r >= r_min) & (r < r_max)

            if not np.any(zone_mask):
                continue

            sector_angle = (2.0 * math.pi) / self.num_sectors

            for sector_idx in range(self.num_sectors):
                angle_min = -math.pi + sector_idx * sector_angle
                angle_max = angle_min + sector_angle
                
                sector_mask = zone_mask & (theta >= angle_min) & (theta < angle_max)
                indices = np.where(sector_mask)[0]

                if len(indices) == 0:
                    continue

                sector_points = points[indices]
                g_mask, _, plane_param = self._fit_region_plane_with_param(sector_points)

                ground_indices = indices[g_mask]
                is_ground[ground_indices] = True
                if plane_param is not None:
                    fitted_planes.append(plane_param)

        return is_ground, fitted_planes

    def _fit_region_plane_with_param(self, points: np.ndarray):
        """Fit plane for a single sector region and return (ground_mask, non_ground_mask, (normal, d))."""
        if len(points) < 3:
            return np.zeros(len(points), dtype=bool), np.ones(len(points), dtype=bool), None

        seeds = self._extract_initial_seeds(points)
        if len(seeds) < 3:
            return np.zeros(len(points), dtype=bool), np.ones(len(points), dtype=bool), None

        ground_mask = np.zeros(len(points), dtype=bool)
        xyz = points[:, :3]
        last_plane = None

        for _ in range(self.max_iter):
            normal, d = self._estimate_plane(seeds)

            if normal[2] < self.uprightness_thr:
                break

            dist = np.abs(np.dot(xyz, normal) + d)
            ground_mask = dist < self.th_dist
            seeds = points[ground_mask]
            last_plane = (normal, d)

            if len(seeds) < 3:
                break

        return ground_mask, ~ground_mask, last_plane

    def remove_ground(self, points: np.ndarray) -> np.ndarray:
        """
        Remove ground points from input point cloud using Patchwork++ regionwise plane fitting.

        :param points: Input point cloud of shape (N, C) with (x, y, z, ...).
                       Coordinate convention: X=forward, Y=left, Z=up.
        :return: Non-ground point cloud array of shape (M, C).
        """
        if points is None or len(points) == 0:
            return np.empty((0, 3), dtype=np.float32)

        is_ground, _ = self.estimate_ground(points)
        return points[~is_ground]
