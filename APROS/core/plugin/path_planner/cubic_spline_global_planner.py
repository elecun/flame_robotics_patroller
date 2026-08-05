"""
Cubic Spline Global Planner implementation for Ackermann Mobile Robots.
References Cubic Spline interpolation algorithms from PythonRobotics.
"""
import math
import numpy as np
from typing import List, Dict, Tuple
from core.plugin.path_planner.robot_config import RobotConfig
from core.plugin.path_planner.base_global_planner import BaseGlobalPlanner


class CubicSpline1D:
    """
    1D Cubic Spline Interpolator class.
    Calculates 1D position, 1st derivative (slope/velocity), and 2nd derivative (acceleration).
    """

    def __init__(self, x: List[float], y: List[float]):
        h = np.diff(x)
        if np.any(h <= 0):
            raise ValueError("x coordinates must be strictly increasing for 1D cubic spline.")

        self.x = x
        self.y = y
        self.nx = len(x)
        self.a = list(y)

        # Build tridiagonal matrix A and vector b for spline coefficients c
        A = np.zeros((self.nx, self.nx))
        b = np.zeros(self.nx)
        A[0, 0] = 1.0
        A[self.nx - 1, self.nx - 1] = 1.0

        for i in range(1, self.nx - 1):
            A[i, i - 1] = h[i - 1]
            A[i, i] = 2.0 * (h[i - 1] + h[i])
            A[i, i + 1] = h[i]
            b[i] = 3.0 * (self.a[i + 1] - self.a[i]) / h[i] - 3.0 * (self.a[i] - self.a[i - 1]) / h[i - 1]

        self.c = np.linalg.solve(A, b)
        self.d = [(self.c[i + 1] - self.c[i]) / (3.0 * h[i]) for i in range(self.nx - 1)]
        self.b = [(self.a[i + 1] - self.a[i]) / h[i] - h[i] * (2.0 * self.c[i] + self.c[i + 1]) / 3.0 for i in range(self.nx - 1)]

    def calc_position(self, x_val: float) -> float:
        if x_val < self.x[0] or x_val > self.x[-1]:
            x_val = np.clip(x_val, self.x[0], self.x[-1])

        i = self._search_index(x_val)
        dx = x_val - self.x[i]
        return self.a[i] + self.b[i] * dx + self.c[i] * (dx ** 2) + self.d[i] * (dx ** 3)

    def calc_first_derivative(self, x_val: float) -> float:
        if x_val < self.x[0] or x_val > self.x[-1]:
            x_val = np.clip(x_val, self.x[0], self.x[-1])

        i = self._search_index(x_val)
        dx = x_val - self.x[i]
        return self.b[i] + 2.0 * self.c[i] * dx + 3.0 * self.d[i] * (dx ** 2)

    def calc_second_derivative(self, x_val: float) -> float:
        if x_val < self.x[0] or x_val > self.x[-1]:
            x_val = np.clip(x_val, self.x[0], self.x[-1])

        i = self._search_index(x_val)
        dx = x_val - self.x[i]
        return 2.0 * self.c[i] + 6.0 * self.d[i] * dx

    def _search_index(self, x_val: float) -> int:
        for i in range(self.nx - 1):
            if self.x[i] <= x_val <= self.x[i + 1]:
                return i
        return self.nx - 2


class CubicSpline2D:
    """
    2D Parametric Cubic Spline interpolator along arc-length s.
    Generates smooth 2D (x, y) trajectory, heading theta, and curvature kappa.
    """

    def __init__(self, x: List[float], y: List[float]):
        self.s = self._calc_s(x, y)
        self.sx = CubicSpline1D(self.s, x)
        self.sy = CubicSpline1D(self.s, y)

    def _calc_s(self, x: List[float], y: List[float]) -> List[float]:
        dx = np.diff(x)
        dy = np.diff(y)
        ds = np.hypot(dx, dy)
        s = [0.0]
        s.extend(np.cumsum(ds))
        return s

    def calc_position(self, s_val: float) -> Tuple[float, float]:
        x = self.sx.calc_position(s_val)
        y = self.sy.calc_position(s_val)
        return x, y

    def calc_curvature(self, s_val: float) -> float:
        dx = self.sx.calc_first_derivative(s_val)
        ddx = self.sx.calc_second_derivative(s_val)
        dy = self.sy.calc_first_derivative(s_val)
        ddy = self.sy.calc_second_derivative(s_val)
        curvature = (ddy * dx - ddx * dy) / ((dx ** 2 + dy ** 2) ** 1.5 + 1e-9)
        return curvature

    def calc_yaw(self, s_val: float) -> float:
        dx = self.sx.calc_first_derivative(s_val)
        dy = self.sy.calc_first_derivative(s_val)
        return math.atan2(dy, dx)


class CubicSplineGlobalPlanner(BaseGlobalPlanner):
    """
    Global Path Planner using Parametric 2D Cubic Spline Interpolation and
    curvature-based reference velocity profiling.
    """

    def __init__(self, config: RobotConfig):
        super().__init__(config)

    def plan(self, waypoints: List[Tuple[float, float]]) -> List[Dict[str, float]]:
        """
        Generate continuous global sub-path from input waypoints.

        Args:
            waypoints: List of (x, y) waypoint tuples in ENU coordinates.

        Returns:
            List[Dict[str, float]]: Sub-path dictionaries containing 'x', 'y', 'heading', 'curvature', 'v_ref'.
        """
        if not waypoints or len(waypoints) < 2:
            return []

        # Remove consecutive duplicate waypoints if any
        clean_wp = [waypoints[0]]
        for wp in waypoints[1:]:
            if math.hypot(wp[0] - clean_wp[-1][0], wp[1] - clean_wp[-1][1]) > 0.01:
                clean_wp.append(wp)

        if len(clean_wp) < 2:
            return []

        x_pts = [wp[0] for wp in clean_wp]
        y_pts = [wp[1] for wp in clean_wp]

        # Handle linear 2-point case by adding midpoint for spline stability
        if len(clean_wp) == 2:
            mx = (x_pts[0] + x_pts[1]) / 2.0
            my = (y_pts[0] + y_pts[1]) / 2.0
            x_pts.insert(1, mx)
            y_pts.insert(1, my)

        sp = CubicSpline2D(x_pts, y_pts)
        s_total = sp.s[-1]
        ds = self.config.ds

        s_sample = np.arange(0.0, s_total, ds)
        if len(s_sample) == 0 or s_sample[-1] < s_total:
            s_sample = np.append(s_sample, s_total)

        local_path: List[Dict[str, float]] = []

        for s in s_sample:
            px, py = sp.calc_position(s)
            heading = sp.calc_yaw(s)
            curvature = sp.calc_curvature(s)

            # Curvature-based velocity profiling: v_ref = sqrt(a_lat_max / |kappa|)
            if abs(curvature) > 1e-4:
                v_curve = math.sqrt(self.config.max_lat_accel / abs(curvature))
            else:
                v_curve = self.config.max_velocity

            # Clip velocity to [min_velocity, max_velocity]
            v_ref = max(self.config.min_velocity, min(v_curve, self.config.max_velocity))

            local_path.append({
                "x": float(px),
                "y": float(py),
                "heading": float(heading),
                "curvature": float(curvature),
                "v_ref": float(v_ref)
            })

        # Decelerate near final goal destination (last 2 meters)
        decel_distance = 2.0
        n_points = len(local_path)
        for i in range(n_points):
            dist_to_end = (n_points - 1 - i) * ds
            if dist_to_end < decel_distance:
                v_decel = self.config.max_velocity * (dist_to_end / decel_distance)
                local_path[i]["v_ref"] = min(local_path[i]["v_ref"], max(0.0, v_decel))

        return local_path
