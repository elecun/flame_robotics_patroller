"""
Drive Executor Plugin for APROS Autonomous Mobile Robot Platform.
Integrates Global Path Planning, Local Path Planning (DWA), and Vehicle Control Execution.
Runs at control_freq (default 10 Hz).
"""
import os
import math
import csv
import time
import threading
import numpy as np
from typing import Optional, List, Dict, Tuple, Any

from core.plugin.base import BasePlugin
from core.plugin.path_planner.robot_config import RobotConfig
from core.plugin.path_planner.base_global_planner import BaseGlobalPlanner
from core.plugin.path_planner.base_local_planner import BaseLocalPlanner
from util.logger.console import ConsoleLogger

logger = ConsoleLogger.get_logger()


class DriveExecutor(BasePlugin):
    """
    Drive Executor Plugin.
    Manages autonomous mission route loading, global path generation, real-time local path planning (DWA),
    and periodic vehicle speed/steering command dispatching.
    """

    def __init__(
        self,
        name: str = "drive_executor",
        robot: Optional[Any] = None,
        global_planner: Optional[BaseGlobalPlanner] = None,
        local_planner: Optional[BaseLocalPlanner] = None,
        control_freq: float = 10.0,
        **kwargs
    ):
        super().__init__(name)
        self.robot = robot
        self.global_planner = global_planner
        self.local_planner = local_planner
        self.control_freq = float(control_freq) if control_freq > 0 else 10.0
        self.control_period = 1.0 / self.control_freq

        # Mission state variables
        self.is_active = False
        self.is_paused = False
        self.current_route_file: Optional[str] = None
        self.raw_waypoints: List[Tuple[float, float]] = []
        self.global_path: List[Dict[str, float]] = []
        self.target_waypoint_index: int = 0
        self.goal_reach_threshold: float = 0.5  # meters

        # Control thread management
        self._thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()

    def initialize(self, config: dict) -> bool:
        """Initialize plugin configuration."""
        if "control_freq" in config:
            self.control_freq = float(config["control_freq"])
            self.control_period = 1.0 / self.control_freq
        logger.info(f"[{self.name}] Initialized with control_freq={self.control_freq}Hz (period={self.control_period:.3f}s)")
        return True

    def process(self, data: dict) -> dict:
        """Process arbitrary pipeline telemetry data if needed."""
        return data

    def load_mission_route(self, route_file_name: str) -> bool:
        """
        Load .route CSV file and compute global path using GlobalPlanner.
        
        Args:
            route_file_name: Name of .route file under APROS/route directory.
        """
        if not route_file_name or route_file_name == "None":
            logger.warning(f"[{self.name}] Invalid route file specified.")
            return False

        route_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "route")
        file_path = os.path.join(route_dir, route_file_name)
        if not os.path.exists(file_path):
            file_path = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), "route", route_file_name)

        if not os.path.exists(file_path):
            logger.error(f"[{self.name}] Route file '{route_file_name}' not found.")
            return False

        waypoints = []
        try:
            with open(file_path, "r", encoding="utf-8") as f:
                reader = csv.reader(f)
                header = next(reader, None)
                for row in reader:
                    if len(row) >= 3:
                        try:
                            lat = float(row[1])
                            lon = float(row[2])
                            waypoints.append((lat, lon))
                        except ValueError:
                            continue
        except Exception as e:
            logger.error(f"[{self.name}] Error reading route file '{route_file_name}': {e}")
            return False

        if not waypoints:
            logger.warning(f"[{self.name}] No valid waypoints found in '{route_file_name}'.")
            return False

        # Convert latitude/longitude to relative meter coordinates
        # Frame Requirement: +X North, -X South, -Y East, +Y West
        lat0, lon0 = waypoints[0]
        points_meter = []
        for lat, lon in waypoints:
            dlat = lat - lat0
            dlon = lon - lon0
            dx = dlat * 111000.0
            dy = -dlon * 111000.0 * np.cos(np.radians(lat0))
            points_meter.append((dx, dy))

        self.raw_waypoints = points_meter
        self.current_route_file = route_file_name
        self.target_waypoint_index = 0

        # Run Global Planner to create smooth sub-path
        if self.global_planner is not None:
            self.global_path = self.global_planner.plan(points_meter)
            logger.info(f"[{self.name}] Global path planned for '{route_file_name}': {len(self.global_path)} path points.")
        else:
            # Fallback if global planner is not set
            self.global_path = [
                {"x": pt[0], "y": pt[1], "heading": 0.0, "curvature": 0.0, "v_ref": 1.0}
                for pt in points_meter
            ]
            logger.warning(f"[{self.name}] Global planner not set. Created raw waypoint fallback path.")

        return True

    def start_mission(self, route_file_name: Optional[str] = None) -> bool:
        """
        Start mission route execution thread.
        Triggered when user clicks 'Start Mission' button in Mission Control tab.
        """
        with self._lock:
            if route_file_name and route_file_name != self.current_route_file:
                success = self.load_mission_route(route_file_name)
                if not success:
                    return False
            elif not self.global_path:
                if self.current_route_file:
                    self.load_mission_route(self.current_route_file)
                else:
                    logger.warning(f"[{self.name}] Cannot start mission: No route loaded.")
                    return False

            if self.is_active and not self.is_paused:
                logger.info(f"[{self.name}] Mission is already running.")
                return True

            self.is_active = True
            self.is_paused = False

            if self._thread is None or not self._thread.is_alive():
                self._thread = threading.Thread(target=self._control_loop, daemon=True)
                self._thread.start()
                logger.info(f"[{self.name}] Started autonomous Drive Executor control loop ({self.control_freq} Hz).")

            return True

    def pause_mission(self):
        """Pause mission route execution."""
        with self._lock:
            self.is_paused = True
            logger.info(f"[{self.name}] Mission paused.")
            self._apply_stop_command()

    def resume_mission(self):
        """Resume paused mission route execution."""
        with self._lock:
            if self.is_active:
                self.is_paused = False
                logger.info(f"[{self.name}] Mission resumed.")

    def abort_mission(self):
        """Abort mission route execution and stop vehicle."""
        with self._lock:
            self.is_active = False
            self.is_paused = False
            logger.info(f"[{self.name}] Mission aborted.")
            self._apply_stop_command()

    def _apply_stop_command(self):
        """Send speed=0 and steer=0 command to drive base."""
        if self.robot and hasattr(self.robot, "drive_base") and self.robot.drive_base:
            drive_dev = self.robot.drive_base
            if hasattr(drive_dev, "set_speed"):
                drive_dev.set_speed(0.0)
            if hasattr(drive_dev, "set_steering_angle"):
                drive_dev.set_steering_angle(0.0)

    def _get_current_robot_pose(self) -> Dict[str, float]:
        """Get current robot pose {'x', 'y', 'heading'} and velocity."""
        rx, ry, rheading = 0.0, 0.0, 0.0
        rvel = 0.0

        if self.robot:
            if hasattr(self.robot, "simulated_x"):
                rx = float(self.robot.simulated_x)
            if hasattr(self.robot, "simulated_y"):
                ry = float(self.robot.simulated_y)
            if hasattr(self.robot, "simulated_heading"):
                rheading = float(self.robot.simulated_heading)

            if hasattr(self.robot, "drive_base") and self.robot.drive_base:
                drive_dev = self.robot.drive_base
                if hasattr(drive_dev, "speed"):
                    # Convert km/h to m/s
                    rvel = float(drive_dev.speed) / 3.6

        return {"x": rx, "y": ry, "heading": rheading}, rvel

    def _get_obstacle_points(self) -> List[Tuple[float, float]]:
        """Get current obstacle points from VLP-16 or Ouster LiDAR point cloud."""
        obstacles: List[Tuple[float, float]] = []
        if not self.robot:
            return obstacles

        # Use VLP-16 points if available
        if hasattr(self.robot, "last_vlp16_points") and self.robot.last_vlp16_points is not None:
            pts = self.robot.last_vlp16_points
            if len(pts) > 0:
                # If ground removal column exists
                if pts.shape[1] >= 5:
                    non_ground = pts[pts[:, 4] < 0.5]
                    if len(non_ground) > 0:
                        obstacles.extend([(float(p[0]), float(p[1])) for p in non_ground[::5]])
                else:
                    obstacles.extend([(float(p[0]), float(p[1])) for p in pts[::10]])

        return obstacles

    def _control_loop(self):
        """Periodic control loop running at control_freq (default 10 Hz)."""
        logger.info(f"[{self.name}] Control loop thread active ({self.control_freq} Hz).")

        while self.is_active:
            start_time = time.time()

            if self.is_paused:
                time.sleep(self.control_period)
                continue

            # Check if vehicle is in Auto Mode
            drive_dev = getattr(self.robot, "drive_base", None) if self.robot else None
            is_auto = False
            if drive_dev:
                ad_flag = getattr(drive_dev, "ad_control_req_flag", 0)
                is_auto = (ad_flag == 1)

            if not is_auto:
                # If not in Auto mode, standby without sending drive commands
                time.sleep(self.control_period)
                continue

            # Check goal reach condition
            pose, vel_ms = self._get_current_robot_pose()
            goal = self.global_path[-1]
            dist_to_goal = math.hypot(goal["x"] - pose["x"], goal["y"] - pose["y"])

            if dist_to_goal < self.goal_reach_threshold:
                logger.info(f"[{self.name}] Goal destination reached! Stopping mission.")
                self.is_active = False
                self._apply_stop_command()
                # Set target gear to P
                if drive_dev and hasattr(drive_dev, "target_gear"):
                    drive_dev.target_gear = "P"
                break

            # Compute Local Planner (DWA) controls
            obstacles = self._get_obstacle_points()
            if self.local_planner is not None:
                target_v_ms, target_delta_rad = self.local_planner.compute_velocity_commands(
                    current_pose=pose,
                    current_vel=vel_ms,
                    local_path=self.global_path,
                    obstacle_points=obstacles
                )
            else:
                target_v_ms = 1.0
                target_delta_rad = 0.0

            # Convert velocity from m/s to km/h and steer angle from rad to deg
            target_v_kmh = target_v_ms * 3.6
            target_delta_deg = math.degrees(target_delta_rad)

            # Apply gear D when driving forward
            if drive_dev:
                if target_v_kmh > 0.01:
                    drive_dev.target_gear = "D"
                    drive_dev.gear = "D"

                if hasattr(drive_dev, "set_speed"):
                    drive_dev.set_speed(target_v_kmh)
                if hasattr(drive_dev, "set_steering_angle"):
                    drive_dev.set_steering_angle(target_delta_deg)

            elapsed = time.time() - start_time
            time.sleep(max(0.0, self.control_period - elapsed))

        logger.info(f"[{self.name}] Control loop thread terminated.")
