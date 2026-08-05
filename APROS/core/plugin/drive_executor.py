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

        # Mission & POI state variables
        self.is_active = False
        self.is_paused = False
        self.hil_simulation = False
        self.current_route_file: Optional[str] = None
        self.current_poi_file: Optional[str] = None
        self.raw_waypoints: List[Tuple[float, float]] = []
        self.global_path: List[Dict[str, float]] = []
        self.poi_tasks: List[Dict[str, Any]] = []  # List of {'x', 'y', 'mast_height_mm', 'executed'}
        self.goal_reach_threshold: float = 0.5  # meters
        self.poi_reach_threshold: float = 0.8   # meters

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

    def load_mission_poi(self, poi_file_name: str, origin_lat: float, origin_lon: float):
        """
        Load .poi CSV file and compute 3D relative meter coordinates.
        """
        if not poi_file_name or poi_file_name == "None":
            self.poi_tasks = []
            return

        route_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "route")
        file_path = os.path.join(route_dir, poi_file_name)
        if not os.path.exists(file_path):
            file_path = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), "route", poi_file_name)

        if not os.path.exists(file_path):
            logger.warning(f"[{self.name}] POI file '{poi_file_name}' not found.")
            self.poi_tasks = []
            return

        tasks = []
        try:
            with open(file_path, "r", encoding="utf-8") as f:
                reader = csv.reader(f)
                header = next(reader, None)
                for row in reader:
                    if len(row) >= 3:
                        try:
                            lat = float(row[1])
                            lon = float(row[2])
                            mast_h = float(row[3]) if len(row) >= 4 else 1800.0
                            dlat = lat - origin_lat
                            dlon = lon - origin_lon
                            dx = dlat * 111000.0
                            dy = -dlon * 111000.0 * np.cos(np.radians(origin_lat))
                            tasks.append({
                                "x": dx,
                                "y": dy,
                                "mast_height_mm": mast_h,
                                "executed": False
                            })
                        except ValueError:
                            continue
            self.poi_tasks = tasks
            self.current_poi_file = poi_file_name
            logger.info(f"[{self.name}] Loaded {len(self.poi_tasks)} POI task points from '{poi_file_name}'.")
        except Exception as e:
            logger.error(f"[{self.name}] Error reading POI file '{poi_file_name}': {e}")
            self.poi_tasks = []

    def load_mission_route(self, route_file_name: str, poi_file_name: Optional[str] = None) -> bool:
        """
        Load .route CSV file and compute global path using GlobalPlanner.
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

        # Run Global Planner to create smooth sub-path
        if self.global_planner is not None:
            self.global_path = self.global_planner.plan(points_meter)
            logger.info(f"[{self.name}] Global path planned for '{route_file_name}': {len(self.global_path)} path points.")
        else:
            self.global_path = [
                {"x": pt[0], "y": pt[1], "heading": 0.0, "curvature": 0.0, "v_ref": 1.0}
                for pt in points_meter
            ]

        # Load POI Tasks relative to route origin (lat0, lon0)
        if poi_file_name:
            self.load_mission_poi(poi_file_name, lat0, lon0)

        return True

    def start_mission(self, route_file_name: Optional[str] = None, poi_file_name: Optional[str] = None, hil_simulation: bool = False) -> bool:
        """
        Start mission route execution thread.
        Triggered when user clicks 'Start Mission' button in Mission Control tab.
        """
        with self._lock:
            self.hil_simulation = hil_simulation
            if route_file_name:
                success = self.load_mission_route(route_file_name, poi_file_name)
                if not success:
                    return False
            elif not self.global_path:
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
                logger.info(f"[{self.name}] Started autonomous Drive Executor control loop ({self.control_freq} Hz, HIL={self.hil_simulation}).")

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
                if pts.shape[1] >= 5:
                    non_ground = pts[pts[:, 4] < 0.5]
                    if len(non_ground) > 0:
                        obstacles.extend([(float(p[0]), float(p[1])) for p in non_ground[::5]])
                else:
                    obstacles.extend([(float(p[0]), float(p[1])) for p in pts[::10]])

        return obstacles

    def _execute_poi_inspection_sequence(self, poi_task: Dict[str, Any]):
        """
        Execute POI Inspection Task Sequence:
        1. Pause robot movement (speed = 0).
        2. Extend mast_joint up to target mast_height_mm.
        3. Retract mast_joint back to 1800 mm.
        4. Resume route tracking to next POI.
        """
        target_height_mm = poi_task["mast_height_mm"]
        logger.info(f"[{self.name}] Arrived at POI task ({poi_task['x']:.2f}m, {poi_task['y']:.2f}m). Pausing drive for mast inspection (target height: {target_height_mm:.1f}mm)...")
        
        self._apply_stop_command()

        mast_dev = None
        if self.robot and hasattr(self.robot, "devices") and "telescopic_mast" in self.robot.devices:
            mast_dev = self.robot.devices["telescopic_mast"]

        # Step 1: Extend Mast to target_height_mm
        if mast_dev and hasattr(mast_dev, "set_height"):
            mast_dev.set_height(target_height_mm)
        
        # Simulate / Wait for mast extension (approx 3 seconds)
        for _ in range(30):
            if not self.is_active:
                return
            time.sleep(0.1)

        # Step 2: Retract Mast back to min height (1800 mm)
        if mast_dev and hasattr(mast_dev, "retract_fully"):
            mast_dev.retract_fully()

        # Simulate / Wait for mast retraction (approx 3 seconds)
        for _ in range(30):
            if not self.is_active:
                return
            time.sleep(0.1)

        poi_task["executed"] = True
        logger.info(f"[{self.name}] POI inspection completed. Resuming mission route navigation.")

    def _control_loop(self):
        """Periodic control loop running at control_freq (default 10 Hz)."""
        logger.info(f"[{self.name}] Control loop thread active ({self.control_freq} Hz).")

        while self.is_active:
            start_time = time.time()

            if self.is_paused:
                time.sleep(self.control_period)
                continue

            # Check if vehicle is in Auto Mode (or HIL simulation mode)
            drive_dev = getattr(self.robot, "drive_base", None) if self.robot else None
            is_auto = False
            if drive_dev:
                ad_flag = getattr(drive_dev, "ad_control_req_flag", 0)
                is_auto = (ad_flag == 1) or self.hil_simulation

            if not is_auto:
                time.sleep(self.control_period)
                continue

            pose, vel_ms = self._get_current_robot_pose()

            # 1. Check POI Arrival
            for poi in self.poi_tasks:
                if not poi["executed"]:
                    dist_to_poi = math.hypot(poi["x"] - pose["x"], poi["y"] - pose["y"])
                    if dist_to_poi <= self.poi_reach_threshold:
                        self._execute_poi_inspection_sequence(poi)
                        break

            if not self.is_active:
                break

            # 2. Check Global Goal Reach Condition
            goal = self.global_path[-1]
            dist_to_goal = math.hypot(goal["x"] - pose["x"], goal["y"] - pose["y"])

            if dist_to_goal < self.goal_reach_threshold:
                logger.info(f"[{self.name}] Goal destination reached! Stopping mission.")
                self.is_active = False
                self._apply_stop_command()
                if drive_dev and hasattr(drive_dev, "target_gear"):
                    drive_dev.target_gear = "P"
                break

            # 3. Compute Local Planner (Ackermann DWA) controls to track route
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

            target_v_kmh = target_v_ms * 3.6
            target_delta_deg = math.degrees(target_delta_rad)

            # 4. Dispatch control command to mobile drive base
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
