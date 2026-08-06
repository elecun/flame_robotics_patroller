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

        # Retrieve default corridor_boundary from apros.cfg [mobile_drive_s1] section or device if available
        default_cb = 2.5
        if self.robot:
            if hasattr(self.robot, "config") and self.robot.config and self.robot.config.has_section("mobile_drive_s1"):
                default_cb = float(self.robot.config.get("mobile_drive_s1", "corridor_boundary", fallback=2.5))
            elif hasattr(self.robot, "devices") and "mobile_drive_s1" in self.robot.devices:
                drive_dev = self.robot.devices["mobile_drive_s1"]
                if hasattr(drive_dev, "corridor_boundary"):
                    default_cb = float(drive_dev.corridor_boundary)

        waypoints = []
        corridor_boundaries = []
        try:
            with open(file_path, "r", encoding="utf-8") as f:
                reader = csv.reader(f)
                header = next(reader, None)
                cb_idx = -1
                if header:
                    for i, col in enumerate(header):
                        if col.strip().lower() == "corridor_boundary":
                            cb_idx = i
                            break

                for row in reader:
                    if len(row) >= 3:
                        try:
                            lat = float(row[1])
                            lon = float(row[2])
                            cb_val = float(row[cb_idx]) if cb_idx != -1 and len(row) > cb_idx else default_cb
                            waypoints.append((lat, lon))
                            corridor_boundaries.append(cb_val)
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
        self.origin_lat = lat0
        self.origin_lon = lon0
        points_meter = []
        for i, (lat, lon) in enumerate(waypoints):
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
            # Attach corridor_boundary to global_path points
            for pt in self.global_path:
                # Find nearest raw waypoint corridor boundary
                nearest_cb = 2.5
                min_d = float("inf")
                for raw_pt, cb in zip(points_meter, corridor_boundaries):
                    d = math.hypot(pt["x"] - raw_pt[0], pt["y"] - raw_pt[1])
                    if d < min_d:
                        min_d = d
                        nearest_cb = cb
                pt["corridor_boundary"] = nearest_cb
            logger.info(f"[{self.name}] Global path planned for '{route_file_name}': {len(self.global_path)} path points (Corridor boundary={corridor_boundaries[0]}m).")
        else:
            self.global_path = [
                {"x": pt[0], "y": pt[1], "heading": 0.0, "curvature": 0.0, "v_ref": 1.0, "corridor_boundary": cb}
                for pt, cb in zip(points_meter, corridor_boundaries)
            ]

        # Load POI Tasks relative to route origin (lat0, lon0)
        if poi_file_name:
            self.load_mission_poi(poi_file_name, lat0, lon0)

        return True

    def start_mission(self, route_file_name: Optional[str] = None, poi_file_name: Optional[str] = None) -> bool:
        """
        Start mission route execution thread.
        Triggered when user clicks 'Start Mission' button in Mission Control tab.
        """
        with self._lock:
            if route_file_name:
                success = self.load_mission_route(route_file_name, poi_file_name)
                if not success:
                    return False
            elif not self.global_path:
                logger.warning(f"[{self.name}] Cannot start mission: No route loaded.")
                return False

            if self.is_active:
                logger.info(f"[{self.name}] Mission is already running.")
                return True

            self.is_active = True

            # Explicitly release brake and DBS Valid when starting mission
            if self.robot and hasattr(self.robot, "drive_base") and self.robot.drive_base:
                drive_dev = self.robot.drive_base
                if hasattr(drive_dev, "ad_dbs_valid"):
                    drive_dev.ad_dbs_valid = 0
                if hasattr(drive_dev, "set_brake"):
                    drive_dev.set_brake(0.0)
                if hasattr(drive_dev, "brake_light"):
                    drive_dev.brake_light = False

            if self._thread is None or not self._thread.is_alive():
                self._thread = threading.Thread(target=self._control_loop, daemon=True)
                self._thread.start()
                logger.info(f"[{self.name}] Started autonomous Drive Executor control loop ({self.control_freq} Hz).")

            return True

    def abort_mission(self):
        """
        Abort mission route execution:
        1. Set target input speed to 0.
        2. Wait until vehicle speed decelerates to <= 0.01 km/h (within tolerance).
        3. Set active brake (ad_dbs_valid=1, brake_stop), which automatically turns on brake light (0x506).
        """
        with self._lock:
            self.is_active = False
            logger.info(f"[{self.name}] Mission abort requested.")
            if self.local_planner and hasattr(self.local_planner, "best_local_path"):
                self.local_planner.best_local_path = []

        def _decelerate_and_brake_thread():
            if self.robot and hasattr(self.robot, "drive_base") and self.robot.drive_base:
                drive_dev = self.robot.drive_base
                if hasattr(drive_dev, "set_speed"):
                    drive_dev.set_speed(0.0)
                if hasattr(drive_dev, "set_steering_angle"):
                    drive_dev.set_steering_angle(0.0)

                # Poll current speed until decelerated to <= 0.01 km/h (timeout 3.0s)
                start_t = time.time()
                while time.time() - start_t < 3.0:
                    curr_speed = abs(getattr(drive_dev, "speed", 0.0))
                    if curr_speed <= 0.01:
                        break
                    time.sleep(0.05)

                # Set Brake Stop (ad_dbs_valid=1, brake_val=10) & brake light
                if hasattr(drive_dev, "brake_stop"):
                    drive_dev.brake_stop()
                elif hasattr(drive_dev, "set_brake"):
                    drive_dev.set_brake(10.0)
                logger.info(f"[{self.name}] Vehicle speed decelerated to <= 0.01 km/h. Active Brake & Brake Light set.")

        threading.Thread(target=_decelerate_and_brake_thread, daemon=True).start()

    def _apply_stop_command(self):
        """Send speed=0 and steer=0 command to drive base without changing gear or applying sudden brake."""
        if self.robot and hasattr(self.robot, "drive_base") and self.robot.drive_base:
            drive_dev = self.robot.drive_base
            if hasattr(drive_dev, "set_speed"):
                drive_dev.set_speed(0.0)
            if hasattr(drive_dev, "set_steering_angle"):
                drive_dev.set_steering_angle(0.0)

    def _get_current_robot_pose(self) -> Dict[str, float]:
        """
        Get current robot pose {'x', 'y', 'heading'} and velocity.
        - Converted from SynerexRTK lat/lon relative to route origin (lat0, lon0), or fallback to simulated pose.
        """
        rx, ry, rheading = 0.0, 0.0, 0.0
        rvel = 0.0

        if self.robot:
            # Use SynerexRTK GNSS position converted to relative meters
            rtk_dev = None
            if hasattr(self.robot, "devices") and "synerex_rtk" in self.robot.devices:
                rtk_dev = self.robot.devices["synerex_rtk"]

            if rtk_dev and hasattr(rtk_dev, "latitude") and rtk_dev.latitude is not None and hasattr(self, "origin_lat") and self.origin_lat is not None:
                dlat = rtk_dev.latitude - self.origin_lat
                dlon = rtk_dev.longitude - self.origin_lon
                rx = dlat * 111000.0
                ry = -dlon * 111000.0 * np.cos(np.radians(self.origin_lat))
                rheading = np.radians(getattr(rtk_dev, "heading", 0.0))
                # Also update simulated pose so Viser robot visualization tracks RTK position
                if hasattr(self.robot, "simulated_x"):
                    self.robot.simulated_x = rx
                    self.robot.simulated_y = ry
                    self.robot.simulated_heading = rheading
            else:
                # Fallback to simulated pose
                rx = getattr(self.robot, "simulated_x", 0.0)
                ry = getattr(self.robot, "simulated_y", 0.0)
                rheading = getattr(self.robot, "simulated_heading", 0.0)

            if hasattr(self.robot, "drive_base") and self.robot.drive_base:
                drive_dev = self.robot.drive_base
                if hasattr(drive_dev, "speed"):
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
                        obstacles.extend([(float(p[0]), float(p[1])) for p in non_ground[::10]])
                else:
                    obstacles.extend([(float(p[0]), float(p[1])) for p in pts[::20]])

        return obstacles

    def _get_local_path_window(self, pose: Dict[str, float], global_path: List[Dict[str, float]], lookahead: float = 5.0, max_points: int = 50) -> List[Dict[str, float]]:
        """Extract a local window of the global path around the robot's current position.
        Returns at most max_points path points within lookahead distance ahead of the nearest point."""
        if not global_path:
            return global_path

        rx, ry = pose["x"], pose["y"]

        # Find nearest path point index
        min_dist = float("inf")
        nearest_idx = 0
        for i, pt in enumerate(global_path):
            d = math.hypot(pt["x"] - rx, pt["y"] - ry)
            if d < min_dist:
                min_dist = d
                nearest_idx = i

        # Extract window: from nearest_idx to nearest_idx + max_points (or until lookahead distance exceeded)
        start_idx = max(0, nearest_idx - 2)  # Include 2 points behind for corridor boundary context
        end_idx = min(len(global_path), start_idx + max_points)

        return global_path[start_idx:end_idx]

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

            drive_dev = getattr(self.robot, "drive_base", None) if self.robot else None
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
                # Extract local path window (nearby segment only, not full global path)
                local_window = self._get_local_path_window(pose, self.global_path, lookahead=5.0, max_points=50)
                target_v_ms, target_delta_rad = self.local_planner.compute_velocity_commands(
                    current_pose=pose,
                    current_vel=vel_ms,
                    local_path=local_window,
                    obstacle_points=obstacles
                )
            else:
                target_v_ms = 1.0
                target_delta_rad = 0.0

            target_v_kmh = target_v_ms * 3.6
            target_delta_deg = math.degrees(target_delta_rad)

            logger.debug(f"[{self.name}] Planner command -> Target Speed: {target_v_kmh:.2f} km/h, Steer Angle: {target_delta_deg:.2f}°")

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
            logger.debug(f"[{self.name}][_control_loop] Iteration elapsed: {elapsed * 1000.0:.2f} ms ({elapsed:.4f} s) / Target period: {self.control_period * 1000.0:.2f} ms")
            time.sleep(max(0.0, self.control_period - elapsed))

        logger.info(f"[{self.name}] Control loop thread terminated.")
