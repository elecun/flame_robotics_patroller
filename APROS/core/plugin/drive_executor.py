"""
Drive Executor Plugin for APROS Autonomous Mobile Robot Platform.
Integrates global path planning, the configured local planner, and vehicle control.
Runs at control_freq (default 10 Hz).
"""
import os
import importlib
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
    Manages autonomous mission route loading, global path generation, real-time local path planning,
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
        self.mission_status: str = "Idle"  # Idle | Patrolling... | Done. | Aborted Mission
        self.current_route_file: Optional[str] = None
        self.current_poi_file: Optional[str] = None
        self.raw_waypoints: List[Tuple[float, float]] = []
        self.raw_wgs84_waypoints: List[Tuple[float, float]] = []
        self.sampled_wgs84_waypoints: List[Tuple[float, float]] = []
        self.global_path: List[Dict[str, float]] = []
        self.poi_tasks: List[Dict[str, Any]] = []  # List of {'x', 'y', 'mast_height_mm', 'executed'}
        self.poi_enabled: bool = True           # Toggle POI inspection & mast control
        self.is_paused_for_poi: bool = False
        self.hil_simulation_enabled: bool = False  # Toggle HIL Simulation mode (1s step route pose update)
        self._hil_step_index: int = 0
        self._hil_last_update_time: float = 0.0
        self.goal_reach_threshold: float = 0.5  # meters
        self.poi_reach_threshold: float = 0.8   # meters

        # Motion-derived heading state (disables reliance on RTK heading)
        self._prev_pose_x: Optional[float] = None
        self._prev_pose_y: Optional[float] = None
        self._current_motion_heading: float = 0.0

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
                            mast_h = float(row[3]) if len(row) >= 4 else 2900.0
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

        # Store original raw route WGS84 waypoints (before 1m sampling) for HIL simulation
        self.raw_wgs84_waypoints = list(waypoints)

        # Sample route waypoints by 1m distance intervals for navigation global path
        try:
            mod_ru = importlib.import_module("APROS.util.route_utils" if __name__.startswith("APROS") else "util.route_utils")
            sample_route_by_distance = getattr(mod_ru, "sample_route_by_distance")
            waypoints, corridor_boundaries = sample_route_by_distance(waypoints, corridor_boundaries, sample_step_m=1.0)
            logger.info(f"[{self.name}] Sampled route '{route_file_name}' to 1m distance intervals ({len(waypoints)} waypoints).")
        except Exception as e:
            logger.error(f"[{self.name}] Error sampling route by 1m: {e}")

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
        self.sampled_wgs84_waypoints = waypoints
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

        # In --simulation mode, place simulated robot at the route origin with initial path heading
        if self.robot and getattr(self.robot, "simulation_mode", False) and self.global_path:
            init_h = float(self.global_path[0].get("heading", 0.0))
            if len(self.global_path) >= 2:
                dx0 = self.global_path[1]["x"] - self.global_path[0]["x"]
                dy0 = self.global_path[1]["y"] - self.global_path[0]["y"]
                if math.hypot(dx0, dy0) > 0.001:
                    init_h = math.atan2(dy0, dx0)
            self.robot.simulated_x = float(self.global_path[0]["x"])
            self.robot.simulated_y = float(self.global_path[0]["y"])
            self.robot.simulated_heading = init_h
            self._current_motion_heading = init_h
            if hasattr(self.robot, "drive_base") and self.robot.drive_base:
                self.robot.drive_base.simulated_heading = init_h
                self.robot.drive_base.lat = lat0
                self.robot.drive_base.lon = lon0
            if self.local_planner and hasattr(self.local_planner, "reset_state"):
                self.local_planner.reset_state()

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
            self.mission_status = "Patrolling..."

            # In --simulation mode, if robot is far from route or at end of previous run, reset to route start
            if self.robot and getattr(self.robot, "simulation_mode", False) and self.global_path:
                sim_x = float(getattr(self.robot, "simulated_x", 0.0))
                sim_y = float(getattr(self.robot, "simulated_y", 0.0))
                goal_pt = self.global_path[-1]
                dist_to_start = math.hypot(self.global_path[0]["x"] - sim_x, self.global_path[0]["y"] - sim_y)
                dist_to_goal = math.hypot(goal_pt["x"] - sim_x, goal_pt["y"] - sim_y)
                if dist_to_goal <= self.goal_reach_threshold * 2.0 or dist_to_start > 5.0:
                    init_h = float(self.global_path[0].get("heading", 0.0))
                    if len(self.global_path) >= 2:
                        dx0 = self.global_path[1]["x"] - self.global_path[0]["x"]
                        dy0 = self.global_path[1]["y"] - self.global_path[0]["y"]
                        if math.hypot(dx0, dy0) > 0.001:
                            init_h = math.atan2(dy0, dx0)
                    self.robot.simulated_x = float(self.global_path[0]["x"])
                    self.robot.simulated_y = float(self.global_path[0]["y"])
                    self.robot.simulated_heading = init_h
                    if hasattr(self.robot, "drive_base") and self.robot.drive_base:
                        self.robot.drive_base.simulated_heading = init_h
                        if hasattr(self, "origin_lat") and self.origin_lat is not None:
                            self.robot.drive_base.lat = self.origin_lat
                            self.robot.drive_base.lon = self.origin_lon
                if self.local_planner and hasattr(self.local_planner, "reset_state"):
                    self.local_planner.reset_state()

            # Trim global_path from current robot position to final goal
            pose, _ = self._get_current_robot_pose()
            if self.global_path:
                nearest_idx = 0
                min_dist = float("inf")
                for i, pt in enumerate(self.global_path):
                    d = math.hypot(pt["x"] - pose["x"], pt["y"] - pose["y"])
                    if d < min_dist:
                        min_dist = d
                        nearest_idx = i

                if nearest_idx > 0:
                    logger.info(f"[{self.name}] Trimming global path starting from nearest index {nearest_idx}/{len(self.global_path)} (dist: {min_dist:.2f}m) to final goal.")
                    self.global_path = self.global_path[nearest_idx:]
                    if self.sampled_wgs84_waypoints and nearest_idx < len(self.sampled_wgs84_waypoints):
                        self.sampled_wgs84_waypoints = self.sampled_wgs84_waypoints[nearest_idx:]
                    if self.raw_wgs84_waypoints and nearest_idx < len(self.raw_wgs84_waypoints):
                        self.raw_wgs84_waypoints = self.raw_wgs84_waypoints[nearest_idx:]

            self._hil_step_index = 0
            self._hil_last_update_time = time.time()

            # Initialize motion heading based on initial global path direction
            if self.global_path:
                if len(self.global_path) >= 2:
                    dx0 = self.global_path[1]["x"] - self.global_path[0]["x"]
                    dy0 = self.global_path[1]["y"] - self.global_path[0]["y"]
                    if math.hypot(dx0, dy0) > 0.001:
                        self._current_motion_heading = math.atan2(dy0, dx0)
                elif "heading" in self.global_path[0]:
                    self._current_motion_heading = self.global_path[0]["heading"]

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
            self.mission_status = "Aborted Mission"
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

        # HIL Simulation Mode: stepped 500ms (0.5s) update along raw route's WGS84 waypoints converted to TM space
        hil_wgs84 = self.raw_wgs84_waypoints or self.sampled_wgs84_waypoints
        if self.hil_simulation_enabled and (hil_wgs84 or self.global_path):
            now = time.time()
            total_steps = len(hil_wgs84) if hil_wgs84 else len(self.global_path)

            if now - self._hil_last_update_time >= 0.5:
                self._hil_last_update_time = now
                if self._hil_step_index < total_steps - 1:
                    self._hil_step_index += 1

            if hil_wgs84 and hasattr(self, "origin_lat") and self.origin_lat is not None:
                curr_lat, curr_lon = hil_wgs84[self._hil_step_index]
                dlat = curr_lat - self.origin_lat
                dlon = curr_lon - self.origin_lon
                rx = dlat * 111000.0
                ry = -dlon * 111000.0 * np.cos(np.radians(self.origin_lat))

                # Compute heading (True North=0 deg, East=90 deg)
                if self._hil_step_index < len(hil_wgs84) - 1:
                    next_lat, next_lon = hil_wgs84[self._hil_step_index + 1]
                    d_n = (next_lat - curr_lat) * 111000.0
                    d_e = (next_lon - curr_lon) * 111000.0 * np.cos(np.radians(curr_lat))
                    hdg_deg = math.degrees(math.atan2(d_e, d_n)) % 360.0
                else:
                    hdg_deg = 0.0

                raw_hdg = hdg_deg % 360.0
                if raw_hdg > 180.0:
                    raw_hdg -= 360.0
                rheading = np.radians(raw_hdg)
                rvel = 1.0

                # Synchronize synerex_rtk device position so Viser & Map UI track the HIL WGS84 position
                rtk_dev = next((self.robot.devices[name] for name in ("synerex_syn2000", "synerex_syn3100", "synerex_rtk")
                                if self.robot and hasattr(self.robot, "devices") and name in self.robot.devices), None)
                if rtk_dev:
                    if hasattr(rtk_dev, "update_pose"):
                        rtk_dev.update_pose(curr_lat, curr_lon, 45.0, hdg_deg)
            else:
                hil_pt = self.global_path[self._hil_step_index]
                rx = hil_pt["x"]
                ry = hil_pt["y"]
                rheading = hil_pt["heading"]
                rvel = hil_pt.get("v_ref", 1.0)

            if self.robot:
                if hasattr(self.robot, "simulated_x"):
                    self.robot.simulated_x = rx
                    self.robot.simulated_y = ry
                    self.robot.simulated_heading = rheading

            return {"x": rx, "y": ry, "heading": rheading}, rvel

        if self.robot:
            # Use SynerexRTK GNSS position converted to relative meters
            rtk_dev = None
            rtk_dev = next((self.robot.devices[name] for name in ("synerex_syn2000", "synerex_syn3100", "synerex_rtk")
                            if hasattr(self.robot, "devices") and name in self.robot.devices), None)

            if rtk_dev and hasattr(rtk_dev, "latitude") and rtk_dev.latitude is not None and hasattr(self, "origin_lat") and self.origin_lat is not None:
                dlat = rtk_dev.latitude - self.origin_lat
                dlon = rtk_dev.longitude - self.origin_lon
                rx = dlat * 111000.0
                ry = -dlon * 111000.0 * np.cos(np.radians(self.origin_lat))

                # Use dual-antenna RTK True Heading (from $GPTHS / $GNTHS: 0 deg = North/+X, 90 deg = East/-Y)
                # Convert clockwise compass heading (deg) to counter-clockwise planner yaw (rad): yaw = -radians(heading_deg)
                rtk_hdg_deg = getattr(rtk_dev, "heading", None)
                if rtk_hdg_deg is not None:
                    rheading = math.atan2(math.sin(-math.radians(float(rtk_hdg_deg))), math.cos(-math.radians(float(rtk_hdg_deg))))
                    self._current_motion_heading = rheading
                else:
                    # Fallback to motion displacement delta if THS heading is unavailable
                    if self._prev_pose_x is not None and self._prev_pose_y is not None:
                        dx_m = rx - self._prev_pose_x
                        dy_m = ry - self._prev_pose_y
                        if math.hypot(dx_m, dy_m) > 0.02:
                            cand_hdg = math.atan2(dy_m, dx_m)
                            hdg_diff = math.atan2(math.sin(cand_hdg - self._current_motion_heading), math.cos(cand_hdg - self._current_motion_heading))
                            if abs(hdg_diff) <= math.pi / 2.0:
                                self._current_motion_heading = cand_hdg
                    rheading = self._current_motion_heading
                self._prev_pose_x = rx
                self._prev_pose_y = ry

                # Update simulated pose so Viser robot visualization tracks position
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
        """Get current obstacle points from the VLP-16 point cloud."""
        obstacles: List[Tuple[float, float]] = []

        # When HIL Simulation mode is enabled, ignore VLP-16 LiDAR obstacles
        if self.hil_simulation_enabled:
            return obstacles

        if not self.robot:
            return obstacles

        # Use height-classified VLP-16 points, transformed into the robot frame.
        if hasattr(self.robot, "last_vlp16_points") and self.robot.last_vlp16_points is not None:
            pts = self.robot.last_vlp16_points
            vlp = getattr(self.robot, "devices", {}).get("vlp-16")
            if vlp is not None and len(pts) > 0 and pts.shape[1] >= 5:
                non_ground = pts[pts[:, 4] < 0.5]
                if len(non_ground) > 0:
                    robot_points = vlp.transform_points_to_robot_frame(non_ground[::10])
                    obstacles.extend((float(p[0]), float(p[1])) for p in robot_points)

        return obstacles

    def _get_local_path_window(self, pose: Dict[str, float], global_path: List[Dict[str, float]], lookahead: float = 5.0, max_points: int = 50) -> List[Dict[str, float]]:
        """Return up to max_points around the closest route point.

        The lookahead argument is currently unused; MPPI applies it when choosing
        its target and trimming the path exposed for visualization.
        """
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

        # Include two points behind the nearest point for corridor context.
        start_idx = max(0, nearest_idx - 2)
        end_idx = min(len(global_path), start_idx + max_points)

        return global_path[start_idx:end_idx]

    def _execute_poi_inspection_sequence(self, poi_task: Dict[str, Any]):
        """
        Execute POI Inspection Task Sequence:
        1. Pause robot movement (speed = 0).
        2. Extend mast_joint up to target mast_height_mm.
        3. Retract mast_joint back to 2900 mm.
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

        # Step 2: Retract Mast back to min height (2900 mm)
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

            if not self.is_active:
                break

            vlp = getattr(self.robot, "devices", {}).get("vlp-16") if self.robot else None
            if vlp is not None and getattr(vlp, "enable", False):
                packet_time = getattr(vlp, "last_packet_time", None)
                scan_time = getattr(vlp, "last_scan_time", None)
                if (packet_time is None or scan_time is None
                        or time.time() - min(packet_time, scan_time) > 2.0):
                    self._apply_stop_command()
                    time.sleep(self.control_period)
                    continue

            # If MissionManager is performing POI mast inspection, hold vehicle stopped
            if self.is_paused_for_poi:
                self._apply_stop_command()
                time.sleep(0.1)
                continue

            # 2. Check Global Goal Reach Condition
            goal = self.global_path[-1]
            dist_to_goal = math.hypot(goal["x"] - pose["x"], goal["y"] - pose["y"])

            if dist_to_goal < self.goal_reach_threshold:
                logger.info(f"[{self.name}] Goal destination reached! Stopping mission.")
                self.is_active = False
                self.mission_status = "Done."
                self._apply_stop_command()
                break

            # 3. Compute controls with the configured local planner to track the route.
            obstacles = self._get_obstacle_points()
            if self.local_planner is not None:
                # Anchor the first predicted steering step to the last issued CAN / simulated command.
                cmd_steer_deg = float(getattr(drive_dev, "cmd_steering_angle", getattr(drive_dev, "steer_angle", 0.0)))
                pose["steering_deg"] = cmd_steer_deg
                pose["steering_rad"] = math.radians(cmd_steer_deg)
                pose["control_dt"] = self.control_period
                # The current window is capped by point count; MPPI applies lookahead later.
                lookahead_dist = getattr(self.local_planner.config, "lookahead_distance", 3.0) if hasattr(self.local_planner, "config") else 3.0
                local_window = self._get_local_path_window(pose, self.global_path, lookahead=lookahead_dist, max_points=200)
                target_v_ms, target_delta = self.local_planner.compute_velocity_commands(
                    current_pose=pose,
                    current_vel=vel_ms,
                    local_path=local_window,
                    obstacle_points=obstacles
                )
            else:
                target_v_ms = 0.0
                target_delta = 0.0

            target_v_kmh = max(0.0, target_v_ms * 3.6)
            steering_unit = getattr(self.local_planner, "steering_output_unit", "rad")
            target_delta_deg = target_delta if steering_unit == "deg" else math.degrees(target_delta)

            # Steer angle polarity (ISO 8855 / ROS convention):
            # + is Counter-Clockwise / Left turn, - is Clockwise / Right turn
            cmd_delta_deg = target_delta_deg

            logger.debug(f"[{self.name}] Planner command -> Target Speed: {target_v_kmh:.2f} km/h, Steer Angle: {cmd_delta_deg:.2f}°")

            # 4. Dispatch control command to mobile drive base (Forward-only D gear)
            if drive_dev:
                drive_dev.target_gear = "D"
                drive_dev.gear = "D"

                if hasattr(drive_dev, "set_speed"):
                    drive_dev.set_speed(target_v_kmh)
                if hasattr(drive_dev, "set_steering_angle"):
                    drive_dev.set_steering_angle(cmd_delta_deg)

            elapsed = time.time() - start_time
            logger.debug(f"[{self.name}][_control_loop] Iteration elapsed: {elapsed * 1000.0:.2f} ms ({elapsed:.4f} s) / Target period: {self.control_period * 1000.0:.2f} ms")
            time.sleep(max(0.0, self.control_period - elapsed))

        logger.info(f"[{self.name}] Control loop thread terminated.")
