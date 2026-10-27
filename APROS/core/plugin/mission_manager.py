"""
Mission Manager Plugin for APROS Autonomous Mobile Robot Platform.
Orchestrates DriveExecutor (autonomous driving) and MastExecutor (telescopic mast extension/retraction),
manages POI (Point of Interest) inspection tasks (0.5m arrival threshold, 3s mast inspection at target height, retract to 2900mm),
and triggers completion notifications along with turn light flash signals (0.5s interval x 3 times).
"""

import os
import time
import math
import threading
import importlib
from typing import Optional, List, Dict, Any, Tuple

from core.plugin.base import BasePlugin
from util.logger.console import ConsoleLogger

logger = ConsoleLogger.get_logger()


class MissionManager(BasePlugin):
    """
    Mission Manager Plugin.
    Manages and orchestrates DriveExecutor (autonomous driving) and MastExecutor (telescopic mast inspection).
    
    Role Separation:
    - HIL Simulation: Independently feeds route waypoints as virtual robot pose data every 500ms.
      Operates completely independently of POI Enable status.
    - POI Enable = True: MissionManager manages BOTH DriveExecutor and MastExecutor (POI arrival 0.5m check,
      extend to target height, hold 3s, retract to 2900mm origin before resuming drive).
    - POI Enable = False: MissionManager manages ONLY DriveExecutor. Performs pure autonomous route tracking
      without stopping or controlling the mast.
    """

    def __init__(
        self,
        name: str = "mission_manager",
        robot: Optional[Any] = None,
        cpa_window_m: float = 2.0,
        cpa_tolerance_m: float = 0.35,
        poi_reach_threshold: Optional[float] = None,  # Kept for backward compatibility
        enable: bool = True
    ):
        super().__init__(name)
        self.enable = enable
        self.robot = robot
        self.cpa_window_m = float(cpa_window_m)
        self.cpa_tolerance_m = float(cpa_tolerance_m)
        self.poi_reach_threshold = float(poi_reach_threshold) if poi_reach_threshold is not None else 0.5
        self.simulated_mast_height: float = 2900.0

        # Mission state variables
        self.is_active = False
        self.mission_status: str = "Idle"  # Idle | Patrolling... | Done. | Aborted
        self.current_route_file: Optional[str] = None
        self.current_poi_file: Optional[str] = None

        self._thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()

    def initialize(self, config: dict) -> bool:
        """Initialize MissionManager plugin from config."""
        if config:
            if "cpa_window_m" in config:
                self.cpa_window_m = float(config["cpa_window_m"])
            if "cpa_tolerance_m" in config:
                self.cpa_tolerance_m = float(config["cpa_tolerance_m"])
            if "poi_reach_threshold" in config:
                self.poi_reach_threshold = float(config["poi_reach_threshold"])
        return True

    def process(self, data: dict) -> dict:
        """Process plugin execution step (returns status)."""
        return self.get_status()

    def _get_drive_executor(self) -> Optional[Any]:
        if self.robot:
            if hasattr(self.robot, "drive_executor") and self.robot.drive_executor:
                return self.robot.drive_executor
            elif hasattr(self.robot, "plugins") and "drive_executor" in self.robot.plugins:
                return self.robot.plugins["drive_executor"]
        return None

    def _get_mast_executor(self) -> Optional[Any]:
        if self.robot:
            if hasattr(self.robot, "mast_executor") and self.robot.mast_executor:
                return self.robot.mast_executor
            elif hasattr(self.robot, "plugins") and "mast_executor" in self.robot.plugins:
                return self.robot.plugins["mast_executor"]
        return None

    def _get_drive_base(self) -> Optional[Any]:
        if self.robot and hasattr(self.robot, "drive_base"):
            return self.robot.drive_base
        return None

    def start_mission(self, route_file_name: Optional[str] = None, poi_file_name: Optional[str] = None) -> bool:
        """
        Start mission execution orchestrated by MissionManager.
        Loads route and POI via DriveExecutor, then launches _mission_worker thread.
        """
        with self._lock:
            drive_exec = self._get_drive_executor()
            if not drive_exec:
                logger.error(f"[{self.name}] Cannot start mission: DriveExecutor plugin unavailable.")
                return False

            if getattr(drive_exec, "is_paused_for_poi", False):
                des = (getattr(self.robot, "devices", {}) or {}).get("des_control")
                try:
                    if not des or not des.is_connected or abs(des.get_mast_height() - 2900.0) > 50.0:
                        raise RuntimeError("Mast must be at 2900 mm ±50 mm before restarting")
                except Exception as exc:
                    self.mission_status = f"Cannot restart: {exc}"
                    logger.error(f"[{self.name}] {self.mission_status}")
                    return False
                drive_exec.is_paused_for_poi = False

            if route_file_name:
                success = drive_exec.load_mission_route(route_file_name, poi_file_name)
                if not success:
                    logger.error(f"[{self.name}] Failed to load route '{route_file_name}'.")
                    return False
            elif not getattr(drive_exec, "global_path", None):
                logger.warning(f"[{self.name}] Cannot start mission: No route loaded in DriveExecutor.")
                return False

            if self.is_active:
                logger.info(f"[{self.name}] Mission is already running.")
                return True

            self.is_active = True
            self._last_path_idx = 0
            self.mission_status = "Patrolling..."
            self.current_route_file = route_file_name or getattr(drive_exec, "current_route_file", None)
            self.current_poi_file = poi_file_name or getattr(drive_exec, "current_poi_file", None)

            # Start DriveExecutor mission control loop
            drive_exec.start_mission(route_file_name=None, poi_file_name=None)

            if self._thread is None or not self._thread.is_alive():
                self._thread = threading.Thread(target=self._mission_worker, daemon=True, name=f"{self.name}_worker")
                self._thread.start()
                logger.info(f"[{self.name}] Started MissionManager orchestration worker.")

            return True

    def abort_mission(self):
        """Abort active mission, stop DriveExecutor and MastExecutor, and stop vehicle."""
        with self._lock:
            self.is_active = False
            self.mission_status = "Aborted"

            drive_exec = self._get_drive_executor()
            if drive_exec and hasattr(drive_exec, "abort_mission"):
                drive_exec.abort_mission()

            mast_exec = self._get_mast_executor()
            if mast_exec and hasattr(mast_exec, "stop_target_control"):
                mast_exec.stop_target_control()

            logger.info(f"[{self.name}] Mission aborted by user.")

    def _trigger_completion_turn_lights_signal(self):
        """
        Trigger 3x dual turn-light completion signal:
        Flashes left_turn_light and right_turn_light simultaneously at 0.5s intervals 3 times.
        """
        drive_dev = self._get_drive_base()
        if not drive_dev or not hasattr(drive_dev, "set_lights"):
            logger.info(f"[{self.name}] Drive base vehicle lighting API unavailable for completion signal.")
            return

        logger.info(f"[{self.name}] Flashing dual turn lights 3 times (0.5s interval) for mission completion signal...")
        for count in range(3):
            if not self._running_in_general():
                break
            # Turn ON both turn lights
            try:
                drive_dev.set_lights(left_turn=True, right_turn=True)
            except Exception as e:
                logger.warning(f"[{self.name}] Turn lights ON error: {e}")
            time.sleep(0.5)

            # Turn OFF both turn lights
            try:
                drive_dev.set_lights(left_turn=False, right_turn=False)
            except Exception as e:
                logger.warning(f"[{self.name}] Turn lights OFF error: {e}")
            time.sleep(0.5)

        logger.info(f"[{self.name}] Dual turn lights completion signal finished.")

    def _running_in_general(self) -> bool:
        return self.is_active

    def _ensure_poi_cpa_initialized(self, poi_tasks: List[Dict[str, Any]], global_path: List[Dict[str, Any]]):
        """Precompute CPA (Closest Point of Approach) along the global path for each POI."""
        if not global_path:
            return

        total_pts = len(global_path)
        for idx, poi in enumerate(poi_tasks):
            if "index" not in poi:
                poi["index"] = idx

            if "cpa_idx" in poi and poi["cpa_idx"] is not None:
                continue

            px = float(poi.get("x", 0.0))
            py = float(poi.get("y", 0.0))

            # Find closest point on global path
            best_idx = 0
            min_d_sq = float("inf")
            for i, pt in enumerate(global_path):
                dx = float(pt["x"]) - px
                dy = float(pt["y"]) - py
                d_sq = dx * dx + dy * dy
                if d_sq < min_d_sq:
                    min_d_sq = d_sq
                    best_idx = i

            cpa_pt = global_path[best_idx]

            # Compute forward tangent vector at CPA path point
            if best_idx < total_pts - 1:
                tx = float(global_path[best_idx + 1]["x"]) - float(cpa_pt["x"])
                ty = float(global_path[best_idx + 1]["y"]) - float(cpa_pt["y"])
            elif best_idx > 0:
                tx = float(cpa_pt["x"]) - float(global_path[best_idx - 1]["x"])
                ty = float(cpa_pt["y"]) - float(global_path[best_idx - 1]["y"])
            else:
                h = float(cpa_pt.get("heading", 0.0))
                tx, ty = math.cos(h), math.sin(h)

            t_len = math.hypot(tx, ty)
            if t_len > 1e-6:
                tx /= t_len
                ty /= t_len
            else:
                tx, ty = 1.0, 0.0

            poi["cpa_idx"] = best_idx
            poi["cpa_x"] = float(cpa_pt["x"])
            poi["cpa_y"] = float(cpa_pt["y"])
            poi["cpa_tx"] = tx
            poi["cpa_ty"] = ty
            poi["cpa_dist"] = math.sqrt(min_d_sq)
            poi["min_dist_seen"] = float("inf")
            poi["entered_cpa_zone"] = False

    def _is_at_cpa(self, poi: Dict[str, Any], rx: float, ry: float, current_path_idx: Optional[int] = None) -> bool:
        """
        Check if current robot position (rx, ry) is at the Closest Point of Approach (CPA) for this POI.
        Uses path projection and distance turning point detection rather than a fixed arbitrary radius.
        """
        cpa_idx = poi.get("cpa_idx")
        # Check path index progress (prevents triggering a POI located at loop finish while still at loop start)
        if current_path_idx is not None and cpa_idx is not None:
            if abs(current_path_idx - cpa_idx) > 35:
                return False

        cpa_x = float(poi.get("cpa_x", 0.0))
        cpa_y = float(poi.get("cpa_y", 0.0))
        cpa_tx = float(poi.get("cpa_tx", 1.0))
        cpa_ty = float(poi.get("cpa_ty", 0.0))
        px = float(poi.get("x", 0.0))
        py = float(poi.get("y", 0.0))

        dist_to_cpa_pt = math.hypot(rx - cpa_x, ry - cpa_y)
        dist_to_poi = math.hypot(rx - px, ry - py)

        # Vector from CPA path point to robot
        dx = rx - cpa_x
        dy = ry - cpa_y
        s_proj = dx * cpa_tx + dy * cpa_ty  # Longitudinal progress along path tangent

        # If outside approach window, not at CPA yet
        if dist_to_cpa_pt > self.cpa_window_m:
            return False

        poi["entered_cpa_zone"] = True
        min_seen = poi.get("min_dist_seen", float("inf"))
        if dist_to_poi < min_seen:
            poi["min_dist_seen"] = dist_to_poi
            min_seen = dist_to_poi

        # Case 1: POI is located at route origin (cpa_idx == 0) and robot is at origin
        if poi.get("cpa_idx", 0) == 0 and dist_to_cpa_pt <= 0.4:
            logger.info(f"[{self.name}] CPA reached for POI #{poi.get('index', 0)} at route origin (dist_to_cpa={dist_to_cpa_pt:.2f}m).")
            return True

        # Case 2: Longitudinal crossing through CPA perpendicular plane (-0.05m <= s_proj <= cpa_tolerance_m)
        if -0.05 <= s_proj <= self.cpa_tolerance_m:
            logger.info(
                f"[{self.name}] CPA reached for POI #{poi.get('index', '?')} via perpendicular plane "
                f"(s_proj={s_proj:.2f}m, dist_to_poi={dist_to_poi:.2f}m, min_path_dist={poi.get('cpa_dist', 0.0):.2f}m)."
            )
            return True

        # Case 3: Distance turning point (distance to POI was decreasing and is now increasing past closest point)
        if dist_to_poi > min_seen + 0.05 and s_proj > 0.0:
            logger.info(
                f"[{self.name}] CPA reached for POI #{poi.get('index', '?')} via turning point "
                f"(curr_d={dist_to_poi:.2f}m > min_d={min_seen:.2f}m, s_proj={s_proj:.2f}m)."
            )
            return True

        return False

    def _execute_real_poi_inspection(self, poi_task: Dict[str, Any]) -> bool:
        """Hold the drive until verified mast and PTU operations are complete."""
        drive = self._get_drive_executor()
        devices = getattr(self.robot, "devices", {}) or {}
        des = devices.get("des_control")
        base = self._get_drive_base()
        success = False
        moving_mast = False
        use_des = bool(des and getattr(des, "is_connected", False))
        try:
            if not drive:
                raise RuntimeError("Drive executor unavailable")
            drive.pause_for_poi()
            stop_requested_at = time.monotonic()
            if getattr(drive, "hil_simulation_enabled", False):
                raise RuntimeError("Hardware POI inspection is disabled in HIL simulation")
            deadline = time.monotonic() + 15.0
            while self.is_active:
                stamp = getattr(base, "last_velocity_feedback_time", None)
                velocity = getattr(base, "parsed_can_status", {}).get("vehicle_velocity_val") if base else None
                if (base and getattr(base, "is_connected", False) and stamp is not None
                        and stamp >= stop_requested_at and time.monotonic() - stamp <= 2.0 and velocity is not None
                        and math.isfinite(float(velocity)) and abs(float(velocity)) <= 0.05):
                    break
                if time.monotonic() >= deadline:
                    raise RuntimeError("Vehicle stop could not be confirmed from fresh speed feedback")
                time.sleep(0.1)
            if not self.is_active:
                return False
            if not use_des:
                raise RuntimeError("DES PTU control unavailable")
            des.acquire_control()

            def height():
                return des.get_mast_height()

            def confirm_stopped_at(target):
                deadline = time.monotonic() + 10.0
                while self.is_active and time.monotonic() < deadline:
                    status = des._require_connected_client().get_status(timeout=5)
                    mast_status = status.get("devices", {}).get("mast", {})
                    motion = mast_status.get("motion", {})
                    current = des._mast_height(status)
                    if (isinstance(motion, dict) and all(motion.get(k) is True for k in ("supported", "valid", "fresh"))
                            and str(motion.get("value", "")).upper() == "STOPPED"):
                        if abs(current - target) <= 50.0:
                            return
                        raise RuntimeError(f"Mast stopped outside target tolerance: {current:.0f} mm")
                    time.sleep(0.1)
                raise RuntimeError("Mast stop could not be confirmed")

            def move_to(target):
                nonlocal moving_mast
                current = height()
                if not math.isfinite(current):
                    raise RuntimeError("Invalid mast height")
                if abs(current - target) <= 50.0:
                    confirm_stopped_at(target)
                    return
                moving_mast = True
                (des.set_mast_extend if target > current else des.set_mast_retract)(target)
                deadline = time.monotonic() + 300.0
                progress = time.monotonic()
                last_height = current
                while self.is_active:
                    current = height()
                    if not math.isfinite(current):
                        raise RuntimeError("Invalid mast height")
                    if abs(current - target) <= 50.0:
                        des.set_mast_stop()
                        moving_mast = False
                        confirm_stopped_at(target)
                        return
                    if abs(current - last_height) >= 2.0:
                        last_height, progress = current, time.monotonic()
                    if time.monotonic() - progress > 8.0 or time.monotonic() >= deadline:
                        raise RuntimeError(f"Mast failed to reach {target:.0f} mm (current {current:.0f} mm)")
                    time.sleep(0.1)
                raise RuntimeError("Mission aborted during mast motion")

            target = float(poi_task.get("mast_height_mm", 2900.0))
            if not math.isfinite(target) or not 2900.0 <= target <= 9000.0:
                raise ValueError("POI mast height must be between 2900 and 9000 mm")
            move_to(target)
            status = des._require_connected_client().get_status(timeout=5)
            measurements = status.get("devices", {}).get("ptu", {}).get("measurements", {})
            def angle(name):
                item = measurements.get(name, {})
                if not isinstance(item, dict) or not all(item.get(key) is True for key in ("supported", "valid", "fresh")):
                    raise RuntimeError(f"Fresh PTU {name} unavailable")
                value = float(item["value"])
                if not math.isfinite(value):
                    raise RuntimeError(f"Invalid PTU {name}")
                return value
            pan, tilt = angle("pan_deg"), angle("tilt_deg")
            for target_pan in (pan + 15.0, pan):
                if not self.is_active:
                    raise RuntimeError("Mission aborted during PTU scan")
                receipt = des.set_ptu_move(target_pan, tilt)
                deadline = time.monotonic() + 1.0
                while self.is_active and time.monotonic() < deadline:
                    time.sleep(min(0.1, max(0.0, deadline - time.monotonic())))
                if not self.is_active:
                    raise RuntimeError("Mission aborted during PTU scan")
                result = des._require_connected_client().get_command_async(receipt.command_id).result(timeout=5)
                if result is None or result.state.value != "completed":
                    raise RuntimeError("PTU did not reach its target within one second")
            move_to(2900.0)
            if not self.is_active:
                raise RuntimeError("Mission aborted before inspection completed")
            poi_task["executed"] = True
            success = True
            logger.info(f"[{self.name}] POI inspection complete; mast returned to 2900 mm ±50 mm.")
            return True
        except Exception as exc:
            logger.error(f"[{self.name}] POI inspection failed; drive remains stopped: {exc}")
            if self.is_active:
                self.mission_status = f"POI inspection failed: {exc}"
                self.is_active = False
                if drive:
                    drive.abort_mission()
            return False
        finally:
            if moving_mast:
                try:
                    if use_des:
                        des.set_mast_stop()
                except Exception as exc:
                    logger.error(f"[{self.name}] Failed to stop mast: {exc}")
            if not success and use_des:
                try:
                    des.set_ptu_stop()
                except Exception as exc:
                    logger.warning(f"[{self.name}] Failed to stop PTU: {exc}")
            if success and drive:
                drive.is_paused_for_poi = False

    def _execute_poi_mast_inspection(self, poi_task: Dict[str, Any]) -> bool:
        """
        Execute POI Mast & PTU Inspection sequence:
        1. Pause vehicle movement (DriveExecutor stop).
        2. Extend or retract mast to target height (poi_task['mast_height_mm']).
        3. Once target height is reached, perform PTU scan:
           - Pan: current -> Left +15 deg -> Right -30 deg (relative -30) -> Left +15 deg -> return to original.
        4. Retract mast back to home position (2900 mm).
        5. Resume drive navigation.
        """
        if not getattr(self.robot, "simulation_mode", False):
            return self._execute_real_poi_inspection(poi_task)

        target_h = float(poi_task.get("mast_height_mm", 2900.0))
        target_h = max(2900.0, min(9000.0, target_h))
        logger.info(f"[{self.name}] Arrived at POI #{poi_task.get('index', '?')} CPA ({poi_task['x']:.2f}m, {poi_task['y']:.2f}m). Pausing drive for mast inspection (target height: {target_h:.0f}mm)...")

        drive_exec = self._get_drive_executor()
        mast_exec = self._get_mast_executor()
        mast_dev = self.robot.devices.get("telescopic_mast") if self.robot and hasattr(self.robot, "devices") and self.robot.devices else None
        des_dev = self.robot.devices.get("des_control") if self.robot and hasattr(self.robot, "devices") and self.robot.devices else None
        is_sim = bool(self.robot and getattr(self.robot, "simulation_mode", False))

        # Helper to read current mast height
        def _get_current_height() -> float:
            if is_sim:
                return float(getattr(self, "simulated_mast_height", 2900.0))
            if des_dev and getattr(des_dev, "is_connected", False):
                try:
                    return float(des_dev.get_mast_height())
                except Exception:
                    pass
            if mast_dev and hasattr(mast_dev, "current_height_mm"):
                return float(mast_dev.current_height_mm)
            return 2900.0

        # Helper to execute PTU inspection sequence (+15 deg left, -30 deg right, +15 deg back)
        def _perform_ptu_inspection_scan():
            logger.info(f"[{self.name}] Starting PTU inspection scan (Left +15° -> Right -30° -> Return to original)...")
            initial_pan = 0.0
            initial_tilt = 0.0
            if des_dev and getattr(des_dev, "is_connected", False):
                try:
                    des_status = des_dev._require_connected_client().get_status(timeout=5)
                    ptu_meas = des_status.get("devices", {}).get("ptu", {}).get("measurements", {})
                    pan_val = ptu_meas.get("pan_deg", {}).get("value")
                    tilt_val = ptu_meas.get("tilt_deg", {}).get("value")
                    if pan_val is not None:
                        initial_pan = float(pan_val)
                    if tilt_val is not None:
                        initial_tilt = float(tilt_val)
                except Exception:
                    pass

            # Step 3-1: Pan Left +15 deg (CCW: +15 deg)
            target_pan_left = initial_pan + 15.0
            if des_dev and getattr(des_dev, "is_connected", False):
                try:
                    des_dev.set_ptu_move(target_pan_left, initial_tilt)
                except Exception as pe:
                    logger.warning(f"[{self.name}] PTU move left error: {pe}")
            for _ in range(3 if is_sim else 20):
                if not self.is_active:
                    return
                time.sleep(0.1)

            # Step 3-2: Pan Right 30 deg (from +15 to -15, relative -30 deg)
            target_pan_right = target_pan_left - 30.0
            if des_dev and getattr(des_dev, "is_connected", False):
                try:
                    des_dev.set_ptu_move(target_pan_right, initial_tilt)
                except Exception as pe:
                    logger.warning(f"[{self.name}] PTU move right error: {pe}")
            for _ in range(5 if is_sim else 30):
                if not self.is_active:
                    return
                time.sleep(0.1)

            # Step 3-3: Pan Left +15 deg back to original position
            if des_dev and getattr(des_dev, "is_connected", False):
                try:
                    des_dev.set_ptu_move(initial_pan, initial_tilt)
                except Exception as pe:
                    logger.warning(f"[{self.name}] PTU return move error: {pe}")
            for _ in range(3 if is_sim else 20):
                if not self.is_active:
                    return
                time.sleep(0.1)

            logger.info(f"[{self.name}] PTU inspection scan completed. Returning mast to home position (2900mm)...")

        # 1. Pause vehicle movement via DriveExecutor and confirm complete vehicle stop
        if drive_exec:
            drive_exec.is_paused_for_poi = True
            if hasattr(drive_exec, "_apply_stop_command"):
                drive_exec._apply_stop_command()

            # Ensure vehicle speed strictly decelerates to 0.0 km/h before extending mast
            drive_dev = getattr(self.robot, "drive_base", None) if self.robot else None
            if drive_dev and not is_sim:
                start_halt_t = time.time()
                while time.time() - start_halt_t < 2.5 and self.is_active:
                    curr_spd = abs(getattr(drive_dev, "speed", 0.0))
                    if curr_spd <= 0.05:
                        break
                    time.sleep(0.05)

        try:
            curr_h = _get_current_height()
            stop_bound = getattr(mast_exec, "stop_trig_bound", 15.0) if mast_exec else 15.0

            # Simulation Mode: Fast and smooth virtual mast & PTU execution (~3s total)
            if is_sim:
                logger.info(f"[{self.name}] [SIMULATION] Extending mast from {curr_h:.0f}mm to target {target_h:.0f}mm...")
                steps = 5
                dh = (target_h - curr_h) / steps
                for _ in range(steps):
                    if not self.is_active:
                        return False
                    curr_h += dh
                    self.simulated_mast_height = curr_h
                    if self.robot:
                        self.robot.simulated_mast_height = curr_h
                    time.sleep(0.2)
                self.simulated_mast_height = target_h
                if self.robot:
                    self.robot.simulated_mast_height = target_h

                _perform_ptu_inspection_scan()

                logger.info(f"[{self.name}] [SIMULATION] Retracting mast back to home position (2900 mm)...")
                dh = (2900.0 - curr_h) / steps
                for _ in range(steps):
                    if not self.is_active:
                        return False
                    curr_h += dh
                    self.simulated_mast_height = curr_h
                    if self.robot:
                        self.robot.simulated_mast_height = curr_h
                    time.sleep(0.2)
                self.simulated_mast_height = 2900.0
                if self.robot:
                    self.robot.simulated_mast_height = 2900.0

                poi_task["executed"] = True
                logger.info(f"[{self.name}] [SIMULATION] POI #{poi_task.get('index', '?')} inspection completed. Resuming drive navigation.")
                return True

            # Real Hardware Mode: Hardware commands and sensor feedback monitoring
            if des_dev and getattr(des_dev, "is_connected", False):
                try:
                    des_dev.acquire_control()
                except Exception as ce:
                    logger.warning(f"[{self.name}] DES acquire control lease check: {ce}")

            if abs(curr_h - target_h) > stop_bound + 10.0:
                if target_h > curr_h:
                    logger.info(f"[{self.name}] Extending mast from {curr_h:.0f}mm to target {target_h:.0f}mm...")
                    if des_dev and getattr(des_dev, "is_connected", False):
                        try:
                            des_dev.set_mast_extend(target_h)
                        except Exception as de_err:
                            logger.warning(f"[{self.name}] DES extend failed, fallback: {de_err}")
                    elif mast_exec and hasattr(mast_exec, "start_target_extend"):
                        mast_exec.start_target_extend(target_h)
                    elif mast_dev and hasattr(mast_dev, "move_up"):
                        mast_dev.move_up()
                else:
                    logger.info(f"[{self.name}] Retracting mast from {curr_h:.0f}mm to target {target_h:.0f}mm...")
                    if des_dev and getattr(des_dev, "is_connected", False):
                        try:
                            des_dev.set_mast_retract(target_h)
                        except Exception as de_err:
                            logger.warning(f"[{self.name}] DES retract failed, fallback: {de_err}")
                    elif mast_exec and hasattr(mast_exec, "start_target_retract"):
                        mast_exec.start_target_retract(target_h)
                    elif mast_dev and hasattr(mast_dev, "move_down"):
                        mast_dev.move_down()

                # Wait for mast to reach target height with progress tracking (up to 120s or 8s without movement)
                start_t = time.time()
                last_h = curr_h
                last_progress_t = time.time()
                while self.is_active:
                    curr_h = _get_current_height()
                    if target_h >= curr_h and curr_h >= target_h - stop_bound - 50.0:
                        break
                    if target_h <= curr_h and curr_h <= target_h + stop_bound + 50.0:
                        break
                    if abs(curr_h - last_h) >= 2.0:
                        last_h = curr_h
                        last_progress_t = time.time()
                    elif time.time() - last_progress_t > 8.0:
                        logger.warning(f"[{self.name}] Mast height unchanged for 8s (curr={curr_h:.0f}mm, target={target_h:.0f}mm). Moving to next step.")
                        break
                    if time.time() - start_t > 120.0:
                        logger.warning(f"[{self.name}] Mast target reach timed out (120s).")
                        break
                    time.sleep(0.1)

            # 3. Perform PTU Inspection Scan at target height
            _perform_ptu_inspection_scan()

            # 4. Retract Mast back to home position (2900 mm)
            curr_h = _get_current_height()
            if curr_h > 2950.0:
                logger.info(f"[{self.name}] Retracting mast back to home position (2900 mm)...")
                if des_dev and getattr(des_dev, "is_connected", False):
                    try:
                        des_dev.set_mast_retract(2900.0)
                    except Exception as de_err:
                        logger.warning(f"[{self.name}] DES home retract failed: {de_err}")
                elif mast_exec and hasattr(mast_exec, "start_target_retract"):
                    mast_exec.start_target_retract(2900.0)
                elif mast_dev and hasattr(mast_dev, "move_down"):
                    mast_dev.move_down()

                # Wait for retraction to strictly reach 2900 mm home with progress tracking
                start_t = time.time()
                last_h = curr_h
                last_progress_t = time.time()
                while self.is_active:
                    curr_h = _get_current_height()
                    if curr_h <= 2900.0 + stop_bound + 50.0:
                        break
                    if abs(curr_h - last_h) >= 2.0:
                        last_h = curr_h
                        last_progress_t = time.time()
                    elif time.time() - last_progress_t > 8.0:
                        logger.warning(f"[{self.name}] Mast home retraction unchanged for 8s. Resuming navigation.")
                        break
                    if time.time() - start_t > 120.0:
                        logger.warning(f"[{self.name}] Mast home retraction timed out (120s).")
                        break
                    time.sleep(0.1)

            poi_task["executed"] = True
            logger.info(f"[{self.name}] POI mast & PTU inspection completed (mast returned to 2900mm home). Resuming drive navigation.")
            return True
        finally:
            # Resume vehicle autonomous driving only after mast safely returned to 2900mm origin
            if drive_exec:
                drive_exec.is_paused_for_poi = False

    def _mission_worker(self):
        """
        Orchestration Worker Loop:
        1. Monitors robot current pose relative to DriveExecutor POI tasks using CPA.
        2. Detects Closest Point of Approach (CPA) for unexecuted POIs.
        3. Invokes mast inspection sequence if POI Enable is checked.
        4. Waits for DriveExecutor completion ("Done.").
        5. Executes vehicle stop (0 km/h), updates status, and triggers 3x turn-light signal.
        """
        logger.info(f"[{self.name}] MissionManager worker thread active (CPA window: {self.cpa_window_m:.1f}m, tol: {self.cpa_tolerance_m:.2f}m).")

        while self.is_active:
            drive_exec = self._get_drive_executor()
            if not drive_exec:
                break

            # 1. Check POI Tasks Arrival via CPA (if poi_enabled in DriveExecutor is True)
            poi_enabled = getattr(drive_exec, "poi_enabled", True)
            poi_tasks = getattr(drive_exec, "poi_tasks", [])
            global_path = getattr(drive_exec, "global_path", [])

            rtk_ok = True
            if not getattr(self.robot, "simulation_mode", False) and not getattr(drive_exec, "hil_simulation_enabled", False):
                devices = getattr(self.robot, "devices", {}) or {}
                rtk = next((devices[name] for name in ("synerex_syn2000", "synerex_syn3100", "synerex_rtk") if name in devices), None)
                rtk_ok = bool(rtk and getattr(rtk, "is_connected", False) and getattr(rtk, "fix_quality", 0)
                              and getattr(rtk, "latitude", None) is not None and getattr(rtk, "longitude", None) is not None)
            if poi_enabled and poi_tasks and global_path and rtk_ok:
                self._ensure_poi_cpa_initialized(poi_tasks, global_path)
                pose, _ = drive_exec._get_current_robot_pose()
                rx, ry = float(pose["x"]), float(pose["y"])

                # Estimate current path index along global_path advancing smoothly
                if not hasattr(self, "_last_path_idx") or self._last_path_idx is None:
                    self._last_path_idx = 0
                total_pts = len(global_path)
                search_start = max(0, self._last_path_idx - 10)
                search_end = min(total_pts, self._last_path_idx + 60)
                best_i = self._last_path_idx
                min_d = float("inf")
                for i in range(search_start, search_end):
                    pt = global_path[i]
                    d = math.hypot(pt["x"] - rx, pt["y"] - ry)
                    if d < min_d:
                        min_d = d
                        best_i = i
                self._last_path_idx = best_i
                current_path_idx = best_i

                for poi in poi_tasks:
                    if not poi.get("executed", False):
                        at_final_goal = (
                            math.hypot(poi["x"] - global_path[-1]["x"], poi["y"] - global_path[-1]["y"]) <= 2.0
                            and math.hypot(rx - global_path[-1]["x"], ry - global_path[-1]["y"])
                            < drive_exec.goal_reach_threshold
                        )
                        if at_final_goal or self._is_at_cpa(poi, rx, ry, current_path_idx=current_path_idx):
                            success = self._execute_poi_mast_inspection(poi)
                            if not success:
                                break

            if not self.is_active:
                break

            # 2. Check DriveExecutor Mission Status
            exec_status = getattr(drive_exec, "mission_status", "Idle")
            if exec_status == "Done." or not drive_exec.is_active:
                logger.info(f"[{self.name}] DriveExecutor finished patrol route (Status: {exec_status}). Finalizing mission...")
                self.is_active = False
                self.mission_status = "Done."
                if hasattr(drive_exec, "mission_status"):
                    drive_exec.mission_status = "Done."

                # Ensure vehicle speed is 0 km/h
                if hasattr(drive_exec, "_apply_stop_command"):
                    drive_exec._apply_stop_command()

                # Trigger 3x dual turn light completion signal (0.5s interval)
                self._trigger_completion_turn_lights_signal()
                break

            time.sleep(0.1)

        logger.info(f"[{self.name}] MissionManager worker thread finished.")

    def get_status(self) -> Dict[str, Any]:
        """Return MissionManager status dictionary."""
        return {
            "name": self.name,
            "is_active": self.is_active,
            "mission_status": self.mission_status,
            "current_route_file": self.current_route_file,
            "current_poi_file": self.current_poi_file,
            "cpa_window_m": self.cpa_window_m,
            "cpa_tolerance_m": self.cpa_tolerance_m,
            "poi_reach_threshold": self.poi_reach_threshold,
        }
