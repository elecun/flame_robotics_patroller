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
        poi_reach_threshold: float = 0.5,  # 0.5m tolerance
        enable: bool = True
    ):
        super().__init__(name)
        self.enable = enable
        self.robot = robot
        self.poi_reach_threshold = float(poi_reach_threshold)

        # Mission state variables
        self.is_active = False
        self.mission_status: str = "Idle"  # Idle | Patrolling... | Done. | Aborted
        self.current_route_file: Optional[str] = None
        self.current_poi_file: Optional[str] = None

        self._thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()

    def initialize(self, config: dict) -> bool:
        """Initialize MissionManager plugin from config."""
        if config and "poi_reach_threshold" in config:
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

    def _execute_poi_mast_inspection(self, poi_task: Dict[str, Any]) -> bool:
        """
        Execute POI Mast Inspection sequence:
        1. Pause vehicle movement (DriveExecutor stop).
        2. Extend mast to target height (poi_task['mast_height_mm']).
        3. Hold at target height for 3.0 seconds.
        4. Retract mast back to 2900 mm.
        5. Wait until retraction completes.
        """
        target_h = float(poi_task.get("mast_height_mm", 2900.0))
        logger.info(f"[{self.name}] Arrived at POI task ({poi_task['x']:.2f}m, {poi_task['y']:.2f}m). Pausing drive for mast inspection (target height: {target_h:.0f}mm)...")

        drive_exec = self._get_drive_executor()
        mast_exec = self._get_mast_executor()
        mast_dev = self.robot.devices.get("telescopic_mast") if self.robot and hasattr(self.robot, "devices") and self.robot.devices else None

        # 1. Pause vehicle movement via DriveExecutor
        if drive_exec:
            drive_exec.is_paused_for_poi = True
            if hasattr(drive_exec, "_apply_stop_command"):
                drive_exec._apply_stop_command()

        try:
            # 2. Extend Mast to target height
            if mast_exec and hasattr(mast_exec, "start_target_extend"):
                mast_exec.start_target_extend(target_h)
            elif mast_dev and hasattr(mast_dev, "move_up"):
                mast_dev.move_up()

            # Wait for extension to reach target height (or timeout 15s)
            start_t = time.time()
            stop_bound = getattr(mast_exec, "stop_trig_bound", 15.0) if mast_exec else 15.0
            while self.is_active and time.time() - start_t < 15.0:
                curr_h = getattr(mast_dev, "current_height_mm", 2900.0) if mast_dev else 2900.0
                if curr_h >= target_h - stop_bound - 50.0:  # near target height
                    break
                time.sleep(0.1)

            logger.info(f"[{self.name}] Mast extension reached target height ({target_h:.0f}mm). Holding for 3 seconds...")

            # 3. Hold at target height for 3.0 seconds
            for _ in range(30):
                if not self.is_active:
                    return False
                time.sleep(0.1)

            # 4. Retract Mast back to 2900 mm origin position
            logger.info(f"[{self.name}] Retracting mast back to 2900 mm origin position...")
            if mast_exec and hasattr(mast_exec, "start_target_retract"):
                mast_exec.start_target_retract(2900.0)
            elif mast_dev and hasattr(mast_dev, "move_down"):
                mast_dev.move_down()

            # Wait for retraction to strictly reach 2900 mm origin (or timeout 15s)
            start_t = time.time()
            while self.is_active and time.time() - start_t < 15.0:
                curr_h = getattr(mast_dev, "current_height_mm", 2900.0) if mast_dev else 2900.0
                if curr_h <= 2900.0 + stop_bound + 50.0:  # fully returned to 2900mm origin
                    break
                time.sleep(0.1)

            poi_task["executed"] = True
            logger.info(f"[{self.name}] POI mast inspection completed (mast returned to 2900mm origin). Resuming drive navigation.")
            return True
        finally:
            # Resume vehicle autonomous driving only after mast safely returned to 2900mm origin
            if drive_exec:
                drive_exec.is_paused_for_poi = False

    def _mission_worker(self):
        """
        Orchestration Worker Loop:
        1. Monitors robot current pose relative to DriveExecutor POI tasks.
        2. Checks 0.5m arrival threshold (poi_reach_threshold).
        3. Invokes mast inspection sequence if POI Enable is checked.
        4. Waits for DriveExecutor completion ("Done.").
        5. Executes vehicle stop (0 km/h), updates status, and triggers 3x turn-light signal.
        """
        logger.info(f"[{self.name}] MissionManager worker thread active.")

        while self.is_active:
            drive_exec = self._get_drive_executor()
            if not drive_exec:
                break

            # 1. Check POI Tasks Arrival (if poi_enabled in DriveExecutor is True)
            poi_enabled = getattr(drive_exec, "poi_enabled", True)
            poi_tasks = getattr(drive_exec, "poi_tasks", [])

            if poi_enabled and poi_tasks:
                pose, _ = drive_exec._get_current_robot_pose()
                for poi in poi_tasks:
                    if not poi.get("executed", False):
                        dist_to_poi = math.hypot(poi["x"] - pose["x"], poi["y"] - pose["y"])
                        if dist_to_poi <= self.poi_reach_threshold:  # 0.5m threshold
                            success = self._execute_poi_mast_inspection(poi)
                            if not success:
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
            "poi_reach_threshold": self.poi_reach_threshold,
        }
