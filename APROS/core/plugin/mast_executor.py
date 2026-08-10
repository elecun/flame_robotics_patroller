"""
Mast Executor Plugin for APROS Autonomous Mobile Robot Platform.
Handles high-level target height control, range validation, automatic stop triggering (with stop_trig_bound offset),
and stuck recovery (re-triggering commands if no height change detected for 1.0s) for TelescopicMast.
"""

import time
import threading
from typing import Optional, Any

from core.plugin.base import BasePlugin
from util.logger.console import ConsoleLogger

logger = ConsoleLogger.get_logger()


class MastExecutor(BasePlugin):
    """
    Mast Executor Plugin.
    Encapsulates high-level target height control logic for TelescopicMast.
    Runs target height tracking in a dedicated background worker thread, while leaving
    single command execution (move_up, move_down, move_stop) to the TelescopicMast device driver.
    """

    MIN_HEIGHT_MM = 2900.0
    MAX_HEIGHT_MM = 9100.0

    def __init__(
        self,
        name: str = "mast_executor",
        robot: Optional[Any] = None,
        stop_trig_bound: float = 15.0,
        min_height: float = 2900.0,
        max_height: float = 9100.0,
        enable: bool = True
    ):
        super().__init__(name)
        self.enable = enable
        self.robot = robot
        self.stop_trig_bound = float(stop_trig_bound)
        self.min_height = float(min_height)
        self.max_height = float(max_height)

        # Target control background thread state
        self._target_control_thread: Optional[threading.Thread] = None
        self._target_control_running: bool = False
        self._target_control_mode: Optional[str] = None  # "extend" or "retract"
        self._target_control_target_mm: float = 2900.0

    def initialize(self, config: dict) -> bool:
        """Initialize MastExecutor plugin with configuration dictionary."""
        if config and "stop_trig_bound" in config:
            self.stop_trig_bound = float(config["stop_trig_bound"])
        return True

    def process(self, data: dict) -> dict:
        """Process plugin execution step (returns current mast status)."""
        mast_dev = self._get_telescopic_mast()
        if mast_dev and hasattr(mast_dev, "get_status"):
            return mast_dev.get_status()
        return {}

    def _get_telescopic_mast() -> Optional[Any]:
        if self.robot and hasattr(self.robot, "devices") and self.robot.devices:
            return self.robot.devices.get("telescopic_mast")
        return None

    def start_target_extend(self, target_height_mm: float) -> bool:
        """
        Start extending mast toward target_height_mm in a dedicated background thread.
        Validates target range [2900, 9100] mm. Automatically stops when current >= target - stop_trig_bound.
        If no height change for 1.0s before target reached, re-triggers move_up command.
        """
        if target_height_mm < self.min_height or target_height_mm > self.max_height:
            logger.warning(f"[{self.name}] Target height {target_height_mm} mm out of range [{self.min_height}, {self.max_height}] mm.")
            return False

        self.stop_target_control()

        self._target_control_mode = "extend"
        self._target_control_target_mm = float(target_height_mm)
        self._target_control_running = True

        self._target_control_thread = threading.Thread(
            target=self._target_control_worker, daemon=True, name=f"{self.name}_target_extend"
        )
        self._target_control_thread.start()
        logger.info(f"[{self.name}] Target EXTEND thread started (target={target_height_mm:.1f} mm, stop_trig_bound={self.stop_trig_bound:.1f} mm).")
        return True

    def start_target_retract(self, target_height_mm: float) -> bool:
        """
        Start retracting mast toward target_height_mm in a dedicated background thread.
        Validates target range [2900, 9100] mm. Automatically stops when current <= target + stop_trig_bound.
        If no height change for 1.0s before target reached, re-triggers move_down command.
        """
        if target_height_mm < self.min_height or target_height_mm > self.max_height:
            logger.warning(f"[{self.name}] Target height {target_height_mm} mm out of range [{self.min_height}, {self.max_height}] mm.")
            return False

        self.stop_target_control()

        self._target_control_mode = "retract"
        self._target_control_target_mm = float(target_height_mm)
        self._target_control_running = True

        self._target_control_thread = threading.Thread(
            target=self._target_control_worker, daemon=True, name=f"{self.name}_target_retract"
        )
        self._target_control_thread.start()
        logger.info(f"[{self.name}] Target RETRACT thread started (target={target_height_mm:.1f} mm, stop_trig_bound={self.stop_trig_bound:.1f} mm).")
        return True

    def stop_target_control(self):
        """Stop background target height control thread and issue move_stop API command to TelescopicMast device."""
        self._target_control_running = False
        if self._target_control_thread and self._target_control_thread.is_alive() and threading.current_thread() != self._target_control_thread:
            self._target_control_thread.join(timeout=1.0)
        self._target_control_thread = None

        mast_dev = self._get_telescopic_mast()
        if mast_dev:
            if hasattr(mast_dev, "move_stop"):
                mast_dev.move_stop()
            elif hasattr(mast_dev, "mast_stop"):
                mast_dev.mast_stop()

    def _get_telescopic_mast(self) -> Optional[Any]:
        if self.robot and hasattr(self.robot, "devices") and self.robot.devices:
            return self.robot.devices.get("telescopic_mast")
        return None

    def _target_control_worker(self):
        """
        Dedicated background worker thread for TelescopicMast target height tracking:
        1. Issues single command API (move_up / move_down) to TelescopicMast device driver.
        2. Monitors current mast height.
        3. Stops automatically when current >= target - stop_trig_bound (extend) or current <= target + stop_trig_bound (retract).
        4. If height doesn't change for 1.0s before target is reached, re-issues single command API once more.
        """
        mode = self._target_control_mode
        target_h = self._target_control_target_mm

        mast_dev = self._get_telescopic_mast()
        if not mast_dev or not mode or target_h is None:
            logger.warning(f"[{self.name}] TelescopicMast device not available for target control worker.")
            return

        # Trigger initial single command API
        if mode == "extend":
            if hasattr(mast_dev, "move_up"):
                mast_dev.move_up()
            elif hasattr(mast_dev, "mast_up"):
                mast_dev.mast_up()
        elif mode == "retract":
            if hasattr(mast_dev, "move_down"):
                mast_dev.move_down()
            elif hasattr(mast_dev, "mast_down"):
                mast_dev.mast_down()

        last_h = getattr(mast_dev, "current_height_mm", 2900.0)
        last_change_time = time.time()

        while self._target_control_running:
            mast_dev = self._get_telescopic_mast()
            if not mast_dev:
                break

            curr_h = getattr(mast_dev, "current_height_mm", 2900.0)

            # Check target stop trigger condition (accounting for deceleration/inertia offset: stop_trig_bound mm)
            extend_stop_h = target_h - self.stop_trig_bound
            retract_stop_h = target_h + self.stop_trig_bound

            if mode == "extend" and curr_h >= extend_stop_h:
                logger.info(f"[{self.name}] Stop trigger boundary reached for EXTEND (curr={curr_h:.1f}mm >= target={target_h:.1f}mm - {self.stop_trig_bound:.1f}mm). Auto-stopping mast.")
                if hasattr(mast_dev, "move_stop"):
                    mast_dev.move_stop()
                elif hasattr(mast_dev, "mast_stop"):
                    mast_dev.mast_stop()
                break
            elif mode == "retract" and curr_h <= retract_stop_h:
                logger.info(f"[{self.name}] Stop trigger boundary reached for RETRACT (curr={curr_h:.1f}mm <= target={target_h:.1f}mm + {self.stop_trig_bound:.1f}mm). Auto-stopping mast.")
                if hasattr(mast_dev, "move_stop"):
                    mast_dev.move_stop()
                elif hasattr(mast_dev, "mast_stop"):
                    mast_dev.mast_stop()
                break

            # Height change tracking (threshold 1.0 mm)
            if abs(curr_h - last_h) >= 1.0:
                last_h = curr_h
                last_change_time = time.time()
            else:
                # No height change detected — check if 1.0 second elapsed
                if time.time() - last_change_time >= 1.0:
                    logger.info(f"[{self.name}] No height change detected for 1.0s (curr={curr_h:.1f}mm, target={target_h:.1f}mm). Re-triggering {mode} command.")
                    if mode == "extend":
                        if hasattr(mast_dev, "move_up"):
                            mast_dev.move_up()
                        elif hasattr(mast_dev, "mast_up"):
                            mast_dev.mast_up()
                    elif mode == "retract":
                        if hasattr(mast_dev, "move_down"):
                            mast_dev.move_down()
                        elif hasattr(mast_dev, "mast_down"):
                            mast_dev.mast_down()
                    last_change_time = time.time()
                    last_h = curr_h

            time.sleep(0.05)

        self._target_control_running = False
