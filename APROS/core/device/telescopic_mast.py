"""
Telescopic Mast Hardware Driver Device (telescopic_mast.py)
Provides interface and control for a linear extensible telescopic mast structure.
Diameter: 100 mm (0.1m, fixed constant)
Extension Range: Min 1800 mm (1.8m) to Max 8000 mm (8.0m)
"""

import time
import threading
from typing import Dict, Any, Optional
from core.device.base import BaseDevice
from util.logger.console import ConsoleLogger

logger = ConsoleLogger.get_logger()


class TelescopicMast(BaseDevice):
    """
    Telescopic Mast device driver.
    Controls height extension of the 100mm diameter mast from 1800mm to 8000mm.
    """

    MIN_HEIGHT_MM = 1800.0  # 1.8m (Default retracted/collapsed height)
    MAX_HEIGHT_MM = 8000.0  # 8.0m (Maximum extended height)
    DIAMETER_MM = 100.0     # 100mm fixed diameter (0.1m)

    def __init__(
        self,
        name: str = "telescopic_mast",
        robot_model: str = "iae_patrol_v1",
        min_height: float = 1800.0,
        max_height: float = 8000.0,
        initial_height: float = 1800.0,
        offset_x: float = 0.0,
        offset_y: float = 0.0,
        offset_z: float = 0.64
    ):
        super().__init__(name)
        self.robot_model = robot_model
        self.min_height = float(min_height)
        self.max_height = float(max_height)

        # Installation offset relative to robot frame (meters)
        self.offset_x = float(offset_x)
        self.offset_y = float(offset_y)
        self.offset_z = float(offset_z)

        # Target and Current Height in millimeters (mm)
        initial_target = max(self.min_height, min(self.max_height, float(initial_height)))
        self._target_height_mm = initial_target
        self._current_height_mm = initial_target

        self._lock = threading.Lock()
        self._running = False
        self._thread: Optional[threading.Thread] = None

    @property
    def current_height_mm(self) -> float:
        with self._lock:
            return self._current_height_mm

    @property
    def current_height_m(self) -> float:
        return self.current_height_mm / 1000.0

    @property
    def target_height_mm(self) -> float:
        with self._lock:
            return self._target_height_mm

    @target_height_mm.setter
    def target_height_mm(self, height_mm: float):
        clamped_height = max(self.min_height, min(self.max_height, float(height_mm)))
        with self._lock:
            self._target_height_mm = clamped_height
        logger.info(f"[{self.name}] Target mast height set to {clamped_height:.1f} mm")

    def set_height(self, height_mm: float):
        """Set target extension height in millimeters (clamped between min and max height)."""
        self.target_height_mm = height_mm

    def extend_fully(self):
        """Command mast to extend to maximum height (8000 mm)."""
        self.set_height(self.max_height)

    def retract_fully(self):
        """Command mast to retract to minimum height (1800 mm)."""
        self.set_height(self.min_height)

    def connect(self) -> bool:
        """Connect device interface and start smooth motion update thread."""
        self.is_connected = True
        self._running = True
        self._thread = threading.Thread(target=self._motion_loop, daemon=True)
        self._thread.start()
        logger.info(f"[{self.name}] Connected. Initial height: {self.current_height_mm:.1f} mm (Min: {self.min_height}mm, Max: {self.max_height}mm)")
        return True

    def disconnect(self) -> bool:
        """Disconnect device interface and stop thread."""
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)
            self._thread = None
        self.is_connected = False
        logger.info(f"[{self.name}] Disconnected.")
        return True

    def _motion_loop(self):
        """Simulate physical mast movement transition towards target height."""
        speed_mm_per_sec = 200.0  # Smooth motion extension speed 200mm/s
        dt = 0.05
        while self._running:
            with self._lock:
                diff = self._target_height_mm - self._current_height_mm
                if abs(diff) > 0.01:
                    step = np_sign = (1.0 if diff > 0 else -1.0) * min(abs(diff), speed_mm_per_sec * dt)
                    self._current_height_mm += step
            time.sleep(dt)

    def get_status(self) -> Dict[str, Any]:
        with self._lock:
            return {
                "name": self.name,
                "connected": self.is_connected,
                "current_height_mm": round(self._current_height_mm, 1),
                "target_height_mm": round(self._target_height_mm, 1),
                "current_height_m": round(self._current_height_mm / 1000.0, 3),
                "min_height_mm": self.min_height,
                "max_height_mm": self.max_height,
                "diameter_mm": self.DIAMETER_MM
            }
