"""DES service control through the bundled Patrol Robot ZMQ client.

Connecting acquires a lease but never changes mode or starts motion. Motion
methods return command receipts; an accepted receipt is not completion.
"""

from __future__ import annotations

import math
import threading
import time
from typing import Any

from core.device.base import BaseDevice
from core.device.des_api.patrol_robot_client import PatrolRobotClient
from core.device.des_api.patrol_robot_common import ControlMode
from util.logger.console import ConsoleLogger

logger = ConsoleLogger.get_logger()


class DesControl(BaseDevice):
    MIN_MAST_MM = 2900.0
    MAX_MAST_MM = 9000.0

    def __init__(self, name="des_control", end_point="tcp://127.0.0.1:5590",
                 enable=True, **kwargs):
        super().__init__(name, enable=enable)
        if not end_point.startswith("tcp://"):
            raise ValueError("DES end_point must use tcp://")
        self.end_point = end_point
        self._client: PatrolRobotClient | None = None
        self._lock = threading.RLock()
        self._mast_cancel = threading.Event()
        self._mast_thread: threading.Thread | None = None

    def connect(self) -> bool:
        if not self.enable:
            return False
        with self._lock:
            if self.is_connected:
                return True
            try:
                client = PatrolRobotClient(self.end_point)
                self._client = client
                self.is_connected = True
            except Exception as exc:
                logger.error(f"[DES] Connection failed at {self.end_point}: {exc}")
                if 'client' in locals():
                    try:
                        client.close()
                    except Exception:
                        pass
                return False
            try:
                control = client.acquire_control(timeout=5)
                if control.get("granted"):
                    logger.info(f"[DES] Connected to {self.end_point}; control acquired")
                else:
                    logger.warning(f"[DES] Connected, but control lease denied: {control.get('reason')}")
            except Exception as exc:
                logger.warning(f"[DES] Connected, but control lease request failed: {exc}")
                self.is_connected = False
                self._client = None
                try:
                    client.close()
                except Exception:
                    pass
                return False
            return True

    def disconnect(self) -> bool:
        if self._cancel_mast_target() and self._client is not None:
            try:
                self._receipt(self._client.mast.stop(timeout=5), "mast.stop")
            except Exception as exc:
                logger.error(f"[DES] Mast stop during disconnect failed: {exc}")
        with self._lock:
            client, self._client = self._client, None
            self.is_connected = False
        if client is None:
            return True
        try:
            client.close()
            return True
        except Exception as exc:
            logger.error(f"[DES] Disconnect failed: {exc}")
            return False

    def _require_connected_client(self) -> PatrolRobotClient:
        with self._lock:
            client = self._client
            if not self.is_connected or client is None:
                raise RuntimeError("DES is disconnected")
            return client

    def _require_client(self) -> PatrolRobotClient:
        client = self._require_connected_client()
        if not client.has_control():
            raise RuntimeError("DES control lease is unavailable")
        return client

    def acquire_control(self) -> bool:
        client = self._require_connected_client()
        if client.has_control():
            return True
        result = client.acquire_control(timeout=5)
        if not result.get("granted"):
            raise RuntimeError(f"DES control lease denied: {result.get('reason')}")
        return True

    @staticmethod
    def _receipt(receipt: Any, action: str) -> Any:
        if not receipt.accepted:
            raise RuntimeError(f"DES {action} rejected: {receipt.reason}: {receipt.message}")
        logger.info(f"[DES] {action} accepted: {receipt.command_id}")
        return receipt

    def set_mode_program(self):
        self.acquire_control()
        return self._require_client().set_control_mode(ControlMode.PROGRAM, timeout=5)

    def set_mode_manual(self):
        if self._cancel_mast_target():
            self._receipt(self._require_connected_client().mast.stop(timeout=5), "mast.stop")
        self.acquire_control()
        return self._require_client().set_control_mode(ControlMode.MANUAL, timeout=5)

    def set_ptu_move(self, pan_deg: float, tilt_deg: float):
        pan, tilt = float(pan_deg), float(tilt_deg)
        if not math.isfinite(pan) or not math.isfinite(tilt):
            raise ValueError("PTU angles must be finite")
        return self._receipt(self._require_client().ptu.move_to_angle(pan, tilt, timeout=5), "ptu.move_to")

    def set_ptu_vel(self, speed_code: int):
        if isinstance(speed_code, bool) or not isinstance(speed_code, int) or not 0 <= speed_code <= 0x3F:
            raise ValueError("PTU speed_code must be an integer from 0x00 to 0x3F")
        return self._receipt(self._require_client().ptu.set_speed(speed_code, timeout=5), "ptu.set_speed")

    def set_ptu_stop(self):
        return self._receipt(self._require_connected_client().ptu.stop(timeout=5), "ptu.stop")

    @staticmethod
    def _mast_height(status: dict) -> float:
        obs = status.get("devices", {}).get("mast", {}).get("measurements", {}).get("height_mm", {})
        if not isinstance(obs, dict) or not all(obs.get(k) is True for k in ("supported", "valid", "fresh")):
            raise RuntimeError("Fresh, valid mast height is unavailable")
        value = obs.get("value")
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise RuntimeError("Mast height is invalid")
        return float(value)

    def _cancel_mast_target(self) -> bool:
        active = self._mast_thread is not None and self._mast_thread.is_alive()
        self._mast_cancel.set()
        thread = self._mast_thread
        if thread and thread.is_alive() and thread is not threading.current_thread():
            thread.join(timeout=6)
        self._mast_thread = None
        return active

    def _mast_motion(self, direction: str, target_height_mm: float | None):
        client = self._require_client()
        if self._cancel_mast_target():
            self._receipt(client.mast.stop(timeout=5), "mast.stop")
        if target_height_mm is not None:
            target = float(target_height_mm)
            if not math.isfinite(target) or not self.MIN_MAST_MM <= target <= self.MAX_MAST_MM:
                raise ValueError("Mast target must be 2900–9000 mm")
            current = self._mast_height(client.get_status(timeout=5))
            if direction == "extend" and target <= current:
                raise ValueError("Extend target must exceed current height")
            if direction == "retract" and target >= current:
                raise ValueError("Retract target must be below current height")
        receipt = self._receipt(getattr(client.mast, direction)(timeout=5), f"mast.{direction}")
        if target_height_mm is not None:
            self._mast_cancel = threading.Event()
            cancel = self._mast_cancel
            self._mast_thread = threading.Thread(
                target=self._watch_mast_target, args=(client, direction, target, cancel),
                name="des-mast-target", daemon=True)
            self._mast_thread.start()
        return receipt

    def _watch_mast_target(self, client, direction, target, cancel):
        previous = None
        last_progress = time.monotonic()
        try:
            while not cancel.wait(0.2):
                current = self._mast_height(client.get_status(timeout=5))
                if (direction == "extend" and current >= target) or (direction == "retract" and current <= target):
                    break
                if previous is None or abs(current - previous) >= 1.0:
                    previous = current
                    last_progress = time.monotonic()
                elif time.monotonic() - last_progress > 5.0:
                    raise RuntimeError("Mast height has not changed for 5 seconds")
        except Exception as exc:
            logger.error(f"[DES] Mast target monitoring failed; stopping mast: {exc}")
        if not cancel.is_set():
            try:
                self._receipt(client.mast.stop(timeout=5), "mast.stop")
            except Exception as exc:
                logger.error(f"[DES] Automatic mast stop failed: {exc}")

    def set_mast_extend(self, target_height_mm: float | None = None):
        return self._mast_motion("extend", target_height_mm)

    def set_mast_retract(self, target_height_mm: float | None = None):
        return self._mast_motion("retract", target_height_mm)

    def set_mast_stop(self):
        self._cancel_mast_target()
        return self._receipt(self._require_connected_client().mast.stop(timeout=5), "mast.stop")

    def set_erector_fold(self):
        return self._receipt(self._require_client().erector.fold(timeout=5), "erector.fold")

    def set_erector_unfold(self):
        return self._receipt(self._require_client().erector.unfold(timeout=5), "erector.unfold")

    def set_erector_stop(self):
        return self._receipt(self._require_connected_client().erector.stop(timeout=5), "erector.stop")

    def get_status(self) -> dict:
        client = self._client
        if client is None or not self.is_connected:
            return {"connected": False, "end_point": self.end_point}
        return {"connected": True, "end_point": self.end_point,
                "has_control": client.has_control()}

    def get_mast_height(self) -> float:
        return self._mast_height(self._require_connected_client().get_status(timeout=5))
