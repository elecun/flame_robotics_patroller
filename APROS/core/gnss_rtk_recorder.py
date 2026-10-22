"""Record newly received GNSS/RTK coordinates to a standalone CSV file."""

import csv
import os
import threading
import time
from datetime import datetime
from typing import Any, Optional

from util.logger.console import ConsoleLogger


logger = ConsoleLogger.get_logger()


class GNSSRTKRecorder:
    def __init__(self, robot: Any, output_dir: Optional[str] = None):
        self.robot = robot
        apros_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        self.output_dir = output_dir or os.path.join(apros_root, "datalog")
        self.is_recording = False
        self.file_path: Optional[str] = None
        self.last_error: Optional[str] = None
        self._file = None
        self._writer = None
        self._thread: Optional[threading.Thread] = None
        self._last_data = None

    def start(self) -> str:
        if self.is_recording:
            return self.file_path

        devices = getattr(self.robot, "devices", {}) or {}
        if not any(name in devices for name in ("synerex_syn3100", "synerex_syn2000", "synerex_rtk")):
            raise RuntimeError("GNSS/RTK device is not configured")

        os.makedirs(self.output_dir, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        for index in range(100):
            suffix = f"_{index:02d}" if index else ""
            path = os.path.join(self.output_dir, f"{timestamp}{suffix}_rtk.csv")
            try:
                self._file = open(path, "x", newline="", encoding="utf-8")
                break
            except FileExistsError:
                continue
        else:
            raise RuntimeError("Could not create a unique GNSS/RTK filename")

        try:
            self._writer = csv.writer(self._file)
            self._writer.writerow(["timestamp", "latitude", "longitude", "rtk_quality"])
            self._file.flush()
            self.file_path = path
            self.last_error = None
            self._last_data = getattr(self.robot, "last_rtk_data", None)
            self.is_recording = True
            self._thread = threading.Thread(target=self._record_loop, daemon=True, name="gnss_rtk_recorder")
            self._thread.start()
            logger.info(f"[GNSSRTKRecorder] Recording started: {path}")
            return path
        except Exception:
            self._file.close()
            self._file = None
            self._writer = None
            raise

    def stop(self) -> None:
        if not self.is_recording and self._file is None:
            return
        self.is_recording = False
        if self._thread is not None:
            self._thread.join()
            self._thread = None
        if self._file is not None:
            self._file.close()
            self._file = None
            self._writer = None
        logger.info(f"[GNSSRTKRecorder] Recording stopped: {self.file_path}")

    def _record_loop(self) -> None:
        while self.is_recording:
            data = getattr(self.robot, "last_rtk_data", None)
            if data is not None and data is not self._last_data:
                self._last_data = data
                if isinstance(data, dict) and data.get("new_updated"):
                    latitude = data.get("latitude")
                    longitude = data.get("longitude")
                    if latitude is not None and longitude is not None:
                        try:
                            self._writer.writerow([
                                datetime.now().astimezone().isoformat(timespec="milliseconds"),
                                latitude,
                                longitude,
                                data.get("fix_quality"),
                            ])
                            self._file.flush()
                        except Exception as error:
                            self.last_error = str(error)
                            logger.error(f"[GNSSRTKRecorder] CSV write failed: {error}")
                            self.is_recording = False
                            break
            time.sleep(0.05)
        if self.last_error and self._file is not None:
            self._file.close()
            self._file = None
            self._writer = None
