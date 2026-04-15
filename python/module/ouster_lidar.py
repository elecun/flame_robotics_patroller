"""
Ouster Lidar Module
@author Byunghun Hwang<bh.hwang@iae.re.kr>
"""

from util.logger.console import ConsoleLogger

try:
    from PyQt6.QtCore import QThread, pyqtSignal
except ImportError:
    print("PyQt6 is required to run this module.")
    class QThread:
        def __init__(self): pass
    class pyqtSignal:
        def __init__(self, *args, **kwargs): pass
        def emit(self, *args, **kwargs): pass

import queue
import threading
from typing import Optional

try:
    from ouster.sdk import core
    from ouster.sdk import sensor as sensor_mod
    from ouster.sdk._bindings.client import Sensor as OusterSensor
    from ouster.sdk.core import SensorConfig
except ImportError:
    print("ouster-sdk is required. Install via 'pip install ouster-sdk'")
    core = None
    sensor_mod = None
    OusterSensor = None
    SensorConfig = None


class component(QThread):
    """
    Ouster LiDAR sensor communication thread.

    * Receives LidarScan frames from the sensor in a background thread.
    * Assembles complete scans and emits them via signal_updated.

    --- Qt signal & large-data note ---
    PyQt signals pass Python object *references*, so no memory copy occurs at
    emit() time. A LidarScan (~5 MB of numpy arrays) is therefore cheap to
    signal across threads. The bottleneck is the *receiving* slot: if the UI
    slot takes longer than ~100 ms the internal Qt event queue can back up.
    To guard against this, a bounded queue (max_queue_size from cfg) is used so
    that slow consumers cause the producer to drop the oldest frame rather than
    accumulate unbounded memory.
    """

    # Emits a dict with LidarScan fields (numpy arrays) so the receiver is
    # independent of the ouster-sdk object lifecyle.
    signal_updated = pyqtSignal(object)

    def __init__(self, **kwargs):
        """
        Parameters loaded from ouster_lidar.cfg via **kwargs.
        """
        super().__init__()

        self.__console = ConsoleLogger.get_logger()

        self.hostname = kwargs.get("hostname", "os-1234.local")
        self.lidar_port = int(kwargs.get("lidar_port", 7502))
        self.imu_port = int(kwargs.get("imu_port", 7503))
        self.lidar_mode = kwargs.get("lidar_mode", "1024x10")
        self.timestamp_mode = kwargs.get("timestamp_mode", "TIME_FROM_INTERNAL_OSC")
        self.sync_pulse_in = kwargs.get("sync_pulse_in", False)
        self.return_mode = kwargs.get("return_mode", "SINGLE_RETURN_FIRST")
        self.signal_multiplier = int(kwargs.get("signal_multiplier", 1))
        self.range_min = float(kwargs.get("range_filter_min_m", 0.5))
        self.range_max = float(kwargs.get("range_filter_max_m", 100.0))
        self.max_queue_size = int(kwargs.get("max_queue_size", 2))
        self.coordinate = kwargs.get("coordinate", "standard")
        self.show_coordinate = bool(kwargs.get("show_coordinate", True))
        self.filter_angles = kwargs.get("filter", [-45, 45])

        self._scan_queue: queue.Queue = queue.Queue(maxsize=self.max_queue_size)
        self._stop_event = threading.Event()
        # Bounded queue — producer drops oldest when full
        
        # Recording
        self._recording = False
        self._record_path: Optional[str] = None

    # ------------------------------------------------------------------
    # Thread entry point
    # ------------------------------------------------------------------
    def run(self):
        if core is None:
            self.__console.error("ouster-sdk not available — aborting LiDAR thread.")
            return

        self.running = True
        self._stop_event.clear()
        self.__console.debug(f"[Ouster] Connecting to {self.hostname} "
              f"(lidar={self.lidar_port}, imu={self.imu_port}) ...")

        scan_source = None
        try:
            # -- Sensor configuration --
            config = SensorConfig()
            config.lidar_mode = core.LidarMode.from_string(self.lidar_mode)
            config.timestamp_mode = core.TimestampMode.from_string(self.timestamp_mode)
            config.operating_mode = core.OperatingMode.OPERATING_NORMAL
            sensor_mod.set_config(self.hostname, config, persist=False)

            # -- Create sensor and scan source (ouster-sdk 0.16.1 API) --
            ouster_sensor = OusterSensor(self.hostname, config)
            scan_source = sensor_mod.SensorScanSource(
                [ouster_sensor], config_timeout=40, queue_size=2)

            info = scan_source.sensor_info[0]
            self.__console.debug(f"[Ouster] Connected. Mode={info.config.lidar_mode} "
                  f"Product={info.prod_line}")

            for scan_set in scan_source.single_iter(0):
                if self._stop_event.is_set():
                    break

                scan = scan_set[0]
                if scan is None:
                    continue

                payload = self._build_payload(scan, info)
                self._enqueue_and_emit(payload)

        except Exception as e:
            self.__console.error(f"[Ouster] Stream error: {e}")
        finally:
            if scan_source is not None:
                try:
                    scan_source.close()
                except Exception:
                    pass
            self.running = False
            self.__console.debug("[Ouster] Stream stopped.")

    # ------------------------------------------------------------------
    # Payload construction
    # ------------------------------------------------------------------
    def _build_payload(self, scan, info) -> dict:
        """
        Convert a LidarScan into a plain dict of numpy arrays.

        Extracting arrays at this point means the caller does not need the
        ouster-sdk objects after the dict is created, which simplifies
        lifecycle management in the consumer.
        """
        import numpy as np

        xyzlut = core.XYZLut(info)
        xyz = xyzlut(scan)                    # (H, W, 3) float64, metres

        # Coordinate transformation based on physical mounting
        if self.coordinate == "rotate_ccw_90":
            # CCW 90 degrees around X-axis: x'=x, y'=-z, z'=y
            tmp_y = xyz[..., 1].copy()
            xyz[..., 1] = -xyz[..., 2]
            xyz[..., 2] = tmp_y
        elif self.coordinate == "rotate_cw_90":
            # CW 90 degrees around X-axis: x'=x, y'=z, z'=-y
            tmp_y = xyz[..., 1].copy()
            xyz[..., 1] = xyz[..., 2]
            xyz[..., 2] = -tmp_y

        range_m = scan.field(core.ChanField.RANGE) * 0.001  # mm → m

        # Azimuth filter
        angles_deg = np.degrees(np.arctan2(xyz[..., 1], xyz[..., 0]))
        if len(self.filter_angles) >= 2:
            min_ang, max_ang = self.filter_angles[0], self.filter_angles[1]
        else:
            min_ang, max_ang = -180, 180
            
        angle_mask = (angles_deg >= min_ang) & (angles_deg <= max_ang)

        # Range filter & combine masks
        mask = (range_m >= self.range_min) & (range_m <= self.range_max) & angle_mask

        signal = scan.field(core.ChanField.SIGNAL) * self.signal_multiplier
        reflectivity = scan.field(core.ChanField.REFLECTIVITY)

        return {
            "xyz":          xyz,           # (H, W, 3) full cloud
            "range_m":      range_m,       # (H, W)
            "signal":       signal,        # (H, W) uint32
            "reflectivity": reflectivity,  # (H, W) uint16
            "valid_mask":   mask,          # (H, W) bool — passes range filter
            "timestamp":    scan.timestamp,
            "frame_id":     scan.frame_id,
            "show_coordinate": self.show_coordinate,
        }

    # ------------------------------------------------------------------
    # Queue + emit  (drop oldest on overflow to prevent memory growth)
    # ------------------------------------------------------------------
    def _enqueue_and_emit(self, payload: dict):
        try:
            self._scan_queue.put_nowait(payload)
        except queue.Full:
            # Consumer is slow — discard oldest, insert newest
            try:
                self._scan_queue.get_nowait()
            except queue.Empty:
                pass
            self._scan_queue.put_nowait(payload)

        # Emit a reference to the payload (no copy — Qt signals pass refs)
        # if "valid_mask" in payload:
        #     valid_pts = payload["valid_mask"].sum()
        #     self.__console.debug(f"[Ouster] Emitting scan: {valid_pts} valid points.")
        self.signal_updated.emit(payload)

    # ------------------------------------------------------------------
    # Recording helpers
    # ------------------------------------------------------------------
    def start_record(self, filepath: str):
        """Begin recording raw pcap to *filepath*."""
        self._record_path = filepath
        self._recording = True

    def stop_record(self):
        self._recording = False

    # ------------------------------------------------------------------
    # Shutdown
    # ------------------------------------------------------------------
    def stop(self):
        self.__console.debug("[Ouster] Stopping ...")
        self.running = False
        self._recording = False
        self._stop_event.set()
        if not self.wait(5000):
            self.__console.warning("[Ouster] Thread did not terminate in time.")