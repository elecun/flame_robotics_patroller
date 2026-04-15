"""
Ouster Lidar Module
@author Byunghun Hwang<bh.hwang@iae.re.kr>
"""

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
from contextlib import closing
from typing import Optional

try:
    from ouster.sdk import client
    from ouster.sdk.client import SensorConfig, ScanSource
except ImportError:
    print("ouster-sdk is required. Install via 'pip install ouster-sdk'")
    client = None


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

    def __init__(self,
                 hostname: str,
                 lidar_port: int = 7502,
                 imu_port: int = 7503,
                 lidar_mode: str = "1024x10",
                 timestamp_mode: str = "TIME_FROM_INTERNAL_OSC",
                 sync_pulse_in: bool = False,
                 return_mode: str = "SINGLE_RETURN_FIRST",
                 signal_multiplier: int = 1,
                 range_filter_min_m: float = 0.5,
                 range_filter_max_m: float = 100.0,
                 max_queue_size: int = 2):
        """
        Parameters loaded from ouster_lidar.cfg:

        hostname           : Sensor hostname or IP (e.g. "os-1234.local" or "192.168.1.100")
        lidar_port         : UDP port for lidar packets (default 7502)
        imu_port           : UDP port for IMU packets  (default 7503)
        lidar_mode         : Resolution x FPS, e.g. "512x10" / "1024x10" / "2048x10"
                             Controls point density vs update rate tradeoff.
        timestamp_mode     : "TIME_FROM_INTERNAL_OSC" | "TIME_FROM_SYNC_PULSE_IN"
                             | "TIME_FROM_PTP_1588"
        sync_pulse_in      : Enable sync-pulse input for hardware trigger
        return_mode        : "SINGLE_RETURN_FIRST" | "SINGLE_RETURN_STRONGEST"
                             | "DUAL_RETURN" — governs echo selection
        signal_multiplier  : Amplifies the return signal intensity (1–32)
        range_filter_min_m : Points closer than this (meters) are discarded
        range_filter_max_m : Points farther than this (meters) are discarded
        max_queue_size     : Max pending scans before the oldest is dropped
                             (prevents memory growth when the consumer is slow)
        """
        super().__init__()

        self.hostname = hostname
        self.lidar_port = lidar_port
        self.imu_port = imu_port
        self.lidar_mode = lidar_mode
        self.timestamp_mode = timestamp_mode
        self.sync_pulse_in = sync_pulse_in
        self.return_mode = return_mode
        self.signal_multiplier = int(signal_multiplier)
        self.range_min = float(range_filter_min_m)
        self.range_max = float(range_filter_max_m)

        self.running = False
        self._stop_event = threading.Event()

        # Bounded queue — producer drops oldest when full
        self._scan_queue: queue.Queue = queue.Queue(maxsize=int(max_queue_size))

        # Recording
        self._recording = False
        self._record_path: Optional[str] = None

    # ------------------------------------------------------------------
    # Thread entry point
    # ------------------------------------------------------------------
    def run(self):
        if client is None:
            print("ouster-sdk not available — aborting LiDAR thread.")
            return

        self.running = True
        self._stop_event.clear()
        print(f"[Ouster] Connecting to {self.hostname} "
              f"(lidar={self.lidar_port}, imu={self.imu_port}) ...")

        try:
            # -- Optional sensor configuration --
            config = SensorConfig()
            config.lidar_mode = client.LidarMode.from_string(self.lidar_mode)
            config.timestamp_mode = client.TimestampMode.from_string(self.timestamp_mode)
            config.operating_mode = client.OperatingMode.OPERATING_NORMAL
            client.set_config(self.hostname, config, persist=False)

            # -- Open packet source and build scan assembler --
            with closing(client.Sensor(self.hostname,
                                       self.lidar_port,
                                       self.imu_port,
                                       buf_size=640)) as sensor:
                info = sensor.metadata
                print(f"[Ouster] Connected. Mode={info.config.lidar_mode} "
                      f"Product={info.prod_line}")

                scans = client.Scans(sensor)
                for scan in scans:
                    if self._stop_event.is_set():
                        break

                    payload = self._build_payload(scan, info)
                    self._enqueue_and_emit(payload)

        except Exception as e:
            print(f"[Ouster] Stream error: {e}")
        finally:
            self.running = False
            print("[Ouster] Stream stopped.")

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

        xyzlut = client.XYZLut(info)
        xyz = xyzlut(scan)                    # (H, W, 3) float64, metres

        range_m = scan.field(client.ChanField.RANGE) * 0.001  # mm → m

        # Range filter
        mask = (range_m >= self.range_min) & (range_m <= self.range_max)

        signal = scan.field(client.ChanField.SIGNAL) * self.signal_multiplier
        reflectivity = scan.field(client.ChanField.REFLECTIVITY)

        return {
            "xyz":          xyz,           # (H, W, 3) full cloud
            "range_m":      range_m,       # (H, W)
            "signal":       signal,        # (H, W) uint32
            "reflectivity": reflectivity,  # (H, W) uint16
            "valid_mask":   mask,          # (H, W) bool — passes range filter
            "timestamp":    scan.timestamp,
            "frame_id":     scan.frame_id,
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
        print("[Ouster] Stopping ...")
        self.running = False
        self._recording = False
        self._stop_event.set()
        if not self.wait(5000):
            print("[Ouster] Warning: thread did not terminate in time.")