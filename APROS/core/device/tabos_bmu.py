"""TABOS BMU RS232 transport scaffold.

The BMU packet format and request command are not defined yet. This module
opens the configured 19200 8N1 serial link and checks for data once per second.
"""

import threading
import time
from typing import Optional

from core.device.base import BaseDevice
from util.logger.console import ConsoleLogger

try:
    import serial
except ImportError:
    serial = None


logger = ConsoleLogger.get_logger()


class TabosBMU(BaseDevice):
    def __init__(self, name: str = "tabos_bmu", port: str = "",
                 baudrate: int = 19200, word_length: int = 8,
                 parity: str = "N", flow_control: str = "none",
                 stop_bits: int = 1, poll_interval_sec: float = 1.0,
                 enable: bool = False, **kwargs):
        super().__init__(name, enable=enable)
        self.port = str(port).strip()
        self.baudrate = int(baudrate)
        self.word_length = int(word_length)
        self.parity = str(parity).upper()
        self.flow_control = str(flow_control).lower()
        self.stop_bits = int(stop_bits)
        self.poll_interval_sec = float(poll_interval_sec)
        if (self.baudrate != 19200 or self.word_length != 8 or self.parity != "N"
                or self.flow_control != "none" or self.stop_bits != 1):
            raise ValueError("TABOS BMU serial settings must be 19200 8N1 without flow control")
        if self.poll_interval_sec <= 0:
            raise ValueError("poll_interval_sec must be positive")

        self.serial_connection = None
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self.bytes_received = 0
        self.last_rx_timestamp: Optional[float] = None

    def build_request(self) -> Optional[bytes]:
        """Return a polling command when the BMU protocol is known."""
        return None

    def handle_response(self, data: bytes) -> None:
        """Decode received bytes when the BMU packet format is known."""

    def _poll_loop(self) -> None:
        while not self._stop.is_set():
            try:
                request = self.build_request()
                if request:
                    self.serial_connection.write(request)
                pending = self.serial_connection.in_waiting
                if pending:
                    data = self.serial_connection.read(pending)
                    if data:
                        with self._lock:
                            self.bytes_received += len(data)
                            self.last_rx_timestamp = time.time()
                        self.handle_response(data)
            except Exception as exc:
                logger.warning("[%s] Serial communication failed: %s", self.name, exc)
                self.is_connected = False
                self._stop.set()
                break
            self._stop.wait(self.poll_interval_sec)

    def connect(self) -> bool:
        if not self.enable:
            return False
        if not self.port:
            logger.error("[%s] Serial port is not configured", self.name)
            return False
        if serial is None:
            logger.error("[%s] pyserial is unavailable", self.name)
            return False
        try:
            self.serial_connection = serial.Serial(
                port=self.port, baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE, timeout=0,
                xonxoff=False, rtscts=False, dsrdtr=False,
            )
            self._stop.clear()
            self.is_connected = True
            self._thread = threading.Thread(target=self._poll_loop, daemon=True,
                                            name=f"{self.name}_serial")
            self._thread.start()
            return True
        except Exception as exc:
            logger.error("[%s] Serial connection failed: %s", self.name, exc)
            self.disconnect()
            return False

    def disconnect(self) -> bool:
        self._stop.set()
        if self._thread and self._thread is not threading.current_thread():
            self._thread.join(timeout=2.0)
        self._thread = None
        if self.serial_connection is not None:
            try:
                self.serial_connection.close()
            except Exception as exc:
                logger.warning("[%s] Serial close failed: %s", self.name, exc)
            self.serial_connection = None
        self.is_connected = False
        return True

    def get_status(self) -> dict:
        with self._lock:
            return {
                "connected": self.is_connected,
                "port": self.port,
                "bytes_received": self.bytes_received,
                "last_rx_timestamp": self.last_rx_timestamp,
            }
