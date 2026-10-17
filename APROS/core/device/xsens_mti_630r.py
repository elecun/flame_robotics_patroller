"""Xsens MTi-630R AHRS receiver for RS232/Xbus or preconfigured CAN output.

RS232 mode reads MTData2 Xbus frames. CAN mode receives an already configured
MT CAN Euler-angle frame (data identifier 0x22); configure CAN output and its
CAN arbitration ID on the sensor before using this mode.
"""
import struct
import threading
import time
from typing import Any, Dict, Optional

from core.device.base import BaseDevice
from util.logger.console import ConsoleLogger

logger = ConsoleLogger.get_logger()

try:
    import serial
except ImportError:
    serial = None

canlib = None


class XsensMTi630R(BaseDevice):
    """Read orientation and inertial measurements over a selected bus."""

    def __init__(self, name: str = "xsens_mti_630r", interface: str = "rs232",
                 port: str = "/dev/ttyUSB2", baudrate: int = 115200,
                 can_channel: int = 0, can_bitrate: int = 500000,
                 can_message_id: Optional[int] = None,
                 enable: bool = True, **kwargs):
        super().__init__(name, enable=enable)
        self.interface = str(interface).strip().lower()
        if self.interface in ("serial", "uart", "rs-232"):
            self.interface = "rs232"
        if self.interface not in ("rs232", "can"):
            raise ValueError("interface must be 'rs232' or 'can'")
        self.port, self.baudrate = str(port), int(baudrate)
        self.can_channel, self.can_bitrate = int(can_channel), int(can_bitrate)
        self.can_message_id = (int(can_message_id, 0) if can_message_id.strip() else None) if isinstance(can_message_id, str) else can_message_id
        self.serial_connection = None
        self.can_channel_handle = None
        self._running = False
        self._reader: Optional[threading.Thread] = None
        self._buffer = bytearray()
        self._lock = threading.Lock()
        self.telemetry: Dict[str, Any] = {
            "roll": None, "pitch": None, "yaw": None,
            "quaternion": None, "acceleration": None,
            "rate_of_turn": None, "timestamp": None, "sample_count": 0,
        }

    @staticmethod
    def parse_mtdata2(payload: bytes) -> Dict[str, Any]:
        """Decode common MTData2 XDI fields (Euler, quaternion, accel, gyro)."""
        out: Dict[str, Any] = {}
        pos = 0
        while pos + 3 <= len(payload):
            data_id = int.from_bytes(payload[pos:pos + 2], "big")
            size = payload[pos + 2]
            pos += 3
            if pos + size > len(payload):
                break
            data = payload[pos:pos + size]
            pos += size
            # XDI commonly uses 32-bit IEEE floats in network byte order.
            if data_id == 0x2030 and size == 12:  # Euler angles, degrees
                out["euler"] = struct.unpack(">3f", data)
            elif data_id == 0x2010 and size == 16:  # Quaternion, w/x/y/z
                out["quaternion"] = struct.unpack(">4f", data)
            elif data_id in (0x4020, 0x4030) and size == 12:
                out["acceleration"] = struct.unpack(">3f", data)
            elif data_id == 0x8020 and size == 12:
                out["rate_of_turn"] = struct.unpack(">3f", data)
        return out

    def _update(self, values: Dict[str, Any]):
        if not values:
            return
        with self._lock:
            euler = values.get("euler")
            if euler:
                self.telemetry.update(roll=euler[0], pitch=euler[1], yaw=euler[2])
            for key in ("quaternion", "acceleration", "rate_of_turn"):
                if key in values:
                    self.telemetry[key] = values[key]
            self.telemetry["timestamp"] = time.time()
            self.telemetry["sample_count"] += 1

    def _consume_xbus(self, data: bytes):
        self._buffer.extend(data)
        while True:
            start = self._buffer.find(b"\xfa")
            if start < 0:
                self._buffer.clear()
                return
            if start:
                del self._buffer[:start]
            if len(self._buffer) < 4:
                return
            length = self._buffer[3]
            header_len = 4
            if length == 0xff:
                if len(self._buffer) < 6:
                    return
                length = int.from_bytes(self._buffer[4:6], "big")
                header_len = 6
            frame_len = header_len + length + 1
            if len(self._buffer) < frame_len:
                return
            frame = bytes(self._buffer[:frame_len])
            del self._buffer[:frame_len]
            if sum(frame[1:]) & 0xff:
                continue
            # MTData2 MID is 0x36; other valid Xbus frames carry no samples.
            if frame[2] == 0x36:
                self._update(self.parse_mtdata2(frame[header_len:-1]))

    def connect(self) -> bool:
        if not self.enable:
            return False
        try:
            if self.interface == "rs232":
                if serial is None:
                    logger.error("[%s] pyserial is unavailable", self.name)
                    return False
                self.serial_connection = serial.Serial(self.port, self.baudrate, timeout=0.2)
            else:
                # Import CANlib only when CAN is selected. This keeps RS232 and
                # simulation startup independent of the Kvaser driver library.
                global canlib
                if canlib is None:
                    try:
                        from canlib import canlib as canlib_module
                        canlib = canlib_module
                    except ImportError:
                        canlib = None
                if canlib is None:
                    logger.error("[%s] Kvaser CANlib is unavailable", self.name)
                    return False
                self.can_channel_handle = canlib.openChannel(self.can_channel)
                bitrate = getattr(canlib.Bitrate, f"BITRATE_{self.can_bitrate // 1000}K", None)
                if bitrate is None:
                    raise ValueError(f"Unsupported CAN bitrate: {self.can_bitrate}")
                self.can_channel_handle.setBusParams(bitrate)
                self.can_channel_handle.busOn()
            self.is_connected = self._running = True
            self._reader = threading.Thread(target=self._read_loop, daemon=True, name=f"{self.name}_reader")
            self._reader.start()
            logger.info("[%s] Connected using %s", self.name, self.interface.upper())
            return True
        except Exception as exc:
            logger.error("[%s] %s connection failed: %s", self.name, self.interface.upper(), exc)
            self.disconnect()
            return False

    def _read_loop(self):
        while self._running:
            try:
                if self.interface == "rs232":
                    data = self.serial_connection.read(512)
                    if data:
                        self._consume_xbus(data)
                else:
                    frame = self.can_channel_handle.read(timeout=100)
                    can_id = getattr(frame, "id", getattr(frame, "id_", None))
                    # CAN output identifiers are assigned by the sensor's CAN
                    # configuration. Without an arbitration ID there is no safe
                    # way to distinguish Euler data from other CAN output fields.
                    if self.can_message_id is None:
                        continue
                    if self.can_message_id is not None and can_id != self.can_message_id:
                        continue
                    data = bytes(getattr(frame, "data", b""))
                    # Xsens CAN Euler output: three signed big-endian int16 values,
                    # scale 2^-7 degrees.
                    if len(data) >= 6:
                        euler = tuple(v / 128.0 for v in struct.unpack(">3h", data[:6]))
                        self._update({"euler": euler})
            except Exception as exc:
                if self._running and "nomsg" not in str(exc).lower() and "timeout" not in str(exc).lower():
                    logger.debug("[%s] Read error: %s", self.name, exc)

    def disconnect(self) -> bool:
        self._running = False
        if self.serial_connection is not None:
            try:
                self.serial_connection.close()
            except Exception:
                pass
            self.serial_connection = None
        if self.can_channel_handle is not None:
            try:
                self.can_channel_handle.busOff()
                self.can_channel_handle.close()
            except Exception:
                pass
            self.can_channel_handle = None
        self.is_connected = False
        return True

    def get_status(self) -> dict:
        with self._lock:
            return {"name": self.name, "enabled": self.enable,
                    "connected": self.is_connected, "interface": self.interface,
                    **self.telemetry.copy()}
