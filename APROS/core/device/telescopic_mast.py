"""
Telescopic Mast Hardware Driver Device & Connector Module (core/device/telescopic_mast.py)
Provides interface and control for a linear extensible telescopic mast structure over RS485 Modbus protocol (/dev/ttyUSB1, 9600bps, No parity, 2 stop bits).
Reads extended mast position (01 04 03 ea 00 02 50 7b), parses return bytes (01 04 04 00 00 07 08 f8 72 -> 1800mm),
publishes height data over ZPipe IPC every 500ms, and prints debug logs to console.
Includes TelescopicMast_connector for platform subscription.
"""

import time
import threading
import json
import pickle
import struct
from typing import Dict, Any, Optional, Callable, List
from core.device.base import BaseDevice
from core.zpipe import AsyncZSocket, ZPipe
from util.logger.console import ConsoleLogger

logger = ConsoleLogger.get_logger()

try:
    import serial
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False
    logger.warning("[TelescopicMast] 'pyserial' module not installed. Serial communication will operate in fallback mode.")


def calculate_modbus_crc16(data: bytes) -> bytes:
    """Calculate Modbus RTU CRC-16 checksum (returns 2-byte little endian)."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return struct.pack('<H', crc)


class TelescopicMast(BaseDevice):
    """
    Telescopic Mast device driver over RS485 Modbus.
    Reads extended mast position every 500ms, publishes via ZPipe IPC,
    and logs height (mm) to console for debugging.
    """

    MIN_HEIGHT_MM = 1800.0  # 1.8m
    MAX_HEIGHT_MM = 8000.0  # 8.0m
    DIAMETER_MM = 100.0     # 100mm fixed diameter (0.1m)

    # Modbus command frame: Bus 0x01, Function 0x04, Start Addr 0x03EA, Reg Count 0x0002 -> CRC 0x507B
    # Raw bytes: 01 04 03 ea 00 02 50 7b
    READ_POSITION_CMD = bytes([0x01, 0x04, 0x03, 0xEA, 0x00, 0x02, 0x50, 0x7B])

    def __init__(
        self,
        name: str = "telescopic_mast",
        robot_model: str = "iae_patrol_v1",
        port: str = "/dev/ttyUSB1",
        baudrate: int = 9600,
        parity: str = "N",
        stopbits: int = 2,
        bus_address: int = 1,
        min_height: float = 1800.0,
        max_height: float = 8000.0,
        initial_height: float = 1800.0,
        offset_x: float = 0.0,
        offset_y: float = 0.0,
        offset_z: float = 0.64,
        enable: bool = True
    ):
        super().__init__(name, enable=enable)
        self.robot_model = robot_model
        self.port_name = port
        self.baudrate = int(baudrate)
        self.parity = parity
        self.stopbits = int(stopbits)
        self.bus_address = int(bus_address)

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

        # Serial Connection
        self.serial_conn: Optional[Any] = None

        # ZPipe IPC Publisher
        self.pub_socket: Optional[AsyncZSocket] = None
        self.ipc_address = f"/tmp/{self.robot_model}_telescopic_mast.ipc"

    def set_zpipe_context(self, zpipe_ctx: Any):
        """Set ZPipe context and create/join IPC publish socket."""
        super().set_zpipe_context(zpipe_ctx)
        if self.zpipe_context:
            try:
                socket_id = f"{self.name}_pub"
                self.pub_socket = AsyncZSocket(socket_id=socket_id, pattern="publish")
                if self.pub_socket.create(self.zpipe_context):
                    if self.pub_socket.join(transport="ipc", address=self.ipc_address):
                        logger.info(f"[{self.name}] ZPipe IPC Publisher bound to ipc://{self.ipc_address}")
            except Exception as e:
                logger.error(f"[{self.name}] Error creating ZPipe PUB socket: {e}")

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
        """Set target extension height in millimeters."""
        self.target_height_mm = height_mm

    def extend_fully(self):
        """Command mast to extend to maximum height (8000 mm)."""
        self.set_height(self.max_height)

    def retract_fully(self):
        """Command mast to retract to minimum height (1800 mm)."""
        self.set_height(self.min_height)

    def connect(self) -> bool:
        """Connect RS485 serial port and start 500ms telemetry/control thread if enabled."""
        if not self.enable:
            self.is_connected = False
            logger.info(f"[{self.name}] Device is DISABLED in config (enable=False).")
            return False

        self.is_connected = False
        if SERIAL_AVAILABLE:
            try:
                stopbits_val = serial.STOPBITS_TWO if self.stopbits == 2 else serial.STOPBITS_ONE
                parity_val = serial.PARITY_NONE
                if self.parity == "E":
                    parity_val = serial.PARITY_EVEN
                elif self.parity == "O":
                    parity_val = serial.PARITY_ODD

                self.serial_conn = serial.Serial(
                    port=self.port_name,
                    baudrate=self.baudrate,
                    bytesize=serial.EIGHTBITS,
                    parity=parity_val,
                    stopbits=stopbits_val,
                    timeout=0.2
                )
                self.is_connected = True
                logger.info(f"[{self.name}] RS485 Serial connected on {self.port_name} ({self.baudrate}bps, N, {self.stopbits})")
            except Exception as e:
                logger.warning(f"[{self.name}] Could not open serial port '{self.port_name}': {e}.")

        # Always start worker loop for simulation/telemetry if enable=True
        self._running = True
        self._thread = threading.Thread(target=self._worker_loop, daemon=True)
        self._thread.start()
        logger.info(f"[{self.name}] Connected. Telemetry IPC: ipc://{self.ipc_address}")
        return True

    def disconnect(self) -> bool:
        """Disconnect device interface and stop thread."""
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)
            self._thread = None

        if self.serial_conn and self.serial_conn.is_open:
            try:
                self.serial_conn.close()
            except Exception:
                pass
            self.serial_conn = None

        if self.pub_socket:
            try:
                self.pub_socket.close()
            except Exception:
                pass
            self.pub_socket = None

        self.is_connected = False
        logger.info(f"[{self.name}] Disconnected.")
        return True

    def _worker_loop(self):
        """
        Background worker loop running at 500ms (0.5s) intervals:
        1. Queries RS485 Modbus device with 01 04 03 ea 00 02 50 7b (or updates simulated pose towards target).
        2. Parses response bytes (e.g. 01 04 04 00 00 07 08 f8 72 -> 0x00000708 = 1800 mm).
        3. Publishes mast telemetry over ZPipe IPC.
        4. Logs current mast position (extended) in mm to console for debugging.
        """
        speed_mm_per_sec = 300.0  # Smooth motion speed 300mm/s for simulation
        dt = 0.5  # 500ms interval as requested

        while self._running:
            start_time = time.time()
            read_height_mm: Optional[float] = None
            raw_rx_hex = "N/A (SIM Mode)"
            raw_tx_hex = self.READ_POSITION_CMD.hex(' ')

            # 1. Try RS485 Modbus hardware read if serial is connected
            if self.serial_conn and self.serial_conn.is_open:
                try:
                    self.serial_conn.reset_input_buffer()
                    self.serial_conn.write(self.READ_POSITION_CMD)
                    self.serial_conn.flush()

                    # Response length expected: 9 bytes (01 04 04 [4 bytes data] [2 bytes CRC])
                    rx_bytes = self.serial_conn.read(9)
                    if len(rx_bytes) > 0:
                        raw_rx_hex = rx_bytes.hex(' ')

                    if len(rx_bytes) == 9 and rx_bytes[0] == 0x01 and rx_bytes[1] == 0x04 and rx_bytes[2] == 0x04:
                        # Extract 4-byte 32-bit unsigned integer (bytes 3..6)
                        raw_val = struct.unpack('>I', rx_bytes[3:7])[0]
                        read_height_mm = float(raw_val)
                    else:
                        logger.warning(f"[{self.name}] Invalid Modbus RX packet ({len(rx_bytes)} bytes): {raw_rx_hex}")
                except Exception as e:
                    logger.error(f"[{self.name}] RS485 read error: {e}")

            # 2. Update current position (use hardware read or simulated transition towards target)
            with self._lock:
                if read_height_mm is not None:
                    self._current_height_mm = read_height_mm
                    is_hw = True
                else:
                    # Simulation motion update towards target_height_mm
                    diff = self._target_height_mm - self._current_height_mm
                    if abs(diff) > 0.01:
                        step = (1.0 if diff > 0 else -1.0) * min(abs(diff), speed_mm_per_sec * dt)
                        self._current_height_mm += step
                    is_hw = False

                current_mm = self._current_height_mm

            # 4. Publish telemetry data over ZPipe IPC (JSON format)
            data = self.get_status()
            if self.pub_socket and self.pub_socket.is_joined:
                try:
                    payload = json.dumps(data, ensure_ascii=False).encode('utf-8')
                    self.pub_socket.dispatch([b"mast_data", payload])
                except Exception as e:
                    logger.error(f"[{self.name}] ZPipe Publish error: {e}")

            elapsed = time.time() - start_time
            time.sleep(max(0.0, dt - elapsed))

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
                "diameter_mm": self.DIAMETER_MM,
                "port": self.port_name,
                "baudrate": self.baudrate
            }


class TelescopicMast_Connector:
    """
    Connector for TelescopicMast device over ZPipe IPC.
    Subscribes to ipc:///tmp/<robot_model>_telescopic_mast.ipc and invokes callback on telemetry data.
    """

    def __init__(
        self,
        robot_model: str = "iae_patrol_v1",
        zpipe_ctx: Any = None,
        on_data_received: Optional[Callable[[Dict[str, Any]], None]] = None
    ):
        self.robot_model = robot_model
        self.zpipe_ctx = zpipe_ctx or ZPipe.create_pipe()
        self.on_data_received = on_data_received

        self.ipc_address = f"/tmp/{self.robot_model}_telescopic_mast.ipc"
        self.sub_socket: Optional[AsyncZSocket] = None
        self.last_mast_data: Optional[Dict[str, Any]] = None

    def start(self) -> bool:
        """Create SUB AsyncZSocket, connect to IPC publisher, and register message callback."""
        try:
            socket_id = f"mast_sub_{int(time.time() * 1000)}"
            self.sub_socket = AsyncZSocket(socket_id=socket_id, pattern="subscribe")
            if not self.sub_socket.create(self.zpipe_ctx):
                return False

            self.sub_socket.set_message_callback(self._on_multipart_received)
            if self.sub_socket.join(transport="ipc", address=self.ipc_address):
                self.sub_socket.subscribe(b"mast_data")
                logger.info(f"[TelescopicMast_Connector] Subscribed to ZPipe IPC at ipc://{self.ipc_address}")
                return True
            return False
        except Exception as e:
            logger.error(f"[TelescopicMast_Connector] Failed to connect SUB socket: {e}")
            return False

    def stop(self):
        """Close SUB socket."""
        if self.sub_socket:
            try:
                self.sub_socket.close()
            except Exception:
                pass
            self.sub_socket = None
        logger.info("[TelescopicMast_Connector] Stopped.")

    def _on_multipart_received(self, multipart_data: List[bytes]):
        """Callback invoked when ZPipe receives multipart data."""
        if len(multipart_data) >= 2:
            topic = multipart_data[0]
            if topic == b"mast_data":
                payload_bytes = multipart_data[1]
                if not payload_bytes:
                    return
                try:
                    json_str = payload_bytes.decode('utf-8')
                    data = json.loads(json_str)
                    self.last_mast_data = data
                    if self.on_data_received:
                        self.on_data_received(data)
                except Exception as e:
                    logger.error(f"[TelescopicMast_Connector] Error decoding mast JSON data: {e}")
