"""
Telescopic Mast Hardware Driver Device & Connector Module (core/device/telescopic_mast.py)
Provides interface and control for a linear extensible telescopic mast structure over RS485 Modbus protocol (/dev/ttyUSB1, 9600bps, No parity, 2 stop bits).
Reads extended mast position (01 04 03 ea 00 02 50 7b), parses return bytes (01 04 04 00 00 07 08 f8 72 -> 1800mm),
publishes height data over ZPipe IPC every 500ms, and prints debug logs to console.
Includes TelescopicMast_connector for platform subscription.

Mast control commands mast_up, mast_down, mast_stop) are published via a separate
ZPipe IPC proxy channel so that FoldableTelescopicMast (running in another process)
can subscribe and execute raise_mast / lower_mast / stop_mast_action accordingly.
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
    from pymodbus.client import ModbusSerialClient
    PYMODBUS_AVAILABLE = True
except ImportError:
    PYMODBUS_AVAILABLE = False
    ModbusSerialClient = None
    logger.warning("[TelescopicMast] 'pymodbus' module not installed. Communication will operate in fallback mode.")


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
    Reads extended mast position every 100ms using pymodbus, publishes via ZPipe IPC,
    and logs height (mm) to console for debugging.
    """

    MIN_HEIGHT_MM = 2900.0  # 2.9m (1.8m mast + 1.1m ground offset)
    MAX_HEIGHT_MM = 9100.0  # 9.1m (8.0m mast + 1.1m ground offset)
    DIAMETER_MM = 100.0     # 100mm fixed diameter (0.1m)

    def __init__(
        self,
        name: str = "telescopic_mast",
        robot_model: str = "iae_patrol_v1",
        port: str = "/dev/ttyUSB0",
        baudrate: int = 9600,
        parity: str = "N",
        stopbits: int = 2,
        bus_address: int = 1,
        min_height: float = 2900.0,
        max_height: float = 9100.0,
        initial_height: float = 2900.0,
        stop_trig_bound: float = 15.0,
        offset_x: float = 0.0,
        offset_y: float = 0.0,
        offset_z: float = 0.78,
        enable: bool = True,
        **kwargs
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
        self.stop_trig_bound = float(stop_trig_bound)

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

        # Modbus Serial Client
        self.modbus_client: Optional[Any] = None

        # ZPipe IPC Publisher (height telemetry)
        self.pub_socket: Optional[AsyncZSocket] = None
        self.ipc_address = f"/tmp/{self.robot_model}_telescopic_mast.ipc"

        # ZPipe IPC Publisher (mast control proxy → FoldableTelescopicMast)
        self.proxy_pub_socket: Optional[AsyncZSocket] = None
        self.proxy_ipc_address = f"/tmp/{self.robot_model}_telescopic_mast_proxy.ipc"

        # Mast action state tracking
        self._mast_action_state: str = "stopped"  # "raising", "lowering", "stopped"

        # move_height monitoring thread
        self._move_height_target: Optional[float] = None
        self._move_height_thread: Optional[threading.Thread] = None
        self._move_height_running = False
        self._move_height_tolerance_mm: float = 50.0  # ±50mm tolerance for target reached

    def set_zpipe_context(self, zpipe_ctx: Any):
        """Set ZPipe context and create/join IPC publish sockets (telemetry + proxy control)."""
        super().set_zpipe_context(zpipe_ctx)
        if self.zpipe_context:
            # Telemetry publisher (height data)
            try:
                socket_id = f"{self.name}_pub"
                self.pub_socket = AsyncZSocket(socket_id=socket_id, pattern="publish")
                if self.pub_socket.create(self.zpipe_context):
                    if self.pub_socket.join(transport="ipc", address=self.ipc_address):
                        logger.info(f"[{self.name}] ZPipe IPC Publisher bound to ipc://{self.ipc_address}")
            except Exception as e:
                logger.error(f"[{self.name}] Error creating ZPipe PUB socket: {e}")

            # Proxy publisher (mast control commands → FoldableTelescopicMast)
            try:
                proxy_socket_id = f"{self.name}_proxy_pub"
                self.proxy_pub_socket = AsyncZSocket(socket_id=proxy_socket_id, pattern="publish")
                if self.proxy_pub_socket.create(self.zpipe_context):
                    if self.proxy_pub_socket.join(transport="ipc", address=self.proxy_ipc_address):
                        logger.info(f"[{self.name}] ZPipe Proxy Publisher bound to ipc://{self.proxy_ipc_address}")
            except Exception as e:
                logger.error(f"[{self.name}] Error creating ZPipe Proxy PUB socket: {e}")

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

    @property
    def mast_action_state(self) -> str:
        """Current mast action state: 'raising', 'lowering', or 'stopped'."""
        with self._lock:
            return self._mast_action_state

    def set_height(self, height_mm: float):
        """Set target extension height in millimeters."""
        self.target_height_mm = height_mm

    def extend_fully(self):
        """Command mast to extend to maximum height (9100 mm)."""
        self.set_height(self.max_height)

    def retract_fully(self):
        """Command mast to retract to minimum height (2900 mm)."""
        self.set_height(self.min_height)

    # ── Mast control commands (published via proxy IPC) ─────────────────

    def _publish_mast_command(self, command: str):
        """Publish a mast control command via the proxy IPC channel."""
        # Check if HIL Simulation mode is active from DriveExecutor
        hil_active = False
        if hasattr(self, "robot") and self.robot:
            drive_exec = getattr(self.robot, "drive_executor", None)
            if drive_exec:
                hil_active = getattr(drive_exec, "hil_simulation_enabled", False)

        if hil_active:
            logger.info(f"[{self.name}] [HIL Simulation Mode] Hardware mast command '{command}' bypassed (virtual 40mm/s movement active).")
            return

        if self.proxy_pub_socket and self.proxy_pub_socket.is_joined:
            try:
                payload = json.dumps({"command": command}).encode('utf-8')
                self.proxy_pub_socket.dispatch([b"mast_command", payload])
                logger.info(f"[{self.name}] Published mast command: {command}")
            except Exception as e:
                logger.error(f"[{self.name}] Failed to publish mast command '{command}': {e}")
        else:
            logger.warning(f"[{self.name}] Proxy PUB socket not ready, cannot send command: {command}")

    def mast_up(self):
        """Send raise command to FoldableTelescopicMast via proxy IPC."""
        with self._lock:
            self._mast_action_state = "raising"
        self._publish_mast_command("raise_mast")

    def mast_down(self):
        """Send lower command to FoldableTelescopicMast via proxy IPC."""
        with self._lock:
            self._mast_action_state = "lowering"
        self._publish_mast_command("lower_mast")

    def mast_stop(self):
        """Send stop command to FoldableTelescopicMast via proxy IPC."""
        with self._lock:
            self._mast_action_state = "stopped"
        self._publish_mast_command("stop_mast_action")

    # ── High-level move functions ────────────────────────────────────────

    def move_up(self):
        """Move the mast upward (continuous until move_stop is called)."""
        self._stop_move_height_monitor()
        self.mast_up()

    def move_down(self):
        """Move the mast downward (continuous until move_stop is called)."""
        self._stop_move_height_monitor()
        self.mast_down()

    def move_stop(self):
        """Stop mast movement immediately."""
        self._stop_move_height_monitor()
        self.mast_stop()

    def move_height(self, target_height_mm: float):
        """
        Move the mast to a specific target height (mm).
        Uses a monitoring thread that periodically checks current height and
        stops the mast when the target is reached (within tolerance).

        If called again while a move is in progress, only updates the target
        height — the existing monitoring thread continues with the new target.

        Args:
            target_height_mm: Desired mast height in millimeters.
        """
        clamped = max(self.min_height, min(self.max_height, float(target_height_mm)))
        logger.info(f"[{self.name}] move_height requested: {clamped:.1f} mm")

        with self._lock:
            self._move_height_target = clamped

            # If monitoring thread is already running, just update the target
            if self._move_height_running and self._move_height_thread and self._move_height_thread.is_alive():
                logger.info(f"[{self.name}] move_height target updated to {clamped:.1f} mm (thread already running)")
                return

        # Start the monitoring thread
        self._move_height_running = True
        self._move_height_thread = threading.Thread(
            target=self._move_height_worker, daemon=True, name=f"{self.name}_move_height"
        )
        self._move_height_thread.start()

    def _move_height_worker(self):
        """Background worker that drives the mast toward _move_height_target."""
        check_interval = 0.3  # check every 300ms

        while self._move_height_running and self._running:
            with self._lock:
                target = self._move_height_target
                current = self._current_height_mm
                tolerance = self._move_height_tolerance_mm

            if target is None:
                break

            diff = target - current

            if abs(diff) <= tolerance:
                # Target reached — stop
                logger.info(
                    f"[{self.name}] move_height target reached "
                    f"(current={current:.1f}, target={target:.1f}, tol={tolerance:.1f})"
                )
                self.mast_stop()
                break

            # Decide direction
            if diff > 0:
                # Need to go up — send raise if not already raising
                if self.mast_action_state != "raising":
                    self.mast_up()
            else:
                # Need to go down — send lower if not already lowering
                if self.mast_action_state != "lowering":
                    self.mast_down()

            time.sleep(check_interval)

        with self._lock:
            self._move_height_running = False
            self._move_height_target = None
        logger.debug(f"[{self.name}] move_height worker exited.")

    def _stop_move_height_monitor(self):
        """Stop the move_height monitoring thread if running."""
        self._move_height_running = False
        if self._move_height_thread and self._move_height_thread.is_alive():
            self._move_height_thread.join(timeout=1.0)
        self._move_height_thread = None
        with self._lock:
            self._move_height_target = None



    def connect(self) -> bool:
        """Connect RS485 Modbus client and start telemetry/control thread if enabled."""
        if not self.enable:
            self.is_connected = False
            logger.info(f"[{self.name}] Device is DISABLED in config (enable=False).")
            return False

        self.is_connected = False
        if PYMODBUS_AVAILABLE and ModbusSerialClient is not None:
            try:
                self.modbus_client = ModbusSerialClient(
                    port=self.port_name,
                    baudrate=self.baudrate,
                    bytesize=8,
                    parity=self.parity,
                    stopbits=self.stopbits,
                    timeout=0.2,
                    retries=1
                )
                if self.modbus_client.connect():
                    self.is_connected = True
                    logger.info(f"[{self.name}] RS485 Modbus connected on {self.port_name} ({self.baudrate}bps, {self.parity}, {self.stopbits})")
                else:
                    logger.warning(f"[{self.name}] Could not connect Modbus client on '{self.port_name}'.")
            except Exception as e:
                logger.warning(f"[{self.name}] Failed to initialize Modbus client on '{self.port_name}': {e}.")

        # Always start worker loop for simulation/telemetry if enable=True
        self._running = True
        self._thread = threading.Thread(target=self._worker_loop, daemon=True)
        self._thread.start()
        logger.info(f"[{self.name}] Connected. Telemetry IPC: ipc://{self.ipc_address}")
        return True

    def start(self) -> bool:
        """Start device communication worker thread."""
        return self.connect()

    def disconnect(self) -> bool:
        """Disconnect device interface and stop thread."""
        self._running = False

        # Stop move_height monitor if running
        self._stop_move_height_monitor()

        if self._thread:
            self._thread.join(timeout=1.0)
            self._thread = None

        if self.modbus_client:
            try:
                self.modbus_client.close()
            except Exception:
                pass
            self.modbus_client = None

        if self.pub_socket:
            try:
                self.pub_socket.close()
            except Exception:
                pass
            self.pub_socket = None

        if self.proxy_pub_socket:
            try:
                self.proxy_pub_socket.close()
            except Exception:
                pass
            self.proxy_pub_socket = None

        self.is_connected = False
        logger.info(f"[{self.name}] Disconnected.")
        return True

    def stop(self) -> bool:
        """Stop device communication worker thread."""
        return self.disconnect()

    def _read_input_registers(self, address: int, count: int, slave: int):
        """Helper to read input registers compatible across pymodbus versions."""
        if not self.modbus_client:
            return None
        try:
            return self.modbus_client.read_input_registers(address, count=count, device_id=slave)
        except TypeError:
            try:
                return self.modbus_client.read_input_registers(address, count=count, slave=slave)
            except TypeError:
                return self.modbus_client.read_input_registers(address, count=count, unit=slave)

    def _worker_loop(self):
        """
        Background worker loop running at 100ms (0.1s) intervals:
        1. Queries RS485 Modbus device for position (Address 0x03EA, 2 input registers).
        2. Parses response registers (e.g. [0, 1800] -> 1800 mm).
        3. Publishes mast telemetry over ZPipe IPC.
        4. Logs current mast position in mm & m to console for debugging.
        """
        speed_mm_per_sec = 40.0  # Virtual 40mm/s movement speed for HIL simulation
        dt = 0.1  # 100ms interval

        while self._running:
            start_time = time.time()
            read_height_mm: Optional[float] = None

            hil_active = False
            if hasattr(self, "robot") and self.robot:
                drive_exec = getattr(self.robot, "drive_executor", None)
                if drive_exec:
                    hil_active = getattr(drive_exec, "hil_simulation_enabled", False)

            # 1. Try RS485 Modbus hardware read if pymodbus client is connected (and NOT in HIL mode)
            if not hil_active and self.modbus_client and self.is_connected:
                try:
                    rr = self._read_input_registers(address=0x03EA, count=2, slave=self.bus_address)
                    if rr is not None and not rr.isError() and hasattr(rr, 'registers') and len(rr.registers) >= 2:
                        raw_val = (rr.registers[0] << 16) | rr.registers[1]
                        read_height_mm = float(raw_val)
                except Exception as e:
                    logger.error(f"[{self.name}] Modbus read error: {e}")

            # 2. Update current position (use hardware read or 40mm/s simulated transition towards target)
            with self._lock:
                if read_height_mm is not None and not hil_active:
                    self._current_height_mm = read_height_mm
                else:
                    # Simulation motion update based on mast_action_state or target_height_mm (40mm/s)
                    if self._mast_action_state == "raising":
                        self._current_height_mm = min(self.max_height, self._current_height_mm + speed_mm_per_sec * dt)
                    elif self._mast_action_state == "lowering":
                        self._current_height_mm = max(self.min_height, self._current_height_mm - speed_mm_per_sec * dt)
                    else:
                        diff = self._target_height_mm - self._current_height_mm
                        if abs(diff) > 0.01:
                            step = (1.0 if diff > 0 else -1.0) * min(abs(diff), speed_mm_per_sec * dt)
                            self._current_height_mm += step

            # 3. Publish telemetry data over ZPipe IPC (JSON format)
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
                "stop_trig_bound_mm": self.stop_trig_bound,
                "diameter_mm": self.DIAMETER_MM,
                "port": self.port_name,
                "baudrate": self.baudrate,
                "mast_action_state": self._mast_action_state,
                "move_height_target_mm": round(self._move_height_target, 1) if self._move_height_target is not None else None,
                "move_height_active": self._move_height_running
            }


class TelescopicMast_Connector:
    """
    Connector for TelescopicMast device over ZPipe IPC.
    Subscribes to:
      - ipc:///tmp/<robot_model>_telescopic_mast.ipc  (topic: mast_data)  — height & status telemetry
    Delivers current height information together with mast action state
    ('raising', 'lowering', 'stopped') via the on_data_received callback.
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
        """Callback invoked when ZPipe receives multipart data (height + action state)."""
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
