"""
Baumer GIM700DR 2-axis CANopen Inclination Sensor Device Module (core/device/baumer_incline.py)
Receives 2-axis inclination data via CANopen (PDO1 0x180 + node_id) on CAN Channel 1 at 500k bitrate,
parses X-axis (slope_z * 0.1 deg) and Z-axis (slope_y * 0.1 deg) tilt angles,
publishes tilt status over ZPipe AsyncZSocket (IPC pub/sub), and provides BaumerIncline_Connector for subscriber reception.
"""

import time
import threading
import json
import struct
from typing import Optional, Dict, Any, List, Callable
from core.device.base import BaseDevice
from core.zpipe import AsyncZSocket, ZPipe
from util.logger.console import ConsoleLogger

logger = ConsoleLogger.get_logger()

try:
    from canlib import canlib, Frame
    CANLIB_AVAILABLE = True
except (ImportError, Exception, BaseException) as e:
    CANLIB_AVAILABLE = False
    canlib = None
    Frame = None
    logger.error(f"[BaumerIncline] CANlib (libcanlib.so/dll) is unavailable on this system ({e}). Disabling CAN hardware interface.")




class BaumerIncline(BaseDevice):
    """
    Baumer GIM700DR 2-axis inclination sensor device module.
    Runs in a dedicated thread to listen for CANopen PDO frames, parse tilt angles,
    and publish human-readable JSON status over ZPipe IPC AsyncZSocket (PUB pattern).
    """

    def __init__(
        self,
        name: str = "baumer_incline",
        robot_model: str = "iae_patrol_v1",
        can_channel: int = 1,
        can_bitrate: int = 500000,
        node_id: int = 1,
        status_monitor: Optional[Any] = None,
        enable: bool = True
    ):
        super().__init__(name, enable=enable, status_monitor=status_monitor)
        self.robot_model = robot_model
        self.channel = int(can_channel)
        self.bitrate = int(can_bitrate)
        self.node_id = int(node_id)

        self.ch = None
        self.tilt_x_deg = 0.0  # X-axis rotation angle in degrees (corresponds to slope_z * 0.1 deg)
        self.tilt_z_deg = 0.0  # Z-axis rotation angle in degrees (corresponds to slope_y * 0.1 deg)
        self.temperature_degC = 0.0

        # AsyncZSocket for publishing IPC
        self.pub_socket: Optional[AsyncZSocket] = None
        self.ipc_address = f"/tmp/{self.robot_model}_baumer_incline.ipc"

        # Worker thread control
        self._running = False
        self._thread: Optional[threading.Thread] = None

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

    def connect(self) -> bool:
        """
        Open Kvaser CANlib channel 1 (500k), send CANopen NMT start remote node, and start RX thread.
        """
        if not self.enable:
            self.is_connected = False
            logger.info(f"[{self.name}] Device is DISABLED in config (enable=False).")
            return False

        if not CANLIB_AVAILABLE or canlib is None:
            self.is_connected = False
            self.ch = None
            logger.warning(f"[{self.name}] CANlib is not available on this system. Operating without physical CAN hardware.")
            return False
        try:

            self.ch = canlib.openChannel(self.channel, flags=canlib.Open.ACCEPT_VIRTUAL)
            self.ch.setBusParams(canlib.Bitrate.BITRATE_500K)
            self.ch.busOn()
            self.is_connected = True
            logger.info(f"[{self.name}] Connected to Kvaser CANlib Channel {self.channel} (500k).")
        except canlib.CanError:
            try:
                self.ch = canlib.openChannel(self.channel)
                self.ch.setBusParams(canlib.Bitrate.BITRATE_500K)
                self.ch.busOn()
                self.is_connected = True
                logger.info(f"[{self.name}] Connected to Kvaser CANlib Channel {self.channel} (500k).")
            except Exception as ex:
                self.is_connected = False
                self.ch = None
                logger.error(f"[{self.name}] Failed to connect to CAN Channel {self.channel}: {ex}")

        self._send_nmt_start_remote_node()
        self._start_thread()
        return True

    def disconnect(self) -> bool:
        """Stop background worker thread and close CAN & ZPipe sockets."""
        self._stop_thread()
        if self.ch is not None:
            try:
                self.ch.busOff()
                self.ch.close()
            except Exception:
                pass
            self.ch = None
        if self.pub_socket:
            try:
                self.pub_socket.close()
            except Exception:
                pass
            self.pub_socket = None
        self.is_connected = False
        logger.info(f"[{self.name}] Disconnected.")
        return True

    def _send_nmt_start_remote_node(self):
        """Send CANopen NMT Start Remote Node command (COB-ID 0x000, Data [0x01, node_id])."""
        if self.ch is not None and self.is_connected:
            try:
                payload = bytearray([0x01, self.node_id & 0xFF])
                frame = Frame(id_=0x000, data=payload)
                self.ch.write(frame)
                logger.info(f"[{self.name}] Sent CANopen NMT Start command to Node ID {self.node_id}.")
            except Exception as e:
                logger.error(f"[{self.name}] Failed to send NMT command: {e}")

    def _start_thread(self):
        if not self._running:
            self._running = True
            self._thread = threading.Thread(target=self._worker_loop, daemon=True)
            self._thread.start()
            logger.info(f"[{self.name}] RX Background worker thread started.")

    def _stop_thread(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)
            self._thread = None

    def _worker_loop(self):
        """
        Worker thread loop: read incoming CAN frames (PDO1: 0x180 + node_id),
        parse temperature in ℃, slope_z (X-axis tilt in deg), slope_y (Z-axis tilt in deg),
        and publish formatted JSON payload over ZPipe IPC.
        """
        target_cob_id = 0x180 + self.node_id
        resolution = 0.1

        while self._running:
            got_data = False
            if self.is_connected and self.ch is not None:
                try:
                    frame = self.ch.read(timeout=100)
                    if frame.id == target_cob_id and len(frame.data) >= 6:
                        # Data layout:
                        # bytes 0-1: int16 temperature (°C)
                        # bytes 2-3: int16 slope_z (X-axis tilt, 0.1 deg/LSB)
                        # bytes 4-5: int16 slope_y (Z-axis tilt, 0.1 deg/LSB)
                        temp_raw = int.from_bytes(frame.data[0:2], 'little', signed=True)
                        slope_z_raw = int.from_bytes(frame.data[2:4], 'little', signed=True)
                        slope_y_raw = int.from_bytes(frame.data[4:6], 'little', signed=True)

                        self.temperature_degC = float(temp_raw)
                        self.tilt_x_deg = round(float(slope_z_raw) * resolution, 2)
                        self.tilt_z_deg = round(float(slope_y_raw) * resolution, 2)
                        got_data = True
                except canlib.CanNoMsg:
                    pass
                except Exception:
                    pass

            if got_data:
                # Publish tilt status JSON payload over ZPipe IPC ONLY when physical CAN frame is received
                self._publish_status()
            else:
                time.sleep(0.05)

    def _publish_status(self):
        """Publish status dictionary serialized as JSON string over ZPipe IPC AsyncZSocket."""
        if self.pub_socket and self.pub_socket.is_joined:
            try:
                status_data = self.get_status()
                json_payload = json.dumps(status_data, ensure_ascii=False).encode('utf-8')
                self.pub_socket.dispatch([b"baumer_incline_data", json_payload])
            except Exception as e:
                logger.error(f"[{self.name}] Publish JSON error: {e}")

    def get_status(self) -> Dict[str, Any]:
        """Return status dictionary with explicit degree (deg) and Celsius (°C) units."""
        return {
            "name": self.name,
            "channel": self.channel,
            "connected": self.is_connected,
            "node_id": self.node_id,
            "tilt_x": self.tilt_x_deg,
            "tilt_z": self.tilt_z_deg,
            "temperature": self.temperature_degC,
            "unit_tilt": "deg",
            "unit_temp": "degC",
            "ipc_address": self.ipc_address
        }


class BaumerIncline_Connector:
    """
    Asynchronous receiver connector for Baumer GIM700DR published inclination data over ZPipe IPC.
    Connects to SUB socket at ipc:///tmp/<robot_model>_baumer_incline.ipc and parses JSON payloads.
    """

    def __init__(
        self,
        robot_model: str = "iae_patrol_v1",
        zpipe_ctx: Optional[ZPipe] = None,
        on_data_received: Optional[Callable[[Dict[str, Any]], None]] = None
    ):
        self.robot_model = robot_model
        self.zpipe_ctx = zpipe_ctx or ZPipe.create_pipe()
        self.on_data_received = on_data_received

        self.ipc_address = f"/tmp/{self.robot_model}_baumer_incline.ipc"
        self.sub_socket: Optional[AsyncZSocket] = None
        self.last_status: Optional[Dict[str, Any]] = None

    def start(self) -> bool:
        """Create SUB AsyncZSocket, connect to IPC publisher, and register message callback."""
        try:
            socket_id = f"baumer_sub_{int(time.time() * 1000)}"
            self.sub_socket = AsyncZSocket(socket_id=socket_id, pattern="subscribe")
            if not self.sub_socket.create(self.zpipe_ctx):
                return False

            self.sub_socket.set_message_callback(self._on_multipart_received)
            if self.sub_socket.join(transport="ipc", address=self.ipc_address):
                self.sub_socket.subscribe(b"baumer_incline_data")
                logger.info(f"[BaumerIncline_Connector] Subscribed to ZPipe IPC at ipc://{self.ipc_address}")
                return True
            return False
        except Exception as e:
            logger.error(f"[BaumerIncline_Connector] Failed to connect SUB socket: {e}")
            return False

    def stop(self):
        """Close SUB socket."""
        if self.sub_socket:
            try:
                self.sub_socket.close()
            except Exception:
                pass
            self.sub_socket = None
        logger.info("[BaumerIncline_Connector] Stopped.")

    def _on_multipart_received(self, multipart_data: List[bytes]):
        """Callback invoked when ZPipe receives JSON multipart data."""
        if len(multipart_data) >= 2:
            topic = multipart_data[0]
            if topic == b"baumer_incline_data":
                try:
                    json_str = multipart_data[1].decode('utf-8')
                    status_dict = json.loads(json_str)
                    self.last_status = status_dict
                    if self.on_data_received:
                        self.on_data_received(status_dict)
                except Exception as e:
                    logger.error(f"[BaumerIncline_Connector] Error parsing JSON inclination data: {e}")
