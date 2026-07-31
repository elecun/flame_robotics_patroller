"""
Velodyne VLP-16 16-channel LiDAR device driver and Connector module.
Receives 16-channel Velodyne LiDAR data via UDP socket, parses raw point cloud data,
filters specified azimuth angle range, and publishes point cloud data via ZPipe AsyncZSocket (IPC pub/sub).
Also provides VLP16_Connector for asynchronous reception.
"""

import socket
import struct
import math
import time
import threading
import pickle
import numpy as np
from typing import Optional, Tuple, Dict, Any, List, Callable
from core.device.base import BaseDevice
from core.zpipe import AsyncZSocket, ZPipe
from util.logger.console import ConsoleLogger

logger = ConsoleLogger.get_logger()



class VLP16(BaseDevice):
    """
    Velodyne VLP-16 16-channel LiDAR device module.
    Runs in a dedicated thread to receive UDP packets, parse & filter point cloud,
    and publish processed data over ZPipe IPC AsyncZSocket (PUB pattern).
    """

    # VLP-16 factory vertical angles in degrees for laser IDs 0..15
    VERTICAL_ANGLES = [
        -15.0, 1.0, -13.0, 3.0, -11.0, 5.0, -9.0, 7.0,
        -7.0, 9.0, -5.0, 11.0, -3.0, 13.0, -1.0, 15.0
    ]

    def __init__(
        self,
        name: str = "vlp-16",
        robot_model: str = "iae_patrol_v1",
        ip: str = "192.168.100.10",
        port: int = 2368,
        min_angle: float = -90.0,
        max_angle: float = 90.0,
        offset_x: float = 1.027,
        offset_y: float = 0.0,
        offset_z: float = 0.32,
        roll_deg: float = 0.0,
        pitch_deg: float = 15.0,
        yaw_deg: float = 0.0,
        enable: bool = True
    ):
        super().__init__(name, enable=enable)
        self.robot_model = robot_model
        self.ip = ip
        self.port = int(port)
        self.min_angle = float(min_angle)
        self.max_angle = float(max_angle)

        # Installation offset relative to robot origin frame (meters)
        self.offset_x = float(offset_x)
        self.offset_y = float(offset_y)
        self.offset_z = float(offset_z)

        # Installation orientation relative to robot origin frame (degrees)
        self.roll_deg = float(roll_deg)
        self.pitch_deg = float(pitch_deg)
        self.yaw_deg = float(yaw_deg)

        # Compute 3x3 rotation matrix from Roll, Pitch, Yaw
        self.R = self._compute_rotation_matrix(self.roll_deg, self.pitch_deg, self.yaw_deg)

        self.sock: Optional[socket.socket] = None
        self._last_points: Optional[np.ndarray] = None  # Array of shape (N, 4) -> [x, y, z, intensity]

        # AsyncZSocket for publishing IPC
        self.pub_socket: Optional[AsyncZSocket] = None
        self.ipc_address = f"/tmp/{self.robot_model}_vlp_16.ipc"

        # Worker thread control
        self._running = False
        self._thread: Optional[threading.Thread] = None

    def _compute_rotation_matrix(self, roll_deg: float, pitch_deg: float, yaw_deg: float) -> np.ndarray:
        """Compute 3x3 rotation matrix R = Rz(yaw) * Ry(pitch) * Rx(roll)."""
        r = np.radians(roll_deg)
        # Note: Since Y-axis is inverted (Left/Right mirroring), Y-axis rotation (Pitch down) direction is inverted (-p)
        p = -np.radians(pitch_deg)
        y = np.radians(yaw_deg)

        Rx = np.array([
            [1, 0, 0],
            [0, np.cos(r), -np.sin(r)],
            [0, np.sin(r), np.cos(r)]
        ], dtype=np.float32)

        Ry = np.array([
            [np.cos(p), 0, np.sin(p)],
            [0, 1, 0],
            [-np.sin(p), 0, np.cos(p)]
        ], dtype=np.float32)

        Rz = np.array([
            [np.cos(y), -np.sin(y), 0],
            [np.sin(y), np.cos(y), 0],
            [0, 0, 1]
        ], dtype=np.float32)

        return Rz @ Ry @ Rx

    def transform_points_to_robot_frame(self, points: np.ndarray) -> np.ndarray:
        """Return raw sensor-relative point cloud. Transformations are handled by URDF frame links."""
        if points is None or len(points) == 0:
            return np.empty((0, 4), dtype=np.float32)

        return points

    def set_zpipe_context(self, zpipe_ctx: Any):
        """Set ZPipe context and create/join IPC publish socket."""
        super().set_zpipe_context(zpipe_ctx)
        if self.zpipe_context:
            try:
                socket_id = f"{self.name}_pub"
                self.pub_socket = AsyncZSocket(socket_id=socket_id, pattern="publish")
                if self.pub_socket.create(self.zpipe_context):
                    # Bind PUB socket to IPC address
                    if self.pub_socket.join(transport="ipc", address=self.ipc_address):
                        logger.info(f"[{self.name}] ZPipe IPC Publisher bound to ipc://{self.ipc_address}")
            except Exception as e:
                logger.error(f"[{self.name}] Error creating ZPipe PUB socket: {e}")

    def set_angle_filter(self, min_angle: float, max_angle: float):
        """Set azimuth angle filtering range in degrees."""
        self.min_angle = float(min_angle)
        self.max_angle = float(max_angle)

    def connect(self) -> bool:
        """
        Open UDP socket on specified port and start worker loop thread if enabled.
        """
        if not self.enable:
            self.is_connected = False
            logger.info(f"[{self.name}] Device is DISABLED in config (enable=False).")
            return False

        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.sock.bind(("", self.port))
            self.sock.settimeout(1.0)
            self.is_connected = True
            logger.info(f"[{self.name}] Listening for VLP-16 data on UDP port {self.port} (Expected IP: {self.ip})")

            # Start worker thread
            self._start_thread()
            return True
        except Exception as e:
            self.is_connected = False
            self.sock = None
            logger.error(f"[{self.name}] Failed to open UDP socket: {e}")
            return False

    def disconnect(self) -> bool:
        """Stop background thread and close UDP socket & ZPipe socket."""
        self._stop_thread()
        if self.sock:
            try:
                self.sock.close()
            except Exception:
                pass
            self.sock = None
        if self.pub_socket:
            try:
                self.pub_socket.close()
            except Exception:
                pass
            self.pub_socket = None
        self.is_connected = False
        logger.info(f"[{self.name}] Disconnected.")
        return True

    def _start_thread(self):
        if not self._running:
            self._running = True
            self._thread = threading.Thread(target=self._worker_loop, daemon=True)
            self._thread.start()
            logger.info(f"[{self.name}] Background processing thread started.")

    def _stop_thread(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)
            self._thread = None

    def _worker_loop(self):
        """
        Worker thread loop: collects UDP packets over one complete 360-degree revolution cycle,
        parses & filters point cloud, and publishes the full revolution scan via ZPipe IPC.
        If no hardware packets are received (timeout), generates a full 360-degree simulated revolution.
        """
        logger.info(f"[{self.name}] Worker loop active (Revolution cycle mode).")
        last_azimuth = -1
        cycle_blocks = []

        while self._running:
            if not self.is_connected or not self.sock:
                time.sleep(0.05)
                continue

            try:
                packet, addr = self.sock.recvfrom(2048)
                if len(packet) == 1206:
                    # Check first block azimuth (0.01 deg units) for revolution wrap-around detection
                    azimuth_raw = struct.unpack_from("<H", packet, 2)[0]
                    
                    if last_azimuth >= 0 and azimuth_raw < last_azimuth:
                        # Revolution boundary detected: process complete revolution scan
                        if cycle_blocks:
                            points = self._parse_cycle_blocks(cycle_blocks)
                            if points is not None and len(points) > 0:
                                self._publish_points(points)
                            cycle_blocks = []

                    last_azimuth = azimuth_raw
                    cycle_blocks.append(packet)

            except socket.timeout:
                # Hardware timeout: emit simulated full revolution scan
                simulated_points = self._generate_simulated_points()
                self._publish_points(simulated_points)
                time.sleep(0.1)

    def _publish_points(self, raw_points: np.ndarray):
        """Transform points to robot center origin frame and publish full scan points over ZPipe IPC."""
        points = self.transform_points_to_robot_frame(raw_points)
        self._last_points = points
        if self.pub_socket and self.pub_socket.is_joined and len(points) > 0:
            try:
                payload = pickle.dumps(points)
                self.pub_socket.dispatch([b"vlp16_points", payload])
            except Exception as e:
                logger.error(f"[{self.name}] Publish error: {e}")

    def filter_points(self, points: np.ndarray) -> np.ndarray:
        """
        Filter point cloud array based on horizontal (azimuth) angle [min_angle, max_angle] in degrees.
        Horizontal azimuth is evaluated in degrees [-180, 180].
        """
        if points is None or len(points) == 0:
            return np.empty((0, 4), dtype=np.float32)

        # Compute horizontal azimuth angle in degrees [-180, 180] in sensor local frame (arctan2(Y, X))
        azimuth_deg = np.degrees(np.arctan2(points[:, 1], points[:, 0]))

        if self.min_angle <= self.max_angle:
            mask = (azimuth_deg >= self.min_angle) & (azimuth_deg <= self.max_angle)
        else:
            mask = (azimuth_deg >= self.min_angle) | (azimuth_deg <= self.max_angle)

        return points[mask]

    def _parse_cycle_blocks(self, packet_list: List[bytes]) -> np.ndarray:
        """
        Parse accumulated 1206-byte VLP-16 UDP packets into a complete 360-degree revolution point cloud array.
        Coordinate mapping for Viser (ROS/ISO Standard LiDAR frame):
          X_viser = distance * cos(vertical) * sin(azimuth)    [Right / Left]
          Y_viser = distance * sin(vertical)                   [Up / Down]
          Z_viser = distance * cos(vertical) * cos(azimuth)    [Front / Forward]
        """
        points = []
        vert_rad = np.radians(self.VERTICAL_ANGLES)

        for packet in packet_list:
            if len(packet) != 1206:
                continue

            for block_idx in range(12):
                offset = block_idx * 100
                flag, azimuth_raw = struct.unpack_from("<HH", packet, offset)
                if flag != 0xEEFF:
                    continue

                azimuth_deg = azimuth_raw / 100.0
                # Normalize azimuth angle to [-180, 180] range
                norm_azimuth_deg = (azimuth_deg + 180.0) % 360.0 - 180.0

                # Horizontal azimuth angle filtering check [-90° to +90°]
                if self.min_angle <= self.max_angle:
                    if not (self.min_angle <= norm_azimuth_deg <= self.max_angle):
                        continue
                else:
                    if not (norm_azimuth_deg >= self.min_angle or norm_azimuth_deg <= self.max_angle):
                        continue

                azimuth_rad = math.radians(azimuth_deg)

                for channel in range(32):
                    channel_offset = offset + 4 + channel * 3
                    distance_raw, intensity = struct.unpack_from("<HB", packet, channel_offset)
                    distance_m = distance_raw * 0.002  # 2mm resolution

                    if distance_m <= 0.1 or distance_m > 100.0:
                        continue

                    laser_id = channel % 16
                    omega = vert_rad[laser_id]

                    # Velodyne Sensor Frame with inverted Y-axis alignment:
                    # X_sensor = distance * cos(omega) * cos(azimuth)    [Forward / Front]
                    # Y_sensor = -distance * cos(omega) * sin(azimuth)   [Inverted Y-axis alignment]
                    # Z_sensor = distance * sin(omega)                   [Up]
                    xy = distance_m * math.cos(omega)
                    x_sensor = xy * math.cos(azimuth_rad)
                    y_sensor = -xy * math.sin(azimuth_rad)
                    z_sensor = distance_m * math.sin(omega)

                    points.append([x_sensor, y_sensor, z_sensor, float(intensity)])

        if not points:
            return np.empty((0, 4), dtype=np.float32)

        return np.array(points, dtype=np.float32)

    def parse_packet(self, packet: bytes) -> np.ndarray:
        """Parse a single packet into point cloud array."""
        return self._parse_cycle_blocks([packet])

    def receive_and_filter(self) -> np.ndarray:
        """Receive packet from UDP socket and parse/filter points."""
        if not self.is_connected or not self.sock:
            return np.empty((0, 4), dtype=np.float32)

        try:
            packet, addr = self.sock.recvfrom(2048)
            return self.parse_packet(packet)
        except socket.timeout:
            return np.empty((0, 4), dtype=np.float32)
        except Exception:
            return np.empty((0, 4), dtype=np.float32)

    def _generate_simulated_points(self) -> np.ndarray:
        """Generate simulated 16-channel 360-degree revolution point cloud."""
        num_points = 1600
        min_rad = np.radians(self.min_angle)
        max_rad = np.radians(self.max_angle)
        angles = np.linspace(min_rad, max_rad, num_points)
        distances = 3.0 + 1.5 * np.cos(angles * 2)

        points = []
        vert_rad = np.radians(self.VERTICAL_ANGLES)

        for i, angle in enumerate(angles):
            dist = distances[i]
            # Cycle through 16 vertical laser channels
            omega = vert_rad[i % 16]
            xy = dist * math.cos(omega)
            x_sensor = xy * math.cos(angle)
            y_sensor = -xy * math.sin(angle)
            z_sensor = dist * math.sin(omega)

            intensity = float(100 + (i % 155))

            points.append([x_sensor, y_sensor, z_sensor, intensity])

        return np.array(points, dtype=np.float32)

    def get_status(self) -> Dict[str, Any]:
        return {
            "name": self.name,
            "ip": self.ip,
            "port": self.port,
            "connected": self.is_connected,
            "min_angle": self.min_angle,
            "max_angle": self.max_angle,
            "ipc_address": self.ipc_address,
            "last_point_count": len(self._last_points) if self._last_points is not None else 0
        }


class VLP16_Connector:
    """
    Asynchronous receiver connector for VLP-16 published point cloud data over ZPipe IPC.
    Connects to SUB socket at ipc:///tmp/<robot_model>_vlp_16.ipc.
    """

    def __init__(
        self,
        robot_model: str = "iae_patrol_v1",
        zpipe_ctx: Optional[ZPipe] = None,
        on_data_received: Optional[Callable[[np.ndarray], None]] = None
    ):
        self.robot_model = robot_model
        self.zpipe_ctx = zpipe_ctx or ZPipe.create_pipe()
        self.on_data_received = on_data_received

        self.ipc_address = f"/tmp/{self.robot_model}_vlp_16.ipc"
        self.sub_socket: Optional[AsyncZSocket] = None
        self.last_points: Optional[np.ndarray] = None

    def start(self) -> bool:
        """Create SUB AsyncZSocket, connect to IPC publisher, and register message callback."""
        try:
            socket_id = f"vlp16_sub_{int(time.time() * 1000)}"
            self.sub_socket = AsyncZSocket(socket_id=socket_id, pattern="subscribe")
            if not self.sub_socket.create(self.zpipe_ctx):
                return False

            self.sub_socket.set_message_callback(self._on_multipart_received)
            if self.sub_socket.join(transport="ipc", address=self.ipc_address):
                self.sub_socket.subscribe(b"vlp16_points")
                logger.info(f"[VLP16_Connector] Subscribed to ZPipe IPC at ipc://{self.ipc_address}")
                return True
            return False
        except Exception as e:
            logger.error(f"[VLP16_Connector] Failed to connect SUB socket: {e}")
            return False

    def stop(self):
        """Close SUB socket."""
        if self.sub_socket:
            try:
                self.sub_socket.close()
            except Exception:
                pass
            self.sub_socket = None
        logger.info("[VLP16_Connector] Stopped.")

    def _on_multipart_received(self, multipart_data: List[bytes]):
        """Callback invoked when ZPipe receives multipart data."""
        if len(multipart_data) >= 2:
            topic = multipart_data[0]
            if topic == b"vlp16_points":
                try:
                    points = pickle.loads(multipart_data[1])
                    self.last_points = points
                    if self.on_data_received:
                        self.on_data_received(points)
                except Exception as e:
                    logger.error(f"[VLP16_Connector] Error unpickling points: {e}")
