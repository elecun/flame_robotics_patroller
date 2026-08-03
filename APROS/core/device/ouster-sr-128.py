import socket
import time
import threading
import pickle
import numpy as np
from typing import Optional, Tuple, Dict, Any, List, Callable
from core.device.base import BaseDevice
from core.zpipe import AsyncZSocket, ZPipe
from util.logger.console import ConsoleLogger

import ouster.sdk._bindings.client as cl

logger = ConsoleLogger.get_logger()


class OusterSR128(BaseDevice):
    """
    Ouster OS0 / OS1 128-channel LiDAR receiver device using official ouster-sdk bindings.
    Connects to live sensor via cl.Sensor & cl.SensorFrameSetSource and computes
    XYZ Cartesian point cloud using official intrinsic cl.XYZLut.
    """

    def __init__(
        self,
        name: str = "ouster-sr-128",
        robot_model: str = "iae_patrol_v1",
        model: str = "OS0",
        ip: str = "192.168.100.12",
        port: int = 7502,
        min_angle: float = -90.0,
        max_angle: float = 90.0,
        vertical_fov: Optional[float] = None,
        status_monitor: Optional[Any] = None,
        enable: bool = True
    ):
        super().__init__(name, enable=enable, status_monitor=status_monitor)
        self.robot_model = robot_model
        self.model = model.upper() if isinstance(model, str) else "OS0"
        self.ip = ip
        self.port = int(port)
        self.min_angle = float(min_angle)
        self.max_angle = float(max_angle)

        if vertical_fov is not None:
            self.vertical_fov = float(vertical_fov)
        else:
            if self.model == "OS1":
                self.vertical_fov = 45.0
            else:
                self.vertical_fov = 90.0

        self._sensor: Optional[cl.Sensor] = None
        self._source: Optional[cl.SensorFrameSetSource] = None
        self._xyz_lut: Optional[cl.XYZLut] = None
        self._last_points: Optional[np.ndarray] = None  # Array of shape (N, 4) -> [x, y, z, intensity]

        # AsyncZSocket for publishing IPC
        self.pub_socket: Optional[AsyncZSocket] = None
        self.ipc_address = f"/tmp/{self.robot_model}_ouster_sr_128.ipc"

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

    def set_angle_filter(self, min_angle: float, max_angle: float):
        """Set azimuth angle filtering range in degrees."""
        self.min_angle = float(min_angle)
        self.max_angle = float(max_angle)

    def connect(self) -> bool:
        """
        Connect to live Ouster sensor via ouster-sdk bindings and start background thread.
        """
        if not self.enable:
            self.is_connected = False
            logger.info(f"[{self.name}] Device is DISABLED in config (enable=False).")
            return False

        try:
            logger.info(f"[{self.name}] Connecting to live Ouster LiDAR at {self.ip} via ouster-sdk...")
            self._sensor = cl.Sensor(self.ip)
            sensor_info = self._sensor.fetch_metadata(timeout=10)
            
            self._source = cl.SensorFrameSetSource([self._sensor], [sensor_info], config_timeout=10.0)
            self._xyz_lut = cl.XYZLut(sensor_info, use_extrinsics=False)
            
            self.is_connected = True
            logger.info(f"[{self.name}] Connected to Ouster LiDAR (SN: {sensor_info.sn}, Mode: {sensor_info.prod_line}) via ouster-sdk")

            self._start_thread()
            return True
        except Exception as e:
            self.is_connected = False
            self._source = None
            self._sensor = None
            logger.error(f"[{self.name}] Failed to connect via ouster-sdk: {e}")
            return False

    def disconnect(self) -> bool:
        """Stop worker thread and close ouster-sdk source & ZPipe socket."""
        self._stop_thread()
        if self._source is not None:
            try:
                self._source.close()
            except Exception:
                pass
            self._source = None
        self._sensor = None
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
            logger.info(f"[{self.name}] ouster-sdk worker thread started.")

    def _stop_thread(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)
            self._thread = None

    def _worker_loop(self):
        """
        Worker thread reading FrameSet / LidarFrame from ouster-sdk source,
        projecting XYZ coordinates via official XYZLut, applying azimuth filter,
        and publishing over ZPipe IPC.
        """
        logger.info(f"[{self.name}] ouster-sdk worker loop active.")
        if self._source is None or self._xyz_lut is None:
            return

        try:
            for frame_set in self._source:
                if not self._running:
                    break

                lidar_frames = frame_set.valid_frames()
                if not lidar_frames:
                    continue

                frame = lidar_frames[0]
                
                # Compute XYZ Cartesian point cloud array (H, W, 3) using official XYZLut
                xyz_arr = self._xyz_lut(frame)
                
                # Reshape (H, W, 3) -> (N, 3)
                points_xyz = xyz_arr.reshape(-1, 3).astype(np.float32)

                # Extract REFLECTIVITY field if available
                if frame.has_field("REFLECTIVITY"):
                    refl_arr = frame.field("REFLECTIVITY").reshape(-1, 1).astype(np.float32)
                else:
                    refl_arr = np.zeros((len(points_xyz), 1), dtype=np.float32)

                # Filter out zero / invalid range points where (x, y, z) == (0, 0, 0)
                norm_sq = np.sum(points_xyz ** 2, axis=1)
                valid_mask = norm_sq > 0.04  # Filter out points closer than 0.2m (0.2^2 = 0.04)

                valid_xyz = points_xyz[valid_mask]
                valid_refl = refl_arr[valid_mask]

                if len(valid_xyz) == 0:
                    continue

                points_4d = np.hstack((valid_xyz, valid_refl))
                filtered_points = self.filter_points(points_4d)

                if len(filtered_points) > 0:
                    self._publish_points(filtered_points)

        except Exception as e:
            logger.error(f"[{self.name}] Error in ouster-sdk worker loop: {e}")

    def _publish_points(self, points: np.ndarray):
        """Publish sensor-relative points over ZPipe IPC socket."""
        self._last_points = points
        if self.pub_socket and self.pub_socket.is_joined and points is not None and len(points) > 0:
            try:
                payload = pickle.dumps(points)
                self.pub_socket.dispatch([b"ouster_points", payload])
            except Exception as e:
                logger.error(f"[{self.name}] Publish error: {e}")

    def filter_points(self, points: np.ndarray) -> np.ndarray:
        """
        Filter point cloud array (N, 3 or N, 4) based on azimuth angle [min_angle, max_angle] in degrees.
        Azimuth angle is calculated as arctan2(y, x) in degrees, normalized to [-180, 180].
        """
        if points is None or len(points) == 0:
            return np.empty((0, 4), dtype=np.float32)

        # Compute azimuth in degrees [-180, 180]
        azimuth_deg = np.degrees(np.arctan2(points[:, 1], points[:, 0]))

        if self.min_angle <= self.max_angle:
            mask = (azimuth_deg >= self.min_angle) & (azimuth_deg <= self.max_angle)
        else:
            # Wrapped angle range across +/-180 deg boundary
            mask = (azimuth_deg >= self.min_angle) | (azimuth_deg <= self.max_angle)

        return points[mask]

    def receive_and_filter(self) -> np.ndarray:
        """
        Receive packet from socket, parse raw points, apply angle filter, and return filtered points.
        """
        if not self.is_connected or not self.sock:
            return np.empty((0, 4), dtype=np.float32)

        try:
            packet, addr = self.sock.recvfrom(65535)

            raw_points = self.parse_packet(packet)
            filtered_points = self.filter_points(raw_points)
            self._last_points = filtered_points
            return filtered_points

        except socket.timeout:
            return np.empty((0, 4), dtype=np.float32)
        except Exception as e:
            logger.error(f"[{self.name}] Error receiving packet: {e}")
            return np.empty((0, 4), dtype=np.float32)

    def get_status(self) -> Dict[str, Any]:
        return {
            "name": self.name,
            "model": self.model,
            "vertical_fov": self.vertical_fov,
            "ip": self.ip,
            "port": self.port,
            "connected": self.is_connected,
            "min_angle": self.min_angle,
            "max_angle": self.max_angle,
            "ipc_address": self.ipc_address,
            "last_point_count": len(self._last_points) if self._last_points is not None else 0
        }


class OusterSR128_Connector:
    """
    Asynchronous receiver connector for Ouster-SR-128 published point cloud data over ZPipe IPC.
    Connects to SUB socket at ipc:///tmp/<robot_model>_ouster_sr_128.ipc.
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

        self.ipc_address = f"/tmp/{self.robot_model}_ouster_sr_128.ipc"
        self.sub_socket: Optional[AsyncZSocket] = None
        self.last_points: Optional[np.ndarray] = None

    def start(self) -> bool:
        """Create SUB AsyncZSocket, connect to IPC publisher, and register message callback."""
        try:
            socket_id = f"ouster_sub_{int(time.time() * 1000)}"
            self.sub_socket = AsyncZSocket(socket_id=socket_id, pattern="subscribe")
            if not self.sub_socket.create(self.zpipe_ctx):
                return False

            self.sub_socket.set_message_callback(self._on_multipart_received)
            if self.sub_socket.join(transport="ipc", address=self.ipc_address):
                self.sub_socket.subscribe(b"ouster_points")
                logger.info(f"[OusterSR128_Connector] Subscribed to ZPipe IPC at ipc://{self.ipc_address}")
                return True
            return False
        except Exception as e:
            logger.error(f"[OusterSR128_Connector] Failed to connect SUB socket: {e}")
            return False

    def stop(self):
        """Close SUB socket."""
        if self.sub_socket:
            try:
                self.sub_socket.close()
            except Exception:
                pass
            self.sub_socket = None
        logger.info("[OusterSR128_Connector] Stopped.")

    def _on_multipart_received(self, multipart_data: List[bytes]):
        """Callback invoked when ZPipe receives multipart data."""
        if len(multipart_data) >= 2:
            topic = multipart_data[0]
            if topic == b"ouster_points":
                try:
                    points = pickle.loads(multipart_data[1])
                    self.last_points = points
                    if self.on_data_received:
                        self.on_data_received(points)
                except Exception as e:
                    logger.error(f"[OusterSR128_Connector] Error unpickling points: {e}")
