import os
import socket
import time
import threading
import pickle
import numpy as np
from typing import Optional, Dict, Any, List, Callable
from core.device.base import BaseDevice
from core.zpipe import AsyncZSocket, ZPipe
from util.logger.console import ConsoleLogger

import ouster.sdk._bindings.client as cl
try:
    import ouster.sdk.core as client_core
except ImportError:
    try:
        import ouster.sdk.client as client_core
    except ImportError:
        import ouster.client as client_core

logger = ConsoleLogger.get_logger()


class OusterSR128(BaseDevice):
    """
    Ouster OS0 / OS1 128-channel LiDAR receiver device using official ouster-sdk bindings.
    Receives UDP packets locally and computes XYZ point clouds using saved
    sensor metadata and the official Ouster SDK.
    """

    def __init__(
        self,
        name: str = "ouster-sr-128",
        robot_model: str = "iae_patrol_v1",
        model: str = "OS0",
        bind_ip: str = "0.0.0.0",
        port: int = 7502,
        metadata_path: str = "mapping/ouster-sr-128.json",
        vertical_fov: Optional[float] = None,
        enable: bool = True,
        **kwargs
    ):
        super().__init__(name, enable=enable)
        self.robot_model = robot_model
        self.model = model.upper() if isinstance(model, str) else "OS0"
        self.bind_ip = bind_ip
        self.port = int(port)
        self.metadata_path = metadata_path
        self.ip = ""  # Last observed packet source; not needed to receive UDP.

        if vertical_fov is not None:
            self.vertical_fov = float(vertical_fov)
        else:
            if self.model == "OS1":
                self.vertical_fov = 45.0
            else:
                self.vertical_fov = 90.0

        self._sensor_info = None
        self.sock: Optional[socket.socket] = None
        self._xyz_lut: Optional[cl.XYZLut] = None
        self._pf: Optional[cl.PacketFormat] = None
        self._last_points: Optional[np.ndarray] = None  # Array of shape (N, 4) -> [x, y, z, intensity]
        self.packet_count = 0
        self.last_packet_time: Optional[float] = None

        # AsyncZSocket for publishing IPC
        self.pub_socket: Optional[AsyncZSocket] = None
        self.ipc_address = f"/tmp/{self.robot_model}_ouster_sr_128.ipc"

        # Worker thread control
        self._running = False
        self._thread: Optional[threading.Thread] = None

        self.packet_recorder: Optional[Callable[[Any], None]] = None

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
        Bind the UDP receive port and start the point cloud worker.
        """
        if not self.enable:
            self.is_connected = False
            logger.info(f"[{self.name}] Device is DISABLED in config (enable=False).")
            return False

        try:
            metadata_path = self.metadata_path
            if not os.path.isabs(metadata_path):
                project_dir = os.path.dirname(os.path.dirname(os.path.dirname(__file__)))
                metadata_path = os.path.join(project_dir, metadata_path)
            with open(metadata_path, "r", encoding="utf-8") as metadata_file:
                self._sensor_info = cl.SensorInfo(metadata_file.read())
            self._pf = cl.PacketFormat.from_info(self._sensor_info)
            self._xyz_lut = cl.XYZLut(self._sensor_info, use_extrinsics=False)
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.bind((self.bind_ip, self.port))
            self.sock.settimeout(0.1)
            self.is_connected = True
            logger.info(f"[{self.name}] Listening for Ouster UDP on {self.bind_ip}:{self.port} (SN: {self._sensor_info.sn})")

            self._start_thread()
            return True
        except Exception as e:
            self.is_connected = False
            if self.sock is not None:
                self.sock.close()
                self.sock = None
            logger.error(f"[{self.name}] Failed to open Ouster UDP receiver: {e}")
            return False

    def disconnect(self) -> bool:
        """Stop worker thread and close UDP and ZPipe sockets."""
        self._stop_thread()
        if self.sock is not None:
            self.sock.close()
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
            logger.info(f"[{self.name}] ouster-sdk worker thread started.")

    def _stop_thread(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)
            self._thread = None

    def _worker_loop(self):
        """
        Worker thread reading local UDP packets, invoking packet_recorder,
        batching packets into LidarFrame, projecting XYZ coordinates,
        and publishing processed points over ZPipe IPC.
        """
        logger.info(f"[{self.name}] ouster-sdk packet worker loop active.")
        if self.sock is None or self._sensor_info is None or self._xyz_lut is None or self._pf is None:
            return

        batcher_cls = getattr(cl, 'FrameBatcher', getattr(client_core, 'FrameBatcher', getattr(client_core, 'ScanBatcher', None)))
        batcher = batcher_cls(self._sensor_info)
        scan_cls = getattr(cl, 'LidarFrame', getattr(client_core, 'LidarFrame', getattr(client_core, 'LidarScan', None)))
        scan = scan_cls(self._sensor_info)

        while self._running:
            try:
                try:
                    raw_packet, addr = self.sock.recvfrom(65535)
                except socket.timeout:
                    continue
                recorder = getattr(self, "data_logger_packet", None)
                if recorder:
                    recorder(raw_packet, addr)
                if len(raw_packet) != self._pf.lidar_packet_size:
                    continue
                self.ip = addr[0]
                self.packet_count += 1
                self.last_packet_time = time.time()
                pkt = cl.LidarPacket(self._pf)
                pkt.buf[:] = np.frombuffer(raw_packet, dtype=np.uint8)

                # 1. Forward raw packet buffer directly to registered packet recorder
                if self.packet_recorder is not None:
                    try:
                        self.packet_recorder(pkt)
                    except Exception:
                        pass

                # 2. Batch packet into a frame for 3D visualization
                is_batch_complete = batcher.batch(pkt, scan) if hasattr(batcher, 'batch') else batcher(pkt, scan)
                if is_batch_complete:
                    # Complete frame ready -> compute point cloud
                    xyz_arr = self._xyz_lut(scan)
                    points_xyz = xyz_arr.reshape(-1, 3).astype(np.float32)

                    field_class = getattr(client_core, 'ChanField', getattr(cl, 'FieldClass', None))
                    field_refl = getattr(field_class, 'REFLECTIVITY', None) if field_class else None
                    if field_refl is not None and scan.has_field(field_refl):
                        refl_arr = scan.field(field_refl).reshape(-1, 1).astype(np.float32)
                    else:
                        refl_arr = np.zeros((len(points_xyz), 1), dtype=np.float32)

                    norm_sq = np.sum(points_xyz ** 2, axis=1)
                    valid_mask = norm_sq > 0.04

                    valid_xyz = points_xyz[valid_mask]
                    valid_refl = refl_arr[valid_mask]

                    if len(valid_xyz) > 0:
                        points_4d = np.hstack((valid_xyz, valid_refl))
                        self._publish_points(points_4d)

                    # Reset scan for next frame
                    scan = scan_cls(self._sensor_info)

            except Exception as e:
                logger.warning(f"[{self.name}] Transient error in packet iteration: {e}")
                time.sleep(0.01)

    def _publish_points(self, points: np.ndarray):
        """Publish sensor-relative points over ZPipe IPC socket."""
        self._last_points = points
        if self.pub_socket and self.pub_socket.is_joined and points is not None and len(points) > 0:
            try:
                payload = pickle.dumps(points)
                self.pub_socket.dispatch([b"ouster_points", payload])
            except Exception as e:
                logger.error(f"[{self.name}] Publish error: {e}")

    def get_status(self) -> Dict[str, Any]:
        return {
            "name": self.name,
            "model": self.model,
            "vertical_fov": self.vertical_fov,
            "ip": self.ip,
            "bind_ip": self.bind_ip,
            "port": self.port,
            "connected": self.is_connected,
            "last_packet_time": self.last_packet_time,
            "ipc_address": self.ipc_address,
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
