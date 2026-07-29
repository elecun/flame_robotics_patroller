"""
Synerex RTK GPS Device Driver and WebSocket Server Module (core/device/synerex_rtk.py)
Receives Synerex RTK GNSS latitude, longitude, altitude, and heading data,
publishes RTK telemetry via ZPipe AsyncZSocket (IPC PUB/SUB),
and runs a WebSocket server to broadcast real-time GNSS data to map viewers (e.g. Leaflet Map Panel).
"""

import time
import threading
import json
import pickle
import math
import asyncio
from typing import Optional, Dict, Any, List, Callable
from core.device.base import BaseDevice
from core.zpipe import AsyncZSocket, ZPipe
from util.logger.console import ConsoleLogger

logger = ConsoleLogger.get_logger()

try:
    import websockets
    WEBSOCKETS_AVAILABLE = True
except ImportError:
    WEBSOCKETS_AVAILABLE = False
    logger.warning("[SynerexRTK] 'websockets' module not installed. WebSocket server will be disabled.")


class SynerexRTK(BaseDevice):
    """
    Synerex RTK GPS Device module.
    Simulates / receives RTK GNSS telemetry, publishes via ZPipe IPC,
    and runs a WebSocket server on specified port (default: 8765) to feed Leaflet map viewers.
    """

    def __init__(
        self,
        name: str = "synerex_rtk",
        robot_model: str = "iae_patrol_v1",
        ws_host: str = "0.0.0.0",
        ws_port: int = 8765,
        default_lat: float = 34.7971754,
        default_lon: float = 127.6607499,
        default_alt: float = 45.0,
        default_heading: float = 0.0
    ):
        super().__init__(name)
        self.robot_model = robot_model
        self.ws_host = ws_host
        self.ws_port = int(ws_port)

        # Telemetry state
        self.latitude = float(default_lat)
        self.longitude = float(default_lon)
        self.altitude = float(default_alt)
        self.heading = float(default_heading)
        self.status_str = "FIX_RTK"
        self.satellites = 18

        # ZPipe IPC Publisher
        self.pub_socket: Optional[AsyncZSocket] = None
        self.ipc_address = f"/tmp/{self.robot_model}_synerex_rtk.ipc"

        # Worker & WebSocket Server Thread control
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._ws_thread: Optional[threading.Thread] = None

        # Connected WebSocket clients
        self._ws_clients = set()
        self._loop: Optional[asyncio.AbstractEventLoop] = None

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

    def update_pose(self, lat: float, lon: float, alt: float = 45.0, heading: float = 0.0):
        """Update current RTK position and heading coordinates."""
        self.latitude = float(lat)
        self.longitude = float(lon)
        self.altitude = float(alt)
        self.heading = float(heading)

    def connect(self) -> bool:
        """Start hardware simulation worker loop and WebSocket server."""
        self.is_connected = True
        self._running = True

        # Start simulation / telemetry loop thread
        self._thread = threading.Thread(target=self._worker_loop, daemon=True)
        self._thread.start()

        # Start WebSocket server thread if websockets library is available
        if WEBSOCKETS_AVAILABLE:
            self._ws_thread = threading.Thread(target=self._start_ws_server, daemon=True)
            self._ws_thread.start()

        logger.info(f"[{self.name}] Connected. Telemetry IPC: ipc://{self.ipc_address}, WS Server: ws://{self.ws_host}:{self.ws_port}")
        return True

    def disconnect(self) -> bool:
        """Disconnect device and stop all background threads."""
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)
            self._thread = None

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
        """Background thread loop: periodic telemetry publishing over ZPipe & broadcasting via WebSocket."""
        t = 0.0
        base_lat = self.latitude
        base_lon = self.longitude

        while self._running:
            start_time = time.time()
            t += 0.1

            # Simulated slight movement if standalone
            # Circular drift: ~0.0001 deg ~ 10m radius
            self.latitude = base_lat + 0.00008 * math.sin(t * 0.2)
            self.longitude = base_lon + 0.00008 * math.cos(t * 0.2)
            self.heading = (t * 11.45) % 360.0

            data = self.get_status()

            # 1. Publish over ZPipe IPC (pickle format)
            if self.pub_socket and self.pub_socket.is_joined:
                try:
                    payload = pickle.dumps(data)
                    self.pub_socket.dispatch([b"rtk_data", payload])
                except Exception as e:
                    logger.error(f"[{self.name}] ZPipe Publish error: {e}")

            # 2. Broadcast via WebSocket server
            if WEBSOCKETS_AVAILABLE and self._ws_clients and self._loop and self._loop.is_running():
                msg = json.dumps(data)
                asyncio.run_coroutine_threadsafe(self._broadcast_ws(msg), self._loop)

            elapsed = time.time() - start_time
            time.sleep(max(0.0, 0.1 - elapsed))

    def _start_ws_server(self):
        """Run asyncio event loop for WebSocket server."""
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)

        async def handler(websocket, path=None):
            self._ws_clients.add(websocket)
            logger.info(f"[{self.name}] WebSocket client connected: {websocket.remote_address}")
            try:
                # Send immediate current state upon connection
                await websocket.send(json.dumps(self.get_status()))
                async for message in websocket:
                    pass
            except Exception:
                pass
            finally:
                self._ws_clients.remove(websocket)
                logger.info(f"[{self.name}] WebSocket client disconnected.")

        async def main_ws():
            async with websockets.serve(handler, self.ws_host, self.ws_port):
                logger.info(f"[{self.name}] WebSocket Server listening on ws://{self.ws_host}:{self.ws_port}")
                while self._running:
                    await asyncio.sleep(0.5)

        try:
            self._loop.run_until_complete(main_ws())
        except Exception as e:
            logger.error(f"[{self.name}] WebSocket server error: {e}")

    async def _broadcast_ws(self, message: str):
        if self._ws_clients:
            disconnected = set()
            for ws in list(self._ws_clients):
                try:
                    await ws.send(message)
                except Exception:
                    disconnected.add(ws)
            self._ws_clients -= disconnected

    def get_status(self) -> Dict[str, Any]:
        return {
            "name": self.name,
            "connected": self.is_connected,
            "latitude": round(self.latitude, 7),
            "longitude": round(self.longitude, 7),
            "altitude": round(self.altitude, 2),
            "heading": round(self.heading, 1),
            "status": self.status_str,
            "satellites": self.satellites,
            "ws_port": self.ws_port
        }


class SynerexRTK_Connector:
    """
    Asynchronous receiver connector for Synerex RTK data published over ZPipe IPC.
    Connects to SUB socket at ipc:///tmp/<robot_model>_synerex_rtk.ipc.
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

        self.ipc_address = f"/tmp/{self.robot_model}_synerex_rtk.ipc"
        self.sub_socket: Optional[AsyncZSocket] = None
        self.last_rtk_data: Optional[Dict[str, Any]] = None

    def start(self) -> bool:
        """Create SUB AsyncZSocket, connect to IPC publisher, and register message callback."""
        try:
            socket_id = f"synerex_rtk_sub_{int(time.time() * 1000)}"
            self.sub_socket = AsyncZSocket(socket_id=socket_id, pattern="subscribe")
            if not self.sub_socket.create(self.zpipe_ctx):
                return False

            self.sub_socket.set_message_callback(self._on_multipart_received)
            if self.sub_socket.join(transport="ipc", address=self.ipc_address):
                self.sub_socket.subscribe(b"rtk_data")
                logger.info(f"[SynerexRTK_Connector] Subscribed to ZPipe IPC at ipc://{self.ipc_address}")
                return True
            return False
        except Exception as e:
            logger.error(f"[SynerexRTK_Connector] Failed to connect SUB socket: {e}")
            return False

    def stop(self):
        """Close SUB socket."""
        if self.sub_socket:
            try:
                self.sub_socket.close()
            except Exception:
                pass
            self.sub_socket = None
        logger.info("[SynerexRTK_Connector] Stopped.")

    def _on_multipart_received(self, multipart_data: List[bytes]):
        """Callback invoked when ZPipe receives multipart data."""
        if len(multipart_data) >= 2:
            topic = multipart_data[0]
            if topic == b"rtk_data":
                try:
                    data = pickle.loads(multipart_data[1])
                    self.last_rtk_data = data
                    if self.on_data_received:
                        self.on_data_received(data)
                except Exception as e:
                    logger.error(f"[SynerexRTK_Connector] Error unpickling rtk data: {e}")
