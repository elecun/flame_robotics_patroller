"""
Synerex RTK GPS Device Driver and WebSocket Server Module (core/device/synerex_rtk.py)
Receives Synerex RTK GNSS latitude, longitude, altitude, heading, and NMEA fix quality data via serial or simulation,
publishes RTK telemetry via ZPipe AsyncZSocket (IPC PUB/SUB),
and runs a WebSocket server to broadcast real-time GNSS data to map viewers (e.g. Leaflet Map Panel).
"""

import time
import threading
import json
import math
import asyncio
from typing import Optional, Dict, Any, List, Callable

try:
    import serial
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False
    serial = None

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
    Parses RTK GNSS telemetry from serial port (or simulates), publishes via ZPipe IPC,
    and runs a WebSocket server to feed Leaflet map viewers.
    """

    def __init__(
        self,
        name: str = "synerex_rtk",
        robot_model: str = "iae_patrol_v1",
        port: str = "/dev/ttyACM0",
        baudrate: int = 9600,
        utc_offset: int = 9,
        update_interval_sec: float = 1.0,
        ws_host: str = "0.0.0.0",
        ws_port: int = 18765,
        default_lat: float = 34.7971754,
        default_lon: float = 127.6607499,
        default_alt: float = 45.0,
        default_heading: float = 0.0,
        enable: bool = True,
        **kwargs
    ):
        super().__init__(name, enable=enable)
        self.robot_model = robot_model
        self.port = str(port)
        self.baudrate = int(baudrate)
        self.utc_offset = int(utc_offset)
        self.update_interval_sec = float(update_interval_sec)
        self.ws_host = str(ws_host)
        self.ws_port = int(ws_port)

        # Telemetry state
        self.latitude: Optional[float] = None
        self.longitude: Optional[float] = None
        self.altitude: Optional[float] = None
        self.heading: float = float(default_heading)
        self.fix_quality: int = 0  # 0 = Invalid
        self.status_str: str = self.quality2str(self.fix_quality)
        self.satellites: int = 0

        self.serial_connection: Optional[Any] = None

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

    @staticmethod
    def quality2str(quality: int) -> str:
        quality_map = {
            0: "Invalid",
            1: "3D",
            2: "DGPS/DGNSS",
            4: "RTK Fixed",
            5: "RTK Float"
        }
        return quality_map.get(quality, "Unknown")

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

    def update_pose(self, lat: float, lon: float, alt: float = 45.0, heading: float = 0.0, fix_quality: int = 4):
        """Update current RTK position, heading and fix quality coordinates."""
        self.latitude = float(lat)
        self.longitude = float(lon)
        self.altitude = float(alt)
        self.heading = float(heading)
        self.fix_quality = int(fix_quality)
        self.status_str = self.quality2str(self.fix_quality)

    def connect_serial(self) -> bool:
        if not SERIAL_AVAILABLE or serial is None:
            return False
        try:
            self.serial_connection = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=1.0
            )
            return self.serial_connection.is_open
        except Exception as e:
            logger.warning(f"[{self.name}] Serial connection failed on {self.port}: {e}")
            return False

    def disconnect_serial(self):
        if self.serial_connection and hasattr(self.serial_connection, "is_open") and self.serial_connection.is_open:
            try:
                self.serial_connection.close()
            except Exception:
                pass
        self.serial_connection = None

    def connect(self) -> bool:
        """Start worker thread and WebSocket server if enabled."""
        if not self.enable:
            self.is_connected = False
            logger.info(f"[{self.name}] Device is DISABLED in config (enable=False).")
            return False

        self.is_connected = True
        self._running = True

        # Start worker thread
        self._thread = threading.Thread(target=self._worker_loop, daemon=True)
        self._thread.start()

        # Start WebSocket server thread if websockets library is available
        if WEBSOCKETS_AVAILABLE:
            self._ws_thread = threading.Thread(target=self._start_ws_server, daemon=True)
            self._ws_thread.start()

        logger.info(f"[{self.name}] Connected. Port={self.port}, Baud={self.baudrate}, Telemetry IPC: ipc://{self.ipc_address}")
        return True

    def disconnect(self) -> bool:
        """Disconnect device and stop all background threads."""
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)
            self._thread = None

        if self._loop and self._loop.is_running():
            try:
                self._loop.call_soon_threadsafe(self._loop.stop)
            except Exception:
                pass

        if self._ws_thread:
            self._ws_thread.join(timeout=1.0)
            self._ws_thread = None

        if self.pub_socket:
            try:
                self.pub_socket.close()
            except Exception:
                pass
            self.pub_socket = None

        self.disconnect_serial()
        self.is_connected = False
        logger.info(f"[{self.name}] Disconnected.")
        return True

    def _convert_to_decimal_degrees(self, coord: str, direction: str) -> float:
        if not coord or len(coord) < 4:
            return 0.0
        if '.' in coord:
            dot_index = coord.index('.')
            if dot_index >= 3:
                degrees = float(coord[:dot_index-2])
                minutes = float(coord[dot_index-2:])
            else:
                degrees = 0.0
                minutes = float(coord)
        else:
            if len(coord) >= 4:
                degrees = float(coord[:-2])
                minutes = float(coord[-2:])
            else:
                degrees = 0.0
                minutes = float(coord)

        decimal_degrees = degrees + minutes / 60.0
        if direction in ['S', 'W']:
            decimal_degrees = -decimal_degrees
        return decimal_degrees

    def parse_gga(self, parts: list):
        if len(parts) < 15:
            return
        try:
            lat_raw, lat_dir = parts[2], parts[3]
            lon_raw, lon_dir = parts[4], parts[5]
            if lat_raw and lat_dir:
                self.latitude = self._convert_to_decimal_degrees(lat_raw, lat_dir)
            if lon_raw and lon_dir:
                self.longitude = self._convert_to_decimal_degrees(lon_raw, lon_dir)
            if parts[6]:
                self.fix_quality = int(parts[6])
                self.status_str = self.quality2str(self.fix_quality)
            if parts[7]:
                self.satellites = int(parts[7])
            if parts[9]:
                self.altitude = float(parts[9])
        except (ValueError, IndexError):
            pass

    def parse_hdt(self, parts: list):
        if len(parts) < 2:
            return
        try:
            if parts[1]:
                self.heading = float(parts[1])
        except (ValueError, IndexError):
            pass

    def parse_rmc(self, parts: list):
        if len(parts) < 9:
            return
        try:
            lat_raw, lat_dir = parts[3], parts[4]
            lon_raw, lon_dir = parts[5], parts[6]
            if lat_raw and lat_dir:
                self.latitude = self._convert_to_decimal_degrees(lat_raw, lat_dir)
            if lon_raw and lon_dir:
                self.longitude = self._convert_to_decimal_degrees(lon_raw, lon_dir)
            if parts[8]:  # Track angle / course over ground (heading in deg)
                self.heading = float(parts[8])
        except (ValueError, IndexError):
            pass

    def parse_vtg(self, parts: list):
        if len(parts) < 2:
            return
        try:
            if parts[1]:  # True track made good (heading in deg)
                self.heading = float(parts[1])
        except (ValueError, IndexError):
            pass

    def parse_nmea_line(self, line: str):
        line = line.strip()
        if not line.startswith('$'):
            return
        parts = line.split(',')
        if len(parts) < 2:
            return
        sentence_id = parts[0][1:]
        if sentence_id.endswith('GGA'):
            self.parse_gga(parts)
        elif sentence_id.endswith('HDT'):
            self.parse_hdt(parts)
        elif sentence_id.endswith('RMC'):
            self.parse_rmc(parts)
        elif sentence_id.endswith('VTG'):
            self.parse_vtg(parts)

    def _worker_loop(self):
        """Background thread loop: read serial or simulate movement, periodically publish ZPipe & WebSocket."""
        has_serial = self.connect_serial()
        if not has_serial:
            logger.info(f"[{self.name}] Serial port {self.port} unavailable. Running in simulation mode.")

        t = 0.0
        base_lat = self.latitude
        base_lon = self.longitude

        while self._running:
            start_time = time.time()
            if self.serial_connection and hasattr(self.serial_connection, "is_open") and self.serial_connection.is_open:
                try:
                    if self.serial_connection.in_waiting > 0:
                        line = self.serial_connection.readline().decode('utf-8', errors='ignore').strip()
                        if line.startswith('$'):
                            self.parse_nmea_line(line)
                except Exception as e:
                    logger.error(f"[{self.name}] Serial read error: {e}")
                    self.disconnect_serial()
            else:
                # Simulation mode
                t += 0.1
                base_lat = base_lat or 34.7971754
                base_lon = base_lon or 127.6607499
                self.latitude = base_lat + 0.00008 * math.sin(t * 0.2)
                self.longitude = base_lon + 0.00008 * math.cos(t * 0.2)
                self.heading = (t * 11.45) % 360.0

            data = self.get_status()

            # 1. Publish over ZPipe IPC
            if self.pub_socket and self.pub_socket.is_joined:
                try:
                    payload = json.dumps(data, ensure_ascii=False).encode('utf-8')
                    self.pub_socket.dispatch([b"rtk_data", payload])
                except Exception as e:
                    logger.error(f"[{self.name}] ZPipe Publish error: {e}")

            # 2. Broadcast via WebSocket server
            if WEBSOCKETS_AVAILABLE and self._ws_clients and self._loop and self._loop.is_running():
                msg = json.dumps(data)
                asyncio.run_coroutine_threadsafe(self._broadcast_ws(msg), self._loop)

            elapsed = time.time() - start_time
            sleep_time = max(0.001, self.update_interval_sec - elapsed)
            time.sleep(sleep_time)

    def _start_ws_server(self):
        """Run asyncio event loop for WebSocket server."""
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)

        async def handler(websocket, path=None):
            self._ws_clients.add(websocket)
            logger.info(f"[{self.name}] WebSocket client connected: {websocket.remote_address}")
            try:
                await websocket.send(json.dumps(self.get_status()))
                async for message in websocket:
                    pass
            except Exception:
                pass
            finally:
                self._ws_clients.remove(websocket)
                logger.info(f"[{self.name}] WebSocket client disconnected.")

        async def main_ws():
            async with websockets.serve(handler, self.ws_host, self.ws_port) as server:
                logger.info(f"[{self.name}] WebSocket Server listening on ws://{self.ws_host}:{self.ws_port}")
                while self._running:
                    await asyncio.sleep(0.2)
                server.close()
                await server.wait_closed()

        try:
            self._loop.run_until_complete(main_ws())
        except (asyncio.CancelledError, OSError, Exception):
            pass
        finally:
            try:
                tasks = asyncio.all_tasks(self._loop)
                for task in tasks:
                    task.cancel()
                self._loop.run_until_complete(asyncio.gather(*tasks, return_exceptions=True))
                self._loop.close()
            except Exception:
                pass

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
            "latitude": round(self.latitude, 7) if self.latitude is not None else None,
            "longitude": round(self.longitude, 7) if self.longitude is not None else None,
            "altitude": round(self.altitude, 2) if self.altitude is not None else None,
            "heading": round(self.heading, 1) if self.heading is not None else 0.0,
            "fix_quality": self.fix_quality,
            "quality_str": self.quality2str(self.fix_quality),
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
                payload_bytes = multipart_data[1]
                if not payload_bytes:
                    return
                try:
                    json_str = payload_bytes.decode('utf-8')
                    data = json.loads(json_str)
                    self.last_rtk_data = data
                    if self.on_data_received:
                        self.on_data_received(data)
                except Exception as e:
                    logger.error(f"[SynerexRTK_Connector] Error decoding RTK JSON data: {e}")
