"""
Basler GigE Camera Device Driver and ZPipe Integration Module (core/device/basler_gige_camera.py)
Captures image frames from a Basler GigE camera using pypylon,
publishes JPEG compressed frames / raw metadata via ZPipe AsyncZSocket (IPC PUB/SUB),
and provides BaslerGigECamera_Connector for subscriber reception.
"""

import time
import threading
import json
import pickle
from typing import Optional, Dict, Any, List, Callable
from core.device.base import BaseDevice
from core.zpipe import AsyncZSocket, ZPipe
from util.logger.console import ConsoleLogger

logger = ConsoleLogger.get_logger()

try:
    from pypylon import pylon
    PYPYLON_AVAILABLE = True
except (ImportError, Exception, BaseException) as e:
    PYPYLON_AVAILABLE = False
    pylon = None
    logger.warning(f"[BaslerGigECamera] pypylon is unavailable on this system ({e}). Disabling hardware camera capture.")

try:
    import numpy as np
    import cv2
    OPENCV_AVAILABLE = True
except (ImportError, Exception, BaseException) as e:
    OPENCV_AVAILABLE = False
    np = None
    cv2 = None
    logger.warning(f"[BaslerGigECamera] opencv-python or numpy is unavailable on this system ({e}).")


class BaslerGigECamera(BaseDevice):
    """
    Basler GigE Camera device module.
    Connects to Basler GigE camera (by IP or device index), captures frames at specified FPS,
    encodes image to JPEG, and publishes frame data over ZPipe IPC AsyncZSocket (PUB pattern).
    """

    def __init__(
        self,
        name: str = "basler_gige_camera",
        robot_model: str = "iae_patrol_v1",
        ip: str = "192.168.101.13",
        fps: float = 15.0,
        mode: str = "continuous",
        rotate: str = "",
        resolution: Optional[List[int]] = None,
        roi_resolution: Optional[List[int]] = None,
        exposure_time: int = 5000,
        device_index: int = 0,
        enable: bool = True
    ):
        super().__init__(name, enable=enable)
        self.robot_model = robot_model
        self.ip = str(ip)
        self.fps = float(fps) if float(fps) > 0 else 15.0
        self.mode = mode
        self.rotate = rotate
        self.device_index = int(device_index)

        resolution = resolution or [1920, 1200]
        roi_resolution = roi_resolution or [1920, 1200]
        self._hw_w = int(resolution[0]) if len(resolution) >= 2 else 1920
        self._hw_h = int(resolution[1]) if len(resolution) >= 2 else 1200
        self._emit_w = int(roi_resolution[0]) if len(roi_resolution) >= 2 else self._hw_w
        self._emit_h = int(roi_resolution[1]) if len(roi_resolution) >= 2 else self._hw_h
        self._exposure_time = int(exposure_time)

        self._camera: Optional[Any] = None
        self._converter: Optional[Any] = None

        if PYPYLON_AVAILABLE and pylon is not None:
            try:
                self._converter = pylon.ImageFormatConverter()
                self._converter.OutputPixelFormat = pylon.PixelType_RGB8packed
                self._converter.OutputBitAlignment = pylon.OutputBitAlignment_MsbAligned
            except Exception as e:
                logger.error(f"[{self.name}] Failed to create ImageFormatConverter: {e}")

        # AsyncZSocket for publishing IPC
        self.pub_socket: Optional[AsyncZSocket] = None
        self.ipc_address = f"/tmp/{self.robot_model}_basler_gige_camera.ipc"

        # Worker thread control
        self._running = False
        self._stop_requested = False
        self._thread: Optional[threading.Thread] = None

        # Frame statistics
        self.frame_count = 0
        self.last_frame_timestamp = 0.0

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
        """Connect to camera hardware (or setup loop) and start background capture thread if enabled."""
        if not self.enable:
            self.is_connected = False
            logger.info(f"[{self.name}] Device is DISABLED in config (enable=False).")
            return False

        self._stop_requested = False
        self._running = True

        if not PYPYLON_AVAILABLE or pylon is None:
            self.is_connected = False
            logger.warning(f"[{self.name}] pypylon library is not available. Operating without physical camera hardware.")
            self._start_thread()
            return False

        try:
            tl_factory = pylon.TlFactory.GetInstance()
            devices = tl_factory.EnumerateDevices()

            selected_device = None
            if self.ip:
                for dev in devices:
                    if dev.GetIpAddress() == self.ip:
                        selected_device = dev
                        break
            
            if selected_device is None and devices:
                if 0 <= self.device_index < len(devices):
                    selected_device = devices[self.device_index]

            if selected_device is None:
                logger.warning(f"[{self.name}] Basler camera with IP {self.ip} or index {self.device_index} not found.")
                self.is_connected = False
                self._start_thread()
                return False

            self._camera = pylon.InstantCamera(tl_factory.CreateDevice(selected_device))
            self._camera.Open()
            logger.info(f"[{self.name}] Connected to camera: {self._camera.GetDeviceInfo().GetFriendlyName()} (IP: {self.ip})")

            # Configure resolution
            try:
                self._camera.Width.SetValue(self._hw_w)
                self._camera.Height.SetValue(self._hw_h)
            except Exception as e:
                logger.warning(f"[{self.name}] Could not set camera resolution: {e}")

            # Exposure setting
            try:
                self._camera.ExposureAuto.SetValue("Off")
                self._camera.ExposureTime.SetValue(self._exposure_time)
            except Exception as e:
                logger.warning(f"[{self.name}] Could not set exposure time: {e}")

            # Mode setting
            if self.mode == "continuous":
                self._camera.AcquisitionMode.SetValue("Continuous")
                self._camera.TriggerMode.SetValue("Off")

            self._camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)
            self.is_connected = True

        except Exception as e:
            logger.error(f"[{self.name}] Failed to open Basler camera: {e}")
            self.is_connected = False

        self._start_thread()
        return self.is_connected

    def disconnect(self) -> bool:
        """Stop background capture thread and release camera resources."""
        self._stop_thread()
        self._safe_cleanup()
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
        if not self._thread or not self._thread.is_alive():
            self._running = True
            self._thread = threading.Thread(target=self._worker_loop, daemon=True)
            self._thread.start()
            logger.info(f"[{self.name}] Capture worker thread started.")

    def _stop_thread(self):
        self._stop_requested = True
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)
            self._thread = None

    def _safe_cleanup(self):
        try:
            if self._camera is not None:
                if self._camera.IsGrabbing():
                    self._camera.StopGrabbing()
                if self._camera.IsOpen():
                    self._camera.Close()
        except Exception as e:
            logger.error(f"[{self.name}] Cleanup error: {e}")
        finally:
            self.is_connected = False
            self._camera = None

    def _process_frame(self, image: Any) -> Any:
        """Apply rotation and resize if needed."""
        if cv2 is not None and self.rotate:
            if self.rotate == "cw":
                image = cv2.rotate(image, cv2.ROTATE_90_CLOCKWISE)
            elif self.rotate == "ccw":
                image = cv2.rotate(image, cv2.ROTATE_90_COUNTERCLOCKWISE)
            elif self.rotate == "vflip":
                image = cv2.rotate(image, cv2.ROTATE_180)

        if cv2 is not None and (image.shape[1] != self._emit_w or image.shape[0] != self._emit_h):
            image = cv2.resize(image, (self._emit_w, self._emit_h), interpolation=cv2.INTER_LINEAR)

        return image

    def _worker_loop(self):
        """Worker thread loop: captures frame according to target FPS and publishes via ZPipe IPC."""
        target_interval = 1.0 / self.fps

        while self._running and not self._stop_requested:
            start_time = time.time()
            frame_data = None
            got_frame = False

            if self.is_connected and self._camera is not None and self._camera.IsGrabbing():
                grab_result = None
                try:
                    grab_result = self._camera.RetrieveResult(500, pylon.TimeoutHandling_Return)
                    if grab_result is not None and grab_result.IsValid():
                        if grab_result.GrabSucceeded():
                            if self._converter and not self._converter.ImageHasDestinationFormat(grab_result):
                                converted = self._converter.Convert(grab_result)
                                image = converted.GetArray().copy()
                            else:
                                image = grab_result.Array.copy()

                            image = self._process_frame(image)
                            frame_data = image
                            got_frame = True
                        else:
                            logger.error(f"[{self.name}] Grab error: {grab_result.GetErrorCode()} {grab_result.GetErrorDescription()}")
                except Exception as e:
                    if not self._stop_requested and "Timeout" not in str(e):
                        logger.error(f"[{self.name}] Grab exception: {e}")
                finally:
                    if grab_result is not None and grab_result.IsValid():
                        grab_result.Release()

            if got_frame and frame_data is not None:
                self.frame_count += 1
                self.last_frame_timestamp = time.time()
                self._publish_frame(frame_data)

            elapsed = time.time() - start_time
            sleep_time = target_interval - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    def _publish_frame(self, frame: Any):
        """Encode image to JPEG and dispatch over ZPipe IPC AsyncZSocket."""
        if self.pub_socket and self.pub_socket.is_joined:
            try:
                jpeg_bytes = b""
                if cv2 is not None:
                    # RGB to BGR for OpenCV JPEG encoding if needed
                    bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR) if frame.ndim == 3 and frame.shape[2] == 3 else frame
                    ret, buf = cv2.imencode('.jpg', bgr, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
                    if ret:
                        jpeg_bytes = buf.tobytes()

                header = {
                    "name": self.name,
                    "timestamp": self.last_frame_timestamp,
                    "frame_id": self.frame_count,
                    "width": frame.shape[1],
                    "height": frame.shape[0],
                    "channels": frame.shape[2] if frame.ndim == 3 else 1,
                    "format": "jpeg"
                }

                header_bytes = json.dumps(header, ensure_ascii=False).encode('utf-8')
                self.pub_socket.dispatch([b"basler_gige_camera_data", header_bytes, jpeg_bytes])
            except Exception as e:
                logger.error(f"[{self.name}] Publish frame error: {e}")

    def get_status(self) -> Dict[str, Any]:
        """Return status dictionary of the camera device."""
        return {
            "name": self.name,
            "ip": self.ip,
            "fps": self.fps,
            "connected": self.is_connected,
            "frame_count": self.frame_count,
            "last_frame_timestamp": self.last_frame_timestamp,
            "ipc_address": self.ipc_address
        }


class BaslerGigECamera_Connector:
    """
    Asynchronous receiver connector for Basler GigE Camera image data published over ZPipe IPC.
    Connects to SUB socket at ipc:///tmp/<robot_model>_basler_gige_camera.ipc.
    """

    def __init__(
        self,
        robot_model: str = "iae_patrol_v1",
        zpipe_ctx: Optional[ZPipe] = None,
        on_frame_received: Optional[Callable[[Dict[str, Any], bytes], None]] = None
    ):
        self.robot_model = robot_model
        self.zpipe_ctx = zpipe_ctx or ZPipe.create_pipe()
        self.on_frame_received = on_frame_received

        self.ipc_address = f"/tmp/{self.robot_model}_basler_gige_camera.ipc"
        self.sub_socket: Optional[AsyncZSocket] = None
        self.last_header: Optional[Dict[str, Any]] = None
        self.last_jpeg_data: Optional[bytes] = None

    def start(self) -> bool:
        """Create SUB AsyncZSocket, connect to IPC publisher, and register message callback."""
        try:
            socket_id = f"basler_gige_sub_{int(time.time() * 1000)}"
            self.sub_socket = AsyncZSocket(socket_id=socket_id, pattern="subscribe")
            if not self.sub_socket.create(self.zpipe_ctx):
                return False

            self.sub_socket.set_message_callback(self._on_multipart_received)
            if self.sub_socket.join(transport="ipc", address=self.ipc_address):
                self.sub_socket.subscribe(b"basler_gige_camera_data")
                logger.info(f"[BaslerGigECamera_Connector] Subscribed to ZPipe IPC at ipc://{self.ipc_address}")
                return True
            return False
        except Exception as e:
            logger.error(f"[BaslerGigECamera_Connector] Failed to connect SUB socket: {e}")
            return False

    def stop(self):
        """Close SUB socket."""
        if self.sub_socket:
            try:
                self.sub_socket.close()
            except Exception:
                pass
            self.sub_socket = None
        logger.info("[BaslerGigECamera_Connector] Stopped.")

    def _on_multipart_received(self, multipart_data: List[bytes]):
        """Callback invoked when ZPipe receives camera frame multipart data."""
        if len(multipart_data) >= 3:
            topic = multipart_data[0]
            if topic == b"basler_gige_camera_data":
                try:
                    header_str = multipart_data[1].decode('utf-8')
                    header = json.loads(header_str)
                    jpeg_bytes = multipart_data[2]
                    self.last_header = header
                    self.last_jpeg_data = jpeg_bytes
                    if self.on_frame_received:
                        self.on_frame_received(header, jpeg_bytes)
                except Exception as e:
                    logger.error(f"[BaslerGigECamera_Connector] Error parsing camera payload: {e}")
