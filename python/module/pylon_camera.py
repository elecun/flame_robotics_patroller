"""
Pylon Camera Module
@author Byunghun Hwang<bh.hwang@iae.re.kr>
"""

try:
    from PyQt6.QtCore import QThread, pyqtSignal
except ImportError:
    print("PyQt6 is required to run this module.")
    class QThread:
        def __init__(self): pass
    class pyqtSignal:
        def __init__(self, *args, **kwargs): pass
        def emit(self, *args, **kwargs): pass

try:
    from pypylon import pylon
except ImportError:
    print("pypylon is required to run this module. Please install it via 'pip install pypylon'")
    pylon = None

import threading
from typing import Optional

try:
    import numpy as np
    import cv2
except ImportError:
    np = None
    cv2 = None


class component(QThread):
    """
    Pylon camera communication thread using a Basler camera.
    Handles real-time image streaming in continuous or trigger mode.

    Config keys (pylon_camera.cfg):
        resolution     : [w, h]  - Camera sensor resolution to configure on hardware
        roi_resolution : [w, h]  - Fixed output resolution (what gets emitted)
        zoom_step      : float   - Fraction of base resolution per zoom step (e.g. 0.1)
    """
    signal_updated = pyqtSignal(object)

    def __init__(self,
                 mode: str = "continuous",
                 device_index: int = 0,
                 resolution: list = None,
                 roi_resolution: list = None,
                 zoom_step: float = 0.1):
        super().__init__()

        if pylon is None:
            raise ImportError("pypylon library is not installed.")
        if mode not in ["continuous", "trigger"]:
            raise ValueError("Mode must be either 'continuous' or 'trigger'.")

        self.mode = mode
        self.device_index = device_index
        self.running = False
        self._camera: Optional[pylon.InstantCamera] = None
        self._stop_requested = False
        self._lock = threading.Lock()

        # -- Hardware resolution (set on camera at startup) --
        self._hw_w: int = int(resolution[0]) if resolution and len(resolution) >= 2 else 1920
        self._hw_h: int = int(resolution[1]) if resolution and len(resolution) >= 2 else 1200

        # -- Fixed emit resolution (output always resized to this) --
        self._emit_w: int = int(roi_resolution[0]) if roi_resolution and len(roi_resolution) >= 2 else self._hw_w
        self._emit_h: int = int(roi_resolution[1]) if roi_resolution and len(roi_resolution) >= 2 else self._hw_h

        # -- Digital zoom / ROI state --
        #   Current ROI is the region cropped from the full hardware image.
        #   Initially it equals the full hardware resolution.
        self._zoom_step: float = float(zoom_step)

        # How many pixels one zoom step removes per axis
        self._step_w: int = int(self._hw_w * self._zoom_step)
        self._step_h: int = int(self._hw_h * self._zoom_step)

        # Zoom limits: max 5 steps inward (ROI >= 50 % of hw_res)
        self._zoom_level: int = 0          # 0 = no zoom, positive = zoomed in
        self._max_zoom_level: int = 5

        # Current ROI dimensions (protected by _lock)
        self._roi_w: int = self._hw_w
        self._roi_h: int = self._hw_h

    # ------------------------------------------------------------------
    # Public zoom control API (called from UI thread)
    # ------------------------------------------------------------------
    def zoom_in(self):
        """Reduce ROI by one step (zoom in). Max 5 steps."""
        with self._lock:
            if self._zoom_level < self._max_zoom_level:
                self._zoom_level += 1
                self._roi_w = self._hw_w - self._zoom_level * self._step_w
                self._roi_h = self._hw_h - self._zoom_level * self._step_h
                print(f"[Camera] Zoom IN  level={self._zoom_level}  ROI={self._roi_w}x{self._roi_h}")
            else:
                print(f"[Camera] Zoom IN  already at max level {self._max_zoom_level}")

    def zoom_out(self):
        """Expand ROI by one step (zoom out). Min is original hw resolution."""
        with self._lock:
            if self._zoom_level > 0:
                self._zoom_level -= 1
                self._roi_w = self._hw_w - self._zoom_level * self._step_w
                self._roi_h = self._hw_h - self._zoom_level * self._step_h
                print(f"[Camera] Zoom OUT level={self._zoom_level}  ROI={self._roi_w}x{self._roi_h}")
            else:
                print("[Camera] Zoom OUT already at minimum (full frame).")

    # ------------------------------------------------------------------
    # Thread main loop
    # ------------------------------------------------------------------
    def run(self):
        self.running = True
        self._stop_requested = False
        print(f"Connecting to Pylon camera [{self.device_index}] in '{self.mode}' mode ...")

        try:
            tl_factory = pylon.TlFactory.GetInstance()
            devices = tl_factory.EnumerateDevices()
            if not devices:
                raise Exception("No Pylon cameras found.")
            if self.device_index >= len(devices):
                raise IndexError(f"Camera index {self.device_index} out of range ({len(devices)} found).")

            self._camera = pylon.InstantCamera(tl_factory.CreateDevice(devices[self.device_index]))
            self._camera.Open()
            print(f"Connected: {self._camera.GetDeviceInfo().GetFriendlyName()}")

            # Apply hardware resolution
            self._camera.Width.SetValue(self._hw_w)
            self._camera.Height.SetValue(self._hw_h)
            print(f"Camera hardware resolution set to {self._hw_w}x{self._hw_h}")

            # Acquisition mode
            if self.mode == "continuous":
                self._camera.AcquisitionMode.SetValue("Continuous")
                self._camera.TriggerMode.SetValue("Off")
            elif self.mode == "trigger":
                self._camera.AcquisitionMode.SetValue("Continuous")
                self._camera.TriggerSelector.SetValue("FrameStart")
                self._camera.TriggerMode.SetValue("On")
                self._camera.TriggerSource.SetValue("Line2")
                self._camera.TriggerActivation.SetValue("RisingEdge")

            self._camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)

            while not self._stop_requested and self._camera.IsGrabbing():
                grab_result = None
                try:
                    grab_result = self._camera.RetrieveResult(500, pylon.TimeoutHandling_Return)

                    if grab_result is not None and grab_result.IsValid():
                        if grab_result.GrabSucceeded():
                            image = grab_result.Array.copy()
                            image = self._process_frame(image)
                            self.signal_updated.emit(image)
                        else:
                            print(f"Grab error: {grab_result.GetErrorCode()} {grab_result.GetErrorDescription()}")

                except pylon.GenericException as e:
                    if self._stop_requested:
                        break
                    if "Timeout" not in str(e):
                        print(f"Grab exception: {e}")
                        break
                finally:
                    if grab_result is not None and grab_result.IsValid():
                        grab_result.Release()

        except Exception as e:
            print(f"Camera error: {e}")
        finally:
            self._safe_cleanup()

    # ------------------------------------------------------------------
    # Image processing
    # ------------------------------------------------------------------
    def _process_frame(self, image: "np.ndarray") -> "np.ndarray":
        """
        1. Center-crop according to current ROI (zoom level).
        2. Resize to fixed emit resolution (roi_resolution from cfg).
        """
        # Step 1: center-crop to current ROI
        with self._lock:
            roi_w = self._roi_w
            roi_h = self._roi_h

        if image.ndim == 2:
            src_h, src_w = image.shape
        else:
            src_h, src_w = image.shape[:2]

        crop_w = min(roi_w, src_w)
        crop_h = min(roi_h, src_h)
        x1 = (src_w - crop_w) // 2
        y1 = (src_h - crop_h) // 2

        if image.ndim == 2:
            cropped = image[y1:y1 + crop_h, x1:x1 + crop_w]
        else:
            cropped = image[y1:y1 + crop_h, x1:x1 + crop_w, :]

        # Step 2: resize to fixed emit resolution
        if cv2 is not None:
            resized = cv2.resize(cropped, (self._emit_w, self._emit_h), interpolation=cv2.INTER_LINEAR)
        else:
            # Fallback: return cropped without resize if cv2 is unavailable
            resized = cropped

        return resized

    # ------------------------------------------------------------------
    # Cleanup / stop
    # ------------------------------------------------------------------
    def _safe_cleanup(self):
        try:
            if self._camera is not None:
                if self._camera.IsGrabbing():
                    self._camera.StopGrabbing()
                if self._camera.IsOpen():
                    self._camera.Close()
        except Exception as e:
            print(f"Cleanup error: {e}")
        finally:
            self.running = False
            self._camera = None
            print("Pylon camera stream stopped.")

    def stop(self):
        print("Stopping Pylon camera ...")
        self._stop_requested = True
        self.running = False
        try:
            if self._camera is not None and self._camera.IsGrabbing():
                self._camera.StopGrabbing()
        except Exception as e:
            print(f"StopGrabbing error: {e}")
        if not self.wait(3000):
            print("Warning: camera thread did not terminate in time.")