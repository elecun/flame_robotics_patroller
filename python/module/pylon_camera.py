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

from typing import Optional


class component(QThread):
    """
    Pylon camera communication thread using a Basler camera.
    Handles real-time image streaming in continuous or trigger mode.
    """
    # Unified signal to emit the grabbed image (NumPy array) - compatible with module system
    signal_updated = pyqtSignal(object)

    def __init__(self, mode: str = "continuous", device_index: int = 0):
        """
        Initializes the Pylon Camera thread.

        :param mode: Acquisition mode, 'continuous' or 'trigger'.
        :param device_index: The index of the camera to use.
        """
        super().__init__()

        if pylon is None:
            raise ImportError("pypylon library is not installed.")

        if mode not in ["continuous", "trigger"]:
            raise ValueError("Mode must be either 'continuous' or 'trigger'.")

        self.mode = mode
        self.device_index = device_index
        self.running = False
        self._camera: Optional[pylon.InstantCamera] = None
        # Event to safely unblock RetrieveResult during shutdown
        self._stop_requested = False

    def run(self):
        """
        Main thread execution loop. Connects to the camera and starts grabbing images.
        """
        self.running = True
        self._stop_requested = False
        print(f"Connecting to Pylon camera with index {self.device_index} in '{self.mode}' mode...")

        try:
            tl_factory = pylon.TlFactory.GetInstance()
            devices = tl_factory.EnumerateDevices()
            if not devices:
                raise Exception("No Pylon cameras found.")
            if self.device_index >= len(devices):
                raise IndexError(f"Camera index {self.device_index} is out of range. Found {len(devices)} cameras.")

            # Create and open the camera
            self._camera = pylon.InstantCamera(tl_factory.CreateDevice(devices[self.device_index]))
            self._camera.Open()
            print(f"Successfully connected to camera: {self._camera.GetDeviceInfo().GetFriendlyName()}")

            # Configure acquisition mode
            if self.mode == "continuous":
                self._camera.AcquisitionMode.SetValue("Continuous")
                self._camera.TriggerMode.SetValue("Off")
            elif self.mode == "trigger":
                self._camera.AcquisitionMode.SetValue("Continuous")
                self._camera.TriggerSelector.SetValue("FrameStart")
                self._camera.TriggerMode.SetValue("On")
                self._camera.TriggerSource.SetValue("Line2")
                self._camera.TriggerActivation.SetValue("RisingEdge")

            # Start grabbing
            self._camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)

            # Use a short timeout so the loop can check self._stop_requested frequently
            while not self._stop_requested and self._camera.IsGrabbing():
                grab_result = None
                try:
                    # Use a short 500ms timeout so we can respond to stop requests quickly
                    grab_result = self._camera.RetrieveResult(500, pylon.TimeoutHandling_Return)

                    if grab_result is not None and grab_result.IsValid():
                        if grab_result.GrabSucceeded():
                            # Emit a copy of the array so it remains valid after Release()
                            self.signal_updated.emit(grab_result.Array.copy())
                        else:
                            print(f"Grab error: {grab_result.GetErrorCode()} {grab_result.GetErrorDescription()}")

                except pylon.GenericException as e:
                    if self._stop_requested:
                        break
                    if "Timeout" not in str(e):
                        print(f"An error occurred during grabbing: {e}")
                        break
                finally:
                    if grab_result is not None and grab_result.IsValid():
                        grab_result.Release()

        except Exception as e:
            print(f"Failed to connect or stream from Pylon camera: {e}")
        finally:
            self._safe_cleanup()

    def _safe_cleanup(self):
        """Safely stops grabbing and closes the camera."""
        try:
            if self._camera is not None:
                if self._camera.IsGrabbing():
                    self._camera.StopGrabbing()
                if self._camera.IsOpen():
                    self._camera.Close()
        except Exception as e:
            print(f"Error during camera cleanup: {e}")
        finally:
            self.running = False
            self._camera = None
            print("Pylon camera stream stopped.")

    def stop(self):
        """Signals the grabbing loop to stop and waits for the thread to finish."""
        print("Stopping Pylon camera stream...")
        self._stop_requested = True
        self.running = False

        # If the camera is still grabbing, call StopGrabbing to unblock RetrieveResult
        try:
            if self._camera is not None and self._camera.IsGrabbing():
                self._camera.StopGrabbing()
        except Exception as e:
            print(f"Error during StopGrabbing: {e}")

        # Wait up to 3 seconds for the thread to terminate gracefully
        if not self.wait(3000):
            print("Warning: Pylon camera thread did not terminate in time.")