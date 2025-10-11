"""
Pylon Camera Module
@author Byunghun Hwang<bh.hwang@iae.re.kr>
"""

try:
    from PyQt6.QtCore import QThread, pyqtSignal
except ImportError:
    print("PyQt6 is required to run this module.")
    # Provide dummy classes if PyQt6 is not available for non-GUI use
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


class pylonCamera(QThread):
    """
    Pylon camera communication thread using a Basler camera.
    Handles real-time image streaming in continuous or trigger mode.
    """
    # Signal to emit the grabbed image (NumPy array)
    image_received = pyqtSignal(object)

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
        self._grab_result: Optional[pylon.GrabResult] = None

    def run(self):
        """
        Main thread execution loop. Connects to the camera and starts grabbing images.
        """
        self.running = True
        print(f"Connecting to Pylon camera with index {self.device_index} in '{self.mode}' mode...")

        try:
            # Get the transport layer factory.
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
                self._camera.AcquisitionMode.SetValue("Continuous") # Grab continuously when triggered
                self._camera.TriggerSelector.SetValue("FrameStart")
                self._camera.TriggerMode.SetValue("On")
                self._camera.TriggerSource.SetValue("Line2") # Assuming hardware trigger on Line 2
                self._camera.TriggerActivation.SetValue("RisingEdge")

            # Start grabbing
            self._camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)

            while self.running and self._camera.IsGrabbing():
                try:
                    self._grab_result = self._camera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)

                    if self._grab_result.GrabSucceeded():
                        # Emit the image as a numpy array
                        self.image_received.emit(self._grab_result.Array)
                    else:
                        print(f"Error: {self._grab_result.GetErrorCode()} {self._grab_result.GetErrorDescription()}")

                    self._grab_result.Release()
                except pylon.GenericException as e:
                    # Timeout can happen in trigger mode if no trigger arrives
                    if "Timeout" not in str(e) and self.running:
                        print(f"An error occurred during grabbing: {e}")
                        break

        except Exception as e:
            print(f"Failed to connect or stream from Pylon camera: {e}")
        finally:
            if self._camera and self._camera.IsOpen():
                self._camera.StopGrabbing()
                self._camera.Close()
            self.running = False
            print("Pylon camera stream stopped.")

    def stop(self):
        """Stops the image streaming thread."""
        print("Stopping Pylon camera stream...")
        if self.running:
            self.running = False
            self.wait(2000) # Wait up to 2 seconds for the thread to finish