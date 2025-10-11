"""
Generic Top Module
@author Byunghun Hwang<bh.hwang@iae.re.kr>
"""

try:
    from PyQt6.QtCore import QThread, pyqtSignal
except ImportError:
    print("PyQt6 is required to run this module.")
    # Provide dummy classes if PyQt6 is not available for non-GUI use
    class QThread:
        def __init__(self): pass
        def msleep(self, *args, **kwargs): pass
        def wait(self, *args, **kwargs): pass

    class pyqtSignal:
        def __init__(self, *args, **kwargs): pass
        def emit(self, *args, **kwargs): pass


class TopModule(QThread):
    """
    A generic QThread-based module with start and stop interfaces.
    This class serves as a template for creating new device or logic threads.
    """
    # Signal to emit data (e.g., a string or a dictionary)
    data_received = pyqtSignal(object)

    def __init__(self):
        """
        Initializes the TopModule thread.
        """
        super().__init__()
        self.running = False

    def run(self):
        """
        Main thread execution loop. This is where the module's logic goes.
        """
        self.running = True
        print("TopModule thread started.")

        counter = 0
        while self.running:
            # Example logic: emit a counter value every second
            self.data_received.emit(f"Heartbeat: {counter}")
            counter += 1
            self.msleep(1000)  # Sleep for 1000 milliseconds (1 second)

        print("TopModule thread stopped.")

    def stop(self):
        """Stops the data streaming thread."""
        print("Stopping TopModule thread...")
        if self.running:
            self.running = False
            self.wait()  # Wait for the thread to finish cleanly