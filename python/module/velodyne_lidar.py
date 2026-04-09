"""
Velodyne Lidar Module
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

import socket
from contextlib import closing
from typing import Optional
from scapy.all import wrpcap, Ether, IP, UDP


class component(QThread):
    """
    Velodyne VLP-16 LiDAR sensor communication thread.
    Handles real-time data streaming and recording of Lidar packets.
    """
    # Signal to emit raw packet data (bytes)
    packet_received = pyqtSignal(object)

    def __init__(self, device_ip: str = "192.168.1.201", port: int = 2368):
        """
        Initializes the Velodyne LiDAR thread.

        :param device_ip: IP address of the LiDAR sensor.
        :param port: UDP port for LiDAR data.
        """
        super().__init__()
        self.device_ip = device_ip
        self.port = port
        self.running = False
        self._sock: Optional[socket.socket] = None
        self._recording = False
        self._record_path: Optional[str] = None
        self._pcap_writer = None

    def run(self):
        """
        Main thread execution loop. Connects to the sensor and streams packets.
        """
        self.running = True
        print(f"Connecting to Velodyne sensor at {self.device_ip}:{self.port}...")

        try:
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._sock.bind(('', self.port))
            self._sock.settimeout(1.0)  # Set a timeout to allow checking self.running

            print("Successfully connected to Velodyne sensor.")

            while self.running:
                try:
                    # Read one packet at a time
                    data, addr = self._sock.recvfrom(2048)

                    # We only care about data from the sensor's IP
                    if addr[0] == self.device_ip:
                        self.packet_received.emit(data)

                        if self._recording and self._record_path:
                            # Create pcap packet and write to file
                            pcap_packet = Ether() / IP(src=addr[0], dst="255.255.255.255") / UDP(sport=self.port, dport=self.port) / data
                            wrpcap(self._record_path, pcap_packet, append=True)

                except socket.timeout:
                    # Timeout allows the loop to check the self.running flag
                    continue
                except Exception as e:
                    if self.running:
                        print(f"Error receiving packet: {e}")

        except Exception as e:
            print(f"Failed to connect or stream from Velodyne sensor: {e}")
        finally:
            if self._sock:
                self._sock.close()
            self.running = False
            print("Velodyne LiDAR stream stopped.")

    def start_record(self, filepath: str):
        """
        Starts recording all incoming packets to a pcap file.
        Note: This implementation appends to the file if it exists.
        A new file is created for each call to avoid mixing recordings.
        """
        if self._recording:
            print("Recording is already in progress.")
            return

        print(f"Recording to {filepath}...")
        self._record_path = filepath
        self._recording = True

    def stop(self):
        """Stops the data streaming thread."""
        print("Stopping Velodyne LiDAR stream...")
        if self.running:
            self.running = False
            self._recording = False
            self.wait(2000) # Wait up to 2 seconds for the thread to finish

    def __del__(self):
        self.stop()