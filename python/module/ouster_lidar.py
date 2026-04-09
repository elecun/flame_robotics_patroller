"""
Ouster Lidar Module
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

import os
from contextlib import closing
from datetime import datetime
from typing import Optional, Any

from ouster.sdk import client
from ouster.sdk.pcap import RecordingPacketSource


class component(QThread):
    """
    Ouster OS0-128 LiDAR sensor communication thread.
    Handles real-time data streaming and recording of Lidar and IMU packets.
    """
    # Signal to emit raw packet data (LidarPacket or ImuPacket)
    packet_received = pyqtSignal(object)

    def __init__(self, hostname: str, lidar_port: int = 7502, imu_port: int = 7503):
        """
        Initializes the Ouster LiDAR thread.

        :param hostname: Hostname or IP address of the LiDAR sensor.
        :param lidar_port: UDP port for LiDAR data.
        :param imu_port: UDP port for IMU data.
        """
        super().__init__()
        self.hostname = hostname
        self.lidar_port = lidar_port
        self.imu_port = imu_port
        self.running = False
        self._source: Optional[client.PacketSource] = None
        self._recording = False
        self._record_path: Optional[str] = None

    def run(self):
        """
        Main thread execution loop. Connects to the sensor and streams packets.
        """
        self.running = True
        print(f"Connecting to Ouster sensor at {self.hostname}...")

        try:
            # Create a packet source to receive data from the sensor
            source = client.SensorPacketSource(
                self.hostname,
                lidar_port=self.lidar_port,
                imu_port=self.imu_port
            )
            self._source = source

            with closing(source) as self._source:
                print(f"Successfully connected to Ouster sensor.")
                while self.running:
                    if self._recording and self._record_path:
                        self._start_recording_internal()
                        self._recording = False # Reset flag after recording is done or stopped

                    # Read one packet at a time
                    for packet in self._source:
                        if not self.running:
                            break
                        self.packet_received.emit(packet)
                    
                    if not self.running:
                        break

        except Exception as e:
            print(f"Failed to connect or stream from Ouster sensor: {e}")
        finally:
            self.running = False
            print("Ouster LiDAR stream stopped.")

    def _start_recording_internal(self):
        """Internal method to handle the recording process."""
        if not self._source or not self._record_path:
            return

        print(f"Recording to {self._record_path}...")
        try:
            # Wrap the source with RecordingPacketSource
            recording_source = RecordingPacketSource(self._source, self._record_path, n_frames=None)
            for packet in recording_source:
                if not self.running or not self._recording:
                    break
                self.packet_received.emit(packet)
        except Exception as e:
            print(f"Error during recording: {e}")
        finally:
            print("Recording stopped.")
            self._recording = False

    def start_record(self, filepath: str):
        """Starts recording all incoming packets to a pcap file."""
        self._record_path = filepath
        self._recording = True

    def stop(self):
        """Stops the data streaming thread."""
        print("Stopping Ouster LiDAR stream...")
        self.running = False
        self._recording = False # Also stop recording if active
        self.wait()