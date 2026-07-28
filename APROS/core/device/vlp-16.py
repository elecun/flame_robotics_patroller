"""
Velodyne VLP-16 16-channel LiDAR device driver module.
Receives 16-channel Velodyne LiDAR data via UDP socket, parses raw point cloud data,
and provides filtering capability for user-specified azimuth angle range (default [-90.0, +90.0] degrees).
"""

import socket
import struct
import math
import numpy as np
from typing import Optional, Tuple, Dict, Any, List
from core.device.base import BaseDevice

class VLP16(BaseDevice):
    """
    Velodyne VLP-16 16-channel LiDAR receiver and parser device.
    """

    # VLP-16 factory vertical angles in degrees for laser IDs 0..15
    VERTICAL_ANGLES = [
        -15.0, 1.0, -13.0, 3.0, -11.0, 5.0, -9.0, 7.0,
        -7.0, 9.0, -5.0, 11.0, -3.0, 13.0, -1.0, 15.0
    ]

    def __init__(
        self,
        name: str = "VLP-16",
        ip: str = "192.168.100.10",
        port: int = 2368,
        min_angle: float = -90.0,
        max_angle: float = 90.0
    ):
        super().__init__(name)
        self.ip = ip
        self.port = int(port)
        self.min_angle = float(min_angle)
        self.max_angle = float(max_angle)

        self.sock: Optional[socket.socket] = None
        self._last_points: Optional[np.ndarray] = None  # Array of shape (N, 4) -> [x, y, z, intensity]

    def set_angle_filter(self, min_angle: float, max_angle: float):
        """
        Set azimuth angle filtering range in degrees.
        """
        self.min_angle = float(min_angle)
        self.max_angle = float(max_angle)

    def connect(self) -> bool:
        """
        Open UDP socket bound to specified host/port to listen for VLP-16 packets.
        """
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            # Bind to listen on all interfaces or specific port
            self.sock.bind(("", self.port))
            self.sock.settimeout(1.0)
            self.is_connected = True
            print(f"[{self.name}] Listening for VLP-16 data on UDP port {self.port} (Expected IP: {self.ip})")
            return True
        except Exception as e:
            self.is_connected = False
            self.sock = None
            print(f"[{self.name}] Failed to open UDP socket: {e}")
            return False

    def disconnect(self) -> bool:
        """
        Close UDP socket.
        """
        if self.sock:
            try:
                self.sock.close()
            except Exception:
                pass
            self.sock = None
        self.is_connected = False
        print(f"[{self.name}] Disconnected.")
        return True

    def filter_points(self, points: np.ndarray) -> np.ndarray:
        """
        Filter point cloud array (N, 3 or N, 4) based on azimuth angle [min_angle, max_angle] in degrees.
        Azimuth angle is calculated as arctan2(y, x) in degrees, normalized to [-180, 180].
        """
        if points is None or len(points) == 0:
            return np.empty((0, 4), dtype=np.float32)

        # Compute azimuth in degrees [-180, 180]
        azimuth_deg = np.degrees(np.arctan2(points[:, 1], points[:, 0]))

        if self.min_angle <= self.max_angle:
            mask = (azimuth_deg >= self.min_angle) & (azimuth_deg <= self.max_angle)
        else:
            # Wrapped angle range across +/-180 deg boundary
            mask = (azimuth_deg >= self.min_angle) | (azimuth_deg <= self.max_angle)

        return points[mask]

    def parse_packet(self, packet: bytes) -> np.ndarray:
        """
        Parse standard 1206-byte VLP-16 MSOP data packet into point cloud array [x, y, z, intensity].
        VLP-16 packet layout: 12 data blocks (100 bytes each) + 4 bytes timestamp + 2 bytes factory field.
        """
        if len(packet) != 1206:
            return np.empty((0, 4), dtype=np.float32)

        points = []
        vert_rad = np.radians(self.VERTICAL_ANGLES)

        for block_idx in range(12):
            offset = block_idx * 100
            flag, azimuth_raw = struct.unpack_from("<HH", packet, offset)
            if flag != 0xEEFF:
                continue

            azimuth_deg = azimuth_raw / 100.0
            azimuth_rad = math.radians(azimuth_deg)

            # Each block contains 2 firings (16 channels each)
            for channel in range(32):
                channel_offset = offset + 4 + channel * 3
                distance_raw, intensity = struct.unpack_from("<HB", packet, channel_offset)
                distance_m = distance_raw * 0.002  # 2mm resolution

                if distance_m <= 0.1:  # Skip invalid / zero readings
                    continue

                laser_id = channel % 16
                omega = vert_rad[laser_id]

                # Spherical to Cartesian coordinate conversion
                x = distance_m * math.cos(omega) * math.sin(azimuth_rad)
                y = distance_m * math.cos(omega) * math.cos(azimuth_rad)
                z = distance_m * math.sin(omega)

                points.append([x, y, z, float(intensity)])

        if not points:
            return np.empty((0, 4), dtype=np.float32)

        return np.array(points, dtype=np.float32)

    def receive_and_filter(self) -> np.ndarray:
        """
        Receive packet from socket, parse raw points, apply angle filter, and return filtered points.
        """
        if not self.is_connected or not self.sock:
            return np.empty((0, 4), dtype=np.float32)

        try:
            packet, addr = self.sock.recvfrom(2048)
            # Filter source IP if specified
            if self.ip and addr[0] != self.ip:
                pass  # Accept or process packet; can be modified if strict IP check is required

            raw_points = self.parse_packet(packet)
            filtered_points = self.filter_points(raw_points)
            self._last_points = filtered_points
            return filtered_points

        except socket.timeout:
            return np.empty((0, 4), dtype=np.float32)
        except Exception as e:
            print(f"[{self.name}] Error receiving packet: {e}")
            return np.empty((0, 4), dtype=np.float32)

    def get_status(self) -> Dict[str, Any]:
        return {
            "name": self.name,
            "ip": self.ip,
            "port": self.port,
            "connected": self.is_connected,
            "min_angle": self.min_angle,
            "max_angle": self.max_angle,
            "last_point_count": len(self._last_points) if self._last_points is not None else 0
        }
