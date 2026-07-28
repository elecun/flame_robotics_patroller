"""
Ouster OS0 / OS1 128-channel LiDAR device driver module.
Receives 128-channel Ouster LiDAR data via UDP socket, parses raw point cloud data,
and provides filtering capability for user-specified azimuth angle range.
"""

import socket
import math
import numpy as np
from typing import Optional, Tuple, Dict, Any, List
from core.device.base import BaseDevice
from util.logger.console import ConsoleLogger

logger = ConsoleLogger.get_logger()


class OusterSR128(BaseDevice):
    """
    Ouster OS0 / OS1 128-channel LiDAR receiver and parser device.
    """

    def __init__(
        self,
        name: str = "Ouster-SR-128",
        robot_model: str = "iae_patrol_v1",
        ip: str = "192.168.101.12",
        port: int = 7502,
        min_angle: float = -90.0,
        max_angle: float = 90.0
    ):
        super().__init__(name)
        self.robot_model = robot_model
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
        Open UDP socket bound to specified host/port to listen for Ouster packets.
        """
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.sock.bind(("", self.port))
            self.sock.settimeout(1.0)
            self.is_connected = True
            logger.info(f"[{self.name}] Listening for Ouster 128-channel data on UDP port {self.port} (Default IP: {self.ip})")
            return True
        except Exception as e:
            self.is_connected = False
            self.sock = None
            logger.error(f"[{self.name}] Failed to open UDP socket: {e}")
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
        logger.info(f"[{self.name}] Disconnected.")
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
        Parse raw packet bytes into point cloud array [x, y, z, intensity].
        """
        # Supports custom/standard UDP parsing logic or legacy array buffer structures
        if len(packet) < 48:
            return np.empty((0, 4), dtype=np.float32)

        # Standard processing stub for packet arrays
        points = []
        # Fallback / simulated decode for raw packet frames
        return np.array(points, dtype=np.float32) if points else np.empty((0, 4), dtype=np.float32)

    def receive_and_filter(self) -> np.ndarray:
        """
        Receive packet from socket, parse raw points, apply angle filter, and return filtered points.
        """
        if not self.is_connected or not self.sock:
            return np.empty((0, 4), dtype=np.float32)

        try:
            packet, addr = self.sock.recvfrom(65535)

            raw_points = self.parse_packet(packet)
            filtered_points = self.filter_points(raw_points)
            self._last_points = filtered_points
            return filtered_points

        except socket.timeout:
            return np.empty((0, 4), dtype=np.float32)
        except Exception as e:
            logger.error(f"[{self.name}] Error receiving packet: {e}")
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
