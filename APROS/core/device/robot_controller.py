"""
Robot Base Controller Device (CAN / Motor Driver Interface placeholder).
"""
import time
import random
import numpy as np
from core.device.base import BaseDevice

class RobotController(BaseDevice):
    def __init__(self, name: str = "RobotController", enable: bool = True):
        super().__init__(name, enable=enable)
        self.speed = 0.0  # km/h
        self.steer_angle = 0.0  # degrees
        self.lat = 37.5665  # Default latitude (Seoul)
        self.lon = 126.9780  # Default longitude (Seoul)
        self.gear = "P"
        self.drive_mode = "Auto"
        self.simulated_heading = 0.0  # in radians

    def connect(self) -> bool:
        if not self.enable:
            self.is_connected = False
            return False
        self.is_connected = True
        return True

    def disconnect(self) -> bool:
        self.is_connected = False
        return True

    def set_drive_cmd(self, speed_kmh: float, steer_deg: float):
        self.speed = max(0.0, min(speed_kmh, 25.0))
        self.steer_angle = max(-35.0, min(steer_deg, 35.0))

    def update_simulation_step(self, dt: float = 0.05):
        """Update simulated pose, speed, and GPS position for autonomous driving view."""
        if not self.is_connected:
            return

        # Simple kinematic model for visualization
        speed_m_s = (self.speed * 1000.0) / 3600.0
        wheelbase = 1.5  # meters
        
        # Steering angular velocity
        if abs(self.steer_angle) > 0.01:
            turning_radius = wheelbase / float(np.tan(np.radians(self.steer_angle)))
            angular_velocity = speed_m_s / turning_radius
        else:
            angular_velocity = 0.0

        self.simulated_heading += angular_velocity * dt

        # Update GPS latitude and longitude roughly (1 deg lat ~ 111,000 m)
        d_lat = (speed_m_s * dt * float(np.cos(self.simulated_heading))) / 111000.0
        d_lon = (speed_m_s * dt * float(np.sin(self.simulated_heading))) / (111000.0 * 0.79)
        
        self.lat += d_lat
        self.lon += d_lon

    def get_status(self) -> dict:
        return {
            "speed": self.speed,
            "steer_angle": self.steer_angle,
            "latitude": self.lat,
            "longitude": self.lon,
            "gear": self.gear,
            "drive_mode": self.drive_mode,
            "connected": self.is_connected,
            "heading": self.simulated_heading
        }
