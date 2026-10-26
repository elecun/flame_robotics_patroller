"""
Robot Base Controller Device (Simulation Drive Model for --simulation mode).
Provides the same interface as MobileDriveS1 without hardware CAN dependencies.
"""
import math
from typing import Optional
import numpy as np
from core.device.base import BaseDevice
from util.logger.console import ConsoleLogger

logger = ConsoleLogger.get_logger()


class RobotController(BaseDevice):
    def __init__(
        self,
        name: str = "RobotController",
        max_steering_angle: float = 30.0,
        min_velocity: float = 0.0,
        max_velocity: float = 4.0,
        wheelbase: float = 1.08,
        max_steer_rate_deg_s: float = 15.0,
        lookahead_distance: float = 3.0,
        predict_time: float = 3.0,
        corridor_boundary: float = 3.0,
        enable: bool = True,
        **kwargs
    ):
        super().__init__(name, enable=enable)
        # Steering angle bounds (degrees): strictly bounded within [-30.0, +30.0]
        # Polarity: - is Right turn (CW), + is Left turn (CCW)
        if "max_steer_angle" in kwargs:
            max_steering_angle = kwargs["max_steer_angle"]
        self.max_steering_angle = min(30.0, abs(float(max_steering_angle)))
        self.min_steering_angle = -self.max_steering_angle
        self.max_steer_rate_deg_s = float(max_steer_rate_deg_s)
        self.wheelbase = float(wheelbase)

        # Velocity bounds (km/h)
        self.MIN_VELOCITY_KMH = float(min_velocity)
        self.MAX_VELOCITY_KMH = float(max_velocity)

        self.lookahead_distance = float(lookahead_distance)
        self.predict_time = float(predict_time)
        self.corridor_boundary = float(corridor_boundary)

        self.MIN_CMD_VAL = -2000
        self.MAX_CMD_VAL = 2000

        # State variables matching MobileDriveS1
        self.speed = 0.0              # km/h (actual simulated speed)
        self.steer_angle = 0.0        # degrees (actual simulated steering angle)
        self.cmd_speed = 0.0          # km/h (commanded speed)
        self.cmd_steering_angle = 0.0 # degrees (commanded steering angle)
        self.lat = float(kwargs.get("default_lat", 34.824628))
        self.lon = float(kwargs.get("default_lon", 127.661044))
        self.target_gear = "P"
        self.gear = "P"
        self.drive_mode = "Remote Control Mode"
        self.ad_control_req_flag = 0
        self.simulated_heading = 0.0  # radians (0 = +X North, +pi/2 = +Y West / CCW Left)
        self.parsed_can_status = {}

        # Brake & Lighting states
        self.cmd_brake = 0.0
        self.ad_dbs_valid = 0
        self.left_turn_light = False
        self.right_turn_light = False
        self.head_light = False
        self.brake_light = False

    def connect(self) -> bool:
        if not self.enable:
            self.is_connected = False
            return False
        self.is_connected = True
        return True

    def disconnect(self) -> bool:
        self.is_connected = False
        return True

    def _clamp_speed_kmh(self, speed_kmh: float) -> float:
        requested = float(speed_kmh)
        if not math.isfinite(requested) or abs(requested) <= 0.01:
            return 0.0
        if requested > 0:
            return max(self.MIN_VELOCITY_KMH, min(requested, self.MAX_VELOCITY_KMH))
        return min(-self.MIN_VELOCITY_KMH, max(requested, -self.MAX_VELOCITY_KMH))

    def set_drive_cmd(self, speed_kmh: float, steer_deg: float):
        self.set_speed(speed_kmh)
        self.set_steering_angle(steer_deg)

    def set_speed(self, speed_kmh: float):
        clamped_speed = self._clamp_speed_kmh(speed_kmh)
        self.cmd_speed = clamped_speed
        self.speed = clamped_speed
        if clamped_speed < -0.01:
            self.target_gear = "R"
            self.gear = "R"
        elif clamped_speed > 0.01:
            self.target_gear = "D"
            self.gear = "D"
        self.ad_dbs_valid = 0

    def set_steering_angle(self, steer_deg: float):
        clamped_angle = max(self.min_steering_angle, min(float(steer_deg), self.max_steering_angle))
        self.cmd_steering_angle = clamped_angle
        self.steer_angle = clamped_angle

    def set_brake(self, brake_pct: float):
        self.cmd_brake = max(0.0, min(100.0, float(brake_pct)))

    def slow_stop(self):
        self.ad_dbs_valid = 0
        self.cmd_brake = 0.0
        self.brake_light = False
        self.cmd_speed = 0.0
        self.speed = 0.0
        self.cmd_steering_angle = 0.0
        self.steer_angle = 0.0

    def brake_stop(self):
        self.ad_dbs_valid = 1
        self.cmd_brake = 10.0
        self.cmd_speed = 0.0
        self.speed = 0.0
        self.cmd_steering_angle = 0.0
        self.steer_angle = 0.0
        self.target_gear = "P"
        self.gear = "P"

    def set_lights(
        self,
        left_turn: Optional[bool] = None,
        right_turn: Optional[bool] = None,
        head: Optional[bool] = None,
        brake: Optional[bool] = None,
    ):
        if left_turn is not None:
            self.left_turn_light = bool(left_turn)
        if right_turn is not None:
            self.right_turn_light = bool(right_turn)
        if head is not None:
            self.head_light = bool(head)
        if brake is not None:
            self.brake_light = bool(brake)

    def change_drive_mode(self, mode: str):
        clean_mode = str(mode).strip()
        is_auto = clean_mode.lower().startswith("auto")
        self.cmd_speed = 0.0
        self.cmd_steering_angle = 0.0
        self.speed = 0.0
        self.steer_angle = 0.0
        self.target_gear = "P"
        self.gear = "P"
        self.ad_control_req_flag = 1 if is_auto else 0
        self.drive_mode = "Auto Mode" if is_auto else "Remote Control Mode"
        logger.info(f"[{self.name}] Simulation drive mode changed to '{self.drive_mode}'.")

    def set_mode_remote(self):
        self.change_drive_mode("Remote")

    def set_mode_auto(self):
        self.change_drive_mode("Auto")

    def set_gear(self, gear_str: str) -> bool:
        target_g = str(gear_str).strip().upper()
        if target_g == "P" and abs(self.speed) > 0.01:
            return False
        self.target_gear = target_g
        self.gear = target_g
        return True

    def degree_to_can_cmd(self, angle_deg: float) -> int:
        clamped_deg = max(self.min_steering_angle, min(self.max_steering_angle, float(angle_deg)))
        cmd_val = int(round((clamped_deg / max(1e-6, self.max_steering_angle)) * self.MAX_CMD_VAL))
        return max(self.MIN_CMD_VAL, min(self.MAX_CMD_VAL, cmd_val))

    def update_simulation_step(self, dt: float = 0.05):
        """Update simulated pose, speed, and GPS position for autonomous driving view."""
        if not self.is_connected:
            return

        speed_m_s = (self.speed * 1000.0) / 3600.0
        wheelbase = max(0.5, self.wheelbase)

        # Steering angular velocity (+steer_angle is Left turn / CCW -> +angular_velocity)
        if abs(self.steer_angle) > 0.01:
            turning_radius = wheelbase / float(np.tan(np.radians(self.steer_angle)))
            angular_velocity = speed_m_s / turning_radius
        else:
            angular_velocity = 0.0

        self.simulated_heading += angular_velocity * dt
        self.simulated_heading = math.atan2(math.sin(self.simulated_heading), math.cos(self.simulated_heading))

        # Update GPS latitude and longitude (+X = North -> +lat, +Y = West -> -lon)
        d_lat = (speed_m_s * dt * float(np.cos(self.simulated_heading))) / 111000.0
        lon_scale = max(0.1, math.cos(math.radians(self.lat)))
        d_lon = -(speed_m_s * dt * float(np.sin(self.simulated_heading))) / (111000.0 * lon_scale)

        self.lat += d_lat
        self.lon += d_lon

    def get_status(self) -> dict:
        return {
            "name": self.name,
            "channel": "sim0",
            "connected": self.is_connected,
            "speed": self.speed,
            "steer_angle": self.steer_angle,
            "brake_pressure_cmd": self.cmd_brake,
            "left_turn_light": self.left_turn_light,
            "right_turn_light": self.right_turn_light,
            "head_light": self.head_light,
            "brake_light": self.brake_light,
            "vehicle_velocity": f"{self.speed:.1f} km/h",
            "vehicle_steer_angle": f"{self.steer_angle:.1f} deg",
            "can_cmd_val": self.degree_to_can_cmd(self.steer_angle),
            "latitude": self.lat,
            "longitude": self.lon,
            "gear": self.gear,
            "drive_mode": self.drive_mode,
            "heading": self.simulated_heading,
            "parsed_can_status": self.parsed_can_status,
        }
