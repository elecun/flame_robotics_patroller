"""
Robot configuration dataclass and URDF parser module for Ackermann mobile robots.
"""
from dataclasses import dataclass, field
import os
import math
import xml.etree.ElementTree as ET
from typing import Optional, List, Tuple
from util.logger.console import ConsoleLogger

logger = ConsoleLogger.get_logger()

@dataclass
class RobotConfig:
    """
    Standard Robot Parameter Specifications for Ackermann Mobile Robot.
    Default parameters correspond to IAE Patrol Robot v1/v2 specs.
    """
    # Robot Physical Dimensions (meters)
    width: float = 1.000       # 1000 mm
    length: float = 2.055      # 2055 mm
    wheelbase: float = 1.150   # 1150 mm (Distance between front and rear axles)
    tread: float = 0.834       # 834 mm (Track width: 2 * 0.417m)
    
    # Steering and Velocity Limits
    max_steer_angle: float = math.radians(30.0)  # 28 deg = 0.4886 rad
    min_steer_angle: float = math.radians(-30.0)
    max_velocity: float = 3.0 / 3.6              # 3.0 km/h = 0.833 m/s
    min_velocity: float = 0.0                    # Forward only (or negative if reverse allowed)
    max_accel: float = 1.0                       # m/s^2
    max_steer_rate: float = math.radians(45.0)   # rad/s

    # Global Planner Constraints
    max_lat_accel: float = 1.8                   # m/s^2 (Maximum lateral acceleration for curvature speed limiting)
    ds: float = 0.1                              # Waypoint sampling resolution (meters)

    # Local Planner DWA Constraints & Weights
    lookahead_distance: float = 3.0              # Target lookahead distance for reference point and local window (meters)
    dt: float = 0.1                              # DWA simulation time step (seconds)
    predict_time: float = 4.0                    # DWA forward prediction horizon (seconds, extended to 4.0s)
    v_samples: int = 10                          # Linear velocity sample grid size
    steer_samples: int = 20                      # Steering angle sample grid size

    # DWA Cost Function Weights
    path_distance_weight: float = 1.0            # Heading / path tracking error weight
    obstacle_weight: float = 1.5                # Obstacle clearance weight
    velocity_weight: float = 1.0                # Target velocity tracking weight
    steer_smoothness_weight: float = 0.5         # Steering angle change penalty weight
    corridor_weight: float = 5.0                 # Corridor boundary constraint weight
    
    # Safety margins
    inflation_radius: float = 0.15                # Footprint inflation margin (meters)

    # Oriented Bounding Box Footprint relative to rear-axle center or robot base_link
    # Default: base_link is center of robot (x in [-length/2, length/2], y in [-width/2, width/2])
    footprint_offsets: List[Tuple[float, float]] = field(default_factory=lambda: [
        (1.0275, 0.500),   # Front Left
        (1.0275, -0.500),  # Front Right
        (-1.0275, -0.500), # Rear Right
        (-1.0275, 0.500)   # Rear Left
    ])

    def update_from_config(self, config: any) -> None:
        """
        Update RobotConfig parameters from ConfigParser object (e.g. apros.cfg).
        Supports section [ackermann_dwa_local_planner] and fallback [mobile_drive_s1].
        """
        if not config:
            return

        sec = None
        if config.has_section("ackermann_dwa_local_planner"):
            sec = config["ackermann_dwa_local_planner"]
        elif config.has_section("mobile_drive_s1"):
            sec = config["mobile_drive_s1"]

        if not sec:
            return

        if "predict_time" in sec:
            self.predict_time = float(sec["predict_time"])
        if "dt" in sec:
            self.dt = float(sec["dt"])
        if "v_samples" in sec:
            self.v_samples = int(sec["v_samples"])
        if "steer_samples" in sec:
            self.steer_samples = int(sec["steer_samples"])
        if "lookahead_distance" in sec:
            self.lookahead_distance = float(sec["lookahead_distance"])

        if "min_velocity" in sec:
            val = float(sec["min_velocity"])
            self.min_velocity = val / 3.6 if val > 0.5 else val
        if "max_velocity" in sec:
            val = float(sec["max_velocity"])
            self.max_velocity = val / 3.6 if val > 0.5 else val
        if "max_accel" in sec:
            self.max_accel = float(sec["max_accel"])

        if "min_steer_angle" in sec:
            val = float(sec["min_steer_angle"])
            self.min_steer_angle = math.radians(val) if abs(val) > 1.5 else val
        if "max_steer_angle" in sec or "max_steering_angle" in sec:
            val_str = sec.get("max_steer_angle", sec.get("max_steering_angle"))
            val = float(val_str)
            self.max_steer_angle = math.radians(val) if abs(val) > 1.5 else val
        if "max_steer_rate" in sec:
            val = float(sec["max_steer_rate"])
            self.max_steer_rate = math.radians(val) if abs(val) > 1.5 else val

        if "inflation_radius" in sec:
            self.inflation_radius = float(sec["inflation_radius"])
        if "path_distance_weight" in sec:
            self.path_distance_weight = float(sec["path_distance_weight"])
        if "obstacle_weight" in sec:
            self.obstacle_weight = float(sec["obstacle_weight"])
        if "velocity_weight" in sec:
            self.velocity_weight = float(sec["velocity_weight"])
        if "steer_smoothness_weight" in sec:
            self.steer_smoothness_weight = float(sec["steer_smoothness_weight"])
        if "corridor_weight" in sec:
            self.corridor_weight = float(sec["corridor_weight"])

        logger.info(
            f"[RobotConfig] Updated from apros.cfg: predict_time={self.predict_time:.1f}s, dt={self.dt:.2f}s, "
            f"lookahead={self.lookahead_distance:.1f}m, max_v={self.max_velocity * 3.6:.1f}km/h, "
            f"max_steer={math.degrees(self.max_steer_angle):.1f}deg, weights(path={self.path_distance_weight}, "
            f"obs={self.obstacle_weight}, vel={self.velocity_weight}, steer_smooth={self.steer_smoothness_weight}, corridor={self.corridor_weight})"
        )

    @classmethod
    def from_urdf(cls, urdf_path: str) -> "RobotConfig":
        """
        Parse robot parameters from URDF file.
        If parsing fails or file is missing, fallback to default RobotConfig parameters.
        """
        config = cls()
        if not os.path.exists(urdf_path):
            logger.warning(f"[RobotConfig] URDF file not found at '{urdf_path}'. Using default config parameters.")
            return config

        try:
            tree = ET.parse(urdf_path)
            root = tree.getroot()

            # 1. Parse Steering Angle Limit from 'front_steer_joint'
            for joint in root.findall("joint"):
                jname = joint.get("name", "")
                if jname in ("front_steer_joint", "steer_joint"):
                    limit = joint.find("limit")
                    if limit is not None:
                        upper = limit.get("upper")
                        lower = limit.get("lower")
                        if upper is not None:
                            config.max_steer_angle = abs(float(upper))
                        if lower is not None:
                            config.min_steer_angle = float(lower)
                        logger.info(f"[RobotConfig] Parsed max_steer_angle: {math.degrees(config.max_steer_angle):.1f} deg from URDF.")

            # 2. Parse Chassis Box Dimensions & Footprint from 'base_link' visual/collision boxes
            base_link = None
            for link in root.findall("link"):
                if link.get("name") == "base_link":
                    base_link = link
                    break

            if base_link is not None:
                max_l, max_w = 0.0, 0.0
                for elem in list(base_link.findall("visual")) + list(base_link.findall("collision")):
                    geom = elem.find("geometry")
                    if geom is not None:
                        box = geom.find("box")
                        if box is not None:
                            size_str = box.get("size")
                            if size_str:
                                dims = [float(s) for s in size_str.split()]
                                if len(dims) >= 2:
                                    max_l = max(max_l, dims[0])
                                    max_w = max(max_w, dims[1])
                if max_l > 0.0:
                    config.length = max_l
                if max_w > 0.0:
                    config.width = max_w

            # 3. Parse Wheelbase & Tread from Wheel Joint origins
            # Wheelbase L = Front wheel X - Rear wheel X
            # Tread = Front Left Y - Front Right Y
            fx, rx = None, None
            fly, fry = None, None
            for joint in root.findall("joint"):
                jname = joint.get("name", "")
                origin = joint.find("origin")
                if origin is not None:
                    xyz_str = origin.get("xyz")
                    if xyz_str:
                        xyz = [float(val) for val in xyz_str.split()]
                        if "front_" in jname and "wheel" in jname:
                            fx = xyz[0]
                            if "left" in jname:
                                fly = xyz[1]
                            elif "right" in jname:
                                fry = xyz[1]
                        elif "rear_" in jname and "wheel" in jname:
                            rx = xyz[0]

            if fx is not None and rx is not None:
                config.wheelbase = abs(fx - rx)
            if fly is not None and fry is not None:
                config.tread = abs(fly - fry)

            # Update Footprint Offsets from parsed length and width
            half_l = config.length / 2.0
            half_w = config.width / 2.0
            config.footprint_offsets = [
                (half_l, half_w),    # Front Left
                (half_l, -half_w),   # Front Right
                (-half_l, -half_w),  # Rear Right
                (-half_l, half_w)    # Rear Left
            ]

            logger.info(f"[RobotConfig] Successfully parsed URDF ('{os.path.basename(urdf_path)}'): L={config.wheelbase:.2f}m, W={config.width:.2f}m, Length={config.length:.2f}m, Tread={config.tread:.2f}m, SteerMax={math.degrees(config.max_steer_angle):.1f}deg")

        except Exception as e:
            logger.error(f"[RobotConfig] Error parsing URDF file '{urdf_path}': {e}. Using fallback default config.")

        return config
