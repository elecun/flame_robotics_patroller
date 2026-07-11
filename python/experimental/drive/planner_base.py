from abc import ABC, abstractmethod
import numpy as np
from typing import Tuple, List

class BaseGlobalPlanner(ABC):
    def __init__(self, waypoints: List[Tuple[float, float, float]]):
        """
        Initialize the global planner with a set of 3D waypoints.
        
        Args:
            waypoints: A list of (latitude, longitude, z) tuples representing the path.
        """
        self.waypoints = waypoints
        
    @abstractmethod
    def get_next_target(self, current_lat: float, current_lon: float, current_z: float) -> Tuple[float, float, float]:
        """
        Computes the next intermediate target along the path.
        
        Args:
            current_lat: The current latitude of the robot.
            current_lon: The current longitude of the robot.
            current_z: The current Z height of the mast.
            
        Returns:
            Tuple[float, float, float]: The (latitude, longitude, z) of the next target.
        """
        pass

class BaseLocalPlanner(ABC):
    @abstractmethod
    def compute_commands(self, 
                         target_lat: float, target_lon: float, target_z: float,
                         current_lat: float, current_lon: float, current_z: float,
                         current_heading: float,
                         current_speed: float, 
                         current_steering_angle: float,
                         current_z_speed: float,
                         lidar_data: np.ndarray) -> Tuple[float, float, float]:
        """
        Computes the speed, steering, and mast Z-speed commands.
        
        Args:
            target_lat, target_lon, target_z: Target coordinates.
            current_lat, current_lon, current_z: Current coordinates.
            current_heading: The current heading/yaw of the robot in radians.
            current_speed: The current speed of the robot (m/s).
            current_steering_angle: The current steering angle (radians).
            current_z_speed: The current speed of the mast (m/s).
            lidar_data: VLP-16 data.
            
        Returns:
            Tuple[float, float, float]: (target_steering_angle, target_speed, target_z_speed).
        """
        pass
