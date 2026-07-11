import math
from typing import Tuple, List
from .planner_base import BaseGlobalPlanner
from .utils import haversine_distance

class WaypointFollower(BaseGlobalPlanner):
    """
    A simple global planner that sequentially targets the next 3D waypoint.
    """
    def __init__(self, waypoints: List[Tuple[float, float, float]], threshold_distance: float = 2.0):
        super().__init__(waypoints)
        self.threshold_distance = threshold_distance
        self.current_target_index = 0

    def get_next_target(self, current_lat: float, current_lon: float, current_z: float) -> Tuple[float, float, float]:
        if not self.waypoints:
            return current_lat, current_lon, current_z
            
        if self.current_target_index >= len(self.waypoints):
            return self.waypoints[-1]
            
        target_lat, target_lon, target_z = self.waypoints[self.current_target_index]
        
        # Calculate 2D distance for waypoint progression
        distance = haversine_distance(current_lat, current_lon, target_lat, target_lon)
        
        # If we are close enough to the current target, move to the next
        if distance < self.threshold_distance:
            self.current_target_index += 1
            if self.current_target_index < len(self.waypoints):
                target_lat, target_lon, target_z = self.waypoints[self.current_target_index]
                
        return target_lat, target_lon, target_z


class LookaheadPathTracker(BaseGlobalPlanner):
    """
    Finds the closest point on the path and looks ahead by a fixed distance.
    """
    def __init__(self, waypoints: List[Tuple[float, float, float]], lookahead_distance: float = 5.0):
        super().__init__(waypoints)
        self.lookahead_distance = lookahead_distance
        
    def get_next_target(self, current_lat: float, current_lon: float, current_z: float) -> Tuple[float, float, float]:
        if not self.waypoints:
            return current_lat, current_lon, current_z
            
        min_dist = float('inf')
        closest_idx = 0
        
        for i, (wlat, wlon, wz) in enumerate(self.waypoints):
            dist = haversine_distance(current_lat, current_lon, wlat, wlon)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
                
        target_idx = closest_idx
        for i in range(closest_idx, len(self.waypoints)):
            wlat, wlon, wz = self.waypoints[i]
            dist = haversine_distance(current_lat, current_lon, wlat, wlon)
            if dist >= self.lookahead_distance:
                target_idx = i
                break
                
        if target_idx == closest_idx and target_idx == len(self.waypoints) - 1:
            return self.waypoints[-1]
            
        return self.waypoints[target_idx]
