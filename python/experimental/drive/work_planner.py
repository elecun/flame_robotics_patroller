from typing import List, Tuple
from .global_planners import WaypointFollower # or accept any global planner
from .planner_base import BaseGlobalPlanner

class WorkPlanner:
    """
    High-level Work Planner that manages a sequence of 3D tasks.
    Each task is a tuple of (Latitude, Longitude, Z_Height).
    It feeds these tasks to the Global Planner.
    """
    def __init__(self, tasks: List[Tuple[float, float, float]], global_planner: BaseGlobalPlanner):
        self.tasks = tasks
        self.global_planner = global_planner
        # Initialize the global planner with the tasks
        self.global_planner.waypoints = self.tasks
        
    def get_current_task(self) -> Tuple[float, float, float]:
        """Returns the current target task/waypoint from the global planner."""
        # For simplicity, we just use the global planner's logic to fetch the next target
        pass 
    
    def step(self, current_lat: float, current_lon: float, current_z: float) -> Tuple[float, float, float]:
        """
        Computes the next target for the local planner.
        Delegates the logic to the global planner to interpolate or pick the next waypoint.
        """
        return self.global_planner.get_next_target(current_lat, current_lon, current_z)
