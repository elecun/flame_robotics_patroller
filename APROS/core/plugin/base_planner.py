"""
Global Planner Base module containing abstract base classes for global path planning strategies.
"""

from abc import ABC, abstractmethod
from typing import List, Tuple, Optional, Any
from core.plugin.base import BasePlugin


class GlobalPlannerBase(BasePlugin):
    """
    Abstract Base Class for Global Path Planners.
    Provides a common interface for all global path planning algorithms.
    """

    def __init__(self, name: str = "global_planner"):
        super().__init__(name)
        self.goal: Optional[Any] = None

    def initialize(self, config: dict) -> bool:
        """Initialize planner configuration."""
        return True

    def process(self, data: dict) -> dict:
        """Process plugin data."""
        return {}

    @abstractmethod
    def set_goal(self, goal: Any) -> None:
        """Set the final goal destination."""
        pass

    @abstractmethod
    def get_next_waypoint(self) -> Optional[Any]:
        """Retrieve the next target waypoint along the plan."""
        pass


class StaticGlobalPlanner(GlobalPlannerBase):
    """
    Static Global Planner.
    Handles global planning where all waypoints are injected externally beforehand.
    """

    def __init__(self, name: str = "static_global_planner"):
        super().__init__(name)
        self.waypoints: List[Any] = []
        self.current_index: int = 0

    def set_goal(self, goal: Any) -> None:
        """Set the final goal position."""
        self.goal = goal

    def set_waypoints(self, waypoints: List[Any]) -> None:
        """Inject a pre-defined list of waypoints."""
        self.waypoints = waypoints
        self.current_index = 0

    def get_next_waypoint(self) -> Optional[Any]:
        """Return the next waypoint from the static list in sequential order."""
        if self.current_index < len(self.waypoints):
            waypoint = self.waypoints[self.current_index]
            self.current_index += 1
            return waypoint
        return None


class DynamicGlobalPlanner(GlobalPlannerBase):
    """
    Dynamic Global Planner.
    Handles global planning where path/waypoints are dynamically computed based on a target goal.
    """

    def __init__(self, name: str = "dynamic_global_planner"):
        super().__init__(name)
        self.path: List[Any] = []
        self.current_index: int = 0

    def set_goal(self, goal: Any) -> None:
        """Set the target goal position."""
        self.goal = goal

    @abstractmethod
    def find_path(self, start: Any, goal: Any) -> List[Any]:
        """
        Abstract method to dynamically compute a path from start to goal position.
        :param start: Starting position
        :param goal: Target goal position
        :return: List of generated waypoints
        """
        pass

    def get_next_waypoint(self) -> Optional[Any]:
        """Return the next waypoint along the dynamically generated path."""
        if self.current_index < len(self.path):
            waypoint = self.path[self.current_index]
            self.current_index += 1
            return waypoint
        return None
