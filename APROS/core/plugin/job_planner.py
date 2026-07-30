"""
JobPlanner module for APROS plugin system (core/plugin/job_planner.py).
Manages multi-waypoint patrol mission jobs, task queues, and waypoint dispatching.
"""

from typing import List, Dict, Any, Optional, Tuple
from enum import Enum, auto
from core.plugin.base import BasePlugin
from util.logger.console import ConsoleLogger

logger = ConsoleLogger.get_logger()


class MissionState(Enum):
    IDLE = auto()
    RUNNING = auto()
    PAUSED = auto()
    COMPLETED = auto()
    FAILED = auto()


class JobPlanner(BasePlugin):
    """
    Job Planner Plugin.
    Manages high-level patrol tasks, waypoint queue sequences, and mission status execution.
    """

    def __init__(self, name: str = "job_planner"):
        super().__init__(name)
        self.state = MissionState.IDLE
        self.waypoints: List[Tuple[float, float]] = []
        self.current_waypoint_index: int = 0
        self.reach_threshold: float = 0.5  # meters

    def initialize(self, config: dict) -> bool:
        """Initialize JobPlanner configuration."""
        self.reach_threshold = float(config.get("reach_threshold", self.reach_threshold))
        logger.info(f"[{self.name}] Initialized JobPlanner (reach_threshold: {self.reach_threshold}m)")
        return True

    def set_mission(self, waypoints: List[Tuple[float, float]]):
        """Set new patrol mission waypoint sequence."""
        self.waypoints = waypoints
        self.current_waypoint_index = 0
        self.state = MissionState.IDLE
        logger.info(f"[{self.name}] Set new mission with {len(waypoints)} waypoints.")

    def start_mission(self) -> bool:
        """Start or resume mission execution."""
        if not self.waypoints:
            logger.warning(f"[{self.name}] Cannot start mission: Waypoint queue is empty.")
            return False
        self.state = MissionState.RUNNING
        logger.info(f"[{self.name}] Mission started.")
        return True

    def pause_mission(self):
        """Pause running mission."""
        if self.state == MissionState.RUNNING:
            self.state = MissionState.PAUSED
            logger.info(f"[{self.name}] Mission paused.")

    def stop_mission(self):
        """Stop and reset current mission."""
        self.state = MissionState.IDLE
        self.current_waypoint_index = 0
        logger.info(f"[{self.name}] Mission stopped and reset.")

    def get_current_target(self) -> Optional[Tuple[float, float]]:
        """Retrieve current target waypoint (x, y) if mission is running."""
        if self.state == MissionState.RUNNING and 0 <= self.current_waypoint_index < len(self.waypoints):
            return self.waypoints[self.current_waypoint_index]
        return None

    def update_robot_position(self, robot_x: float, robot_y: float) -> Optional[Tuple[float, float]]:
        """
        Update current robot position and check if target waypoint is reached.
        Returns the next target waypoint tuple (x, y) or None if mission is finished.
        """
        if self.state != MissionState.RUNNING:
            return None

        target = self.get_current_target()
        if target is None:
            self.state = MissionState.COMPLETED
            logger.info(f"[{self.name}] Mission COMPLETED successfully!")
            return None

        # Calculate distance to current target waypoint
        dist = ((robot_x - target[0])**2 + (robot_y - target[1])**2)**0.5
        if dist <= self.reach_threshold:
            logger.info(f"[{self.name}] Reached Waypoint #{self.current_waypoint_index + 1} ({target[0]:.2f}, {target[1]:.2f})")
            self.current_waypoint_index += 1

            if self.current_waypoint_index >= len(self.waypoints):
                self.state = MissionState.COMPLETED
                logger.info(f"[{self.name}] All waypoints reached. Mission COMPLETED.")
                return None

        return self.get_current_target()

    def process(self, data: dict) -> dict:
        """Process job planner updates."""
        robot_x = data.get("robot_x", 0.0)
        robot_y = data.get("robot_y", 0.0)
        target = self.update_robot_position(robot_x, robot_y)

        return {
            "state": self.state.name,
            "current_index": self.current_waypoint_index,
            "total_waypoints": len(self.waypoints),
            "target_waypoint": target
        }
