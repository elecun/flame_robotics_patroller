"""
FLAME Robotics Patroller - Global Path Planner Base Interface
@description Standard interface for global path planning algorithms.
              Provides base class and two concrete implementations:
              - StaticPlanner: Pre-defined waypoint-based navigation
              - DynamicPlanner: Dynamic path generation with map awareness
@author Byunghun Hwang <bh.hwnag@iae.re.kr>
"""

import math
import heapq
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import List, Optional, Tuple
from enum import Enum, auto


@dataclass
class Position:
    """2D position coordinate in meters."""
    x: float = 0.0  # x-coordinate (meters)
    y: float = 0.0  # y-coordinate (meters)

    def distance_to(self, other: "Position") -> float:
        """Calculate Euclidean distance to another position in meters."""
        return math.sqrt((self.x - other.x) ** 2 + (self.y - other.y) ** 2)

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, Position):
            return NotImplemented
        return math.isclose(self.x, other.x, abs_tol=1e-6) and \
               math.isclose(self.y, other.y, abs_tol=1e-6)

    def __hash__(self) -> int:
        return hash((round(self.x, 6), round(self.y, 6)))

    def __repr__(self) -> str:
        return f"Position(x={self.x:.3f}, y={self.y:.3f})"


class PlannerState(Enum):
    """Planner operational state."""
    IDLE = auto()        # No goal set
    PLANNING = auto()    # Computing path
    NAVIGATING = auto()  # Following path
    REACHED = auto()     # Goal reached
    ERROR = auto()       # Error state


class GlobalPlannerBase(ABC):
    """
    Abstract base class for global path planning algorithms.

    This class defines the standard interface that all global planners
    must implement. Subclasses should override abstract methods to provide
    specific planning strategies.
    """

    def __init__(self, goal_tolerance: float = 1.0):
        """
        Initialize the global planner base.

        Args:
            goal_tolerance: Distance threshold (meters) to consider the goal reached.
        """
        self._state: PlannerState = PlannerState.IDLE
        self._goal_tolerance: float = goal_tolerance
        self._current_position: Optional[Position] = None
        self._goal_position: Optional[Position] = None

    @property
    def state(self) -> PlannerState:
        """Get current planner state."""
        return self._state

    @property
    def goal_tolerance(self) -> float:
        """Get goal tolerance distance in meters."""
        return self._goal_tolerance

    @goal_tolerance.setter
    def goal_tolerance(self, value: float) -> None:
        """Set goal tolerance distance in meters."""
        if value <= 0:
            raise ValueError("Goal tolerance must be positive.")
        self._goal_tolerance = value

    @abstractmethod
    def set_goal(self, goal: Position) -> None:
        """
        Set the navigation goal.

        Args:
            goal: Target position to navigate to.
        """
        pass

    @abstractmethod
    def get_next_waypoint(self, current_position: Position) -> Optional[Position]:
        """
        Get the next waypoint to navigate toward given the current position.

        Args:
            current_position: Current robot position.

        Returns:
            Next waypoint position, or None if goal is reached or no path available.
        """
        pass

    @abstractmethod
    def reset(self) -> None:
        """Reset the planner to its initial state."""
        pass

    def is_goal_reached(self, current_position: Position) -> bool:
        """
        Check if the current position is within goal tolerance.

        Args:
            current_position: Current robot position.

        Returns:
            True if the goal is reached within tolerance.
        """
        if self._goal_position is None:
            return False
        return current_position.distance_to(self._goal_position) <= self._goal_tolerance


class StaticPlanner(GlobalPlannerBase):
    """
    Static global path planner with pre-defined waypoints.

    This planner receives all waypoint data in advance (from start to goal)
    and returns the most appropriate next waypoint based on the robot's
    current position. Waypoint selection is distance-based (meters).

    Usage:
        planner = StaticPlanner(goal_tolerance=1.0, waypoint_tolerance=2.0)
        planner.set_waypoints([Position(0,0), Position(5,0), Position(10,0)])
        planner.set_goal(Position(10, 0))
        next_wp = planner.get_next_waypoint(Position(0.5, 0.1))
    """

    def __init__(self, goal_tolerance: float = 1.0, waypoint_tolerance: float = 2.0):
        """
        Initialize the static planner.

        Args:
            goal_tolerance: Distance threshold (meters) to consider the final goal reached.
            waypoint_tolerance: Distance threshold (meters) to consider a waypoint reached,
                                triggering advance to the next waypoint.
        """
        super().__init__(goal_tolerance=goal_tolerance)
        self._waypoints: List[Position] = []
        self._waypoint_tolerance: float = waypoint_tolerance
        self._current_waypoint_index: int = 0

    @property
    def waypoint_tolerance(self) -> float:
        """Get waypoint tolerance distance in meters."""
        return self._waypoint_tolerance

    @waypoint_tolerance.setter
    def waypoint_tolerance(self, value: float) -> None:
        """Set waypoint tolerance distance in meters."""
        if value <= 0:
            raise ValueError("Waypoint tolerance must be positive.")
        self._waypoint_tolerance = value

    @property
    def waypoints(self) -> List[Position]:
        """Get the list of waypoints (read-only copy)."""
        return list(self._waypoints)

    @property
    def current_waypoint_index(self) -> int:
        """Get the index of the current target waypoint."""
        return self._current_waypoint_index

    @property
    def remaining_waypoints(self) -> int:
        """Get the number of remaining waypoints."""
        if not self._waypoints:
            return 0
        return max(0, len(self._waypoints) - self._current_waypoint_index)

    def set_waypoints(self, waypoints: List[Position]) -> None:
        """
        Set the sequence of waypoints to follow.

        This replaces any existing waypoints and resets the waypoint index.

        Args:
            waypoints: Ordered list of waypoint positions from start to goal.

        Raises:
            ValueError: If waypoints list is empty.
        """
        if not waypoints:
            raise ValueError("Waypoints list must not be empty.")

        self._waypoints = list(waypoints)
        self._current_waypoint_index = 0
        self._state = PlannerState.NAVIGATING

    def set_goal(self, goal: Position) -> None:
        """
        Set the navigation goal.

        If waypoints are already set, the goal is stored as the final
        destination for goal-reached checking. If the goal differs from
        the last waypoint, it is appended to the waypoint list.

        Args:
            goal: Target position to navigate to.
        """
        self._goal_position = goal

        # If waypoints exist and the last one differs from goal, append goal
        if self._waypoints:
            if self._waypoints[-1] != goal:
                self._waypoints.append(goal)
        else:
            # No waypoints set yet; just store the goal
            self._waypoints = [goal]
            self._current_waypoint_index = 0

        self._state = PlannerState.NAVIGATING

    def get_next_waypoint(self, current_position: Position) -> Optional[Position]:
        """
        Get the next waypoint based on the current position.

        Selection logic (distance-based in meters):
        1. If the goal is reached, return None.
        2. If the current waypoint is within waypoint_tolerance, advance
           to the next waypoint.
        3. Additionally, skip any ahead waypoints that are also within
           tolerance (handles cases where robot moved fast or jumped).
        4. Return the current target waypoint.

        Args:
            current_position: Current robot position.

        Returns:
            Next waypoint position to navigate toward, or None if goal is
            reached or no waypoints are available.
        """
        self._current_position = current_position

        if not self._waypoints:
            self._state = PlannerState.IDLE
            return None

        # Check if final goal is reached
        if self.is_goal_reached(current_position):
            self._state = PlannerState.REACHED
            return None

        # Bounds check
        if self._current_waypoint_index >= len(self._waypoints):
            self._state = PlannerState.REACHED
            return None

        # Advance past waypoints that are within tolerance
        while self._current_waypoint_index < len(self._waypoints):
            wp = self._waypoints[self._current_waypoint_index]
            dist = current_position.distance_to(wp)

            if dist <= self._waypoint_tolerance:
                self._current_waypoint_index += 1
            else:
                break

        # If we've gone past all waypoints
        if self._current_waypoint_index >= len(self._waypoints):
            self._state = PlannerState.REACHED
            return None

        self._state = PlannerState.NAVIGATING
        return self._waypoints[self._current_waypoint_index]

    def get_nearest_waypoint(self, current_position: Position) -> Optional[Position]:
        """
        Find the nearest waypoint from the remaining waypoints.

        This does NOT advance the waypoint index; it only queries.

        Args:
            current_position: Current robot position.

        Returns:
            The nearest remaining waypoint, or None if no waypoints remain.
        """
        remaining = self._waypoints[self._current_waypoint_index:]
        if not remaining:
            return None

        return min(remaining, key=lambda wp: current_position.distance_to(wp))

    def reset(self) -> None:
        """Reset the planner to its initial state, clearing all waypoints."""
        self._waypoints.clear()
        self._current_waypoint_index = 0
        self._current_position = None
        self._goal_position = None
        self._state = PlannerState.IDLE


@dataclass
class GridMap:
    """
    2D occupancy grid map for path planning.

    Coordinate system:
    - Origin (origin_x, origin_y) corresponds to grid cell [0][0].
    - Each cell has size `resolution` meters.
    - grid[row][col]: 0 = free, 1 = obstacle.
    """
    width: int          # Number of columns
    height: int         # Number of rows
    resolution: float   # Cell size in meters
    origin_x: float = 0.0  # World x-coordinate of grid origin
    origin_y: float = 0.0  # World y-coordinate of grid origin
    grid: List[List[int]] = field(default_factory=list)

    def __post_init__(self) -> None:
        """Initialize empty (free) grid if none provided."""
        if not self.grid:
            self.grid = [[0] * self.width for _ in range(self.height)]

    def world_to_grid(self, position: Position) -> Tuple[int, int]:
        """
        Convert world position to grid cell indices.

        Args:
            position: World position.

        Returns:
            (row, col) grid indices.
        """
        col = int((position.x - self.origin_x) / self.resolution)
        row = int((position.y - self.origin_y) / self.resolution)
        return (row, col)

    def grid_to_world(self, row: int, col: int) -> Position:
        """
        Convert grid cell indices to world position (cell center).

        Args:
            row: Grid row index.
            col: Grid column index.

        Returns:
            World position at the center of the grid cell.
        """
        x = self.origin_x + (col + 0.5) * self.resolution
        y = self.origin_y + (row + 0.5) * self.resolution
        return Position(x=x, y=y)

    def is_valid(self, row: int, col: int) -> bool:
        """Check if grid indices are within bounds."""
        return 0 <= row < self.height and 0 <= col < self.width

    def is_free(self, row: int, col: int) -> bool:
        """Check if a grid cell is free (not an obstacle)."""
        return self.is_valid(row, col) and self.grid[row][col] == 0

    def set_obstacle(self, row: int, col: int) -> None:
        """Mark a grid cell as obstacle."""
        if self.is_valid(row, col):
            self.grid[row][col] = 1

    def clear_obstacle(self, row: int, col: int) -> None:
        """Mark a grid cell as free."""
        if self.is_valid(row, col):
            self.grid[row][col] = 0


class DynamicPlanner(GlobalPlannerBase):
    """
    Dynamic global path planner with on-the-fly path generation.

    This planner only requires a goal position (no pre-defined waypoints).
    It generates a global path using A* search on a grid map. If the goal
    changes, the path is regenerated automatically.

    If no map is provided, the planner assumes an open (obstacle-free) space
    and generates a straight-line path with interpolated waypoints.

    Usage:
        # Open space (no map)
        planner = DynamicPlanner(goal_tolerance=1.0)
        planner.set_goal(Position(50, 30))
        next_wp = planner.get_next_waypoint(Position(0, 0))

        # With map
        grid_map = GridMap(width=100, height=100, resolution=0.5)
        planner = DynamicPlanner(goal_tolerance=1.0, grid_map=grid_map)
        planner.set_goal(Position(25, 15))
        next_wp = planner.get_next_waypoint(Position(0, 0))
    """

    def __init__(self, goal_tolerance: float = 1.0,
                 waypoint_tolerance: float = 2.0,
                 grid_map: Optional[GridMap] = None,
                 path_interval: float = 2.0):
        """
        Initialize the dynamic planner.

        Args:
            goal_tolerance: Distance threshold (meters) to consider the goal reached.
            waypoint_tolerance: Distance threshold (meters) to consider a waypoint reached.
            grid_map: Occupancy grid map. If None, open space is assumed.
            path_interval: Interval (meters) between generated waypoints in open space mode.
        """
        super().__init__(goal_tolerance=goal_tolerance)
        self._waypoint_tolerance: float = waypoint_tolerance
        self._grid_map: Optional[GridMap] = grid_map
        self._path_interval: float = path_interval
        self._planned_path: List[Position] = []
        self._current_waypoint_index: int = 0
        self._needs_replan: bool = False

    @property
    def grid_map(self) -> Optional[GridMap]:
        """Get the current grid map."""
        return self._grid_map

    @property
    def planned_path(self) -> List[Position]:
        """Get the currently planned path (read-only copy)."""
        return list(self._planned_path)

    @property
    def remaining_waypoints(self) -> int:
        """Get the number of remaining waypoints in the planned path."""
        if not self._planned_path:
            return 0
        return max(0, len(self._planned_path) - self._current_waypoint_index)

    def set_map(self, grid_map: GridMap) -> None:
        """
        Set or update the grid map.

        Triggers replanning if a goal is already set.

        Args:
            grid_map: New occupancy grid map.
        """
        self._grid_map = grid_map
        if self._goal_position is not None:
            self._needs_replan = True

    def set_goal(self, goal: Position) -> None:
        """
        Set or update the navigation goal.

        If the goal changes from the previous one, the path is marked
        for replanning. The actual replanning occurs on the next call
        to get_next_waypoint().

        Args:
            goal: Target position to navigate to.
        """
        if self._goal_position is not None and self._goal_position == goal:
            return  # Same goal, no action needed

        self._goal_position = goal
        self._needs_replan = True
        self._state = PlannerState.PLANNING

    def get_next_waypoint(self, current_position: Position) -> Optional[Position]:
        """
        Get the next waypoint based on the current position.

        If the path needs replanning (new goal or map update), the path is
        regenerated before returning the next waypoint.

        Args:
            current_position: Current robot position.

        Returns:
            Next waypoint position, or None if goal is reached or planning failed.
        """
        self._current_position = current_position

        if self._goal_position is None:
            self._state = PlannerState.IDLE
            return None

        # Check if goal is reached
        if self.is_goal_reached(current_position):
            self._state = PlannerState.REACHED
            return None

        # Replan if needed
        if self._needs_replan:
            success = self._plan_path(current_position, self._goal_position)
            self._needs_replan = False
            if not success:
                self._state = PlannerState.ERROR
                return None
            self._state = PlannerState.NAVIGATING

        # No path available
        if not self._planned_path:
            self._state = PlannerState.ERROR
            return None

        # Bounds check
        if self._current_waypoint_index >= len(self._planned_path):
            self._state = PlannerState.REACHED
            return None

        # Advance past waypoints within tolerance
        while self._current_waypoint_index < len(self._planned_path):
            wp = self._planned_path[self._current_waypoint_index]
            if current_position.distance_to(wp) <= self._waypoint_tolerance:
                self._current_waypoint_index += 1
            else:
                break

        if self._current_waypoint_index >= len(self._planned_path):
            self._state = PlannerState.REACHED
            return None

        self._state = PlannerState.NAVIGATING
        return self._planned_path[self._current_waypoint_index]

    def reset(self) -> None:
        """Reset the planner to its initial state."""
        self._planned_path.clear()
        self._current_waypoint_index = 0
        self._current_position = None
        self._goal_position = None
        self._needs_replan = False
        self._state = PlannerState.IDLE

    def _plan_path(self, start: Position, goal: Position) -> bool:
        """
        Plan a path from start to goal.

        Uses A* on the grid map if available, otherwise generates
        a straight-line path with interpolated waypoints (open space).

        Args:
            start: Start position.
            goal: Goal position.

        Returns:
            True if path planning succeeded, False otherwise.
        """
        self._planned_path.clear()
        self._current_waypoint_index = 0

        if self._grid_map is None:
            # Open space: straight-line interpolation
            self._planned_path = self._generate_straight_path(start, goal)
            return True
        else:
            # A* search on grid map
            path = self._astar_search(start, goal)
            if path is not None:
                self._planned_path = path
                return True
            return False

    def _generate_straight_path(self, start: Position, goal: Position) -> List[Position]:
        """
        Generate a straight-line path with evenly spaced waypoints.

        Args:
            start: Start position.
            goal: Goal position.

        Returns:
            List of waypoint positions from start to goal.
        """
        total_dist = start.distance_to(goal)
        if total_dist < self._path_interval:
            return [goal]

        n_segments = max(1, int(math.ceil(total_dist / self._path_interval)))
        path: List[Position] = []

        for i in range(1, n_segments + 1):
            t = i / n_segments
            x = start.x + t * (goal.x - start.x)
            y = start.y + t * (goal.y - start.y)
            path.append(Position(x=x, y=y))

        # Ensure the final waypoint is exactly the goal
        if path and path[-1] != goal:
            path[-1] = goal

        return path

    def _astar_search(self, start: Position, goal: Position) -> Optional[List[Position]]:
        """
        A* path search on the grid map.

        Args:
            start: Start position in world coordinates.
            goal: Goal position in world coordinates.

        Returns:
            List of waypoint positions from start to goal, or None if no path found.
        """
        grid_map = self._grid_map
        if grid_map is None:
            return None

        start_cell = grid_map.world_to_grid(start)
        goal_cell = grid_map.world_to_grid(goal)

        # Validate start and goal cells
        if not grid_map.is_valid(*start_cell):
            return None
        if not grid_map.is_valid(*goal_cell):
            return None
        if not grid_map.is_free(*goal_cell):
            return None

        # If start cell is occupied, try to find nearest free cell
        if not grid_map.is_free(*start_cell):
            start_cell = self._find_nearest_free_cell(start_cell)
            if start_cell is None:
                return None

        # A* algorithm
        # 8-directional movement
        directions = [
            (-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),      # cardinal
            (-1, -1, 1.414), (-1, 1, 1.414), (1, -1, 1.414), (1, 1, 1.414)  # diagonal
        ]

        open_set: List[Tuple[float, int, Tuple[int, int]]] = []
        counter = 0  # Tie-breaker for heap
        start_h = self._heuristic(start_cell, goal_cell, grid_map.resolution)
        heapq.heappush(open_set, (start_h, counter, start_cell))

        came_from: dict = {}
        g_score: dict = {start_cell: 0.0}

        while open_set:
            _, _, current = heapq.heappop(open_set)

            if current == goal_cell:
                # Reconstruct path
                return self._reconstruct_path(came_from, current, grid_map)

            for dr, dc, move_cost in directions:
                neighbor = (current[0] + dr, current[1] + dc)
                if not grid_map.is_free(*neighbor):
                    continue

                tentative_g = g_score[current] + move_cost * grid_map.resolution
                if tentative_g < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f = tentative_g + self._heuristic(neighbor, goal_cell, grid_map.resolution)
                    counter += 1
                    heapq.heappush(open_set, (f, counter, neighbor))

        return None  # No path found

    def _heuristic(self, cell: Tuple[int, int], goal: Tuple[int, int],
                   resolution: float) -> float:
        """Octile distance heuristic for 8-directional grid."""
        dr = abs(cell[0] - goal[0])
        dc = abs(cell[1] - goal[1])
        return resolution * (max(dr, dc) + (math.sqrt(2) - 1) * min(dr, dc))

    def _reconstruct_path(self, came_from: dict, current: Tuple[int, int],
                          grid_map: GridMap) -> List[Position]:
        """
        Reconstruct path from A* result and convert to world coordinates.

        Applies path simplification to reduce redundant waypoints on
        straight segments.

        Args:
            came_from: Parent map from A* search.
            current: Goal cell.
            grid_map: Grid map for coordinate conversion.

        Returns:
            Simplified list of waypoint positions in world coordinates.
        """
        # Build raw cell path
        cells: List[Tuple[int, int]] = [current]
        while current in came_from:
            current = came_from[current]
            cells.append(current)
        cells.reverse()

        # Simplify: remove collinear intermediate points
        if len(cells) <= 2:
            simplified = cells
        else:
            simplified = [cells[0]]
            for i in range(1, len(cells) - 1):
                prev = simplified[-1]
                next_cell = cells[i + 1]
                # Direction from prev to current
                d1 = (cells[i][0] - prev[0], cells[i][1] - prev[1])
                # Direction from current to next
                d2 = (next_cell[0] - cells[i][0], next_cell[1] - cells[i][1])
                if d1 != d2:
                    simplified.append(cells[i])
            simplified.append(cells[-1])

        # Convert to world coordinates (skip start cell)
        path: List[Position] = []
        for cell in simplified[1:]:
            path.append(grid_map.grid_to_world(cell[0], cell[1]))

        return path

    def _find_nearest_free_cell(self, cell: Tuple[int, int],
                                max_radius: int = 10) -> Optional[Tuple[int, int]]:
        """
        Find the nearest free cell using BFS from the given cell.

        Args:
            cell: Starting cell (row, col).
            max_radius: Maximum search radius in cells.

        Returns:
            Nearest free cell, or None if none found within radius.
        """
        grid_map = self._grid_map
        if grid_map is None:
            return None

        from collections import deque
        visited = {cell}
        queue = deque([cell])

        while queue:
            r, c = queue.popleft()
            if grid_map.is_free(r, c):
                return (r, c)

            for dr in range(-1, 2):
                for dc in range(-1, 2):
                    if dr == 0 and dc == 0:
                        continue
                    nr, nc = r + dr, c + dc
                    if (nr, nc) not in visited and grid_map.is_valid(nr, nc):
                        # Check within max_radius
                        if abs(nr - cell[0]) <= max_radius and abs(nc - cell[1]) <= max_radius:
                            visited.add((nr, nc))
                            queue.append((nr, nc))

        return None
