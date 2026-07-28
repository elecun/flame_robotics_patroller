"""
A* (A-Star) Global Path Planner Plugin implementation for APROS (core/plugin/astar_planner.py).
Inherits from BaseGlobalPathPlanner and implements PythonRobotics 2D A* grid search algorithm.
"""

import math
import heapq
from typing import List, Tuple, Optional
import numpy as np
from core.plugin.global_path_planner import BaseGlobalPathPlanner


class Node:
    def __init__(self, x: int, y: int, cost: float, parent_index: int):
        self.x = x  # grid index x
        self.y = y  # grid index y
        self.cost = cost
        self.parent_index = parent_index

    def __lt__(self, other):
        return self.cost < other.cost


class AStarPlanner(BaseGlobalPathPlanner):
    """
    2D A* (A-Star) Grid Path Planner Plugin.
    Computes shortest obstacle-avoiding global path using 8-connectivity grid search.
    """

    def __init__(self, name: str = "astar_planner", grid_size: float = 0.5, robot_radius: float = 0.5):
        super().__init__(name)
        self.grid_size = float(grid_size)  # grid resolution [m]
        self.robot_radius = float(robot_radius)  # robot safety clearance radius [m]
        self.motion = self._get_motion_model()

        # Internal grid map boundaries
        self.min_x = 0
        self.min_y = 0
        self.max_x = 0
        self.max_y = 0
        self.x_width = 0
        self.y_width = 0
        self.obstacle_map = []

    def initialize(self, config: dict) -> bool:
        super().initialize(config)
        self.grid_size = float(config.get("grid_size", self.grid_size))
        self.robot_radius = float(config.get("robot_radius", self.robot_radius))
        return True

    def _get_motion_model(self) -> List[List[float]]:
        # 8-connectivity grid motions: [dx, dy, cost]
        return [
            [1, 0, 1.0],
            [0, 1, 1.0],
            [-1, 0, 1.0],
            [0, -1, 1.0],
            [-1, -1, math.sqrt(2)],
            [-1, 1, math.sqrt(2)],
            [1, -1, math.sqrt(2)],
            [1, 1, math.sqrt(2)]
        ]

    def _calc_obstacle_map(self, ox: List[float], oy: List[float]):
        self.min_x = round(min(ox))
        self.min_y = round(min(oy))
        self.max_x = round(max(ox))
        self.max_y = round(max(oy))

        self.x_width = round((self.max_x - self.min_x) / self.grid_size)
        self.y_width = round((self.max_y - self.min_y) / self.grid_size)

        # Distance-based obstacle map generation with robot clearance radius
        self.obstacle_map = [[False for _ in range(self.y_width + 1)] for _ in range(self.x_width + 1)]
        for ix in range(self.x_width + 1):
            x = self._calc_grid_position(ix, self.min_x)
            for iy in range(self.y_width + 1):
                y = self._calc_grid_position(iy, self.min_y)
                for iox, ioy in zip(ox, oy):
                    d = math.hypot(iox - x, ioy - y)
                    if d <= self.robot_radius:
                        self.obstacle_map[ix][iy] = True
                        break

    def _calc_grid_position(self, index: int, min_position: float) -> float:
        return index * self.grid_size + min_position

    def _calc_xy_index(self, position: float, min_pos: float) -> int:
        return round((position - min_pos) / self.grid_size)

    def _calc_grid_index(self, node: Node) -> int:
        return node.y * self.x_width + node.x

    def _verify_node(self, node: Node) -> bool:
        px = self._calc_grid_position(node.x, self.min_x)
        py = self._calc_grid_position(node.y, self.min_y)

        if px < self.min_x or py < self.min_y or px >= self.max_x or py >= self.max_y:
            return False

        # Collision check against obstacle map boundary
        if self.obstacle_map[node.x][node.y]:
            return False

        return True

    @staticmethod
    def _heuristic(n1: Node, n2: Node) -> float:
        """Euclidean distance heuristic function."""
        w = 1.0  # weight of heuristic
        return w * math.hypot(n1.x - n2.x, n1.y - n2.y)

    def plan(
        self,
        start: Tuple[float, float],
        goal: Tuple[float, float],
        ox: Optional[List[float]] = None,
        oy: Optional[List[float]] = None
    ) -> Tuple[Optional[List[float]], Optional[List[float]]]:
        """
        Compute A* shortest path from start (sx, sy) to goal (gx, gy).
        """
        sx, sy = start
        gx, gy = goal

        # Default rectangular boundary obstacles if none provided
        if ox is None or oy is None or len(ox) == 0:
            ox, oy = [], []
            for i in range(-15, 15):
                ox.append(i)
                oy.append(-15.0)
                ox.append(i)
                oy.append(15.0)
                ox.append(-15.0)
                oy.append(i)
                ox.append(15.0)
                oy.append(i)

        self._calc_obstacle_map(ox, oy)

        start_node = Node(
            self._calc_xy_index(sx, self.min_x),
            self._calc_xy_index(sy, self.min_y),
            0.0,
            -1
        )
        goal_node = Node(
            self._calc_xy_index(gx, self.min_x),
            self._calc_xy_index(gy, self.min_y),
            0.0,
            -1
        )

        open_set: dict = {}
        closed_set: dict = {}
        pq: List[Tuple[float, int]] = []

        start_index = self._calc_grid_index(start_node)
        open_set[start_index] = start_node
        heapq.heappush(pq, (start_node.cost + self._heuristic(start_node, goal_node), start_index))

        while pq:
            _, current_id = heapq.heappop(pq)

            if current_id in closed_set:
                continue

            current = open_set[current_id]

            # Goal check
            if current.x == goal_node.x and current.y == goal_node.y:
                goal_node.parent_index = current.parent_index
                goal_node.cost = current.cost
                break

            closed_set[current_id] = current

            # Expand 8-neighbor nodes
            for move in self.motion:
                node = Node(
                    current.x + move[0],
                    current.y + move[1],
                    current.cost + move[2],
                    current_id
                )
                node_id = self._calc_grid_index(node)

                if not self._verify_node(node):
                    continue

                if node_id in closed_set:
                    continue

                if node_id not in open_set or open_set[node_id].cost > node.cost:
                    open_set[node_id] = node
                    f_cost = node.cost + self._heuristic(node, goal_node)
                    heapq.heappush(pq, (f_cost, node_id))

        # Reconstruct path from goal_node back to start_node
        if goal_node.parent_index == -1 and not (start_node.x == goal_node.x and start_node.y == goal_node.y):
            print(f"[{self.name}] Path not found!")
            return None, None

        rx, ry = [self._calc_grid_position(goal_node.x, self.min_x)], [self._calc_grid_position(goal_node.y, self.min_y)]
        parent_id = goal_node.parent_index
        while parent_id != -1:
            n = closed_set.get(parent_id) or open_set.get(parent_id)
            if n is None:
                break
            rx.append(self._calc_grid_position(n.x, self.min_x))
            ry.append(self._calc_grid_position(n.y, self.min_y))
            parent_id = n.parent_index

        # Reverse path to list from start to goal
        rx.reverse()
        ry.reverse()

        return rx, ry
