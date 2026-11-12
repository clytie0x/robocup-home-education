"""Moving the robot: follow a person, or go to a point, without touching obstacles.

The robot has no map. Every plan is made only from the shapes that perception sees now.
We lay a grid over the area around the robot and the goal, mark the cells that are too
close to a shape, and search for the shortest free path with A*.
"""

import heapq
import math

from hmc.world import ROBOT_RADIUS, Point, Robot, Shape, distance

FOLLOW_DISTANCE = 1.5  # metres to keep behind the operator
GOAL_TOLERANCE = 0.5  # metres, "close enough" for go_to

CELL = 0.2  # metres, size of one grid cell
SAFETY = 0.15  # metres of extra room, because the robot cuts corners between cells
INFLATE = ROBOT_RADIUS + SAFETY  # a cell is blocked when a shape is closer than this
MARGIN = 3.0  # metres of grid around the robot and the goal
REPLAN_EVERY = 0.5  # seconds
REPLAN_IF_GOAL_MOVED = 0.3  # metres
WAYPOINT_REACHED = 0.15  # metres

Cell = tuple[int, int]


def plan_path(start: Point, goal: Point, shapes: list[Shape]) -> list[Point] | None:
    """Shortest free path from start to goal as a list of points, or None if there is none.

    If the goal itself is blocked (for example a person we want to walk behind), the
    path ends at the free cell closest to the goal instead.
    """
    x0 = min(start[0], goal[0]) - MARGIN
    y0 = min(start[1], goal[1]) - MARGIN
    width = int((max(start[0], goal[0]) + MARGIN - x0) / CELL) + 1
    height = int((max(start[1], goal[1]) + MARGIN - y0) / CELL) + 1

    def cell_of(p: Point) -> Cell:
        return (int(round((p[0] - x0) / CELL)), int(round((p[1] - y0) / CELL)))

    def centre(c: Cell) -> Point:
        return (x0 + c[0] * CELL, y0 + c[1] * CELL)

    def inside(c: Cell) -> bool:
        return 0 <= c[0] < width and 0 <= c[1] < height

    # Mark blocked cells. Only look at the cells near each shape, which keeps it fast.
    blocked: set[Cell] = set()
    for shape in shapes:
        bx0, by0, bx1, by1 = shape.bounds()
        lo = cell_of((bx0 - INFLATE, by0 - INFLATE))
        hi = cell_of((bx1 + INFLATE, by1 + INFLATE))
        for i in range(max(lo[0], 0), min(hi[0], width - 1) + 1):
            for j in range(max(lo[1], 0), min(hi[1], height - 1) + 1):
                if shape.clearance(centre((i, j))) < INFLATE:
                    blocked.add((i, j))

    start_cell = cell_of(start)
    blocked.discard(start_cell)  # we are already here, so we must be able to leave
    goal_cell = cell_of(goal)
    if goal_cell in blocked:
        free = [
            (i, j) for i in range(width) for j in range(height) if (i, j) not in blocked
        ]
        goal_cell = min(free, key=lambda c: (distance(centre(c), goal), c))

    # A* search. Steps go to the 8 neighbours. A diagonal step needs both side cells free.
    came_from: dict[Cell, Cell] = {}
    cost = {start_cell: 0.0}
    frontier = [(distance(start, centre(goal_cell)), start_cell)]
    while frontier:
        _, current = heapq.heappop(frontier)
        if current == goal_cell:
            path = [current]
            while path[-1] in came_from:
                path.append(came_from[path[-1]])
            return [centre(c) for c in reversed(path[:-1])]  # drop the start cell
        for di in (-1, 0, 1):
            for dj in (-1, 0, 1):
                nxt = (current[0] + di, current[1] + dj)
                if (di, dj) == (0, 0) or not inside(nxt) or nxt in blocked:
                    continue
                if di and dj and ((current[0] + di, current[1]) in blocked
                                  or (current[0], current[1] + dj) in blocked):  # fmt: skip
                    continue
                new_cost = cost[current] + CELL * math.hypot(di, dj)
                if new_cost < cost.get(nxt, math.inf):
                    cost[nxt] = new_cost
                    came_from[nxt] = current
                    guess = new_cost + distance(centre(nxt), centre(goal_cell))
                    heapq.heappush(frontier, (guess, nxt))
    return None


class Navigator:
    """Drives the robot along a planned path and replans when the world changes."""

    def __init__(self) -> None:
        self.path: list[Point] = []
        self._goal: Point | None = None
        self._shape_count = -1
        self._planned_at = -math.inf

    def _needs_plan(self, goal: Point, shapes: list[Shape], now: float) -> bool:
        return (
            self._goal is None
            or distance(goal, self._goal) > REPLAN_IF_GOAL_MOVED
            or len(shapes) != self._shape_count  # we just saw something new
            or now - self._planned_at >= REPLAN_EVERY - 1e-9
        )

    def step(
        self,
        robot: Robot,
        goal: Point,
        shapes: list[Shape],
        now: float,
        dt: float,
        stop_distance: float = GOAL_TOLERANCE,
    ) -> bool:
        """One step toward the goal. Returns True once we are within stop_distance."""
        if distance(robot.pos, goal) <= stop_distance + 1e-9:
            return True
        if self._needs_plan(goal, shapes, now):
            self.path = plan_path(robot.pos, goal, shapes) or []  # no path: wait here
            self._goal, self._shape_count, self._planned_at = goal, len(shapes), now
        while self.path and distance(robot.pos, self.path[0]) <= WAYPOINT_REACHED:
            self.path.pop(0)
        if self.path:
            robot.drive_toward(self.path[0], dt)
        return distance(robot.pos, goal) <= stop_distance + 1e-9


def follow_step(navigator: Navigator, robot: Robot, operator_pos: Point,
                shapes: list[Shape], now: float, dt: float) -> bool:  # fmt: skip
    """One step of following: close the gap, but stop FOLLOW_DISTANCE away."""
    return navigator.step(robot, operator_pos, shapes, now, dt, FOLLOW_DISTANCE)
