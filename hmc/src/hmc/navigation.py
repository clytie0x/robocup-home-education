"""Moving the robot: follow a person, or go to a point, without touching obstacles.

The robot has no map. Every plan is made only from the shapes that perception sees now.
We lay a grid over the area around the robot and the goal, mark the cells that are too
close to a shape, and search for the shortest free path with A*.
"""

import heapq
import math

from hmc import world as hmc_world
from hmc.world import Point, Robot, Shape, distance

FOLLOW_DISTANCE = 1.5  # metres to keep behind the operator
GOAL_TOLERANCE = 0.5  # metres, "close enough" for go_to

CELL = 0.1  # metres, size of one grid cell
# Extra room, because the robot cuts corners between cells. Small enough that the robot
# still fits through a 0.915 m door, the width the rulebook recommends.
SAFETY = 0.1
# A cell is blocked when a shape is closer than this. This is the simulation's value:
# the code reads hmc_world.ROBOT_RADIUS when it runs, so the robot can set its own.
INFLATE = hmc_world.ROBOT_RADIUS + SAFETY
MARGIN = 3.0  # metres of grid around the robot and the goal
REPLAN_EVERY = 0.5  # seconds
REPLAN_IF_GOAL_MOVED = 0.3  # metres
WAYPOINT_REACHED = 0.15  # metres
# We aim at a waypoint up to this far ahead. The base cannot change its turn rate at
# once, so aiming at the next cell, just in front of it, would make it wobble.
AIM_AHEAD = 0.4  # metres
STOP_MARGIN = 0.02  # metres left between the robot and a shape after a full stop
LOOK_AHEAD = 1.5  # metres; more than the 1.0 m the robot needs to stop from 1 m/s

Cell = tuple[int, int]


def plan_path(start: Point, goal: Point, shapes: list[Shape]) -> list[Point] | None:
    """Shortest free path from start to goal as a list of points, or None if there is none.

    If the goal itself is blocked (for example a person we want to walk behind), the
    path ends at the free cell closest to the goal instead.
    """
    inflate = hmc_world.ROBOT_RADIUS + SAFETY
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
        lo = cell_of((bx0 - inflate, by0 - inflate))
        hi = cell_of((bx1 + inflate, by1 + inflate))
        for i in range(max(lo[0], 0), min(hi[0], width - 1) + 1):
            for j in range(max(lo[1], 0), min(hi[1], height - 1) + 1):
                if shape.clearance(centre((i, j))) < inflate:
                    blocked.add((i, j))

    # If we ended up inside the safety zone, there would be no way out. So free the cells
    # around us that are no closer to anything than we are now: we may only move away.
    start_cell = cell_of(start)
    here_clear = min((s.clearance(start) for s in shapes), default=math.inf)
    if here_clear < inflate:
        reach = int(inflate / CELL) + 1
        for i in range(start_cell[0] - reach, start_cell[0] + reach + 1):
            for j in range(start_cell[1] - reach, start_cell[1] + reach + 1):
                c = centre((i, j))
                if distance(c, start) <= inflate and all(
                    s.clearance(c) >= here_clear for s in shapes
                ):
                    blocked.discard((i, j))
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
            robot.brake(dt)
            return True
        if self._needs_plan(goal, shapes, now):
            self.path = plan_path(robot.pos, goal, shapes) or []  # no path: wait here
            self._goal, self._shape_count, self._planned_at = goal, len(shapes), now
        # Keep the last waypoint, the goal itself, until we are there.
        while len(self.path) > 1 and distance(robot.pos, self.path[0]) <= WAYPOINT_REACHED:
            self.path.pop(0)
        if self.path:
            aim = self.path[0]
            for point in self.path:
                if distance(robot.pos, point) > AIM_AHEAD:
                    break
                aim = point
            # Brake in time for the goal, not for every waypoint on the way. The robot
            # also needs room to stop when the plan changes (it rolls on while it brakes),
            # so it slows down when something is close in front of it, both where it
            # faces now and where it is turning to.
            free_there = free_ahead(robot, shapes, _heading_to(robot, aim))
            if free_there == 0.0:
                # The straight line to the far waypoint cuts too close to something:
                # aim at the next waypoint instead, the planner says it is free.
                aim = self.path[0]
                free_there = free_ahead(robot, shapes, _heading_to(robot, aim))
            stop_in = min(
                distance(robot.pos, goal) - stop_distance,
                free_ahead(robot, shapes, robot.heading),
                free_there,
            )
            robot.drive_toward(aim, dt, stop_in=stop_in)
        else:
            robot.brake(dt)
        return distance(robot.pos, goal) <= stop_distance + 1e-9


def _heading_to(robot: Robot, point: Point) -> float:
    return math.atan2(point[1] - robot.y, point[0] - robot.x)


def free_ahead(robot: Robot, shapes: list[Shape], heading: float) -> float:
    """How far the robot can roll straight on in this heading before it gets too close.

    A shape only counts when rolling on brings us closer to it, so we can still drive
    along a wall or away from something we are already close to (like the planner).
    """
    free = 0.0
    here = [s.clearance(robot.pos) for s in shapes]
    while free < LOOK_AHEAD:
        x = robot.x + math.cos(heading) * (free + CELL)
        y = robot.y + math.sin(heading) * (free + CELL)
        for shape, now in zip(shapes, here):
            there = shape.clearance((x, y))
            if there <= hmc_world.ROBOT_RADIUS + STOP_MARGIN and there < now:
                return free
        free += CELL
    return free


def follow_step(navigator: Navigator, robot: Robot, operator_pos: Point,
                shapes: list[Shape], now: float, dt: float,
                follow_distance: float = FOLLOW_DISTANCE) -> bool:  # fmt: skip
    """One step of following: close the gap, but stop follow_distance away."""
    return navigator.step(robot, operator_pos, shapes, now, dt, follow_distance)
