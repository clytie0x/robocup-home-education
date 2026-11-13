"""Moving the robot: follow a person, or go to a point, without touching obstacles.

The robot has no map. Every plan is made only from the shapes that perception sees now.
We lay a grid over the area around the robot and the goal, mark the cells that are too
close to a shape, and search for the shortest free path with A*.
"""

import heapq
import math

from hmc import world as hmc_world
from hmc.world import Point, Robot, Shape, distance, wrap_angle

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
STOP_MARGIN = SAFETY / 2  # metres left after a full stop: room for moving people and noisy shapes, with slack to drive along the planner's safety zone
LOOK_AHEAD = 1.5  # metres; more than the 1.0 m the robot needs to stop from 1 m/s
# Step of the look ahead. A full cell (0.1 m) could report up to 0.1 m more room than
# there is, and with noise the robot then braked into a person standing in a crowd.
FREE_STEP = 0.02  # metres
# The shortest range at which the robot is sure to see any obstacle: the hard-to-see
# object of the rulebook, seen only up close (sensors.py, task spec).
SURE_SEEN = 1.5  # metres
SWEEP_STEP = math.radians(15)  # headings checked between where we face and where we turn to
# When no path keeps that wide berth, the planner tries again closer, the way the real robot
# fits its 0.40 m square through a 0.65 m gap: the grid keeps the half-width plus TIGHT_SLACK
# from every shape (so a gap 0.15 m wider than the body always has a free cell, at any grid
# offset), and the real outline, checked along the path, keeps OUTLINE_MARGIN.
TIGHT_SLACK = 0.05  # metres
# Off in the simulation's own base (1 m/s): under hard noise it let the robot touch more often
# (107 against 110 of 120, 2026-11-14). The real base (0.12 m/s) and hmc.run's "real" base turn it on.
CLOSE_PLAN = False
OUTLINE_MARGIN = 0.07  # metres: the 0.03 m the robot must keep, plus room for not driving the line exactly
OUTLINE_STEP = 0.05  # metres between two outline checks along a segment
CORNER_STEP = math.radians(5)  # headings checked where the path turns
TIGHT_ROUNDS = 3  # tries to plan around the spots where the outline did not fit
# Outline checks one close plan may spend (on the Pi a plan with 200 shapes took 0.29 s at 5 rounds).
CHECK_BUDGET = 400
NEAR = 0.3  # metres: a shape farther than this from the corner circle cannot come within the margin
# True when the last plan_path needed the close plan. The Navigator then judges the room ahead by
# the real outline too, or the circle would stop it in front of the gap it just planned through.
LAST_PLAN_TIGHT = False

Cell = tuple[int, int]


def plan_path(start: Point, goal: Point, shapes: list[Shape],
              near_side: bool = False, heading: float | None = None) -> list[Point] | None:  # fmt: skip
    """Shortest free path from start to goal as a list of points, or None if there is none.

    If the goal itself is blocked (for example a person we want to walk behind), the
    path ends at the free cell closest to the goal instead. With near_side, it ends at
    the free cell on our side of the goal (see follow_step).
    First with the wide berth (the circle around the robot plus SAFETY); only when that
    finds nothing, closer, with the real outline checked along the path (TIGHT_SLACK).
    `heading` is where the robot faces at the start, for the turn onto the first segment.
    """
    global LAST_PLAN_TIGHT
    LAST_PLAN_TIGHT = False
    path = _grid_path(start, goal, shapes, near_side, hmc_world.ROBOT_RADIUS + SAFETY, [])
    if path is not None or not CLOSE_PLAN:
        return path
    LAST_PLAN_TIGHT = True
    avoid: list[Point] = []
    # The goal itself (the operator we follow) is not checked: the robot stops short of it anyway.
    others = [s for s in shapes if s.clearance(goal) > 0]
    budget = [CHECK_BUDGET]
    for _ in range(TIGHT_ROUNDS):
        path = _grid_path(start, goal, shapes, near_side, hmc_world.INSCRIBED + TIGHT_SLACK, avoid)
        if path is None:
            return None
        bad = path_clear(path, start, heading, others, OUTLINE_MARGIN, budget)
        if bad == -1:
            return None  # out of checks: no close plan this time, the next replan tries again
        if bad is None:
            return path
        points = [start] + path
        a, b = points[bad], points[bad + 1]
        # block the middle of the segment that did not fit, or its end if the middle is where we stand
        mid = ((a[0] + b[0]) / 2, (a[1] + b[1]) / 2)
        avoid.append(b if distance(mid, start) < CELL else mid)
    return None


def outline_gap(pos: Point, heading: float, shapes: list[Shape]) -> float:
    """Smallest clearance of the robot's real outline at pos, facing heading, to the shapes.

    Shapes farther than the outline can reach plus NEAR only give a lower bound, which is
    above NEAR, so the answer is exact wherever it is below NEAR.
    """
    reach = max(math.hypot(px, py) for px, py in hmc_world.FOOTPRINT)
    near, far = [], math.inf
    for s in shapes:
        here = s.clearance(pos)
        if here < reach + NEAR:
            near.append(s)
        else:
            far = min(far, here - reach)
    if not near:
        return far
    body = hmc_world.outline(pos[0], pos[1], heading)
    return min(far, min(s.clearance(p) for s in near for p in body))


def path_clear(path: list[Point], start: Point, start_heading: float | None,
               shapes: list[Shape], margin: float, budget: list[int] | None = None) -> int | None:  # fmt: skip
    """Index of the first segment of start + path where the outline comes within margin, else None.

    budget, if given, is a one-item list of outline checks left; when it runs out the answer is -1.

    Along a segment the robot faces the segment; where the path turns, it turns on the spot
    through the headings between (CORNER_STEP). On the first segment a shape the robot already
    stands closer to than margin only has to get no closer.
    """
    points = [start] + path
    facing = start_heading
    for i, (a, b) in enumerate(zip(points, points[1:])):
        length = distance(a, b)
        if length < 1e-9:
            continue
        along = math.atan2(b[1] - a[1], b[0] - a[0])
        least = margin
        if i == 0:
            least = min(margin, outline_gap(a, along if facing is None else facing, shapes)) - 1e-6
        checks: list[tuple[Point, float]] = []
        if facing is not None:
            turn = wrap_angle(along - facing)
            n = int(abs(turn) / CORNER_STEP)
            checks += [(a, facing + turn * k / (n + 1)) for k in range(1, n + 1)]
        n = max(1, math.ceil(length / OUTLINE_STEP))
        checks += [((a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n), along) for k in range(n + 1)]
        if budget is not None:
            budget[0] -= len(checks)
            if budget[0] < 0:
                return -1
        if any(outline_gap(p, h, shapes) < least for p, h in checks):
            return i
        facing = along
    return None


def _grid_path(start: Point, goal: Point, shapes: list[Shape], near_side: bool,
               inflate: float, avoid: list[Point]) -> list[Point] | None:  # fmt: skip
    """A* on a grid whose cells closer than inflate to a shape, or holding an avoid point, are blocked."""
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
    blocked.update(cell_of(p) for p in avoid)

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
        if near_side:
            # The free cells around a person form a ring, all almost as close to the
            # goal. Take the one on our side, or the path end jumps around the person.
            nearest = distance(centre(goal_cell), goal)
            ring = [c for c in free if distance(centre(c), goal) <= nearest + CELL]
            goal_cell = min(ring, key=lambda c: (distance(centre(c), start), c))

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
        self.tight = False  # the path came from the close plan (LAST_PLAN_TIGHT)

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
        near_side: bool = False,
    ) -> bool:
        """One step toward the goal. Returns True once we are within stop_distance."""
        if distance(robot.pos, goal) <= stop_distance + 1e-9:
            robot.brake(dt)
            return True
        if self._needs_plan(goal, shapes, now):
            self.path = plan_path(robot.pos, goal, shapes, near_side, robot.heading) or []  # no path: wait here
            self.tight = LAST_PLAN_TIGHT
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
            free_there = free_ahead(robot, shapes, _heading_to(robot, aim), self.tight)
            if free_there == 0.0:
                # The straight line to the far waypoint cuts too close to something:
                # aim at the next waypoint instead, the planner says it is free.
                aim = self.path[0]
                free_there = free_ahead(robot, shapes, _heading_to(robot, aim), self.tight)
            # While it turns toward the aim it also rolls through every heading in between,
            # so check those too. Checking only the two ends let a fast robot swing into
            # something beside its path before it could brake.
            here = robot.heading
            turn = wrap_angle(_heading_to(robot, aim) - here)
            steps = int(abs(turn) / SWEEP_STEP)
            swept = min(
                (free_ahead(robot, shapes, here + turn * k / (steps + 1), self.tight) for k in range(1, steps + 1)),
                default=math.inf,
            )
            stop_in = min(
                distance(robot.pos, goal) - stop_distance,
                free_ahead(robot, shapes, robot.heading, self.tight),
                free_there,
                swept,
                # Never drive faster than we can stop within what we are sure to see:
                # a hard-to-see object shows up only SURE_SEEN metres from its edge.
                SURE_SEEN - hmc_world.ROBOT_RADIUS - STOP_MARGIN,
            )
            robot.drive_toward(aim, dt, stop_in=stop_in)
        else:
            robot.brake(dt)
        return distance(robot.pos, goal) <= stop_distance + 1e-9


def _heading_to(robot: Robot, point: Point) -> float:
    return math.atan2(point[1] - robot.y, point[0] - robot.x)


def free_ahead(robot: Robot, shapes: list[Shape], heading: float, outline: bool = False) -> float:
    """How far the robot can roll straight on in this heading before it gets too close.

    A shape only counts when rolling on brings us closer to it, so we can still drive
    along a wall or away from something we are already close to (like the planner).
    With outline, a shape the circle flags still counts only if the real outline gets that close.
    """
    free = 0.0
    here = [s.clearance(robot.pos) for s in shapes]
    body_now: dict[int, float] = {}  # outline gap now, per shape, worked out only when needed
    while free < LOOK_AHEAD:
        x = robot.x + math.cos(heading) * (free + FREE_STEP)
        y = robot.y + math.sin(heading) * (free + FREE_STEP)
        for k, (shape, now) in enumerate(zip(shapes, here)):
            there = shape.clearance((x, y))
            if there <= hmc_world.ROBOT_RADIUS + STOP_MARGIN and there < now:
                if not outline:
                    return free
                # The circle around the robot is too close; the real outline may still pass
                # (a side of the square beside a door post). It decides, by the same rule.
                if k not in body_now:
                    body_now[k] = outline_gap(robot.pos, heading, [shape])
                body = outline_gap((x, y), heading, [shape])
                if body <= STOP_MARGIN and body < body_now[k]:
                    return free
        free += FREE_STEP
    return free


def follow_step(navigator: Navigator, robot: Robot, operator_pos: Point,
                shapes: list[Shape], now: float, dt: float,
                follow_distance: float = FOLLOW_DISTANCE) -> bool:  # fmt: skip
    """One step of following: close the gap, but stop follow_distance away."""
    # The real robot stops so close (0.8 m) that it drives right up to the free ring
    # around the operator, where it matters which ring cell the path ends at. The
    # simulation (1.5 m) keeps its old choice, so its six maps stay byte-identical.
    ring = hmc_world.PERSON_RADIUS + hmc_world.ROBOT_RADIUS + SAFETY
    near_side = follow_distance < ring + 0.3
    return navigator.step(robot, operator_pos, shapes, now, dt, follow_distance, near_side)
