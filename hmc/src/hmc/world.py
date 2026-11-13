"""A tiny 2D kinematic world: one robot, one scripted operator, a few bags.

Everything here is "ground truth". The robot code must not read the world
directly. It asks the perception module instead.
"""

import math
from dataclasses import dataclass, field

Point = tuple[float, float]

TIME_LIMIT = 300.0  # seconds, the 5:00 limit from the score sheet
DT = 0.1  # seconds per simulation step

# The real base is a 0.40 m square with 0.04 m corners cut off.
# ROBOT_RADIUS is a circle around that outline: the planner uses it, on the safe side.
ROBOT_RADIUS = 0.26  # metres
# The outline itself, in the robot frame (x forward), used by the referee to judge touching.
FOOTPRINT = [(0.20, 0.16), (0.16, 0.20), (-0.16, 0.20), (-0.20, 0.16),
             (-0.20, -0.16), (-0.16, -0.20), (0.16, -0.20), (0.20, -0.16)]  # fmt: skip
PERSON_RADIUS = 0.25  # metres, a person seen from above
CAR_RADIUS = 3.0  # metres, the robot counts as "at the car" inside this
# After the robot reaches the car, the crowd waits this long, then starts to queue.
QUEUE_DELAY = 2.0  # seconds
# Time between two people starting to walk, so they form a line.
QUEUE_SPACING = 2.0  # seconds
# A person in the queue does not walk closer than this to the robot (centre to centre).
KEEP_FROM_ROBOT = ROBOT_RADIUS + PERSON_RADIUS + 0.3
# While the heading is off by more than this, turn on the spot without driving.
TURN_ON_SPOT = math.radians(30)
# While the heading is off by more than this, brake to a stop before turning.
STOP_TO_TURN = math.radians(90)
# The base cannot change speed at once. 0.5 m/s^2 is a gentle limit for a wheeled base.
MAX_ACCEL = 0.5  # m/s^2
# The same wheel acceleration on a two-wheel base about 0.34 m wide, turning on the spot:
# each wheel at 0.5 m/s^2 gives 2 * 0.5 / 0.34 = about 3 rad/s^2.
MAX_TURN_ACCEL = 3.0  # rad/s^2
# When the robot asks the operator to come back, they stand this far in front of it,
# and wait there this long so the robot can see them, before they walk on.
COME_BACK_DISTANCE = 1.5  # metres
COME_BACK_WAIT = 2.0  # seconds


def distance(a: Point, b: Point) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def wrap_angle(angle: float) -> float:
    """Wrap an angle to the range -pi .. pi."""
    return math.atan2(math.sin(angle), math.cos(angle))


def outline(x: float, y: float, heading: float, step: float = 0.02) -> list[Point]:
    """Points every `step` metres along the robot's real outline, in the world frame."""
    c, s = math.cos(heading), math.sin(heading)
    points = []
    for (ax, ay), (bx, by) in zip(FOOTPRINT, FOOTPRINT[1:] + FOOTPRINT[:1]):
        n = max(1, int(math.hypot(bx - ax, by - ay) / step))
        for k in range(n):
            px, py = ax + (bx - ax) * k / n, ay + (by - ay) * k / n
            points.append((x + c * px - s * py, y + s * px + c * py))
    return points


def body_clearance(robot, clearance) -> float:
    """Smallest clearance(point) over the robot's real body. Below 0 means touching.

    The outline, a half-size outline and the centre: anything wider than 0.1 m that is
    under the robot crosses one of them (the smallest object is 0.24 m wide).
    """
    points = outline(robot.x, robot.y, robot.heading)
    points += [
        (robot.x + (px - robot.x) / 2, robot.y + (py - robot.y) / 2)
        for px, py in points
    ]
    points.append(robot.pos)
    return min(clearance(p) for p in points)


@dataclass(frozen=True)
class Circle:
    x: float
    y: float
    r: float

    def clearance(self, p: Point) -> float:
        """Distance from p to the edge of the circle (negative inside)."""
        return distance(p, (self.x, self.y)) - self.r

    def bounds(self) -> tuple[float, float, float, float]:
        return (self.x - self.r, self.y - self.r, self.x + self.r, self.y + self.r)


@dataclass(frozen=True)
class Box:
    x_min: float
    y_min: float
    x_max: float
    y_max: float

    def clearance(self, p: Point) -> float:
        """Distance from p to the box (0 inside)."""
        dx = max(self.x_min - p[0], 0.0, p[0] - self.x_max)
        dy = max(self.y_min - p[1], 0.0, p[1] - self.y_max)
        return math.hypot(dx, dy)

    def bounds(self) -> tuple[float, float, float, float]:
        return (self.x_min, self.y_min, self.x_max, self.y_max)

    def blocks(self, a: Point, b: Point) -> bool:
        """True if the straight line from a to b passes through the box (slab method)."""
        t_in, t_out = 0.0, 1.0
        for start, delta, low, high in (
            (a[0], b[0] - a[0], self.x_min, self.x_max),
            (a[1], b[1] - a[1], self.y_min, self.y_max),
        ):
            if abs(delta) < 1e-12:
                if not low <= start <= high:
                    return False
                continue
            t1, t2 = (low - start) / delta, (high - start) / delta
            t_in, t_out = max(t_in, min(t1, t2)), min(t_out, max(t1, t2))
            if t_in > t_out:
                return False
        return True


Shape = Circle | Box


@dataclass
class Obstacle:
    """Something on the ground the robot must not touch."""

    kind: str  # "small_object", "hard_to_see" or "barriers"
    shape: Shape
    seen_within: float | None = None  # hard-to-see objects are only seen this close


def walk_route(walker, dt: float) -> None:
    """Move a person (anything with x, y, speed, route) along its route."""
    if not walker.route:
        return
    goal = walker.route[0]
    left = distance((walker.x, walker.y), goal)
    move = min(walker.speed * dt, left)
    if left > 0:
        walker.x += (goal[0] - walker.x) / left * move
        walker.y += (goal[1] - walker.y) / left * move
    if distance((walker.x, walker.y), goal) < 1e-9:
        walker.route.pop(0)


@dataclass
class Robot:
    x: float = 0.0
    y: float = 0.0
    heading: float = 0.0  # radians, 0 = facing +x
    max_speed: float = 1.0  # m/s
    max_turn_rate: float = 2.0  # rad/s
    speed: float = 0.0  # m/s, how fast the robot drives right now
    turn_speed: float = 0.0  # rad/s, how fast it turns right now

    @property
    def pos(self) -> Point:
        return (self.x, self.y)

    def _turn(self, error: float, dt: float) -> float:
        """Turn toward a heading error, with the turn acceleration limit. Returns the turn."""
        # Slow down the turn in time to stop at the heading: v = sqrt(2 * a * angle left).
        wanted = min(self.max_turn_rate, math.sqrt(2 * MAX_TURN_ACCEL * abs(error)))
        wanted = math.copysign(wanted, error)
        self.turn_speed = _ramp(self.turn_speed, wanted, MAX_TURN_ACCEL * dt)
        turn = self.turn_speed * dt
        if turn * error > 0 and abs(turn) > abs(error):
            # We reach the heading within this step: stop there, do not swing past it.
            turn = error
            self.turn_speed = error / dt
        self.heading = wrap_angle(self.heading + turn)
        return turn

    def turn_toward(self, target: Point, dt: float) -> float:
        """Turn on the spot toward the target. Returns the heading error left."""
        # shortcut: drops any speed left at once; fine while the task only calls this
        # after the navigator stopped the robot, add braking if that changes.
        self.speed = 0.0
        wanted = math.atan2(target[1] - self.y, target[0] - self.x)
        error = wrap_angle(wanted - self.heading)
        return error - self._turn(error, dt)

    def drive_toward(
        self,
        target: Point,
        dt: float,
        stop_distance: float = 0.0,
        stop_in: float | None = None,
    ) -> None:
        """Turn toward the target and drive until stop_distance away from it.

        stop_in is how far away the robot must stand still (the end of the path, when
        the target is only the next waypoint). The robot brakes in time for it.
        """
        remaining = distance(self.pos, target) - stop_distance
        if remaining <= 0:
            self.speed = 0.0
            return
        wanted = math.atan2(target[1] - self.y, target[0] - self.x)
        error = wrap_angle(wanted - self.heading)
        # The path turned back behind us: stop first, rolling straight on (where the
        # navigator checked there is room), and only then turn on the spot. Turning while
        # still rolling would sweep the robot sideways into whatever is next to it.
        must_stop = abs(error) > STOP_TO_TURN and self.speed > 0
        turn = self._turn(0.0 if must_stop else error, dt)
        # Turn on the spot first when facing the wrong way (a two-wheel base can do that),
        # then drive slower while still turning. Never drive backwards.
        left = abs(error - turn)
        speed = 0.0 if left > TURN_ON_SPOT else self.max_speed * math.cos(left)
        if stop_in is not None:
            speed = min(speed, braking_speed(stop_in, dt))
        self.speed = _ramp(self.speed, speed, MAX_ACCEL * dt)
        step = min(self.speed * dt, remaining)
        self.x += math.cos(self.heading) * step
        self.y += math.sin(self.heading) * step

    def brake(self, dt: float) -> None:
        """Nothing to drive to: slow down. The robot still rolls on while it brakes."""
        self.turn_speed = 0.0  # shortcut: stops turning at once, ramp it if it matters
        self.speed = _ramp(self.speed, 0.0, MAX_ACCEL * dt)
        self.x += math.cos(self.heading) * self.speed * dt
        self.y += math.sin(self.heading) * self.speed * dt


def braking_speed(room: float, dt: float) -> float:
    """The top speed from which the robot can still stop within `room` metres.

    v = sqrt(2 * a * room), less one step of braking because we only brake from the
    next step on. Close to the end we still creep at one step of braking, so we arrive.
    """
    if room <= 0:
        return 0.0
    step = MAX_ACCEL * dt
    return max(math.sqrt(2 * MAX_ACCEL * room) - step, step)


def _ramp(now: float, wanted: float, max_change: float) -> float:
    """Move now toward wanted by at most max_change."""
    return now + max(-max_change, min(max_change, wanted - now))


# Bag states
ON_FLOOR = "on_floor"
CARRIED = "carried"
DROPPED = "dropped"  # fell by accident, costs points
PLACED = "placed"  # put down on purpose, free


@dataclass
class Bag:
    name: str
    x: float
    y: float
    state: str = ON_FLOOR

    @property
    def pos(self) -> Point:
        return (self.x, self.y)


@dataclass
class Operator:
    """A volunteer that follows a fixed script.

    Waits until told to start, walks along the route, stops at the last
    point (the car), waits a moment, then points at the target bag.
    """

    x: float
    y: float
    route: list[Point]
    target_bag: str
    speed: float = 1.2  # m/s, people walk about 1.2 to 1.4 m/s
    point_delay: float = 2.0  # seconds between stopping and pointing
    walking: bool = False
    waiting: bool = False  # the robot asked us to wait for it
    pointing: bool = False
    _stopped_for: float = 0.0
    coming_back: bool = False  # walking back because the robot asked
    came_back: int = 0  # how often they came back when asked (for the referee)
    _wait: float = 0.0
    _target_pos: Point = (0.0, 0.0)  # filled in by World

    @property
    def pos(self) -> Point:
        return (self.x, self.y)

    def _step_round(self, robot: Point, x: float, y: float, dt: float) -> None:
        """A person walks round a robot standing in their way, never into it."""
        near = ROBOT_RADIUS + PERSON_RADIUS + 0.1
        if distance(self.pos, robot) >= near or distance(self.pos, robot) >= distance((x, y), robot):
            return
        # Step sideways instead, on the side we already are, at walking speed.
        ax, ay = x - robot[0], y - robot[1]
        length = math.hypot(ax, ay) or 1.0
        mx, my = self.x - x, self.y - y
        side = (-ay / length, ax / length)
        if mx * side[0] + my * side[1] < 0:
            side = (-side[0], -side[1])
        self.x, self.y = x + side[0] * self.speed * dt, y + side[1] * self.speed * dt

    def start_walking(self) -> None:
        self.walking = True

    def come_back(self, robot: "Robot") -> None:
        """Walk to stand in front of the robot, then back to here and on along the route."""
        spot = (
            robot.x + COME_BACK_DISTANCE * math.cos(robot.heading),
            robot.y + COME_BACK_DISTANCE * math.sin(robot.heading),
        )
        # shortcut: walks in a straight line, even through walls; enough for a scripted
        # volunteer, upgrade to a planned path if a map needs it
        self.route = [spot, self.pos] + self.route
        self.coming_back = True
        self.waiting = False  # coming back overrides an earlier "please wait for me"
        self.pointing, self._stopped_for = False, 0.0

    def step(self, dt: float, robot: Point | None = None) -> None:
        if not self.walking:
            return
        if self._wait > 0:
            self._wait -= dt
            return
        if self.route:
            if not self.waiting:
                before = len(self.route)
                x, y = self.x, self.y
                walk_route(self, dt)
                if robot is not None:
                    self._step_round(robot, x, y, dt)
                if (
                    self.coming_back and len(self.route) < before
                ):  # standing in front now
                    self.coming_back = False
                    self.came_back += 1
                    self._wait = COME_BACK_WAIT
        else:
            self._stopped_for += dt
            if self._stopped_for >= self.point_delay:
                self.pointing = True


@dataclass
class Person:
    """One of the crowd. Stands still at first, then walks back to the start in the queue."""

    id: int
    x: float
    y: float
    route: list[Point]
    speed: float = 0.5  # m/s, a slow queue
    walking: bool = False

    def __post_init__(self) -> None:
        # Where the person stood in the crowd, for the plot
        self.home = (self.x, self.y)

    @property
    def pos(self) -> Point:
        return (self.x, self.y)

    def step(self, dt: float, robot: Point) -> None:
        if not self.walking or not self.route:
            return
        # A person in the queue waits rather than walk into the robot in front of them.
        goal = self.route[0]
        ahead = (goal[0] - self.x) * (robot[0] - self.x) + (goal[1] - self.y) * (
            robot[1] - self.y
        )
        if ahead > 0 and distance(self.pos, robot) < KEEP_FROM_ROBOT:
            return
        walk_route(self, dt)


@dataclass
class World:
    robot: Robot
    operator: Operator
    bags: dict[str, Bag]
    start: Point
    car: Point  # the robot is never told this
    obstacles: list[Obstacle] = field(default_factory=list)
    people: list[Person] = field(default_factory=list)  # the crowd, later the queue
    bystanders: list = field(default_factory=list)  # passers-by, see bystanders.py
    noise: object = None  # sensor noise, see noise.py (None = perfect sensors)
    sensors: str = "ideal"  # "ideal": see all round to 10 m; "robot": see sensors.py
    sensor_state: dict = field(default_factory=dict)  # e.g. the lidar's operator track
    odometry: object = (
        None  # where the robot thinks it is, see odometry.py (None: exact)
    )
    time: float = 0.0
    drop_count: int = 0
    queue_start_time: float | None = None  # set once the robot reaches the car
    trail: list[Point] = field(default_factory=list)

    def pointing_ray(self) -> tuple[Point, float] | None:
        """The operator's pointing ray (origin, angle) or None if not pointing."""
        if not self.operator.pointing:
            return None
        target = self.bags[self.operator.target_bag]
        angle = math.atan2(target.y - self.operator.y, target.x - self.operator.x)
        return (self.operator.pos, angle)

    def carried_bag(self) -> Bag | None:
        for bag in self.bags.values():
            if bag.state == CARRIED:
                return bag
        return None

    def step(self, dt: float = DT) -> None:
        if self.odometry is not None:
            self.odometry.update(self.robot, self.time)
        self.operator.step(dt, self.robot.pos)
        # The rulebook: after the robot has reached the car, the crowd returns in a queue.
        if (
            self.queue_start_time is None
            and distance(self.robot.pos, self.car) <= CAR_RADIUS
        ):
            self.queue_start_time = self.time + QUEUE_DELAY
        for i, person in enumerate(self.people):
            if (
                self.queue_start_time is not None
                and self.time >= self.queue_start_time + i * QUEUE_SPACING
            ):
                person.walking = True
            person.step(dt, self.robot.pos)
        for bystander in self.bystanders:
            bystander.step(dt, self)
        bag = self.carried_bag()
        if bag is not None:  # a carried bag moves with the robot
            bag.x, bag.y = self.robot.pos
        self.time = round(self.time + dt, 6)


def build_world(target_bag: str = "blue", robot_speed: float = 1.0) -> World:
    """The default scenario.

    The operator walks from the start to the car at (12, 6). On the way there are the four
    obstacles of the rulebook: a small object (the operator steps over it), a hard-to-see
    object (the operator walks closely around it), a crowd with gaps too narrow for the
    robot, and retractable barriers that the operator walks around.
    """
    start = (0.0, 0.0)
    car = (12.0, 6.0)
    bags = {
        "red": Bag("red", 13.5, 7.5),
        "blue": Bag("blue", 13.5, 4.5),
        "green": Bag("green", 12.0, 8.5),
    }
    operator = Operator(
        x=1.5,  # standing in front of the robot
        y=0.0,
        route=[(8.0, 0.0), (8.0, 6.0), (9.0, 7.1), (11.0, 7.1), car],
        target_bag=target_bag,
    )
    obstacles = [
        Obstacle("small_object", Circle(4.0, 0.0, 0.12)),
        Obstacle("hard_to_see", Circle(6.0, 0.35, 0.15), seen_within=1.5),
        Obstacle("barriers", Box(9.4, 5.2, 10.6, 6.6)),
    ]
    # The crowd stands across the operator's path. The gaps fit a person, not the robot.
    # The queue walks back and stops in a line next to the start (as on every map), each
    # person 0.7 m behind the one in front. A queue that walked on past the start could
    # only be joined by a robot much faster than people walk.
    people = [
        Person(i + 1, x, 3.0, [(8.0, -1.0), (0.5 + 0.7 * i, -1.0)])
        for i, x in enumerate((7.35, 8.65, 9.95))
    ]
    robot = Robot(x=start[0], y=start[1], max_speed=robot_speed)
    return World(
        robot=robot, operator=operator, bags=bags, start=start, car=car,
        obstacles=obstacles, people=people,
    )  # fmt: skip
