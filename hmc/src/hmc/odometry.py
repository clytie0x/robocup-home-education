"""Where the robot thinks it is, versus where it really is.

The real robot knows its position only from its wheels (odometry). Wheels slip and the
wheel size is never exact, so the estimate drifts away from the truth the further the
robot drives. Coming back to the start after 20 m to 40 m can be off by a metre.

    --odometry perfect   the robot knows exactly where it is (the default)
    --odometry wheel     wheels only: about 2 % error in distance and a slow heading drift
    --odometry slam      wheels plus a SLAM correction every second, which keeps the error
                         to a few centimetres (a simple model of what SLAM achieves)

The robot code never sees the truth. It gets a RobotView (pose in its own belief), and
perception reports everything in that same believed frame. The referee uses the truth.
"""

import math
import random
from dataclasses import dataclass, field

from hmc.world import Point, Robot, wrap_angle


@dataclass
class Odometry:
    scale_error: float = 0.02  # the wheels measure distances 2 % too long
    turn_error: float = 0.01  # and turns 1 % too large
    heading_walk: float = 0.01  # radians of random heading drift per metre driven
    slam_every: float | None = None  # seconds between SLAM corrections (None: no SLAM)
    slam_sigma: float = 0.05  # metres of error left after a SLAM correction
    seed: int = 0
    rng: random.Random = field(init=False, repr=False)
    belief: list = field(init=False, default=None)  # [x, y, heading] the robot believes
    _last_truth: tuple = field(init=False, default=None)
    _last_slam: float = field(init=False, default=0.0)

    def __post_init__(self) -> None:
        self.rng = random.Random(self.seed * 104729 + 7)

    def start(self, robot: Robot) -> None:
        self.belief = [robot.x, robot.y, robot.heading]
        self._last_truth = (robot.x, robot.y, robot.heading)

    def update(self, robot: Robot, time: float) -> None:
        """Integrate the motion since the last step, with the wheel errors."""
        x0, y0, h0 = self._last_truth
        step = math.hypot(robot.x - x0, robot.y - y0)
        turn = wrap_angle(robot.heading - h0)
        bx, by, bh = self.belief
        bh = wrap_angle(bh + turn * (1 + self.turn_error))
        bh = wrap_angle(bh + self.rng.gauss(0.0, self.heading_walk) * step)
        # The robot drives forward along its heading, so the step goes along its belief.
        moved_forward = (robot.x - x0) * math.cos(h0) + (robot.y - y0) * math.sin(h0)
        sign = 1.0 if moved_forward >= 0 else -1.0
        bx += sign * step * (1 + self.scale_error) * math.cos(bh)
        by += sign * step * (1 + self.scale_error) * math.sin(bh)
        if self.slam_every is not None and time - self._last_slam >= self.slam_every:
            # SLAM matches the scans to its map and pulls the belief back near the truth.
            bx = robot.x + self.rng.gauss(0.0, self.slam_sigma)
            by = robot.y + self.rng.gauss(0.0, self.slam_sigma)
            bh = wrap_angle(robot.heading + self.rng.gauss(0.0, math.radians(1.0)))
            self._last_slam = time
        self.belief = [bx, by, bh]
        self._last_truth = (robot.x, robot.y, robot.heading)

    # Frames: "true" is the world, "belief" is the robot's own map.

    def to_belief(self, p: Point, robot: Robot) -> Point:
        dx, dy = p[0] - robot.x, p[1] - robot.y
        c, s = math.cos(-robot.heading), math.sin(-robot.heading)
        rx, ry = c * dx - s * dy, s * dx + c * dy  # relative to the robot
        bx, by, bh = self.belief
        c, s = math.cos(bh), math.sin(bh)
        return (bx + c * rx - s * ry, by + s * rx + c * ry)

    def to_true(self, p: Point, robot: Robot) -> Point:
        bx, by, bh = self.belief
        dx, dy = p[0] - bx, p[1] - by
        c, s = math.cos(-bh), math.sin(-bh)
        rx, ry = c * dx - s * dy, s * dx + c * dy
        c, s = math.cos(robot.heading), math.sin(robot.heading)
        return (robot.x + c * rx - s * ry, robot.y + s * rx + c * ry)

    def heading_offset(self, robot: Robot) -> float:
        """Believed heading minus true heading."""
        return wrap_angle(self.belief[2] - robot.heading)


PRESETS = {
    "perfect": None,
    "wheel": dict(),
    "slam": dict(slam_every=1.0),
    # The real base after the 2026-11-13 calibration, measured with LiDAR scan matching:
    # 360-degree turns within 0.3 %, 0.27 m lines within 1 %.
    "calibrated": dict(scale_error=0.01, turn_error=0.003, heading_walk=0.005),
    # The calibrated base plus scan-matching corrections (base/docs return-trip design):
    # the same stand-in as "slam", on top of the measured wheel errors.
    "calibrated_slam": dict(scale_error=0.01, turn_error=0.003, heading_walk=0.005, slam_every=1.0),
}


def make_odometry(level: str, seed: int = 0) -> Odometry | None:
    settings = PRESETS[level]
    return None if settings is None else Odometry(seed=seed, **settings)


class RobotView:
    """The robot as its own code sees it: its believed pose, and commands in that frame."""

    def __init__(self, world) -> None:
        self.world = world

    @property
    def _odo(self):
        return self.world.odometry

    @property
    def pos(self) -> Point:
        robot = self.world.robot
        return (
            robot.pos
            if self._odo is None
            else (self._odo.belief[0], self._odo.belief[1])
        )

    @property
    def heading(self) -> float:
        robot = self.world.robot
        return robot.heading if self._odo is None else self._odo.belief[2]

    @property
    def x(self) -> float:
        return self.pos[0]

    @property
    def y(self) -> float:
        return self.pos[1]

    @property
    def max_speed(self) -> float:
        return self.world.robot.max_speed

    @property
    def speed(self) -> float:
        """How fast the wheels drive now (the robot knows this from its motors)."""
        return self.world.robot.speed

    def _true(self, p: Point) -> Point:
        return p if self._odo is None else self._odo.to_true(p, self.world.robot)

    def drive_toward(self, target: Point, dt: float, **options) -> None:
        self.world.robot.drive_toward(self._true(target), dt, **options)

    def brake(self, dt: float) -> None:
        self.world.robot.brake(dt)

    def turn_toward(self, target: Point, dt: float) -> float:
        return self.world.robot.turn_toward(self._true(target), dt)
