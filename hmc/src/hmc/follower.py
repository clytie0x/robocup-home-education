"""Following the operator: who the operator is, follow them, and search when they are lost.

This is the one brain for the simulation and the real robot. It never sees the sensors
directly. Each step it gets a Senses record (filled by task.py in the simulation, or by
the robot's own sensor code) and a body to drive. The body needs pos, heading, speed,
drive_toward, turn_toward and brake (odometry.RobotView and hmc's Robot both have them).
"""

import math
from dataclasses import dataclass, field
from enum import Enum

from hmc import hri, navigation, perception
from hmc.appearance import LookMemory
from hmc.route import Route
from hmc.world import PERSON_RADIUS, Circle, Point, Shape, distance

# Noise handling.
OPERATOR_JUMP = 0.5  # metres a detection may be off from where we expect the operator
PERSON_MAX_SPEED = 2.0  # m/s; a detection further than this could walk is someone else
REMEMBER_FOR = 1.0  # seconds we still avoid a person the detector missed
MEMORY_CELL = 0.3  # metres; obstacles closer than this in place and size are the same
MEMORY_KEEP = 10.0  # seconds we remember an obstacle we no longer see
KEEP_NEAR = 1.5  # metres; closer than this we keep a small object while the camera cannot see it
SMALL_SIZE = 0.5  # metres; up to this wide counts as a small object
# Where the depth camera sees well, forward: closer than VIEW_NEAR it is blind.
VIEW_NEAR, VIEW_FAR, VIEW_HALF_ANGLE = 0.7, 3.0, math.radians(25)
# Re-identification by look (see appearance.py).
OPERATOR_LEARN = 20  # frames of the operator's look we learn at the start
OPERATOR_MATCH = 0.65  # a detection must look at least this much like the operator
# Pace.
WAIT_GAP = 4.0  # metres; further behind than this, we ask the operator to wait
GO_ON_GAP = 2.5  # metres; once we are this close again, we ask them to go on
# Losing the operator while following (Deus ex Machina in the rulebook).
LOST_AFTER = 2.0  # seconds without seeing the operator before we start to search
DRIVE_FOR = 10.0  # seconds at most to reach the last seen spot (it may be blocked)
LOOK_AROUND = 5.0  # seconds of turning on the spot before we ask them to come back


def _small(shape) -> bool:
    """A small object, the kind that only the camera sees (walls the lidar sees anyway)."""
    x0, y0, x1, y1 = shape.bounds()
    return max(x1 - x0, y1 - y0) <= SMALL_SIZE


@dataclass
class Senses:
    """What the robot senses in one step, in the frame of its believed pose."""

    time: float
    operator: Point | None  # the operator this step, if a detector reports them
    operator_look: tuple[float, ...] | None  # their look, if the camera saw them
    people: dict[int, Point]  # everyone else, by tracker id
    obstacles: list[Shape]
    # The look of each person in people the camera saw this step, by the same id.
    looks: dict[int, tuple[float, ...]] = field(default_factory=dict)


class Mode(Enum):
    LEARN = "learn"  # learning the operator's look (we already follow while we learn)
    FOLLOW = "follow"
    SEARCH = "search"  # we lost the operator: look for them, then ask them back


class Follower:
    def __init__(self, body, speech, follow_distance: float = navigation.FOLLOW_DISTANCE,
                 search_again_every: float | None = None):  # fmt: skip
        self.body = body
        self.speech = speech
        self.follow_distance = follow_distance
        # After asking the operator back, turn on the spot again every this many
        # seconds. None (the simulation) stands still instead. The robot passes 10.0.
        self.search_again_every = search_again_every
        self.mode = Mode.LEARN
        self.last_operator_pos: Point | None = None
        self.operator_now: Point | None = None  # this step's detection of the operator
        self.tracker = perception.MotionTracker()
        self.people_trackers: dict[int, perception.MotionTracker] = {}
        self.operator_look = LookMemory()
        self.navigator = navigation.Navigator()
        # Breadcrumbs for the way back, from where we stand now (the start).
        self.route = Route(body.pos)
        # Obstacles we saw before. The camera looks forward only and the depth image is
        # blind closer than 0.6 m, so a low object we pass would vanish without this.
        self.obstacle_memory: dict = {}  # rough place and size -> (time, shape)
        self.asked_to_wait = False
        # Searching for the lost operator: when we started to turn, and if we asked.
        self.search_since = 0.0
        self.look_since: float | None = None
        self.asked_back = False
        self.asked_at = 0.0

    def score(self, look) -> float:
        """How much a look is like the operator's: 1.0 is the same, around 0 unrelated."""
        return self.operator_look.similarity(look)

    def watch(self, senses: Senses) -> Point | None:
        """Update where the people and the operator are. Returns this step's detection.

        Call it every step, also when not following, so we keep avoiding the operator.
        """
        for pid, pos in senses.people.items():
            tracker = self.people_trackers.setdefault(pid, perception.MotionTracker())
            tracker.update(senses.time, pos)
        operator = senses.operator
        look = senses.operator_look
        if operator is not None and look is not None:
            if self.operator_look.count < OPERATOR_LEARN:
                self.operator_look.add(
                    look
                )  # at the start the operator is in front of us
                if (
                    self.operator_look.count == OPERATOR_LEARN
                    and self.mode == Mode.LEARN
                ):
                    self.mode = Mode.FOLLOW
            elif self.score(look) < OPERATOR_MATCH:
                operator = None  # does not look like the operator: someone else
        if operator is not None and self.last_operator_pos is not None:
            # A detection far from where the operator can be is someone else: ignore it.
            since = senses.time - self.tracker.last_time
            reach = OPERATOR_JUMP + PERSON_MAX_SPEED * since
            if distance(operator, self.tracker.last()) > reach:
                operator = None
        if operator is not None:
            self.tracker.update(senses.time, operator)
            # Smoothed, and kept when we lose sight of them.
            self.last_operator_pos = self.tracker.last()
        self.operator_now = operator
        return operator

    def step(self, senses: Senses, dt: float) -> None:
        """Follow or search, and drive the body. Call watch(senses) first in each step."""
        self.route.add(self.body.pos)
        if self.mode == Mode.SEARCH:
            self._search(senses, dt)
        else:
            self._follow(senses, dt)

    def _follow(self, senses: Senses, dt: float) -> None:
        # Standing still is a command too: on the robot the wheels keep the last one.
        if self.last_operator_pos is None:
            self.body.brake(dt)
            return
        if senses.time - self.tracker.last_time > LOST_AFTER:
            self.search_since, self.look_since = senses.time, None
            self.asked_back = False
            self.mode = Mode.SEARCH
            # no brake here: one cycle keeps the last command, as the sim always did (byte-identical refactor); the Pi's must_stop covers it
            return
        near = navigation.follow_step(
            self.navigator, self.body, self.last_operator_pos,
            self.shapes(senses), senses.time, dt, self.follow_distance,
        )  # fmt: skip
        if near:
            # Waiting behind the operator: face them, so the camera keeps them in view.
            self.body.turn_toward(self.last_operator_pos, dt)

        # Too far behind (a detour around the crowd): ask the operator to wait for us.
        # Without a path (the operator stands in a door we need) they must go on instead.
        gap = distance(self.body.pos, self.last_operator_pos)
        has_path = bool(self.navigator.path)
        if not self.asked_to_wait and gap > WAIT_GAP and has_path:
            self.speech.say(senses.time, hri.WAIT_TEXT)
            self.asked_to_wait = True
        elif self.asked_to_wait and (gap < GO_ON_GAP or not has_path):
            self.speech.say(senses.time, hri.GO_ON_TEXT)
            self.asked_to_wait = False

    def _search(self, senses: Senses, dt: float) -> None:
        """We lost the operator. Go where we saw them last, look around, then ask."""
        now = senses.time
        # Found again only with a look that matches the operator (checked in watch),
        # so a stranger who walks by is not taken for them.
        if self.operator_now is not None and senses.operator_look is not None:
            self.mode = Mode.FOLLOW
            return
        if self.look_since is None:
            reached = self.navigator.step(
                self.body, self.last_operator_pos, self.shapes(senses), now, dt,
                navigation.GOAL_TOLERANCE,
            )  # fmt: skip
            if reached or now - self.search_since > DRIVE_FOR:
                self.look_since = now
            return
        if now - self.look_since < LOOK_AROUND:
            # The camera sees only 58 degrees forward, so turn on the spot.
            # Aim a quarter turn to the left, so we keep turning left on the spot.
            x, y = self.body.pos
            h = self.body.heading + math.pi / 2
            self.body.turn_toward((x + math.cos(h), y + math.sin(h)), dt)
            return
        if not self.asked_back:
            # Costs -150 (Deus ex Machina), but only if we score again afterwards.
            self.speech.say(now, hri.COME_BACK_TEXT)
            self.asked_back, self.asked_at = True, now
        elif (
            self.search_again_every is not None
            and now - self.asked_at >= self.search_again_every
        ):
            self.look_since, self.asked_at = now, now  # turn on the spot once more
        # Stand still and face forward, where we asked them to stand.
        self.body.brake(dt)

    def _in_view(self, shape) -> bool:
        """True if the camera should see this shape now (its centre in the good view)."""
        x0, y0, x1, y1 = shape.bounds()
        dx, dy = (x0 + x1) / 2 - self.body.pos[0], (y0 + y1) / 2 - self.body.pos[1]
        angle = abs(math.atan2(math.sin(math.atan2(dy, dx) - self.body.heading),
                               math.cos(math.atan2(dy, dx) - self.body.heading)))  # fmt: skip
        return VIEW_NEAR <= math.hypot(dx, dy) <= VIEW_FAR and angle <= VIEW_HALF_ANGLE

    def shapes(self, senses: Senses) -> list[Shape]:
        """Everything we must not touch: seen obstacles, other people and the operator."""
        now = senses.time
        for shape in senses.obstacles:
            # The same obstacle comes back a little moved every frame (our own pose
            # drifts), so we key it by its rough place and size and keep the newest.
            x0, y0, x1, y1 = shape.bounds()
            key = (
                round((x0 + x1) / 2 / MEMORY_CELL), round((y0 + y1) / 2 / MEMORY_CELL),
                round((x1 - x0) / MEMORY_CELL), round((y1 - y0) / MEMORY_CELL),
            )  # fmt: skip
            self.obstacle_memory[key] = (now, shape)
        # Forget what we have not seen for a while: our own pose drifts, so an old
        # memory is in the wrong place, and could block a door that is really open.
        # But keep what is near us and out of the camera's view: there we have no news
        # of it, and forgetting it let the robot drive over a low object it had
        # stood next to for a while. Something the camera should see but does not, is gone.
        here = self.body.pos
        self.obstacle_memory = {
            key: (seen, shape)
            for key, (seen, shape) in self.obstacle_memory.items()
            if now - seen <= MEMORY_KEEP
            or (_small(shape) and shape.clearance(here) <= KEEP_NEAR and not self._in_view(shape))
        }
        shapes = [shape for _, shape in self.obstacle_memory.values()]
        # People we saw a moment ago are still there, even if this frame missed them.
        trackers = list(self.people_trackers.values())
        if self.last_operator_pos is not None:
            trackers.append(self.tracker)
        for tracker in trackers:
            if tracker.seen and now - tracker.last_time <= REMEMBER_FOR:
                # Where they are going and where we last saw them: a walking person
                # may stop at any moment, so avoid both.
                for x, y in {tracker.predict(now), tracker.last()}:
                    shapes.append(Circle(x, y, PERSON_RADIUS))
        return shapes
