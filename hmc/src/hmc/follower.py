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
# Pace. These are the simulation's, set for its 1 m/s base; the real base passes its own (Follower).
WAIT_GAP = 4.0  # metres; further behind than this, we ask the operator to wait
GO_ON_GAP = 2.5  # metres; once we are this close again, we ask them to go on
# Losing the operator while following (Deus ex Machina in the rulebook).
LOST_AFTER = 2.0  # seconds without seeing the operator before we start to search
DRIVE_FOR = 10.0  # seconds at most to reach the last seen spot (it may be blocked)
LOOK_AROUND = 5.0  # seconds of turning on the spot before we ask them to come back
# On a loss the camera is still pointed the way we were going. Turn back to the last heading
# before driving off, and ask them to wait well before the long look-around ends in "come back".
FACE_FOR = 1.5  # seconds at most spent turning to the last heading
FACE_TOL = math.radians(12)  # close enough to that heading to start the search drive
ASK_WAIT = 0.5  # seconds into the search: ask them to wait
# A look that is a bit off still passes when the LiDAR track never jumped and the clothes match.
# Weights are on top of the appearance score (weight 1). Continuity or colour alone stay under
# OPERATOR_MATCH, so a stranger on the same track is not enrolled.
COLOUR_W = 0.55
TRACK_W = 0.35
COLOUR_AGREE = 0.75  # cosine of the clothing histogram; below this the clothes are not the operator's


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
    # Clothing histogram of the operator this step, and a look whose light was too far to use for
    # identity but close enough to count while the LiDAR track is still continuous. Both optional.
    operator_colour: tuple[float, ...] | None = None
    soft_look: tuple[float, ...] | None = None


class Mode(Enum):
    LEARN = "learn"  # learning the operator's look (we already follow while we learn)
    FOLLOW = "follow"
    SEARCH = "search"  # we lost the operator: look for them, then ask them back


class Follower:
    def __init__(self, body, speech, follow_distance: float = navigation.FOLLOW_DISTANCE,
                 search_again_every: float | None = None, restart_gap: float = 0.0,
                 wait_gap: float = WAIT_GAP, go_on_gap: float = GO_ON_GAP):  # fmt: skip
        self.body = body
        self.speech = speech
        self.follow_distance = follow_distance
        # Once stopped at the follow distance, start again only when the operator is this much further.
        # 0 (the simulation) starts at once; the real base started at 1.31 m and stopped at 1.30 m,
        # 4.8 to 13.1 start-stops a minute (logs of 2026-11-15), and passes 0.25.
        self.restart_gap = restart_gap
        self.standing = False  # stopped at the follow distance, until the operator is past restart_gap
        self.wait_gap, self.go_on_gap = wait_gap, go_on_gap
        # After asking the operator back, turn on the spot again every this many
        # seconds. None (the simulation) stands still instead. The robot passes 10.0.
        self.search_again_every = search_again_every
        self.mode = Mode.LEARN
        self.last_operator_pos: Point | None = None
        self.operator_now: Point | None = None  # this step's detection of the operator
        self.tracker = perception.MotionTracker()
        self.people_trackers: dict[int, perception.MotionTracker] = {}
        self.operator_look = LookMemory()
        self.colour = LookMemory()  # clothing histogram, separate from the re-id vector
        self.soft_credits = 0  # continuous-track frames whose light was too far to store the vector
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
        self.faced_last = False  # on the next loss, turn to the last heading before driving off

    @property
    def learned_count(self) -> int:
        """Looks stored, plus continuous-track frames that confirmed the operator without a usable vector.

        The extra frames count only once five real looks are in, so a run of bad light cannot finish
        enrolment on its own.
        """
        extra = self.soft_credits if self.operator_look.count >= 5 else 0
        return self.operator_look.count + extra

    def score(self, look) -> float:
        """How much a look is like the operator's: 1.0 is the same, around 0 unrelated."""
        return self.operator_look.similarity(look)

    def _within_reach(self, pos, time) -> bool:
        """True when pos is where the operator's LiDAR track can still be (the same gate as a rejected jump)."""
        if pos is None or self.last_operator_pos is None or not self.tracker.seen:
            return False
        since = max(0.0, time - self.tracker.last_time)
        reach = OPERATOR_JUMP + PERSON_MAX_SPEED * since
        return distance(pos, self.tracker.last()) <= reach

    def fused_score(self, look, colour=None, continuous=False) -> float:
        """Appearance, plus the clothing histogram and whether the LiDAR track is still continuous.

        A different vector length (the re-id network fell back to a histogram mid-run) is not compared
        with the old embedding: only matching clothes on a continuous track then pass.
        """
        if (
            look is not None
            and self.operator_look.count
            and len(look) != len(self.operator_look.total)
        ):
            if (
                continuous
                and colour is not None
                and self.colour.count
                and self.colour.similarity(colour) >= COLOUR_AGREE
            ):
                return OPERATOR_MATCH
            return 0.0
        appearance = self.score(look) if look is not None and self.operator_look.count else 0.0
        total, weight = appearance, 1.0
        if colour is not None and self.colour.count:
            total += COLOUR_W * self.colour.similarity(colour)
            weight += COLOUR_W
        if continuous and self.operator_look.count:
            total += TRACK_W
            weight += TRACK_W
        return total / weight

    def _remember(self, memory: LookMemory, sample, adapt: bool) -> None:
        """Add one sample. Once the window is full, drop one average sample so a new light can move it."""
        if not sample:
            return  # an empty look (re-id failed) must not set the vector length
        if memory.count and len(sample) != len(memory.total):
            return  # a histogram after a 512-d embedding is not the same vector
        if adapt and memory.count >= OPERATOR_LEARN:
            mean = [value / memory.count for value in memory.total]
            memory.total = [value - part for value, part in zip(memory.total, mean)]
            memory.count -= 1
        memory.add(sample)

    def _note_soft(self, soft, colour, continuous) -> None:
        """Count a frame whose light was too far to trust the vector, when the track and the clothes agree."""
        if soft is None or not continuous or not self.operator_look.count:
            return
        clothes = colour is not None and (not self.colour.count or self.colour.similarity(colour) >= 0.6)
        if clothes and self.colour.count < OPERATOR_LEARN:
            self.colour.add(colour)
        if clothes or self.score(soft) >= 0.45:
            self.soft_credits = min(OPERATOR_LEARN, self.soft_credits + 1)

    def watch(self, senses: Senses) -> Point | None:
        """Update where the people and the operator are. Returns this step's detection.

        Call it every step, also when not following, so we keep avoiding the operator.
        """
        for pid, pos in senses.people.items():
            tracker = self.people_trackers.setdefault(pid, perception.MotionTracker())
            tracker.update(senses.time, pos)
        operator = senses.operator
        look = senses.operator_look
        if look is not None and len(look) == 0:
            look = None
        colour = getattr(senses, "operator_colour", None)
        continuous = self._within_reach(operator, senses.time)
        if operator is not None and look is not None:
            if self.operator_look.count < OPERATOR_LEARN:
                self._remember(self.operator_look, look, adapt=False)
                if colour is not None:
                    self._remember(self.colour, colour, adapt=False)
            elif self.fused_score(look, colour, continuous) < OPERATOR_MATCH:
                operator = None  # does not look like the operator: someone else
            elif continuous:
                # Same LiDAR track: keep the look current while we follow (a room gets brighter, clothes crease).
                self._remember(self.operator_look, look, adapt=True)
                if colour is not None:
                    self._remember(self.colour, colour, adapt=True)
        elif operator is not None and look is None:
            self._note_soft(getattr(senses, "soft_look", None), colour, continuous)
        if self.mode == Mode.LEARN and self.operator_look.count >= 5 and self.learned_count >= OPERATOR_LEARN:
            self.mode = Mode.FOLLOW
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
            self.faced_last = False
            self.mode = Mode.SEARCH
            # no brake here: one cycle keeps the last command, as the sim always did (byte-identical refactor); the Pi's must_stop covers it
            return
        turning = getattr(self.body, "turn_speed", None)  # the simulation's RobotView has none: as before
        gap = distance(self.body.pos, self.last_operator_pos)
        if self.restart_gap > 0 and self.standing and gap <= self.follow_distance + self.restart_gap:
            near = True  # stopped at the follow distance: stay until they are restart_gap further
            self.body.brake(dt)
        else:
            near = navigation.follow_step(
                self.navigator, self.body, self.last_operator_pos,
                self.shapes(senses), senses.time, dt, self.follow_distance,
            )  # fmt: skip
        self.standing = near
        if near:
            # Waiting behind the operator: face them, so the camera keeps them in view. brake() stops the
            # turn at once, so go on from the turn so far: from 0 every cycle it only ever reached one step
            # of turn_accel (0.07 rad/s on the real base, 273 of 319 such cycles on 2026-11-15).
            if turning is not None:
                self.body.turn_speed = turning
            self.body.turn_toward(self.last_operator_pos, dt)

        # Too far behind (a detour around the crowd): ask the operator to wait for us.
        # Without a path (the operator stands in a door we need) they must go on instead.
        gap = distance(self.body.pos, self.last_operator_pos)
        has_path = bool(self.navigator.path)
        if not self.asked_to_wait and gap > self.wait_gap and has_path:
            self.speech.say(senses.time, hri.WAIT_TEXT)
            self.asked_to_wait = True
        elif self.asked_to_wait and (gap < self.go_on_gap or not has_path):
            self.speech.say(senses.time, hri.GO_ON_TEXT)
            self.asked_to_wait = False

    def _search(self, senses: Senses, dt: float) -> None:
        """We lost the operator. Go where we saw them last, look around, then ask."""
        now = senses.time
        # Found again only with a look that matches the operator (checked in watch),
        # so a stranger who walks by is not taken for them.
        if self.operator_now is not None and senses.operator_look is not None:
            self.mode = Mode.FOLLOW
            self.faced_last = False
            return
        # Face the heading we last saw them on before driving away or spinning. The camera only
        # sees 58 degrees, and a blind left turn looks at the empty side of the room first.
        if not self.faced_last and self.last_operator_pos is not None:
            self.body.turn_toward(self.last_operator_pos, dt)
            dx = self.last_operator_pos[0] - self.body.pos[0]
            dy = self.last_operator_pos[1] - self.body.pos[1]
            err = math.atan2(math.sin(math.atan2(dy, dx) - self.body.heading),
                             math.cos(math.atan2(dy, dx) - self.body.heading))
            if abs(err) < FACE_TOL or now - self.search_since >= FACE_FOR:
                self.faced_last = True
            if not self.asked_to_wait and now - self.search_since >= ASK_WAIT:
                self.speech.say(now, hri.WAIT_TEXT)
                self.asked_to_wait = True
            if not self.faced_last:
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
