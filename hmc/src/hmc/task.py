"""The Help Me Carry state machine. It decides what the robot does next.

The task never reads the car position. It only knows where it started,
and what perception tells it.
"""

import math
from enum import Enum

from hmc import hri, manipulation, navigation, perception
from hmc.appearance import LookMemory
from hmc.odometry import RobotView
from hmc.world import (
    PERSON_RADIUS, TIME_LIMIT, Circle, Point, Shape, World, distance,
)  # fmt: skip

QUEUE_GAP = 1.0  # metres to keep behind the last person in the queue
QUEUE_SPOT = 0.3  # metres; close enough to our spot in the line
# The rulebook: "The queue is composed of the same people that form the crowd." So on
# the way out we remember the people who stand together in a group near us.
CROWD_NEAR_US = 4.0  # metres from the robot
CROWD_TOGETHER = 1.5  # metres between two people standing in the same group
# The crowd stands there from the start. Passers-by only stop for a moment.
CROWD_STANDING = 6.0  # seconds a person must stand in one place to count
STAND_STILL = 0.3  # metres; moving less than this is standing in one place
# Noise handling.
OPERATOR_JUMP = 0.5  # metres a detection may be off from where we expect the operator
PERSON_MAX_SPEED = 2.0  # m/s; a detection further than this could walk is someone else
BAG_VOTES = 5  # frames that must point at the same bag (and 3 more than at any other)
RAY_FRAMES = 5  # frames of the pointing arm we average before we look for the bag
LOOK_ALONG_RAY = 2.0  # metres; to find the bag we look at this point along the arm
REMEMBER_FOR = 1.0  # seconds we still avoid a person the detector missed
MEMORY_CELL = 0.3  # metres; obstacles closer than this in place and size are the same
MEMORY_KEEP = 10.0  # seconds we remember an obstacle we no longer see
# Re-identification by look (see appearance.py).
OPERATOR_LEARN = 20  # frames of the operator's look we learn at the start
OPERATOR_MATCH = 0.65  # a detection must look at least this much like the operator
CROWD_MATCH = 0.65  # a person must look at least this much like someone in the crowd
SAME_PERSON = 0.8  # two looks this similar are taken to be the same person
# If we lose sight of the last person of the queue (a corner), keep going this long.
LOST_TAIL_WAIT = 3.0  # seconds
# The queue walks on past the start, so we leave it this close to the start.
LEAVE_QUEUE_NEAR_START = 2.0  # metres
# Pace. We ask the operator to walk a bit slower than we can drive, so we keep up.
PACE_SHARE = 0.8  # the pace we ask for, as a share of our top speed
WAIT_GAP = 4.0  # metres; further behind than this, we ask the operator to wait
GO_ON_GAP = 2.5  # metres; once we are this close again, we ask them to go on
# Losing the operator while following (Deus ex Machina in the rulebook).
LOST_AFTER = 2.0  # seconds without seeing the operator before we start to search
DRIVE_FOR = 10.0  # seconds at most to reach the last seen spot (it may be blocked)
LOOK_AROUND = 5.0  # seconds of turning on the spot before we ask them to come back


class State(Enum):
    ANNOUNCE_READY = "announce_ready"  # tell the operator we can follow
    FOLLOW = "follow"  # follow the operator towards the unknown car
    SEARCH = "search"  # we lost the operator: look for them, then ask them back
    PERCEIVE_BAG = "perceive_bag"  # work out which bag is pointed at
    GO_TO_BAG = "go_to_bag"
    PICK = "pick"
    RETURN = "return"  # carry the bag back to the start
    JOIN_QUEUE = "join_queue"  # walk back behind the last person of the queue
    DONE = "done"
    TIMEOUT = "timeout"


# Which state may follow which. Anything else is a bug.
ALLOWED = {
    State.ANNOUNCE_READY: {State.FOLLOW, State.TIMEOUT},
    State.FOLLOW: {State.PERCEIVE_BAG, State.SEARCH, State.TIMEOUT},
    State.SEARCH: {State.FOLLOW, State.TIMEOUT},
    State.PERCEIVE_BAG: {State.GO_TO_BAG, State.TIMEOUT},
    State.GO_TO_BAG: {State.PICK, State.TIMEOUT},
    State.PICK: {State.RETURN, State.GO_TO_BAG, State.TIMEOUT},
    State.RETURN: {State.DONE, State.GO_TO_BAG, State.JOIN_QUEUE, State.TIMEOUT},
    State.JOIN_QUEUE: {State.RETURN, State.GO_TO_BAG, State.TIMEOUT},
    State.DONE: set(),
    State.TIMEOUT: set(),
}


class Task:
    def __init__(self, world: World, speech: hri.Speech):
        self.world = world
        self.speech = speech
        self.state = State.ANNOUNCE_READY
        self.history: list[tuple[float, State]] = [(world.time, self.state)]
        self.body = RobotView(world)  # our pose as we believe it (odometry)
        self.start = self.body.pos  # remembered from odometry
        self.tracker = perception.MotionTracker()
        self.people_trackers: dict[int, perception.MotionTracker] = {}
        self.navigator = navigation.Navigator()
        self.perceived_bag: str | None = None
        self.last_operator_pos = None
        self.operator_now: Point | None = None  # this frame's detection of the operator
        self.queue_done = False  # we leave the queue only once
        self.last_tail: tuple[float, int] | None = None  # (time, id) we last saw it
        # The crowd we passed on the way out: one look memory per person. Kept by look,
        # not by tracker id, because tracker ids get swapped.
        self.crowd: list[LookMemory] = []
        self.operator_look = LookMemory()
        self.standing: dict[int, tuple[float, tuple]] = {}  # id: (since when, where)
        self.bag_votes: dict[str, int] = {}
        self.rays: list = []  # pointing rays seen, averaged into self.pointing
        self.pointing = None
        # Obstacles we saw before. The camera looks forward only and the depth image is
        # blind closer than 0.6 m, so a low object we pass would vanish without this.
        self.obstacle_memory: dict = {}  # rough place and size -> (time, shape)
        self.bag_tracker = perception.MotionTracker()
        self.asked_to_wait = False
        # Searching for the lost operator: when we started to turn, and if we asked.
        self.search_since = 0.0
        self.look_since: float | None = None
        self.asked_back = False

    @property
    def finished(self) -> bool:
        return self.state in (State.DONE, State.TIMEOUT)

    def _go(self, new_state: State) -> None:
        if new_state not in ALLOWED[self.state]:
            raise ValueError(f"illegal transition {self.state} -> {new_state}")
        self.state = new_state
        self.history.append((self.world.time, new_state))

    def step(self, dt: float) -> None:
        if self.finished:
            return
        if self.world.time >= TIME_LIMIT:
            self._go(State.TIMEOUT)
            return
        for pid, pos in perception.see_people(self.world).items():
            tracker = self.people_trackers.setdefault(pid, perception.MotionTracker())
            tracker.update(self.world.time, pos)
            pos = tracker.last()  # smoothed
            since, spot = self.standing.get(pid, (self.world.time, pos))
            if distance(pos, spot) > STAND_STILL:
                since, spot = self.world.time, pos  # they moved: start counting again
            self.standing[pid] = (since, spot)
        self.operator_now = self._watch_operator()
        if self.state in (State.FOLLOW, State.SEARCH):
            self._remember_crowd()

        handlers = {
            State.ANNOUNCE_READY: self._announce_ready,
            State.FOLLOW: self._follow,
            State.SEARCH: self._search,
            State.PERCEIVE_BAG: self._perceive_bag,
            State.GO_TO_BAG: self._go_to_bag,
            State.PICK: self._pick,
            State.RETURN: self._return,
            State.JOIN_QUEUE: self._join_queue,
        }
        handlers[self.state](dt)

    def _announce_ready(self, dt: float) -> None:
        self.speech.say(self.world.time, hri.READY_TEXT)
        pace = PACE_SHARE * self.world.robot.max_speed
        self.speech.say(self.world.time, hri.walk_slowly_text(pace))
        self._go(State.FOLLOW)

    def _watch_operator(self) -> Point | None:
        """Update where the operator is. Returns this frame's detection, or None.

        Called every frame in every state, so we also avoid the operator after following.
        """
        operator = perception.see_operator(self.world)
        look = perception.see_operator_look(self.world)
        if operator is not None and look is not None:
            if self.operator_look.count < OPERATOR_LEARN:
                self.operator_look.add(
                    look
                )  # at the start the operator is in front of us
            elif self.operator_look.similarity(look) < OPERATOR_MATCH:
                operator = None  # does not look like the operator: someone else
        if operator is not None and self.last_operator_pos is not None:
            # A detection far from where the operator can be is someone else: ignore it.
            since = self.world.time - self.tracker.last_time
            reach = OPERATOR_JUMP + PERSON_MAX_SPEED * since
            if distance(operator, self.tracker.last()) > reach:
                operator = None
        if operator is not None:
            self.tracker.update(self.world.time, operator)
            # Smoothed, and kept when we lose sight of them.
            self.last_operator_pos = self.tracker.last()
        return operator

    def _follow(self, dt: float) -> None:
        operator = self.operator_now
        if self.last_operator_pos is None:
            return
        if self.world.time - self.tracker.last_time > LOST_AFTER:
            self.search_since, self.look_since = self.world.time, None
            self.asked_back = False
            self._go(State.SEARCH)
            return
        near = navigation.follow_step(
            self.navigator, self.body, self.last_operator_pos,
            self._shapes(), self.world.time, dt,
        )  # fmt: skip
        if near:
            # Waiting behind the operator: face them, so the camera keeps them in view.
            self.body.turn_toward(self.last_operator_pos, dt)

        # Too far behind (a detour around the crowd): ask the operator to wait for us.
        # Without a path (the operator stands in a door we need) they must go on instead.
        gap = distance(self.body.pos, self.last_operator_pos)
        has_path = bool(self.navigator.path)
        if not self.asked_to_wait and gap > WAIT_GAP and has_path:
            self.speech.say(self.world.time, hri.WAIT_TEXT)
            self.asked_to_wait = True
        elif self.asked_to_wait and (gap < GO_ON_GAP or not has_path):
            self.speech.say(self.world.time, hri.GO_ON_TEXT)
            self.asked_to_wait = False

        # Arrived at the car when the operator stands still and points.
        close = operator is not None and (
            distance(self.body.pos, self.last_operator_pos)
            <= navigation.FOLLOW_DISTANCE + 0.5
        )
        if (
            close
            and self.tracker.is_stationary()
            and perception.see_pointing(self.world)
        ):
            self._go(State.PERCEIVE_BAG)

    def _search(self, dt: float) -> None:
        """We lost the operator. Go where we saw them last, look around, then ask."""
        now = self.world.time
        # Found again only with a look that matches the operator (checked in
        # _watch_operator), so a stranger who walks by is not taken for them.
        if self.operator_now is not None and perception.see_operator_look(self.world):
            self._go(State.FOLLOW)
            return
        if self.look_since is None:
            reached = self._drive_to(
                self.last_operator_pos, dt, navigation.GOAL_TOLERANCE
            )
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
            self.asked_back = True
        # Then stand still and face forward, where we asked them to stand.

    def _perceive_bag(self, dt: float) -> None:
        robot = self.body
        if self.pointing is None:
            # First read the arm: face the operator and average a few frames.
            ray = perception.see_pointing(self.world)
            if ray is None:
                robot.turn_toward(self.last_operator_pos, dt)
                return
            self.rays.append(ray)
            if len(self.rays) < RAY_FRAMES:
                return
            x = sum(r.origin[0] for r in self.rays) / len(self.rays)
            y = sum(r.origin[1] for r in self.rays) / len(self.rays)
            angle = math.atan2(
                sum(math.sin(r.angle) for r in self.rays),
                sum(math.cos(r.angle) for r in self.rays),
            )
            self.pointing = perception.Ray((x, y), angle)
        # Then look along the arm to find the bag (the camera looks forward only).
        ray = self.pointing
        spot = (
            ray.origin[0] + LOOK_ALONG_RAY * math.cos(ray.angle),
            ray.origin[1] + LOOK_ALONG_RAY * math.sin(ray.angle),
        )
        robot.turn_toward(spot, dt)
        name = perception.choose_pointed_bag(ray, perception.see_bags(self.world))
        if name is None:
            return  # not clear which bag, look again next step
        # One noisy frame is not enough: wait until one bag clearly wins the vote.
        self.bag_votes[name] = self.bag_votes.get(name, 0) + 1
        votes = sorted(self.bag_votes.values(), reverse=True) + [0]
        if self.bag_votes[name] < BAG_VOTES or votes[0] - votes[1] < 3:
            return
        self.perceived_bag = name
        self.speech.say(self.world.time, f"I see you pointing at the {name} bag.")
        self._go(State.GO_TO_BAG)

    def _go_to_bag(self, dt: float) -> None:
        seen = perception.see_bags(self.world).get(self.perceived_bag)
        if seen is not None:
            self.bag_tracker.update(self.world.time, seen)
        elif not self.bag_tracker.seen:
            return  # not seen yet
        bag_pos = self.bag_tracker.last()  # smoothed, and kept when a frame misses it
        if self._drive_to(bag_pos, dt, manipulation.PICK_RANGE / 2):
            self._go(State.PICK)

    def _pick(self, dt: float) -> None:
        if manipulation.pick(self.world, self.perceived_bag):
            self._go(State.RETURN)
        else:
            self._go(State.GO_TO_BAG)

    def _return(self, dt: float) -> None:
        if self.world.carried_bag() is None:
            self._go(State.GO_TO_BAG)  # we dropped it, fetch it again
            return
        if not self.queue_done and self._queue_tail() is not None:
            self._go(State.JOIN_QUEUE)  # optional goal: join the queue on the way back
            return
        if self._drive_to(self.start, dt, navigation.GOAL_TOLERANCE):
            manipulation.place(self.world)  # allowed by the rules, no penalty
            self._go(State.DONE)

    def _join_queue(self, dt: float) -> None:
        if self.world.carried_bag() is None:
            self._go(State.GO_TO_BAG)  # we dropped it, fetch it again
            return
        now = self.world.time
        tail = self._queue_tail()
        if tail is not None:
            self.last_tail = (now, tail)
        elif self.last_tail is not None:
            # Nobody of the queue is in view right now. If we still see the person we
            # followed, or lost them only a moment ago (a corner), keep following them.
            # The id may have been swapped to someone else: check the look too.
            followed = self.last_tail[1]
            looks = perception.see_looks(self.world)
            seen = followed in looks and self._looks_like_crowd(looks[followed])
            if seen:
                self.last_tail = (now, self.last_tail[1])
            if seen or now - self.last_tail[0] <= LOST_TAIL_WAIT:
                tail = self.last_tail[1]
        robot = self.body.pos
        # Leave when there is no queue, when we are near the start, or when the person
        # we follow has stopped walking (the queue has arrived).
        stopped = tail is not None and self.people_trackers[tail].is_stationary()
        if (
            tail is None
            or stopped
            or distance(robot, self.start) <= LEAVE_QUEUE_NEAR_START
        ):
            self.queue_done = True
            self._go(State.RETURN)
            return
        # Stand QUEUE_GAP behind the last person, in the direction they walk, so we are
        # in the line and not next to it.
        tracker = self.people_trackers[tail]
        x, y = tracker.last()
        walk = tracker.direction()
        goal = (x, y)
        if walk is not None:
            goal = (x - QUEUE_GAP * walk[0], y - QUEUE_GAP * walk[1])
        self._drive_to(goal, dt, QUEUE_SPOT)

    # Helpers

    def _shapes(self) -> list[Shape]:
        """Everything we must not touch: seen obstacles, other people and the operator."""
        for shape in perception.see_obstacles(self.world):
            # The same obstacle comes back a little moved every frame (our own pose
            # drifts), so we key it by its rough place and size and keep the newest.
            x0, y0, x1, y1 = shape.bounds()
            key = (
                round((x0 + x1) / 2 / MEMORY_CELL), round((y0 + y1) / 2 / MEMORY_CELL),
                round((x1 - x0) / MEMORY_CELL), round((y1 - y0) / MEMORY_CELL),
            )  # fmt: skip
            self.obstacle_memory[key] = (self.world.time, shape)
        # Forget what we have not seen for a while: our own pose drifts, so an old
        # memory is in the wrong place, and could block a door that is really open.
        self.obstacle_memory = {
            key: (seen, shape)
            for key, (seen, shape) in self.obstacle_memory.items()
            if self.world.time - seen <= MEMORY_KEEP
        }
        shapes = [shape for _, shape in self.obstacle_memory.values()]
        now = self.world.time
        # People we saw a moment ago are still there, even if this frame missed them.
        trackers = list(self.people_trackers.values())
        if self.last_operator_pos is not None:
            trackers.append(self.tracker)
        for tracker in trackers:
            if tracker.seen and now - tracker.last_time <= REMEMBER_FOR:
                x, y = tracker.last()
                shapes.append(Circle(x, y, PERSON_RADIUS))
        return shapes

    def _drive_to(self, goal, dt: float, stop_distance: float) -> bool:
        return self.navigator.step(
            self.body, goal, self._shapes(), self.world.time, dt, stop_distance
        )

    def _remember_crowd(self) -> None:
        """Remember people standing in a group near us: that is the crowd."""
        robot = self.body.pos
        now = self.world.time
        standing = {
            pid: pos
            for pid, pos in perception.see_people(self.world).items()
            if now - self.standing[pid][0] >= CROWD_STANDING
            and distance(pos, robot) <= CROWD_NEAR_US
        }
        looks = perception.see_looks(self.world)
        for pid, pos in standing.items():
            if looks.get(pid) is None:
                continue
            if any(
                other != pid and distance(pos, other_pos) <= CROWD_TOGETHER
                for other, other_pos in standing.items()
            ):
                self._remember_look(looks[pid])

    def _remember_look(self, look) -> None:
        """Add a look to the crowd member it matches, or remember a new member."""
        best = max(self.crowd, key=lambda memory: memory.similarity(look), default=None)
        if best is not None and best.similarity(look) >= SAME_PERSON:
            best.add(look)
        else:
            memory = LookMemory()
            memory.add(look)
            self.crowd.append(memory)

    def _looks_like_crowd(self, look) -> bool:
        """True if this look matches someone we remember from the crowd."""
        return look is not None and any(
            memory.similarity(look) >= CROWD_MATCH for memory in self.crowd
        )

    def _queue_tail(self) -> int | None:
        """The tracker id of the last person of the queue walking back ahead of us, or None.

        Only people who look like someone of the crowd count (re-ID), so passers-by
        are not taken for the queue, even when the tracker swapped their ids.
        "Ahead of us" means closer to the start than we are. The last person is the one
        of them closest to us. (Not the one farthest from the start: the queue walks on
        past the start.)
        """
        robot = self.body.pos
        looks = perception.see_looks(self.world)
        ahead = [
            (pid, pos)
            for pid, pos in perception.see_people(self.world).items()
            if self._looks_like_crowd(looks.get(pid))
            and self.people_trackers[pid].has_moved()
            and distance(pos, self.start) < distance(robot, self.start)
        ]
        if not ahead:
            return None
        return min(ahead, key=lambda seen: distance(seen[1], robot))[0]
