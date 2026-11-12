"""The Help Me Carry state machine. It decides what the robot does next.

The task never reads the car position. It only knows where it started,
and what perception tells it.
"""

from enum import Enum

from hmc import hri, manipulation, navigation, perception
from hmc.appearance import LookMemory
from hmc.world import PERSON_RADIUS, TIME_LIMIT, Circle, Point, Shape, World, distance

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
REMEMBER_FOR = 1.0  # seconds we still avoid a person the detector missed
# Re-identification by look (see appearance.py).
OPERATOR_LEARN = 20  # frames of the operator's look we learn at the start
OPERATOR_MATCH = 0.65  # a detection must look at least this much like the operator
CROWD_MATCH = 0.65  # a person must look at least this much like someone in the crowd
SAME_PERSON = 0.8  # two looks this similar are taken to be the same person
# If we lose sight of the last person of the queue (a corner), keep going this long.
LOST_TAIL_WAIT = 3.0  # seconds
# The queue walks on past the start, so we leave it this close to the start.
LEAVE_QUEUE_NEAR_START = 2.0  # metres


class State(Enum):
    ANNOUNCE_READY = "announce_ready"  # tell the operator we can follow
    FOLLOW = "follow"  # follow the operator towards the unknown car
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
    State.FOLLOW: {State.PERCEIVE_BAG, State.TIMEOUT},
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
        self.start = world.robot.pos  # remembered from odometry
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
        self.bag_tracker = perception.MotionTracker()

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
        if self.state == State.FOLLOW:
            self._remember_crowd()

        handlers = {
            State.ANNOUNCE_READY: self._announce_ready,
            State.FOLLOW: self._follow,
            State.PERCEIVE_BAG: self._perceive_bag,
            State.GO_TO_BAG: self._go_to_bag,
            State.PICK: self._pick,
            State.RETURN: self._return,
            State.JOIN_QUEUE: self._join_queue,
        }
        handlers[self.state](dt)

    def _announce_ready(self, dt: float) -> None:
        self.speech.say(self.world.time, hri.READY_TEXT)
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
        navigation.follow_step(
            self.navigator, self.world.robot, self.last_operator_pos,
            self._shapes(), self.world.time, dt,
        )  # fmt: skip

        # Arrived at the car when the operator stands still and points.
        close = operator is not None and (
            distance(self.world.robot.pos, self.last_operator_pos)
            <= navigation.FOLLOW_DISTANCE + 0.5
        )
        if (
            close
            and self.tracker.is_stationary()
            and perception.see_pointing(self.world)
        ):
            self._go(State.PERCEIVE_BAG)

    def _perceive_bag(self, dt: float) -> None:
        ray = perception.see_pointing(self.world)
        if ray is None:
            return
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
        robot = self.world.robot.pos
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
        shapes = list(perception.see_obstacles(self.world))
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
            self.world.robot, goal, self._shapes(), self.world.time, dt, stop_distance
        )

    def _remember_crowd(self) -> None:
        """Remember people standing in a group near us: that is the crowd."""
        robot = self.world.robot.pos
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
        robot = self.world.robot.pos
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
