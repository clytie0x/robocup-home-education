"""The Help Me Carry state machine. It decides what the robot does next.

The task never reads the car position. It only knows where it started,
and what perception tells it.
"""

import math
from enum import Enum

from hmc import hri, manipulation, navigation, perception
from hmc.appearance import LookMemory
from hmc.follower import Follower, Mode, Senses
from hmc.odometry import RobotView
from hmc.world import TIME_LIMIT, Point, World, distance

QUEUE_GAP = 1.0  # metres to keep behind the last person in the queue
QUEUE_SPOT = 0.3  # metres; close enough to our spot in the line
# The rulebook: "The queue is composed of the same people that form the crowd." So on
# the way out we remember the people who stand together in a group near us.
CROWD_NEAR_US = 4.0  # metres from the robot
CROWD_TOGETHER = 1.5  # metres between two people standing in the same group
# The crowd stands there from the start. Passers-by only stop for a moment.
CROWD_STANDING = 6.0  # seconds a person must stand in one place to count
STAND_STILL = 0.3  # metres; moving less than this is standing in one place
STANDING_FORGET = 2.0  # seconds; a place nobody was seen at for this long is empty
# The queue walks back to the start: cos of the largest angle between where a queue
# member walks and the way to the start (60 degrees leaves room for corners and doors).
QUEUE_HEADING = 0.5
# A queue that stands still this close to the start has arrived and waits there in line.
LINE_NEAR_START = 4.0  # metres
LINE_UP_FOR = 10.0  # seconds at most to take our place at the end of that line
# Close enough to the start to put the bag down. The referee counts 1 m; the queue
# waiting in its line next to the start can keep us from getting much closer.
AT_START = 0.6  # metres; less than 1 m, as our pose can be a few tenths off
# Noise handling. (Following the operator has its own, in follower.py.)
BAG_VOTES = 5  # frames that must point at the same bag (and 3 more than at any other)
RAY_FRAMES = 5  # frames of the pointing arm we average before we look for the bag
LOOK_ALONG_RAY = 2.0  # metres; to find the bag we look at this point along the arm
# Re-identification by look (see appearance.py).
CROWD_MATCH = 0.65  # a person must look at least this much like someone in the crowd
SAME_PERSON = 0.8  # two looks this similar are taken to be the same person
# If we lose sight of the last person of the queue (a corner), keep going this long.
LOST_TAIL_WAIT = 3.0  # seconds
# The queue walks on past the start, so we leave it this close to the start.
LEAVE_QUEUE_NEAR_START = 2.0  # metres
# Pace. We ask the operator to walk a bit slower than we can drive, so we keep up.
PACE_SHARE = 0.8  # the pace we ask for, as a share of our top speed


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
        # Who the operator is, following and searching (one brain for sim and robot).
        self.follower = Follower(self.body, speech)
        self.senses: Senses | None = None  # this step's senses, filled in step()
        self.perceived_bag: str | None = None
        self.queue_done = False  # we leave the queue only once
        self.tail_walked = 0.0  # when we last saw the queue's last person walking
        self.lined_up = False  # we took our place at the end of a queue waiting by the start
        self.line_since: float | None = None
        self.last_tail: tuple[float, int] | None = None  # (time, id) we last saw it
        # The crowd we passed on the way out: one look memory per person. Kept by look,
        # not by tracker id, because tracker ids get swapped.
        self.crowd: list[LookMemory] = []
        # Places where someone stands: [since when, where, last seen]. Kept by place, not by
        # tracker id: passers-by make the tracker swap ids, which would restart the clock.
        self.standing: list[list] = []
        self.bag_votes: dict[str, int] = {}
        self.rays: list = []  # pointing rays seen, averaged into self.pointing
        self.pointing = None
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
        people = perception.see_people(self.world)
        self.senses = Senses(
            time=self.world.time,
            operator=perception.see_operator(self.world),
            operator_look=perception.see_operator_look(self.world),
            people=people,
            obstacles=perception.see_obstacles(self.world),
        )
        # Every step in every state, so we also avoid the operator after following.
        self.follower.watch(self.senses)
        now = self.world.time
        for pid in people:
            pos = self.follower.people_trackers[pid].last()  # smoothed
            spot = self._spot(pos)
            if spot is None:
                self.standing.append([now, pos, now])  # someone new here: start counting
            else:
                spot[2] = now
        # A place nobody was seen at for a while is empty (they walked on).
        self.standing = [s for s in self.standing if now - s[2] <= STANDING_FORGET]
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

    def _follow(self, dt: float) -> None:
        follower = self.follower
        follower.step(self.senses, dt)
        if follower.mode == Mode.SEARCH:
            self._go(State.SEARCH)
            return
        # Arrived at the car when the operator stands still and points.
        close = follower.operator_now is not None and (
            distance(self.body.pos, follower.last_operator_pos)
            <= follower.follow_distance + 0.5
        )
        if (
            close
            and follower.tracker.is_stationary()
            and perception.see_pointing(self.world)
        ):
            self._go(State.PERCEIVE_BAG)

    def _search(self, dt: float) -> None:
        """We lost the operator. The follower looks for them, then asks them back."""
        self.follower.step(self.senses, dt)
        if self.follower.mode != Mode.SEARCH:
            self._go(State.FOLLOW)

    def _perceive_bag(self, dt: float) -> None:
        robot = self.body
        if self.pointing is None:
            # First read the arm: face the operator and average a few frames.
            ray = perception.see_pointing(self.world)
            if ray is None:
                robot.turn_toward(self.follower.last_operator_pos, dt)
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
        tail = None if self.queue_done else self._queue_tail()
        if tail is not None:
            # Remember who we saw, so one frame without them does not end the queue.
            self.last_tail = (self.world.time, tail)
            self.tail_walked = self.world.time
            self._go(State.JOIN_QUEUE)  # optional goal: join the queue on the way back
            return
        if not self.lined_up:
            # The queue may already stand in its line by the start. Joining it means
            # taking our place at its end before we go to the start ourselves.
            spot = self._end_of_line()
            if spot is not None:
                if self.line_since is None:
                    self.line_since = self.world.time
                reached = self._drive_to(spot, dt, QUEUE_SPOT)
                # Give up when the spot cannot be reached (a wall, people): the start
                # and the bag count for more than the queue.
                late = self.world.time - self.line_since > LINE_UP_FOR
                if reached or late or not self.follower.navigator.path:
                    self.lined_up = True
                return
        if self._drive_to(self.start, dt, AT_START):
            manipulation.place(self.world)  # allowed by the rules, no penalty
            self._go(State.DONE)

    def _end_of_line(self) -> Point | None:
        """Where to stand at the end of a queue that waits by the start, or None."""
        looks = perception.see_looks(self.world)
        trackers = self.follower.people_trackers
        line = [
            trackers[pid].last()
            for pid in perception.see_people(self.world)
            if self._looks_like_crowd(looks.get(pid))
            and trackers[pid].is_stationary()
            and distance(trackers[pid].last(), self.start) <= LINE_NEAR_START
        ]
        if not line:
            return None
        last = max(line, key=lambda p: distance(p, self.start))
        first = min(line, key=lambda p: distance(p, self.start))
        if first == last:
            first = self.start
        dx, dy = last[0] - first[0], last[1] - first[1]
        length = math.hypot(dx, dy) or 1.0
        return (last[0] + QUEUE_GAP * dx / length, last[1] + QUEUE_GAP * dy / length)

    def _join_queue(self, dt: float) -> None:
        if self.world.carried_bag() is None:
            self._go(State.GO_TO_BAG)  # we dropped it, fetch it again
            return
        now = self.world.time
        tail = self._queue_tail()
        if tail is not None:
            self.last_tail = (now, tail)
            self.tail_walked = now
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
        trackers = self.follower.people_trackers
        # A tail that looks stationary for a moment (a missed frame, an id swap with
        # someone standing) has not arrived yet: only a queue that stopped for
        # LOST_TAIL_WAIT has.
        stopped = (
            tail is not None
            and trackers[tail].is_stationary()
            and now - self.tail_walked > LOST_TAIL_WAIT
        )
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
        tracker = trackers[tail]
        x, y = tracker.last()
        walk = tracker.direction()
        goal = (x, y)
        if walk is not None:
            goal = (x - QUEUE_GAP * walk[0], y - QUEUE_GAP * walk[1])
        self._drive_to(goal, dt, QUEUE_SPOT)

    # Helpers

    def _drive_to(self, goal, dt: float, stop_distance: float) -> bool:
        follower = self.follower
        return follower.navigator.step(
            self.body, goal, follower.shapes(self.senses), self.world.time, dt,
            stop_distance,
        )  # fmt: skip

    def _remember_crowd(self) -> None:
        """Remember people standing in a group near us: that is the crowd."""
        robot = self.body.pos
        now = self.world.time
        standing = {
            pid: pos
            for pid, pos in perception.see_people(self.world).items()
            if (spot := self._spot(self.follower.people_trackers[pid].last())) is not None
            and now - spot[0] >= CROWD_STANDING
            # A passer-by who steps onto a crowd member's place must not take its time.
            and self.follower.people_trackers[pid].is_stationary()
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

    def _walks_to_start(self, pid: int, pos: Point) -> bool:
        """True if this person walks roughly toward the start, as the queue does.

        A passer-by who stood near the crowd for a while can look like a crowd member;
        walking anywhere else tells them apart.
        """
        walk = self.follower.people_trackers[pid].direction()
        if walk is None:
            return False
        to_start = (self.start[0] - pos[0], self.start[1] - pos[1])
        length = math.hypot(*to_start)
        return length < 1e-9 or (walk[0] * to_start[0] + walk[1] * to_start[1]) / length >= QUEUE_HEADING

    def _spot(self, pos):
        """The place record someone at pos stands on, or None."""
        near = [s for s in self.standing if distance(pos, s[1]) <= STAND_STILL]
        return min(near, key=lambda s: distance(pos, s[1])) if near else None

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
            and self._walks_to_start(pid, pos)
            and distance(pos, self.start) < distance(robot, self.start)
        ]
        if not ahead:
            return None
        return min(ahead, key=lambda seen: distance(seen[1], robot))[0]
