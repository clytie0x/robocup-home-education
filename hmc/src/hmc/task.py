"""The Help Me Carry state machine. It decides what the robot does next.

The task never reads the car position. It only knows where it started,
and what perception tells it.
"""

import math
from enum import Enum

from hmc import hri, manipulation, navigation, perception
from hmc.follower import Follower, Mode, Senses
from hmc.odometry import RobotView
from hmc.queue import QUEUE_GAP, QUEUE_SPOT, QueueFollower
from hmc.route import Returner
from hmc.world import TIME_LIMIT, Point, World, distance

# The crowd and the queue are in queue.py. Who stands where is kept here, by place:
STAND_STILL = 0.3  # metres; moving less than this is standing in one place
STANDING_FORGET = 2.0  # seconds; a place nobody was seen at for this long is empty
# A queue that stands still this close to the start has arrived and waits there in line.
LINE_NEAR_START = 4.0  # metres
LINE_UP_FOR = 10.0  # seconds at most to take our place at the end of that line
# Noise handling. (Following the operator has its own, in follower.py.)
BAG_VOTES = 5  # frames that must point at the same bag (and 3 more than at any other)
RAY_FRAMES = 5  # frames of the pointing arm we average before we look for the bag
LOOK_ALONG_RAY = 2.0  # metres; to find the bag we look at this point along the arm
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
        self.start_heading = self.body.heading
        # Who the operator is, following and searching (one brain for sim and robot).
        self.follower = Follower(self.body, speech)
        self.senses: Senses | None = None  # this step's senses, filled in step()
        self.perceived_bag: str | None = None
        # The crowd on the way out and the queue on the way back.
        self.queue = QueueFollower(self.body, self.start, self.follower.people_trackers)
        self.lined_up = False  # we took our place at the end of a queue waiting by the start
        self.line_since: float | None = None
        # Drives the follower's breadcrumbs back to the start. Made anew each time we
        # start driving back (after the queue we stand somewhere else on the route).
        self.returner: Returner | None = None
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
        self.returner = None

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
            looks={pid: look for pid, look in perception.see_looks(self.world).items() if look is not None},
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
            # Since when someone stands on each person's place, for the crowd.
            standing = {
                pid: (spot[0], pos)
                for pid, pos in people.items()
                if (spot := self._spot(self.follower.people_trackers[pid].last())) is not None
            }
            self.queue.remember_crowd(self.senses, standing)

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
            self.speech.say(self.world.time, hri.RETURN_TEXT)
            self._go(State.RETURN)
        else:
            self._go(State.GO_TO_BAG)

    def _return(self, dt: float) -> None:
        if self.world.carried_bag() is None:
            self._go(State.GO_TO_BAG)  # we dropped it, fetch it again
            return
        if self.queue.tail(self.senses) is not None:
            self._go(State.JOIN_QUEUE)  # optional goal: join the queue on the way back
            return
        if not self.lined_up:
            # The queue may already stand in its line by the start. Joining it means
            # taking our place at its end before we go to the start ourselves.
            spot = self._end_of_line()
            if spot is not None:
                if self.line_since is None:
                    self.line_since = self.world.time
                self.returner = None
                reached = self._drive_to(spot, dt, QUEUE_SPOT)
                # Give up when the spot cannot be reached (a wall, people): the start
                # and the bag count for more than the queue.
                late = self.world.time - self.line_since > LINE_UP_FOR
                if reached or late or not self.follower.navigator.path:
                    self.lined_up = True
                return
        # Back along the breadcrumbs of the way out, not straight at the start: that
        # way is known to be passable, and a straight plan can end in a dead end.
        follower = self.follower
        if self.returner is None:
            self.returner = Returner(
                self.body, self.speech, follower.navigator, follower.route, self.start_heading
            )
        self.returner.step(follower.shapes(self.senses), self.world.time, dt)
        if self.returner.done:
            manipulation.place(self.world)  # allowed by the rules, no penalty
            self._go(State.DONE)

    def _end_of_line(self) -> Point | None:
        """Where to stand at the end of a queue that waits by the start, or None."""
        looks = perception.see_looks(self.world)
        trackers = self.follower.people_trackers
        line = [
            trackers[pid].last()
            for pid in perception.see_people(self.world)
            if self.queue.looks_like_crowd(looks.get(pid))
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
        follower = self.follower
        shapes = follower.shapes(self.senses)
        if not self.queue.step(self.senses, shapes, follower.navigator, dt):
            self._go(State.RETURN)

    # Helpers

    def _drive_to(self, goal, dt: float, stop_distance: float) -> bool:
        follower = self.follower
        return follower.navigator.step(
            self.body, goal, follower.shapes(self.senses), self.world.time, dt,
            stop_distance,
        )  # fmt: skip

    def _spot(self, pos):
        """The place record someone at pos stands on, or None."""
        near = [s for s in self.standing if distance(pos, s[1]) <= STAND_STILL]
        return min(near, key=lambda s: distance(pos, s[1])) if near else None
