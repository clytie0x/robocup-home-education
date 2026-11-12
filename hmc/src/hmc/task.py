"""The Help Me Carry state machine. It decides what the robot does next.

The task never reads the car position. It only knows where it started,
and what perception tells it.
"""

from enum import Enum

from hmc import hri, manipulation, navigation, perception
from hmc.world import PERSON_RADIUS, TIME_LIMIT, Circle, Shape, World, distance

QUEUE_GAP = 1.0  # metres to keep behind the last person in the queue
LEAVE_QUEUE_NEAR_START = (
    2.0  # metres; the queue walks past the start, so we leave it here
)


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
        self.queue_done = False  # we leave the queue only once

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
            self.people_trackers.setdefault(pid, perception.MotionTracker()).update(
                self.world.time, pos
            )

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

    def _follow(self, dt: float) -> None:
        operator = perception.see_operator(self.world)
        if operator is not None:
            self.last_operator_pos = operator  # remember it in case we lose sight
            self.tracker.update(self.world.time, operator)
        if self.last_operator_pos is None:
            return
        navigation.follow_step(
            self.navigator, self.world.robot, self.last_operator_pos,
            self._shapes(), self.world.time, dt,
        )  # fmt: skip

        # Arrived at the car when the operator stands still and points.
        close = operator is not None and (
            distance(self.world.robot.pos, operator) <= navigation.FOLLOW_DISTANCE + 0.5
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
        self.perceived_bag = name
        self.speech.say(self.world.time, f"I see you pointing at the {name} bag.")
        self._go(State.GO_TO_BAG)

    def _go_to_bag(self, dt: float) -> None:
        bag_pos = perception.see_bags(self.world).get(self.perceived_bag)
        if bag_pos is None:
            return
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
        tail = self._queue_tail()
        if (
            tail is None
            or distance(self.world.robot.pos, self.start) <= LEAVE_QUEUE_NEAR_START
        ):
            self.queue_done = True
            self._go(State.RETURN)
            return
        self._drive_to(tail, dt, QUEUE_GAP)

    # Helpers

    def _shapes(self) -> list[Shape]:
        """Everything we must not touch: seen obstacles, other people and the operator."""
        shapes = list(perception.see_obstacles(self.world))
        for pos in perception.see_people(self.world).values():
            shapes.append(Circle(pos[0], pos[1], PERSON_RADIUS))
        operator = perception.see_operator(self.world)
        if operator is not None:
            shapes.append(Circle(operator[0], operator[1], PERSON_RADIUS))
        return shapes

    def _drive_to(self, goal, dt: float, stop_distance: float) -> bool:
        return self.navigator.step(
            self.world.robot, goal, self._shapes(), self.world.time, dt, stop_distance
        )

    def _queue_tail(self):
        """The last person of a queue walking back ahead of us, or None.

        A queue is people who are walking. "Ahead of us" means closer to the start than we
        are. The last person of the queue is the walking person ahead of us who is closest
        to us. (Not the one farthest from the start: the queue walks on past the start.)
        """
        robot = self.world.robot.pos
        ahead = [
            pos
            for pid, pos in perception.see_people(self.world).items()
            if self.people_trackers[pid].has_moved()
            and distance(pos, self.start) < distance(robot, self.start)
        ]
        if not ahead:
            return None
        return min(ahead, key=lambda pos: distance(pos, robot))
