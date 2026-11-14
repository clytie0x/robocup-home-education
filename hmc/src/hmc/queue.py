"""Joining the queue on the way back: remember the crowd, find the queue's end, walk behind it.

The rulebook: "The queue is composed of the same people that form the crowd." So on the way
out we remember the people who stand together in a group near us, by look, and on the way
back we walk behind the last of them who walks toward the start ahead of us.

Like follower.py this is one brain for the simulation and the real robot. It never sees
the world: it gets a body (pos), the people trackers the Follower keeps up to date, and
one Senses record per step.
"""

import math

from hmc.appearance import LookMemory
from hmc.follower import Senses
from hmc.perception import MotionTracker
from hmc.world import Point, Shape, distance

QUEUE_GAP = 1.0  # metres to keep behind the last person in the queue
QUEUE_SPOT = 0.3  # metres; close enough to our spot in the line
CROWD_NEAR_US = 4.0  # metres from the robot
CROWD_TOGETHER = 1.5  # metres between two people standing in the same group
# The crowd stands there from the start. Passers-by only stop for a moment.
CROWD_STANDING = 6.0  # seconds a person must stand in one place to count
# The queue walks back to the start: cos of the largest angle between where a queue
# member walks and the way to the start (60 degrees leaves room for corners and doors).
QUEUE_HEADING = 0.5
# Re-identification by look (see appearance.py).
CROWD_MATCH = 0.65  # a person must look at least this much like someone in the crowd
SAME_PERSON = 0.8  # two looks this similar are taken to be the same person
# If we lose sight of the last person of the queue (a corner), keep going this long.
LOST_TAIL_WAIT = 3.0  # seconds
# The queue walks on past the start, so we leave it this close to the start.
LEAVE_QUEUE_NEAR_START = 2.0  # metres


class QueueFollower:
    def __init__(self, body, start: Point, people_trackers: dict[int, MotionTracker]):
        self.body = body
        self.start = start
        self.people_trackers = people_trackers  # kept up to date by Follower.watch
        # The crowd we passed on the way out: one look memory per person. Kept by look,
        # not by tracker id, because tracker ids get swapped.
        self.crowd: list[LookMemory] = []
        self.done = False  # we leave the queue only once
        self.tail_walked = 0.0  # when we last saw the queue's last person walking
        self.last_tail: tuple[float, int] | None = None  # (time, id) we last saw it

    def remember_crowd(
        self, senses: Senses, standing: dict[int, tuple[float, Point]]
    ) -> None:
        """Remember people standing in a group near us: that is the crowd.

        standing: for each person in senses.people who stands on a known place, since
        when someone stands there and where we see them now.
        """
        robot = self.body.pos
        now = senses.time
        trackers = self.people_trackers
        standing = {
            pid: pos
            for pid, (since, pos) in standing.items()
            if now - since >= CROWD_STANDING
            # A passer-by who steps onto a crowd member's place must not take its time.
            and trackers[pid].is_stationary()
            and distance(pos, robot) <= CROWD_NEAR_US
        }
        for pid, pos in standing.items():
            if senses.looks.get(pid) is None:
                continue
            if any(
                other != pid and distance(pos, other_pos) <= CROWD_TOGETHER
                for other, other_pos in standing.items()
            ):
                self._remember_look(senses.looks[pid])

    def tail(self, senses: Senses) -> int | None:
        """The tracker id of the last person of the queue walking back ahead of us, or None.

        Only people who look like someone of the crowd count (re-ID), so passers-by
        are not taken for the queue, even when the tracker swapped their ids.
        "Ahead of us" means closer to the start than we are. The last person is the one
        of them closest to us. (Not the one farthest from the start: the queue walks on
        past the start.) Whoever we find is remembered, so one frame without them does
        not end the queue.
        """
        if self.done:
            return None
        robot = self.body.pos
        ahead = [
            (pid, pos)
            for pid, pos in senses.people.items()
            if self.looks_like_crowd(senses.looks.get(pid))
            and self._walks_to_start(pid, pos)
            and distance(pos, self.start) < distance(robot, self.start)
        ]
        if not ahead:
            return None
        tail = min(ahead, key=lambda seen: distance(seen[1], robot))[0]
        self.last_tail = (senses.time, tail)
        self.tail_walked = senses.time
        return tail

    def step(self, senses: Senses, shapes: list[Shape], navigator, dt: float) -> bool:
        """Drive QUEUE_GAP behind the queue's last person. False when we must leave the queue."""
        now = senses.time
        tail = self.tail(senses)
        if tail is None and self.last_tail is not None:
            # Nobody of the queue is in view right now. If we still see the person we
            # followed, or lost them only a moment ago (a corner), keep following them.
            # The id may have been swapped to someone else: check the look too.
            followed = self.last_tail[1]
            looks = senses.looks
            seen = followed in looks and self.looks_like_crowd(looks[followed])
            if seen:
                self.last_tail = (now, self.last_tail[1])
            if seen or now - self.last_tail[0] <= LOST_TAIL_WAIT:
                tail = self.last_tail[1]
        robot = self.body.pos
        # Leave when there is no queue, when we are near the start, or when the person
        # we follow has stopped walking (the queue has arrived).
        trackers = self.people_trackers
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
            self.done = True
            return False
        # Stand QUEUE_GAP behind the last person, in the direction they walk, so we are
        # in the line and not next to it.
        tracker = trackers[tail]
        x, y = tracker.last()
        walk = tracker.direction()
        goal = (x, y)
        if walk is not None:
            goal = (x - QUEUE_GAP * walk[0], y - QUEUE_GAP * walk[1])
        navigator.step(self.body, goal, shapes, now, dt, QUEUE_SPOT)
        return True

    def looks_like_crowd(self, look) -> bool:
        """True if this look matches someone we remember from the crowd."""
        return look is not None and any(
            memory.similarity(look) >= CROWD_MATCH for memory in self.crowd
        )

    def _walks_to_start(self, pid: int, pos: Point) -> bool:
        """True if this person walks roughly toward the start, as the queue does.

        A passer-by who stood near the crowd for a while can look like a crowd member;
        walking anywhere else tells them apart.
        """
        walk = self.people_trackers[pid].direction()
        if walk is None:
            return False
        to_start = (self.start[0] - pos[0], self.start[1] - pos[1])
        length = math.hypot(*to_start)
        return (
            length < 1e-9
            or (walk[0] * to_start[0] + walk[1] * to_start[1]) / length >= QUEUE_HEADING
        )

    def _remember_look(self, look) -> None:
        """Add a look to the crowd member it matches, or remember a new member."""
        best = max(self.crowd, key=lambda memory: memory.similarity(look), default=None)
        if best is not None and best.similarity(look) >= SAME_PERSON:
            best.add(look)
        else:
            memory = LookMemory()
            memory.add(look)
            self.crowd.append(memory)
