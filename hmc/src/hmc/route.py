"""The way back: breadcrumbs dropped on the way out, driven in reverse (teach and repeat).

The area outside the arena is not mapped. But the route the robot drove on the way out is
known to be passable, because it just drove it. So while following, the robot drops a
breadcrumb every SPACING metres at its believed position (Route), and the way back drives
to them newest first with the same planner and obstacle handling as following (Returner).
A chair that moved onto the route is driven around, because A* plans around what the
sensors see now.

Like follower.py this is one brain for the simulation and the real robot. The body needs
pos, heading, drive_toward, turn_toward and brake.
"""

import math

from hmc import hri, navigation
from hmc import world as hmc_world
from hmc.world import Point, Shape, distance, wrap_angle

SPACING = 0.5  # metres between two breadcrumbs
LOOP = 0.5  # metres; a new breadcrumb this close to an older one closes a loop
REACHED = 0.4  # metres; a breadcrumb this close counts as reached
ARRIVE = 0.3  # metres; we stop this close to the believed start
# Someone may stand at the start (the queue waits in its line next to it). Kept from it
# for SKIP_AFTER seconds, this close will do: the referee counts 1 m.
ARRIVE_BLOCKED = 0.6  # metres
SKIP_AFTER = 5.0  # seconds without reaching a breadcrumb before we aim at the next one
MAX_SKIP = 3  # breadcrumbs we may aim past
ASK_AFTER = 20.0  # seconds without reaching any before we ask to pass
FACE = math.radians(5)  # we face the start heading when this close to it
# We steer at the breadcrumb this many past the one we must reach. The navigator brakes
# for its goal, so steering at a breadcrumb 0.5 m away made the robot brake at every one.
LEAD = 3


class Route:
    """Breadcrumbs, oldest first. points[0] is the start."""

    def __init__(self, start: Point):
        self.points: list[Point] = [start]

    def add(self, pos: Point) -> bool:
        """Drop a breadcrumb if we moved SPACING on. True when one was dropped or a loop cut."""
        if distance(pos, self.points[-1]) < SPACING - 1e-9:
            return False
        # Back near an older breadcrumb (not one of the last three, those are just
        # behind us): we drove a loop, searching or waiting. Cut it out, so the way back
        # does not drive it again. The older breadcrumb stands for where we are now.
        for i, old in enumerate(self.points[:-3]):
            if distance(pos, old) <= LOOP:
                del self.points[i + 1 :]
                return True
        self.points.append(pos)
        return True

    def back(self) -> list[Point]:
        """The breadcrumbs newest first, ending at the start."""
        return self.points[::-1]


class Returner:
    """Drives the breadcrumbs of a route back to the start, then faces the start heading."""

    def __init__(self, body, speech, navigator, route: Route, start_heading: float):
        self.body = body
        self.speech = speech
        self.navigator = navigator
        self.start_heading = start_heading
        self.back = route.back()
        # Start from where we stand: the breadcrumb nearest to us (ties: the older one).
        # After a search or the queue we may stand far from the newest breadcrumb.
        here = body.pos
        self.index = min(
            range(len(self.back)), key=lambda i: (distance(here, self.back[i]), -i)
        )
        self.aim = 0  # breadcrumbs we aim past self.index, because it cannot be reached
        self.since: float | None = None  # when we last reached a breadcrumb
        self.asked = False  # asked to pass since then
        self.target: Point | None = None
        self.done = False

    def step(self, shapes: list[Shape], now: float, dt: float) -> None:
        body = self.body
        if self.done:
            body.brake(dt)
            return
        if self.since is None:
            self.since = now
        last = len(self.back) - 1  # the start
        # Passing any breadcrumb up to the one we steer at counts as reaching it (the
        # furthest wins).
        for k in range(min(self.index + self.aim + LEAD, last - 1), self.index - 1, -1):
            if distance(body.pos, self.back[k]) <= REACHED:
                self.index, self.aim, self.since, self.asked = k + 1, 0, now, False
                break
        # Not reached for SKIP_AFTER seconds (blocked, or a person stands on it): aim at
        # the next one, then the one after, up to MAX_SKIP ahead. Never past the start.
        stuck = now - self.since
        if (
            self.aim < MAX_SKIP
            and self.index + self.aim < last
            and stuck > SKIP_AFTER * (self.aim + 1)
        ):
            self.aim += 1
        if stuck > ASK_AFTER and not self.asked:
            self.speech.say(now, hri.PASS_TEXT)  # and keep trying
            self.asked = True
        target = min(self.index + self.aim, last)
        self.target = self.back[target]
        arrive = ARRIVE_BLOCKED if stuck > SKIP_AFTER else ARRIVE
        if target < last or distance(body.pos, self.target) > arrive:
            # The furthest of the next breadcrumbs that nothing stands on. A goal someone
            # stands on makes the planner aim beside it, and that side flips as they move.
            room = hmc_world.ROBOT_RADIUS + navigation.SAFETY
            goal = min(target + LEAD, last)
            free = [
                k for k in range(target, goal + 1)
                if all(s.clearance(self.back[k]) >= room for s in shapes)
            ]
            goal = free[-1] if free else goal
            stop = REACHED if goal < last else ARRIVE
            self.navigator.step(body, self.back[goal], shapes, now, dt, stop)
            return
        # At the start: face the way we started, then we are back.
        if abs(wrap_angle(body.heading - self.start_heading)) > FACE:
            x, y = body.pos
            h = self.start_heading
            body.turn_toward((x + math.cos(h), y + math.sin(h)), dt)
            return
        body.brake(dt)
        self.speech.say(now, hri.ARRIVED_TEXT)
        self.done = True
