"""The power cable's winding (owner's request, 2026-11-15).

  python cable.py [--reset] [--path FILE]     print the net turn; --reset: the cable was just straightened

The robot runs on a cable from a wall socket, attached at its front-left. Every turn on the spot winds it,
and it must never be wound past +-180 deg (one real run searched by spinning: the odometry yaw went from
7.07 to 17.87 rad, +619 deg). Cable keeps the net turn since the owner last straightened the cable, from
the odometry yaw of every cycle while run_follow drives, in data/cable.json next to this file
(~/hmc-base/data/cable.json on the Pi): {"net_deg", "set_at"}, set_at the wall time (s) of the last
reset or null when that is not known. Written at most once a second and at the end of the run, through
a temporary file, so a crash never leaves half a file. reset() (the console's button) is taken by a
running Cable at its next write.

Rules for a command (steer; owner, 2026-11-15): from WARN_DEG on, a turn that would wind the cable more goes
the long way round when that still reaches its heading within LIMIT_DEG, else the robot stops turning and
says TEXT once; an arc that would wind more drives straight. A turn free to go either way (one with no
heading of its own: the search spin; one toward a heading FREE_TURN or more round: a goal behind, the way
back's turnaround, an escape) goes the way that leaves |net| smaller. No command takes the net, with its
run-on after a stop, past LIMIT_DEG.

Lay: where the cable lies. It is dragged, so it lies along the path the post it hangs from (ATTACH) drove
since the cable was straightened (this run only: the odometry frame starts anew each run), the newest
cable_free_m of it. The planner gets it as small circles (plan): beside it when there is a way, else over
it, slowly (cap). The NEAR metres next to the robot move with it and are left out. Turns on the spot lay
nothing: the cable winds round the robot then, it does not trace a circle on the floor.
"""
import argparse
import json
import math
import os
import time
from pathlib import Path

from hmc.world import Circle

import geometry
import safety

PATH = Path(__file__).resolve().parent / "data" / "cable.json"  # on the Pi: ~/hmc-base/data/cable.json
WARN_DEG = 200.0  # from here a turn that winds the cable more goes the other way round, or not at all
LIMIT_DEG = 240.0  # never past this because of a command
FREE_TURN = math.radians(135)  # a turn toward a heading this far round or more may go either way: it unwinds
KEEP_WAY = math.radians(120)  # a chosen way is kept until the heading left is less than this (the plan may change)
SAVE_EVERY = 1.0  # s between writes of the file while driving
CYCLE = 0.4  # s a command may run before the next one (run_follow.SLOW_CYCLE): its run-on counts too
TEXT = "My cable is wound. Please straighten it."
ATTACH = (-0.08, 0.24)  # m, base frame: the left post the cable hangs from (seen in the scan, 2026-11-14)
CABLE_FREE_M = 8.0  # m of cable that can lie on the floor (params.json "cable_free_m" overrides)
STEP = 0.05  # m between two points of the post's path
SIMPLIFY = 0.05  # m, Douglas-Peucker tolerance
NEAR = 0.6  # m of the cable next to the robot: it moves with the robot and would trap it
SPACING, RADIUS = 0.1, 0.03  # m: the circles the planner gets
CROSS_HOLD = 5.0  # s planned over the cable once no way beside it was found, then beside it is tried again
CROSS_SPEED = 0.06  # m/s near the cable while planned over it
CROSS_NEAR = 0.3  # m from the robot's circle to the cable: near


def load(path):
    """{"net_deg", "set_at"} from the file; a missing or unreadable file is a straight cable of unknown age."""
    try:
        data = json.loads(Path(path).read_text())
        return {"net_deg": float(data["net_deg"]), "set_at": data.get("set_at")}
    except (OSError, ValueError, TypeError, KeyError):
        return {"net_deg": 0.0, "set_at": None}


def save(path, net_deg, set_at):
    """Write the file through a temporary one in the same folder, then replace (atomic)."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(json.dumps({"net_deg": round(net_deg, 2), "set_at": set_at}))
    os.replace(temporary, path)


def reset(path=PATH):
    """The owner straightened the cable: net 0 from now. Returns what was written."""
    data = {"net_deg": 0.0, "set_at": round(time.time(), 3)}
    save(path, data["net_deg"], data["set_at"])
    return data


def _wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def simplify(points, tolerance=SIMPLIFY):
    """Douglas-Peucker: the polyline's points that matter by more than `tolerance` m."""
    points = [tuple(q) for q in points]
    if len(points) < 3:
        return points
    (ax, ay), (bx, by) = points[0], points[-1]
    length = math.hypot(bx - ax, by - ay)
    if length < 1e-12:
        far = [math.dist(q, points[0]) for q in points[1:-1]]
    else:
        far = [abs((bx - ax) * (ay - y) - (ax - x) * (by - ay)) / length for x, y in points[1:-1]]
    i = max(range(len(far)), key=far.__getitem__)
    if far[i] <= tolerance:
        return [points[0], points[-1]]
    return simplify(points[:i + 2], tolerance)[:-1] + simplify(points[i + 1:], tolerance)


class Lay:
    """The cable on the floor (module docstring): the post's path in the odometry frame, newest last."""

    def __init__(self, p):
        self.p = p
        self.free = float(p.get("cable_free_m", CABLE_FREE_M))
        self.trail = []  # odometry points of the post, oldest first
        self.pose = None
        self.shapes = []  # this cycle's circles in the brain's frame (refresh)
        self.now = 0.0
        self.crossing = None  # {"since": t} while planned over the cable, else None

    def clear(self):
        """The cable was straightened: nothing lies on the floor yet."""
        self.trail, self.pose, self.shapes, self.crossing = [], None, [], None

    def update(self, pose):
        """Each cycle, with the odometry pose (x, y, yaw)."""
        post = geometry.base_to_odom(ATTACH, pose)
        if self.pose is not None:
            turned = abs(_wrap(pose[2] - self.pose[2])) * math.hypot(*ATTACH)
            moved = math.dist(pose[:2], self.pose[:2])
            if moved > turned and (not self.trail or math.dist(post, self.trail[-1]) > STEP):
                self.trail.append(post)
        elif not self.trail:
            self.trail.append(post)
        self.pose = tuple(pose)
        while len(self.trail) > 2 and self._length(self.trail[1:] + [post]) > self.free:
            self.trail.pop(0)

    @staticmethod
    def _length(points):
        return sum(math.dist(a, b) for a, b in zip(points, points[1:]))

    def _cable(self, pose):
        """The polyline from the post at `pose` back along the trail, at most the free length."""
        poly = [geometry.base_to_odom(ATTACH, pose)] + self.trail[::-1]
        out, left = [poly[0]], self.free
        for a, b in zip(poly, poly[1:]):
            d = math.dist(a, b)
            if d >= left:
                out.append((a[0] + (b[0] - a[0]) * left / d, a[1] + (b[1] - a[1]) * left / d) if d else b)
                break
            out.append(b)
            left -= d
        return out

    def length_m(self):
        """m of cable on the floor by this estimate."""
        return self._length(self._cable(self.pose)) if self.pose is not None else 0.0

    def circles(self, pose, to_frame):
        """The planner's circles: every SPACING along the lay, counted from its far end so they stay put as the
        robot turns, without the NEAR metres next to the post; moved by to_frame (odometry -> the brain's frame)."""
        if self.pose is None:
            return []
        cable = self._cable(pose)
        near = math.dist(cable[0], cable[1]) if len(cable) > 1 else 0.0  # the post to the newest trail point
        poly = simplify(cable[1:][::-1])  # the trail on the floor, far end first
        total = self._length(poly)
        out, at, base = [], 0.0, 0.0
        for a, b in zip(poly, poly[1:]):
            d = math.dist(a, b)
            while d > 0 and at <= base + d + 1e-9:
                if near + total - at >= NEAR - 1e-9:
                    k = (at - base) / d
                    out.append(Circle(*to_frame((a[0] + (b[0] - a[0]) * k, a[1] + (b[1] - a[1]) * k)), RADIUS))
                at += SPACING
            base += d
        return out

    def refresh(self, pose, to_frame, now):
        """This cycle's circles, and the end of a crossing after CROSS_HOLD."""
        self.now = now
        self.shapes = self.circles(pose, to_frame)
        if self.crossing is not None and now - self.crossing["since"] >= CROSS_HOLD - 1e-9:
            self.crossing = None

    def plan(self, planner, start, goal, shapes, near_side=False, heading=None, waiting=lambda: False):
        """planner's path beside the cable; when it finds none (not just no answer yet: waiting), over it."""
        if not self.shapes or self.crossing is not None:
            return planner(start, goal, shapes, near_side, heading)
        path = planner(start, goal, list(shapes) + self.shapes, near_side, heading)
        if path is None and not waiting():
            self.crossing = {"since": self.now}
            path = planner(start, goal, shapes, near_side, heading)
        return path

    def cap(self, where):
        """The speed cap (m/s) at `where` (the brain's frame): CROSS_SPEED near the cable while planned over it."""
        if self.crossing is None:
            return math.inf
        reach = self.p["robot_radius"] + CROSS_NEAR
        near = any(math.dist(where, (c.x, c.y)) < reach for c in self.shapes)
        return CROSS_SPEED if near else math.inf


class Cable:
    """The net turn (deg, + to the left) and the rules for commands. path None: kept in memory only."""

    def __init__(self, p, path=None, speech=None):
        self.p, self.path, self.speech = p, path, speech
        data = load(path) if path is not None else {"net_deg": 0.0, "set_at": None}
        self.net_deg, self.set_at = data["net_deg"], data["set_at"]
        self.yaw = None  # the odometry yaw at the previous update
        self.saved_at = -math.inf
        self.way = None  # +1 or -1 while a turn goes the other way round, else None
        self.open = False  # that turn has no heading of its own (the search spin)
        self.said = False  # TEXT said since |net| last fell below WARN_DEG
        self.why = None  # what steer did to this cycle's command, or None
        self.holding = False  # this cycle's turn was stopped for the cable
        self.lay = Lay(p)

    def update(self, yaw, now, pose=None):
        """The odometry yaw of this cycle (rad, wrapped or not) at monotonic time now; pose: the whole
        odometry pose, for the lay."""
        if pose is not None:
            self.lay.update(pose)
        if self.yaw is not None:
            self.net_deg += math.degrees(_wrap(yaw - self.yaw))
        self.yaw = yaw
        if abs(self.net_deg) < WARN_DEG:
            self.said = False
        if self.path is not None and now - self.saved_at >= SAVE_EVERY - 1e-6:
            self.saved_at = now
            self.save()

    def save(self):
        """Write the file; a reset written there since (another set_at) is taken first."""
        if self.path is None:
            return
        there = load(self.path)
        if there["set_at"] is not None and there["set_at"] != self.set_at:
            self.net_deg, self.set_at = there["net_deg"], there["set_at"]
            self.way, self.said = None, False
            self.lay.clear()
        save(self.path, self.net_deg, self.set_at)

    def turns(self, angle):
        """Of the turn by `angle` rad and the long way round to the same heading, the ones the rules allow."""
        ways = [angle] if angle == 0 else [angle, angle - math.copysign(2 * math.pi, angle)]
        allowed = []
        for a in ways:
            grows = a * self.net_deg > 0
            if abs(self.net_deg + math.degrees(a)) <= LIMIT_DEG and not (grows and abs(self.net_deg) >= WARN_DEG):
                allowed.append(a)
        if abs(angle) >= FREE_TURN:  # either way will do: the one that leaves the cable less wound first
            allowed.sort(key=lambda a: abs(self.net_deg + math.degrees(a)))
        return allowed

    def steer(self, v, w, error, now, planned=False):
        """The turn rate to send instead of w. error: the heading error (rad) the turn goes for, None for a
        turn without one (the search spin). planned: the turn was chosen with turns() (an escape)."""
        self.why, self.holding = None, False
        if not w or v < 0:  # no turn, or backing (always straight)
            if not w:
                self.way = None
            return w
        if planned:
            return self._hard(w, now)
        if v > 0:  # an arc: the cable rules are for turns on the spot
            self.way = None
            if w * self.net_deg > 0 and abs(self.net_deg) >= WARN_DEG:
                self.why = "arc held straight"
                return 0.0
            return self._hard(w, now)
        if self.way is not None and not self.open and error is not None:
            # the heading left the chosen way round (error is the short way: going the long way, the rest
            # is 360 deg minus it); dropping the way on the short |error| shook the robot at net 200 (review)
            left = error if error * self.way > 0 else error - math.copysign(2 * math.pi, error)
            if abs(left) < KEEP_WAY:
                self.way = None  # the heading is not far round any more: the brain's own turn from here
        if self.way is not None:  # a turn whose way was chosen: until it ends (w 0) or the robot drives
            turned = math.copysign(abs(w), self.way)
            if self.open and turned * self.net_deg > 0 and abs(self.net_deg) >= WARN_DEG:
                self.way, turned = -self.way, -turned  # the search spin: round again at the other side
            self.why = "the other way round" if turned != w else None
            return self._hard(turned, now)
        if error is None or error * w <= 0:  # no heading of its own, or slowing a turn the other way
            if error is None and w * self.net_deg > 0:  # either way will do: the one that unwinds
                self.way, self.open, self.why = -math.copysign(1.0, w), True, "the other way round"
                return self._hard(-w, now)
            return self._hard(w, now)
        allowed = self.turns(error)
        if not allowed:
            return self._wound(now)
        if allowed[0] * w < 0 or abs(error) >= FREE_TURN:  # the way is chosen once, at the start of the turn
            self.way, self.open = math.copysign(1.0, allowed[0]), False
            if allowed[0] * w < 0:
                self.why = "the other way round"
                return self._hard(-w, now)
        return self._hard(w, now)

    def _hard(self, w, now):
        """w, unless it and its run-on after a stop would take the net past LIMIT_DEG."""
        run_on = safety.run_on_poses(0.0, w, self.p, CYCLE)[-1][2]
        if w * self.net_deg > 0 and abs(self.net_deg + math.degrees(run_on)) > LIMIT_DEG:
            return self._wound(now)
        return w

    def _wound(self, now):
        self.why, self.holding = "wound", True
        if not self.said and self.speech is not None:
            self.speech.say(now, TEXT)
        self.said = True
        return 0.0


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--reset", action="store_true", help="the cable was just straightened: net turn 0 from now")
    ap.add_argument("--path", type=Path, default=PATH, help="the file (default %s)" % PATH)
    args = ap.parse_args(argv)
    print(json.dumps(reset(args.path) if args.reset else load(args.path)))


if __name__ == "__main__":
    main()
