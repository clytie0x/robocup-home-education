"""Escape from corners (owner's request, 2026-11-15).

Following or returning, the robot is stuck when for STUCK_S it has had no path or been held by the obstacle
stop, or has not moved STUCK_M nor turned STUCK_TURN, while its goal (the operator, or the breadcrumb it
drives to) is known and farther than the follow gap. Then, from the 360 deg LiDAR points (robot frame):
the freest direction is the one whose corridor (robot width + 0.1 m wide, CORRIDOR_LEN past the front edge)
is clear and longest (up to FREE_CAP), nearest the goal on a tie. If a turn on the spot toward it sweeps
clear (footprint.sweep_clear, the run-on included) and the cable allows it (cable.Cable.turns: the long way
round when that is the one allowed), turn at TURN_W; else, if the rear is clear (safety.rear_clear), back
straight up to BACK_M at wheels.REVERSE_MAX, checked again every cycle as tools/back_up.py does. Then replan.

At most MAX_ESCAPES in WINDOW s. Never while held (the page's Wait or pause, the client's wait or silence,
the wheel latch): a hold also ends a running escape. run_follow's obstacle stop still checks every command
and ends a running escape (note). An escape failed when it could do nothing, was cut short, or the robot was
stuck again before it drove RECOVERED_M; after two failed in a row it says HELP_TEXT, once until it moves on.
"""
import math

import numpy

from hmc.follower import Mode

import footprint
import safety
import wheels as wheels_module

STUCK_S = 4.0  # s
STUCK_M = 0.05  # m
STUCK_TURN = math.radians(20)  # a turn on the spot this large is not stuck
GOAL_SLACK = 0.3  # m past the follow distance: closer than this the follower stands where it should
CORRIDOR_LEN = 0.8  # m past the front edge
CORRIDOR_SIDE = 0.05  # m each side of the body: robot width + 0.1 m
STEP_DEG = 5.0  # between two directions tried
FREE_CAP = 2.0  # m; farther free counts the same, and the direction nearest the goal wins
MIN_TURN = math.radians(10)  # the freest way is ahead: no turn, back up instead
TURN_W = 0.2  # rad/s; on the spot the wheels do not turn at 0.05 rad/s, they do at 0.10 (2026-11-15)
SWEEP_STEP = math.radians(3)  # between two outline poses of a planned turn (a corner moves 1.3 cm)
DONE_TURN = math.radians(3)  # turned this close to the angle: done (its run-on after S# adds about 3 deg)
BACK_M = 0.3  # m at most; less when that is all the rear allows
BACK_TRIES = (0.3, 0.2, 0.1)  # m
BACK_SPEED = wheels_module.REVERSE_MAX  # m/s
DONE_BACK = 0.01  # m
TIME_SLACK = 2.0  # s on top of the time a move should take
CYCLE = 0.4  # s a command may run (run_follow.SLOW_CYCLE), for the run-on of a planned turn
MAX_ESCAPES, WINDOW = 2, 30.0  # escapes, s
RECOVERED_M = 0.3  # m driven after an escape: it helped
HELP_TEXT = "I am stuck. Please help me."


def _wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def goal(state, believed):
    """The bearing (rad, robot frame) of the goal run_follow's state drives to, or None when there is none to be
    stuck short of: no real wheels (a dry run), not following or returning, the operator not known (lost:
    the brain's own search) or within reach, behind the queue, or at the start."""
    if state.wheels is None:
        return None
    follower = state.follower
    if state.phase == "follow":
        if getattr(follower, "mode", None) is Mode.SEARCH:
            return None
        where = getattr(follower, "last_operator_pos", None)
        reach = getattr(follower, "follow_distance", 0.0) + GOAL_SLACK
    elif state.phase == "return" and not state.joined and state.returner is not None and not state.returner.done:
        where, reach = state.returner.target, state.p["follow_gap"]
    else:
        return None
    if where is None or math.dist(believed[:2], where) <= reach:
        return None
    return _wrap(math.atan2(where[1] - believed[1], where[0] - believed[0]) - believed[2])


def freest(points, p, bearing):
    """The direction (rad, robot frame) of the clear corridor with the most free length, nearest `bearing` on a
    tie, or None when no corridor is clear."""
    pts = numpy.asarray(points, dtype=float).reshape(-1, 2)
    heads = numpy.radians(numpy.arange(-180.0 + STEP_DEG, 180.0 + 1e-9, STEP_DEG))
    c, s = numpy.cos(heads)[:, None], numpy.sin(heads)[:, None]
    along = pts[:, 0] * c + pts[:, 1] * s
    across = -pts[:, 0] * s + pts[:, 1] * c
    inside = (along > 0) & (numpy.abs(across) <= p["robot_half_width"] + CORRIDOR_SIDE)
    free = numpy.where(inside, along, numpy.inf).min(axis=1) if len(pts) else numpy.full(len(heads), numpy.inf)
    clear = free >= safety.FRONT + CORRIDOR_LEN
    if not clear.any():
        return None
    score = numpy.minimum(free, FREE_CAP)
    off = numpy.abs(numpy.arctan2(numpy.sin(heads - bearing), numpy.cos(heads - bearing)))
    best = min(numpy.flatnonzero(clear), key=lambda i: (-score[i], off[i]))
    return float(heads[best])


def turn_clear(points, angle, p):
    """True when turning on the spot by `angle` rad, then the run-on after S#, keeps the outline clear."""
    n = max(1, math.ceil(abs(angle) / SWEEP_STEP))
    poses = [(0.0, 0.0, angle * k / n) for k in range(1, n + 1)]
    poses += [(0.0, 0.0, angle + th) for _, _, th in safety.run_on_poses(0.0, math.copysign(TURN_W, angle), p, CYCLE)]
    return footprint.sweep_clear(points, poses)


class Escape:
    """Watches for being stuck and drives the escape. robot: the brain's body (its speeds are zeroed while an
    escape drives, so it starts again from standstill); replan: () -> None, plan anew after an escape."""

    def __init__(self, p, speech, robot, replan):
        self.p, self.speech, self.robot, self.replan = p, speech, robot, replan
        self.anchor = None  # (time, pose) the robot has stayed near since
        self.blocked_since = None  # since when every cycle had no path or the obstacle stop
        self.blocked_why = None
        self.move = None  # the escape running, or None
        self.times = []  # when the escapes of the last WINDOW started
        self.failed = 0  # failed escapes in a row
        self.pending = None  # (x, y) where the last escape ended, until it is judged
        self.said = False
        self.event = None  # this cycle's escape event for the record, or None

    def command(self, now, pose, bearing, hold, scan_points, points, cable):
        """(v, w) for this cycle, or None to leave the brain's command. pose: odometry (x, y, yaw); bearing: goal()
        (used only to start one); hold: why the robot is held still, or None; scan_points: the LiDAR's (robot
        frame, all round), points: everything the stops check; cable: cable.Cable."""
        self.event = None
        if hold is not None:
            if self.move is not None:
                self._end(now, pose, "held: %s" % hold, failed=False)
            self._watch_from(now, pose)
            return None
        if self.move is not None:
            return self._drive(now, pose, points)
        if bearing is None:
            self._watch_from(now, pose)
            return None
        if self.pending is not None and math.dist(pose[:2], self.pending) >= RECOVERED_M:
            self.failed, self.pending, self.said = 0, None, False
        if self.anchor is None:
            self._watch_from(now, pose, keep_blocked=True)
        a_pose = self.anchor[1]
        if math.dist(pose[:2], a_pose[:2]) > STUCK_M or abs(_wrap(pose[2] - a_pose[2])) > STUCK_TURN:
            self._watch_from(now, pose, keep_blocked=True)
        still = now - self.anchor[0]
        blocked = 0.0 if self.blocked_since is None else now - self.blocked_since
        if max(still, blocked) < STUCK_S - 1e-6:
            return None
        why = "%s %.1f s" % (self.blocked_why, blocked) if blocked >= still else "not moved %.1f s" % still
        self._watch_from(now, pose)
        if self.pending is not None:  # stuck again before it drove on: the last escape did not help
            self.failed, self.pending = self.failed + 1, None
        self._help(now)
        self.times = [t for t in self.times if now - t < WINDOW - 1e-6]
        if len(self.times) >= MAX_ESCAPES:
            self.event = {"skipped": "%d escapes in %.0f s" % (MAX_ESCAPES, WINDOW), "why": why}
            return None
        self.times.append(now)
        self.move = self._plan(now, pose, bearing, scan_points, points, cable)
        if self.move is None:
            self.failed += 1
            self._help(now)
            self.event = {"start": "none", "why": why, "failed": self.failed,
                          "note": "no clear way to turn to and the rear is not clear"}
            return None
        start = {"start": self.move["kind"], "why": why, "failed": self.failed}
        if self.move["kind"] == "turn":
            start["deg"] = round(math.degrees(self.move["angle"]), 1)
        else:
            start["m"] = self.move["m"]
        self.event = start
        return self._drive(now, pose, points)

    def note(self, record):
        """After the cycle's stops: put the event in its record, end a running escape the obstacle stop held,
        and watch for no path or the obstacle stop."""
        stopped = record["stop"] == "obstacle"
        if stopped and self.move is not None:
            self._end(record["time"], record["pose"], "obstacle stop", failed=True)
        record["escape"] = self.event
        if self.move is None and (stopped or (record["phase"] in ("follow", "return") and not record["path"])):
            self.blocked_why = "obstacle stop" if stopped else "no path"
            if self.blocked_since is None:
                self.blocked_since = record["time"]
        else:
            self.blocked_since = None

    def _watch_from(self, now, pose, keep_blocked=False):
        self.anchor = (now, tuple(pose))
        if not keep_blocked:
            self.blocked_since = None

    def _help(self, now):
        if self.failed >= 2 and not self.said:
            self.speech.say(now, HELP_TEXT)
            self.said = True

    def _plan(self, now, pose, bearing, scan_points, points, cable):
        way = freest(scan_points, self.p, bearing)
        if way is not None and abs(way) >= MIN_TURN:
            for angle in cable.turns(way):
                if turn_clear(points, angle, self.p):
                    return {"kind": "turn", "angle": angle, "turned": 0.0, "yaw": pose[2],
                            "until": now + abs(angle) / TURN_W + TIME_SLACK}
        for m in BACK_TRIES:
            if safety.rear_clear(points, BACK_SPEED, self.p, m):
                return {"kind": "back", "m": m, "start": tuple(pose), "until": now + m / BACK_SPEED + TIME_SLACK}
        return None

    def _drive(self, now, pose, points):
        move = self.move
        if now > move["until"] + 1e-6:
            return self._end(now, pose, "time", failed=True)
        if move["kind"] == "turn":
            move["turned"] += _wrap(pose[2] - move["yaw"])
            move["yaw"] = pose[2]
            if (move["angle"] - move["turned"]) * math.copysign(1.0, move["angle"]) <= DONE_TURN:
                return self._end(now, pose, "done")
            command = (0.0, math.copysign(TURN_W, move["angle"]))
        else:
            x0, y0, th0 = move["start"]
            back = -((pose[0] - x0) * math.cos(th0) + (pose[1] - y0) * math.sin(th0))
            left = move["m"] - back
            if left <= DONE_BACK:
                return self._end(now, pose, "done")
            if not safety.rear_clear(points, BACK_SPEED, self.p, left):
                return self._end(now, pose, "rear blocked", failed=True)
            command = (-BACK_SPEED, 0.0)
        self.robot.speed = self.robot.turn_speed = 0.0
        return command

    def _end(self, now, pose, result, failed=False):
        """End the running escape; (0, 0) for this cycle, then the brain plans anew."""
        self.event = {"end": self.move["kind"], "result": result}
        self.move = None
        if failed:
            self.failed += 1
            self.pending = None
            self._help(now)
        else:
            self.pending = (pose[0], pose[1])
        self._watch_from(now, pose)
        self.robot.speed = self.robot.turn_speed = 0.0
        self.replan()
        return (0.0, 0.0)
