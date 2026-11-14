"""Escape from corners (escape.py): find the freest way out, turn toward it or back up, then replan."""
import math
import unittest
from types import SimpleNamespace

import numpy

import cable
import escape
import params
import wheels

P = params.load()


def wall(x0, y0, x1, y1, step=0.02):
    """Base-frame points every `step` metres from (x0, y0) to (x1, y1)."""
    n = max(1, int(math.dist((x0, y0), (x1, y1)) / step))
    return [(x0 + (x1 - x0) * k / n, y0 + (y1 - y0) * k / n) for k in range(n + 1)]


CORNER = numpy.array(wall(0.45, -1.0, 0.45, 1.0) + wall(-1.0, 0.45, 0.45, 0.45))  # in front and to the left
FRONT = numpy.array(wall(0.45, -1.0, 0.45, 1.0))
BOXED = numpy.array([(0.35 * math.cos(a), 0.35 * math.sin(a)) for a in numpy.linspace(-math.pi, math.pi, 400)])
# A dead end as wide as the body and a little more: no turn fits (the corners reach 0.256 m), backing does.
DEAD_END = numpy.array(wall(0.3, -0.28, 0.3, 0.28) + wall(-0.1, 0.28, 0.6, 0.28) + wall(-0.1, -0.28, 0.6, -0.28))


class Said:
    def __init__(self):
        self.lines = []

    def say(self, time, text):
        self.lines.append(text)


class Escapes(unittest.TestCase):
    def setUp(self):
        self.speech = Said()
        self.robot = SimpleNamespace(speed=0.1, turn_speed=0.2)
        self.replans = []
        self.escape = escape.Escape(P, self.speech, self.robot, lambda: self.replans.append(True))
        self.cable = cable.Cable(P, None)

    def cycle(self, now, pose, points, bearing=0.0, hold=None):
        return self.escape.command(now, pose, bearing, hold, points, points, self.cable)

    def stand(self, points, until, start=0.0, pose=(0.0, 0.0, 0.0), **kw):
        """Cycles every 0.1 s from start to until with the robot standing still, up to the first escape command.
        Returns (time, command) pairs."""
        out = []
        for i in range(int(round(start * 10)), int(round(until * 10)) + 1):
            now = round(0.1 * i, 1)
            out.append((now, self.cycle(now, pose, points, **kw)))
            if out[-1][1] is not None:
                break
        return out

    def first(self, out):
        return next(((t, c) for t, c in out if c is not None), (None, None))

    def test_the_freest_way_is_the_open_side_nearest_the_goal(self):
        a = escape.freest(CORNER, P, 0.0)
        self.assertTrue(math.radians(-136) <= a <= math.radians(-79), math.degrees(a))
        self.assertIsNone(escape.freest(BOXED, P, 0.0))

    def test_four_seconds_without_moving_turn_toward_the_free_side_then_replan(self):
        t, command = self.first(self.stand(CORNER, 5.0))
        self.assertAlmostEqual(t, escape.STUCK_S)
        self.assertEqual(command, (0.0, -escape.TURN_W))
        self.assertGreaterEqual(escape.TURN_W, 0.15)  # the wheels do not turn on the spot at 0.05 rad/s
        self.assertEqual(self.escape.event["start"], "turn")
        self.assertIn("not moved", self.escape.event["why"])
        self.assertEqual((self.robot.speed, self.robot.turn_speed), (0.0, 0.0))  # the brain starts again from standstill
        yaw, now = 0.0, t
        while command != (0.0, 0.0):
            yaw, now = yaw - escape.TURN_W * 0.1, now + 0.1
            command = self.cycle(now, (0.0, 0.0, yaw), CORNER)
            self.assertLess(now, t + 15.0)
        self.assertEqual(self.escape.event, {"end": "turn", "result": "done"})
        self.assertEqual(self.replans, [True])
        self.assertTrue(math.radians(-136) <= yaw <= math.radians(-75), math.degrees(yaw))

    def test_no_safe_turn_backs_up_straight_at_the_reverse_speed(self):
        t, command = self.first(self.stand(DEAD_END, 5.0))
        self.assertEqual(command, (-wheels.REVERSE_MAX, 0.0))
        self.assertEqual(self.escape.event["start"], "back")
        x, now = 0.0, t
        while command != (0.0, 0.0):
            x, now = x - wheels.REVERSE_MAX * 0.1, now + 0.1
            command = self.cycle(now, (x, 0.0, 0.0), DEAD_END - [x, 0.0])
            self.assertLess(now, t + 10.0)
        self.assertEqual(self.escape.event, {"end": "back", "result": "done"})
        self.assertAlmostEqual(x, -escape.BACK_M, delta=0.01)

    def test_backing_stops_when_the_rear_closes(self):
        t, command = self.first(self.stand(DEAD_END, 5.0))
        behind = numpy.vstack([DEAD_END, wall(-0.3, -0.3, -0.3, 0.3)])  # someone stepped in behind
        self.assertEqual(self.cycle(t + 0.1, (0.0, 0.0, 0.0), behind), (0.0, 0.0))
        self.assertEqual(self.escape.event, {"end": "back", "result": "rear blocked"})
        self.assertEqual(self.escape.failed, 1)

    def test_holds_and_a_near_or_unknown_goal_never_escape_and_a_hold_ends_a_running_one(self):
        for kw in ({"hold": "page"}, {"hold": "wheels"}, {"hold": "silent"}, {"bearing": None}):
            self.setUp()
            self.assertEqual(self.first(self.stand(CORNER, 10.0, **kw)), (None, None), kw)
        self.setUp()
        t, command = self.first(self.stand(CORNER, 5.0))
        self.assertIsNotNone(command)
        self.assertIsNone(self.cycle(t + 0.1, (0.0, 0.0, 0.0), CORNER, hold="wait"))
        self.assertEqual(self.escape.event, {"end": "turn", "result": "held: wait"})
        self.assertEqual(self.escape.failed, 0)

    def test_at_most_two_escapes_in_30_s_and_help_once_after_two_failed_in_a_row(self):
        events = []  # no corridor, no room behind: every escape fails at once
        for i in range(401):
            self.cycle(round(0.1 * i, 1), (0.0, 0.0, 0.0), BOXED)
            if self.escape.event:
                events.append((round(0.1 * i, 1), dict(self.escape.event)))
        starts = [t for t, e in events if "start" in e]
        self.assertEqual(starts, [4.0, 8.0, 36.0, 40.0])  # 12 s to 32 s: 2 in the last 30 s, skipped
        self.assertTrue(any("skipped" in e for _, e in events))
        self.assertEqual(self.speech.lines, [escape.HELP_TEXT])

    def test_an_escape_that_got_the_robot_moving_starts_the_count_again(self):
        t, command = self.first(self.stand(CORNER, 5.0))
        self.escape.note({"time": t, "pose": [0.0, 0.0, 0.0], "stop": "obstacle", "path": 1, "phase": "follow"})
        self.assertEqual(self.escape.event, {"end": "turn", "result": "obstacle stop"})  # the stop overrides it
        self.assertEqual(self.escape.failed, 1)
        self.escape.failed, self.escape.pending = 1, (0.0, 0.0)
        self.cycle(t + 0.1, (escape.RECOVERED_M + 0.01, 0.0, 0.0), CORNER)
        self.assertEqual(self.escape.failed, 0)

    def test_no_path_for_four_seconds_counts_while_it_still_creeps(self):
        for i in range(41):
            now = round(0.1 * i, 1)
            command = self.cycle(now, (0.003 * i, 0.0, 0.0), CORNER)  # 0.12 m in 4 s: it moved
            self.escape.note({"time": now, "pose": [0.003 * i, 0.0, 0.0], "stop": None, "path": 0, "phase": "follow"})
        self.assertIsNotNone(command)
        self.assertIn("no path", self.escape.event["why"])

    def test_a_turn_on_the_spot_is_not_stuck(self):
        out = [self.cycle(round(0.1 * i, 1), (0.0, 0.0, 0.05 * i), CORNER) for i in range(80)]
        self.assertEqual(out, [None] * 80)

    def test_the_escape_turn_goes_the_long_way_round_a_wound_cable(self):
        self.cable.net_deg = 170.0  # +80 would end at 250, past cable.LIMIT_DEG
        t, command = self.first(self.stand(FRONT, 5.0, bearing=math.radians(60)))
        self.assertEqual(command, (0.0, -escape.TURN_W))  # the freest near the goal is left: round to the right
        self.assertLess(self.escape.move["angle"], -math.pi)
        self.setUp()  # a straight cable
        t, command = self.first(self.stand(FRONT, 5.0, bearing=math.radians(60)))
        self.assertEqual(command, (0.0, escape.TURN_W))


if __name__ == "__main__":
    unittest.main()
