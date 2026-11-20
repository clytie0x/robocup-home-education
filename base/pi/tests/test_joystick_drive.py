import contextlib
import io
import json
import os
import signal
import struct
import sys
import unittest
from pathlib import Path

import lidar
import params
import wheels
from test_scanmatch import room_scan

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "tools"))
import joystick_drive  # noqa: E402

P = params.load()
HELD = {5: 1}  # R1, the deadman


class CommandTest(unittest.TestCase):
    def test_parse_event_removes_the_init_flag(self):
        self.assertEqual(
            joystick_drive.parse_event(struct.pack("IhBB", 1000, -32767, 0x82, 1)),
            (2, 1, -32767),
        )
        self.assertEqual(
            joystick_drive.parse_event(struct.pack("IhBB", 5, 1, 0x01, 5)), (1, 5, 1)
        )

    def test_stick_up_is_forward_at_the_asked_top_speed(self):
        kind, v, w = joystick_drive.command({1: -32767}, HELD, P, 0.08)
        self.assertEqual(kind, "forward")
        self.assertAlmostEqual(v, 0.08)
        self.assertAlmostEqual(w, 0.0)

    def test_speed_never_above_params_max_speed(self):
        _, v, _ = joystick_drive.command({1: -32767}, HELD, P, 0.5)
        self.assertAlmostEqual(v, P["max_speed"])

    def test_no_deadman_is_stop_whatever_the_sticks_say(self):
        self.assertEqual(
            joystick_drive.command({1: -32767, 3: 32767}, {}, P, 0.08),
            ("stop", 0.0, 0.0),
        )
        self.assertEqual(
            joystick_drive.command({1: -32767}, {5: 0}, P, 0.08), ("stop", 0.0, 0.0)
        )

    def test_dead_zone_is_stop_and_half_stick_is_slower(self):
        self.assertEqual(
            joystick_drive.command({1: -3000, 3: 3000}, HELD, P, 0.08),
            ("stop", 0.0, 0.0),
        )
        _, v, _ = joystick_drive.command({1: -16384}, HELD, P, 0.08)
        self.assertTrue(0 < v < 0.08)

    def test_stick_down_is_reverse_and_ignores_the_turn(self):
        self.assertEqual(
            joystick_drive.command({1: 32767}, HELD, P, 0.08), ("reverse", 0.04, 0.0)
        )
        self.assertEqual(
            joystick_drive.command({1: 32767, 3: 32767}, HELD, P, 0.08),
            ("reverse", 0.04, 0.0),
        )

    def test_right_stick_right_turns_clockwise_on_the_spot(self):
        kind, v, w = joystick_drive.command({3: 32767}, HELD, P, 0.08)
        self.assertEqual(kind, "forward")
        self.assertEqual(v, 0.0)
        self.assertAlmostEqual(w, -P["max_turn_rate"])

    def test_a_small_turn_still_moves_the_wheels(self):
        _, _, w = joystick_drive.command(
            {3: 12000}, HELD, P, 0.08
        )  # just outside the dead zone
        self.assertGreaterEqual(abs(w), 0.10)  # wheels.MIN_TURN

    def test_both_sticks_full_stay_inside_the_limits(self):
        _, v, w = joystick_drive.command({1: -32767, 3: -32767}, HELD, P, 0.5)
        self.assertLessEqual(v, P["max_speed"] + 1e-9)
        self.assertLessEqual(abs(w), P["max_turn_rate"] + 1e-9)
        self.assertGreater(w, 0)  # stick left is a left, positive turn


class Clock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t

    def sleep(self, seconds):
        self.t += seconds


class FakeWheels:
    def __init__(self):
        self.calls = []

    def send(self, v, w):
        self.calls.append(("send", v, w))

    def send_reverse(self, speed, w):
        self.calls.append(("reverse", speed, w))

    def stop(self):
        self.calls.append("stop")

    def close(self):
        self.calls.append("close")


class FakeSession:
    def __init__(self, scan):
        self.scan, self.closed, self.error = scan, False, None

    def latest(self):
        if self.error:
            raise self.error
        return self.scan

    def close(self):
        self.closed = True


class FakePad:
    """Applies one scripted (axes, buttons) state per poll(); "gone" raises OSError. After the script it keeps the
    last state, as a held stick does (a js device sends events only when something changes)."""

    def __init__(self, script):
        self.script, self.i, self.axes, self.buttons, self.closed = (
            list(script),
            0,
            {},
            {},
            False,
        )

    def poll(self):
        if self.i < len(self.script):
            item = self.script[self.i]
            self.i += 1
            if item == "gone":
                raise OSError("no such device")
            self.axes, self.buttons = item

    def close(self):
        self.closed = True


UP, DOWN, RELEASED = ({1: -32767}, HELD), ({1: 32767}, HELD), ({}, {})
OPEN_ROOM = room_scan((0.0, 0.0, 0.0), half=2.0)
FRONT_WALL = room_scan(
    ((2.0 - 0.20 - 0.10), 0.0, 0.0), half=2.0
)  # 0.10 m ahead of the front edge, inside the stopping distance
REAR_WALL = room_scan(
    (-(2.0 - 0.20 - 0.10), 0.0, 0.0), half=2.0
)  # 0.10 m behind the back edge
SIDE_WALL = room_scan((0.0, 2.0 - 0.20 - 0.05, 0.0), half=2.0)  # 0.05 m from the left wall: a turn swings a corner into it


def run(script, scan=OPEN_ROOM, seconds=2.0, max_speed=0.08, error=None):
    clock, w, session, log = Clock(), FakeWheels(), FakeSession(scan), []
    session.error = error
    result = joystick_drive.drive(
        FakePad(script),
        w,
        session,
        P,
        max_speed,
        seconds,
        clock,
        clock.sleep,
        log.append,
    )
    return result, w.calls, log


def moves(calls):
    return [c for c in calls if c not in ("stop", "close")]


class DriveTest(unittest.TestCase):
    def test_deadman_held_at_start_moves_nothing_until_released(self):
        _, calls, _ = run([UP] * 10)
        self.assertEqual(moves(calls), [])  # never released: never armed
        _, calls, _ = run([UP] * 5 + [RELEASED] * 2 + [UP])
        self.assertTrue(moves(calls))
        self.assertEqual(calls[0][0], "send")

    def test_forward_then_release_stops(self):
        result, calls, _ = run([RELEASED] * 2 + [UP] * 10 + [RELEASED])
        sends = [c for c in calls if c != "stop"]
        self.assertEqual(sends, [("send", 0.08, 0.0)] * 10)
        self.assertEqual(calls[-1], "stop")
        self.assertEqual(result["stop"], "time")

    def test_front_wall_stops_forward(self):
        result, calls, log = run([RELEASED] + [UP] * 10, scan=FRONT_WALL)
        self.assertEqual(moves(calls), [])
        self.assertIn("front blocked", " ".join(log))

    def test_rear_wall_blocks_reverse_and_an_open_room_allows_it(self):
        _, calls, log = run([RELEASED] + [DOWN] * 10, scan=REAR_WALL)
        self.assertEqual(moves(calls), [])
        self.assertIn("rear blocked", " ".join(log))
        _, calls, _ = run([RELEASED] + [DOWN] * 3)
        self.assertEqual(moves(calls)[0], ("reverse", 0.04, 0.0))

    def test_an_empty_scan_never_drives(self):
        _, calls, log = run([RELEASED] + [UP] * 10, scan=[])
        self.assertEqual(moves(calls), [])
        self.assertIn("no LiDAR points", " ".join(log))

    def test_controller_gone_stops(self):
        result, calls, _ = run([RELEASED] + [UP] * 5 + ["gone"])
        self.assertEqual(result["stop"], "controller gone")
        self.assertEqual(calls[-1], "stop")

    def test_lidar_error_stops(self):
        result, calls, _ = run(
            [RELEASED] + [UP] * 5, error=RuntimeError("serial unplugged")
        )
        self.assertEqual(result["stop"], "lidar error")
        self.assertEqual(moves(calls), [])
        self.assertEqual(calls[-1], "stop")

    def test_a_turn_beside_a_wall_is_stopped_and_an_open_room_allows_it(self):
        right = ({3: 32767}, HELD)
        _, calls, log = run([RELEASED] + [right] * 10, scan=SIDE_WALL)
        self.assertEqual(moves(calls), [])  # the guard must see the turn, not only the forward speed
        self.assertIn("blocked", " ".join(log))
        _, calls, _ = run([RELEASED] + [right] * 3)
        self.assertTrue(moves(calls))

    def test_a_stick_resting_off_centre_never_arms(self):
        rest, held = ({3: -32767}, {}), ({3: -32767}, HELD)  # an axis that rests at the end of its travel
        _, calls, _ = run([rest] * 2 + [held] * 10)
        self.assertEqual(moves(calls), [])  # a wrong axis number must not turn pressing R1 into a full spin

    def test_a_held_stick_keeps_driving_without_new_events(self):
        _, calls, _ = run(
            [RELEASED] * 2 + [UP]
        )  # one event, then nothing new for the rest of the 2 s
        self.assertGreater(len(moves(calls)), 20)


class TermPad(FakePad):
    """The console's STOP: SIGTERM arrives while the loop reads the controller."""

    def poll(self):
        signal.raise_signal(signal.SIGTERM)


class FakeOpens:
    """Stands in for Pad, LiDAR and Wheels in main(): records what main opened and in what order."""

    def __init__(self, test, script, pad_class=FakePad):
        self.opened, self.pad, self.session, self.wheels = (
            [],
            pad_class(script),
            FakeSession(OPEN_ROOM),
            FakeWheels(),
        )
        test.saved = (
            joystick_drive.Pad,
            joystick_drive.lidar.LidarSession,
            joystick_drive.wheels.Wheels,
            joystick_drive.base_check.first_full_scan,
            sys.argv,
            os.environ.get(wheels.SAFE_RUN_ENV),
        )
        joystick_drive.Pad = lambda path: self.opened.append("pad") or self.pad
        joystick_drive.lidar.LidarSession = lambda: (
            self.opened.append("lidar") or self.session
        )
        joystick_drive.wheels.Wheels = lambda p: (
            self.opened.append("wheels") or self.wheels
        )
        joystick_drive.base_check.first_full_scan = lambda *args: OPEN_ROOM


class MainTest(unittest.TestCase):
    def tearDown(self):
        (
            joystick_drive.Pad,
            joystick_drive.lidar.LidarSession,
            joystick_drive.wheels.Wheels,
            joystick_drive.base_check.first_full_scan,
            sys.argv,
            env,
        ) = self.saved
        if env is None:
            os.environ.pop(wheels.SAFE_RUN_ENV, None)
        else:
            os.environ[wheels.SAFE_RUN_ENV] = env

    def main(self, argv, script=(), safe_run=True, pad_class=FakePad):
        self.fakes = FakeOpens(self, script, pad_class)
        if safe_run:
            os.environ[wheels.SAFE_RUN_ENV] = "1"
        else:
            os.environ.pop(wheels.SAFE_RUN_ENV, None)
        sys.argv = ["joystick_drive.py"] + argv
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            try:
                joystick_drive.main()
            except SystemExit as stopped:
                return str(stopped.code), out.getvalue()
        return None, out.getvalue()

    def test_refusals_come_before_any_device_opens(self):
        flag = "--yes-owner-at-robot"
        for argv, safe_run in (
            ([], True),
            ([flag], False),
            ([flag, "--seconds", "601"], True),
            ([flag, "--seconds", "0"], True),
            ([flag, "--max-speed", "0.2"], True),
            ([flag, "--max-speed", "0"], True),
        ):
            code, _ = self.main(argv, safe_run=safe_run)
            self.assertTrue(code and code.startswith("REFUSED"), (argv, safe_run, code))
            self.assertEqual(self.fakes.opened, [], (argv, safe_run))
            self.tearDown()

    def test_dry_run_opens_only_the_pad_and_prints_what_it_would_send(self):
        code, out = self.main(
            ["--dry", "--seconds", "0.2"], script=[UP], safe_run=False
        )
        self.assertIsNone(code)
        self.assertEqual(self.fakes.opened, ["pad"])
        self.assertIn("forward", out)
        self.assertTrue(self.fakes.pad.closed)

    def test_a_drive_opens_pad_lidar_wheels_and_closes_all_of_them(self):
        code, out = self.main(["--yes-owner-at-robot"], script=[RELEASED, UP, "gone"])
        self.assertIsNone(code)
        self.assertEqual(self.fakes.opened, ["pad", "lidar", "wheels"])
        self.assertEqual(
            json.loads(out.strip().splitlines()[-1])["stop"], "controller gone"
        )
        self.assertEqual(self.fakes.wheels.calls[-2:], ["stop", "close"])
        self.assertTrue(self.fakes.session.closed and self.fakes.pad.closed)

    def test_sigterm_from_the_consoles_stop_ends_the_drive_cleanly(self):
        self.addCleanup(signal.signal, signal.SIGTERM, signal.getsignal(signal.SIGTERM))
        code, out = self.main(["--yes-owner-at-robot"], pad_class=TermPad)
        self.assertIsNone(code)
        self.assertEqual(json.loads(out.strip().splitlines()[-1])["stop"], "terminated")
        self.assertEqual(self.fakes.wheels.calls[-2:], ["stop", "close"])
        self.assertTrue(self.fakes.session.closed and self.fakes.pad.closed)


if __name__ == "__main__":
    unittest.main()
