"""Drive the base by hand with the DualShock 4 plugged into the Pi (/dev/input/js0).

Started by the console as the check "joystick_drive" (python tools/safe_run.py ... tools/joystick_drive.py): the page's
button (robotctl cannot send the presence flag --yes-owner-at-robot). Hold the deadman (R1) to move; left stick up is forward, down is backward,
right stick sideways turns. The LiDAR guards every move. See base/docs/superpowers/specs/2026-11-20-joystick-drive-design.md.

  python tools/safe_run.py .venv/bin/python tools/joystick_drive.py --yes-owner-at-robot [--max-speed 0.08] [--seconds 300]
  python tools/joystick_drive.py --dry      # reads the controller and prints what it would send; opens no wheel or LiDAR

Nothing moves until the deadman has been seen released with both sticks centred (a button held at start-up, or an axis
that rests off centre, is not a go). SIGTERM from the console's STOP ends it cleanly. It stops on: the
deadman released, a stick in its dead zone, no or stale LiDAR points, a blocked front (safety.must_stop) or rear
(safety.rear_clear), the controller going away, the time limit. Backward is straight, at REAR_SPEED, and the LiDAR does
not see the power cable: the person driving watches it. Prints one JSON line at the end.
"""

import argparse
import json
import math
import os
import select
import signal
import struct
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import base_check  # noqa: E402
import lidar  # noqa: E402
import params  # noqa: E402
import run_follow  # noqa: E402
import safety  # noqa: E402
import wheels  # noqa: E402

# js numbering of the DS4 under the kernel's hid-playstation (13 buttons, 8 axes). Confirm with --dry before a drive.
DEADMAN = 5  # R1
AXIS_FWD = 1  # left stick Y, up is negative
AXIS_TURN = 3  # right stick X, right is positive
AXIS_FULL = 32767
DEAD_ZONE = 0.15  # of the stick's travel
REAR_SPEED = 0.04  # m/s backward, as tools/back_up.py (wheels.REVERSE_MAX is 0.06)
REAR_LOOK = 0.30  # m of backing the LiDAR must show clear (as back_up.MAX_METRES), on top of the run-on
PERIOD = 0.05  # s between two commands; the wheels' watchdog stops the base after 0.3 s of silence
SECONDS, SECONDS_MAX = 300.0, 600.0
DEVICE = "/dev/input/js0"
EVENT = struct.Struct("IhBB")  # struct js_event: time ms, value, type, number
INIT = 0x80  # the type flag of the start-up events that report the state at open


def parse_event(data):
    """(type, number, value) of one 8-byte js_event, the init flag removed: type 1 is a button, 2 an axis."""
    _, value, kind, number = EVENT.unpack(data)
    return kind & ~INIT, number, value


def _stick(axes, number):
    """The stick's position in -1..1 with the dead zone cut out (its edge is 0)."""
    x = axes.get(number, 0) / AXIS_FULL
    beyond = (abs(x) - DEAD_ZONE) / (1 - DEAD_ZONE)
    return math.copysign(min(1.0, beyond), x) if beyond > 0 else 0.0


def command(axes, buttons, p, max_speed):
    """(kind, v, w) for the controller's state: kind "stop", "forward" (also a turn on the spot) or "reverse".
    axes and buttons map a js number to its value; a missing one counts as 0."""
    if not buttons.get(DEADMAN):
        return "stop", 0.0, 0.0
    ahead = -_stick(axes, AXIS_FWD)  # up is negative on the stick
    turn = -_stick(axes, AXIS_TURN)  # right is a clockwise, negative, turn
    if ahead < 0:
        return "reverse", REAR_SPEED, 0.0
    v = min(max_speed, p["max_speed"]) * ahead
    w = turn * p["max_turn_rate"]
    if v == 0 and w == 0:
        return "stop", 0.0, 0.0
    v, w = wheels.least(v, w)
    v, w = wheels.clamp_curvature(v, w, p)
    return "forward", v, w


class Pad:
    """The controller's js device: the state of every button and axis, kept up to date by poll()."""

    def __init__(self, path=DEVICE):
        self.fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
        self.axes, self.buttons = {}, {}

    def poll(self):
        """Apply every event waiting; never blocks. OSError when the controller is gone (unplugged or closed)."""
        while select.select([self.fd], [], [], 0)[0]:
            data = os.read(self.fd, EVENT.size * 32)
            if not data:
                raise OSError("the controller closed the device")
            for i in range(0, len(data) - EVENT.size + 1, EVENT.size):
                kind, number, value = parse_event(data[i : i + EVENT.size])
                if kind == 1:
                    self.buttons[number] = value
                elif kind == 2:
                    self.axes[number] = value

    def close(self):
        os.close(self.fd)


def _terminate(signum, frame):
    raise KeyboardInterrupt("signal %d" % signum)


def blocked(kind, v, w, points, p):
    """Why this move is not safe now, else None. No points is never "nothing there": safety.must_stop says
    False for an empty scan, so a dead or stale LiDAR must stop the base here."""
    if not len(points):
        return "no LiDAR points"
    if kind == "reverse":
        return (
            None
            if safety.rear_clear(points, REAR_SPEED, p, REAR_LOOK)
            else "rear blocked"
        )
    return "front blocked" if safety.must_stop(points, v, w, p) else None


def drive(pad, wheels_, session, p, max_speed, seconds, clock, sleep, log):
    """Drive from the controller for `seconds`; S# however it ends. The result: {"stop": why, "seconds": s}."""
    start, armed, last, shown = clock(), False, "stop", None
    result = {"stop": "time"}
    try:
        while clock() - start < seconds:
            try:
                pad.poll()
            except OSError:
                result["stop"] = "controller gone"
                break
            # Armed once the deadman is seen released with the sticks centred: a button held since start-up, or an axis
            # number that is really a trigger resting off centre, never moves the base.
            if (
                not pad.buttons.get(DEADMAN)
                and command(pad.axes, {DEADMAN: 1}, p, max_speed)[0] == "stop"
            ):
                armed = True
            kind, v, w = (
                command(pad.axes, pad.buttons, p, max_speed)
                if armed
                else ("stop", 0.0, 0.0)
            )
            reason = None
            if kind != "stop":
                try:
                    points = run_follow.scan_points(session.latest(), p)
                except Exception:
                    result["stop"] = "lidar error"
                    break
                reason = blocked(kind, v, w, points, p)
                if reason:
                    kind = "stop"
            if kind == "forward":
                wheels_.send(v, w)
            elif kind == "reverse":
                wheels_.send_reverse(REAR_SPEED, 0.0)
            elif last != "stop":
                wheels_.stop()
            if (kind, reason) != shown:
                log(
                    reason
                    or (
                        "%s v=%.2f w=%+.2f" % (kind, v, w) if kind != "stop" else "stop"
                    )
                )
                shown = (kind, reason)
            last = kind
            sleep(PERIOD)
    finally:
        wheels_.stop()
    result["seconds"] = round(clock() - start, 2)
    return result


def dry(pad, p, max_speed, seconds, clock, sleep, log):
    """Print the buttons held, the sticks (to the nearest eighth of their travel) and what drive() would send."""
    start, shown = clock(), None
    while clock() - start < seconds:
        try:
            pad.poll()
        except OSError:
            return {"stop": "controller gone"}
        held = sorted(n for n, value in pad.buttons.items() if value)
        sticks = {
            n: round(value / AXIS_FULL * 8) / 8
            for n, value in sorted(pad.axes.items())
            if abs(value) > 0.1 * AXIS_FULL
        }
        asked = command(pad.axes, pad.buttons, p, max_speed)
        if (held, sticks, asked[0]) != shown:
            log("buttons %s axes %s -> %s v=%.2f w=%+.2f" % (held, sticks, *asked))
            shown = (held, sticks, asked[0])
        sleep(PERIOD)
    return {"stop": "time"}


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument(
        "--max-speed",
        type=float,
        default=0.08,
        help="top forward speed, m/s, at most params.json max_speed",
    )
    ap.add_argument(
        "--seconds",
        type=float,
        default=SECONDS,
        help="how long to run, at most %g" % SECONDS_MAX,
    )
    ap.add_argument("--device", default=DEVICE)
    ap.add_argument(
        "--dry",
        action="store_true",
        help="print what would be sent; open no wheel and no LiDAR",
    )
    ap.add_argument(
        "--yes-owner-at-robot",
        action="store_true",
        help="a person is at the robot and watches the power cable",
    )
    args = ap.parse_args()
    p = params.load()
    if not 0 < args.seconds <= SECONDS_MAX:
        sys.exit("REFUSED: --seconds must be in (0, %g]" % SECONDS_MAX)
    if not 0 < args.max_speed <= p["max_speed"]:
        sys.exit(
            "REFUSED: --max-speed must be in (0, %g] (params.json max_speed)"
            % p["max_speed"]
        )
    if not args.dry:
        if not args.yes_owner_at_robot:
            sys.exit("REFUSED: pass --yes-owner-at-robot (this tool drives the base)")
        if not wheels.under_safe_run():
            sys.exit("REFUSED: " + wheels.NOT_SAFE_RUN)
    pad = Pad(args.device)
    session = wheels_ = None
    signal.signal(signal.SIGTERM, _terminate)  # the console's STOP: end through the finally blocks, like Ctrl+C
    try:
        try:
            if args.dry:
                result = dry(
                    pad, p, args.max_speed, args.seconds, time.monotonic, time.sleep, print
                )
            else:
                session = lidar.LidarSession()
                base_check.first_full_scan(
                    session, {"scans": []}, time.monotonic, time.sleep
                )  # the first revolution is partial
                wheels_ = wheels.Wheels(p)
                result = drive(
                    pad,
                    wheels_,
                    session,
                    p,
                    args.max_speed,
                    args.seconds,
                    time.monotonic,
                    time.sleep,
                    print,
                )
        except KeyboardInterrupt:
            result = {"stop": "terminated"}
    finally:
        if wheels_ is not None:
            wheels_.stop()
            wheels_.close()
        if session is not None:
            session.close()
        pad.close()
    print(json.dumps(result), flush=True)


if __name__ == "__main__":
    main()
