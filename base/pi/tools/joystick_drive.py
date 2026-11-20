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

Heartbeat: a js device sends an event only when a value changes, so a lost Bluetooth link could leave R1 reading
held. The tool therefore also reads the motion-sensor device of the same controller (found by itself in
/proc/bus/input/devices; about 170 reports a second, also at rest, measured on USB on 2026-11-20). No report for
HEARTBEAT_S stops the base and it must be armed again (R1 seen released). A Bluetooth controller without that
stream is refused; --device skips the search and the heartbeat (a repair option, not allowed through the console).
"""

import argparse
import collections
import json
import math
import os
import re
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
REAR_LOOK = 0.30  # m of backing the LiDAR must show clear (as back_up.MAX_METRES), on top of the run-on and the
# margin: backing is refused with a wall within about 0.40 m behind the back edge, even when the stick asks for a little
PERIOD = 0.05  # s between two commands; the wheels' watchdog stops the base after 0.3 s of silence
SECONDS, SECONDS_MAX = 300.0, 600.0
DEVICE = "/dev/input/js0"  # when no controller is found in /proc/bus/input/devices
HEARTBEAT_S = 0.25  # s without a report from the controller: the link is lost (the wheels' watchdog is 0.3 s)
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


Controller = collections.namedtuple("Controller", "js sensor bus")
BUSES = {"0003": "usb", "0005": "bluetooth"}


def devices_text(path="/proc/bus/input/devices"):
    """The kernel's list of input devices, or "" when there is none (not Linux)."""
    try:
        return Path(path).read_text()
    except OSError:
        return ""


def locate(text):
    """The DualShock 4 in the text of /proc/bus/input/devices: Controller(js device, event device of the motion
    sensors of the same controller or None, bus "usb" or "bluetooth"); None when there is no such controller."""
    found = []
    for block in text.split("\n\n"):
        name = re.search(r'^N: Name="(.*)"', block, re.M)
        if not name or "Wireless Controller" not in name.group(1):
            continue
        uniq = re.search(r"^U: Uniq=(.*)$", block, re.M)
        handlers = re.search(r"^H: Handlers=(.*)$", block, re.M)
        bus = re.search(r"^I: Bus=(\w+)", block, re.M)
        found.append((name.group(1), uniq.group(1).strip() if uniq else "", handlers.group(1).split() if handlers else [], bus.group(1) if bus else ""))
    for _, uniq, handlers, bus in found:
        js = next((h for h in handlers if re.fullmatch(r"js\d+", h)), None)
        if js is None:
            continue
        sensor = next((h for n, u, hs, _ in found if n.endswith("Motion Sensors") and u == uniq for h in hs if re.fullmatch(r"event\d+", h)), None)
        return Controller("/dev/input/" + js, "/dev/input/" + sensor if sensor else None, BUSES.get(bus, bus))
    return None


def _open(path):
    return os.open(path, os.O_RDONLY | os.O_NONBLOCK)


class Pad:
    """The controller: the buttons and axes of its js device, kept up to date by poll(), and, when `sensor` is given,
    the event device of its motion sensors, read only to know the link is alive (silent_for). opener, ready and read
    are the OS calls, replaceable in tests."""

    def __init__(self, path=DEVICE, sensor=None, clock=time.monotonic, opener=_open, ready=select.select, read=os.read):
        self.fd = opener(path)
        self.sensor_fd = opener(sensor) if sensor else None
        self.axes, self.buttons = {}, {}
        self.clock, self.last_seen, self._ready, self._read = clock, clock(), ready, read

    def poll(self):
        """Apply every event waiting; never blocks. OSError when the controller is gone (unplugged or closed)."""
        fds = [self.fd] + ([self.sensor_fd] if self.sensor_fd is not None else [])
        while True:
            waiting = self._ready(fds, [], [], 0)[0]
            if not waiting:
                return
            for fd in waiting:
                data = self._read(fd, EVENT.size * 32 if fd == self.fd else 24 * 256)
                if not data:
                    raise OSError("the controller closed the device")
                self.last_seen = self.clock()
                if fd != self.fd:
                    continue  # a sensor report only proves the link is alive
                for i in range(0, len(data) - EVENT.size + 1, EVENT.size):
                    kind, number, value = parse_event(data[i : i + EVENT.size])
                    if kind == 1:
                        self.buttons[number] = value
                    elif kind == 2:
                        self.axes[number] = value

    def silent_for(self):
        """Seconds since the last report from the controller; 0.0 when there is no sensor stream to tell."""
        return self.clock() - self.last_seen if self.sensor_fd is not None else 0.0

    def close(self):
        os.close(self.fd)
        if self.sensor_fd is not None:
            os.close(self.sensor_fd)


def _terminate(signum, frame):
    raise KeyboardInterrupt("signal %d" % signum)


def released(pad, p, max_speed):
    """True when the deadman is up and both sticks are centred: the only state that arms the tool. A button held
    since start-up, or an axis number that is really a trigger resting off centre, never moves the base."""
    return (
        not pad.buttons.get(DEADMAN)
        and command(pad.axes, {DEADMAN: 1}, p, max_speed)[0] == "stop"
    )


def blocked(kind, v, w, points, p, coast=None, now=0.0):
    """Why this move is not safe now, else None. No points is never "nothing there": safety.must_stop says
    False for an empty scan, so a dead or stale LiDAR must stop the base here. coast (safety.Coast, with the time
    `now`): what the base may still be doing after the commands it got, as run_follow checks it, so a turn right
    after forward driving is also checked with the forward roll that is not over yet."""
    if not len(points):
        return "no LiDAR points"
    if kind == "reverse":
        return (
            None
            if safety.rear_clear(points, REAR_SPEED, p, REAR_LOOK)
            else "rear blocked"
        )
    motions = coast.motions(v, w, now) if coast else [(v, w)]
    return (
        "front blocked"
        if any(safety.must_stop(points, v2, w2, p) for v2, w2 in motions)
        else None
    )


def drive(pad, wheels_, session, p, max_speed, seconds, clock, sleep, log):
    """Drive from the controller for `seconds`; S# however it ends. The result: {"stop": why, "seconds": s}."""
    start, armed, last, shown = clock(), False, "stop", None
    coast = safety.Coast(p)
    result = {"stop": "time"}
    try:
        while clock() - start < seconds:
            try:
                pad.poll()
            except OSError:
                result["stop"] = "controller gone"
                break
            quiet = pad.silent_for() > HEARTBEAT_S
            if quiet:
                armed = False  # the link may have dropped with R1 held: nothing moves until R1 is seen released again
            elif released(pad, p, max_speed):
                armed = True
            kind, v, w = (
                command(pad.axes, pad.buttons, p, max_speed)
                if armed
                else ("stop", 0.0, 0.0)
            )
            reason = "controller silent" if quiet else None
            if kind != "stop":
                try:
                    points = run_follow.scan_points(session.latest(), p)
                except Exception:
                    result["stop"] = "lidar error"
                    break
                reason = blocked(kind, v, w, points, p, coast, clock() - start)
                if reason:
                    kind = "stop"
            if kind == "forward":
                wheels_.send(v, w)
                coast.sent(v, w, clock() - start)
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
    start, shown, armed = clock(), None, False
    while clock() - start < seconds:
        try:
            pad.poll()
        except OSError:
            return {"stop": "controller gone"}
        quiet = pad.silent_for() > HEARTBEAT_S
        armed = False if quiet else armed or released(pad, p, max_speed)  # as drive()
        held = sorted(n for n, value in pad.buttons.items() if value)
        sticks = {
            n: round(value / AXIS_FULL * 8) / 8
            for n, value in sorted(pad.axes.items())
            if abs(value) > 0.1 * AXIS_FULL
        }
        asked = (
            command(pad.axes, pad.buttons, p, max_speed) if armed else ("stop", 0.0, 0.0)
        )
        if (held, sticks, asked[0], quiet) != shown:
            log("buttons %s axes %s -> %s v=%.2f w=%+.2f%s" % (held, sticks, *asked, " (controller silent)" if quiet else ""))
            shown = (held, sticks, asked[0], quiet)
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
    ap.add_argument("--device", help="the js device; skips the search for the controller and the heartbeat")
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
    if args.device:
        js, sensor, bus = args.device, None, None
    else:
        js, sensor, bus = locate(devices_text()) or (DEVICE, None, None)
    if bus == "bluetooth" and not sensor:
        sys.exit("REFUSED: the Bluetooth controller shows no motion-sensor stream, so a lost link could not be noticed")
    print("controller %s (%s), heartbeat from %s" % (js, bus or "bus unknown", sensor or "nothing"), flush=True)
    try:
        pad = Pad(js, sensor)
    except OSError as error:
        sys.exit("REFUSED: cannot open the controller %s (%s): plug it in, or pair and connect it" % (js, error.strerror or error))
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
