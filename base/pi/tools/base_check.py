"""Speed ladder: drive straight or turn on the spot, stop with S#, and measure it with the LiDAR.

  python tools/base_check.py --line 0.15 --metres 1.5 --yes-owner-at-robot
  python tools/base_check.py --line 0.3 --metres 1.5 --max-speed 0.3 --linear-scale 1.0 --yes-owner-at-robot
  python tools/base_check.py --turn 0.26 --degrees 180 --yes-owner-at-robot

The base moves. Only run it with the owner at the robot and 3 m clear in front.
  python tools/base_check.py --turn 0.8 --degrees 180 --max-turn-rate 0.8 --yes-owner-at-robot
--max-speed, --max-turn-rate and --linear-scale change a copy of params.json for this run only.
Odometry (20 Hz), every scan and the report go to logs/base-check/ next to tools/.
--velocity-loop (with --line): the command comes from the velocity loop, run as in run_follow, so
the run measures the drift the loop leaves (accurate-motion Task 9).
"""

import argparse
import contextlib
import json
import math
import signal
import statistics
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))  # lidar.py, params.py, ...
import calib  # noqa: E402
import geometry  # noqa: E402
import lidar  # noqa: E402
import motion  # noqa: E402
import params  # noqa: E402
import safety  # noqa: E402
import scanmatch  # noqa: E402
import velocity  # noqa: E402
import wheels as wheels_module  # noqa: E402

DEFAULT_OUT = Path(__file__).resolve().parent.parent / "logs" / "base-check"
PERIOD = 0.05  # s, 20 Hz: the wheel watchdog stops the base after 0.3 s without send()
HALF_STRIP = 0.25  # m, half the width of the swept strip ahead
LINE_MARGIN = 0.6  # m, free space wanted beyond the planned distance
TURN_CLEAR = 0.45  # m, free radius wanted around the centre before turning
EMERGENCY = 0.35  # m, the least stop distance (ahead, or around for turns); it grows with speed
LIDAR_SILENCE = 0.3  # s without a new scan while driving means stop
SCAN_PERIOD = 0.1  # s, LDS-03 revolution (median of the first real run)
ASSUMED_DECEL = 0.5  # m/s^2, a cautious guess until this ladder measures decel
SEND_GAP = 0.15  # s between two send() calls; longer and the watchdog may have stopped the base
LINE_OVERRUN = 0.3  # m, a line may run this far past the target before the time limit stops it
TURN_OVERRUN = math.radians(30)  # the same for a turn
MAX_SPEED_LIMIT = 0.6  # m/s, the highest rung of the ladder
MAX_TURN_LIMIT = 1.2  # rad/s, the highest turn rung: each wheel then runs at about 0.19 m/s
SCALE_RANGE = (0.5, 2.0)  # allowed --linear-scale
SETTLE = 2.0  # s to wait after S# before the final scan
TOP_RATIO = 1.3  # --velocity-loop may command up to this times the asked speed (a slow base needs more)


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def ahead(scan, p):
    """Distances in front of the body (x - FRONT) of the usable points inside the strip."""
    pts = scanmatch.points(scan, p)
    inside = (abs(pts[:, 1]) < HALF_STRIP) & (pts[:, 0] - safety.FRONT > 0)
    return pts[inside, 0] - safety.FRONT


def around(scan, p):
    """Distances of the usable points from the turning centre."""
    pts = scanmatch.points(scan, p)
    return (pts ** 2).sum(axis=1) ** 0.5


def clear_to_drive(scan, p, mode, metres):
    """(ok, nearest m): line needs nothing in the strip up to metres + 0.6, turn nothing within 0.45 m."""
    if not lidar.usable(scan):
        return False, None
    near = ahead(scan, p) if mode == "line" else around(scan, p)
    nearest = float(near.min()) if len(near) else None
    need = metres + LINE_MARGIN if mode == "line" else TURN_CLEAR
    return nearest is None or nearest >= need, nearest


def emergency_distance(v):
    """Scan age plus a silent LiDAR, then braking at ASSUMED_DECEL, plus 0.15 m (as safety.py)."""
    return max(EMERGENCY, v * (SCAN_PERIOD + LIDAR_SILENCE) + v * v / (2 * ASSUMED_DECEL) + 0.15)


def emergency(scan, p, mode, v=0.0):
    near = ahead(scan, p) if mode == "line" else around(scan, p)
    return bool(len(near)) and float(near.min()) < emergency_distance(v)


# Ctrl+C, kill, and SIGHUP from a dropped ssh session (SIGHUP does not exist on Windows).
STOP_SIGNALS = [s for s in (signal.SIGINT, signal.SIGTERM, getattr(signal, "SIGHUP", None)) if s is not None]


@contextlib.contextmanager
def signals_ignored():
    """A second signal must not interrupt the stop itself."""
    old = [(s, signal.signal(s, signal.SIG_IGN)) for s in STOP_SIGNALS]
    try:
        yield
    finally:
        for s, handler in old:
            signal.signal(s, handler)


def log_odom(wheels, log, clock):
    x, y, yaw = wheels.read_odom()
    log["odom"].append([clock(), x, y, yaw])
    return x, y, yaw


def log_scan(session, log):
    """The scan if a new revolution came in since the last logged one, else None."""
    scan = session.latest()
    stamp = session.stream.updated_at
    if not scan or (log["scans"] and log["scans"][-1][0] == stamp):
        return None
    log["scans"].append([stamp, scan])
    return scan


def wait_scan(session, log, clock, sleep, timeout=3.0):
    deadline = clock() + timeout
    while clock() < deadline:
        scan = log_scan(session, log)
        if scan:
            return scan
        sleep(0.02)
    raise RuntimeError("no LiDAR scan in %.0f s" % timeout)


def first_full_scan(session, log, clock, sleep):
    """The first revolution after START begins at an arbitrary angle, so it is dropped."""
    wait_scan(session, log, clock, sleep)
    scan = wait_scan(session, log, clock, sleep)
    log["scans"].pop(0)
    return scan


class Loop:
    """--velocity-loop for a line: the run_follow chain (motion.MotionEstimator, calib.BaseEstimates,
    velocity.VelocityLoop) turns the asked true speed into the command, from the LiDAR-checked motion.

    top caps the command at TOP_RATIO times the asked speed. With the odometry dead the run's time
    limit then covers up to top * limit, so the strip must be clear that far plus the margin left
    for braking: clear_metres() is what clear_to_drive is given.
    """

    def __init__(self, p, rate, target):
        self.rate = rate
        self.top = rate * TOP_RATIO
        self.motion = motion.MotionEstimator(p)
        self.estimates = calib.BaseEstimates(p)
        self.velocity = velocity.VelocityLoop(p, self.estimates)

    def command(self, t, odom, scan, sent, sent_at):
        measured = self.motion.update(t, odom, scan, sent)
        self.estimates.observe(measured, sent, sent_at)
        return self.velocity.command(self.rate, 0.0, measured, self.top)

    @staticmethod
    def clear_metres(target):
        """The metres clear_to_drive must find free (it adds LINE_MARGIN) for a loop run."""
        return (target + LINE_OVERRUN) * TOP_RATIO - LINE_OVERRUN

    def summary(self):
        names = ("speed_gain", "turn_gain", "drift_per_m", "latency")
        return {n: [getattr(self.estimates, n)[0], getattr(self.estimates, n)[2]] for n in names}


def drive(wheels, session, p, mode, rate, target, log, clock=time.monotonic, sleep=time.sleep, loop=None):
    """send() at 20 Hz until odometry covers target (m, or rad for a turn). S# however it ends.

    loop: a Loop (line only) that sets the command from the measured motion each time a new scan
    has come in (between scans the last command holds: without a new scan nothing is measured).
    """
    v, w = (rate, 0.0) if mode == "line" else (0.0, rate)
    # Even with odometry reading zero, the asked speed for this long stays inside the checked-clear length.
    limit = (target + (LINE_OVERRUN if mode == "line" else TURN_OVERRUN)) / abs(rate)
    log["sent"] = []
    try:
        start = odom = log_odom(wheels, log, clock)
        log["drive_start"] = last_scan = clock()
        turned, last_send = 0.0, None
        newest = log["scans"][-1][1] if log["scans"] else None  # the newest revolution, for the loop
        while True:
            tick = clock()
            if last_send is not None and tick - last_send > SEND_GAP:
                log["stop_reason"] = "send gap"  # a slow odometry read or loop: the base may have stopped and restarted
                break
            wheels.send(v, w)
            last_send = clock()
            log["sent"].append([last_send, v, w])
            previous, odom = odom, log_odom(wheels, log, clock)
            turned += wrap(odom[2] - previous[2])
            scan = log_scan(session, log)
            if scan:
                last_scan = clock()
                newest = scan
            done = math.hypot(odom[0] - start[0], odom[1] - start[1]) if mode == "line" else abs(turned)
            if scan and emergency(scan, p, mode, v):
                log["stop_reason"] = "emergency"
            elif done >= target:
                log["stop_reason"] = "target"
            elif clock() - last_scan > LIDAR_SILENCE:
                log["stop_reason"] = "lidar silent"
            elif clock() - log["drive_start"] > limit:
                log["stop_reason"] = "timeout"
            if "stop_reason" in log:
                break
            if loop is not None and scan:  # at once, inside this period: its time comes off the sleep
                v, w = loop.command(clock(), odom, newest, (v, w), last_send)
            sleep(max(0.0, PERIOD - (clock() - tick)))
    finally:  # any exit, Ctrl+C and SIGTERM included: S# before anything else
        with signals_ignored():
            wheels.stop()
        log["stop_time"] = clock()
        error = sys.exc_info()[1]
        if error is not None:
            log["stop_reason"], log["error"] = "error", repr(error)


def settle(wheels, session, log, clock=time.monotonic, sleep=time.sleep):
    """Keep logging for SETTLE s after S# (repeating S#), then take the final scan."""
    while clock() - log["stop_time"] < SETTLE:
        tick = clock()
        wheels.stop()
        log_odom(wheels, log, clock)
        log_scan(session, log)
        sleep(max(0.0, PERIOD - (clock() - tick)))
    wait_scan(session, log, clock, sleep)


# ---- the report, from the log alone ----


def pose_times(stamps):
    """A revolution is stamped when it ends, so the base was there half a period earlier."""
    half = statistics.median(b - a for a, b in zip(stamps, stamps[1:])) / 2 if len(stamps) > 2 else 0.0
    return [t - half for t in stamps]


def chain(steps, i, j):
    """The motion from scan i to scan j as the sum of the successive steps."""
    pose = (0.0, 0.0, 0.0)
    for step in steps[i:j]:
        x, y = geometry.base_to_odom(step[:2], pose)
        pose = (x, y, pose[2] + step[2])
    return pose


def shift(scans, steps, i, j, p):
    """Scan i to scan j matched directly, starting from the chained steps (a turn is too big for ICP alone)."""
    return scanmatch.match(scans[i], scans[j], p, guess=chain(steps, i, j))


def report(log, p):
    """The numbers of one run. Distances in m, angles in rad, times in s (clock of the log)."""
    line = log["mode"] == "line"
    sign = 1.0 if log["rate"] >= 0 else -1.0
    stop = log["stop_time"]
    # Partial revolutions (the first one after START) make a poor reference: leave them out.
    full = statistics.median(len(s[1]) for s in log["scans"]) * 0.8
    kept = [s for s in log["scans"] if len(s[1]) >= full]
    scans = [s[1] for s in kept]
    times = pose_times([s[0] for s in kept])
    steps = [scanmatch.scan_shift(a, b, p) for a, b in zip(scans, scans[1:])]

    def progress(motion):  # forward metres for a line, turned radians (in the asked direction) for a turn
        return motion[0] if line else sign * motion[2]

    speeds = [[(t0 + t1) / 2, progress(s) / (t1 - t0)] for t0, t1, s in zip(times, times[1:], steps)]

    chained = chain(steps, 0, len(scans) - 1)
    total = shift(scans, steps, 0, len(scans) - 1, p)
    middle = (log["drive_start"] + stop) / 2
    # The scans of the middle second, matched directly: small steps add up their small errors.
    window = [k for k, t in enumerate(times) if middle - 0.5 <= t <= middle + 0.5]
    middle_speed = None
    if len(window) >= 2:
        i, j = window[0], window[-1]
        middle_speed = progress(shift(scans, steps, i, j, p)) / (times[j] - times[i])

    # The last scan before S# was taken a little before it; take off what was driven in between.
    k = max(i for i, t in enumerate(times) if t <= stop)
    after_stop = progress(shift(scans, steps, k, len(scans) - 1, p)) - (middle_speed or 0.0) * (stop - times[k])

    # stop_delay: from S# to the first of two scan steps in a row clearly slower than the middle
    # second (one slow step can be scan-matching noise).
    stop_delay = None
    if middle_speed:
        after = [(t, speed < 0.9 * middle_speed) for t, speed in speeds if t >= stop]
        drops = [a[0] for a, b in zip(after, after[1:]) if a[1] and b[1]]
        stop_delay = drops[0] - stop if drops else None

    odom = log["odom"]
    at_stop = max((o for o in odom if o[0] <= stop), key=lambda o: o[0])
    odom_yaw = sum(wrap(b[3] - a[3]) for a, b in zip(odom, odom[1:]))
    result = {
        "mode": log["mode"], "rate": log["rate"], "stop_reason": log.get("stop_reason"),
        "scan_dx": total[0], "scan_dy": total[1], "scan_yaw": total[2], "scan_agree": total[3],
        "chain_dx": chained[0], "chain_dy": chained[1], "chain_yaw": chained[2],
        "partial_scans": len(log["scans"]) - len(kept),
        "odom_yaw": odom_yaw,
        "middle_speed": middle_speed, "after_stop": after_stop, "stop_delay": stop_delay,
        "scan_period": 2 * (log["scans"][-1][0] - times[-1]),
        "speeds": speeds,
    }
    if line:
        result.update({
            "real_distance": math.hypot(total[0], total[1]),
            "odom_distance": math.hypot(odom[-1][1] - odom[0][1], odom[-1][2] - odom[0][2]),
            "odom_distance_at_stop": math.hypot(at_stop[1] - odom[0][1], at_stop[2] - odom[0][2]),
            "drift_per_m": total[1] / total[0] if total[0] else None,
            # heading change per metre: a fixed LiDAR mount angle cancels between two scans
            "yaw_per_m": total[2] / math.hypot(total[0], total[1]) if total[0] else None,
            "asked_over_real": log["rate"] / middle_speed if middle_speed else None,
        })
    return result


def table(r):
    """A few lines for the terminal."""
    def f(value, fmt):
        return "n/a" if value is None else fmt % value

    rows = ["stop reason              %s" % r["stop_reason"]]
    if r["mode"] == "line":
        rows += [
            "real distance (scan)     %s" % f(r["real_distance"], "%.3f m"),
            "odometry distance        %s (at S# %s)" % (f(r["odom_distance"], "%.3f m"), f(r["odom_distance_at_stop"], "%.3f m")),
            "real speed, middle 1 s   %s (asked %.2f, asked/real %s)" % (f(r["middle_speed"], "%.3f m/s"), r["rate"], f(r["asked_over_real"], "%.3f")),
            "distance after S#        %s" % f(r["after_stop"], "%.3f m"),
            "sideways drift           %s" % f(None if r["drift_per_m"] is None else 100 * r["drift_per_m"], "%+.1f cm per m (+ = left)"),
            "yaw change scan / odom   %.1f / %.1f deg" % (math.degrees(r["scan_yaw"]), math.degrees(r["odom_yaw"])),
            "heading drift            %s" % f(None if r.get("yaw_per_m") is None else math.degrees(r["yaw_per_m"]), "%+.2f deg per m (+ = left)"),
            "note: drift carries the unresolved lidar_front_deg error (about 1.6-2 deg, about 3 cm per m);",
            "      the yaw change does not (a fixed mount angle cancels between two scans)",
        ]
    else:
        rows += [
            "real rate, middle 1 s    %s (asked %.2f)" % (f(r["middle_speed"], "%.3f rad/s"), abs(r["rate"])),
            "yaw scan / odom          %.1f / %.1f deg" % (math.degrees(r["scan_yaw"]), math.degrees(r["odom_yaw"])),
            "coast angle after S#     %s" % f(None if r["after_stop"] is None else math.degrees(r["after_stop"]), "%.1f deg"),
        ]
    rows += [
        "stop_delay estimate      %s (scan period %.2f s)" % (f(r["stop_delay"], "%.2f s"), r["scan_period"]),
        "scan agreement start-end %.0f %%" % (100 * r["scan_agree"]),
    ]
    if r["scan_agree"] < 0.5:
        rows.append("WARNING: under 50 %% of the points agree, the start-end match is weak. Chained steps give"
                    " dx %.3f m, dy %+.3f m, yaw %.1f deg" % (r["chain_dx"], r["chain_dy"], math.degrees(r["chain_yaw"])))
    if r["stop_reason"] == "send gap":
        rows.append("WARNING: a send() gap over %.2f s: the base may have stopped and restarted, discard this run" % SEND_GAP)
    return "\n".join(rows)


# ---- the run ----


def run_params(p, args):
    """A copy of params for this run only; params.json is never written."""
    p = dict(p)
    if args.max_speed is not None:
        p["max_speed"] = args.max_speed
    if args.max_turn_rate is not None:
        p["max_turn_rate"] = args.max_turn_rate
    p["linear_cmd_scale"] = args.linear_scale
    return p


def _interrupt(signum, frame):
    raise KeyboardInterrupt("signal %d" % signum)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--line", type=float, metavar="V", help="drive straight at V m/s (true speed)")
    mode.add_argument("--turn", type=float, metavar="W", help="turn on the spot at W rad/s (positive = left)")
    parser.add_argument("--metres", type=float, default=1.5, help="--line: odometry distance before S#")
    parser.add_argument("--degrees", type=float, default=180.0, help="--turn: odometry angle before S#")
    parser.add_argument("--max-speed", type=float, metavar="V", help="raise max_speed for this run only")
    parser.add_argument("--max-turn-rate", type=float, metavar="W", help="raise max_turn_rate for this run only")
    parser.add_argument("--linear-scale", type=float, default=1.0, metavar="S", help="linear_cmd_scale for this run")
    parser.add_argument("--yes-owner-at-robot", action="store_true", help="the owner is at the robot for this run")
    parser.add_argument("--out", type=Path, default=DEFAULT_OUT, help="folder for the JSON log")
    parser.add_argument("--velocity-loop", action="store_true",
                        help="--line: drive the asked true speed through the velocity loop (Task M3)")
    args = parser.parse_args()
    if not args.yes_owner_at_robot:
        sys.exit("REFUSED: the base moves. Pass --yes-owner-at-robot only when the owner is at the robot.")

    p = run_params(params.load(), args)
    line = args.line is not None
    name, rate = ("line", args.line) if line else ("turn", args.turn)
    target = args.metres if line else math.radians(args.degrees)
    if args.max_speed is not None and not 0 < args.max_speed <= MAX_SPEED_LIMIT:
        sys.exit("REFUSED: --max-speed %.2f is outside (0, %.2f]" % (args.max_speed, MAX_SPEED_LIMIT))
    if args.max_turn_rate is not None and not 0 < args.max_turn_rate <= MAX_TURN_LIMIT:
        sys.exit("REFUSED: --max-turn-rate %.2f is outside (0, %.2f]" % (args.max_turn_rate, MAX_TURN_LIMIT))
    if not SCALE_RANGE[0] <= args.linear_scale <= SCALE_RANGE[1]:
        sys.exit("REFUSED: --linear-scale %.3f is outside [%.1f, %.1f]" % ((args.linear_scale,) + SCALE_RANGE))
    if line and not 0 < rate <= p["max_speed"]:
        sys.exit("REFUSED: --line %.2f is outside 0 .. max_speed %.2f (raise it with --max-speed)" % (rate, p["max_speed"]))
    if not line and not 0 < abs(rate) <= p["max_turn_rate"]:
        sys.exit("REFUSED: --turn %.2f is outside 0 .. max_turn_rate %.2f" % (rate, p["max_turn_rate"]))
    if args.velocity_loop and not line:
        sys.exit("REFUSED: --velocity-loop is for --line runs")
    if not wheels_module.under_safe_run():
        sys.exit("REFUSED: " + wheels_module.NOT_SAFE_RUN)

    loop = Loop(p, rate, target) if args.velocity_loop else None
    clear_metres = target if loop is None else Loop.clear_metres(target)
    log = {"mode": name, "rate": rate, "target": target, "velocity_loop": args.velocity_loop, "date": time.strftime("%Y-%m-%d %H:%M:%S"),
           "params": p, "odom": [], "scans": []}
    path = args.out / ("%s-%s-%s.json" % (time.strftime("%Y%m%d-%H%M%S"), name, rate))
    session = lidar.LidarSession()
    try:
        ok, nearest = clear_to_drive(first_full_scan(session, log, time.monotonic, time.sleep), p, name, clear_metres)
        what = "ahead of the front in the strip" if line else "from the turning centre"
        print("nearest usable point %s: %s" % (what, "none" if nearest is None else "%.2f m" % nearest))
        if not ok:
            sys.exit("REFUSED: not clear (need %.2f m)" % (clear_metres + LINE_MARGIN if line else TURN_CLEAR))
        for sig in STOP_SIGNALS[1:]:  # SIGINT already raises KeyboardInterrupt
            signal.signal(sig, _interrupt)
        wheels = wheels_module.Wheels(p)
        try:
            drive(wheels, session, p, name, rate, target, log, loop=loop)
            settle(wheels, session, log)
            if loop is not None:
                log["estimates"] = loop.summary()
        finally:
            with signals_ignored():
                wheels.close()  # sends S# once more
    finally:
        session.close()
        args.out.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(log))
        print("wrote", path)
    if log["stop_reason"] == "emergency":
        print("EMERGENCY STOP: a point came within %.2f m" % emergency_distance(rate if line else 0.0))
    log["report"] = report(log, p)
    path.write_text(json.dumps(log))
    print(table(log["report"]))


if __name__ == "__main__":
    main()
