"""Follow one operator with the hmc brain on the real base.

    run_follow.py --seconds N [--drive] [--max-speed V] [--no-jetson]

Without --drive the wheel port is never opened: the brain runs on the real sensors, the
odometry stays at zero motion and nothing is sent to the wheels (a dry run).

One cycle per LiDAR revolution (10 Hz): read the sensors, let the brain decide, check the
safety stop, send the wheel command, log the cycle. Logs go to logs/<start time>/ next to
this file: log.jsonl (one JSON record per cycle) and every 10th scan with its camera frame.
"""
import argparse
import contextlib
import json
import math
import signal
import sys
import time
from pathlib import Path

import numpy

from hmc.follower import Follower

import camera
import depth_obstacles
import geometry
import lidar
import link as link_module
import params
import safety
import wheels as wheels_module
from body import Body
from senses import LEARNED, LOOK_MATCH, RealSenses

PERIOD = 0.1  # s, one LiDAR revolution
SLOW_CYCLE = 0.4  # s; a longer cycle stops the wheels and ends the run
SAVE_EVERY = 10  # save every 10th JPEG; every scan is saved, so a lost track can be replayed
SEARCH_AGAIN_EVERY = 10.0  # s; after asking the operator back, turn on the spot again (hmc.follower)
CLOSED_GATE_SPEED = 0.15  # m/s; above this --drive needs the braking numbers measured (params.speed_gates_open)
STILL = (0.0, 0.0, 0.0)  # the dry run's pose: zero motion
LOG_ROOT = Path(__file__).resolve().parent / "logs"  # on the Pi: ~/hmc-base/logs


class Log:
    """logs/<start time>/log.jsonl, flushed every line so a crash keeps what came before."""

    def __init__(self, folder):
        self.folder = Path(folder)
        self.folder.mkdir(parents=True, exist_ok=True)
        self.file = open(self.folder / "log.jsonl", "a", encoding="utf-8")

    def write(self, record):
        self.file.write(json.dumps(record) + "\n")
        self.file.flush()

    def save_scan(self, n, scan, jpeg):
        (self.folder / ("scan-%06d.json" % n)).write_text(json.dumps(scan))
        if jpeg is not None:
            (self.folder / ("frame-%06d.jpg" % n)).write_bytes(jpeg)

    def close(self):
        self.file.close()


class Speech:
    """What the brain says. Printed and logged for now; the voice work plugs in here later."""

    def __init__(self, log):
        self.log = log

    def say(self, time, text):
        print("[%.1f] SAY: %s" % (time, text))
        self.log.write({"time": time, "say": text})


def _no_jpeg(frame):
    return None


class State:
    """Everything one cycle needs. Devices are injected, so tests pass fakes.

    wheels: wheels.Wheels, or None for a dry run. lidar: has latest() -> scan.
    camera: has latest() -> Frame or None. link: JetsonLink or None (--no-jetson).
    encode: frame -> JPEG bytes, for the saved frames. clock: the same clock as cycle's `now`.
    """

    def __init__(self, p, wheels, lidar, camera, link, log, max_speed=None, encode=_no_jpeg, clock=time.monotonic):
        self.p = p
        self.wheels, self.lidar, self.camera, self.link, self.log = wheels, lidar, camera, link, log
        self.max_speed = p["max_speed"] if max_speed is None else max_speed
        self.encode, self.clock = encode, clock
        self.body = Body(p)
        self.senses = RealSenses(p)
        # The brain stops follow_distance from the operator, measured centre to centre
        # (navigation.follow_step -> Robot.drive_toward uses distance(robot.pos, target)).
        # follow_gap is measured from the robot's front edge, so add the centre-to-front length.
        self.follower = Follower(self.body.robot, Speech(log), follow_distance=p["follow_gap"] + safety.FRONT,
                                 search_again_every=SEARCH_AGAIN_EVERY)
        self.sent = (0.0, 0.0)  # the last command the wheels got: the motion since the last scan
        self.last_now = None
        self.n = 0  # cycles so far
        self.enrol_said = None


def scan_points(scan, p):
    """Usable scan points (the robot's own posts left out) as an (N, 2) base-frame array."""
    pts = [geometry.lidar_to_base((m * math.cos(math.radians(deg)), m * math.sin(math.radians(deg))), p)
           for deg, m in lidar.usable(scan)]
    return numpy.array(pts, dtype=float).reshape(-1, 2)


def _stop_wheels(state):
    """S# now, several times. A dead port cannot take it, and whether the firmware then stops
    on its own silence is NOT known (tools/air_check.py measures it)."""
    if state.wheels is None:
        return
    with contextlib.suppress(OSError, ValueError):
        state.wheels.stop()


def _end(state, now, reason, detail):
    _stop_wheels(state)
    record = {"time": now, "stop": reason, "end": "%s: %s" % (reason, detail)}
    state.log.write(record)
    return record


def cycle(state, now):
    """One control cycle. Returns the log record; a record with "end" means stop the run."""
    p = state.p
    period = None if state.last_now is None else now - state.last_now
    if period is not None and period > SLOW_CYCLE:
        return _end(state, now, "slow cycle", "%.2f s since the last cycle" % period)
    dt = PERIOD if period is None else period
    state.last_now = now

    # 1. Sensors. Broken odometry means we no longer know where we are: stop (Review Focus 5).
    try:
        pose = state.wheels.read_odom() if state.wheels is not None else STILL
    except (ValueError, OSError) as error:  # pyserial's SerialException is an OSError
        return _end(state, now, "odometry", error)
    scan = state.lidar.latest()
    frame = state.camera.latest()
    answer = state.link.latest() if state.link is not None else None
    link_alive = state.link is not None and state.link.alive(now, p["jetson_silence"])

    # 2. Depth obstacles (only once the camera pose is measured), the speed cap, the pose.
    depth_points = None
    if frame is not None and params.depth_gates_open(p):
        depth_points = depth_obstacles.points(frame.depth, p)
    state.body.limit(min(state.max_speed, safety.speed_cap(depth_points, p)))
    state.body.sync(pose)

    # 3. The brain. While learning, RealSenses cannot lock by itself: enrol again whenever unlocked.
    senses, follower = state.senses, state.follower
    senses.learned = follower.operator_look.count
    enrol = None
    if senses.learned < LEARNED and not senses.locked and answer is not None:
        # The first enrol takes the one person in view; once a look is learned, a re-enrol
        # must look like the operator, or a stranger who steps in would be learned too.
        accept = (lambda look: follower.score(look) >= LOOK_MATCH) if senses.learned else None
        result = senses.enrol(now, scan, answer, accept)
        enrol = result if isinstance(result, str) else "locked"
        if enrol != state.enrol_said:
            print("enrol: %s" % enrol)
            state.enrol_said = enrol
    v_was, w_was = state.sent
    seen = senses.read(now, pose, v_was, w_was, dt, scan, depth_points, answer, link_alive, follower.score)
    follower.watch(seen)  # every cycle, before step: it keeps track of everyone
    follower.step(seen, dt)

    # 4. The command, and the stops that override the brain.
    stop = None
    if senses.ambiguous:
        stop = "ambiguous"  # someone is right at the operator: wait, never guess who is who
    points = scan_points(scan, p)
    if depth_points is not None:
        points = numpy.vstack([points, depth_points])
    v, w = state.body.command()
    if stop is None and safety.must_stop(points, v, w, p):
        stop = "obstacle"
    if stop is None and not scan:
        stop = "no scan"  # the LiDAR went quiet: blind, so do not move
    if stop is not None:
        v = w = 0.0
        state.body.robot.speed = state.body.robot.turn_speed = 0.0  # the brain ramps up again from standstill

    # 5. Wheels, Jetson, log. A command computed from sensors read too long ago is stale: stop instead.
    cycle_time = state.clock() - now  # compute time, cycle start to send
    if cycle_time > SLOW_CYCLE:
        return _end(state, now, "slow cycle", "%.2f s from cycle start to send" % cycle_time)
    if state.wheels is not None:
        try:
            state.wheels.send(v, w)
        except (ValueError, OSError) as error:
            return _end(state, now, "wheels", error)
        state.sent = (v, w)
    if frame is not None and state.link is not None:
        state.link.send(frame)
    if state.n % SAVE_EVERY == 0:
        state.log.save_scan(state.n, scan, state.encode(frame) if frame is not None else None)
    state.n += 1

    operator = follower.last_operator_pos
    record = {
        "time": now,
        "pose": list(pose),
        "v": v,
        "w": w,
        "mode": follower.mode.value,
        "locked": senses.locked,
        "learned": follower.operator_look.count,
        "link": link_alive,
        "operator": None if seen.operator is None else list(seen.operator),
        "operator_kept": None if operator is None else list(operator),  # the brain's smoothed operator
        "people": [list(pt) for pt in seen.people.values()],
        "obstacles": len(seen.obstacles),
        "nearest": float(numpy.hypot(points[:, 0], points[:, 1]).min()) if len(points) else None,  # m from the base centre
        "stop": stop,
        "why": getattr(senses, "why", None),  # the senses rule behind an ambiguous cycle or a dropped track
        "enrol": enrol,
        "cycle_time": cycle_time,  # s from cycle start to send
        "period": period,  # s since the previous cycle started
    }
    state.log.write(record)
    return record


def run(state, seconds, clock=time.monotonic, sleep=time.sleep):
    """Cycle at PERIOD for `seconds`. Returns why the run ended early, or None."""
    start = next_at = clock()
    while clock() - start < seconds:
        record = cycle(state, clock())
        if "end" in record:
            print("STOPPED: %s" % record["end"])
            return record["end"]
        next_at = max(next_at + PERIOD, clock())  # never try to catch up with a burst of cycles
        sleep(max(0.0, next_at - clock()))
    return None


def refusal(drive, max_speed, p):
    """Why this run must not start, or None."""
    if max_speed is not None and not max_speed > 0:
        return "--max-speed must be above 0"
    if drive:
        speed = p["max_speed"] if max_speed is None else max_speed
        if speed > p["max_speed"]:
            return "--max-speed %.2f is above max_speed %.2f in params.json" % (speed, p["max_speed"])
        if speed > CLOSED_GATE_SPEED and not params.speed_gates_open(p):
            return "--drive above %.2f m/s needs decel, stop_delay and linear_cmd_scale measured" % CLOSED_GATE_SPEED
    return None


def open_wheels(drive, p):
    """The wheel port is opened only with --drive."""
    return wheels_module.Wheels(p) if drive else None


# Ctrl+C, kill, and SIGHUP from a dropped ssh session (SIGHUP does not exist on Windows).
STOP_SIGNALS = [s for s in (signal.SIGINT, signal.SIGTERM, getattr(signal, "SIGHUP", None)) if s is not None]


def _interrupt(signum, frame):
    raise KeyboardInterrupt("signal %d" % signum)


@contextlib.contextmanager
def signals_ignored():
    """A second signal must not interrupt the stop itself."""
    old = [(s, signal.signal(s, signal.SIG_IGN)) for s in STOP_SIGNALS]
    try:
        yield
    finally:
        for s, handler in old:
            signal.signal(s, handler)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--seconds", type=float, required=True, help="how long to run")
    parser.add_argument("--drive", action="store_true", help="open the wheel port and drive (default: dry run)")
    parser.add_argument("--max-speed", type=float, metavar="V", help="top speed for this run, m/s")
    parser.add_argument("--no-jetson", action="store_true", help="run without the Jetson (nobody can be enrolled)")
    args = parser.parse_args(argv)
    p = params.load()
    why = refusal(args.drive, args.max_speed, p)
    if why:
        sys.exit("REFUSED: " + why)
    if args.drive and not wheels_module.under_safe_run():
        sys.exit("REFUSED: --drive: " + wheels_module.NOT_SAFE_RUN)

    for sig in STOP_SIGNALS[1:]:  # SIGINT already raises KeyboardInterrupt
        signal.signal(sig, _interrupt)
    log = Log(LOG_ROOT / time.strftime("%Y%m%d-%H%M%S"))
    print("logging to", log.folder)
    session = cam = link = wheels = None
    try:
        session = lidar.LidarSession()
        cam = camera.Camera()
        cam.start()
        if not args.no_jetson:
            link = link_module.JetsonLink(p["jetson_host"], p["jetson_port"], p["camera_hfov_deg"])
            link.start()
        wheels = open_wheels(args.drive, p)
        state = State(p, wheels, session, cam, link, log, args.max_speed, encode=link_module._encode_jpeg)
        print("%s. Operator: stand alone within 2.5 m in front of the robot, then press Enter."
              % ("DRIVE" if args.drive else "DRY RUN, the wheels stay off"))
        input()
        run(state, args.seconds)
    except KeyboardInterrupt as stop:
        with signals_ignored():
            if wheels is not None:
                with contextlib.suppress(OSError):
                    wheels.stop()  # S# first
            print("STOPPED: %s" % (stop or "Ctrl+C"))
            log.write({"time": time.monotonic(), "stop": "signal", "end": "signal: %s" % (stop or "Ctrl+C")})
    finally:
        with signals_ignored():
            if wheels is not None:
                wheels.close()  # S# again, before anything else is shut down
            for device in (link, cam):
                if device is not None:
                    device.stop()
            if session is not None:
                session.close()
            log.close()
    print("log:", log.folder)


if __name__ == "__main__":
    main()
