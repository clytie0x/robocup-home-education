"""Follow one operator with the hmc brain on the real base.

    run_follow.py --seconds N [--drive] [--max-speed V] [--no-jetson] [--return-after N] [--remote [PORT]]

Without --drive the wheel port is never opened: the brain runs on the real sensors, the
odometry stays at zero motion and nothing is sent to the wheels (a dry run).

Phases: FOLLOW until Enter on the console (or --return-after N seconds), then RETURN along the
breadcrumbs of the way out (hmc.route), behind the queue while one walks ahead (hmc.queue), with
the pose corrected against the scans stored at the breadcrumbs (keyframes.py). DONE ends the run.

--remote: the Jetson's commands (command_server.py, port 5604) replace Enter. The run starts in
IDLE (wheels stopped, senses running, enrol allowed); follow, wait, resume, return and stop come
over TCP, a status line goes back at most 5 Hz, and 2 s without a command line holds the robot
still until resume, follow or return. The way back starts where follow came.

BAG (--remote): the client's "bag" while it holds the robot with wait in FOLLOW (at the car) finds the bag the
operator points at and drives up to it until the arm reaches it (bag.py); "bag_again" measures and approaches
the last bag once more; "bag_near" goes to the nearest bag in front, nobody points, also from IDLE (a step test
from the console). The status line carries "bag": its id, state, the chosen bag and the result; the
record "bag" is the chosen bag in the odometry frame (the PC's virtual operator app shows it). Wait, resume,
follow or return cancel it; the drive counts as path for the way back. Then the phase it came from: FOLLOW,
still waiting, or IDLE.

The dashboard's task buttons (Follow, Wait, Resume, Go back) work when voice or the Jetson is down:
with --remote they are taken as the client's commands, without it Go back is Enter and Wait holds
still until Resume. With no console (nohup) the run waits for Follow on the page instead of Enter.

One cycle per LiDAR revolution (10 Hz): read the sensors, let the brain decide, check the
safety stop, send the wheel command, log the cycle. Logs go to logs/<start time>/ next to
this file: log.jsonl (one JSON record per cycle) and every 10th scan with its camera frame.
"""
import argparse
import contextlib
import dataclasses
import json
import math
import select
import signal
import socket
import subprocess
import sys
import threading
import time
from pathlib import Path

import numpy

import hmc.follower as hmc_follower
from hmc import hri
from hmc import world as hmc_world
from hmc.follower import Follower, Mode
from hmc.queue import QueueFollower
from hmc.route import Returner, Route
from hmc.task import STAND_STILL, STANDING_FORGET
from hmc.world import Circle

import bag as bag_module
import cable as cable_module
import calib
import camera
import command_server
import dashboard as dashboard_module
import depth_obstacles
import escape as escape_module
import footprint
import geometry
import keyframes as keyframes_module
import lidar
import link as link_module
import live_feed
import motion as motion_module
import params
import ports
import remote_plan
import safety
import velocity
import virtual_operator as virtual_module
import wheels as wheels_module
from body import Body
from senses import LEARNED, LOOK_MATCH, RealSenses

# The simulation keeps a small obstacle near the robot for ever while the camera cannot see it.
# This camera sees only 58 deg ahead, so every LiDAR and depth cell the robot turned away from
# stayed, until no path was left (2026-11-13, second drive: no path from 21 s on). Forget them
# after MEMORY_KEEP like everything else; the camera turns toward where the robot will drive.
hmc_follower.KEEP_NEAR = 0.0
# The brain's own memory (MEMORY_KEEP, 10 s) came on top of RealSenses' depth memory (shapes.DEPTH_KEEP,
# 10 s): one stray circle stayed 20 s, and such circles on open floor left no way to the operator 2.5 m
# away (2026-11-15: 73 of 81 failed plans had a way with the current LiDAR circles alone). The LiDAR
# sees all round every scan and RealSenses keeps the depth points: the brain only bridges a second
# (a LiDAR circle hidden for a moment behind a walking person).
hmc_follower.MEMORY_KEEP = 1.0

# Diagnostic (2026-11-13): while following, the planner often found no way to the operator
# though a replay of the saved scans did. Keep the inputs of the last failed plan for the log.
import hmc.navigation as hmc_navigation  # noqa: E402

_plan_path = hmc_navigation.plan_path
NO_PATH = {}


# The planner in its own process (plan_server.py), when run_follow started one: see main().
REMOTE = None
PLANNER = {"remote": 0, "local": 0}  # plans of each kind since the last log record


LAY = {"lay": None}  # the running cycle's cable.Lay: plans keep off the cable when they can (owner, 2026-11-15)


def _plan_path_logged(start, goal, shapes, near_side=False, heading=None):
    if LAY["lay"] is not None:
        return LAY["lay"].plan(_plan_once, start, goal, shapes, near_side, heading,
                               waiting=lambda: REMOTE is not None and REMOTE.waiting())
    return _plan_once(start, goal, shapes, near_side, heading)


def _plan_once(start, goal, shapes, near_side=False, heading=None):
    if REMOTE is not None:
        REMOTE.close_plan = hmc_navigation.CLOSE_PLAN  # the page's switch goes with each request
    if REMOTE is not None and REMOTE.alive():
        path = REMOTE.plan(start, goal, shapes, near_side, heading)
        PLANNER["remote"] += 1
    else:
        if REMOTE is not None:
            REMOTE.plan(start, goal, shapes, near_side, heading)  # keep asking: it answers once it is up
        path = _plan_path(start, goal, shapes, near_side, heading)
        PLANNER["local"] += 1
    if path is None and not (REMOTE is not None and REMOTE.waiting()):  # waiting: no answer yet, not no way
        NO_PATH.update(start=list(start), goal=list(goal), near_side=near_side, heading=heading,
                       shapes=[[type(s).__name__] + [round(v, 3) for v in vars(s).values()] for s in shapes])
    return path


hmc_navigation.plan_path = _plan_path_logged

PERIOD = 0.1  # s, one LiDAR revolution
SLOW_CYCLE = 0.4  # s; a longer cycle stops the wheels and ends the run
PLAN_PORT = ports.PLAN  # plan_server.py
SAVE_EVERY = 10  # save every 10th JPEG; every scan is saved, so a lost track can be replayed
SEARCH_AGAIN_EVERY = 10.0  # s; after asking the operator back, turn on the spot again (hmc.follower)
# hmc.follower's pace and stop-and-go for this base (the simulation keeps its own, set for 1 m/s). It started at
# 1.31 m and stopped at 1.30 m, 4.8 to 13.1 start-stops a minute (logs of 2026-11-15): start again only past
# RESTART_GAP more. hmc's WAIT_GAP 4.0 and GO_ON_GAP 2.5 m were set for the simulation's 1 m/s base, not 0.12 m/s.
RESTART_GAP = 0.25  # m past the follow distance
WAIT_GAP, GO_ON_GAP = 2.0, 1.2  # m
# Finding the operator before the first lock (Task 7): turn on the spot while every frame goes to the
# detector, and stop the moment a person is near the middle of the image, so enrol can lock onto them.
SEARCH_TURN = 0.3  # rad/s, slow enough for the camera answers (about 0.3 s old) to stop it in time
SEARCH_CENTRE_DEG = 20.0  # a camera person within this of the axis stops the turn
SEARCH_MAX = 4 * math.pi  # rad; after two full turns, stand and wait
SEARCH_HOLD = 3.0  # s held on a centred person who does not lock (a stranger, too far): then turn past them
SEARCH_SKIP = 1.5  # s of turning on, past them, before a centred person can stop the turn again
HISTORY = 2.0  # s of sent commands kept for the latency prediction
CLOSED_GATE_SPEED = 0.15  # m/s; above this --drive needs the braking numbers measured (params.speed_gates_open).
# 0.15 itself is allowed once params.json max_speed is raised. tools/brake_distance.py and follow_speed.py
# measure the stop first; this file does not change the cap.
STILL = (0.0, 0.0, 0.0)  # the dry run's pose: zero motion
LOG_ROOT = Path(__file__).resolve().parent / "logs"  # on the Pi: ~/hmc-base/logs
FOLLOW, RETURN, DONE = "follow", "return", "done"  # the phases of a run
IDLE = "idle"  # --remote only: before the client's follow; the wheels stay stopped
BAG = "bag"  # --remote only: from FOLLOW held by the client's wait, up to the pointed bag (bag.py), then FOLLOW again
BAG_COMMANDS = {"bag": "point", "bag_again": "again", "bag_near": "near"}  # command -> bag.BagTask mode
STATUS_EVERY = 0.2  # s; --remote status lines at most 5 Hz (and after each command)
REMOTE_SILENCE = 2.0  # s without a command line from the --remote client: hold still until resume, follow or return
REFUSED_IDLE = {"return": "not following yet", "stop": "idle: nothing to stop"}  # --remote commands IDLE answers with an error
# m; a larger pose correction leaves the brain's remembered obstacles in the wrong place (Review Focus 2)
CORRECTED = 0.1
TURN_REACH = 2.0  # m; a turn correction counts by how far it moves a point this far away
# Wheels that do not turn (2026-11-14: the motor supply died and the run commanded 0.114 m/s for 60 s, its status
# saying nothing). Found by the Pi (wheels.StallWatch) or by the firmware's encoder-fault flags (E#); each time
# the robot stops, sends S# (which clears the fault), says so and holds, latched like the silence hold, until an
# explicit resume or follow (client or page): someone plugging the battery in must not see it drive off by itself.
# The WHEELS_EPISODES-th time ends the run.
WHEELS_EPISODES = 3
WHEELS_POLL = 0.5  # s between E# reads: the port already carries an O# every cycle
WHEELS_MISSES = 3  # unreadable E# answers in a row: the old firmware (no E#), not asked again this run
WHEELS_TEXT = "My wheels are not turning. Please check the motor power."
# Good motion since the last time forgets the count, so short false alarms minutes apart do not end the run.
WHEELS_FORGET_M = 0.3  # m driven
WHEELS_FORGET_S = 10.0  # s of a watched command with the wheels turning


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

    def __init__(self, p, wheels, lidar, camera, link, log, max_speed=None, encode=_no_jpeg, clock=time.monotonic,
                 velocity_loop=False, return_after=None, remote=None, cable_path=None):
        self.p = p
        self.wheels, self.lidar, self.camera, self.link, self.log = wheels, lidar, camera, link, log
        self.max_speed = p["max_speed"] if max_speed is None else max_speed
        self.encode, self.clock = encode, clock
        self.body = Body(p)
        self.senses = RealSenses(p)
        # The brain stops follow_distance from the operator, measured centre to centre
        # (navigation.follow_step -> Robot.drive_toward uses distance(robot.pos, target)).
        # follow_gap is measured from the robot's front edge, so add the centre-to-front length.
        self.speech = Speech(log)
        self.follower = Follower(self.body.robot, self.speech, follow_distance=p["follow_gap"] + safety.FRONT,
                                 search_again_every=SEARCH_AGAIN_EVERY, restart_gap=RESTART_GAP,
                                 wait_gap=WAIT_GAP, go_on_gap=GO_ON_GAP)
        self.remote = remote  # command_server.CommandServer (--remote), or None: Enter and --return-after only
        self.phase = FOLLOW if remote is None else IDLE
        self.waiting = False  # --remote "wait": hold still, the senses and the brain keep running
        self.silent = False  # --remote: nothing heard from the client for REMOTE_SILENCE
        self.page_leads = False  # --remote: a task button came after the client's last line: while the page polls, its silence holds nothing
        self.page_wait = False  # Wait on the page: held ("page") until the page's resume, follow or return (or Enter)
        self.ack = None  # the client's last command (a ping is not one)
        self.status_at = -math.inf  # when the last status line went to the client
        self.last_record = None  # the last cycle record, for a status sent with an end record
        self.go_back = threading.Event()  # set: start the way back (Enter, or a test)
        self.enter = None  # () -> True when Enter was pressed on the console, polled every cycle (main), or None
        self.return_after = return_after  # s of following before RETURN by itself (--return-after), or None
        self.started = None  # time of the first cycle
        self.start = None  # (x, y, heading) of the first cycle: where the way back ends
        # odometry frame -> corrected frame: the keyframe corrections so far. believed = _compose(fix, odometry)
        self.fix = (0.0, 0.0, 0.0)
        self.keyframes = keyframes_module.Keyframes(p)
        self.queue = None  # hmc QueueFollower, made in the first cycle
        self.joined = False  # walking behind the queue (hmc.task's JOIN_QUEUE)
        self.returner = None  # hmc Returner, made anew each time the drive along the breadcrumbs starts
        self.standing = []  # places where someone stands: [since, where, last seen] (hmc.task.Task.standing)
        self.sent = (0.0, 0.0)  # the last command the wheels got: the motion since the last scan
        self.coast = safety.Coast(p)  # what the base may still be doing: it slows at decel, not at once
        self.history = []  # (time, v, w) of every command sent in the last HISTORY seconds
        self.searched = 0.0  # rad turned while searching before the first lock
        self.search_yaw = None  # yaw at the previous search cycle
        self.hold_since = None  # when the search started holding on a centred person
        self.skip_until = -math.inf  # until then a centred person does not stop the search
        self.last_now = None
        self.n = 0  # cycles so far
        self.enrol_said = None
        self.controls = dashboard_module.Controls(self.max_speed, clock=clock)  # the page's switches (dashboard.py)
        self.motion = motion_module.MotionEstimator(p)  # measured motion: odometry checked by scan matching (M1)
        self.estimates = calib.BaseEstimates(p)  # how the base really drives, measured while it drives (M2)
        self.velocity = velocity.VelocityLoop(p, self.estimates) if velocity_loop else None  # M3, --velocity-loop
        self.measured = None  # the newest motion.MotionState
        self.dashboard = None  # the page's server, when main started one
        self.stall = wheels_module.StallWatch()  # the Pi's own check that the wheels turn
        self.wheels_alarm = None  # "not turning" or "fault 03" while held for it, else None
        self.wheels_held = False  # latched for wheels that did not turn, until resume or follow
        self.wheels_episodes = 0
        self.wheels_good = [0.0, 0.0]  # m and s of good motion since the last time (WHEELS_FORGET_M, WHEELS_FORGET_S)
        self.wheels_last = None  # (time, pose) of the previous watched cycle
        self.firmware = None  # the newest E# answer (wheels.parse_status), None until one is read
        self.firmware_at = -math.inf  # when E# was last asked
        self.firmware_misses = 0  # unreadable E# answers in a row
        self.cable = cable_module.Cable(p, cable_path, self.speech)  # the power cable's net turn; path None: not saved
        self.escape = escape_module.Escape(p, self.speech, self.body.robot, lambda: _replan(self))  # out of corners
        self.virtual = None  # virtual_operator.VirtualOperator (--virtual-operator): a simulated person, or None
        self.bag = None  # bag.BagTask: the running one in BAG, then the last one (its result stays on the status)
        self.bag_seq = 0  # bag tasks so far this run
        self.bag_from = FOLLOW  # the phase the running bag task came from, and goes back to (bag_near: IDLE too)
        self.bag_odom = None  # odometry (x, y) of the last chosen bag (record "bag", and what bag_again looks for)
        self.bag_look = None  # bag.Look: the Jetson's front camera measures the bag when depth cannot (None: no Jetson)


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
    _report(state, now, record)
    return record


def _poll_firmware(state, now):
    """E# at most every WHEELS_POLL: the firmware's own encoder-fault flags. After WHEELS_MISSES unreadable
    answers in a row (the old firmware has no E#) it is not asked again this run; the Pi's check stays."""
    if state.firmware_misses >= WHEELS_MISSES or now - state.firmware_at < WHEELS_POLL - 1e-6:
        return
    state.firmware_at = now
    try:
        state.firmware = state.wheels.read_status()
    except (ValueError, OSError):  # a dead port also fails the odometry read, which ends the run
        state.firmware_misses += 1
        if state.firmware_misses == WHEELS_MISSES:
            state.log.write({"time": now, "wheels": "no E# answer from the firmware: its fault flags are not read this run"})
        return
    state.firmware_misses = 0


def _watch_wheels(state, now, pose):
    """FOLLOW, RETURN and BAG with real wheels: stop and hold when the wheels do not turn. Returns the end record
    at the WHEELS_EPISODES-th time, else None."""
    if state.wheels is None:
        return None
    if state.wheels_held:
        return None
    _poll_firmware(state, now)
    stalled = state.stall.update(now, state.sent, pose)
    if state.phase not in (FOLLOW, RETURN, BAG):
        return None
    flags = 0 if state.firmware is None else state.firmware["flags"]
    if flags & (wheels_module.FAULT_LEFT | wheels_module.FAULT_RIGHT):
        state.wheels_alarm = "fault %02x" % flags
    elif stalled:
        state.wheels_alarm = "not turning"
    else:
        _good_motion(state, now, pose)
        return None
    state.wheels_episodes += 1
    state.wheels_good = [0.0, 0.0]
    state.log.write({"time": now, "wheels": state.wheels_alarm, "episode": state.wheels_episodes, "firmware": state.firmware})
    state.speech.say(now, WHEELS_TEXT)
    if state.wheels_episodes >= WHEELS_EPISODES:
        return _end(state, now, "wheels not turning", "%s, %d times: is the motor power on?"
                    % (state.wheels_alarm, state.wheels_episodes))
    _stop_wheels(state)  # S# now: it also clears the firmware's encoder fault
    state.wheels_held = True
    return None


def _release_wheels(state, now, cmd):
    """resume or follow (client or page) after a wheel stop: motion allowed again, watched afresh."""
    if not state.wheels_held:
        return
    state.log.write({"time": now, "wheels": "hold released by %s" % cmd})
    state.wheels_held = False
    state.wheels_alarm = state.firmware = state.wheels_last = None  # a fault flag read before the S# is gone
    state.stall.reset()


def _good_motion(state, now, pose):
    """Count the motion of a cycle without an alarm; enough of it forgets the times so far."""
    last, state.wheels_last = state.wheels_last, (now, pose)
    if not state.wheels_episodes or last is None:
        return
    state.wheels_good[0] += math.dist(pose[:2], last[1][:2])
    if abs(state.sent[0]) >= wheels_module.STALL_V or abs(state.sent[1]) >= wheels_module.STALL_W:
        state.wheels_good[1] += now - last[0]
    if state.wheels_good[0] >= WHEELS_FORGET_M or state.wheels_good[1] >= WHEELS_FORGET_S:
        state.log.write({"time": now, "wheels": "%d times forgotten after %.2f m, %.1f s of good motion"
                         % (state.wheels_episodes, state.wheels_good[0], state.wheels_good[1])})
        state.wheels_episodes, state.wheels_good = 0, [0.0, 0.0]


def _page_command(state, cmd):
    """The page's own hold: its Wait holds until its resume, follow or return, whatever the client sends."""
    if cmd == "wait":
        state.page_wait = True
    elif cmd in ("resume", "follow", "return"):
        state.page_wait = False


def _remote_commands(state, now):
    """--remote: take the client's commands that arrived since the last cycle. Returns the names of
    those taken (a refused one is answered with an error and changes nothing)."""
    remote = state.remote
    quiet = remote.silent(now)  # how long the client had been silent before this cycle's lines
    commands = remote.poll(now)
    page = state.controls.take_commands()  # the page's task buttons, taken as if the client had sent them
    for event in remote.events:
        state.log.write({"time": now, "remote": event})
    remote.events.clear()
    # No Jetson: the operator drives the task from the page, but only while the page polls (its heartbeat);
    # a phone that lost the Wi-Fi leads nothing, and the silence hold is back.
    leading = (state.page_leads or bool(page)) and state.controls.page_alive(now)
    if commands:
        state.page_leads = False  # the client is heard again: its silence holds again, from now on
    elif page:
        state.page_leads = True
    if quiet > REMOTE_SILENCE and not state.silent and not leading:
        state.silent = True  # latched: only resume, follow or return releases it, not a ping
        state.log.write({"time": now, "remote": "silent %.1f s: held until resume, follow or return" % quiet})
    taken = []
    for i, cmd in enumerate(commands + page):
        by_page = i >= len(commands)
        # a return in a bag task from IDLE is IDLE's too: taken, it would wait in go_back and end the next follow
        idle = state.phase == IDLE or (state.phase == BAG and state.bag_from == IDLE and cmd == "return")
        refused = REFUSED_IDLE.get(cmd) if idle else None
        if refused is None and cmd in BAG_COMMANDS:
            refused = _bag_refused(state, cmd)
        if refused is not None:
            if not by_page:
                remote.send(dict({"error": refused}, **({"refused": cmd} if cmd in BAG_COMMANDS else {})))
            state.log.write({"time": now, "remote_refused": cmd, "why": refused, "by": "page" if by_page else "client",
                             "wall": time.time()})  # wall: the console merges these into its command list
            continue
        taken.append(cmd)
        if cmd == "ping":
            continue
        if not by_page:
            state.ack = cmd  # the client's own last command: the bridge must not take a page press for its own
        state.log.write(dict({"time": now, "page_cmd" if by_page else "remote_cmd": cmd},
                             **({} if by_page else {"wall": time.time()})))  # wall: for the console's command list
        if by_page:
            _page_command(state, cmd)
        if state.phase == BAG and cmd in ("wait", "resume", "follow", "return"):
            _end_bag(state, now, cmd)  # cancelled: FOLLOW again, then the command as always
        if cmd in BAG_COMMANDS:
            if by_page:
                state.page_wait = False  # the page asks the robot to move to the bag: its own Wait is over
            _start_bag(state, now, cmd)
        if cmd in ("resume", "follow"):
            _release_wheels(state, now, cmd)
        if cmd == "follow" and state.phase == IDLE:
            # The way back starts where follow came, not where the program started (the robot may
            # have been carried to the start line): the first-cycle setup runs again this cycle.
            state.phase, state.start = FOLLOW, None
            state.keyframes = keyframes_module.Keyframes(state.p)
        if cmd == "return":
            state.go_back.set()
        if cmd in ("wait", "resume", "follow", "return"):
            state.waiting = cmd == "wait"  # follow and return may move again, as resume
        # bag_near too: from IDLE with no Jetson the silence latched long before the console's step test
        if cmd in ("resume", "follow", "return", "bag_near") and state.silent:
            state.silent = False
            state.log.write({"time": now, "remote": "silence hold released by %s" % cmd})
    return taken


def _bag_refused(state, cmd):
    """Why a bag command cannot start now, or None: only at the car, in FOLLOW held by the client's wait; bag_near
    also from IDLE (no operator needed: a step test from the console)."""
    if state.phase == BAG:
        return "bag: a bag task is running"
    if cmd == "bag_near" and state.phase == IDLE:
        pass
    elif state.phase != FOLLOW or not state.waiting:
        return "bag: only while waiting at the car (FOLLOW after wait)%s, not in %s" % (
            " or in IDLE" if cmd == "bag_near" else "", state.phase.upper())
    if state.wheels_held:
        return "bag: held for the wheels"
    return None


def _start_bag(state, now, cmd):
    state.bag_seq += 1
    state.bag = bag_module.BagTask(state.p, now, state.bag_seq, BAG_COMMANDS[cmd], last=state.bag_odom, look=True,
                                   rgb=state.bag_look is not None)
    state.bag_from, state.phase = state.phase, BAG
    state.log.write({"time": now, "bag": "start", "mode": state.bag.mode, "seq": state.bag_seq})


def _end_bag(state, now, by=None):
    """Back to the phase it came from: FOLLOW, held by the wait the bag came in, or IDLE (a command that cancelled
    it then does what it does)."""
    if by is not None:
        state.bag.cancel(now, by)
    state.phase = state.bag_from
    state.body.robot.speed = state.body.robot.turn_speed = 0.0  # the brain ramps up from standstill
    _replan(state)
    state.log.write({"time": now, "bag": "end", "status": state.bag.status()})


def _bag_step(state, now, pose, believed, seen, sensed, answer, frame, points):
    """BAG: this cycle's (v, w, turn error) from the bag task, with its view of the cycle."""
    p = state.p
    operator = None if seen.operator is None else geometry.odom_to_base(seen.operator, believed)
    found = virtual_bags = None
    report = state.virtual.report if state.virtual is not None else None
    if report is not None and report.get("person") is not None:  # --virtual-operator: the PC's person and spot
        operator = geometry.odom_to_base(report["person"][:2], pose)
        if report.get("pointing") is not None:
            found = bag_module.virtual_pointing(geometry.odom_to_base(report["pointing"], pose), operator)
        virtual_bags = [geometry.odom_to_base(b, pose) for b in report.get("bags") or []]
    elif state.bag.wants_pose:
        found = bag_module.camera_pointing(answer, operator, (getattr(frame, "width", 320), getattr(frame, "height", 240)), p)
    boxes = [] if answer is None else [o.box for o in getattr(answer, "objects", [])]
    view = bag_module.View(
        now, pose, seen=scan_points(sensed, p), clusters=bag_module.lidar_clusters(sensed, p), operator=operator,
        found=found, boxes=boxes, frame=(getattr(frame, "width", 320), getattr(frame, "height", 240)),
        virtual_bags=virtual_bags or (), lay=[geometry.odom_to_base((c.x, c.y), believed) for c in state.cable.lay.shapes],
        speed_cap=state.body.robot.max_speed, cable_holding=state.cable.holding, real=points,
        depth=getattr(frame, "depth", None))
    if state.bag_look is not None:
        state.bag_look.serve(state.bag, now)
    return state.bag.step(view)


def _page_commands(state, now):
    """Without --remote: the page's task buttons. Go back does what Enter does; Wait holds still until
    Resume, Follow or Go back. Returns the names taken."""
    taken = state.controls.take_commands()
    for cmd in taken:
        state.log.write({"time": now, "page_cmd": cmd})
        if cmd == "return":
            state.go_back.set()
        _page_command(state, cmd)
        if cmd in ("resume", "follow"):
            _release_wheels(state, now, cmd)
    return taken


def _held(state):
    """Why the page's Wait ("page"), wheels that did not turn ("wheels") or the client's side (--remote: "wait",
    "silent") hold the robot still, or None."""
    if state.page_wait:
        return "page"
    return "wheels" if state.wheels_held else "wait" if state.waiting else "silent" if state.silent else None


def _held_now(state):
    """_held, except that in BAG the client's wait (the one the bag came in) does not hold: the bag task moves."""
    held = _held(state)
    return None if state.phase == BAG and held == "wait" else held


def _report(state, now, record, force=False):
    """--remote: the status line to the client, at most every STATUS_EVERY unless forced or the end."""
    if state.remote is None:
        return
    if "pose" in record:
        state.last_record = record
    if not (force or "end" in record or now - state.status_at >= STATUS_EVERY - 1e-6):
        return
    last = state.last_record or {}
    operator, believed = last.get("operator"), last.get("believed")
    if operator is None and last.get("locked"):
        operator = last.get("operator_kept")  # not seen this cycle, but the senses still hold the track
    state.remote.send({
        "t": now,
        "phase": DONE.upper() if "end" in record else state.phase.upper(),
        "waiting": state.waiting,
        "held": _held_now(state),
        "locked": bool(last.get("locked", False)),
        "operator_m": None if operator is None or believed is None else round(math.dist(operator, believed[:2]), 3),
        "breadcrumbs": last.get("breadcrumbs", 0),
        "queue_tail": last.get("queue"),
        "at_start": state.returner is not None and state.returner.done,
        "stop": record.get("stop"),
        "ack": state.ack,
        "ack_by": None if state.ack is None else "client",  # a page press is never the ack (base_bridge reads it)
        "wheels": state.wheels_alarm,  # "not turning" or "fault 03" while held for wheels that did not turn
        "cable_deg": round(state.cable.net_deg, 1),  # the power cable's net turn since it was straightened
        "bag": None if state.bag is None else state.bag.status(),  # the running or last bag task (bag.py)
        "mode": last.get("mode"),  # the brain's mode ("follow", "search", ...): base_bridge reads "search" from it
    })
    state.status_at = now


def _least_error(state, escaped, error):
    """The heading error wheels.least may use for its deadband: none for an escape's turn, and none in the bag
    task, whose turns are bag.py's own (review 2026-11-15: the brain's unrelated turn_error, small, zeroed the
    bag's last 1.5 to 2 deg turns and used up its MAX_TURNS)."""
    return None if escaped is not None or state.phase == BAG else error


def predict(pose, v, w, seconds):
    """Where the base will be after driving (v, w) for `seconds` from pose (an arc)."""
    x, y, th = pose
    if abs(w) < 1e-9:
        return (x + v * seconds * math.cos(th), y + v * seconds * math.sin(th), th)
    end = th + w * seconds
    r = v / w
    return (x + r * (math.sin(end) - math.sin(th)), y - r * (math.cos(end) - math.cos(th)), end)


def ahead(pose, sent, now, latency):
    """The pose once the commands sent in the last `latency` seconds have played out.

    sent: (time, v, w) in time order. A command sent at t runs until the next one is sent; the
    part of that run inside (now - latency, now] has not reached the wheels yet.
    """
    for (t, v, w), end in zip(sent, [s[0] for s in sent[1:]] + [now]):
        run = min(end, now) - max(t, now - latency)
        if run > 0:
            pose = predict(pose, v, w, run)
    return pose


def _search(state, follower, pose, answer, link_alive, now):
    """(v, w) to search with, or None to keep the brain's command.

    Before the first lock the brain only stands still: turn on the spot instead, slowly, while the
    Jetson answers. In both that and the brain's own search, hold the moment a camera person is
    near the middle of the image, so enrol or the camera re-lock can take them.
    """
    if state.senses.locked or not link_alive or not state.controls.search:
        state.search_yaw = None
        return None
    never_locked = follower.last_operator_pos is None
    if not never_locked and follower.mode is not Mode.SEARCH:
        return None  # following on a short gap: the brain drives to where it saw them
    # The link hands each answer out once (about every 0.3 s); the senses keep the newest one.
    answer = getattr(state.senses, "_answer", None) or answer
    fresh = answer is not None and now - answer.t <= state.p["jetson_silence"]
    centred = fresh and any(abs(s.bearing_deg) <= SEARCH_CENTRE_DEG for s in answer.people)
    if centred and now >= state.skip_until:
        state.hold_since = now if state.hold_since is None else state.hold_since
        if now - state.hold_since < SEARCH_HOLD:
            state.search_yaw = None
            return (0.0, 0.0)
        state.skip_until = now + SEARCH_SKIP  # they never locked: turn on past them
    if not centred:
        state.hold_since = None
    if not never_locked:
        return None  # the brain's own search turns; it is only stopped above
    if not fresh:
        state.search_yaw = None
        return (0.0, 0.0)  # no recent answer: turning would be blind
    if state.search_yaw is not None:
        state.searched += abs(math.atan2(math.sin(pose[2] - state.search_yaw), math.cos(pose[2] - state.search_yaw)))
    state.search_yaw = pose[2]
    return (0.0, SEARCH_TURN) if state.searched < SEARCH_MAX else (0.0, 0.0)


def _looking_around(state):
    """True while the brain turns on the spot looking for the lost operator (hmc Follower._search after the drive)."""
    follower = state.follower
    return (state.phase == FOLLOW and getattr(follower, "mode", None) is Mode.SEARCH
            and getattr(follower, "look_since", None) is not None)


def _spot(standing, pos):
    """The place record someone at pos stands on, or None (hmc.task.Task._spot)."""
    near = [s for s in standing if math.dist(pos, s[1]) <= STAND_STILL]
    return min(near, key=lambda s: math.dist(pos, s[1])) if near else None


def _remember_crowd(state, seen, now):
    """hmc.task.Task.step's bookkeeping of who stands where (by place, not by tracker id), then the crowd."""
    trackers = state.follower.people_trackers
    for pid in seen.people:
        pos = trackers[pid].last()  # smoothed
        spot = _spot(state.standing, pos)
        if spot is None:
            state.standing.append([now, pos, now])
        else:
            spot[2] = now
    state.standing = [s for s in state.standing if now - s[2] <= STANDING_FORGET]
    standing = {pid: (spot[0], pos) for pid, pos in seen.people.items()
                if (spot := _spot(state.standing, trackers[pid].last())) is not None}
    state.queue.remember_crowd(seen, standing)


def _return(state, seen, now, dt):
    """hmc.task.Task's RETURN and JOIN_QUEUE: behind the queue while one walks ahead, else the breadcrumbs."""
    follower, queue = state.follower, state.queue
    shapes_now = follower.shapes(seen)
    if not state.joined and queue.tail(seen) is not None:
        state.joined, state.returner = True, None
        _replan(state)  # the goal is the queue now
    if state.joined and not queue.step(seen, shapes_now, follower.navigator, dt):
        state.joined = False  # left the queue for good (queue.done): on along the breadcrumbs from here
        _replan(state)
    if state.joined:
        return
    if state.returner is None:
        state.returner = Returner(state.body.robot, state.speech, follower.navigator, follower.route, state.start[2])
    state.returner.step(shapes_now, now, dt)
    if state.returner.done:
        state.phase = DONE


def _inverse(t):
    """The transform that undoes t; its yaw is not wrapped, so a turn count survives the round trip."""
    x, y, _ = motion_module._relative(t, (0.0, 0.0, 0.0))
    return (x, y, -t[2])


def _moved(t, point):
    """point moved by the transform t."""
    x, y, _ = motion_module._compose(t, (point[0], point[1], 0.0))
    return (x, y)


def _to_believed(fix, seen):
    """The senses work in the odometry frame, which never jumps; the brain gets their output moved
    into the corrected frame. RealSenses' obstacles are all Circles (shapes.ObstacleMemory)."""
    if fix == (0.0, 0.0, 0.0):
        return seen
    return dataclasses.replace(
        seen, operator=None if seen.operator is None else _moved(fix, seen.operator),
        people={pid: _moved(fix, pt) for pid, pt in seen.people.items()},
        obstacles=[Circle(*_moved(fix, (c.x, c.y)), c.r) for c in seen.obstacles])


def _replan(state):
    """Plan anew at the next step, and never take a remote answer to a request made before now."""
    state.follower.navigator._goal = None  # the Navigator plans on its next step when it has no goal
    if REMOTE is not None:
        REMOTE.forget()


def _correct(state, pose, believed, scan):
    """RETURN: match the scan against the nearest keyframe. Returns (believed pose, keyframe report)."""
    corrected, report = state.keyframes.correct(believed, scan)
    if not report["used"]:
        return believed, report
    old, state.fix = state.fix, motion_module._compose(corrected, _inverse(pose))
    # What the brain keeps in the believed frame moves with it: the people trackers' past (else a standing
    # person seems to walk) and the operator. private: hmc's MotionTracker has no call to move its samples.
    jump = motion_module._compose(state.fix, _inverse(old))
    follower = state.follower
    for tracker in list(follower.people_trackers.values()) + [follower.tracker]:
        for i, (t, point) in enumerate(tracker._samples):
            tracker._samples[i] = (t, _moved(jump, point))
    if follower.last_operator_pos is not None:
        follower.last_operator_pos = _moved(jump, follower.last_operator_pos)
    if report["shift"] > CORRECTED or 2 * TURN_REACH * math.sin(abs(report["turn"]) / 2) > CORRECTED:
        # Review Focus 2: the brain's obstacle memory is now in the wrong place, and so is the path. (The
        # senses' depth memory is in the odometry frame and comes out moved by the new fix.) The speed
        # stays: the brain ramps from it within the acceleration limits, so the wheels do not jerk.
        follower.obstacle_memory.clear()
        _replan(state)
    return corrected, report


def cycle(state, now):
    """One control cycle. Returns the log record; a record with "end" means stop the run."""
    p = state.p
    period = None if state.last_now is None else now - state.last_now
    if period is not None and period > SLOW_CYCLE:
        return _end(state, now, "slow cycle", "%.2f s since the last cycle" % period)
    dt = PERIOD if period is None else period
    state.last_now = now
    switches = state.controls.snapshot()
    if switches["end"]:
        return _end(state, now, "dashboard", "STOP pressed on the page")
    hmc_navigation.CLOSE_PLAN = switches["close_plan"]
    commands = _remote_commands(state, now) if state.remote is not None else _page_commands(state, now)
    if "stop" in commands:
        state.phase = DONE
        return _end(state, now, "remote stop", "stop from the client")

    # 1. Sensors. Broken odometry means we no longer know where we are: stop (Review Focus 5).
    try:
        if state.wheels is None:
            pose = STILL
        elif hasattr(state.wheels, "cached_odom"):
            pose = state.wheels.cached_odom()
        else:
            pose = state.wheels.read_odom()
    except (ValueError, OSError) as error:  # pyserial's SerialException is an OSError
        return _end(state, now, "odometry", error)
    if state.wheels is not None:
        state.cable.update(pose[2], now, pose)  # the odometry pose, never the keyframe-corrected one
    scan = state.lidar.latest()
    frame = state.camera.latest()
    answer = state.link.latest() if state.link is not None else None
    link_alive = state.link is not None and state.link.alive(now, p["jetson_silence"])
    sensed = scan  # what the senses read; the safety stop, the motion, the keyframes and the logs keep the real scan
    if state.virtual is not None:  # --virtual-operator: the simulated person in the scan and the camera answer
        sensed, answer, link_alive = state.virtual.apply(now, pose, scan, answer, link_alive)

    if state.wheels is not None:  # a dry run has no motion to measure
        state.measured = state.motion.update(now, pose, scan, state.sent)
        sent_at = state.history[-1][0] if state.history else now  # when the last command really went out
        state.estimates.observe(state.measured, state.sent, sent_at)
        world_decel, world_delay = state.estimates.safe_braking()  # never better than params.json
        hmc_world.MAX_DECEL, hmc_world.STOP_DELAY = world_decel, world_delay
    end = _watch_wheels(state, now, pose)
    if end is not None:
        return end
    t_read = state.clock() - now
    follower = state.follower
    # The brain's frame: the odometry, moved by the keyframe corrections of the way back.
    believed = motion_module._compose(state.fix, pose)
    if state.start is None:  # the first cycle (--remote: also the cycle of follow): the start of the way back is here
        state.started, state.start = now, believed
        follower.route = Route(believed[:2])
        state.queue = QueueFollower(state.body.robot, believed[:2], follower.people_trackers)
        state.keyframes.add(believed, scan)
    if state.phase == FOLLOW and state.enter is not None and state.enter():
        state.go_back.set()
        state.waiting = state.page_wait = False  # Enter is the operator's own go: it ends a Wait
    if state.phase == FOLLOW and (state.go_back.is_set() or (
            state.return_after is not None and now - state.started >= state.return_after)):
        state.phase = RETURN
        state.speech.say(now, hri.RETURN_TEXT)
        _replan(state)  # the goal is a breadcrumb now, not the operator
    report = None
    if state.phase == RETURN and scan:
        believed, report = _correct(state, pose, believed, scan)
    # 2. Depth obstacles (only once the camera pose is measured), the speed cap, the pose.
    depth_points = None
    if frame is not None and params.depth_gates_open(p):
        depth_points = depth_obstacles.points(frame.depth, p)
    state.cable.lay.refresh(pose, lambda q: _moved(state.fix, q), now)  # the cable's lay for this cycle's plans
    LAY["lay"] = state.cable.lay
    state.body.limit(min(state.max_speed, switches["max_speed"], safety.speed_cap(depth_points, p),
                         state.cable.lay.cap(believed[:2])))  # slow over the cable when planned across it
    # The base acts on a command command_latency after it is sent: the brain plans from where the
    # commands already on their way will have taken it, not from where it is now (Task 5).
    state.body.sync(ahead(believed, state.history, now, p["command_latency"]))

    # 3. The brain. While learning, RealSenses cannot lock by itself: enrol again whenever unlocked.
    senses = state.senses
    senses.learned = getattr(follower, "learned_count", follower.operator_look.count)
    enrol = None
    if senses.learned < LEARNED and not senses.locked and answer is not None:
        # The first enrol takes the one person in view; once a look is learned, a re-enrol
        # must look like the operator, or a stranger who steps in would be learned too.
        # fused_score also counts a continuous LiDAR track, so a look dimmed by the light still passes.
        if senses.learned and hasattr(follower, "fused_score"):
            accept = lambda look: follower.fused_score(look, continuous=True) >= LOOK_MATCH
        else:
            accept = None
        result = senses.enrol(now, sensed, answer, accept)
        if isinstance(result, str):
            enrol = result
        else:
            enrol = "locked" if not senses.enrol_how else "locked (%s)" % senses.enrol_how
        if enrol != state.enrol_said:
            print("enrol: %s" % enrol)
            state.enrol_said = enrol
    t_depth = state.clock() - now
    v_was, w_was = state.sent
    # The senses keep their state in the odometry frame (it never jumps): what moved and what is
    # furniture, the yaw history for old camera answers. Only their output goes to the believed frame.
    # The frame's depth gives the range of the people in its answer; a simulated person has none.
    seen = _to_believed(state.fix, senses.read(now, pose, v_was, w_was, dt, sensed, depth_points, answer, link_alive,
                                               follower.score, frame if state.virtual is None else None))
    t_senses = state.clock() - now
    if REMOTE is not None and REMOTE.waiting():
        follower.navigator._goal = None  # no answer to a request after forget yet: ask again this cycle
    follower.watch(seen)  # every cycle, before step: it keeps track of everyone
    if state.phase == FOLLOW:
        _remember_crowd(state, seen, now)
        crumbs = len(follower.route.points)
        follower.step(seen, dt)
        if len(follower.route.points) > crumbs:  # a new breadcrumb (a loop cut only shortens the route)
            state.keyframes.add(believed, scan)
    elif state.phase == RETURN:
        _return(state, seen, now, dt)
    elif state.phase == BAG:  # the drive up to the bag is path too: the way back drives it again
        crumbs = len(follower.route.points)
        follower.route.add(believed[:2])
        if len(follower.route.points) > crumbs:
            state.keyframes.add(believed, scan)
    LAY["lay"] = None  # plans outside a cycle (tools, other tests) never get this run's cable
    t_brain = state.clock() - now

    # 4. The command, and the stops that override the brain.
    stop = None
    if state.wheels_held:
        stop = "wheels"  # held for wheels that did not turn (_watch_wheels), until resume or follow
    if stop is None and senses.ambiguous and state.phase == FOLLOW:
        stop = "ambiguous"  # someone is right at the operator: wait, never guess who is who
    raw_lidar = scan_points(scan, p)
    # The arm, cable and bag are rigid in the base frame. Leaving them in this cloud stopped every
    # cycle on 18 Nov for an "obstacle" 0.26 m away, the outline's own corner.
    lidar_points = footprint.drop_self(raw_lidar, p, sensor="lidar")
    points = lidar_points
    if depth_points is not None:
        points = numpy.vstack([points, depth_points])
    v, w = state.body.command()
    if state.phase == DONE:
        v = w = 0.0
    turn_error = state.body.robot.turn_error
    if state.phase == BAG:  # the bag task drives, not the brain (it does not step in BAG)
        v, w, turn_error = _bag_step(state, now, pose, believed, seen, sensed, answer, frame, points)
    search = _search(state, follower, pose, answer, link_alive, now) if state.phase == FOLLOW else None
    if search is not None:
        v, w = search
    # Out of a corner (escape.py), never while held; the obstacle stop below still checks its command.
    held = (stop or ("paused" if switches["paused"] else None) or _held(state) or ("cable" if state.cable.holding else None)
            or (None if state.phase in (FOLLOW, RETURN) else state.phase))
    escaped = state.escape.command(now, pose, escape_module.goal(state, believed), held, lidar_points, points, state.cable)
    if escaped is not None:
        v, w = escaped
    # A turn with no heading of its own: run_follow's search, or the brain's look-around (Mode.SEARCH aims a quarter
    # turn left every cycle: its +90 deg is no heading, and the cable took it for a headed turn: review major 2).
    error = None if search is not None or _looking_around(state) else turn_error
    w = state.cable.steer(v, w, error, now, planned=escaped is not None)
    if state.velocity is not None and state.measured is not None and escaped is None and state.phase != BAG:
        v, w = state.velocity.command(v, w, state.measured, state.body.robot.max_speed)  # 0 stays 0
    # The smallest commands that move the wheels, here only: the stops below judge this final command. A turn made
    # larger meets the cable's limit again. An escape's turn has no heading error of the brain's.
    v, least_w = wheels_module.least(v, w, _least_error(state, escaped, error))
    w = state.cable.check(least_w, now) if abs(least_w) > abs(w) else least_w
    # SLOW_CYCLE: the next cycle may come that late (a later one ends the run), so the command runs that long.
    # BAG creeping: the locked bag's own points are left out, only they may come closer than the clearance.
    checked = state.bag.safety_points(points) if state.phase == BAG else points
    judged = (v, w)
    if stop is None and any(safety.must_stop(checked, mv, mw, p, cycle=SLOW_CYCLE) for mv, mw in state.coast.motions(v, w, now)):
        stop = "obstacle"
    if stop is None and v < 0 and not escape_module.rear_clear(points, state.cable.lay.points(pose), -v, p, -v * SLOW_CYCLE):
        stop = "obstacle"  # backing (an escape): the rear's sweep, the cable's lay included, is its obstacle stop
    if stop is None and not scan:
        stop = "no scan"  # the LiDAR went quiet: blind, so do not move
    if stop is None and switches["paused"]:
        stop = "paused"  # from the page: hold still, the senses and the brain keep running
    if stop is None:  # the same hold, from the client, its silence or the page's Wait
        if state.phase == IDLE:  # --remote only
            stop = "idle"
        elif state.phase != DONE:
            stop = _held_now(state)
    if stop == "obstacle":
        cause = safety.stop_cause(lidar_points, depth_points, judged[0], judged[1], p, cycle=SLOW_CYCLE)
    elif stop is not None:
        cause = safety.nearest_return(lidar_points, depth_points)
    else:
        cause = None
    if stop is not None:
        v = w = 0.0
        if state.velocity is not None:
            state.velocity.reset(now)  # nothing was driven: the integral must not grow while held
        state.body.robot.speed = state.body.robot.turn_speed = 0.0  # the brain ramps up again from standstill
    if state.phase == BAG:
        state.bag.after(now, stop, (v, w))
        if state.bag.bag is not None:
            state.bag_odom = tuple(state.bag.bag)
        if state.bag.done:
            _end_bag(state, now)

    # 5. Wheels, Jetson, log. A command computed from sensors read too long ago is stale: stop instead.
    cycle_time = state.clock() - now  # compute time, cycle start to send
    if cycle_time > SLOW_CYCLE:
        return _end(state, now, "slow cycle", "%.2f s from cycle start to send" % cycle_time)
    if state.wheels is not None:
        if v > 0:
            v, w = wheels_module.clamp_curvature(v, w, p)
        try:
            state.wheels.send_reverse(-v, w) if v < 0 else state.wheels.send(v, w)  # backward only in an escape
        except (ValueError, OSError) as error:
            return _end(state, now, "wheels port", error)
        state.sent = (v, w)
        state.history = [h for h in state.history if h[0] > now - HISTORY] + [(now, v, w)]  # only what reached the wheels
    state.coast.sent(v, w, now)
    if frame is not None and state.link is not None:
        state._pose_tick = getattr(state, "_pose_tick", 0) + 1
        want_pose = (state.phase == BAG and state.bag.wants_pose) or (
            state.dashboard is not None and state._pose_tick % 2 == 0)
        if want_pose:
            try:
                state.link.send(frame, pose=True)  # keypoints for bag pointing and live skeleton overlay (5 Hz)
            except TypeError:
                state.link.send(frame)
        else:
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
        "link_host": getattr(state.link, "host", None),  # which path to the Jetson is in use (cable or a backup)
        "operator": None if seen.operator is None else list(seen.operator),
        "operator_kept": None if operator is None else list(operator),  # the brain's smoothed operator
        "people": [list(pt) for pt in seen.people.values()],
        # The newest camera answer this cycle: its frame is this old, it saw this many people,
        # and answers arrive this often (link.replies_per_s, the last second).
        "cam": {
            "age": None if answer is None else round(now - answer.t, 3),
            "people": 0 if answer is None else len(answer.people),
            "replies_per_s": (state.link.replies_per_s(now)
                              if getattr(state.link, "replies_per_s", None) is not None else 0),
        },
        "obstacles": len(seen.obstacles),
        "path": len(getattr(getattr(follower, "navigator", None), "path", []) or []),  # 0 while following: no way found
        "depth_points": None if depth_points is None else len(depth_points),
        "memory": len(getattr(follower, "obstacle_memory", {}) or {}),  # the brain's remembered obstacles
        "nearest": float(numpy.hypot(points[:, 0], points[:, 1]).min()) if len(points) else None,  # m from the base centre
        "stop": stop,
        # sensor + bearing of the return that stopped us (blocking) or the nearest return on any other stop
        "stop_cause": cause,
        "self_mask": {"lidar": footprint.LAST.get("lidar"), "depth": getattr(depth_obstacles.points, "last", None)},
        "search": None if search is None else ("hold" if search == (0.0, 0.0) else "turn"),
        "why": getattr(senses, "why", None),  # the senses rule behind an ambiguous cycle or a dropped track
        "enrol": enrol,
        "cycle_time": cycle_time,  # s from cycle start to send
        "t": [round(x, 4) for x in (t_read, t_depth, t_senses, t_brain)],  # s from cycle start: sensors read, depth, senses, brain
        "planner": dict(PLANNER),
        "measured": None if state.measured is None else [round(state.measured.v, 4), round(state.measured.w, 4),
                                                          state.measured.trusted, round(state.measured.agree, 2),
                                                          round(state.measured.span, 3)],  # s the v and w average over
        "estimates": {name: [round(getattr(state.estimates, name)[0], 4), getattr(state.estimates, name)[2]]
                      for name in ("speed_gain", "turn_gain", "drift_per_m", "latency", "decel", "stop_delay")},
        "period": period,  # s since the previous cycle started
        "phase": state.phase,
        "believed": list(believed),  # the pose the brain used: odometry moved by the keyframe corrections
        "breadcrumbs": len(follower.route.points),
        "target": None if state.returner is None or state.returner.target is None else list(state.returner.target),
        "at_start": state.returner is not None and state.returner.done,  # back at the start (the status line's at_start)
        "keyframe": report,  # the match against the nearest keyframe (RETURN), or None
        "queue": state.queue.last_tail[1] if state.joined and state.queue.last_tail else None,  # tail's id
        "wheels": state.wheels_alarm,  # "not turning" or "fault 03" while held for it (_watch_wheels), else None
        "cable_deg": round(state.cable.net_deg, 1),  # the power cable's net turn (cable.py)
        "cable": state.cable.why,  # what the cable rules did to this cycle's turn, or None
        "cable_lay_m": round(state.cable.lay.length_m(), 2),  # m of cable on the floor by the estimate
        "cable_cross": state.cable.lay.crossing,  # {"since"} while planned over the cable (no way beside it)
        # the last chosen bag, odometry frame (the PC's virtual operator app reads it), and the bag task
        "bag": None if state.bag_odom is None else [round(state.bag_odom[0], 3), round(state.bag_odom[1], 3)],
        "bag_task": None if state.bag is None else dict(state.bag.status(), why=state.bag.why),
    }
    state.escape.note(record)  # "escape": what it started or ended this cycle and why, or None
    if state.virtual is not None:
        record["virtual"] = state.virtual.report  # where the simulated person was and what the senses got
        record["virtual_pointing"] = state.virtual.report.get("pointing")  # [x, y] in place of the camera's pointing
    if state.remote is not None:
        record["remote"] = {"waiting": state.waiting, "held": _held_now(state), "silent_s": round(state.remote.silent(now), 2),
                            "last_cmd": state.ack,
                            # a command now would move the robot: the client was heard lately, or the page polls
                            "supervised": state.remote.silent(now) <= REMOTE_SILENCE or state.controls.page_alive(now)}
    if state.phase == DONE:
        _stop_wheels(state)
        record["end"] = "done: back at the start"
    state.log.write(record)
    if state.dashboard is not None:
        state.dashboard.latest = record
        if link_alive and answer is not None:
            w = int(getattr(frame, "width", 0) or (frame.rgb.shape[1] if getattr(frame, "rgb", None) is not None else 320))
            h = int(getattr(frame, "height", 0) or (frame.rgb.shape[0] if getattr(frame, "rgb", None) is not None else 240))
            people = []
            for s in answer.people:
                x1, y1, x2, y2 = s.box
                kp = [[w - k[0], k[1], k[2]] for k in s.kp] if s.kp else None
                people.append({"box": [w - x2, y1, w - x1, y2], "kp": kp})
            state.dashboard.skeleton = {"w": w, "h": h, "people": people}
        else:
            state.dashboard.skeleton = {"w": None, "h": None, "people": []}
        try:  # references only, for the console's Live tab (live_feed.py): never let it stop a cycle
            state.dashboard.feed.put(frame, raw_lidar, record, now)
        except Exception:
            pass
    PLANNER.update(remote=0, local=0)
    if NO_PATH and state.n % 20 == 0:  # at most one dump every 2 s
        state.log.write({"time": now, "no_path": dict(NO_PATH)})
        NO_PATH.clear()
    _report(state, now, record, force=bool(commands))
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


def open_console(stream):
    """The raw byte stream under stream, if select can poll it (a POSIX console or pipe), else None.

    Polled in the cycle, never read by a thread: a thread blocked in a read of stdin can abort the
    interpreter at exit ("could not acquire lock for stdin")."""
    try:
        console = stream.buffer.raw
        select.select([console], [], [], 0)
    except (AttributeError, OSError, ValueError):
        return None
    return console


def enter_pressed(console):
    """True when a line (Enter) is waiting on the console; never waits. The end of input is not Enter,
    nor is a console that went away (a closed stream, a hung-up terminal): that must not end the cycle."""
    try:
        return bool(select.select([console], [], [], 0)[0]) and console.readline().endswith(b"\n")
    except (OSError, ValueError):
        return False


def console_is_tty(stream):
    """True when stream is a terminal someone can press Enter on. Under nohup or a service stdin is
    /dev/null, a pipe or missing, and input() raises EOFError (2026-11-14: a run died of it)."""
    try:
        return stream is not None and stream.isatty()
    except (AttributeError, OSError, ValueError):
        return False


def wait_for_follow(controls, sleep=time.sleep):
    """No console: Follow on the page starts the run instead of Enter. STOP on the page also ends the
    wait (the first cycle then ends the run); other buttons pressed before Follow are dropped."""
    while not (controls.snapshot()["end"] or "follow" in controls.take_commands()):
        sleep(PERIOD)


def discard_typed(console):
    """Throw away what was typed so far, so an Enter pressed while the run started does not end it."""
    while select.select([console], [], [], 0)[0]:
        if not console.read(4096):
            break  # the end of input


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
    return wheels_module.Wheels(p, poll_odom=True) if drive else None


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
    parser.add_argument("--planner", choices=["pi", "jetson", "here"], default="pi",
                        help="where paths are planned: a second process on the Pi (default), the Jetson's"
                             " plan_server.py, or in the cycle itself (slow on the Pi)")
    parser.add_argument("--velocity-loop", action="store_true",
                        help="correct the commands until the measured speed matches (Task M3; off until field-checked)")
    parser.add_argument("--return-after", type=float, metavar="N",
                        help="start the way back by itself after N s of following (a test aid; Enter does it too)")
    parser.add_argument("--dashboard", type=int, default=dashboard_module.PORT, metavar="PORT",
                        help="serve the status page on this port (0: no page)")
    parser.add_argument("--dashboard-host", default="127.0.0.1", metavar="ADDRESS",
                        help="the address the page listens on (default 127.0.0.1: only the console, which checks"
                             " its PIN, reaches the switches; 0.0.0.0 opens it to the whole network on purpose)")
    parser.add_argument("--remote", type=int, nargs="?", const=command_server.PORT, metavar="PORT",
                        help="take follow, wait, resume, return, stop and bag from a TCP client (the Jetson) on this port"
                             " (default %d) instead of Enter; the run starts standing still" % command_server.PORT)
    parser.add_argument("--remote-allow", metavar="ADDRESSES",
                        help="comma list of the addresses --remote takes commands from (default: the Jetson's"
                             " jetson_host and jetson_hosts_backup, and 127.0.0.1)")
    parser.add_argument("--virtual-operator", type=int, metavar="PORT",
                        help="follow a simulated person sent from the owner's PC (UDP on this port, virtual_operator.py)"
                             " instead of the camera's people; needs --remote")
    parser.add_argument("--virtual-allow", metavar="ADDRESSES",
                        help="comma list of the PC addresses --virtual-operator takes (127.0.0.1 and Tailscale always)")
    parser.add_argument("--cable", type=Path, default=cable_module.PATH, metavar="FILE",
                        help="the power cable's net turn, kept across runs (default %s)" % cable_module.PATH)
    args = parser.parse_args(argv)
    p = params.load()
    why = refusal(args.drive, args.max_speed, p)
    if why:
        sys.exit("REFUSED: " + why)
    if args.drive and not wheels_module.under_safe_run():
        sys.exit("REFUSED: --drive: " + wheels_module.NOT_SAFE_RUN)
    tty = console_is_tty(sys.stdin)
    if args.remote is None and not args.dashboard and not tty:
        sys.exit("REFUSED: no console, no --remote and no --dashboard: nothing could start the run")
    if args.virtual_operator is not None and args.remote is None:
        sys.exit("REFUSED: --virtual-operator needs --remote (the console's runs)")
    try:
        virtual_allowed = virtual_module.parse_allow(args.virtual_allow)
    except ValueError as error:
        sys.exit("REFUSED: --virtual-allow: %s" % error)

    for sig in STOP_SIGNALS[1:]:  # SIGINT already raises KeyboardInterrupt
        signal.signal(sig, _interrupt)
    log = Log(LOG_ROOT / time.strftime("%Y%m%d-%H%M%S"))
    print("logging to", log.folder)
    session = cam = link = wheels = server = page = remote = state = None
    global REMOTE
    try:
        if args.planner == "pi":
            server = subprocess.Popen([sys.executable, str(Path(__file__).resolve().parent / "plan_server.py"),
                                       "--host", "127.0.0.1"], stdout=subprocess.DEVNULL)
        if args.planner != "here":
            host = "127.0.0.1" if args.planner == "pi" else p["jetson_host"]
            REMOTE = remote_plan.RemotePlanner(host, PLAN_PORT, p["robot_radius"])
            REMOTE.start()
        session = lidar.LidarSession()
        cam = camera.Camera()
        cam.start()
        if not args.no_jetson:
            link = link_module.JetsonLink(link_module.hosts(p), p["jetson_port"], p["camera_hfov_deg"])  # cable first, then Wi-Fi
            link.start()
        wheels = open_wheels(args.drive, p)
        if args.remote is not None:
            # only the Jetson (cable, Wi-Fi, Tailscale) and this computer: anyone else on the LAN could drive
            allow = ([a.strip() for a in args.remote_allow.split(",") if a.strip()] if args.remote_allow
                     else link_module.hosts(p) + ["127.0.0.1"])
            remote = command_server.CommandServer(port=args.remote, allow=allow)
            remote.start()
        state = State(p, wheels, session, cam, link, log, args.max_speed, encode=link_module._encode_jpeg,
                      velocity_loop=args.velocity_loop, return_after=args.return_after, remote=remote,
                      cable_path=args.cable)
        if link is not None:
            state.bag_look = bag_module.Look(link_module.hosts(p))  # the Jetson's front camera (bag_arm.py serves it)
        if args.virtual_operator is not None:
            state.virtual = virtual_module.VirtualOperator(p, args.virtual_operator, allow=virtual_allowed,
                                                           controls=state.controls)
            state.virtual.start()
            print("virtual operator: UDP port %d from 127.0.0.1, Tailscale%s"
                  % (state.virtual.port, "".join(", " + a for a in virtual_allowed)))
        if REMOTE is not None:
            hmc_navigation.REPLAN_ON_NEW_SHAPES = True  # a plan is only a message now: ask every cycle
        if args.dashboard:
            state.dashboard = page = dashboard_module.Dashboard(state.controls, {"drive": args.drive, "seconds": args.seconds},
                                                                 port=args.dashboard, remote=args.remote is not None,
                                                                 host=args.dashboard_host)
            page.feed = live_feed.LiveFeed()  # the camera and the LiDAR on the page's /stream and /scan
            page.start()
            print("dashboard: http://%s:%d/" % (socket.gethostname() if args.dashboard_host == "0.0.0.0"
                                                else args.dashboard_host, args.dashboard))
        if remote is not None:
            print("%s. Remote: commands on port %d (follow, wait, resume, return, stop); standing still until follow."
                  % ("DRIVE" if args.drive else "DRY RUN, the wheels stay off", remote.port))
        elif not tty:
            print("%s. No console: press Follow on the dashboard to start, Go back there to send the robot back."
                  % ("DRIVE" if args.drive else "DRY RUN, the wheels stay off"))
            wait_for_follow(state.controls)
        else:
            print("%s. Operator: stand alone within 2.5 m in front of the robot, then press Enter."
                  " Press Enter again to send the robot back to the start."
                  % ("DRIVE" if args.drive else "DRY RUN, the wheels stay off"))
            input()
            state.controls.take_commands()  # buttons pressed while Enter was awaited do not count
            console = open_console(sys.stdin)
            if console is None:
                print("Enter cannot be read on this console: only --return-after starts the way back.")
            else:
                discard_typed(console)
                state.enter = lambda: enter_pressed(console)
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
            if state is not None and wheels is not None:
                with contextlib.suppress(OSError):
                    state.cable.save()  # the last turns of the run
            for device in (link, cam, REMOTE, page, remote):
                if device is not None:
                    device.stop()
            if state is not None and state.virtual is not None:
                state.virtual.stop()
            REMOTE = None  # a later main() (a test) must not plan through the stopped one
            if server is not None:
                server.terminate()
            if session is not None:
                session.close()
            log.close()
    print("log:", log.folder)


if __name__ == "__main__":
    main()
