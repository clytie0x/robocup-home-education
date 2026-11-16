"""Help Me Carry, arm side: pick the pointed bag from the floor and put it down at the start.

The grasp itself is the arm team's: /arm/pick_bag and /arm/place (~/so100_ws/src/API.md), so their
collision model of the robot body and their carry pose are used, and their nodes are never changed.
Ours is what they need as input:
  - where: the bag's x, y from the Pi (robot frame, LiDAR after the approach), refined by the gripper
    camera when it is calibrated (floor_pick.py --calibrate-self);
  - how high its top is: measured by the Pi from a distance ("top": Xtion depth or the front camera,
    base/docs/bag-sensing.md), else triangulated from 3 gripper camera views (gripper_camera.triangulate), else
    found by touch (z NaN: their pick_bag lowers its closed jaws until the joint loads change; PROBE_UNKNOWN),
    or, with probe=False, DEFAULT_TOP: too low is safe (the jaws close lower on the handles or the gathered
    mouth), too high closes on air (MISSED), so that fallback is just above the LiDAR plane the bag crossed.
  - which way its handles run: the Pi's "yaw" (robot frame, the long axis of its LiDAR points), turned into the
    arm frame by arm_yaw; without it their default (the part across the arm's view).
Before anything moves, /arm/home (an API task) restarts the arm team's idle timer: their auto rest
fires 120 s after their last task and would replace our camera moves (seen 2026-11-15 04:15).
The top goes to them in the arm frame of their floor: z = ARM_FLOOR_Z + top (their floor_z for mount:=robot
is the measured -0.331 itself, so no offset of ours is added); x, y go through Mount.to_arm.

/robot/command (JSON {"action", "params"}):
  pick_bag {"x", "y"[, "top", "yaw"]}  robot frame, metres, rad -> grab_success | grab_failed|CODE|message
                                 (with their grasp reading: grab_success|SUCCESS|gripper_pos=R gripper_load=L,
                                 grab_failed|CODE|message [gripper_pos=R gripper_load=L])
  drop_bag {"x", "y"}            floor point, robot frame -> drop_done | drop_failed|CODE|message
  stop                           while a job runs: /arm/stop (the arm holds where it is)
Events go out on /robot/event, the names hmc_flow.py reads.

The console's arm buttons (agent.py POST /arm) on TCP 127.0.0.1:ARM_CMD_PORT, one JSON object per line in,
one JSON line out ({"ok", "error", ...}): {"cmd": "home" | "rest" | "center" | "stop" | "state"},
{"cmd": "torque", "on": true | false}. Refused while a bag job runs, except stop and state; torque off only
while /arm/state is "resting" (the arm drops when released elsewhere). center = /arm/home, then MoveIt to
the middle of each arm joint's limits (the robot description's), then the gripper to the middle of its own.
While center or torque on holds the arm in the owner's pose, their rest_after_s is 0 (their auto rest would
fold it); the old value comes back on home, rest or a bag job. state = /arm/state, the joints (rad) and
"busy" (a bag job or an arm command runs).

  source /opt/ros/jazzy/setup.bash; source ~/so100_ws/install/setup.bash; export ROS_DOMAIN_ID=42
  python3 bag_arm.py                        # the node, and the front camera's bag measurements for the Pi
                                            # (front_camera.serve, TCP 5606)
  python3 bag_arm.py --pick 0.31 0.0        # one pick from the shell, then exit
  python3 bag_arm.py --place 0.31 0.0
"""
import argparse
import json
import math
import socketserver
import sys
import threading
import time
import xml.etree.ElementTree as ET

import numpy as np

import front_camera
import gripper_camera as gc
from floor_pick import JAW_OPEN, READY, Failure, FloorPick, Planner, Result
from so100_ik import ARM, JOINTS, Kinematics, Mount, Unreachable, load_limits

DEFAULT_TOP = 0.26        # m above the floor: just above the LiDAR plane (0.24 to 0.25) the bag crossed
PROBE_UNKNOWN = True      # top neither measured nor triangulated: z = NaN, their pick_bag finds it by touch
                          # (arm-system def69dc); False = DEFAULT_TOP as before
ARM_FLOOR_Z = -0.331      # floor in the arm frame = their launch floor_z for mount:=robot (arm.launch.py,
                          # base_z 0.305 + z_offset 0.026, base/docs/floor-pick.md Frames)
TOP_RANGE = (0.15, 0.50)  # a triangulated top outside this is a wrong match
MAX_SPREAD = 0.02         # m, RMS distance of the triangulated point from the view rays
MAX_SHIFT = 0.08          # m, the camera may move the grasp point this far from the LiDAR's bag centre
LOOK_CLEAR = 0.08         # m, the gripper stays this far above the assumed top while it looks
HOME_TIMEOUT = 60.0
PICK_TIMEOUT = 150.0      # their pick: up to 3 tries, slow carry (bag_speed 0.08)
PLACE_TIMEOUT = 120.0
ARM_CMD_PORT = 5607       # TCP on 127.0.0.1: the console's arm commands from agent.py (ArmCommands)
CENTER_HOME_TIMEOUT = 30.0  # center's steps fit in agent.py's ARM_TIMEOUT (90 s)
CENTER_MOVE_TIMEOUT = 45.0
GRIP_TIMEOUT = 10.0


def arm_yaw(yaw, mount):
    """A direction in the robot frame (rad, + left of forward) -> the arm team's yaw in the arm frame (rad from
    their +x): their forward is -y and their left is +x (API.md), so robot forward is -pi/2 there. A line, so the
    result is in [-pi/2, pi/2)."""
    a = yaw - mount.yaw - math.pi / 2
    return (a + math.pi / 2) % math.pi - math.pi / 2


def look_poses(plan, kin, target, count=3):
    """Up to `count` poses above and behind `target` (robot frame x, y, z) with the tool axis within
    22 deg of it, spread in height, reach and pitch for a stereo baseline (the tool always looks out
    along the pan direction, so poses beside the target fail the 22 deg test). The grasp point stays
    LOOK_CLEAR above the target."""
    t = np.asarray(target, float)
    cands = []
    for dx in (-0.12, -0.08, -0.04):
        for dy in (-0.07, 0.0, 0.07):
            for grip_z in (t[2] + 0.10, t[2] + 0.14, t[2] + 0.18):
                for pitch_d in (-75, -60, -45):
                    g = np.array([t[0] + dx, t[1] + dy, grip_z])
                    try:
                        q = kin.ik(g, math.radians(pitch_d), jaw=JAW_OPEN)
                        plan.check(q, JAW_OPEN, False)
                    except (Unreachable, Failure):
                        continue
                    if kin.grasp_point(q)[2] < t[2] + LOOK_CLEAR:
                        continue
                    v = t - g
                    if float(kin.tool_dir(q) @ v / np.linalg.norm(v)) > math.cos(math.radians(22)):
                        cands.append(((dy / 0.035, grip_z / 0.04, pitch_d / 15.0, dx / 0.04), q))
    return Planner._spread(cands, count)


class BagArm:
    """Pick and place jobs over a transport with arm_task / arm_state / moving / stop (floor_pick.RosArm)."""

    def __init__(self, kin, transport, camera=None, detector=None, log=print, event=None, probe=PROBE_UNKNOWN):
        self.k, self.t = kin, transport
        self.probe = probe
        self.fp = FloorPick(kin, transport, Planner(kin), log=log)
        self.camera, self.detector = camera, detector or gc.BagHandleDetector()
        self.log = log
        self.event = event or getattr(transport, "event", lambda _e: None)
        self.reading = {}          # the last API result's grasp reading (gripper_pos rad, gripper_load %)

    def _ready(self):
        st = self.t.arm_state()
        if st is None:
            raise Failure("FAILED", "no /arm/state from the arm team's nodes")
        if st.get("holding"):
            raise Failure("HOLDING", "the arm already holds something")
        if st.get("state") == "busy":
            raise Failure("BUSY", "the arm team's node is busy (%s)" % (st.get("task") or "?"))
        moving = getattr(self.t, "moving", None)
        if moving is not None and moving():
            raise Failure("BUSY", "someone else is moving the arm")

    def _task(self, name, timeout, **goal):
        res = self.t.arm_task(name, timeout, **goal)
        ok, code, msg = res[:3]
        self.reading = res[3] if len(res) > 3 else {}     # their grasp reading (pick_bag), if they send one
        if not ok:
            raise Failure(code or "FAILED", "/arm/%s: %s" % (name, msg))
        return msg

    def measure_top(self, x, y):
        """Triangulated bag point (robot frame) seen by the detector from look_poses, or None (logged)."""
        if self.camera is None:
            self.log("no gripper camera calibration: top not measured")
            return None
        target = np.array([x, y, DEFAULT_TOP + 0.05])
        poses = look_poses(self.fp.plan, self.k, target)
        if len(poses) < 2:
            self.log("fewer than 2 look poses reach the bag at (%.2f, %.2f)" % (x, y))
            return None
        views = []
        for i, q in enumerate(poses):
            self.fp._goto("look at the bag %d" % i, q, JAW_OPEN)
            self.t.sleep(0.6)
            det = self.detector.detect(self.t.snapshot())
            if det is None:
                self.log("bag not seen from look %d" % i)
                continue
            views.append((self.fp._current(), (det.u, det.v)))
        if len(views) < 2:
            self.log("bag seen from %d look(s): top not measured" % len(views))
            return None
        p, spread = gc.triangulate(self.k, self.camera, views)
        self.log("bag point (%.3f, %.3f, %.3f), spread %.3f m from %d views" % (p[0], p[1], p[2], spread, len(views)))
        if spread > MAX_SPREAD or not TOP_RANGE[0] <= p[2] <= TOP_RANGE[1] \
                or math.hypot(p[0] - x, p[1] - y) > MAX_SHIFT:
            self.log("bag point rejected (spread, height or shift out of range)")
            return None
        return p

    def _reading(self):
        """'gripper_pos=R gripper_load=L' from the last pick_bag result, '' without a finite reading."""
        pos, load = self.reading.get("gripper_pos"), self.reading.get("gripper_load")
        if pos is None or load is None or not (math.isfinite(pos) and math.isfinite(load)):
            return ""
        return "gripper_pos=%.3f gripper_load=%.1f" % (pos, load)

    def pick(self, x, y, top=None, yaw=None):
        """Pick the bag at robot (x, y); top (m above the floor) and yaw (robot frame rad, the handles' line)
        when the Pi measured them. Sends grab_success or grab_failed|CODE|message (with their grasp
        reading when they send one, for the flow log)."""
        self.reading = {}
        try:
            self._ready()
            self._task("home", HOME_TIMEOUT)
            where = np.array([x, y], float)
            if top is None:
                p = self.measure_top(x, y)
                if p is not None:
                    where, top = p[:2], float(p[2])
                    self.fp._goto("back to ready", READY, JAW_OPEN)
                elif self.probe:
                    top = math.nan  # their pick_bag lowers its closed jaws until they touch the top
                else:
                    top = DEFAULT_TOP
            grasp = np.array([*self.k.mount.to_arm([where[0], where[1], 0.0])[:2], ARM_FLOOR_Z + top])
            self.log("pick_bag at robot (%.3f, %.3f), top %s -> arm (%.3f, %.3f, %s)"
                     % (where[0], where[1], "unknown (found by touch)" if math.isnan(top) else "%.3f m" % top,
                        grasp[0], grasp[1], "NaN" if math.isnan(grasp[2]) else "%.3f" % grasp[2]))
            ayaw = None if yaw is None else arm_yaw(yaw, self.k.mount)
            if ayaw is not None:
                self.log("yaw %.1f deg robot -> %.1f deg arm" % (math.degrees(yaw), math.degrees(ayaw)))
            msg = self._task("pick_bag", PICK_TIMEOUT, grasp=tuple(grasp), yaw=ayaw)
        except Failure as exc:
            self.log("FAILED %s: %s" % (exc.code, exc.message))
            note = self._reading()
            self.event("grab_failed|%s|%s%s" % (exc.code, exc.message.replace("|", "/"),
                                                " [%s]" % note if note else ""))
            return Result(False, exc.code, exc.message)
        note = self._reading()
        self.event("grab_success|SUCCESS|%s" % note if note else "grab_success")
        return Result(True, "SUCCESS", msg)

    def place(self, x, y):
        """Put the held bag down at robot floor point (x, y). Sends drop_done or drop_failed|CODE|message."""
        try:
            target = self.k.mount.to_arm([x, y, 0.0])
            self.log("place at robot (%.3f, %.3f) -> arm (%.3f, %.3f)" % (x, y, target[0], target[1]))
            msg = self._task("place", PLACE_TIMEOUT, target=(target[0], target[1]))
        except Failure as exc:
            self.log("FAILED %s: %s" % (exc.code, exc.message))
            self.event("drop_failed|%s|%s" % (exc.code, exc.message.replace("|", "/")))
            return Result(False, exc.code, exc.message)
        self.event("drop_done")
        return Result(True, "SUCCESS", msg)


class Jobs:
    """/robot/command -> one BagArm job at a time on a thread (pure: tests drive it without ROS)."""

    def __init__(self, bag_arm, log=print, before=None):
        self.arm, self.log = bag_arm, log
        self.before = before or (lambda: None)  # a job starts: ArmCommands.release
        self.thread = None
        self.console = None  # the console's arm command running now (ArmCommands), or None
        self.lock = threading.Lock()

    def busy(self):
        return self.thread is not None and self.thread.is_alive()

    def claim(self, what):
        """The console takes the arm for `what` (None gives it back). False while a job or another command runs."""
        with self.lock:
            if what is not None and (self.busy() or self.console is not None):
                return False
            self.console = what
            return True

    def on_command(self, text):
        try:
            cmd = json.loads(text)
        except ValueError:
            return
        action, p = cmd.get("action"), cmd.get("params") or {}
        if action == "stop":
            if self.busy():
                self.log("stop: /arm/stop")
                self.arm.t.stop()
            return
        if action not in ("pick_bag", "drop_bag"):
            return
        fail = "grab_failed" if action == "pick_bag" else "drop_failed"
        try:
            x, y = float(p["x"]), float(p["y"])
            top = float(p["top"]) if p.get("top") is not None else None
            yaw = float(p["yaw"]) if p.get("yaw") is not None else None
        except (KeyError, TypeError, ValueError):
            self.arm.event("%s|FAILED|%s needs x and y" % (fail, action))
            return
        if not (math.isfinite(x) and math.isfinite(y)):
            self.arm.event("%s|FAILED|%s needs finite x and y" % (fail, action))
            return
        run = (lambda: self.arm.pick(x, y, top, yaw)) if action == "pick_bag" else (lambda: self.arm.place(x, y))

        def job():
            try:
                self.before()
            except Exception as exc:  # noqa: BLE001 (the bag job goes on without the console's auto rest)
                self.log("rest_after_s not put back: %s" % exc)
            run()
        with self.lock:
            if self.console is not None:
                self.arm.event("%s|BUSY|the console is moving the arm (%s)" % (fail, self.console))
                return
            if self.busy():
                self.arm.event("%s|BUSY|another arm job is running" % fail)
                return
            self.thread = threading.Thread(target=job, daemon=True)
            self.thread.start()


def urdf_limits(urdf_xml):
    """{joint: (lower, upper)} rad from a robot description (the driver's urdf_limits, driver_node.py)."""
    out = {}
    for j in ET.fromstring(urdf_xml).findall("joint"):
        lim = j.find("limit")
        if j.get("name") in JOINTS and lim is not None:
            out[j.get("name")] = (float(lim.get("lower")), float(lim.get("upper")))
    return out


def mid_pose(limits):
    """The middle of each joint's (lower, upper) limits, the arm joints and the gripper (rad)."""
    missing = [n for n in JOINTS if n not in limits]
    if missing:
        raise ValueError("no limits for %s in the robot description" % ", ".join(missing))
    return {n: (limits[n][0] + limits[n][1]) / 2 for n in JOINTS}


def _finite(d):
    """d with NaN and inf as None (JSON has neither); None stays None."""
    if d is None:
        return None
    return {k: None if isinstance(v, float) and not math.isfinite(v) else v for k, v in d.items()}


class ArmCommands:
    """The console's arm commands (the module docstring) over an api with task / stop / torque / state / joints /
    limits / move / grip / get_param / set_param (RosConsoleArm; pure: tests use a fake)."""

    def __init__(self, jobs, api, log=print):
        self.jobs, self.api, self.log = jobs, api, log
        self.saved_rest = None  # their rest_after_s while the owner's pose holds it at 0
        self.stopped = False

    def handle(self, text):
        try:
            cmd = json.loads(text)
        except ValueError:
            return {"ok": False, "error": "not JSON"}
        name = cmd.get("cmd") if isinstance(cmd, dict) else None
        if name == "state":
            return {"ok": True, "error": None, "busy": self.jobs.busy() or self.jobs.console is not None,
                    "state": _finite(self.api.state()), "joints": _finite(self.api.joints())}
        if name == "stop":
            self.stopped = True
            self.log("console: stop")
            self.api.stop()
            return {"ok": True, "error": None}
        moves = {"home": self._task, "rest": self._task, "center": self._center, "torque": self._torque}
        if name not in moves:
            return {"ok": False, "error": "cmd must be one of home, rest, center, torque, stop, state"}
        if name == "torque" and not isinstance(cmd.get("on"), bool):
            return {"ok": False, "error": "torque needs \"on\": true or false"}
        if not self.jobs.claim(name):
            return {"ok": False, "error": "a bag job is running" if self.jobs.busy()
                    else "the arm is busy with %s" % (self.jobs.console or "another command")}
        self.stopped = False
        self.log("console: %s" % name)
        try:
            return moves[name](name, cmd)
        except Exception as exc:  # noqa: BLE001 (an answer for the console, never a dead server)
            return {"ok": False, "error": "%s: %s" % (name, exc)}
        finally:
            self.jobs.claim(None)

    def hold(self):
        """The owner's pose stays: their auto rest off, the old value kept for release."""
        if self.saved_rest is None:
            self.saved_rest = float(self.api.get_param("rest_after_s"))
        self.api.set_param("rest_after_s", 0.0)

    def release(self):
        """Their auto rest back as it was before hold (nothing when not held)."""
        if self.saved_rest is not None:
            self.api.set_param("rest_after_s", self.saved_rest)
            self.saved_rest = None

    @staticmethod
    def _answer(ok, code, msg, **extra):
        return dict({"ok": bool(ok), "error": None if ok else "%s: %s" % (code, msg), "code": code,
                     "message": msg}, **extra)

    def _task(self, name, _cmd):
        self.release()
        ok, code, msg = self.api.task(name, HOME_TIMEOUT)[:3]
        return self._answer(ok, code, msg)

    def _torque(self, _name, cmd):
        on = cmd["on"]
        if not on:
            st = (self.api.state() or {}).get("state")
            if st != "resting":
                return {"ok": False, "error": "torque off only at rest (/arm/state is %s): the arm drops when "
                                              "released elsewhere; send rest first" % (st or "unknown")}
        ok, msg = self.api.torque(on)
        if ok and on:
            self.hold()
        return self._answer(ok, "SUCCESS" if ok else "FAILED", msg)

    def _center(self, _name, _cmd):
        self.release()
        ok, code, msg = self.api.task("home", CENTER_HOME_TIMEOUT)[:3]
        if not ok:
            return self._answer(ok, code, "/arm/home: %s" % msg)
        q = mid_pose(self.api.limits())
        steps = (("MoveIt to the middle of the limits",
                  lambda: self.api.move({n: q[n] for n in ARM}, CENTER_MOVE_TIMEOUT)),
                 ("gripper to the middle", lambda: self.api.grip(q["gripper"], GRIP_TIMEOUT)))
        for what, step in steps:
            if self.stopped:
                return self._answer(False, "STOPPED", "stopped before: %s" % what)
            ok, msg = step()
            if not ok:
                return self._answer(False, "FAILED", "%s: %s" % (what, msg))
        self.hold()
        return self._answer(True, "SUCCESS", "at the middle of the limits", pose=q)


def serve_commands(commands, port=ARM_CMD_PORT, host="127.0.0.1"):
    """ArmCommands on TCP host:port, a thread per connection (stop and state answer during a center). The server."""
    class Handler(socketserver.StreamRequestHandler):
        def handle(self):
            while True:
                line = self.rfile.readline(65536)
                if not line.strip():
                    return
                self.wfile.write((json.dumps(commands.handle(line)) + "\n").encode())

    class Server(socketserver.ThreadingTCPServer):
        allow_reuse_address = True
        daemon_threads = True
    server = Server((host, port), Handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    return server


class RosConsoleArm:
    """ArmCommands' api on bag_arm's node beside floor_pick.RosArm (t): /arm/rest, /arm/torque, all of
    /arm/state, MoveIt's move_action, the gripper and so100_pick_place's parameters. ROS only."""

    def __init__(self, t):
        import rclpy.parameter  # noqa: PLC0415
        from moveit_msgs.action import MoveGroup  # noqa: PLC0415
        from rcl_interfaces.srv import GetParameters, SetParameters  # noqa: PLC0415
        from rclpy.action import ActionClient  # noqa: PLC0415
        from so100_interfaces.action import Rest  # noqa: PLC0415
        from so100_interfaces.msg import ArmState  # noqa: PLC0415
        from std_srvs.srv import SetBool  # noqa: PLC0415
        self.t, n = t, t.node
        self.param, self.MoveGroup, self.Rest, self.SetBool = rclpy.parameter, MoveGroup, Rest, SetBool
        self.GetParameters, self.SetParameters = GetParameters, SetParameters
        self._rest = ActionClient(n, Rest, "/arm/rest")
        self._move = ActionClient(n, MoveGroup, "move_action")
        self._torque = n.create_client(SetBool, "/arm/torque")
        self._get = n.create_client(GetParameters, "/so100_pick_place/get_parameters")
        self._set = n.create_client(SetParameters, "/so100_pick_place/set_parameters")
        self._state, self._state_t = None, 0.0
        self._goal = None
        n.create_subscription(ArmState, "/arm/state", self._on_state, 10)

    def _on_state(self, msg):
        self._state = {f: getattr(msg, f) for f in msg.get_fields_and_field_types() if f != "header"}
        self._state_t = time.monotonic()

    def state(self, wait=3.0):
        end = time.monotonic() + wait
        while time.monotonic() < end:
            if self._state is not None and time.monotonic() - self._state_t < 2.0:   # 2 Hz topic
                return dict(self._state)
            time.sleep(0.05)
        return None

    def joints(self):
        return self.t.joints()

    def _action(self, client, goal, timeout, what):
        """(result, None) or (None, why)."""
        if not client.wait_for_server(timeout_sec=5.0):
            return None, "%s not available" % what
        handle, why = self.t._wait(client.send_goal_async(goal), 5.0)
        if handle is None or not handle.accepted:
            return None, "%s goal not accepted (%s)" % (what, why or "rejected")
        self._goal = handle
        res, _ = self.t._wait(handle.get_result_async(), timeout)
        self._goal = None
        if res is None:
            handle.cancel_goal_async()
            return None, "%s took longer than %.0f s" % (what, timeout)
        return res.result, None

    def task(self, name, timeout):
        if name == "home":
            return self.t.arm_task("home", timeout)
        r, why = self._action(self._rest, self.Rest.Goal(), timeout, "/arm/rest")
        return (False, "FAILED", why) if r is None else (bool(r.success), r.error_code, r.message)

    def move(self, q, timeout):
        """MoveIt to joint pose q (rad): pick_place_node.moveit_to's request, slower."""
        from moveit_msgs.msg import Constraints, JointConstraint  # noqa: PLC0415
        goal = self.MoveGroup.Goal()
        r = goal.request
        r.group_name, r.pipeline_id = "arm", "ompl"
        r.num_planning_attempts, r.allowed_planning_time = 3, 5.0
        r.max_velocity_scaling_factor = r.max_acceleration_scaling_factor = 0.3
        r.start_state.is_diff = True
        c = Constraints()
        for n in ARM:
            c.joint_constraints.append(JointConstraint(joint_name=n, position=float(q[n]), tolerance_above=0.005,
                                                       tolerance_below=0.005, weight=1.0))
        r.goal_constraints = [c]
        goal.planning_options.plan_only = False
        res, why = self._action(self._move, goal, timeout, "MoveIt move_action")
        if res is None:
            return False, why
        code = res.error_code.val
        return code == 1, "MoveIt code %d" % code

    def grip(self, position, timeout):
        r = self.t.grip(position, timeout)
        return r["ok"], ("position %.3f rad" % r["position"]) if r["ok"] else "no gripper result"

    def stop(self):
        if self._goal is not None:
            self._goal.cancel_goal_async()
        self.t.stop()

    def _call(self, client, req, what):
        if not client.wait_for_service(timeout_sec=3.0):
            raise RuntimeError("%s not available" % what)
        r, _ = self.t._wait(client.call_async(req), 5.0)
        if r is None:
            raise RuntimeError("no answer from %s" % what)
        return r

    def torque(self, on):
        try:
            r = self._call(self._torque, self.SetBool.Request(data=bool(on)), "/arm/torque")
        except RuntimeError as exc:
            return False, str(exc)
        return bool(r.success), r.message

    def get_param(self, name):
        r = self._call(self._get, self.GetParameters.Request(names=[name]), "/so100_pick_place parameters")
        return self.param.parameter_value_to_python(r.values[0])

    def set_param(self, name, value):
        msg = self.param.Parameter(name, self.param.Parameter.Type.DOUBLE, float(value)).to_parameter_msg()
        r = self._call(self._set, self.SetParameters.Request(parameters=[msg]), "/so100_pick_place parameters")
        if not r.results[0].successful:
            raise RuntimeError("%s not set: %s" % (name, r.results[0].reason))

    def limits(self):
        return urdf_limits(self.get_param("robot_description"))


def parse(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--pick", nargs=2, type=float, metavar=("X", "Y"))
    ap.add_argument("--place", nargs=2, type=float, metavar=("X", "Y"))
    ap.add_argument("--top", type=float, help="pick: the bag top above the floor (m), skip the camera")
    ap.add_argument("--camera-file", default=gc.DEFAULT_FILE)
    ap.add_argument("--snap-url", default="http://127.0.0.1:8082/snap.jpg")
    ap.add_argument("--front-camera-file", default=front_camera.DEFAULT_FILE)
    return ap.parse_args(argv)


def main(argv=None):
    a = parse(argv)
    from floor_pick import RosArm  # noqa: PLC0415 (ROS only here)
    kin = Kinematics(Mount(), load_limits())
    try:
        camera = gc.CameraModel.load(a.camera_file)
    except (OSError, ValueError, KeyError):
        camera = None
        print("no gripper camera calibration at %s: the bag top is not measured" % a.camera_file)
    t = RosArm(a.snap_url)
    arm = BagArm(kin, t, camera)
    try:
        if a.pick or a.place:
            res = arm.pick(*a.pick, a.top) if a.pick else arm.place(*a.place)
            print("result: %s %s %s" % ("OK" if res.ok else "FAILED", res.code, res.message))
            return 0 if res.ok else 1
        from std_msgs.msg import String  # noqa: PLC0415
        try:
            front_camera.serve(front_camera.PORT, a.front_camera_file, a.snap_url)
            print("bag_arm: front camera bag measurements on TCP %d" % front_camera.PORT)
        except OSError as exc:  # another copy serves it: the arm jobs still run
            print("bag_arm: front camera service not started: %s" % exc)
        jobs = Jobs(arm, before=lambda: commands.release())
        commands = ArmCommands(jobs, RosConsoleArm(t))
        try:
            serve_commands(commands)
            print("bag_arm: the console's arm commands on TCP 127.0.0.1:%d" % ARM_CMD_PORT)
        except OSError as exc:  # the bag jobs still run
            print("bag_arm: arm commands not started: %s" % exc)
        t.node.create_subscription(String, "/robot/command", lambda m: jobs.on_command(m.data), 10)
        print("bag_arm: waiting for pick_bag / drop_bag on /robot/command")
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        return 130
    finally:
        t.close()


if __name__ == "__main__":
    sys.exit(main())
