"""Gripper-camera helper. Run on the Jetson with the arm stack up (domain 42, mount:=robot).

python3 gcam.py state                         gripper pose (user frame) + tool direction
python3 gcam.py look FWD LEFT Z [ROLL_DEG] [--pitch DEG] [--dry] [--snap NAME]
        move so the gripper origin is at (FWD, LEFT, Z base frame) with the tool pointing at --pitch
        (default -90 = straight down), wrist roll ROLL_DEG (default: keep), then save a webcam frame.
python3 gcam.py snap NAME
"""
import json
import math
import sys
import time
import urllib.request

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectoryPoint

from so100_driver.console_gate import require as console_gate
from so100_driver.ik import ARM, GraspPlanner, Kinematics, deck_box

FLOOR, DECK_Z = -0.31, -0.068
DECK = (0.18, 0.22, 0.18, 0.22)
SPEED = 0.5
WEB = "http://localhost:8082/snap.jpg"


class G(Node):
    def __init__(self):
        super().__init__("gcam")
        self.urdf, self.js = None, {}
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, "/robot_description", lambda m: setattr(self, "urdf", m.data), qos)
        self.create_subscription(JointState, "/joint_states", lambda m: self.js.update(zip(m.name, m.position)), 20)
        self.arm = ActionClient(self, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory")

    def spin_until(self, cond, timeout):
        end = time.monotonic() + timeout
        while not cond() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        return cond()

    def q_now(self):
        self.js.clear()
        self.spin_until(lambda: all(j in self.js for j in ARM), 3)
        return {j: self.js[j] for j in ARM}

    def move(self, q, t):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        pt = JointTrajectoryPoint(positions=[float(q[j]) for j in ARM])
        pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
        goal.trajectory.points.append(pt)
        self.arm.wait_for_server(timeout_sec=5)
        f = self.arm.send_goal_async(goal)
        self.spin_until(f.done, 5)
        h = f.result()
        if not h or not h.accepted:
            raise RuntimeError("trajectory rejected")
        r = h.get_result_async()
        self.spin_until(r.done, t + 10)
        if not r.done() or r.result().result.error_code != 0:
            raise RuntimeError(f'move failed: {r.result().result.error_string if r.done() else "timeout"}')


def snap(name):
    for _ in range(3):
        try:
            open(name, "wb").write(urllib.request.urlopen(WEB, timeout=8).read())
            return
        except Exception as exc:  # noqa: BLE001
            err = exc
            time.sleep(0.5)
    print("snap ERROR", err)


def describe(kin, q):
    T = kin.frames(q)["gripper"]
    p = T[:3, 3]
    tool = -T[:3, 1]
    f, l = kin.to_user(p)
    tip = kin.gripper_point(q, (0, -0.106, 0))
    return {"fwd": round(f, 4), "left": round(l, 4), "z": round(float(p[2]), 4),
            "tip_z": round(float(tip[2]), 4), "tool": [round(float(v), 3) for v in tool],
            "jaw_axis": [round(float(v), 3) for v in T[:3, 0]], "q": {k: round(v, 4) for k, v in q.items()}}


def main():
    console_gate("gcam")
    a = sys.argv[1:]
    rclpy.init()
    n = G()
    if a[0] == "snap":
        snap(a[1])
        return
    if not n.spin_until(lambda: n.urdf and all(j in n.js for j in ARM), 10):
        sys.exit("no /robot_description or /joint_states")
    from ament_index_python.packages import get_package_share_directory
    kin = Kinematics(n.urdf, None, get_package_share_directory("so100_moveit_config") + "/meshes")
    if not kin.meshes:
        sys.exit("no arm meshes")
    pl = GraspPlanner(kin, FLOOR, [deck_box(kin, *DECK, DECK_Z, FLOOR)])
    cur = n.q_now()
    if a[0] == "state":
        print(json.dumps(describe(kin, cur)))
        return
    if a[0] == "look":
        fwd, left, z = float(a[1]), float(a[2]), float(a[3])
        roll = None
        if len(a) > 4 and not a[4].startswith("--"):
            roll = math.radians(float(a[4]))
        pitch = math.radians(float(a[a.index("--pitch") + 1])) if "--pitch" in a else -math.pi / 2
        dry = "--dry" in a
        x, y = kin.from_user(fwd, left)
        q, why = None, "no IK"
        for s in kin.solve(np.array([x, y, z]), pitch, depth=0.0):
            s.pop("_branch", None)
            s["wrist_roll"] = cur["wrist_roll"] if roll is None else roll
            lo, hi = kin.limits["wrist_roll"]
            if not lo <= s["wrist_roll"] <= hi:
                why = "roll out of range"
                continue
            steps = max(2, int(max(abs(s[j] - cur[j]) for j in ARM) / 0.03))
            path = [{j: cur[j] + (s[j] - cur[j]) * t for j in ARM} for t in np.linspace(0, 1, steps)]
            if pl.hits_body(s):
                why = "pose hits body"
                continue
            if any(pl.hits_body(p) for p in path):
                why = "path hits body"
                continue
            if min(kin.gripper_point(p, (0, -0.11, 0))[2] for p in path) < FLOOR + 0.02:
                why = "floor"
                continue
            q = s
            break
        if q is None:
            sys.exit(f"cannot: {why}")
        t = max(1.5, max(abs(q[j] - cur[j]) for j in ARM) / SPEED)
        print("plan", json.dumps(describe(kin, q)), f"{t:.1f}s", flush=True)
        if dry:
            return
        n.move(q, t)
        time.sleep(1.0)
        print("now ", json.dumps(describe(kin, n.q_now())), flush=True)
        if "--snap" in a:
            snap(a[a.index("--snap") + 1])


if __name__ == "__main__":
    main()
