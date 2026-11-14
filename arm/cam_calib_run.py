"""Move the gripper (with a colour marker on it) through poses in front of the robot and, at each one,
save the gripper pose (from the measured joints) plus a frame from the webcam and the Xtion.

Run on the Jetson with the arm stack up (ROS_DOMAIN_ID 42, mount:=robot) and after `arm_cli home`:
    python3 cam_calib_run.py OUT_DIR [--dry]
"""
import json
import math
import os
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

from so100_driver.ik import ARM, GraspPlanner, Kinematics, deck_box

FLOOR, DECK_Z = -0.31, -0.068
DECK = (0.18, 0.22, 0.18, 0.22)
SPEED = 0.6          # rad/s of the fastest joint
WEB = 'http://localhost:8082/snap.jpg'
XTION = 'http://192.168.31.21:8081/stream'


def grab_xtion():
    r = urllib.request.urlopen(XTION, timeout=8)
    d, n = b'', 0
    while True:
        d += r.read(4096)
        a = d.find(b'\xff\xd8')
        b = d.find(b'\xff\xd9', a + 2) if a >= 0 else -1
        if b >= 0:
            n += 1
            if n == 2:                 # the first frame in the buffer may be old
                return d[a:b + 2]
            d = d[b + 2:]


class Calib(Node):
    def __init__(self):
        super().__init__('cam_calib_run')
        self.urdf, self.js = None, {}
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/robot_description', lambda m: setattr(self, 'urdf', m.data), qos)
        self.create_subscription(JointState, '/joint_states',
                                 lambda m: self.js.update(zip(m.name, m.position)), 20)
        self.arm = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')

    def spin_until(self, cond, timeout):
        end = time.monotonic() + timeout
        while not cond() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        return cond()

    def q_now(self):
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
            raise RuntimeError('trajectory rejected')
        r = h.get_result_async()
        self.spin_until(r.done, t + 10)
        if not r.done() or r.result().result.error_code != 0:
            raise RuntimeError(f'move failed: {r.result().result.error_string if r.done() else "timeout"}')


def targets():
    """(fwd, left, z, pitch_deg) in a snake order, near to far."""
    out = []
    for i, fwd in enumerate((0.22, 0.27, 0.32)):
        lefts = (-0.12, -0.04, 0.04, 0.12)
        for j, left in enumerate(lefts if i % 2 == 0 else lefts[::-1]):
            for k, z in enumerate((-0.08, 0.0, 0.08) if j % 2 == 0 else (0.08, 0.0, -0.08)):
                out.append((fwd, left, z, (-30, -60)[(i + j + k) % 2]))
    return out


def main():
    out_dir = sys.argv[1]
    dry = '--dry' in sys.argv
    os.makedirs(out_dir, exist_ok=True)
    rclpy.init()
    n = Calib()
    if not n.spin_until(lambda: n.urdf and all(j in n.js for j in ARM), 10):
        sys.exit('no /robot_description or /joint_states')
    from ament_index_python.packages import get_package_share_directory
    kin = Kinematics(n.urdf, None, get_package_share_directory('so100_moveit_config') + '/meshes')
    if not kin.meshes:
        sys.exit('no arm meshes: body collision check would be empty')
    pl = GraspPlanner(kin, FLOOR, [deck_box(kin, *DECK, DECK_Z, FLOOR)])
    cur = n.q_now()
    log = []
    for idx, (fwd, left, z, pitch) in enumerate(targets()):
        x, y = kin.from_user(fwd, left)
        sols = [q for q in kin.solve(np.array([x, y, z]), math.radians(pitch), depth=0.0)]
        q, why = None, 'no IK'
        for s in sols:
            s.pop('_branch', None)
            s['wrist_roll'] = cur['wrist_roll']
            steps = max(2, int(max(abs(s[j] - cur[j]) for j in ARM) / 0.03))
            path = [{j: cur[j] + (s[j] - cur[j]) * t for j in ARM} for t in np.linspace(0, 1, steps)]
            if pl.hits_body(s):
                why = 'pose hits body'
                continue
            if any(pl.hits_body(p) for p in path):
                why = 'path hits body'
                continue
            if min(kin.gripper_point(p, (0, -0.11, 0))[2] for p in path) < FLOOR + 0.03:
                why = 'floor'
                continue
            q = s
            break
        if q is None:
            print(f'{idx:2d} skip fwd {fwd} left {left} z {z} pitch {pitch}: {why}', flush=True)
            continue
        t = max(1.5, max(abs(q[j] - cur[j]) for j in ARM) / SPEED)
        print(f'{idx:2d} go   fwd {fwd} left {left} z {z} pitch {pitch} ({t:.1f}s)', flush=True)
        if dry:
            cur = q
            continue
        n.move(q, t)
        time.sleep(1.0)
        n.js.clear()
        n.spin_until(lambda: all(j in n.js for j in ARM), 2)
        cur = n.q_now()
        rec = {'i': idx, 'target': [fwd, left, z, pitch], 'q': cur,
               'T_gripper': kin.frames(cur)['gripper'].tolist(),
               'user_of_base': {'pan_point': kin.pan_point.tolist(), 'heading0': kin.heading0}}
        for cam, get in (('web', lambda: urllib.request.urlopen(WEB, timeout=8).read()), ('xtion', grab_xtion)):
            try:
                open(os.path.join(out_dir, f'{cam}_{idx:02d}.jpg'), 'wb').write(get())
            except Exception as exc:  # noqa: BLE001
                print('  ', cam, 'ERROR', exc, flush=True)
        log.append(rec)
        json.dump(log, open(os.path.join(out_dir, 'poses.json'), 'w'), indent=1)
    print('done', len(log), 'poses', flush=True)


if __name__ == '__main__':
    main()
