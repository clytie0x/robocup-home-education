"""Large-motion shake test for the real arm (same smooth profile MoveIt produces).

  ros2 run so100_driver motion_test [speed]    # full sequence, ~50 s at speed 1 (2 = twice as fast)
Every segment is a minimum-jerk move; shaking is measured per segment from /joint_states:
'reversals' = back-and-forth direction changes > 0.3 deg while the joint should move one way.
"""
import math
import threading
import time

import rclpy
import rclpy.executors
from rclpy.action import ActionClient
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectoryPoint

ARM = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll']
READY = [0, 90, -90, 0, 0]
SEQUENCE = [  # (label, target deg, seconds)
    ('rest -> ready (shoulder up, arm forward)', READY, 4),
    ('base left 60', [60, 90, -90, 0, 0], 3),
    ('base right 60 (120 deg sweep)', [-60, 90, -90, 0, 0], 5),
    ('base back to centre', READY, 3),
    ('elbow up (lower arm raised)', [0, 90, -150, 0, 0], 3),
    ('elbow DOWN (descent)', READY, 3),
    ('shoulder forward/DOWN (descent)', [0, 150, -150, 0, 0], 3),
    ('shoulder back up', READY, 3),
    ('wrist flex down 60', [0, 90, -90, -60, 0], 2),
    ('wrist flex back', READY, 2),
    ('wrist roll +90', [0, 90, -90, 0, 90], 2),
    ('wrist roll -60', [0, 90, -90, 0, -60], 3),
    ('wrist roll back', READY, 2),
    ('ready -> rest (shoulder DOWN, fold)', [0, 2, -2, 0, 0], 4),
]


NATIVE_STEPS = [  # one joint at a time, constant speed (like a servo's own single-goal move)
    ('elbow up to -90 (setup)', 'elbow_flex', -90),
    ('shoulder up to 90 = ready (setup)', 'shoulder_lift', 90),
    ('BASE left 40', 'shoulder_pan', 40),
    ('BASE back to 0', 'shoulder_pan', 0),
    ('SHOULDER forward/DOWN to 120', 'shoulder_lift', 120),
    ('SHOULDER back up to 90', 'shoulder_lift', 90),
    ('ELBOW raise to -130', 'elbow_flex', -130),
    ('ELBOW DOWN to -90', 'elbow_flex', -90),
    ('shoulder down to 2 (fold)', 'shoulder_lift', 2),
    ('elbow fold to -2', 'elbow_flex', -2),
]


def trapezoid(a, b, vmax=10.0, acc=88.0, dt=0.05):
    """Constant-speed move like the servo's own profile (deg, deg/s, deg/s^2) -> [(t, pos)]."""
    d = b - a
    dist = abs(d)
    if dist < 1e-6:
        return [(dt, b)]
    ta = vmax / acc
    if acc * ta * ta > dist:                      # triangle profile
        ta = math.sqrt(dist / acc)
        vmax = acc * ta
    tc = (dist - acc * ta * ta) / vmax
    total = 2 * ta + tc
    out, t = [], dt
    while t < total:
        if t < ta:
            x = 0.5 * acc * t * t
        elif t < ta + tc:
            x = 0.5 * acc * ta * ta + vmax * (t - ta)
        else:
            r = total - t
            x = dist - 0.5 * acc * r * r
        out.append((t, a + math.copysign(x, d)))
        t += dt
    out.append((total, b))
    return out


def min_jerk(a, b, secs, n=25):
    pts = []
    for k in range(1, n + 1):
        s = k / n
        f = 10 * s**3 - 15 * s**4 + 6 * s**5
        p = JointTrajectoryPoint()
        p.positions = [math.radians(x + (y - x) * f) for x, y in zip(a, b)]
        t = secs * s
        p.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
        pts.append(p)
    return pts


class Tester(Node):
    def __init__(self):
        super().__init__('motion_test')
        self.js, self.rows, self.status = {}, [], ''
        self.recording = False
        self.create_subscription(JointState, 'joint_states', self.on_js, 50)
        self.create_subscription(String, 'so100/status', lambda m: setattr(self, 'status', m.data), 10)
        self.arm = ActionClient(self, FollowJointTrajectory, 'arm_controller/follow_joint_trajectory')
        self.grip = ActionClient(self, GripperCommand, 'gripper_controller/gripper_cmd')

    def on_js(self, m):
        self.js = dict(zip(m.name, m.position))
        if self.recording:
            self.rows.append([math.degrees(self.js[n]) for n in ARM])

    def wait(self, fut):
        while not fut.done():
            time.sleep(0.01)
        return fut.result()

    def move(self, target, secs):
        start = [math.degrees(self.js[n]) for n in ARM]
        g = FollowJointTrajectory.Goal()
        g.trajectory.joint_names = ARM
        g.trajectory.points = min_jerk(start, target, secs)
        h = self.wait(self.arm.send_goal_async(g))
        return self.wait(h.get_result_async()).result

    def gripper(self, rad):
        g = GripperCommand.Goal()
        g.command.position = rad
        g.command.max_effort = 1.0
        h = self.wait(self.grip.send_goal_async(g))
        return self.wait(h.get_result_async()).result


def analyse(rows):
    out = []
    for j, n in enumerate(ARM):
        x = [r[j] for r in rows]
        rev, last, anchor = 0, 0, x[0]
        for v in x[1:]:
            d = v - anchor
            if abs(d) > 0.3:
                s = 1 if d > 0 else -1
                if last and s != last:
                    rev += 1
                last, anchor = s, v
        w = 8
        sm = [sum(x[max(0, i - w):i + w + 1]) / len(x[max(0, i - w):i + w + 1]) for i in range(len(x))]
        jit = max(abs(a - b) for a, b in zip(x[w:-w], sm[w:-w])) if len(x) > 2 * w + 1 else 0.0
        rms = math.sqrt(sum((a - b) ** 2 for a, b in zip(x[w:-w], sm[w:-w])) / max(1, len(x) - 2 * w))
        if max(x) - min(x) > 2 or rev:
            out.append(f'{n} {max(x) - min(x):.0f}deg rev={rev} wobble={jit:.1f} rms={rms:.2f}')
    return ', '.join(out) or 'no motion'


def run_native(t):
    print('driver-commanded copy of native_test (10 deg/s, one joint at a time)', flush=True)
    for label, joint, deg in NATIVE_STEPS:
        start = [math.degrees(t.js[n]) for n in ARM]
        k = ARM.index(joint)
        prof = trapezoid(start[k], deg)
        g = FollowJointTrajectory.Goal()
        g.trajectory.joint_names = ARM
        for tt, x in prof:
            p = JointTrajectoryPoint()
            pos = list(start)
            pos[k] = x
            p.positions = [math.radians(v) for v in pos]
            p.time_from_start = Duration(sec=int(tt), nanosec=int((tt % 1) * 1e9))
            g.trajectory.points.append(p)
        t.rows, t.recording = [], True
        h = t.wait(t.arm.send_goal_async(g))
        res = t.wait(h.get_result_async()).result
        time.sleep(0.5)
        t.recording = False
        print(f'{label:36s} code={res.error_code} | {analyse(t.rows)}', flush=True)
        if res.error_code != 0:
            print('STOPPED:', res.error_string)
            return
        time.sleep(1.0)


def main():
    rclpy.init()
    t = Tester()
    ex = rclpy.executors.SingleThreadedExecutor()
    ex.add_node(t)
    spin = threading.Thread(target=ex.spin, daemon=True)
    spin.start()
    t.arm.wait_for_server(timeout_sec=10)
    while not t.js:
        time.sleep(0.05)
    import sys
    if len(sys.argv) > 1 and sys.argv[1] == 'native':
        run_native(t)
        ex.shutdown()
        spin.join(timeout=2)
        rclpy.shutdown()
        return
    speed = float(sys.argv[1]) if len(sys.argv) > 1 else 1.0   # 2.0 = twice as fast
    print(f'speed factor {speed}', flush=True)
    heavy = len(sys.argv) > 2 and sys.argv[2] == 'heavy'   # only base / shoulder / elbow segments
    for label, target, secs in SEQUENCE:
        if heavy and label.startswith('wrist'):
            continue
        secs = secs / speed
        t.rows = []
        t.recording = True
        res = t.move(target, secs)
        time.sleep(0.5)
        t.recording = False
        err = math.degrees(max(abs(t.js[n] - math.radians(v)) for n, v in zip(ARM, target)))
        print(f'{label:42s} code={res.error_code} final_err={err:4.1f}deg | {analyse(t.rows)} | {t.status.split(" temp")[0]}',
              flush=True)
        if res.error_code != 0:
            print('STOPPED:', res.error_string)
            break
        if label.startswith('wrist roll back') and not heavy:
            for name, pos in (('gripper open', 1.5), ('gripper close', 0.0)):
                r = t.gripper(pos)
                print(f'{name:42s} reached={r.reached_goal} stalled={r.stalled} pos={math.degrees(r.position):.0f}deg', flush=True)
    ex.shutdown()
    spin.join(timeout=2)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
