"""Small Python client for the SO-100 arm API (/arm/*). Use it from any ROS 2 Python node:

    from so100_driver.arm_client import ArmClient
    arm = ArmClient(node)                                     # node = your rclpy Node (must be spinning)
    ok, code, msg = arm.pick(0.0, -0.245, size=(0.03, 0.03, 0.05))   # base frame, metres
    ok, code, msg = arm.place(-0.10, -0.245)
    ok, code, msg = arm.pick_place((0.0, -0.245), (-0.10, -0.245), size=(0.03, 0.03, 0.05))
    arm.home(); arm.rest(); arm.stop(); arm.torque(False); print(arm.state)

Positions can also be a geometry_msgs/PoseStamped in any TF frame (e.g. from the camera):
    arm.pick(pose_stamped, size=(...))
`user_xy(forward, left)` converts tape-measure numbers (from the arm's pan axis) to base-frame x, y.

Every call blocks until the arm finishes (run your node with a MultiThreadedExecutor or spin in a thread)
and returns (success, error_code, message). Pass `wait=False` to get the goal future instead.
Command-line test:  ros2 run so100_driver arm_cli pick 0.20 0.0 --size 0.03 0.03 0.05   (forward, left)
"""
import math
import sys
import threading
import time

from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from std_srvs.srv import SetBool, Trigger

from so100_interfaces.action import Home, Pick, PickPlace, Place, Rest
from so100_interfaces.msg import ArmState

PAN_AXIS = (0.0, -0.0452)      # arm pan axis in the base frame (URDF); "forward" = base -y


def user_xy(forward, left):
    """Tape-measure coordinates (metres from the pan axis: forward, left) -> base-frame (x, y)."""
    return PAN_AXIS[0] + left, PAN_AXIS[1] - forward


def make_pose(where, yaw=None, frame='base'):
    if isinstance(where, PoseStamped):
        return where
    x, y = where
    ps = PoseStamped()
    ps.header.frame_id = frame
    ps.pose.position.x, ps.pose.position.y = float(x), float(y)
    if yaw is None:              # all-zero quaternion = "unknown, assume it faces the arm"
        ps.pose.orientation.w = 0.0
    else:
        ps.pose.orientation.z, ps.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    return ps


class ArmClient:
    def __init__(self, node):
        self.node = node
        cb = ReentrantCallbackGroup()
        self._actions = {name: ActionClient(node, kind, f'/arm/{name}', callback_group=cb)
                         for name, kind in (('pick', Pick), ('place', Place), ('pick_place', PickPlace),
                                            ('home', Home), ('rest', Rest))}
        self._stop = node.create_client(Trigger, '/arm/stop', callback_group=cb)
        self._torque = node.create_client(SetBool, '/arm/torque', callback_group=cb)
        self.state = None
        node.create_subscription(ArmState, '/arm/state', self._on_state, 10, callback_group=cb)
        self.feedback = None     # last progress message (step, step_index, step_count)

    def _on_state(self, msg):
        self.state = msg

    @staticmethod
    def _wait(fut, timeout):
        end = time.monotonic() + timeout
        while not fut.done():
            if time.monotonic() > end:
                return None
            time.sleep(0.01)
        return fut.result()

    def _run(self, name, goal, wait=True, timeout=120.0):
        client = self._actions[name]
        if not client.wait_for_server(timeout_sec=5.0):
            return False, 'FAILED', f'/arm/{name} is not running'
        fut = client.send_goal_async(goal, feedback_callback=lambda f: setattr(self, 'feedback', f.feedback))
        if not wait:
            return fut
        handle = self._wait(fut, 10.0)
        if handle is None or not handle.accepted:
            return False, 'FAILED', 'goal rejected'
        res = self._wait(handle.get_result_async(), timeout)
        if res is None:
            handle.cancel_goal_async()
            return False, 'FAILED', 'timed out'
        r = res.result
        return r.success, r.error_code, r.message

    # ----------------------------------------------------------------------------- tasks
    def pick(self, where, size, yaw=None, grasp_mode='auto', **kw):
        g = Pick.Goal(object_pose=make_pose(where, yaw), object_size=[float(v) for v in size], grasp_mode=grasp_mode)
        return self._run('pick', g, **kw)

    def place(self, where, yaw=None, **kw):
        return self._run('place', Place.Goal(target_pose=make_pose(where, yaw)), **kw)

    def pick_place(self, obj, target, size, obj_yaw=None, target_yaw=None, grasp_mode='auto', **kw):
        g = PickPlace.Goal(object_pose=make_pose(obj, obj_yaw), object_size=[float(v) for v in size],
                           target_pose=make_pose(target, target_yaw), grasp_mode=grasp_mode)
        return self._run('pick_place', g, **kw)

    def home(self, **kw):
        return self._run('home', Home.Goal(), **kw)

    def rest(self, **kw):
        """Close the jaws and fold into the rest pose (refused with HOLDING while holding)."""
        return self._run('rest', Rest.Goal(), **kw)

    # ---------------------------------------------------------------------------- services
    def _call(self, client, req):
        if not client.wait_for_service(timeout_sec=3.0):
            return False, 'arm API not running'
        r = self._wait(client.call_async(req), 10.0)
        return (r.success, r.message) if r else (False, 'no answer')

    def stop(self):
        return self._call(self._stop, Trigger.Request())

    def torque(self, on):
        """True = lock at the current pose, False = release (support the arm!)."""
        return self._call(self._torque, SetBool.Request(data=bool(on)))


def main():
    """ros2 run so100_driver arm_cli <command> ...   (positions: forward left, metres from the pan axis)
      pick FWD LEFT --size D W H [--mode top|side]
      place FWD LEFT
      pick_place FWD LEFT FWD2 LEFT2 --size D W H
      home | rest | stop | lock | release | state
    """
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node

    args = sys.argv[1:]
    if not args:
        print(main.__doc__)
        return
    size, mode = (0.03, 0.03, 0.05), 'auto'
    if '--size' in args:
        k = args.index('--size')
        size = tuple(float(v) for v in args[k + 1:k + 4])
        del args[k:k + 4]
    if '--mode' in args:
        k = args.index('--mode')
        mode = args[k + 1]
        del args[k:k + 2]
    rclpy.init()
    node = Node('arm_cli')
    arm = ArmClient(node)
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()
    cmd, nums = args[0], [float(v) for v in args[1:]]
    if cmd == 'pick':
        out = arm.pick(user_xy(*nums[:2]), size, grasp_mode=mode)
    elif cmd == 'place':
        out = arm.place(user_xy(*nums[:2]))
    elif cmd == 'pick_place':
        out = arm.pick_place(user_xy(*nums[:2]), user_xy(*nums[2:4]), size, grasp_mode=mode)
    elif cmd == 'home':
        out = arm.home()
    elif cmd == 'rest':
        out = arm.rest()
    elif cmd == 'stop':
        out = arm.stop()
    elif cmd in ('lock', 'release'):
        out = arm.torque(cmd == 'lock')
    elif cmd == 'state':
        for _ in range(30):
            if arm.state:
                break
            time.sleep(0.1)
        out = arm.state
    else:
        out = f'unknown command {cmd!r}\n{main.__doc__}'
    print(out)
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
