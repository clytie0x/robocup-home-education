"""Fake SO-100 arm: same /arm/pick, /arm/place actions and /arm/stop service as the real arm, no hardware.
Use it to test the voice side with arm:=actions before connecting the real arm.  Never run it together with
the real arm (both would answer).

  ros2 run voice_assistant fake_arm_server                                  # everything succeeds
  ros2 run voice_assistant fake_arm_server --ros-args -p pick_result:=UNREACHABLE   # test a failure
  ros2 run voice_assistant fake_arm_server --ros-args -p pick_seconds:=15.0         # slow arm

Codes: SUCCESS UNREACHABLE TOO_WIDE MISSED COLLISION BUSY HOLDING NOT_HOLDING HOT STOPPED FAILED
Needs the arm team's so100_interfaces package in the workspace.
"""
import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger

from so100_interfaces.action import Pick, Place

MESSAGES = {
    'SUCCESS': None,  # filled per task below
    'UNREACHABLE': "I can't reach it.",
    'TOO_WIDE': 'It is too wide for my gripper.',
    'MISSED': 'I missed it. Let me try again later.',
    'COLLISION': 'Something is in my way.',
    'BUSY': 'I am busy right now.',
    'HOLDING': 'I am already holding something.',
    'NOT_HOLDING': 'I am not holding anything.',
    'HOT': 'My motors are too hot. Please wait.',
    'STOPPED': 'Stopped.',
    'FAILED': 'Something went wrong with my arm.',
}
PICK_STEPS = ['move_above', 'open', 'approach', 'grasp', 'lift', 'home']
PLACE_STEPS = ['carry', 'move_above_target', 'lower', 'open', 'retreat', 'home']


class FakeArm(Node):
    def __init__(self):
        super().__init__('fake_arm_server')
        self.declare_parameter('pick_seconds', 4.0)
        self.declare_parameter('place_seconds', 3.0)
        self.declare_parameter('pick_result', 'SUCCESS')
        self.declare_parameter('place_result', 'SUCCESS')
        self.holding = False
        self.busy = threading.Lock()
        self.stop_evt = threading.Event()
        cg = ReentrantCallbackGroup()
        ActionServer(self, Pick, '/arm/pick', self._exec_pick, callback_group=cg,
                     cancel_callback=lambda _: CancelResponse.ACCEPT)
        ActionServer(self, Place, '/arm/place', self._exec_place, callback_group=cg,
                     cancel_callback=lambda _: CancelResponse.ACCEPT)
        self.create_service(Trigger, '/arm/stop', self._on_stop, callback_group=cg)
        self.get_logger().info('FAKE arm ready: /arm/pick, /arm/place, /arm/stop (no hardware)')

    def _on_stop(self, req, resp):
        self.stop_evt.set()
        resp.success, resp.message = True, 'Arm stopped.'
        self.get_logger().info('/arm/stop called')
        return resp

    def _run(self, handle, steps, seconds, forced, ok_msg, name):
        result = (Pick if name == 'pick' else Place).Result()
        if not self.busy.acquire(blocking=False):
            code = 'BUSY'
        elif name == 'pick' and self.holding:
            code = 'HOLDING'
        elif name == 'place' and not self.holding:
            code = 'NOT_HOLDING'
        else:
            code = None
        if code:
            if code != 'BUSY':
                self.busy.release()
            handle.abort()
            result.success, result.error_code, result.message = False, code, MESSAGES[code]
            self.get_logger().info(f'{name}: {code}')
            return result
        try:
            self.stop_evt.clear()
            fb = (Pick if name == 'pick' else Place).Feedback()
            for i, step in enumerate(steps, 1):
                fb.step, fb.step_index, fb.step_count = step, i, len(steps)
                handle.publish_feedback(fb)
                self.get_logger().info(f'{name}: {step} ({i}/{len(steps)})')
                t_end = time.time() + seconds / len(steps)
                while time.time() < t_end:
                    if self.stop_evt.is_set() or handle.is_cancel_requested:
                        handle.abort()
                        result.success, result.error_code, result.message = False, 'STOPPED', MESSAGES['STOPPED']
                        return result
                    time.sleep(0.05)
                if forced != 'SUCCESS' and step in ('grasp', 'lower'):
                    handle.abort()
                    result.success, result.error_code, result.message = False, forced, MESSAGES.get(forced, 'Failed.')
                    self.get_logger().info(f'{name}: {forced} (forced by parameter)')
                    return result
            self.holding = (name == 'pick')
            handle.succeed()
            result.success, result.error_code, result.message = True, 'SUCCESS', ok_msg
            return result
        finally:
            self.busy.release()

    def _exec_pick(self, handle):
        g = handle.request
        pos = g.object_pose.pose.position
        self.get_logger().info(f'PICK goal: x={pos.x:.3f} y={pos.y:.3f} frame="{g.object_pose.header.frame_id}" '
                               f'size={list(g.object_size)}')
        p = lambda n: self.get_parameter(n).value
        return self._run(handle, PICK_STEPS, float(p('pick_seconds')), str(p('pick_result')).upper(),
                         'I have picked it up.', 'pick')

    def _exec_place(self, handle):
        pos = handle.request.target_pose.pose.position
        self.get_logger().info(f'PLACE goal: x={pos.x:.3f} y={pos.y:.3f}')
        p = lambda n: self.get_parameter(n).value
        return self._run(handle, PLACE_STEPS, float(p('place_seconds')), str(p('place_result')).upper(),
                         'I have put it down.', 'place')


def main():
    rclpy.init()
    node = FakeArm()
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
