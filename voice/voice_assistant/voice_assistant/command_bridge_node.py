"""Example bridge: /robot/command (JSON String) -> /cmd_vel (geometry_msgs/Twist)

This is the place to connect the voice system to YOUR robot.
- move_forward / move_backward / turn / stop  -> open-loop timed Twist on /cmd_vel (stop also calls /arm/stop)
- carry_bag                                    -> stop, then std_msgs/Empty on carry_topic (grab / hold the bag)
- drop_bag                                     -> stop, then std_msgs/Empty on drop_topic (gripper/servo node)
- go_to / follow_me / look_at                  -> TODO: hook into Nav2, a follower node, a pan-tilt, ...

Replace or extend `_dispatch()` for your robot. Keep the safety limits.
"""
import json
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty, String
from std_srvs.srv import Trigger

try:                                   # SO-100 arm state (optional: only if so100_interfaces is built here)
    from so100_interfaces.msg import ArmState
except ImportError:
    ArmState = None


class CommandBridge(Node):
    def __init__(self):
        super().__init__('command_bridge_node')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('default_speed', 0.15)    # m/s
        self.declare_parameter('max_speed', 0.3)         # m/s safety clamp
        self.declare_parameter('turn_speed', 0.6)        # rad/s
        self.declare_parameter('max_distance', 2.0)      # m per voice command
        self.declare_parameter('drop_topic', '/robot/drop_bag')    # std_msgs/Empty -> your gripper node
        self.declare_parameter('carry_topic', '/robot/carry_bag')  # std_msgs/Empty -> grab / hold the bag
        # False with arm:=actions: arm_action_bridge drives /arm/pick + /arm/place, so the old topics must stay
        # silent (otherwise the arm starts a pick from the old topic AND gets an action -> BUSY)
        self.declare_parameter('bag_topics', True)
        self.declare_parameter('arm_stop_service', '/arm/stop')    # std_srvs/Trigger: "stop" also freezes the arm ('' = off)

        p = lambda n: self.get_parameter(n).value
        self.default_speed = float(p('default_speed'))
        self.max_speed = float(p('max_speed'))
        self.turn_speed = float(p('turn_speed'))
        self.max_distance = float(p('max_distance'))

        self.pub = self.create_publisher(Twist, p('cmd_vel_topic'), 10)
        # '' = do not publish (arm:=actions: arm_action_bridge talks to /arm/pick and /arm/place instead)
        use_topics = bool(p('bag_topics')) and bool(p('carry_topic')) and bool(p('drop_topic'))
        self.pub_drop = self.create_publisher(Empty, p('drop_topic'), 10) if use_topics else None
        self.pub_carry = self.create_publisher(Empty, p('carry_topic'), 10) if use_topics else None
        arm_srv = p('arm_stop_service')
        self.arm_stop = self.create_client(Trigger, arm_srv) if arm_srv else None
        # Only call /arm/stop while the arm is really moving. Calling it on an idle arm marks the last task
        # as STOPPED, which keeps the arm team's auto-rest (go to rest pose after 2 min idle) from running.
        self.arm_state = None          # latest /arm/state.state ('idle', 'busy', ...); None = unknown
        if ArmState is not None:
            self.create_subscription(ArmState, '/arm/state',
                                     lambda m: setattr(self, 'arm_state', m.state), 10)
        self.pub_state = self.create_publisher(String, '/robot/state', 10)
        self.create_subscription(String, '/robot/command', self._on_cmd, 10)

        self.active = Twist()
        self.until = 0.0
        self.create_timer(0.05, self._tick)   # 20 Hz
        self.get_logger().info('command bridge ready (bag topics: '
                               f'{"/robot/carry_bag, /robot/drop_bag" if self.pub_carry else "OFF - arm actions"})')

    def _on_cmd(self, msg: String):
        try:
            cmd = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(f'bad command: {msg.data}')
            return
        self._dispatch(cmd.get('action', 'none'), cmd.get('params') or {})

    def _dispatch(self, action: str, prm: dict):
        t = Twist()
        if action == 'stop':
            self.until = 0.0
            self.pub.publish(Twist())
            if prm.get('arm', True) is not False:     # flow's error-stop sends arm=False
                self._stop_arm()
            self._state('stopped')
            return

        if action in ('move_forward', 'move_backward'):
            speed = min(abs(float(prm.get('speed') or self.default_speed)), self.max_speed)
            dist = min(abs(float(prm.get('distance') or 0.5)), self.max_distance)
            t.linear.x = speed if action == 'move_forward' else -speed
            self._start(t, dist / speed, f'{action} {dist:.2f} m')
        elif action == 'turn':
            ang = math.radians(float(prm.get('angle') or 90.0))
            t.angular.z = math.copysign(self.turn_speed, ang)
            self._start(t, abs(ang) / self.turn_speed, f'turning {math.degrees(ang):.0f} deg')
        elif action == 'carry_bag':
            # stop, then ask the gripper/servo node to grab / hold the bag
            self.until = 0.0
            self.pub.publish(Twist())
            if self.pub_carry:
                self.pub_carry.publish(Empty())
                self.get_logger().info('carry_bag -> stopped, grab sent on /robot/carry_bag')
            else:
                self.get_logger().info('carry_bag -> stopped (arm handled by arm_action_bridge)')
            self._state('carrying bag')
        elif action == 'drop_bag':
            # stop moving first, then ask the gripper/servo node to release the bag
            self.until = 0.0
            self.pub.publish(Twist())
            if self.pub_drop:
                self.pub_drop.publish(Empty())
                self.get_logger().info('drop_bag -> stopped, release sent on /robot/drop_bag')
            else:
                self.get_logger().info('drop_bag -> stopped (arm handled by arm_action_bridge)')
            self._state('bag dropped')
        elif action in ('go_to', 'follow_me', 'look_at'):
            # TODO: send a Nav2 NavigateToPose goal, start a follower, move the head, etc.
            self.get_logger().info(f'[TODO] {action} target={prm.get("target")}')
            self._state(f'{action} not implemented yet')
        else:
            self.get_logger().info(f'ignored action: {action}')

    def _stop_arm(self):
        """Freeze the arm too (SO-100 API /arm/stop). Never blocks; skipped if the arm node is not running."""
        if self.arm_stop is None:
            return
        if self.arm_state is not None and self.arm_state != 'busy':
            self.get_logger().info(f'arm is {self.arm_state}, not moving -> /arm/stop not needed')
            return
        if not self.arm_stop.service_is_ready():
            self.get_logger().info(f'{self.arm_stop.srv_name} not available (arm not running?) - wheels stopped only',
                                   throttle_duration_sec=30.0)
            return
        fut = self.arm_stop.call_async(Trigger.Request())
        fut.add_done_callback(lambda f: self.get_logger().info(
            f'arm stop: {f.result().message if f.result() else "no answer"}'))

    def _start(self, twist: Twist, duration: float, desc: str):
        self.active = twist
        self.until = time.time() + duration
        self.get_logger().info(f'{desc} ({duration:.1f}s)')
        self._state(desc)

    def _tick(self):
        if self.until and time.time() < self.until:
            self.pub.publish(self.active)
        elif self.until:
            self.until = 0.0
            self.pub.publish(Twist())
            self._state('idle')

    def _state(self, s: str):
        self.pub_state.publish(String(data=s))


def main():
    rclpy.init()
    node = CommandBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(Twist())
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
