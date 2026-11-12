"""Turtlesim "bag" simulator: makes carry_bag / drop_bag visible in turtlesim.

- /robot/carry_bag (Empty) -> robot pen becomes thick green (carrying a bag).
                              If a dropped bag is lying within `pickup_radius`, it disappears (picked up).
- /robot/drop_bag  (Empty) -> a bag turtle (bag_N) appears just BEHIND the robot, with a yellow box
                              drawn around it; the robot pen goes back to normal.
Run together with turtlesim_node and command_bridge_node (cmd_vel_topic:=/turtle1/cmd_vel).

Also pretends to be the robot's sensors for flow_node, on /robot/event (std_msgs/String):
  grab_success / grab_failed after carry_bag, drop_done / drop_failed after drop_bag,
  and "see_person" once a second (simulate_person:=false to test "I can't see you").
"""
import math
import threading
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Empty, String
from turtlesim.msg import Pose
from turtlesim.srv import Kill, SetPen, Spawn, TeleportAbsolute

DEFAULT_PEN = (179, 184, 255, 3)   # turtlesim default pen colour / width


class TurtlesimBagSim(Node):
    def __init__(self):
        super().__init__('turtlesim_bag_sim')
        self.declare_parameter('drop_offset', 0.6)     # how far behind the robot the bag lands
        self.declare_parameter('box_size', 0.7)        # yellow box drawn around a dropped bag
        self.declare_parameter('pickup_radius', 1.0)   # carry_bag picks up a bag closer than this
        self.declare_parameter('simulate_person', True)  # publish "see_person" every second
        self.declare_parameter('event_delay', 1.0)       # seconds before grab_success / drop_done (like a real gripper)
        p = lambda n: float(self.get_parameter(n).value)
        self.drop_offset, self.box_size, self.pickup_radius = p('drop_offset'), p('box_size'), p('pickup_radius')

        self.pose = None
        self.carrying = False
        self.n_bags = 0
        self.bags = {}            # name -> (x, y) of bags lying on the floor
        self.lock = threading.Lock()

        self.pen = self.create_client(SetPen, '/turtle1/set_pen')
        self.spawn = self.create_client(Spawn, '/spawn')
        self.kill = self.create_client(Kill, '/kill')
        self.pub_state = self.create_publisher(String, '/robot/state', 10)
        self.pub_event = self.create_publisher(String, '/robot/event', 10)
        self.event_delay = float(self.get_parameter('event_delay').value)
        if self.get_parameter('simulate_person').value:
            self.create_timer(1.0, lambda: self.pub_event.publish(String(data='see_person')))
        self.create_subscription(Pose, '/turtle1/pose', self._on_pose, 10)
        self.create_subscription(Empty, '/robot/carry_bag', self._on_carry, 10)
        self.create_subscription(Empty, '/robot/drop_bag', self._on_drop, 10)
        self.get_logger().info('bag sim ready (green line = carrying, yellow box = dropped bag); '
                               'sending fake robot events on /robot/event')

    def _event(self, name):
        time.sleep(self.event_delay)
        self.pub_event.publish(String(data=name))
        self.get_logger().info(f'event -> {name}')

    # ---------- helpers (run in worker threads; the executor resolves the futures) ----------
    def _call(self, client, req, timeout=2.0):
        if not client.wait_for_service(timeout_sec=timeout):
            self.get_logger().warn(f'service {client.srv_name} not available')
            return None
        fut = client.call_async(req)
        t0 = time.time()
        while not fut.done() and time.time() - t0 < timeout:
            time.sleep(0.02)
        return fut.result() if fut.done() else None

    def _set_robot_pen(self, r, g, b, width):
        self._call(self.pen, SetPen.Request(r=r, g=g, b=b, width=width, off=0))

    def _on_pose(self, msg: Pose):
        self.pose = msg

    # ---------- carry ----------
    def _on_carry(self, _):
        threading.Thread(target=self._carry, daemon=True).start()

    def _carry(self):
        with self.lock:
            if self.carrying:
                self.get_logger().info('already carrying a bag')
                self._event('grab_success')
                return
            # pick up the nearest dropped bag, if any is close enough
            if self.pose and self.bags:
                name, (bx, by) = min(self.bags.items(),
                                     key=lambda kv: math.hypot(kv[1][0] - self.pose.x, kv[1][1] - self.pose.y))
                if math.hypot(bx - self.pose.x, by - self.pose.y) <= self.pickup_radius:
                    self._call(self.kill, Kill.Request(name=name))
                    del self.bags[name]
                    self.get_logger().info(f'picked up {name} from the floor')
            self.carrying = True
            self._set_robot_pen(0, 200, 0, 6)
            self.pub_state.publish(String(data='carrying bag'))
            self._event('grab_success')
            self.get_logger().info('carrying the bag (green line)')

    # ---------- drop ----------
    def _on_drop(self, _):
        threading.Thread(target=self._drop, daemon=True).start()

    def _drop(self):
        with self.lock:
            if not self.carrying:
                self.get_logger().warn('drop_bag received but the robot is not carrying a bag '
                                       '(say "carry this bag" first)')
                self.pub_state.publish(String(data='no bag to drop'))
                self._event('drop_failed')
                return
            if self.pose is None:
                self.get_logger().warn('no /turtle1/pose yet')
                return
            self.carrying = False
            self._set_robot_pen(*DEFAULT_PEN)

            # land the bag just behind the robot, clamped inside the 11x11 turtlesim window
            th = self.pose.theta
            x = min(max(self.pose.x - self.drop_offset * math.cos(th), 0.5), 10.5)
            y = min(max(self.pose.y - self.drop_offset * math.sin(th), 0.5), 10.5)
            self.n_bags += 1
            name = f'bag_{self.n_bags}'
            if self._call(self.spawn, Spawn.Request(x=x, y=y, theta=th, name=name)) is None:
                self.get_logger().error('could not spawn the bag turtle')
                self._event('drop_failed')
                return
            self.bags[name] = (x, y)
            self._draw_box(name, x, y, th)
            self.pub_state.publish(String(data=f'bag dropped at x={x:.1f} y={y:.1f}'))
            self._event('drop_done')
            self.get_logger().info(f'{name} dropped at x={x:.2f} y={y:.2f} (yellow box)')

    def _draw_box(self, name, x, y, th):
        """Use the bag turtle's own pen to draw a yellow square around it, then park it in the middle."""
        pen = self.create_client(SetPen, f'/{name}/set_pen')
        tp = self.create_client(TeleportAbsolute, f'/{name}/teleport_absolute')
        h = self.box_size / 2
        corners = [(x - h, y - h), (x + h, y - h), (x + h, y + h), (x - h, y + h), (x - h, y - h)]
        self._call(pen, SetPen.Request(off=1))                       # move to first corner without drawing
        self._call(tp, TeleportAbsolute.Request(x=corners[0][0], y=corners[0][1], theta=th))
        self._call(pen, SetPen.Request(r=255, g=210, b=0, width=4, off=0))
        for cx, cy in corners[1:]:
            self._call(tp, TeleportAbsolute.Request(x=cx, y=cy, theta=th))
        self._call(pen, SetPen.Request(off=1))
        self._call(tp, TeleportAbsolute.Request(x=x, y=y, theta=th))
        self.destroy_client(pen)
        self.destroy_client(tp)


def main():
    rclpy.init()
    node = TurtlesimBagSim()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
