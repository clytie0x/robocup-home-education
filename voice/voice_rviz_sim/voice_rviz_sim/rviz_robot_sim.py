"""Simple robot simulator for RViz, driven by the voice_assistant flow.

Shows only the robot: body, heading arrow, carried bag, trail and dropped bags.

Subscribes
  <cmd_vel_topic>      geometry_msgs/Twist   velocity from command_bridge_node
  /robot/carry_bag     std_msgs/Empty        robot picks up the bag
  /robot/drop_bag      std_msgs/Empty        robot drops the bag behind it
  /voice_viz/reset     std_msgs/Empty        reset pose, trail and bags

Publishes
  TF odom -> base_link, /odom, /voice_viz/markers (MarkerArray)
  /robot/event         std_msgs/String       fake robot events for flow_node:
                       grab_success / drop_done  (event_delay s after the command)
                       see_person                (every second, if simulate_person)
"""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped, Point
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty, String, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import TransformBroadcaster


def yaw_to_quat(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def rgba(r, g, b, a=1.0):
    return ColorRGBA(r=float(r), g=float(g), b=float(b), a=float(a))


class RvizRobotSim(Node):
    def __init__(self):
        super().__init__('rviz_robot_sim')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('rate', 30.0)
        self.declare_parameter('marker_rate', 10.0)    # Hz, markers need less than TF
        self.declare_parameter('cmd_timeout', 0.5)     # stop if no cmd_vel for this long
        self.declare_parameter('trail_step', 0.02)     # m between trail points
        self.declare_parameter('max_trail', 5000)
        self.declare_parameter('robot_length', 0.5)
        self.declare_parameter('robot_width', 0.4)
        self.declare_parameter('simulate_events', True)   # send grab_success / drop_done
        self.declare_parameter('simulate_person', True)   # send see_person every second
        self.declare_parameter('event_delay', 1.0)

        p = lambda n: self.get_parameter(n).value
        self.odom_frame = p('odom_frame')
        self.base_frame = p('base_frame')
        self.cmd_timeout = float(p('cmd_timeout'))
        self.trail_step = float(p('trail_step'))
        self.max_trail = int(p('max_trail'))
        self.L = float(p('robot_length'))
        self.W = float(p('robot_width'))
        cmd_topic = p('cmd_vel_topic')
        self.simulate_events = bool(p('simulate_events'))
        self.simulate_person = bool(p('simulate_person'))
        self.event_delay = float(p('event_delay'))
        self.pending = []                  # [(due_time, event)]

        self.x = self.y = self.yaw = 0.0
        self.v = self.w = 0.0
        self.last_cmd = None
        self.carrying = False
        self.bags = []                     # list of (x, y)
        self.trail = [(0.0, 0.0, False)]   # (x, y, carrying)

        self.tf = TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/voice_viz/markers', 10)
        self.event_pub = self.create_publisher(String, '/robot/event', 10)

        self.create_subscription(Twist, cmd_topic, self.on_cmd, 10)
        self.create_subscription(Empty, '/robot/carry_bag', self.on_carry, 10)
        self.create_subscription(Empty, '/robot/drop_bag', self.on_drop, 10)
        self.create_subscription(Empty, '/voice_viz/reset', self.on_reset, 10)

        self.dt = 1.0 / float(p('rate'))
        self.tick = 0
        self.marker_every = max(1, round(float(p('rate')) / float(p('marker_rate'))))
        self.last_time = self.get_clock().now()
        self.create_timer(self.dt, self.step)
        if self.simulate_person:
            self.create_timer(1.0, lambda: self.send_event('see_person', log=False))
        self.dup_timer = self.create_timer(3.0, self.check_duplicates)
        self.get_logger().info(
            f'RViz robot sim ready (cmd_vel: {cmd_topic}, events: {self.simulate_events}, '
            f'person: {self.simulate_person})')

    def check_duplicates(self):
        self.dup_timer.cancel()
        n = sum(1 for name in self.get_node_names() if name == self.get_name())
        if n > 1:
            self.get_logger().error(
                f'{n} nodes named {self.get_name()} are running (other terminal, Jetson or a '
                'teammate on the same ROS_DOMAIN_ID). They fight over /odom, TF and markers: '
                'the robot will jump or blink. Stop the others or change ROS_DOMAIN_ID.')

    # ---------------- callbacks ----------------
    def on_cmd(self, msg):
        self.v = msg.linear.x
        self.w = msg.angular.z
        self.last_cmd = self.get_clock().now()

    def on_carry(self, _):
        self.carrying = True
        self.get_logger().info('carrying the bag')
        self.schedule('grab_success')

    def on_drop(self, _):
        if not self.carrying:
            self.get_logger().info('drop_bag received (no bag carried, dropping anyway)')
        self.carrying = False
        bx = self.x - math.cos(self.yaw) * (self.L / 2 + 0.25)
        by = self.y - math.sin(self.yaw) * (self.L / 2 + 0.25)
        self.bags.append((bx, by))
        self.get_logger().info(f'bag_{len(self.bags)} dropped at ({bx:.2f}, {by:.2f})')
        self.schedule('drop_done')

    def on_reset(self, _):
        self.x = self.y = self.yaw = self.v = self.w = 0.0
        self.carrying = False
        self.bags.clear()
        self.trail = [(0.0, 0.0, False)]
        self.pending.clear()
        self.publish_markers(clear=True)
        self.get_logger().info('reset')

    # ---------------- fake robot events ----------------
    def schedule(self, event):
        if not self.simulate_events:
            return
        due = self.get_clock().now().nanoseconds + int(self.event_delay * 1e9)
        self.pending.append((due, event))

    def send_event(self, event, log=True):
        self.event_pub.publish(String(data=event))
        if log:
            self.get_logger().info(f'event -> {event}')

    # ---------------- simulation ----------------
    def step(self):
        now = self.get_clock().now()
        due = [e for t, e in self.pending if t <= now.nanoseconds]
        self.pending = [(t, e) for t, e in self.pending if t > now.nanoseconds]
        for e in due:
            self.send_event(e)
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now
        if dt <= 0 or dt > 1.0:
            dt = self.dt

        if self.last_cmd is None or (now - self.last_cmd).nanoseconds * 1e-9 > self.cmd_timeout:
            self.v = self.w = 0.0

        self.yaw += self.w * dt
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))
        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt

        lx, ly, lc = self.trail[-1]
        if math.hypot(self.x - lx, self.y - ly) >= self.trail_step or lc != self.carrying:
            self.trail.append((self.x, self.y, self.carrying))
            if len(self.trail) > self.max_trail:
                self.trail = self.trail[-self.max_trail:]

        self.publish_tf_odom(now)
        self.tick += 1
        if self.tick % self.marker_every == 0:
            self.publish_markers()

    def publish_tf_odom(self, now):
        q = yaw_to_quat(self.yaw)
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        (t.transform.rotation.x, t.transform.rotation.y,
         t.transform.rotation.z, t.transform.rotation.w) = q
        self.tf.sendTransform(t)

        o = Odometry()
        o.header = t.header
        o.child_frame_id = self.base_frame
        o.pose.pose.position.x = self.x
        o.pose.pose.position.y = self.y
        (o.pose.pose.orientation.x, o.pose.pose.orientation.y,
         o.pose.pose.orientation.z, o.pose.pose.orientation.w) = q
        o.twist.twist.linear.x = self.v
        o.twist.twist.angular.z = self.w
        self.odom_pub.publish(o)

    # ---------------- markers ----------------
    def marker(self, ns, mid, mtype):
        # Everything is drawn in the odom frame with stamp 0, so RViz never has to look
        # up base_link. The robot stays visible even if the TF status flips to error.
        m = Marker()
        m.header.frame_id = self.odom_frame
        m.ns, m.id, m.type, m.action = ns, mid, mtype, Marker.ADD
        m.pose.orientation.w = 1.0
        return m

    def at_robot(self, m, z):
        q = yaw_to_quat(self.yaw)
        m.pose.position.x, m.pose.position.y, m.pose.position.z = self.x, self.y, z
        (m.pose.orientation.x, m.pose.orientation.y,
         m.pose.orientation.z, m.pose.orientation.w) = q
        return m

    def publish_markers(self, clear=False):
        arr = MarkerArray()
        if clear:
            d = Marker()
            d.action = Marker.DELETEALL
            arr.markers.append(d)
            self.marker_pub.publish(arr)
            arr = MarkerArray()

        # robot body, drawn at the robot pose (no TF lookup)
        body = self.at_robot(self.marker('robot', 0, Marker.CUBE), 0.1)
        body.scale.x, body.scale.y, body.scale.z = self.L, self.W, 0.2
        body.color = rgba(0.2, 0.5, 1.0)
        arr.markers.append(body)

        arrow = self.at_robot(self.marker('robot', 1, Marker.ARROW), 0.22)
        arrow.scale.x, arrow.scale.y, arrow.scale.z = self.L * 0.8, 0.06, 0.06
        arrow.color = rgba(1.0, 1.0, 1.0)
        arr.markers.append(arrow)

        carried = self.at_robot(self.marker('robot', 2, Marker.CUBE), 0.3)
        carried.scale.x = carried.scale.y = carried.scale.z = 0.2
        carried.color = rgba(1.0, 0.85, 0.1, 1.0 if self.carrying else 0.0)
        arr.markers.append(carried)

        # trail: blue normally, green while carrying
        trail = self.marker('trail', 0, Marker.LINE_LIST)
        trail.scale.x = 0.03
        for (x0, y0, c0), (x1, y1, _) in zip(self.trail, self.trail[1:]):
            col = rgba(0.1, 0.9, 0.2) if c0 else rgba(0.3, 0.6, 1.0, 0.8)
            trail.points += [Point(x=x0, y=y0, z=0.01), Point(x=x1, y=y1, z=0.01)]
            trail.colors += [col, col]
        if trail.points:
            arr.markers.append(trail)

        # dropped bags
        for i, (bx, by) in enumerate(self.bags):
            b = self.marker('bags', i, Marker.CUBE)
            b.pose.position.x, b.pose.position.y, b.pose.position.z = bx, by, 0.1
            b.scale.x = b.scale.y = b.scale.z = 0.2
            b.color = rgba(1.0, 0.85, 0.1)
            arr.markers.append(b)
            lbl = self.marker('bag_labels', i, Marker.TEXT_VIEW_FACING)
            lbl.pose.position.x, lbl.pose.position.y, lbl.pose.position.z = bx, by, 0.35
            lbl.scale.z = 0.15
            lbl.color = rgba(1.0, 0.85, 0.1)
            lbl.text = f'bag_{i + 1}'
            arr.markers.append(lbl)

        self.marker_pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = RvizRobotSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
