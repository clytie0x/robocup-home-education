"""Arm action bridge: voice flow  <->  SO-100 arm API v2 (/arm/pick, /arm/place)

    flow_node --/robot/command {"action":"carry_bag"}--> arm_action_bridge --/arm/pick (action)--> so100_pick_place
    flow_node <--/robot/event "grab_success" | "grab_failed|CODE|message"-- arm_action_bridge <-- result

The flow keeps using the same events as with the old topics; failures now carry the arm's error code and its
English message (e.g. "grab_failed|UNREACHABLE|I can't reach it."), which the robot says out loud.

Where is the object?  (until the camera is ready)
  1. a PoseStamped on `object_pose_topic` (default /object/pose) received in the last `pose_max_age` s, else
  2. the fixed test position `pick_forward` / `pick_left` (metres, measured from the arm's base axis like a tape
     measure: forward = in front of the arm, left = to the arm's left).
Drop position: `place_forward` / `place_left` (or a PoseStamped on `place_pose_topic`).

Also mirrors /arm/state.holding as events "arm_holding" / "arm_empty", so the flow knows whether the
gripper really holds something (e.g. after a pick that the flow thought had failed).

Needs the arm team's interface package `so100_interfaces` built in the same workspace.
"""
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

try:
    from so100_interfaces.action import Pick, Place
    from so100_interfaces.msg import ArmState
except ImportError:          # package not built here yet -> node explains what to do
    Pick = Place = ArmState = None

import json

BASE_AXIS_Y = -0.0452        # SO-100: base rotation axis is at (x=0, y=-0.0452) in frame "base"


def user_xy(forward: float, left: float, frame: str = 'base') -> PoseStamped:
    """Tape-measure coordinates -> PoseStamped in the arm's `base` frame.
    The arm's "forward" is -y and "left" is +x (see so100-arm-api.md). Orientation all zeros = unknown yaw."""
    p = PoseStamped()
    p.header.frame_id = frame
    p.pose.position.x = float(left)
    p.pose.position.y = BASE_AXIS_Y - float(forward)
    p.pose.position.z = 0.0
    p.pose.orientation.x = p.pose.orientation.y = p.pose.orientation.z = p.pose.orientation.w = 0.0
    return p


def event_text(name: str, ok: bool, code: str = '', message: str = '') -> str:
    """'grab_success'  or  'grab_failed|UNREACHABLE|I can't reach it.'  (flow_node splits on '|')."""
    if ok:
        return f'{name}_success' if name == 'grab' else 'drop_done'
    return '|'.join([f'{name}_failed', code or 'FAILED', (message or '').replace('|', ' ')])


class ArmActionBridge(Node):
    def __init__(self):
        super().__init__('arm_action_bridge')
        self.declare_parameter('pick_forward', 0.20)     # m in front of the arm's base axis
        self.declare_parameter('pick_left', 0.0)         # m to the arm's left (negative = right)
        self.declare_parameter('object_size', [0.03, 0.03, 0.05])   # [depth, width, height] m (gripper max ~6.5 cm)
        self.declare_parameter('grasp_mode', 'auto')     # auto | top | side
        self.declare_parameter('place_forward', 0.20)
        self.declare_parameter('place_left', -0.10)
        self.declare_parameter('object_pose_topic', '/object/pose')   # PoseStamped from the camera (any TF frame)
        self.declare_parameter('place_pose_topic', '/place/pose')
        self.declare_parameter('pose_max_age', 5.0)      # ignore a camera pose older than this (s)
        p = lambda n: self.get_parameter(n).value
        self.cfg = {n: p(n) for n in ('pick_forward', 'pick_left', 'object_size', 'grasp_mode',
                                      'place_forward', 'place_left', 'pose_max_age')}

        self.pub_event = self.create_publisher(String, '/robot/event', 10)
        self.pub_state = self.create_publisher(String, '/robot/state', 10)
        self.create_subscription(String, '/robot/command', self._on_cmd, 10)
        self.last_obj = (None, 0.0)
        self.last_place = (None, 0.0)
        self.create_subscription(PoseStamped, p('object_pose_topic'),
                                 lambda m: setattr(self, 'last_obj', (m, time.time())), 10)
        self.create_subscription(PoseStamped, p('place_pose_topic'),
                                 lambda m: setattr(self, 'last_place', (m, time.time())), 10)
        # Keep the voice side's "am I carrying something?" in sync with the arm (/arm/state.holding):
        # publishes "arm_holding" / "arm_empty" on /robot/event whenever it changes.
        self.holding = None
        if ArmState is not None:
            self.create_subscription(ArmState, '/arm/state', self._on_arm_state, 10)

        if Pick is None:
            self.pick_ac = self.place_ac = None
            self.get_logger().error(
                'so100_interfaces is not installed in this workspace. Copy the arm team\'s '
                'so100_interfaces package into ~/ros2_ws/src, then: colcon build --packages-select so100_interfaces '
                'voice_assistant && source install/setup.bash. Until then carry/drop will fail with NO_INTERFACES.')
        else:
            self.pick_ac = ActionClient(self, Pick, '/arm/pick')
            self.place_ac = ActionClient(self, Place, '/arm/place')
            self.get_logger().info(
                f'arm action bridge ready -> /arm/pick, /arm/place. Test pick point: {self.cfg["pick_forward"]:.2f} m '
                f'forward, {self.cfg["pick_left"]:.2f} m left; drop point: {self.cfg["place_forward"]:.2f} m forward, '
                f'{self.cfg["place_left"]:.2f} m left (or send a PoseStamped on /object/pose)')

    def _on_arm_state(self, st):
        h = bool(st.holding)
        if h != self.holding:
            self.holding = h
            self._event('arm_holding' if h else 'arm_empty')

    # ---------------- helpers ----------------
    def _fresh(self, slot):
        msg, t = slot
        return msg if msg is not None and time.time() - t <= float(self.cfg['pose_max_age']) else None

    def _event(self, text):
        self.get_logger().info(f'event -> {text}')
        self.pub_event.publish(String(data=text))

    # ---------------- commands ----------------
    def _on_cmd(self, msg: String):
        try:
            action = json.loads(msg.data).get('action')
        except json.JSONDecodeError:
            return
        if action == 'carry_bag':
            self._pick()
        elif action == 'drop_bag':
            self._place()

    def _check_single_arm(self, action):
        """Each action server publishes <action>/_action/status; more than one = two arms answering."""
        n = self.count_publishers(f'{action}/_action/status')
        if n > 1:
            self.get_logger().warn(f'{n} servers answer {action} (fake_arm_server AND the real arm?). '
                                   'Close one of them, otherwise you get BUSY / mixed results.')

    def _pick(self):
        self._check_single_arm('/arm/pick')
        if self.pick_ac is None:
            return self._event(event_text('grab', False, 'NO_INTERFACES', 'My arm software is not set up yet.'))
        if not self.pick_ac.server_is_ready():
            return self._event(event_text('grab', False, 'NO_ARM', 'My arm is not connected.'))
        goal = Pick.Goal()
        cam = self._fresh(self.last_obj)
        goal.object_pose = cam if cam is not None else user_xy(self.cfg['pick_forward'], self.cfg['pick_left'])
        goal.object_size = [float(v) for v in self.cfg['object_size']]
        if hasattr(goal, 'grasp_mode'):
            goal.grasp_mode = str(self.cfg['grasp_mode'])
        src = 'camera' if cam is not None else 'fixed test point'
        pos = goal.object_pose.pose.position
        self.get_logger().info(f'/arm/pick at x={pos.x:.3f} y={pos.y:.3f} ({src}, frame "{goal.object_pose.header.frame_id}")')
        self._send(self.pick_ac, goal, 'grab')

    def _place(self):
        self._check_single_arm('/arm/place')
        if self.place_ac is None:
            return self._event(event_text('drop', False, 'NO_INTERFACES', 'My arm software is not set up yet.'))
        if not self.place_ac.server_is_ready():
            return self._event(event_text('drop', False, 'NO_ARM', 'My arm is not connected.'))
        goal = Place.Goal()
        tgt = self._fresh(self.last_place)
        goal.target_pose = tgt if tgt is not None else user_xy(self.cfg['place_forward'], self.cfg['place_left'])
        pos = goal.target_pose.pose.position
        self.get_logger().info(f'/arm/place at x={pos.x:.3f} y={pos.y:.3f}')
        self._send(self.place_ac, goal, 'drop')

    def _send(self, client, goal, name):
        def on_feedback(fb):
            f = fb.feedback
            step = getattr(f, 'step', '?')
            i, n = getattr(f, 'step_index', 0), getattr(f, 'step_count', 0)
            self.get_logger().info(f'arm {name}: {step} ({i}/{n})')
            self.pub_state.publish(String(data=f'arm {name}: {step} {i}/{n}'))

        def on_result(fut):
            try:
                res = fut.result().result
            except Exception as e:  # noqa: BLE001
                return self._event(event_text(name, False, 'FAILED', f'Arm error: {e}'))
            ok = bool(getattr(res, 'success', False))
            code, text = str(getattr(res, 'error_code', '')), str(getattr(res, 'message', ''))
            self.get_logger().info(f'arm {name} result: {code} "{text}"')
            if code == 'STOPPED':
                return                       # the user said stop: flow is already back in listening
            self._event(event_text(name, ok, code, text))

        def on_goal(fut):
            handle = fut.result()
            if not handle.accepted:
                return self._event(event_text(name, False, 'REJECTED', 'My arm refused the task.'))
            handle.get_result_async().add_done_callback(on_result)

        client.send_goal_async(goal, feedback_callback=on_feedback).add_done_callback(on_goal)


def main():
    rclpy.init()
    node = ArmActionBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
