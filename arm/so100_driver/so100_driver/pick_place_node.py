"""SO-100 arm task server: the /arm/* API, the RViz pick-and-place test and the voice-system topics.

API (so100_interfaces, see API.md):
  actions   /arm/pick  /arm/place  /arm/pick_place  /arm/home  /arm/rest
  services  /arm/stop (std_srvs/Trigger)   /arm/torque (std_srvs/SetBool: true = lock, false = release)
  topic     /arm/state (so100_interfaces/ArmState, 2 Hz)
  parameter rest_after_s (default 120, 0 = off): fold into the rest pose after this long without a task
            (only when not holding anything). A new task interrupts the fold / leaves the rest pose itself.

Pick-and-place test with the closed-form IK (ik.py).

In RViz: drag the blue box onto the real object and the purple box to where it should go (ring = turn).
Set the object size in the control panel, then press "หยิบ-วาง".
  services  so100/pick_place (Trigger)  run the whole sequence
            so100/touch      (Trigger)  closed finger tips 1 cm above the table at the blue box (accuracy check)
  topic     so100/pick_place/status (String)  what the planner/sequence is doing
  parameter object_size [depth, width, height] m  (depth = front-back, width = left-right before turning)

Big moves go through MoveIt (collision-checked, object attached to the gripper while carried);
the short straight approach / lift / lower / retreat lines are sent straight to the arm driver.
"""
import math
import os
import threading
import time

import numpy as np
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.duration import Duration as RclDuration
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from control_msgs.msg import JointTolerance
from geometry_msgs.msg import Point, Pose, Quaternion
from interactive_markers import InteractiveMarkerServer
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (AttachedCollisionObject, CollisionObject, Constraints, JointConstraint, ObjectColor,
                             PlanningScene)
from moveit_msgs.srv import ApplyPlanningScene
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import ColorRGBA, Empty, String
from std_srvs.srv import SetBool, Trigger
import tf2_geometry_msgs  # noqa: F401  (registers PoseStamped for tf2)
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectoryPoint
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback, Marker

from so100_interfaces.action import Home, Pick, Place, Rest
from so100_interfaces.action import PickPlace as PickPlaceAction
from so100_interfaces.msg import ArmState

from .calib import Calibration
from .ik import ARM, GRASP_Y, TIP_Y, Box, GraspPlanner, Kinematics, PlanError

READY = {'shoulder_pan': 0.0, 'shoulder_lift': 1.2, 'elbow_flex': -1.2, 'wrist_flex': 0.3, 'wrist_roll': 0.0}
REST = {'shoulder_pan': 0.0, 'shoulder_lift': 0.04, 'elbow_flex': -0.04, 'wrist_flex': 0.0, 'wrist_roll': 0.0}  # SRDF 'rest'
AT_REST = 0.08     # rad: every joint this close to REST counts as resting
OBJ, TGT = 'object', 'target'
BLUE = (0.15, 0.55, 1.0, 1.0)      # same colour for the RViz box and MoveIt's copy of it


def yaw_q(yaw):
    return Quaternion(w=math.cos(yaw / 2), z=math.sin(yaw / 2))


def q_yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Failed(Exception):
    def __init__(self, msg, code=None):
        super().__init__(msg)
        text = str(msg)
        self.code = code or ('HOT' if 'too hot' in text else 'COLLISION' if any(
            w in text for w in ('blocked', 'stopped', 'short', 'ติด', 'ชน')) else 'FAILED')


def copy_box(b):
    return Box(b.x, b.y, b.yaw, b.size)


# English result messages of the API (the voice system can speak them)
MESSAGES = {
    'UNREACHABLE': 'I cannot reach that position.',
    'TOO_WIDE': 'The object is too wide for my gripper.',
    'MISSED': 'I missed the object.',
    'COLLISION': 'My arm was blocked or would hit the table.',
    'BUSY': 'I am busy with another task.',
    'HOLDING': 'I am already holding something.',
    'NOT_HOLDING': 'I am not holding anything.',
    'HOT': 'My motors are too hot. Please wait a minute.',
    'STOPPED': 'Stopped.',
    'FAILED': 'Something went wrong.',
}
SUCCESS = {'pick': 'I have picked it up.', 'place': 'I have put it down.', 'pick_place': 'Done, I moved it.',
           'home': 'I am at the ready pose.', 'rest': 'I am resting.', 'touch': 'Done.'}


class PickPlace(Node):
    def __init__(self):
        super().__init__('so100_pick_place')
        self.declare_parameter('robot_description', '')
        self.declare_parameter('calibration', '')
        self.declare_parameter('table_z', -0.075)
        self.declare_parameter('sim', False)
        self.declare_parameter('object_size', [0.03, 0.03, 0.05])
        self.declare_parameter('line_speed', 0.03)        # m/s on the straight approach/lift lines
        self.declare_parameter('velocity_scaling', 0.3)   # MoveIt moves
        self.declare_parameter('rest_after_s', 120.0)     # idle time before folding into the rest pose, 0 = off
        p = lambda n: self.get_parameter(n).value
        self.table_z = float(p('table_z'))
        self.sim = bool(p('sim'))
        limits, self.grip_max = None, 1.2
        if p('calibration'):
            try:
                cal = Calibration.load(p('calibration'))
                limits = {n: cal.rad_limits(n) for n in ARM}
                self.grip_max = cal.rad_limits('gripper')[1]
            except (OSError, ValueError) as exc:
                self.get_logger().warn(f'calibration not used ({exc}); URDF limits only')
        try:
            from ament_index_python.packages import get_package_share_directory
            mesh_dir = get_package_share_directory('so100_moveit_config') + '/meshes'
        except Exception:  # noqa: BLE001  (no meshes -> finger-tip check only)
            mesh_dir = None
        self.kin = Kinematics(p('robot_description'), limits, mesh_dir)
        if not self.kin.meshes:
            self.get_logger().warn('no meshes: only the finger tips are checked against the table')
        if limits:   # never wider than the URDF (which launch already narrowed to the real range)
            self.kin.limits = {n: (max(lo, self.kin.urdf_limits[n][0]), min(hi, self.kin.urdf_limits[n][1]))
                               for n, (lo, hi) in self.kin.limits.items()}
        self.planner = GraspPlanner(self.kin, self.table_z)
        # the rest pose inside the real (calibrated) travel
        self.rest_q = {n: min(max(v, self.kin.limits[n][0] + 0.02), self.kin.limits[n][1] - 0.02)
                       for n, v in REST.items()}

        size = tuple(p('object_size'))
        x, y = self.kin.from_user(0.20, 0.0)
        tx, ty = self.kin.from_user(0.20, -0.10)
        self.boxes = {OBJ: Box(x, y, self.kin.heading0, size), TGT: Box(tx, ty, self.kin.heading0, size)}
        self.joints = {}
        self.busy = False
        self.task, self.step, self.goal_handle, self.on_lifted = '', '', None, None
        self.cancel_requested = False
        self.moved = False
        self.last = ('SUCCESS', '')
        self.driver_status, self.driver_seen = '', -1e9
        self.idle_since = time.monotonic()  # end of the last task (auto-rest timer)
        self.hold_still = ''                # why the arm must not fold by itself (stopped / object maybe in jaws)
        self.preempt = False                # a new task asked the running fold to give way
        self.current = None                 # goal handle of the running motion (to cancel a fold)
        self.was_torque = False
        self.attached = False
        self.check = ''
        self.lock = threading.Lock()

        cb = ReentrantCallbackGroup()
        self.status_pub = self.create_publisher(String, 'so100/pick_place/status', 10)
        self.map_pub = self.create_publisher(Marker, 'so100/place_map', 1)
        self.map_gen, self.map_key, self.map_points, self.map_note = 0, None, [], ''
        self.create_subscription(JointState, 'joint_states', self.on_js, 20, callback_group=cb)
        self.arm = ActionClient(self, FollowJointTrajectory, 'arm_controller/follow_joint_trajectory', callback_group=cb)
        self.grip = ActionClient(self, GripperCommand, 'gripper_controller/gripper_cmd', callback_group=cb)
        self.move = ActionClient(self, MoveGroup, 'move_action', callback_group=cb)
        self.scene = self.create_client(ApplyPlanningScene, 'apply_planning_scene', callback_group=cb)
        self.create_service(Trigger, 'so100/pick_place', self.on_pick_place, callback_group=cb)
        self.create_service(Trigger, 'so100/touch', self.on_touch, callback_group=cb)
        self.holding = None              # plan of the object in the jaws (voice 'carry' -> 'drop')
        self.event_pub = self.create_publisher(String, '/robot/event', 10)
        self.create_subscription(Empty, '/robot/carry_bag', self.on_carry, 10, callback_group=cb)
        self.create_subscription(Empty, '/robot/drop_bag', self.on_drop, 10, callback_group=cb)
        # ---- /arm/* API
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.feedback_type = {'pick': Pick.Feedback, 'place': Place.Feedback, 'pick_place': PickPlaceAction.Feedback,
                              'home': Home.Feedback, 'rest': Rest.Feedback}
        for action, name, cbk in ((Pick, 'pick', self.exec_pick), (Place, 'place', self.exec_place),
                                  (PickPlaceAction, 'pick_place', self.exec_pick_place), (Home, 'home', self.exec_home),
                                  (Rest, 'rest', self.exec_rest)):
            ActionServer(self, action, f'/arm/{name}', execute_callback=cbk, cancel_callback=self.on_cancel,
                         callback_group=cb)
        self.driver_stop = self.create_client(Trigger, 'so100/stop', callback_group=cb)
        self.driver_enable = self.create_client(SetBool, 'so100/enable', callback_group=cb)
        self.create_service(Trigger, '/arm/stop', self.on_api_stop, callback_group=cb)
        self.create_service(SetBool, '/arm/torque', self.on_api_torque, callback_group=cb)
        self.state_pub = self.create_publisher(ArmState, '/arm/state', 10)
        self.create_subscription(String, 'so100/status', self.on_driver_status, 10, callback_group=cb)
        self.create_timer(0.5, self.publish_state, callback_group=cb)
        self.create_timer(1.0, self.auto_rest, callback_group=cb)
        self.add_on_set_parameters_callback(self.on_params)
        self.server = InteractiveMarkerServer(self, 'so100_objects')
        self.make_markers()
        self.create_timer(1.0, self.publish_status, callback_group=cb)
        self.scene_ready = False
        self.create_timer(2.0, self.init_scene, callback_group=cb)
        self.evaluate()
        self.create_timer(2.0, self.publish_map, callback_group=cb)
        self.request_map()

    # ---------------------------------------------------------------------------------- state
    def on_js(self, msg):
        self.joints = dict(zip(msg.name, msg.position))

    def say(self, text):
        self.check = text
        self.get_logger().info(text)
        self.publish_status()

    def publish_status(self):
        lines = []
        for key, label in ((OBJ, 'ของ'), (TGT, 'จุดวาง')):
            f, l_ = self.kin.to_user((self.boxes[key].x, self.boxes[key].y))
            yaw = math.degrees(self.boxes[key].yaw - self.kin.heading0)
            lines.append(f'{label}: หน้า {f * 100 + 0.0:.1f} ซม. ซ้าย {round(l_ * 100, 1) + 0.0:.1f} ซม. หมุน {round(yaw) + 0:.0f}°')
        s = self.boxes[OBJ].size
        lines.append(f'ขนาด ลึก×กว้าง×สูง {s[0] * 100:.1f}×{s[1] * 100:.1f}×{s[2] * 100:.1f} ซม.')
        lines.append(self.check)
        if self.map_note:
            lines.append(self.map_note)
        self.status_pub.publish(String(data='\n'.join(lines)))

    def on_params(self, params):
        for prm in params:
            if prm.name == 'object_size':
                v = [float(x) for x in prm.value]
                if len(v) != 3 or min(v) < 0.005 or max(v) > 0.3:
                    return SetParametersResult(successful=False, reason='object_size = [ลึก, กว้าง, สูง] เมตร (0.005..0.3)')
                if self.busy:
                    return SetParametersResult(successful=False, reason='กำลังหยิบ-วางอยู่')
                for b in self.boxes.values():
                    b.size = tuple(v)
                threading.Thread(target=self.after_change, daemon=True).start()
        return SetParametersResult(successful=True)

    def after_change(self):
        time.sleep(0.05)
        self.make_markers()
        self.evaluate()
        self.update_scene_object()
        self.request_map()

    # -------------------------------------------------------------------------------- markers
    def make_markers(self):
        # blue / purple: the robot model is orange and MoveIt draws scene objects green
        for key, colour, label in ((OBJ, BLUE, 'ของ'), (TGT, (0.75, 0.3, 0.95, 0.45), 'จุดวาง')):
            b = self.boxes[key]
            im = InteractiveMarker()
            im.header.frame_id = 'base'
            im.name = key
            im.scale = 0.12
            im.pose.position.x, im.pose.position.y = b.x, b.y
            im.pose.position.z = self.table_z + b.size[2] / 2
            im.pose.orientation = yaw_q(b.yaw)
            box = Marker(type=Marker.CUBE)
            box.scale.x, box.scale.y, box.scale.z = b.size
            box.color.r, box.color.g, box.color.b, box.color.a = colour
            text = Marker(type=Marker.TEXT_VIEW_FACING, text=label)
            text.pose.position.z = b.size[2] / 2 + 0.03
            text.scale.z = 0.025
            text.color.r = text.color.g = text.color.b = text.color.a = 1.0
            vertical = Quaternion(w=math.sqrt(0.5), y=math.sqrt(0.5))
            drag = InteractiveMarkerControl(name='drag', orientation=vertical, always_visible=True,
                                            interaction_mode=InteractiveMarkerControl.MOVE_PLANE)
            drag.markers = [box, text]
            turn = InteractiveMarkerControl(name='turn', orientation=vertical,
                                            interaction_mode=InteractiveMarkerControl.ROTATE_AXIS)
            im.controls = [drag, turn]
            self.server.insert(im, feedback_callback=self.on_marker)
        self.server.applyChanges()

    def on_marker(self, fb):
        if fb.event_type not in (InteractiveMarkerFeedback.POSE_UPDATE, InteractiveMarkerFeedback.MOUSE_UP):
            return
        if self.busy:          # boxes are locked while the arm works; snap back
            if fb.event_type == InteractiveMarkerFeedback.MOUSE_UP:
                self.make_markers()
            return
        b = self.boxes[fb.marker_name]
        b.x, b.y, b.yaw = fb.pose.position.x, fb.pose.position.y, q_yaw(fb.pose.orientation)
        if fb.event_type == InteractiveMarkerFeedback.MOUSE_UP:
            # keep the box standing on the table
            fb.pose.position.z = self.table_z + b.size[2] / 2
            self.server.setPose(fb.marker_name, fb.pose)
            self.server.applyChanges()
            self.evaluate()
            if fb.marker_name == OBJ:
                self.update_scene_object()
            self.request_map()

    # ------------------------------------------------- green "can be put down here" area in RViz
    MAP_STEP = 0.025          # grid spacing (m)
    MAP_RADIUS = (0.10, 0.40)  # from the pan axis; closer than 10 cm is the platform the base stands on

    def request_map(self):
        o, t = self.boxes[OBJ], self.boxes[TGT]
        key = (round(o.x, 4), round(o.y, 4), round(o.yaw, 3), o.size, round(t.yaw, 3))
        if key == self.map_key:
            return
        self.map_key = key
        self.map_gen += 1
        threading.Thread(target=self.compute_map, args=(self.map_gen,), daemon=True).start()

    def compute_map(self, gen):
        o, t = self.boxes[OBJ], self.boxes[TGT]
        obj = Box(o.x, o.y, o.yaw, o.size)
        lo, hi = self.MAP_RADIUS
        n = int(hi / self.MAP_STEP)
        cells = []
        for i in range(-n, n + 1):
            for j in range(-n, n + 1):
                v = np.array([i, j]) * self.MAP_STEP
                if lo <= np.hypot(*v) <= hi:
                    cells.append(self.kin.pan_point + v)
        self.map_note = 'กำลังคำนวณพื้นที่วางได้ (สีเขียวใน RViz)...'
        self.map_points = []
        self.publish_map()
        opts = self.planner.pick_options(obj, self.joints or None)
        found = []
        for k, (x, y) in enumerate(cells):
            if gen != self.map_gen:
                return                          # object moved again: a newer map is on its way
            if any(self.planner.can_place(obj, opt, x, y, t.yaw) for opt in opts):
                found.append((x, y))
            if k % 60 == 59:
                self.map_points = list(found)
                self.publish_map()
        self.map_points = found
        self.map_note = '' if found else '✘ ไม่มีจุดไหนวางของชิ้นนี้ได้ (หยิบไม่ได้ หรือของใหญ่เกิน)'
        self.publish_map()
        self.publish_status()

    def publish_map(self):
        m = Marker(ns='place_map', id=0, type=Marker.CUBE_LIST)
        m.header.frame_id = 'base'
        if not self.map_points:
            m.action = Marker.DELETE
        else:
            m.scale.x = m.scale.y = self.MAP_STEP * 0.8
            m.scale.z = 0.002
            m.color.r, m.color.g, m.color.b, m.color.a = 0.25, 0.9, 0.35, 0.6
            m.pose.orientation.w = 1.0
            z = self.table_z + 0.0015
            m.points = [Point(x=float(x), y=float(y), z=z) for x, y in self.map_points]
        self.map_pub.publish(m)

    def evaluate(self):
        try:
            plan = self.planner.plan(self.boxes[OBJ], self.boxes[TGT], self.joints or None)
            self.say(f'✔ หยิบได้: {plan["mode"]}, ของกว้าง {plan["extent"] * 100:.1f} ซม. ระหว่างก้าม')
        except PlanError as exc:
            self.say(f'✘ {exc}')

    # ---------------------------------------------------------------------------- planning scene
    def init_scene(self):
        if self.scene_ready or not self.scene.service_is_ready():
            return
        self.scene_ready = True
        self.update_scene_object()

    def collision_box(self, box, op=CollisionObject.ADD):
        co = CollisionObject(id=OBJ, operation=op)
        co.header.frame_id = 'base'
        if op == CollisionObject.ADD:
            co.primitives.append(SolidPrimitive(type=SolidPrimitive.BOX, dimensions=list(box.size)))
            pose = Pose()
            pose.position.x, pose.position.y = box.x, box.y
            pose.position.z = self.table_z + box.size[2] / 2
            pose.orientation = yaw_q(box.yaw)
            co.primitive_poses.append(pose)
        return co

    def apply(self, scene, wait=False):
        if not self.scene.service_is_ready():
            return False
        fut = self.scene.call_async(ApplyPlanningScene.Request(scene=scene))
        if not wait:
            return True
        return self.wait(fut, 5.0) is not None

    def update_scene_object(self, present=True, wait=False):
        scene = PlanningScene(is_diff=True)
        scene.world.collision_objects.append(
            self.collision_box(self.boxes[OBJ], CollisionObject.ADD if present else CollisionObject.REMOVE))
        if present:
            scene.object_colors.append(ObjectColor(id=OBJ, color=ColorRGBA(r=BLUE[0], g=BLUE[1], b=BLUE[2], a=1.0)))
        return self.apply(scene, wait)

    def attach(self, box):
        scene = PlanningScene(is_diff=True)
        scene.robot_state.is_diff = True
        aco = AttachedCollisionObject(link_name='gripper', touch_links=['gripper', 'jaw', 'wrist'])
        aco.object = self.collision_box(box)
        scene.robot_state.attached_collision_objects.append(aco)
        scene.world.collision_objects.append(self.collision_box(box, CollisionObject.REMOVE))
        return self.apply(scene, wait=True)

    def detach_to(self, box):
        scene = PlanningScene(is_diff=True)
        scene.robot_state.is_diff = True
        aco = AttachedCollisionObject(link_name='gripper')
        aco.object = CollisionObject(id=OBJ, operation=CollisionObject.REMOVE)
        scene.robot_state.attached_collision_objects.append(aco)
        scene.world.collision_objects.append(self.collision_box(box))
        scene.object_colors.append(ObjectColor(id=OBJ, color=ColorRGBA(r=BLUE[0], g=BLUE[1], b=BLUE[2], a=1.0)))
        return self.apply(scene, wait=True)

    # ----------------------------------------------------------------------------- actions
    def wait(self, fut, timeout):
        end = time.monotonic() + timeout
        while not fut.done():
            if time.monotonic() > end:
                return None
            time.sleep(0.01)
        return fut.result()

    def run_action(self, client, goal, timeout, what):
        self.moved = True
        if not client.wait_for_server(timeout_sec=3.0):
            raise Failed(f'{what}: ไม่พบ action server')
        if self.preempt:
            raise Failed(f'{what}: ให้ทางงานใหม่', 'STOPPED')
        handle = self.wait(client.send_goal_async(goal), 5.0)
        if handle is None or not handle.accepted:
            raise Failed(f'{what}: คำสั่งถูกปฏิเสธ')
        self.current = handle
        if self.preempt:                 # asked to give way while the goal was being sent
            handle.cancel_goal_async()
        try:
            res = self.wait(handle.get_result_async(), timeout)
        finally:
            self.current = None
        if self.preempt:
            raise Failed(f'{what}: ให้ทางงานใหม่', 'STOPPED')
        if res is None:
            handle.cancel_goal_async()
            raise Failed(f'{what}: หมดเวลา')
        return res.result

    def moveit_to(self, q, what):
        goal = MoveGroup.Goal()
        r = goal.request
        r.group_name = 'arm'
        r.pipeline_id = 'ompl'
        r.num_planning_attempts = 3
        r.allowed_planning_time = 5.0
        r.max_velocity_scaling_factor = float(self.get_parameter('velocity_scaling').value)
        r.max_acceleration_scaling_factor = 0.6
        r.start_state.is_diff = True
        c = Constraints()
        for n in ARM:
            c.joint_constraints.append(JointConstraint(joint_name=n, position=float(q[n]), tolerance_above=0.005,
                                                       tolerance_below=0.005, weight=1.0))
        r.goal_constraints = [c]
        goal.planning_options.plan_only = False
        res = self.run_action(self.move, goal, 60.0, what)
        code = res.error_code.val
        if code != 1:
            reason = {99999: 'หาเส้นทางไม่ได้ (ท่าปลายทางชนของ/โต๊ะในโมเดล หรือทางถูกบัง)',
                      -10: 'ท่าเริ่มต้นชนอยู่แล้วในโมเดล', -12: 'ท่าปลายทางชนในโมเดล',
                      -4: 'แขนไปไม่ถึง (ติด/ชน หรือมอเตอร์ไม่ตาม)', -6: 'หมดเวลา',
                      -7: 'ถูกหยุดกลางทาง'}.get(code, f'MoveIt code {code}')
            raise Failed(f'{what}: {reason}')

    def line(self, qs, what):
        """Send a straight-line joint path (equal 5 mm steps) with a smooth start/stop."""
        speed = float(self.get_parameter('line_speed').value)
        n = len(qs)
        total = 1.875 * n * self.planner.STEP / speed     # min-jerk peak speed = 1.875 x average
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        goal.path_tolerance = [JointTolerance(name=j, position=0.2) for j in ARM]   # = exact path, no retime
        for i, q in enumerate(qs, 1):
            t = total * self.minjerk_time(i / n)
            pt = JointTrajectoryPoint(positions=[float(q[j]) for j in ARM])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            goal.trajectory.points.append(pt)
        res = self.run_action(self.arm, goal, total + 10.0, what)
        if res.error_code != 0:
            raise Failed(f'{what}: {res.error_string or res.error_code}')

    @staticmethod
    def minjerk_time(s):
        """Time fraction at which a min-jerk move has covered fraction s of the path."""
        lo, hi = 0.0, 1.0
        for _ in range(40):
            m = (lo + hi) / 2
            if 10 * m ** 3 - 15 * m ** 4 + 6 * m ** 5 < s:
                lo = m
            else:
                hi = m
        return (lo + hi) / 2

    def gripper(self, pos, what):
        goal = GripperCommand.Goal()
        goal.command.position = float(min(pos, self.grip_max))
        goal.command.max_effort = 1.0
        return self.run_action(self.grip, goal, 10.0, what)

    # ---------------------------------------------------------------------------- task runner
    # Every motion task (panel button, voice topic, API action) runs through run_task(): one at a time,
    # with the same error codes, progress reporting and clean-up.
    def run_task(self, name, fn, handle=None, auto=False):
        """Run fn() as task `name`. Returns (code, english_message, thai_detail).
        A fold into the rest pose never makes a new task BUSY: it is cancelled and the new task runs."""
        end = time.monotonic() + 10.0
        while True:
            with self.lock:
                if not self.busy:
                    self.busy, self.task, self.step, self.goal_handle = True, name, '', handle
                    self.cancel_requested, self.preempt = False, False
                    self.moved = False       # set by the first motion command; refusals keep the scene as is
                    break
                if self.task != 'rest' or name == 'rest' or time.monotonic() > end:
                    return 'BUSY', MESSAGES['BUSY'], 'กำลังทำงานอื่นอยู่'
                if not self.preempt:
                    self.preempt = True
                    if self.current is not None:
                        self.current.cancel_goal_async()
                    self.get_logger().info(f'{name}: หยุดการไปท่าพัก แล้วทำงานใหม่')
            time.sleep(0.05)
        if not auto:
            self.hold_still = ''         # a new command from someone: the arm may fold again later
        given_way = False
        try:
            if name not in ('rest', 'home') and self.at_rest():
                self.leave_rest()
            done = fn()
            code, detail = 'SUCCESS', done or 'เสร็จ'
            self.say(f'✔ {detail}')
        except (Failed, PlanError) as exc:
            code, detail = getattr(exc, 'code', 'FAILED'), str(exc)
            if self.cancel_requested:
                code = 'STOPPED'
            self.say(f'✘ หยุด: {detail}')
            if self.attached:
                self.hold_still = 'อาจมีของค้างในก้าม'
            if self.moved and name != 'rest':
                self.release_scene()
        except Exception as exc:  # keep the node alive and tell the user  # noqa: BLE001
            code, detail = 'FAILED', repr(exc)
            self.say(f'✘ ผิดพลาด: {detail}')
            self.get_logger().error(detail)
            if self.attached:
                self.hold_still = 'อาจมีของค้างในก้าม'
            if self.moved and name != 'rest':
                self.release_scene()
        finally:
            with self.lock:
                given_way = self.preempt
                self.busy, self.task, self.step, self.goal_handle = False, '', '', None
                self.preempt = False
                self.idle_since = time.monotonic()
        if code == 'STOPPED' and not given_way:
            self.hold_still = 'ถูกสั่งหยุด'      # someone stopped it on purpose: stay put until the next command
        msg = MESSAGES.get(code, MESSAGES['FAILED']) if code != 'SUCCESS' else SUCCESS[name]
        if not (auto or given_way):
            self.last = (code, msg)
        return code, msg, detail

    def start(self, name, fn, after=None, auto=False):
        """Run a task in the background (panel buttons, voice topics). False if the arm is busy
        (a fold into the rest pose does not count: the new task replaces it)."""
        if self.busy and (self.task != 'rest' or name == 'rest'):
            return False

        def job():
            res = self.run_task(name, fn, auto=auto)
            if after:
                after(res)
        threading.Thread(target=job, daemon=True).start()
        return True

    # ----------------------------------------------------------------------------- rest pose
    def at_rest(self):
        return bool(self.joints) and all(abs(self.joints.get(n, 9.0) - v) < AT_REST for n, v in self.rest_q.items())

    def leave_rest(self):
        self.step = 'leave_rest'
        self.say('ออกจากท่าพัก (ไปท่า ready)...')
        self.moveit_to(READY, 'ออกจากท่าพัก')

    def do_rest(self):
        if self.holding:
            raise Failed('ถือของอยู่: วางก่อนค่อยไปท่าพัก', 'HOLDING')
        if self.at_rest():
            return 'อยู่ท่าพักแล้ว'
        self.progress(1, 2, 'close', 'หุบก้าม...')
        self.gripper(0.0, 'หุบก้าม')
        self.progress(2, 2, 'rest', 'ไปท่าพัก...')
        self.moveit_to(self.rest_q, 'ไปท่าพัก')
        return 'อยู่ท่าพัก'

    def auto_rest(self):
        """Fold into the rest pose after rest_after_s seconds without a task (not while holding)."""
        torque, fault = self.driver_flags()
        if torque and not self.was_torque:
            self.idle_since = time.monotonic()      # torque just came back on: count from now
        self.was_torque = torque
        after = float(self.get_parameter('rest_after_s').value)
        if (after <= 0 or self.busy or self.holding or self.hold_still or not torque or fault
                or time.monotonic() - self.driver_seen > 3.0 or self.at_rest()):
            return
        if time.monotonic() - self.idle_since >= after:
            self.get_logger().info(f'ว่างมา {after:.0f} วิ: ไปท่าพัก')
            self.start('rest', self.do_rest, auto=True)

    def progress(self, i, n, key, text):
        self.step = key
        self.say(f'{i}/{n} {text}')
        h = self.goal_handle
        if h is not None:
            fb = self.feedback_type[self.task]()
            fb.step, fb.step_index, fb.step_count = key, i, n
            h.publish_feedback(fb)

    def release_scene(self):
        """After a failure the object may be in the jaws or knocked over: drop MoveIt's copy of it
        (attached or not), so RViz / MoveIt can move the arm again. It is re-added on the next run."""
        self.holding = None
        scene = PlanningScene(is_diff=True)
        scene.robot_state.is_diff = True
        if self.attached:
            aco = AttachedCollisionObject(link_name='gripper')
            aco.object = CollisionObject(id=OBJ, operation=CollisionObject.REMOVE)
            scene.robot_state.attached_collision_objects.append(aco)
            self.attached = False
        scene.world.collision_objects.append(self.collision_box(self.boxes[OBJ], CollisionObject.REMOVE))
        self.apply(scene, wait=True)

    # ------------------------------------------------------------------------- panel buttons
    def on_pick_place(self, _req, res):
        o, t = self.boxes[OBJ], self.boxes[TGT]
        res.success = self.start('pick_place', lambda: self.do_pick_place(copy_box(o), copy_box(t)))
        res.message = 'เริ่มหยิบ-วาง' if res.success else 'กำลังทำงานอยู่'
        return res

    def on_touch(self, _req, res):
        res.success = self.start('touch', self.touch)
        res.message = 'เริ่มแตะจุด' if res.success else 'กำลังทำงานอยู่'
        return res

    def touch(self):
        b = self.boxes[OBJ]
        centre = np.array([b.x, b.y, self.table_z + 0.01 + (GRASP_Y - TIP_Y)])   # closed tips 1 cm above table
        q, err = None, 'เอื้อมไม่ถึง'
        v = centre[:2] - self.kin.pan_point
        h = math.atan2(v[1], v[0])
        side = np.array([-math.sin(h), math.cos(h), 0.0])      # jaws close sideways
        for pitch in self.planner.PITCHES:
            if pitch > self.planner.NEAR_DOWN:
                break
            try:
                q = self.planner._pose(centre, pitch, side, 0.016, None, self.joints.get('wrist_roll', 0.0), x_off=0.0)
                self.planner.check_table([q], 0.016, 'แตะจุด', 0)
                break
            except PlanError as exc:
                err, q = str(exc), None
        if q is None:
            raise Failed(f'แตะจุดไม่ได้: {err}', 'UNREACHABLE')
        self.update_scene_object(present=False, wait=True)   # the box marks the spot, it is not an obstacle
        self.say('กำลังหุบก้าม...')
        self.gripper(0.0, 'หุบก้าม')
        above = self.planner._pose(centre + [0, 0, 0.04], pitch, side, 0.016, None, q['wrist_roll'], x_off=0.0)
        self.say('ไปเหนือจุด...')
        self.moveit_to(above, 'ไปเหนือจุด')
        self.say('ลงไปแตะ...')
        self.line(self.planner._line(centre + [0, 0, 0.04], centre, pitch, side, 0.016, q['_branch'], q['wrist_roll'],
                                     0.0), 'ลงไปแตะ')
        f, l_ = self.kin.to_user(centre)
        return f'ปลายก้ามอยู่เหนือโต๊ะ 1 ซม. ที่ หน้า {f * 100:.1f} ซม. ซ้าย {l_ * 100:.1f} ซม. — วัดดูว่าตรงไหม'

    # ------------------------------------------------------------------------------ the tasks
    def show_object(self, obj):
        """Put the blue box (and MoveIt's copy) where the task's object is."""
        self.boxes[OBJ] = copy_box(obj)
        for b in self.boxes.values():
            b.size = obj.size
        self.make_markers()
        self.update_scene_object(present=True, wait=True)
        self.request_map()

    def do_pick(self, obj, mode='auto'):
        if self.holding:
            raise Failed('ถือของอยู่แล้ว: สั่งวางก่อน', 'HOLDING')
        plan = self.planner.plan_pick(obj, self.joints or None, mode)
        self.show_object(obj)
        self.pick(plan, n=6)
        self.holding = plan
        self.progress(6, 6, 'carry', 'ยกมาถือไว้ (ท่า ready)...')
        self.moveit_to(READY, 'ยกมาถือไว้')
        return f'ถือของอยู่ ({plan["mode"]})'

    def do_place(self, target, fallback_home_spot=False, released=None):
        if not self.holding:
            raise Failed('ไม่ได้ถือของอยู่', 'NOT_HOLDING')
        x, y, yaw, size = self.holding['object']
        obj = Box(x, y, yaw, size)
        try:
            plan = self.planner.plan_place(obj, self.holding, target)
            where = 'ที่จุดวาง'
        except PlanError:
            if not fallback_home_spot:
                raise
            plan = self.planner.plan_place(obj, self.holding, obj)      # voice drop: back where it came from
            where = 'คืนที่เดิม (จุดวางไกลเกิน)'
        self.boxes[TGT] = copy_box(target)
        self.make_markers()
        self.place(plan, first=1, n=3, released=released)
        return f'วางของแล้ว{where}'

    def do_pick_place(self, obj, target, mode='auto'):
        if self.holding:
            raise Failed('ถือของอยู่แล้ว: สั่งวางก่อน', 'HOLDING')
        plan = self.planner.plan(obj, target, self.joints or None, mode)
        self.show_object(obj)
        self.boxes[TGT] = copy_box(target)
        self.make_markers()
        self.pick(plan)
        self.place(plan)
        return 'หยิบ-วางเสร็จ (กล่องฟ้าย้ายไปตำแหน่งใหม่แล้ว)'

    def do_home(self):
        self.progress(1, 1, 'home', 'กลับท่า ready...')
        self.moveit_to(READY, 'กลับท่า ready')
        return 'อยู่ท่า ready'

    def pick(self, plan, n=8):
        s = plan['steps']
        obj = self.boxes[OBJ]
        self.say(f'แผน: {plan["mode"]} อ้าก้าม {plan["gap"] * 100:.1f} ซม.')
        self.update_scene_object(present=True, wait=True)
        self.progress(1, n, 'move_above', 'ไปเหนือของ (MoveIt)...')
        self.moveit_to(s['pre'], 'ไปเหนือของ')
        self.progress(2, n, 'open', 'อ้าก้าม...')
        self.gripper(plan['jaw_open'], 'อ้าก้าม')
        self.progress(3, n, 'approach', 'ลงไปที่ของ...')
        self.line(s['approach'], 'ลงไปที่ของ')
        self.progress(4, n, 'grasp', 'หนีบ...')
        r = self.gripper(0.0, 'หนีบ')
        if not self.sim and (r.reached_goal or r.position < 0.05):
            self.say('✘ จับพลาด (ก้ามหุบสุด) — กำลังถอยขึ้น')
            self.gripper(plan['jaw_open'], 'อ้าก้าม')
            self.line(list(reversed(s['approach'][:-1])) + [s['pre']], 'ถอยขึ้น')
            raise Failed('จับพลาด: ลองเลื่อนกล่องสีฟ้าให้ตรงของจริง หรือเช็กขนาด', 'MISSED')
        self.attach(obj)
        self.attached = True
        self.progress(5, n, 'lift', 'ยกขึ้น...')
        self.line(s['lift'], 'ยกขึ้น')
        if self.on_lifted:
            self.on_lifted()

    def place(self, plan, first=6, n=8, released=None):
        s = plan['steps']
        obj = self.boxes[OBJ]
        self.progress(first, n, 'move_above_target', 'ไปเหนือจุดวาง (MoveIt)...')
        self.moveit_to(s['above_place'], 'ไปเหนือจุดวาง')
        self.progress(first + 1, n, 'lower', 'วางลง...')
        self.line(s['lower'], 'วางลง')
        self.gripper(plan['jaw_open'], 'ปล่อย')
        if released:
            released()               # voice system waits only ~10 s: report before backing away
        # the object now stands at the target: move the blue box there
        obj.x, obj.y, obj.yaw = float(plan['place_centre'][0]), float(plan['place_centre'][1]), plan['final_yaw']
        self.detach_to(obj)
        self.attached = False
        self.holding = None
        self.make_markers()
        self.request_map()
        self.progress(first + 2, n, 'retreat', 'ถอยออกแล้วกลับท่า ready...')
        self.line(s['retreat'], 'ถอยออก')
        self.moveit_to(READY, 'กลับท่า ready')

    # ------------------------------------------------------------------ voice system (topics)
    #   /robot/carry_bag (Empty)  -> pick the blue-box object and hold it at the ready pose
    #   /robot/drop_bag  (Empty)  -> put it down at the purple box (or back where it was) and let go
    #   /robot/event     (String) <- grab_success / grab_failed / drop_done / drop_failed
    # (older than the /arm/* API; kept until the voice system moves to the API)
    def event(self, text):
        self.event_pub.publish(String(data=text))
        self.get_logger().info(f'/robot/event: {text}')

    def on_carry(self, _msg):
        sent = []

        def lifted():                # report as soon as the object is off the table (voice waits 20 s)
            self.event('grab_success')
            sent.append(1)

        def fn():
            self.on_lifted = lifted
            try:
                return self.do_pick(copy_box(self.boxes[OBJ]))
            finally:
                self.on_lifted = None

        def after(_res):
            if not sent:
                self.event('grab_failed')
        if not self.start('pick', fn, after):
            self.say('✘ สั่งหยิบจากเสียง: กำลังทำงานอื่นอยู่')
            self.event('grab_failed')

    def on_drop(self, _msg):
        sent = []

        def released():
            self.event('drop_done')
            sent.append(1)

        def after(_res):
            if not sent:
                self.event('drop_failed')
        if not self.start('place', lambda: self.do_place(copy_box(self.boxes[TGT]), True, released), after):
            self.say('✘ สั่งวางจากเสียง: กำลังทำงานอื่นอยู่')
            self.event('drop_failed')

    # ---------------------------------------------------------------------------- /arm/* API
    def to_box(self, pose, size=None):
        """PoseStamped (any TF frame) -> Box in the arm base frame."""
        ps = pose
        if ps.header.frame_id not in ('', 'base'):
            try:
                ps = self.tf.transform(pose, 'base', timeout=RclDuration(seconds=0.5))
            except Exception as exc:  # noqa: BLE001
                raise Failed(f'แปลงพิกัดจาก frame "{pose.header.frame_id}" ไม่ได้: {exc}', 'FAILED')
        q = ps.pose.orientation
        x, y = ps.pose.position.x, ps.pose.position.y
        if abs(q.x) + abs(q.y) + abs(q.z) + abs(q.w) < 1e-9:      # unknown: assume it faces the arm
            yaw = math.atan2(y - self.kin.pan_point[1], x - self.kin.pan_point[0])
        else:
            yaw = q_yaw(q)
        if size is None:
            size = self.boxes[OBJ].size
        size = tuple(float(v) for v in size)
        if len(size) != 3 or min(size) <= 0.0 or max(size) > 0.3:
            raise Failed('object_size ต้องเป็น [ลึก, กว้าง, สูง] เมตร (0..0.3)', 'FAILED')
        return Box(x, y, yaw, size)

    def finish(self, handle, result, res):
        code, msg, detail = res
        result.success = code == 'SUCCESS'
        result.error_code, result.message = code, msg
        self.get_logger().info(f'API {self.feedback_name(handle)} -> {code}: {detail}')
        if code == 'STOPPED' and handle.is_cancel_requested:
            handle.canceled()
        elif result.success:
            handle.succeed()
        else:
            handle.abort()
        return result

    @staticmethod
    def feedback_name(handle):
        return type(handle.request).__name__.replace('_Goal', '')

    def api_task(self, name, handle, result, build):
        """Shared execute callback: build() turns the goal into a task function (may raise Failed)."""
        try:
            fn = build(handle.request)
        except (Failed, PlanError) as exc:
            code = getattr(exc, 'code', 'FAILED')
            return self.finish(handle, result, (code, MESSAGES.get(code, MESSAGES['FAILED']), str(exc)))
        return self.finish(handle, result, self.run_task(name, fn, handle))

    def exec_pick(self, handle):
        return self.api_task('pick', handle, Pick.Result(), lambda g: (
            lambda obj=self.to_box(g.object_pose, g.object_size): self.do_pick(obj, g.grasp_mode or 'auto')))

    def exec_place(self, handle):
        return self.api_task('place', handle, Place.Result(), lambda g: (
            lambda tgt=self.to_box(g.target_pose): self.do_place(tgt)))

    def exec_pick_place(self, handle):
        def build(g):
            obj = self.to_box(g.object_pose, g.object_size)
            tgt = self.to_box(g.target_pose, g.object_size)
            return lambda: self.do_pick_place(obj, tgt, g.grasp_mode or 'auto')
        return self.api_task('pick_place', handle, PickPlaceAction.Result(), build)

    def exec_home(self, handle):
        return self.api_task('home', handle, Home.Result(), lambda g: self.do_home)

    def exec_rest(self, handle):
        return self.api_task('rest', handle, Rest.Result(), lambda g: self.do_rest)

    def on_cancel(self, _handle):
        self.stop_motion()
        return CancelResponse.ACCEPT

    def stop_motion(self):
        """Stop whatever runs: the driver freezes the arm, which makes the running step fail -> STOPPED."""
        self.cancel_requested = True
        if self.driver_stop.service_is_ready():
            self.driver_stop.call_async(Trigger.Request())

    def on_api_stop(self, _req, res):
        busy = self.busy
        self.stop_motion()
        res.success, res.message = True, 'stopping the running task' if busy else 'nothing was running; arm holds'
        return res

    def on_api_torque(self, req, res):
        if not req.data and self.busy:
            self.stop_motion()
        if not self.driver_enable.wait_for_service(timeout_sec=2.0):
            res.success, res.message = False, 'arm driver not running'
            return res
        r = self.wait(self.driver_enable.call_async(SetBool.Request(data=req.data)), 5.0)
        if r is not None and r.success and not req.data and self.holding:
            self.release_scene()      # released by hand: the object is no longer known to be held
        if r is not None and r.success and not req.data:
            self.hold_still = ''      # someone handles the arm by hand now
        res.success = bool(r and r.success)
        res.message = (r.message if r else 'no answer from the driver') + (
            '' if req.data else ' (support the arm: it falls when the torque is off)')
        return res

    def on_driver_status(self, msg):
        self.driver_status, self.driver_seen = msg.data, time.monotonic()

    def driver_field(self, key):
        text = self.driver_status
        return text.split(key + '=', 1)[1].split(' ', 1)[0] if key + '=' in text else ''

    def driver_flags(self):
        """(torque_on, fault text) from the driver's so100/status line."""
        text = self.driver_status
        fault = '' if self.driver_field('fault') in ('', 'none') else text.split('fault=', 1)[1].split(' load%')[0]
        return self.driver_field('torque') == 'ON', fault

    def publish_state(self):
        st = ArmState()
        st.header.stamp = self.get_clock().now().to_msg()
        st.torque_on, st.fault = self.driver_flags()
        try:
            st.voltage_min = float(self.driver_field('voltage').split('-')[0].rstrip('V'))
            st.temp_max = float(self.driver_field('temp_max').rstrip('C'))
        except ValueError:
            pass
        st.holding = self.holding is not None
        st.task, st.step = self.task, self.step
        st.last_error_code, st.last_message = self.last
        if time.monotonic() - self.driver_seen > 3.0:
            st.state = 'no_arm'
        elif st.fault:
            st.state = 'fault'
        elif self.busy:
            st.state = 'busy'
        elif st.holding:
            st.state = 'holding'
        elif not st.torque_on:
            st.state = 'torque_off'
        elif self.at_rest():
            st.state = 'resting'
        else:
            st.state = 'idle'
        self.state_pub.publish(st)


def main():
    rclpy.init()
    node = PickPlace()
    ex = MultiThreadedExecutor(num_threads=8)
    ex.add_node(node)
    try:
        ex.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        ex.shutdown(timeout_sec=1.0)
        rclpy.try_shutdown()
        os._exit(0)   # worker threads may still wait on an action; never linger after launch stops


if __name__ == '__main__':
    main()
