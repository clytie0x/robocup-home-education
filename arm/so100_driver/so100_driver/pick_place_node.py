"""SO-100 arm task server: the /arm/* API, the RViz pick-and-place test and the voice-system topics.

API (so100_interfaces, see API.md):
  actions   /arm/pick  /arm/place  /arm/pick_place  /arm/home  /arm/rest  /arm/pick_bag
  services  /arm/stop (std_srvs/Trigger)   /arm/torque (std_srvs/SetBool: true = lock, false = release)
  topic     /arm/state (so100_interfaces/ArmState, 2 Hz)
  parameter rest_after_s (default 120, 0 = off): fold into the rest pose after this long without a task
            (only when not holding anything). A new task interrupts the fold / leaves the rest pose itself.
  arm on the robot: deck = [front, back, left, right] deck edges from the pan axis (m), deck_z = deck top;
            table_z is then the floor. /arm/pick_bag holds a bag by its top and carries it at the side
            (bag_carry_side deg, 90 = left) with its bottom bag_bottom_min m above the floor.
            A PickBag grasp_pose z of NaN = top unknown: it is found by touch first (probe_bag, bag_probe_*),
            unless gripper_camera.json exists (gripper_camera_file): then the 0.02 m/s probe is skipped and
            bag_default_top is used. speed_profile safe (default) keeps today's speeds; fast raises them.

Pick-and-place test with the closed-form IK (ik.py).

In RViz: drag the blue box onto the real object and the purple box to where it should go (ring = turn).
Set the object size in the control panel, then press "หยิบ-วาง".
  services  so100/pick_place (Trigger)  run the whole sequence
            so100/touch      (Trigger)  closed finger tips 1 cm above the table at the blue box (accuracy check)
  topic     so100/pick_place/status (String)  what the planner/sequence is doing
  parameter object_size [depth, width, height] m  (depth = front-back, width = left-right before turning)

Big moves go through MoveIt (collision-checked, object attached to the gripper while carried).
The straight approach and lift are Pilz LIN (pipeline already loaded). Other short lines
(probe steps, carry, lower, retreat) are still sent straight to the arm driver.
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
from moveit_msgs.msg import (AttachedCollisionObject, BoundingVolume, CollisionObject, Constraints, JointConstraint,
                             ObjectColor, OrientationConstraint, PlanningScene, PositionConstraint)
from moveit_msgs.srv import ApplyPlanningScene
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import ColorRGBA, Empty, Header, String
from std_srvs.srv import SetBool, Trigger
import tf2_geometry_msgs  # noqa: F401  (registers PoseStamped for tf2)
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectoryPoint
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback, Marker

from so100_interfaces.action import Home, Pick, PickBag, Place, Rest
from so100_interfaces.action import PickPlace as PickPlaceAction
from so100_interfaces.msg import ArmState

from .bag_camera import GAP_MID, CameraBag
from .calib import Calibration
from .ik import ARM, GRASP_Y, TIP_Y, Box, GraspPlanner, Kinematics, PlanError, bag_shifts, deck_box
from . import motion_profile, pick_log
from .task_rules import (PICK_PLACE, PROBE_JOINTS, Failed, api_code, descent_loads, gripper_camera_calibrated,
                         gripper_load, held_by_weight, probe_contact, settled_load, still_held, top_unknown)

READY = {"shoulder_pan": 0.0, "shoulder_lift": 1.2, "elbow_flex": -1.2, "wrist_flex": 0.3, "wrist_roll": 0.0}
REST = {"shoulder_pan": 0.0, "shoulder_lift": 0.04, "elbow_flex": -0.04, "wrist_flex": 0.0, "wrist_roll": 0.0}  # SRDF 'rest'
AT_REST = 0.08     # rad: every joint this close to REST counts as resting
OBJ, TGT = "object", "target"
BLUE = (0.15, 0.55, 1.0, 1.0)      # same colour for the RViz box and MoveIt's copy of it


def yaw_q(yaw):
    return Quaternion(w=math.cos(yaw / 2), z=math.sin(yaw / 2))


def q_yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def copy_box(b):
    return Box(b.x, b.y, b.yaw, b.size)


# English result messages of the API (the voice system can speak them)
MESSAGES = {
    "UNREACHABLE": "I cannot reach that position.",
    "TOO_WIDE": "The object is too wide for my gripper.",
    "MISSED": "I missed the object.",
    "COLLISION": "My arm was blocked or would hit the table.",
    "BUSY": "I am busy with another task.",
    "HOLDING": "I am already holding something.",
    "NOT_HOLDING": "I am not holding anything.",
    "HOT": "My motors are too hot. Please wait a minute.",
    "STOPPED": "Stopped.",
    "REPLACED": "Another program moved my arm, so I stopped.",
    "FAILED": "Something went wrong.",
}
SUCCESS = {"pick": "I have picked it up.", "place": "I have put it down.", "pick_place": "Done, I moved it.",
           "home": "I am at the ready pose.", "rest": "I am resting.", "touch": "Done.",
           "pick_bag": "I have the bag."}


class PickPlace(Node):
    def __init__(self):
        super().__init__("so100_pick_place")
        self.declare_parameter("robot_description", "")
        self.declare_parameter("calibration", "")
        self.declare_parameter("table_z", -0.075)
        self.declare_parameter("sim", False)
        self.declare_parameter("object_size", [0.03, 0.03, 0.05])
        self.declare_parameter("line_speed", 0.03)        # m/s on the straight approach/lift lines
        self.declare_parameter("velocity_scaling", 0.3)   # MoveIt moves
        self.declare_parameter("rest_after_s", 120.0)     # idle time before folding into the rest pose, 0 = off
        self.declare_parameter("deck", [0.0, 0.0, 0.0, 0.0])  # arm on the robot: [front, back, left, right] m
        self.declare_parameter("deck_z", -0.068)          # deck top below the arm base (m)
        self.declare_parameter("bag_carry_side", 90.0)    # deg: 0 = front, 90 = left, -90 = right
        self.declare_parameter("bag_bottom_min", 0.36)    # bag bottom above the floor while carried (m)
        self.declare_parameter("bag_speed", 0.08)         # m/s of the grasp point on the bag carry moves
        self.declare_parameter("bag_miss_below", 0.06)    # rad: bag jaws closing below this = missed, 0 = no check
        self.declare_parameter("bag_tries", 3)            # grasp tries, shifted across the part after a miss
        self.declare_parameter("bag_try_step", 0.05)      # m between tries
        # /arm/pick_bag method: 'camera' = the bag's front wall at its rim, checked with the fixed front webcam
        # (bag_camera.py; forearm camera until 2026-11-18), 'planner' = the planned grasp at the goal's position
        # (probe / shifts / carry at the side)
        self.declare_parameter("bag_method", "camera")
        self.declare_parameter("bag_cam_url", "http://localhost:8082/snap.jpg")    # front webcam (webcam_view.py)
        self.declare_parameter("bag_xtion_url", "http://192.168.31.21:8081/stream")  # Xtion MJPEG on the base Pi
        self.declare_parameter("bag_cam_tries", 4)        # camera grasp tries (a little further in / out each)
        # the HMC team's calibration of the fixed front webcam (robot frame, origin at the turning centre)
        self.declare_parameter("bag_front_cam_file", "~/hmc-follow/front_camera.json")
        # the arm's pan axis in that frame: LiDAR 0.175 m ahead of the turning centre (slam_bridge.LASER_X)
        # + the pan axis from the LiDAR (bag_robot_pan)
        self.declare_parameter("bag_front_cam_pan", [0.187, 0.0145])
        # the arm's pan axis in the robot frame bag_arm sends goals in (origin under the LiDAR; HMC Mount.pan_xy):
        # only for the log lines in that frame
        self.declare_parameter("bag_robot_pan", [0.012, 0.0145])
        self.declare_parameter("bag_cam_close_h", 0.19)   # m above the floor: finger tips at the close (paper rim
        #                                                   27 cm on the test bag: about 8 cm of wall in the jaws;
        #                                                   0.25 and 0.22 slipped out in the lift, 2026-11-18)
        self.declare_parameter("bag_cam_half_depth", 0.055)   # m: bag centre -> its front wall (test bag 11 cm deep)
        # front camera bag finding (bag_camera.locate): the front wall's rim height above the floor (test bag
        # 27 cm) and the rim width the image must show for a standing bag (test bag: 0.178 m measured from two
        # pictures at 11 and 21 cm from the deck, 2026-11-18)
        self.declare_parameter("bag_cam_rim_h", 0.27)
        self.declare_parameter("bag_cam_width", 0.18)
        # the closing axis must line up with the wall's normal at least this well (cos; the good pick of
        # 2026-11-18 had 0.89 at pitch -60, the miss at pitch -50 had 0.78)
        self.declare_parameter("bag_cam_min_align", 0.85)
        # (forward, up) m added to the close pose asked of the arm: on 2026-11-18 the gap middle (forward
        # kinematics) came out 1.1-1.4 cm further and 1.3-1.4 cm higher than asked in all 5 picks (wrist roll
        # ~185 deg, tips ~31 cm out), and the inner jaw pushed the bag before the close
        self.declare_parameter("bag_cam_close_bias", [-0.012, -0.014])
        # test: both methods stop with the jaws OPEN at the close height and hold still (no close, no lift)
        self.declare_parameter("bag_hover_only", False)
        # Bag top unknown (PickBag grasp_pose z NaN): found by touch (probe_bag). Every value is a first guess,
        # to be measured on the robot from the 'probe k/n' log lines of one real run.
        self.declare_parameter("bag_probe_top", 0.40)     # m above the floor the closed tips start (or the highest
        #                                                   the arm reaches there): over the tallest top it grasps (37 cm)
        self.declare_parameter("bag_probe_bottom", 0.12)  # m above the floor: the tips go no lower
        self.declare_parameter("bag_probe_step", 0.01)    # m per step = the most it goes into a bag before a reading
        self.declare_parameter("bag_probe_speed", 0.02)   # m/s on a probe step
        # first steps not used: the arm still settles after the MoveIt move (real run 2026-11-15: shoulder +13,
        # +12, then +2..+4 %)
        self.declare_parameter("bag_probe_settle_steps", 2)
        self.declare_parameter("bag_probe_base_steps", 3)  # next steps (free air): their readings are the baseline
        # % change of the shoulder_lift / elbow_flex load = contact (0 = off). 8 % of the 12 V STS3215's 30 kg.cm
        # would be about 1 N up on the tips at the shoulder's 25 cm lever, if load % is that share of the torque.
        self.declare_parameter("bag_probe_load_jump", 8.0)
        # rad rise of their tracking error = contact (0 = off): ~5 mm at the tips. The driver itself fails a move
        # at 0.06 rad short at the end or 0.14 rad behind on the way (follow_error).
        self.declare_parameter("bag_probe_error_jump", 0.02)
        self.declare_parameter("bag_probe_backoff", 0.02)  # m back up after the contact
        # finger tips this far below a probed top (the bag grasp uses ik.GRIP_DEPTH = 2.8 cm, fingers ~3 cm long)
        self.declare_parameter("bag_probe_grip_depth", 0.028)
        # Held by weight: a bag grasp whose jaws read closed (a thin paper rim reads like an empty close,
        # 0.043-0.051 rad on 2026-11-15) goes back up the approach with the jaws closed; if the shoulder_lift /
        # elbow_flex load there changed by more than this % from the empty-jaws reading at the same pose before
        # going down, it holds the bag (0 = off, the old miss rule only). First guess: 0.1 kg = 0.98 N at the
        # approach pose levers (model, 24-34 cm in front: shoulder 21-28.5 cm, elbow 12-18.5 cm) is 0.21-0.28 N.m
        # = 7-9.5 % shoulder, 4-6 % elbow of the STS3215's 2.94 N.m (30 kg.cm); 0.3 kg three times that. Free air
        # on 2026-11-15 spread 2-6 % (shoulder), 2-5 % (elbow).
        # Unmeasured: on 2026-11-15 13:25 an empty reading taken while settling and arriving from above read 8 % below
        # the closed one arriving from below (nothing lifted) -> both readings now arrive from below and settle.
        # OFF since 13:40 the same day: again nothing lifted (LiDAR and Xtion unchanged) but empty 18,12,9,6,2,4 ->
        # 3 against closed 18,16 -> 17: the shoulder load decays for 5 to 6 s at rest, so a quick reading looks like
        # weight. Two false "holding" out of two; a false hold sends the robot home empty. Turn on only after a
        # measured real hold and a reading that waits out the decay.
        self.declare_parameter("bag_weight_load", 0.0)
        # Before the empty-jaws reading the hand dips this far down the approach line and comes back, so it also
        # arrives from below. At most 1.5 cm: the approach starts with the tips 2 cm above the top (approach_dist).
        self.declare_parameter("bag_weight_dip", 0.015)
        self.declare_parameter("bag_weight_settle_tol", 2.0)  # % two readings in a row may differ: settled
        self.declare_parameter("bag_weight_max_reads", 6)     # status lines (1 Hz) at most for one weight reading
        # safe = the parameters above. fast = motion_profile.FAST_PROFILE, after a supervised arm test.
        self.declare_parameter("speed_profile", "safe")
        # when this file exists, an unknown bag top does not run the 0.02 m/s touch probe
        self.declare_parameter("gripper_camera_file", "~/hmc-follow/gripper_camera.json")
        self.declare_parameter("bag_default_top", 0.26)  # m above the floor, used only when that probe is skipped
        self.declare_parameter("pick_log_dir", "~/hmc-follow/picks")  # per-pick JSON (phase timings)
        p = lambda n: self.get_parameter(n).value
        self.table_z = float(p("table_z"))
        self.sim = bool(p("sim"))
        limits, self.grip_max = None, 1.2
        if p("calibration"):
            try:
                cal = Calibration.load(p("calibration"))
                limits = {n: cal.rad_limits(n) for n in ARM}
                self.grip_max = cal.rad_limits("gripper")[1]
            except (OSError, ValueError) as exc:
                self.get_logger().warn(f"calibration not used ({exc}); URDF limits only")
        try:
            from ament_index_python.packages import get_package_share_directory
            mesh_dir = get_package_share_directory("so100_moveit_config") + "/meshes"
        except Exception:  # noqa: BLE001  (no meshes -> finger-tip check only)
            mesh_dir = None
        self.kin = Kinematics(p("robot_description"), limits, mesh_dir)
        if not self.kin.meshes:
            self.get_logger().warn("no meshes: only the finger tips are checked against the table")
        if limits:   # never wider than the URDF (which launch already narrowed to the real range)
            self.kin.limits = {n: (max(lo, self.kin.urdf_limits[n][0]), min(hi, self.kin.urdf_limits[n][1]))
                               for n, (lo, hi) in self.kin.limits.items()}
        deck = [float(v) for v in p("deck")]
        self.deck = deck_box(self.kin, *deck, float(p("deck_z")), self.table_z) if any(deck) else None
        self.planner = GraspPlanner(self.kin, self.table_z, [self.deck] if self.deck else [])
        self.cam = CameraBag(self)
        # the rest pose inside the real (calibrated) travel
        self.rest_q = {n: min(max(v, self.kin.limits[n][0] + 0.02), self.kin.limits[n][1] - 0.02)
                       for n, v in REST.items()}

        size = tuple(p("object_size"))
        x, y = self.kin.from_user(0.20, 0.0)
        tx, ty = self.kin.from_user(0.20, -0.10)
        self.boxes = {OBJ: Box(x, y, self.kin.heading0, size), TGT: Box(tx, ty, self.kin.heading0, size)}
        self.joints = {}
        self.busy = False
        self.task, self.step, self.goal_handle, self.on_lifted = "", "", None, None
        self.cancel_requested = False
        self.moved = False
        self.last = ("SUCCESS", "")
        self.driver_status, self.driver_seen = "", -1e9
        self.idle_since = time.monotonic()  # end of the last task (auto-rest timer)
        self.hold_still = ""                # why the arm must not fold by itself (stopped / object maybe in jaws)
        self.preempt = False                # a new task asked the running fold to give way
        self.current = None                 # goal handle of the running motion (to cancel a fold)
        self.was_torque = False
        self.attached = False
        self.check = ""
        self.lock = threading.Lock()
        self.grasped = None              # plan of this task's grasp once the jaws closed on something
        self.grip_reading = (math.nan, math.nan)   # (gripper rad, load %) right after this task's last grasp

        cb = ReentrantCallbackGroup()
        self.status_pub = self.create_publisher(String, "so100/pick_place/status", 10)
        self.map_pub = self.create_publisher(Marker, "so100/place_map", 1)
        self.map_gen, self.map_key, self.map_points, self.map_note = 0, None, [], ""
        self.create_subscription(JointState, "joint_states", self.on_js, 20, callback_group=cb)
        self.arm = ActionClient(self, FollowJointTrajectory, "arm_controller/follow_joint_trajectory", callback_group=cb)
        self.grip = ActionClient(self, GripperCommand, "gripper_controller/gripper_cmd", callback_group=cb)
        self.move = ActionClient(self, MoveGroup, "move_action", callback_group=cb)
        self.scene = self.create_client(ApplyPlanningScene, "apply_planning_scene", callback_group=cb)
        self.create_service(Trigger, "so100/pick_place", self.on_pick_place, callback_group=cb)
        self.create_service(Trigger, "so100/touch", self.on_touch, callback_group=cb)
        self.holding = None              # plan of the object in the jaws (voice 'carry' -> 'drop')
        self.event_pub = self.create_publisher(String, "/robot/event", 10)
        self.create_subscription(Empty, "/robot/carry_bag", self.on_carry, 10, callback_group=cb)
        self.create_subscription(Empty, "/robot/drop_bag", self.on_drop, 10, callback_group=cb)
        # ---- /arm/* API
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.feedback_type = {"pick": Pick.Feedback, "place": Place.Feedback, "pick_place": PickPlaceAction.Feedback,
                              "home": Home.Feedback, "rest": Rest.Feedback, "pick_bag": PickBag.Feedback}
        for action, name, cbk in ((Pick, "pick", self.exec_pick), (Place, "place", self.exec_place),
                                  (PickBag, "pick_bag", self.exec_pick_bag),
                                  (PickPlaceAction, "pick_place", self.exec_pick_place), (Home, "home", self.exec_home),
                                  (Rest, "rest", self.exec_rest)):
            ActionServer(self, action, f"/arm/{name}", execute_callback=cbk, cancel_callback=self.on_cancel,
                         callback_group=cb)
        self.driver_stop = self.create_client(Trigger, "so100/stop", callback_group=cb)
        self.driver_enable = self.create_client(SetBool, "so100/enable", callback_group=cb)
        self.create_service(Trigger, "/arm/stop", self.on_api_stop, callback_group=cb)
        self.create_service(SetBool, "/arm/torque", self.on_api_torque, callback_group=cb)
        self.state_pub = self.create_publisher(ArmState, "/arm/state", 10)
        self.create_subscription(String, "so100/status", self.on_driver_status, 10, callback_group=cb)
        self.create_timer(0.5, self.publish_state, callback_group=cb)
        self.create_timer(1.0, self.auto_rest, callback_group=cb)
        self.add_on_set_parameters_callback(self.on_params)
        self.server = InteractiveMarkerServer(self, "so100_objects")
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

    def gap_point(self, q, local_y=TIP_Y):
        """Middle of the fully open jaw gap (one jaw moves: not the tool axis) at the finger tips (or local_y)
        of pose q -> (forward, left, height above the floor) from the pan axis, in m."""
        p = self.kin.gripper_point(q, (GAP_MID, local_y, 0.0))
        f, l = self.kin.to_user(p)
        return f, l, float(p[2] - self.table_z)

    def grasp_report(self, tag, want=None, planned=None):
        """Log where the open gap really is (forward kinematics of the measured joints) against what was asked
        (want: gap middle at the tips, (forward, left, height) from the pan axis; planned: the planned joint pose),
        in the arm's user frame and in bag_arm's robot frame (origin under the LiDAR). Returns a short text."""
        q = {j: float(self.joints.get(j, 0.0)) for j in ARM}
        f, l, h = self.gap_point(q)
        rx, ry = (float(v) for v in self.get_parameter("bag_robot_pan").value)
        g = self.kin.frames(q)["gripper"]
        ax = self.kin.to_user(self.kin.pan_point + g[:2, 0])     # closing axis, horizontal part
        close_yaw = math.degrees(math.atan2(ax[1], ax[0]))
        close_tilt = math.degrees(math.asin(max(-1.0, min(1.0, float(g[2, 0])))))
        tool_pitch = math.degrees(math.asin(max(-1.0, min(1.0, float(-g[2, 1])))))
        lines = [f"{tag}: gap middle at the tips fwd {f:.3f} left {l:+.3f} h {h:.3f} (robot x {rx + f:.3f} "
                 f"y {ry + l:+.3f}); closing axis yaw {close_yaw:+.0f} deg (robot frame, 0 = ahead), tilt "
                 f"{close_tilt:+.0f} deg; tool pitch {tool_pitch:+.0f} deg; wrist_roll "
                 f"{math.degrees(q['wrist_roll']):+.0f} deg"]
        short = f"ช่องก้ามจริง x {rx + f:.3f} y {ry + l:+.3f} สูง {h:.3f} ม."
        for name, w in (("asked", want), ("planned", self.gap_point(planned) if planned else None)):
            if w is not None:
                d = np.subtract((f, l, h), w) * 100
                lines.append(f"  {name} fwd {w[0]:.3f} left {w[1]:+.3f} h {w[2]:.3f} -> real - {name} "
                             f"fwd {d[0]:+.1f} left {d[1]:+.1f} up {d[2]:+.1f} cm")
                short += f", คลาดจากที่สั่ง หน้า {d[0]:+.1f} ซ้าย {d[1]:+.1f} สูง {d[2]:+.1f} ซม."
        for s in lines:
            self.get_logger().info(s)
        return short
        self.publish_status()

    def publish_status(self):
        lines = []
        for key, label in ((OBJ, "ของ"), (TGT, "จุดวาง")):
            f, l_ = self.kin.to_user((self.boxes[key].x, self.boxes[key].y))
            yaw = math.degrees(self.boxes[key].yaw - self.kin.heading0)
            lines.append(f"{label}: หน้า {f * 100 + 0.0:.1f} ซม. ซ้าย {round(l_ * 100, 1) + 0.0:.1f} ซม. หมุน {round(yaw) + 0:.0f}°")
        s = self.boxes[OBJ].size
        lines.append(f"ขนาด ลึก×กว้าง×สูง {s[0] * 100:.1f}×{s[1] * 100:.1f}×{s[2] * 100:.1f} ซม.")
        lines.append(self.check)
        if self.map_note:
            lines.append(self.map_note)
        self.status_pub.publish(String(data="\n".join(lines)))

    def on_params(self, params):
        for prm in params:
            if prm.name == "object_size":
                v = [float(x) for x in prm.value]
                if len(v) != 3 or min(v) < 0.005 or max(v) > 0.3:
                    return SetParametersResult(successful=False, reason="object_size = [ลึก, กว้าง, สูง] เมตร (0.005..0.3)")
                if self.busy:
                    return SetParametersResult(successful=False, reason="กำลังหยิบ-วางอยู่")
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
        for key, colour, label in ((OBJ, BLUE, "ของ"), (TGT, (0.75, 0.3, 0.95, 0.45), "จุดวาง")):
            b = self.boxes[key]
            im = InteractiveMarker()
            im.header.frame_id = "base"
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
            drag = InteractiveMarkerControl(name="drag", orientation=vertical, always_visible=True,
                                            interaction_mode=InteractiveMarkerControl.MOVE_PLANE)
            drag.markers = [box, text]
            turn = InteractiveMarkerControl(name="turn", orientation=vertical,
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
        self.map_note = "กำลังคำนวณพื้นที่วางได้ (สีเขียวใน RViz)..."
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
        self.map_note = "" if found else "✘ ไม่มีจุดไหนวางของชิ้นนี้ได้ (หยิบไม่ได้ หรือของใหญ่เกิน)"
        self.publish_map()
        self.publish_status()

    def publish_map(self):
        m = Marker(ns="place_map", id=0, type=Marker.CUBE_LIST)
        m.header.frame_id = "base"
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
            self.say(f"✘ {exc}")

    # ---------------------------------------------------------------------------- planning scene
    def init_scene(self):
        if self.scene_ready or not self.scene.service_is_ready():
            return
        self.scene_ready = True
        if self.deck is not None:          # the robot body under the arm
            lo, hi = self.deck
            co = CollisionObject(id="robot_body", operation=CollisionObject.ADD)
            co.header.frame_id = "base"
            co.primitives.append(SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[float(v) for v in hi - lo]))
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = (float(v) for v in (lo + hi) / 2)
            pose.orientation.w = 1.0
            co.primitive_poses.append(pose)
            scene = PlanningScene(is_diff=True)
            scene.world.collision_objects.append(co)
            scene.object_colors.append(ObjectColor(id="robot_body", color=ColorRGBA(r=0.4, g=0.4, b=0.45, a=0.6)))
            self.apply(scene, wait=True)
        self.update_scene_object()

    def collision_box(self, box, op=CollisionObject.ADD, top_part=None):
        """MoveIt copy of the object. top_part = only the top this many m of it (a bag held by its top:
        the rest hangs and swings, the bag paths check it themselves)."""
        co = CollisionObject(id=OBJ, operation=op)
        co.header.frame_id = "base"
        if op == CollisionObject.ADD:
            h = min(top_part, box.size[2]) if top_part else box.size[2]
            co.primitives.append(SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[*box.size[:2], h]))
            pose = Pose()
            pose.position.x, pose.position.y = box.x, box.y
            pose.position.z = self.table_z + box.size[2] - h / 2
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

    def attach(self, box, top_part=None):
        scene = PlanningScene(is_diff=True)
        scene.robot_state.is_diff = True
        aco = AttachedCollisionObject(link_name="gripper", touch_links=["gripper", "jaw", "wrist"])
        aco.object = self.collision_box(box, top_part=top_part)
        scene.robot_state.attached_collision_objects.append(aco)
        scene.world.collision_objects.append(self.collision_box(box, CollisionObject.REMOVE))
        return self.apply(scene, wait=True)

    def detach_to(self, box):
        scene = PlanningScene(is_diff=True)
        scene.robot_state.is_diff = True
        aco = AttachedCollisionObject(link_name="gripper")
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
            raise Failed(f"{what}: ไม่พบ action server", "FAILED")
        if self.preempt:
            raise Failed(f"{what}: ให้ทางงานใหม่", "STOPPED")
        handle = self.wait(client.send_goal_async(goal), 5.0)
        if handle is None or not handle.accepted:
            raise Failed(f"{what}: คำสั่งถูกปฏิเสธ", "FAILED")
        self.current = handle
        if self.preempt:                 # asked to give way while the goal was being sent
            handle.cancel_goal_async()
        try:
            res = self.wait(handle.get_result_async(), timeout)
        finally:
            self.current = None
        if self.preempt:
            raise Failed(f"{what}: ให้ทางงานใหม่", "STOPPED")
        if res is None:
            handle.cancel_goal_async()
            raise Failed(f"{what}: หมดเวลา", "FAILED")
        return res.result

    def active_speeds(self):
        """safe keeps velocity_scaling, line_speed and bag_speed. fast replaces them (motion_profile)."""
        name = str(self.get_parameter("speed_profile").value)
        current = {"joint_scale": float(self.get_parameter("velocity_scaling").value),
                   "approach_speed": float(self.get_parameter("line_speed").value),
                   "bag_speed": float(self.get_parameter("bag_speed").value)}
        return motion_profile.resolve(name, current)

    def moveit_to(self, q, what):
        goal = MoveGroup.Goal()
        r = goal.request
        req = motion_profile.ompl_request(self.active_speeds())
        r.group_name = "arm"
        r.pipeline_id = req["pipeline_id"]
        r.planner_id = req["planner_id"]
        r.num_planning_attempts = req["num_planning_attempts"]
        r.allowed_planning_time = req["allowed_planning_time"]
        r.max_velocity_scaling_factor = req["max_velocity_scaling_factor"]
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
            reason, api = {99999: ("หาเส้นทางไม่ได้ (ท่าปลายทางชนของ/โต๊ะในโมเดล หรือทางถูกบัง)", "COLLISION"),
                           -10: ("ท่าเริ่มต้นชนอยู่แล้วในโมเดล", "COLLISION"),
                           -12: ("ท่าปลายทางชนในโมเดล", "COLLISION"),
                           -4: ("แขนไปไม่ถึง (ติด/ชน หรือมอเตอร์ไม่ตาม)", "COLLISION"),
                           -6: ("หมดเวลา", "FAILED"),
                           -7: ("ถูกหยุดกลางทาง", "FAILED")}.get(code, (f"MoveIt code {code}", "FAILED"))
            raise Failed(f"{what}: {reason}", api)

    def _lin_constraints(self, fields):
        x, y, z = fields["position"]
        qx, qy, qz, qw = fields["orientation_xyzw"]
        header = Header(frame_id=fields["frame_id"])
        sphere = SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[fields["position_tolerance_m"]])
        at = Pose(position=Point(x=x, y=y, z=z), orientation=Quaternion(w=1.0))
        region = BoundingVolume(primitives=[sphere], primitive_poses=[at])
        pos = PositionConstraint(header=header, link_name=fields["link"], constraint_region=region, weight=1.0)
        ori = OrientationConstraint(
            header=header, link_name=fields["link"], orientation=Quaternion(x=qx, y=qy, z=qz, w=qw),
            absolute_x_axis_tolerance=fields["orientation_tolerance_rad"],
            absolute_y_axis_tolerance=fields["orientation_tolerance_rad"],
            absolute_z_axis_tolerance=fields["orientation_tolerance_rad"], weight=1.0)
        return Constraints(position_constraints=[pos], orientation_constraints=[ori])

    def moveit_lin(self, q, what, speed):
        """Pilz LIN from the current state to the tip pose of joint position q."""
        link, matrix = motion_profile.tip_frame(self.kin.frames(q))
        fields = motion_profile.lin_request(matrix, speed, link)
        goal = MoveGroup.Goal()
        r = goal.request
        r.group_name = fields["group_name"]
        r.pipeline_id = fields["pipeline_id"]
        r.planner_id = fields["planner_id"]
        r.num_planning_attempts = 1
        r.allowed_planning_time = float(self.active_speeds()["planning_time"])
        r.max_velocity_scaling_factor = fields["velocity_scale"]
        r.max_acceleration_scaling_factor = 0.6
        r.start_state.is_diff = True
        r.goal_constraints = [self._lin_constraints(fields)]
        goal.planning_options.plan_only = False
        res = self.run_action(self.move, goal, 60.0, what)
        code = res.error_code.val
        if code != 1:
            raise Failed(f"{what}: Pilz LIN code {code}", "FAILED")

    def straight(self, qs, what, speed=None):
        """Approach or lift: Pilz LIN to the last pose. The joint line runs if LIN does not."""
        speed = speed or self.active_speeds()["approach_speed"]
        try:
            self.moveit_lin(qs[-1], what, speed)
        except Exception as exc:  # noqa: BLE001 (a bad pose must not brick the pick: the joint line still runs)
            self.get_logger().warn(f"{what}: Pilz LIN failed ({exc}); sending the joint line")
            self.line(qs, what, speed)

    def line(self, qs, what, speed=None, step=None):
        """Send a joint path of equal steps (default: the 5 mm straight lines) with a smooth start/stop."""
        speed = speed or self.active_speeds()["approach_speed"]
        n = len(qs)
        total = 1.875 * n * (step or self.planner.STEP) / speed     # min-jerk peak speed = 1.875 x average
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.header.frame_id = PICK_PLACE     # the driver lets only us take a running arm move
        goal.trajectory.joint_names = list(ARM)
        goal.path_tolerance = [JointTolerance(name=j, position=0.2) for j in ARM]   # = exact path, no retime
        for i, q in enumerate(qs, 1):
            t = total * self.minjerk_time(i / n)
            pt = JointTrajectoryPoint(positions=[float(q[j]) for j in ARM])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            goal.trajectory.points.append(pt)
        res = self.run_action(self.arm, goal, total + 10.0, what)
        if res.error_code != 0:
            raise Failed(f"{what}: {res.error_string or res.error_code}", api_code(res.error_code))

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
                    self.busy, self.task, self.step, self.goal_handle = True, name, "", handle
                    self.cancel_requested, self.preempt = False, False
                    self.moved = False       # set by the first motion command; refusals keep the scene as is
                    self.grasped, self.grip_reading = None, (math.nan, math.nan)
                    self.phase_log = pick_log.PhaseLog()
                    break
                if self.task != "rest" or name == "rest" or time.monotonic() > end:
                    return "BUSY", MESSAGES["BUSY"], "กำลังทำงานอื่นอยู่"
                if not self.preempt:
                    self.preempt = True
                    if self.current is not None:
                        self.current.cancel_goal_async()
                    self.get_logger().info(f"{name}: หยุดการไปท่าพัก แล้วทำงานใหม่")
            time.sleep(0.05)
        if not auto:
            self.hold_still = ""         # a new command from someone: the arm may fold again later
        given_way = False
        code = "FAILED"
        try:
            if name not in ("rest", "home") and self.at_rest():
                self.leave_rest()
            done = fn()
            code, detail = "SUCCESS", done or "เสร็จ"
            self.say(f"✔ {detail}")
        except (Failed, PlanError) as exc:
            code, detail = getattr(exc, "code", "FAILED"), str(exc)
            if self.cancel_requested:
                code = "STOPPED"
            self.say(f"✘ หยุด: {detail}")
            if self.attached:
                self.hold_still = "อาจมีของค้างในก้าม"
            if self.moved and name != "rest":
                self.after_failure()
        except Exception as exc:  # keep the node alive and tell the user  # noqa: BLE001
            code, detail = "FAILED", repr(exc)
            self.say(f"✘ ผิดพลาด: {detail}")
            self.get_logger().error(detail)
            if self.attached:
                self.hold_still = "อาจมีของค้างในก้าม"
            if self.moved and name != "rest":
                self.after_failure()
        finally:
            self._write_phase_log(name, code)
            with self.lock:
                given_way = self.preempt
                self.busy, self.task, self.step, self.goal_handle = False, "", "", None
                self.preempt = False
                self.idle_since = time.monotonic()
        if code == "STOPPED" and not given_way:
            self.hold_still = "ถูกสั่งหยุด"      # someone stopped it on purpose: stay put until the next command
        msg = MESSAGES.get(code, MESSAGES["FAILED"]) if code != "SUCCESS" else SUCCESS[name]
        if not (auto or given_way):
            self.last = (code, msg)
        return code, msg, detail

    def start(self, name, fn, after=None, auto=False):
        """Run a task in the background (panel buttons, voice topics). False if the arm is busy
        (a fold into the rest pose does not count: the new task replaces it)."""
        if self.busy and (self.task != "rest" or name == "rest"):
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
        self.step = "leave_rest"
        self.say("ออกจากท่าพัก (ไปท่า ready)...")
        self.moveit_to(READY, "ออกจากท่าพัก")

    def do_rest(self):
        if self.holding:
            raise Failed("ถือของอยู่: วางก่อนค่อยไปท่าพัก", "HOLDING")
        if self.at_rest():
            return "อยู่ท่าพักแล้ว"
        self.progress(1, 2, "close", "หุบก้าม...")
        self.gripper(0.0, "หุบก้าม")
        self.progress(2, 2, "rest", "ไปท่าพัก...")
        self.moveit_to(self.rest_q, "ไปท่าพัก")
        return "อยู่ท่าพัก"

    def auto_rest(self):
        """Fold into the rest pose after rest_after_s seconds without a task (not while holding)."""
        torque, fault = self.driver_flags()
        if torque and not self.was_torque:
            self.idle_since = time.monotonic()      # torque just came back on: count from now
        self.was_torque = torque
        after = float(self.get_parameter("rest_after_s").value)
        if (after <= 0 or self.busy or self.holding or self.hold_still or not torque or fault
                or time.monotonic() - self.driver_seen > 3.0 or self.at_rest()):
            return
        if time.monotonic() - self.idle_since >= after:
            self.get_logger().info(f"ว่างมา {after:.0f} วิ: ไปท่าพัก")
            self.start("rest", self.do_rest, auto=True)

    def _write_phase_log(self, name, code):
        """One JSON per pick under pick_log_dir: each progress step and how long it took."""
        log = getattr(self, "phase_log", None)
        if log is None or name not in ("pick", "pick_bag", "pick_place", "place"):
            return
        try:
            speeds = self.active_speeds()
            profile = str(self.get_parameter("speed_profile").value)
            directory = os.path.expanduser(str(self.get_parameter("pick_log_dir").value))
        except Exception:  # noqa: BLE001
            return
        body = log.finish(task=name, ok=code == "SUCCESS", code=code, profile=profile, speeds=speeds)
        try:
            path = pick_log.write_pick(directory, body, time.strftime("%Y%m%d-%H%M%S"), name)
            self.get_logger().info("pick log %s" % path)
        except OSError as exc:
            self.get_logger().warn("pick log not written: %s" % exc)

    def progress(self, i, n, key, text):
        self.step = key
        log = getattr(self, "phase_log", None)
        if log is not None:
            log.mark(key)
        self.say(f"{i}/{n} {text}")
        h = self.goal_handle
        if h is not None:
            fb = self.feedback_type[self.task]()
            fb.step, fb.step_index, fb.step_count = key, i, n
            h.publish_feedback(fb)

    MISS_DEFAULT = 0.05   # rad: the planner's own miss threshold (ik.py _pick_part) for a plan without one

    def after_failure(self):
        """A task failed or was stopped after the arm moved. If the jaws had closed on something and still
        read open above the miss threshold, it is still held: keep `holding` (a new pick then answers
        HOLDING instead of opening the jaws, and /arm/place can put it down). Otherwise forget it."""
        held = self.holding or self.grasped
        if held is not None and held.get("camera"):
            self.say("ยังถือถุงอยู่ (หยิบด้วยกล้อง): ไม่ลืม สั่งวางได้")
            return
        pos = self.joints.get("gripper")
        if held is not None and (held.get("by_weight")       # its jaws read closed on a thin rim from the start
                                 or still_held(pos, held.get("miss_below") or self.MISS_DEFAULT, held.get("jaw_open"))):
            if "carry" in held and all(n in self.joints for n in ARM):
                # a bag: the put-down path starts where the arm stopped, not at the carry pose
                held["carry"] = dict(held["carry"], q={n: float(self.joints[n]) for n in ARM})
            self.holding = held
            self.say(f"ยังถือของอยู่ (ก้าม {pos if pos is None else round(pos, 3)} rad): ไม่ลืม สั่งวางได้")
            return
        self.release_scene()

    def release_scene(self):
        """After a failure the object may be in the jaws or knocked over: drop MoveIt's copy of it
        (attached or not), so RViz / MoveIt can move the arm again. It is re-added on the next run."""
        self.holding = None
        scene = PlanningScene(is_diff=True)
        scene.robot_state.is_diff = True
        if self.attached:
            aco = AttachedCollisionObject(link_name="gripper")
            aco.object = CollisionObject(id=OBJ, operation=CollisionObject.REMOVE)
            scene.robot_state.attached_collision_objects.append(aco)
            self.attached = False
        scene.world.collision_objects.append(self.collision_box(self.boxes[OBJ], CollisionObject.REMOVE))
        self.apply(scene, wait=True)

    # ------------------------------------------------------------------------- panel buttons
    def on_pick_place(self, _req, res):
        o, t = self.boxes[OBJ], self.boxes[TGT]
        res.success = self.start("pick_place", lambda: self.do_pick_place(copy_box(o), copy_box(t)))
        res.message = "เริ่มหยิบ-วาง" if res.success else "กำลังทำงานอยู่"
        return res

    def on_touch(self, _req, res):
        res.success = self.start("touch", self.touch)
        res.message = "เริ่มแตะจุด" if res.success else "กำลังทำงานอยู่"
        return res

    def touch(self):
        b = self.boxes[OBJ]
        centre = np.array([b.x, b.y, self.table_z + 0.01 + (GRASP_Y - TIP_Y)])   # closed tips 1 cm above table
        q, err = None, "เอื้อมไม่ถึง"
        v = centre[:2] - self.kin.pan_point
        h = math.atan2(v[1], v[0])
        side = np.array([-math.sin(h), math.cos(h), 0.0])      # jaws close sideways
        for pitch in self.planner.PITCHES:
            if pitch > self.planner.NEAR_DOWN:
                break
            try:
                q = self.planner._pose(centre, pitch, side, 0.016, None, self.joints.get("wrist_roll", 0.0), x_off=0.0)
                self.planner.check_table([q], 0.016, "แตะจุด", 0)
                break
            except PlanError as exc:
                err, q = str(exc), None
        if q is None:
            raise Failed(f"แตะจุดไม่ได้: {err}", "UNREACHABLE")
        self.update_scene_object(present=False, wait=True)   # the box marks the spot, it is not an obstacle
        self.say("กำลังหุบก้าม...")
        self.gripper(0.0, "หุบก้าม")
        above = self.planner._pose(centre + [0, 0, 0.04], pitch, side, 0.016, None, q["wrist_roll"], x_off=0.0)
        self.say("ไปเหนือจุด...")
        self.moveit_to(above, "ไปเหนือจุด")
        self.say("ลงไปแตะ...")
        self.line(self.planner._line(centre + [0, 0, 0.04], centre, pitch, side, 0.016, q["_branch"], q["wrist_roll"],
                                     0.0), "ลงไปแตะ")
        f, l_ = self.kin.to_user(centre)
        return f"ปลายก้ามอยู่เหนือโต๊ะ 1 ซม. ที่ หน้า {f * 100:.1f} ซม. ซ้าย {l_ * 100:.1f} ซม. — วัดดูว่าตรงไหม"

    # ------------------------------------------------------------------------------ the tasks
    def show_object(self, obj):
        """Put the blue box (and MoveIt's copy) where the task's object is."""
        self.boxes[OBJ] = copy_box(obj)
        for b in self.boxes.values():
            b.size = obj.size
        self.make_markers()
        self.update_scene_object(present=True, wait=True)
        self.request_map()

    def do_pick(self, obj, mode="auto"):
        if self.holding:
            raise Failed("ถือของอยู่แล้ว: สั่งวางก่อน", "HOLDING")
        plan = self.planner.plan_pick(obj, self.joints or None, mode)
        self.show_object(obj)
        self.pick(plan, n=6)
        self.holding = plan
        self.progress(6, 6, "carry", "ยกมาถือไว้ (ท่า ready)...")
        self.moveit_to(READY, "ยกมาถือไว้")
        return f'ถือของอยู่ ({plan["mode"]})'

    def do_place(self, target, fallback_home_spot=False, released=None):
        if not self.holding:
            raise Failed("ไม่ได้ถือของอยู่", "NOT_HOLDING")
        if self.holding.get("camera"):
            done = self.cam.place(target)
            self.moveit_to(REST, "พับแขนกลับท่าพัก")
            return done
        x, y, yaw, size = self.holding["object"]
        obj = Box(x, y, yaw, size)
        if "carry" in self.holding and not getattr(target, "yaw_known", True):
            # a bag, no turn asked for: it keeps its turn relative to the arm (no wrist twist)
            px, py = self.kin.pan_point
            target.yaw = yaw + math.atan2(target.y - py, target.x - px) - math.atan2(y - py, x - px)
        put = self.planner.plan_bag_put if "carry" in self.holding else self.planner.plan_place
        try:
            plan = put(obj, self.holding, target)
            where = "ที่จุดวาง"
        except PlanError:
            if not fallback_home_spot:
                raise
            plan = put(obj, self.holding, obj)      # voice drop: back where it came from
            where = "คืนที่เดิม (จุดวางไกลเกิน)"
        path = None
        if "carry" in self.holding:           # a bag at the side: bring it round and down, then let go
            path = self.planner.plan_bag_place(self.holding, plan, self.holding["hang"])
            where += " (วางถุงลงพื้น)"
        self.boxes[TGT] = copy_box(target)
        self.make_markers()
        self.place(plan, first=1, n=3, released=released, path=path)
        return f"วางของแล้ว{where}"

    def do_pick_bag(self, obj):
        """Bag on the floor (table_z): hold its top part `obj`, lift, carry it at the side.
        obj height NaN (grasp_pose z NaN): the top is found by touch first (probe_bag),
        unless gripper_camera.json is calibrated, in which case bag_default_top is used."""
        if self.holding:
            raise Failed("ถือของอยู่แล้ว: สั่งวางก่อน", "HOLDING")
        if self.get_parameter("bag_method").value == "camera":
            return self.cam.pick(obj)
        probed = False
        if top_unknown(obj.size[2]):
            path = os.path.expanduser(str(self.get_parameter("gripper_camera_file").value))
            if gripper_camera_calibrated(path):
                top = float(self.get_parameter("bag_default_top").value)
                self.get_logger().info(
                    "gripper_camera.json is calibrated: skipping the 0.02 m/s touch probe, top %.3f m" % top)
                obj = Box(obj.x, obj.y, obj.yaw, (obj.size[0], obj.size[1], top))
            else:
                obj, probed = self.probe_bag(obj), True
        p = lambda n: float(self.get_parameter(n).value)
        depth = p("bag_probe_grip_depth") if probed else None
        # The jaws open fully (JAW_G_MAX, 7.5 cm) over the part; a miss (jaws close on nothing) is tried again
        # shifted across the part (yaw + 90 deg): 0, +step, -step, +2 step, ... (bag_tries in all), the step
        # at most what the open jaws cover minus 1 cm so the tries overlap (ik.bag_shifts). The position of a
        # soft bag is only known roughly, the part (the gathered mouth / handle roots) is soft and safe to touch.
        across = np.array([-math.sin(obj.yaw), math.cos(obj.yaw)])
        shifts = bag_shifts(obj.size[1], p("bag_try_step"), max(1, int(p("bag_tries"))))
        miss_below = p("bag_miss_below")
        err = None
        for i, d in enumerate(shifts):
            part = Box(obj.x + d * across[0], obj.y + d * across[1], obj.yaw, obj.size)
            try:
                plan = self.planner.plan_pick(part, self.joints or None, "top", wide=True, grip_depth=depth)
                hang = float(plan["grasp_centre"][2] - self.table_z)
                carry = self.planner.plan_carry(plan, hang, math.radians(p("bag_carry_side")),
                                                p("bag_bottom_min"))
            except PlanError as exc:
                err = err or exc
                self.say(f"ลองที่ {i + 1}/{len(shifts)} (เลื่อน {d * 100:+.0f} ซม.): วางแผนไม่ได้ ({exc}) — ข้าม")
                continue
            plan["carry"], plan["hang"] = carry, hang
            plan["miss_below"] = miss_below if miss_below > 0 else None
            self.say(f"ลองที่ {i + 1}/{len(shifts)}: เลื่อนขวางส่วนที่จับ {d * 100:+.0f} ซม.")
            self.show_object(part)
            try:
                self.pick(plan, n=6)
                break
            except Failed as exc:
                if exc.code != "MISSED":
                    raise
                err = exc
        else:
            if isinstance(err, PlanError):
                raise err
            raise Failed(f"จับถุงไม่ได้ใน {len(shifts)} ครั้ง (ก้ามหุบโดนแต่อากาศ)", "MISSED")
        self.holding = plan
        self.progress(6, 6, "carry", "ยกถุงไปถือไว้ข้างตัว...")
        self.line(carry["path"], "ยกถุงไปถือ", self.active_speeds()["bag_speed"], self.planner.CARRY_STEP)
        bottom = self.planner.held_point(carry["q"], plan)[2] - hang - self.table_z
        how = " ถือด้วยน้ำหนัก ก้ามอ่านว่าหุบ (held by weight, jaws read closed)" if plan.get("by_weight") else ""
        return f'ถือถุงอยู่ ({plan["mode"]}) ก้นถุงสูงจากพื้นประมาณ {bottom * 100:.0f} ซม.{how}'

    PROBE_SETTLE = 0.15   # s the arm stands still after a probe step before a load reading counts

    def probe_bag(self, obj):
        """Bag top height unknown: find it by touch. The closed jaws go straight down over the part in
        bag_probe_step steps from bag_probe_top to at most bag_probe_bottom above the floor (ik.plan_probe: no
        floor, no robot body). After each step the arm stands still and the PROBE_JOINTS loads (so100/status,
        1 Hz) and tracking errors are compared with the first bag_probe_base_steps steps (probe_contact); a step
        the arm cannot follow (COLLISION) is contact too. At contact the arm backs up bag_probe_backoff and the
        part's top = the measured tip height there. No contact: back up, MISSED. Every step is logged."""
        p = lambda n: float(self.get_parameter(n).value)
        base, step, speed = max(1, int(p("bag_probe_base_steps"))), p("bag_probe_step"), p("bag_probe_speed")
        skip = max(0, int(p("bag_probe_settle_steps")))
        load_jump, error_jump = p("bag_probe_load_jump"), p("bag_probe_error_jump")
        pr = self.planner.plan_probe(obj.x, obj.y, p("bag_probe_top"), p("bag_probe_bottom"), step,
                                     self.joints or None, min_steps=skip + base + 1)
        tips, n = pr["tip_h"], len(pr["tip_h"])
        start_h = tips[0] + step
        self.get_logger().info(f'probe plan: tips {start_h * 100:.0f} -> {tips[-1] * 100:.0f} cm above the floor in '
                               f'{n} steps, pitch {math.degrees(pr["pitch"]):.0f} deg; skip {skip}, base {base} steps, load_jump '
                               f'{load_jump} %, error_jump {error_jump} rad, joints {PROBE_JOINTS}')
        # MoveIt does not know the bag: its column below the start stands in for it on the way there
        self.show_object(Box(obj.x, obj.y, obj.yaw, (obj.size[0], obj.size[1], start_h - 0.02)))
        self.progress(1, 3, "close", "หุบก้าม (ใช้ปลายก้ามแตะหาความสูงถุง)...")
        self.gripper(0.0, "หุบก้าม")
        self.progress(2, 3, "probe_above", f"ไปเหนือถุง ปลายก้ามสูงจากพื้น {start_h * 100:.0f} ซม. (MoveIt)...")
        self.moveit_to(pr["start"], "ไปเหนือถุง")
        samples, done, contact = [], [pr["start"]], None
        for k, (qs, tip) in enumerate(zip(pr["steps"], tips)):
            self.progress(3, 3, "probe", f"แตะหาความสูงถุง ขั้น {k + 1}/{n}: ปลายก้ามลงไป {tip * 100:.0f} ซม. จากพื้น")
            blocked = ""
            try:
                self.line(qs, "แตะหาความสูงถุง", speed)
            except Failed as exc:
                if exc.code != "COLLISION":
                    raise
                blocked = str(exc)            # the arm could not follow: something is in the way
            done += qs
            loads, errors, measured = self.probe_reading(qs[-1], blocked)
            samples.append((loads, errors))
            hit = probe_contact(samples, base, load_jump, error_jump, skip)
            self.get_logger().info(
                f'probe {k + 1}/{n}: tip {tip * 100:.1f} cm (measured {measured * 100:.1f}) above the floor, '
                f'load% {PROBE_JOINTS[0]} {loads[0]:+.0f} {PROBE_JOINTS[1]} {loads[1]:+.0f}, '
                f'error rad {errors[0]:.3f} {errors[1]:.3f}{" BLOCKED " + blocked if blocked else ""}'
                f'{" CONTACT" if hit is not None else ""} | {self.driver_status}')
            if blocked or hit is not None:
                contact = measured if math.isfinite(measured) else tip
                break
        if contact is None:
            self.progress(3, 3, "probe_back", "แตะไม่โดนอะไร: ถอยกลับขึ้น...")
            self.line(list(reversed(done[:-1])), "ถอยขึ้นจากการแตะ")
            raise Failed(f'แตะหาความสูงถุงไม่โดนอะไร ลงไปถึง {tips[-1] * 100:.0f} ซม. จากพื้น: ไม่มีถุงตรงนี้ หรือถุงนุ่ม'
                         f'จนภาระไม่เปลี่ยน (ดู log "probe")', "MISSED")
        self.progress(3, 3, "probe_back", "แตะโดนยอดถุง: ถอยขึ้น...")
        back = int(round(p("bag_probe_backoff") / self.planner.STEP))
        self.line(list(reversed(done[:-1]))[:max(1, back)], "ถอยขึ้นจากยอดถุง", speed)
        self.say(f"ยอดถุงจากการแตะ: สูงจากพื้น {contact * 100:.1f} ซม.")
        self.get_logger().info(f"probe: top {contact * 100:.1f} cm above the floor (z {self.table_z + contact:.3f})")
        return Box(obj.x, obj.y, obj.yaw, (obj.size[0], obj.size[1], contact))

    def probe_reading(self, q_cmd, blocked=""):
        """(PROBE_JOINTS loads %, their tracking errors rad, measured tip height above the floor) with the arm
        standing still PROBE_SETTLE s: waits for the next so100/status line (1 Hz, its loads are read just
        before it is sent). A fault stops the probe, unless the step was blocked (the driver faults then)."""
        self.fresh_status("แตะหาความสูงถุง", self.PROBE_SETTLE, allow_fault=bool(blocked))
        now = dict(self.joints)
        errors = tuple(abs(q_cmd[j] - now[j]) if j in now else math.nan for j in PROBE_JOINTS)
        tip = math.nan
        if all(j in now for j in ARM):
            tip = float(self.kin.gripper_point({j: now[j] for j in ARM}, (0.0, TIP_Y, 0))[2]) - self.table_z
        return descent_loads(self.driver_status), errors, tip

    def fresh_status(self, what, after, allow_fault=False):
        """Wait for the next so100/status line sent `after` s from now or later (1 Hz, its loads are read just
        before it is sent). /arm/stop -> STOPPED; a fault -> FAILED unless allow_fault."""
        settled = time.monotonic() + after
        end = settled + 2.5
        while self.driver_seen < settled:
            if self.cancel_requested:
                raise Failed(f"{what}: ถูกสั่งหยุด", "STOPPED")
            if time.monotonic() > end:
                raise Failed(f"{what}: ไม่ได้ค่าภาระมอเตอร์จาก so100/status", "FAILED")
            time.sleep(0.02)
        fault = self.driver_flags()[1]
        if fault and not allow_fault:
            raise Failed(f"{what}: มอเตอร์ขัดข้อง ({fault})", "FAILED")

    def weight_loads(self, what):
        """Settled PROBE_JOINTS loads % with the arm standing still: status lines (1 Hz) until two in a row differ
        by at most bag_weight_settle_tol (their mean, settled_load), at most bag_weight_max_reads; None if not."""
        tol = float(self.get_parameter("bag_weight_settle_tol").value)
        cap = max(2, int(self.get_parameter("bag_weight_max_reads").value))
        reads, value = [], None
        while value is None and len(reads) < cap:
            self.fresh_status(what, self.PROBE_SETTLE)
            reads.append(descent_loads(self.driver_status))
            value = settled_load(reads, tol)
        self.get_logger().info(f'weight check {what}: load% {PROBE_JOINTS} readings {reads} -> '
                               f'{"settled " + str(value) if value else f"not settled in {cap} reads"} '
                               f'| {self.driver_status}')
        return value

    def weigh_bag(self, plan, free, i, n):
        """The bag jaws closed to a miss reading: go back up the approach to the pre-grasp pose with the jaws
        closed and compare the loads with `free` (empty jaws, same pose). True = the bag hangs in the jaws."""
        s = plan["steps"]
        self.progress(i, n, "weigh", "ก้ามหุบเกือบสุด: ยกขึ้นดูน้ำหนักถุงก่อน...")
        self.line(list(reversed(s["approach"][:-1])) + [s["pre"]], "ยกขึ้นดูน้ำหนัก")
        loaded = self.weight_loads("closed jaws back up (from below)")
        if loaded is None:
            self.get_logger().info("weight check: closed reading did not settle: nothing held")
            return False
        limit = float(self.get_parameter("bag_weight_load").value)
        held, change = held_by_weight(free, loaded, limit)
        self.get_logger().info(f"weight check: free {free} -> closed {loaded}, change {change} % (limit {limit} %, "
                               f"unmeasured first guess): " + ("held by weight, jaws read closed" if held else
                                                               "nothing held"))
        return held

    def do_pick_place(self, obj, target, mode="auto"):
        if self.holding:
            raise Failed("ถือของอยู่แล้ว: สั่งวางก่อน", "HOLDING")
        plan = self.planner.plan(obj, target, self.joints or None, mode)
        self.show_object(obj)
        self.boxes[TGT] = copy_box(target)
        self.make_markers()
        self.pick(plan)
        self.place(plan)
        return "หยิบ-วางเสร็จ (กล่องฟ้าย้ายไปตำแหน่งใหม่แล้ว)"

    def do_home(self):
        if self.holding and self.holding.get("camera"):
            raise Failed("ถือถุงอยู่: สั่งวางก่อน (ท่า ready จะเหวี่ยงถุงเข้าตัวหุ่น)", "HOLDING")
        if str(self.get_parameter("bag_method").value) == "camera":
            if not self.at_rest():
                self.progress(1, 1, "home", "พับแขนหลบกล้องหน้า (ท่าพัก)...")
                self.moveit_to(REST, "พับแขนหลบกล้องหน้า")
            return "อยู่ท่าพัก (พ้นกล้องหน้า)"
        self.progress(1, 1, "home", "กลับท่า ready...")
        self.moveit_to(READY, "กลับท่า ready")
        return "อยู่ท่า ready"

    def pick(self, plan, n=8):
        s = plan["steps"]
        obj = self.boxes[OBJ]
        self.say(f'แผน: {plan["mode"]} อ้าก้าม {plan["gap"] * 100:.1f} ซม.')
        self.update_scene_object(present=True, wait=True)
        self.progress(1, n, "move_above", "ไปเหนือของ (MoveIt)...")
        self.moveit_to(s["pre"], "ไปเหนือของ")
        self.progress(2, n, "open", "อ้าก้าม...")
        self.gripper(plan["jaw_open"], "อ้าก้าม")
        free = None          # bag: empty-jaws loads here, to weigh a grasp whose jaws read closed (weigh_bag)
        if "hang" in plan and not self.sim and float(self.get_parameter("bag_weight_load").value) > 0:
            # arrive at the pose from below, as the closed reading will (servo friction hysteresis)
            dip = min(float(self.get_parameter("bag_weight_dip").value), 0.015)
            k = min(int(round(dip / self.planner.STEP)), len(s["approach"]) - 1)
            if k > 0:
                self.line(s["approach"][:k], "ลงนิดก่อนชั่งก้ามว่าง")
                self.line(list(reversed(s["approach"][:k - 1])) + [s["pre"]], "ขึ้นกลับท่าเหนือถุง")
            free = self.weight_loads("empty jaws above the bag (from below)")
        self.progress(3, n, "approach", "ลงไปที่ของ...")
        self.straight(s["approach"], "ลงไปที่ของ")
        if "hang" in plan:               # a bag: where the open gap really is, before closing
            c = plan["grasp_centre"]
            cd = plan["close_dir"]
            cu = self.kin.to_user(self.kin.pan_point + np.asarray(cd[:2], float))
            self.get_logger().info(
                f"bag plan: object centre arm ({c[0]:.3f}, {c[1]:.3f}, {c[2]:.3f}), {plan['mode']}, jaws close along "
                f"yaw {math.degrees(math.atan2(cu[1], cu[0])):+.0f} deg (robot frame, 0 = ahead), opening "
                f"{plan['gap'] * 100:.1f} cm")
            time.sleep(0.3)              # joint_states of the settled arm
            report = self.grasp_report("planner at the close", planned=s["approach"][-1])
            if bool(self.get_parameter("bag_hover_only").value):
                self.hold_still = "ทดสอบ bag_hover_only: ค้างไว้ให้วัด"
                raise Failed(f"bag_hover_only: หยุดค้าง อ้าก้ามไว้ที่ระดับหนีบ ({report})", "STOPPED")
        self.progress(4, n, "grasp", "หนีบ...")
        r = self.gripper(0.0, "หนีบ")
        miss = plan.get("miss_below", 0.05)
        self.grip_reading = (float(r.position), float(r.effort))
        self.get_logger().info(f"grip: position {r.position:.3f} rad, load {r.effort:.1f} %, "
                               f"reached {r.reached_goal}, stalled {r.stalled}")
        if miss is None:
            self.say("ของบาง (ผนังถุง/หูหิ้ว): ก้ามหุบเกือบสุดเสมอ บอกไม่ได้ว่าหนีบโดน — ยกขึ้นต่อ")
        elif not self.sim and (r.reached_goal or r.position < miss):
            if free is None or not self.weigh_bag(plan, free, 4, n):
                self.say("✘ จับพลาด (ก้ามหุบสุด) — กำลังถอยขึ้น")
                self.gripper(plan["jaw_open"], "อ้าก้าม")
                if free is None:          # (weigh_bag already went back up)
                    self.line(list(reversed(s["approach"][:-1])) + [s["pre"]], "ถอยขึ้น")
                raise Failed("จับพลาด: ลองเลื่อนกล่องสีฟ้าให้ตรงของจริง หรือเช็กขนาด", "MISSED")
            plan["by_weight"] = True
            self.say("ก้ามอ่านว่าหุบ แต่ยกถุงขึ้นมาด้วย (น้ำหนักถึง): ถือต่อ")
        self.grasped = plan
        self.attach(obj, top_part=0.04 if "hang" in plan else None)
        self.attached = True
        self.progress(5, n, "lift", "ยกขึ้น...")
        if plan.get("by_weight"):        # at the pre-grasp pose already: on to the lift's end, where the carry starts
            self.line(self.planner.joint_path(s["pre"], s["lift"][-1], plan), "ยกขึ้น", None, self.planner.CARRY_STEP)
        else:
            self.straight(s["lift"], "ยกขึ้น")
        if self.on_lifted:
            self.on_lifted()

    def place(self, plan, first=6, n=8, released=None, path=None):
        s = plan["steps"]
        obj = self.boxes[OBJ]
        if path:
            self.progress(first, n, "move_above_target", "พาถุงไปเหนือจุดวาง...")
            self.line(path, "พาถุงไปเหนือจุดวาง", self.active_speeds()["bag_speed"],
                      self.planner.CARRY_STEP)
        else:
            self.progress(first, n, "move_above_target", "ไปเหนือจุดวาง (MoveIt)...")
            self.moveit_to(s["above_place"], "ไปเหนือจุดวาง")
        self.progress(first + 1, n, "lower", "วางลง...")
        self.line(s["lower"], "วางลง")
        self.gripper(plan["jaw_open"], "ปล่อย")
        if released:
            released()               # voice system waits only ~10 s: report before backing away
        # the object now stands at the target: move the blue box there
        obj.x, obj.y, obj.yaw = float(plan["place_centre"][0]), float(plan["place_centre"][1]), plan["final_yaw"]
        self.detach_to(obj)
        self.attached = False
        self.holding = self.grasped = None
        self.make_markers()
        self.request_map()
        self.progress(first + 2, n, "retreat", "ถอยออกแล้วกลับท่า ready...")
        self.line(s["retreat"], "ถอยออก")
        self.moveit_to(READY, "กลับท่า ready")

    # ------------------------------------------------------------------ voice system (topics)
    #   /robot/carry_bag (Empty)  -> pick the blue-box object and hold it at the ready pose
    #   /robot/drop_bag  (Empty)  -> put it down at the purple box (or back where it was) and let go
    #   /robot/event     (String) <- grab_success / grab_failed / drop_done / drop_failed
    # (older than the /arm/* API; kept until the voice system moves to the API)
    def event(self, text):
        self.event_pub.publish(String(data=text))
        self.get_logger().info(f"/robot/event: {text}")

    def on_carry(self, _msg):
        sent = []

        def lifted():                # report as soon as the object is off the table (voice waits 20 s)
            self.event("grab_success")
            sent.append(1)

        def fn():
            self.on_lifted = lifted
            try:
                return self.do_pick(copy_box(self.boxes[OBJ]))
            finally:
                self.on_lifted = None

        def after(_res):
            if not sent:
                self.event("grab_failed")
        if not self.start("pick", fn, after):
            self.say("✘ สั่งหยิบจากเสียง: กำลังทำงานอื่นอยู่")
            self.event("grab_failed")

    def on_drop(self, _msg):
        sent = []

        def released():
            self.event("drop_done")
            sent.append(1)

        def after(_res):
            if not sent:
                self.event("drop_failed")
        if not self.start("place", lambda: self.do_place(copy_box(self.boxes[TGT]), True, released), after):
            self.say("✘ สั่งวางจากเสียง: กำลังทำงานอื่นอยู่")
            self.event("drop_failed")

    # ---------------------------------------------------------------------------- /arm/* API
    def in_base(self, pose):
        """PoseStamped (any TF frame) -> PoseStamped in the arm base frame."""
        if pose.header.frame_id in ("", "base"):
            return pose
        try:
            return self.tf.transform(pose, "base", timeout=RclDuration(seconds=0.5))
        except Exception as exc:  # noqa: BLE001
            raise Failed(f'แปลงพิกัดจาก frame "{pose.header.frame_id}" ไม่ได้: {exc}', "FAILED")

    def to_bag(self, g):
        """PickBag goal -> Box of the part to hold, standing on the floor up to its top (grasp_pose z).
        z NaN = top unknown: the Box height is NaN and do_pick_bag finds it by touch."""
        unknown = top_unknown(g.grasp_pose.pose.position.z)
        if unknown and g.grasp_pose.header.frame_id not in ("", "base"):
            raise Failed('grasp_pose z = NaN (หาความสูงด้วยการแตะ) ใช้ได้เฉพาะ frame "base"', "FAILED")
        ps = self.in_base(g.grasp_pose)
        x, y, top = ps.pose.position.x, ps.pose.position.y, ps.pose.position.z
        h = top - self.table_z
        if not unknown and not 0.03 < h < 0.6:
            raise Failed(f"ส่วนบนของถุงสูงจากพื้น {h * 100:.0f} ซม.: ต้องอยู่ระหว่าง 3-60 ซม. "
                         "(grasp_pose z = ความสูงส่วนบนในพิกัดแขน)", "UNREACHABLE")
        thick, length = float(g.thickness) or 0.02, float(g.length) or 0.05
        if not (0.0 < thick <= 0.065 and 0.0 < length <= 0.4):
            raise Failed("thickness ต้อง 0-0.065 ม. และ length 0-0.4 ม.", "TOO_WIDE" if thick > 0.065 else "FAILED")
        q = ps.pose.orientation
        given = abs(q.x) + abs(q.y) + abs(q.z) + abs(q.w) >= 1e-9
        if not given:                                              # unknown: runs across the arm's view
            yaw = math.atan2(y - self.kin.pan_point[1], x - self.kin.pan_point[0]) + math.pi / 2
        else:
            yaw = q_yaw(q)
        box = Box(x, y, yaw, (length, thick, h))
        box.yaw_given = given        # the camera method then assumes the bag faces the robot
        return box

    def to_box(self, pose, size=None):
        """PoseStamped (any TF frame) -> Box in the arm base frame."""
        ps = self.in_base(pose)
        q = ps.pose.orientation
        x, y = ps.pose.position.x, ps.pose.position.y
        known = abs(q.x) + abs(q.y) + abs(q.z) + abs(q.w) > 1e-9
        if not known:                    # unknown: assume it faces the arm
            yaw = math.atan2(y - self.kin.pan_point[1], x - self.kin.pan_point[0])
        else:
            yaw = q_yaw(q)
        if size is None:                 # place target: the held object's size (a bag part may be taller)
            box = Box(x, y, yaw, self.boxes[OBJ].size)
            box.yaw_known = known
            return box
        size = tuple(float(v) for v in size)
        if len(size) != 3 or min(size) <= 0.0 or max(size) > 0.3:
            raise Failed("object_size ต้องเป็น [ลึก, กว้าง, สูง] เมตร (0..0.3)", "FAILED")
        return Box(x, y, yaw, size)

    def finish(self, handle, result, res, grip=(math.nan, math.nan)):
        code, msg, detail = res
        result.success = code == "SUCCESS"
        result.error_code, result.message = code, msg
        if hasattr(result, "gripper_pos"):     # PickBag: (gripper rad, load %) of the last grasp
            result.gripper_pos, result.gripper_load = grip
        self.get_logger().info(f"API {self.feedback_name(handle)} -> {code}: {detail}")
        if code == "STOPPED" and handle.is_cancel_requested:
            handle.canceled()
        elif result.success:
            handle.succeed()
        else:
            handle.abort()
        return result

    @staticmethod
    def feedback_name(handle):
        return type(handle.request).__name__.replace("_Goal", "")

    def api_task(self, name, handle, result, build):
        """Shared execute callback: build() turns the goal into a task function (may raise Failed)."""
        try:
            fn = build(handle.request)
        except (Failed, PlanError) as exc:
            code = getattr(exc, "code", "FAILED")
            return self.finish(handle, result, (code, MESSAGES.get(code, MESSAGES["FAILED"]), str(exc)))
        res = self.run_task(name, fn, handle)
        return self.finish(handle, result, res, self.grip_reading)

    def exec_pick(self, handle):
        return self.api_task("pick", handle, Pick.Result(), lambda g: (
            lambda obj=self.to_box(g.object_pose, g.object_size): self.do_pick(obj, g.grasp_mode or "auto")))

    def exec_place(self, handle):
        return self.api_task("place", handle, Place.Result(), lambda g: (
            lambda tgt=self.to_box(g.target_pose): self.do_place(tgt)))

    def exec_pick_place(self, handle):
        def build(g):
            obj = self.to_box(g.object_pose, g.object_size)
            tgt = self.to_box(g.target_pose, g.object_size)
            return lambda: self.do_pick_place(obj, tgt, g.grasp_mode or "auto")
        return self.api_task("pick_place", handle, PickPlaceAction.Result(), build)

    def exec_pick_bag(self, handle):
        return self.api_task("pick_bag", handle, PickBag.Result(), lambda g: (
            lambda obj=self.to_bag(g): self.do_pick_bag(obj)))

    def exec_home(self, handle):
        return self.api_task("home", handle, Home.Result(), lambda g: self.do_home)

    def exec_rest(self, handle):
        return self.api_task("rest", handle, Rest.Result(), lambda g: self.do_rest)

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
        res.success, res.message = True, "stopping the running task" if busy else "nothing was running; arm holds"
        return res

    def on_api_torque(self, req, res):
        if not req.data and self.busy:
            self.stop_motion()
        if not self.driver_enable.wait_for_service(timeout_sec=2.0):
            res.success, res.message = False, "arm driver not running"
            return res
        r = self.wait(self.driver_enable.call_async(SetBool.Request(data=req.data)), 5.0)
        if r is not None and r.success and not req.data and self.holding:
            self.release_scene()      # released by hand: the object is no longer known to be held
        if r is not None and r.success and not req.data:
            self.hold_still = ""      # someone handles the arm by hand now
        res.success = bool(r and r.success)
        res.message = (r.message if r else "no answer from the driver") + (
            "" if req.data else " (support the arm: it falls when the torque is off)")
        return res

    def on_driver_status(self, msg):
        self.driver_status, self.driver_seen = msg.data, time.monotonic()

    def driver_field(self, key):
        text = self.driver_status
        return text.split(key + "=", 1)[1].split(" ", 1)[0] if key + "=" in text else ""

    def driver_flags(self):
        """(torque_on, fault text) from the driver's so100/status line."""
        text = self.driver_status
        fault = "" if self.driver_field("fault") in ("", "none") else text.split("fault=", 1)[1].split(" load%")[0]
        return self.driver_field("torque") == "ON", fault

    def publish_state(self):
        st = ArmState()
        st.header.stamp = self.get_clock().now().to_msg()
        st.torque_on, st.fault = self.driver_flags()
        try:
            st.voltage_min = float(self.driver_field("voltage").split("-")[0].rstrip("V"))
            st.temp_max = float(self.driver_field("temp_max").rstrip("C"))
        except ValueError:
            pass
        st.holding = self.holding is not None
        st.task, st.step = self.task, self.step
        st.last_error_code, st.last_message = self.last
        st.gripper_pos = float(self.joints.get("gripper", math.nan))
        st.gripper_load = gripper_load(self.driver_status)
        if time.monotonic() - self.driver_seen > 3.0:
            st.state = "no_arm"
        elif st.fault:
            st.state = "fault"
        elif self.busy:
            st.state = "busy"
        elif st.holding:
            st.state = "holding"
        elif not st.torque_on:
            st.state = "torque_off"
        elif self.at_rest():
            st.state = "resting"
        else:
            st.state = "idle"
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


if __name__ == "__main__":
    main()
