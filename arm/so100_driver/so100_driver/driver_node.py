"""SO-100 ROS 2 driver.

Publishes   /joint_states                      (50 Hz, real encoder angles in URDF radians)
            /so100/status                       (std_msgs/String, 1 Hz: torque, voltage, temps, faults)
Actions     /arm_controller/follow_joint_trajectory   (MoveIt arm execution)
            /gripper_controller/gripper_cmd           (MoveIt / scripts: open-close, grasp with stall detect)
Topic in    /so100/joint_command (sensor_msgs/JointState): follow these angles directly (sliders)
Services    /so100/enable  (std_srvs/SetBool)  torque on (holds current pose) / off (arm goes limp!)
            /so100/stop    (std_srvs/Trigger)  cancel motion, hold where it is

Torque starts OFF; any command (trajectory, gripper, joint_command) turns it on at the current pose.
"""
import math
import threading
import time
import xml.etree.ElementTree as ET

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory, GripperCommand
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

from .calib import ARM_JOINTS, DEFAULT_PATH, JOINTS, Calibration
from .hardware import RealArm, SimArm
from .motion import Motion, Trajectory, ids_to_names, names_to_ids


def urdf_limits(urdf_xml):
    out = {}
    for j in ET.fromstring(urdf_xml).findall('joint'):
        lim = j.find('limit')
        if j.get('name') in JOINTS and lim is not None:
            out[j.get('name')] = (float(lim.get('lower')), float(lim.get('upper')))
    return out


def sim_calibration(limits):
    cal = Calibration.from_zero({n: 2048 for n in JOINTS}, {n: 1 for n in JOINTS})
    for n in JOINTS:
        lo, hi = limits.get(n, (-2.0, 2.0))
        cal.data[n]['min'] = max(0, int(cal.to_raw(n, lo)) - 20)
        cal.data[n]['max'] = min(4095, int(cal.to_raw(n, hi)) + 20)
    return cal


class Driver(Node):
    def __init__(self):
        super().__init__('so100_driver')
        p = self.declare_parameter
        sim = p('sim', False).value
        port = p('port', '/dev/serial/by-id/usb-1a86_USB_Single_Serial_58FA093250-if00').value
        calib_path = p('calibration', str(DEFAULT_PATH)).value
        self.rate = p('rate', 50.0).value
        max_speed = p('max_speed', 1.5).value              # rad/s hard cap per joint
        grip_limit = p('gripper_torque_limit', 500).value  # 0..1000, LeRobot uses 500
        self.min_voltage = p('min_voltage', 10.0).value
        # joints slower than this (rad/s, ~5 deg/s) on average are not made to creep (0 = off)
        self.min_joint_speed = p('min_joint_speed', 0.087).value
        robot_description = p('robot_description', '').value

        limits = urdf_limits(robot_description) if robot_description else {}
        if sim:
            self.calib = sim_calibration(limits)   # sim never needs the real arm's calibration
        else:
            self.calib = Calibration.load(calib_path)
            if not self.calib.complete():
                raise RuntimeError('calibration has no ranges yet: run `ros2 run so100_driver calibrate sweep`')
        # Collision / stall detection: reference and real joint differ > follow_error for follow_error_time.
        self.motion = Motion(self.calib, limits, max_speed=max_speed,
                             follow_error=p('follow_error', 0.14).value,          # rad (~8 deg) when slow/still
                             lag_time=p('lag_time', 0.25).value,                  # + 0.25 s x joint speed when moving
                             follow_error_time=p('follow_error_time', 0.5).value)  # s over the limit -> stop

        if sim:
            self.arm = SimArm(names_to_ids({n: self.calib.data[n]['zero'] for n in JOINTS}))
            self.arm.pos = {i: float(v) for i, v in names_to_ids(
                {n: self.calib.clamp_raw(n, self.calib.data[n]['zero']) for n in JOINTS}).items()}
        else:
            self.arm = RealArm(port, gripper_torque_limit=grip_limit,
                               p=p('servo_p', [12, 16, 16, 16, 16, 16]).value,  # base 12: less backlash hunting (P8 too soft)
                               d=p('servo_d', [32] * 6).value,
                               dead_zone=p('servo_dead_zone', [3, 3, 3, 1, 1, 1]).value)  # best in A/B trial
        self.arm.configure()
        self.sim = sim

        self.lock = threading.RLock()
        self.present = ids_to_names(self.arm.read_positions())
        self.enabled = any(self.arm.torque().values())
        self.motion.hold(self.present)
        self.fault = None
        self.health = {}
        self.read_errors = 0
        self.grip = None   # active gripper goal state
        self.hot_count = 0
        self.last_errors = {}
        self.max_temp = p('max_temp', 60).value

        cb = ReentrantCallbackGroup()
        self.js_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.status_pub = self.create_publisher(String, 'so100/status', 10)
        self.create_subscription(JointState, 'so100/joint_command', self.on_command, 10, callback_group=cb)
        self.create_service(SetBool, 'so100/enable', self.on_enable, callback_group=cb)
        self.create_service(Trigger, 'so100/stop', self.on_stop, callback_group=cb)
        ActionServer(self, FollowJointTrajectory, 'arm_controller/follow_joint_trajectory',
                     execute_callback=self.exec_traj, goal_callback=self.accept_traj,
                     cancel_callback=lambda _: CancelResponse.ACCEPT, callback_group=cb)
        ActionServer(self, GripperCommand, 'gripper_controller/gripper_cmd',
                     execute_callback=self.exec_grip, goal_callback=lambda _: GoalResponse.ACCEPT,
                     cancel_callback=lambda _: CancelResponse.ACCEPT, callback_group=cb)
        self.add_on_set_parameters_callback(self.on_params)
        self.last_tick = time.monotonic()
        self.create_timer(1.0 / self.rate, self.tick)
        self.create_timer(1.0, self.slow_tick)
        lims = ', '.join(f'{n} {math.degrees(self.motion.limits(n)[0]):.0f}..{math.degrees(self.motion.limits(n)[1]):.0f}deg'
                         for n in JOINTS)
        self.get_logger().info(f'{"SIM" if sim else "REAL"} arm ready, torque {"ON" if self.enabled else "OFF"}. Limits: {lims}')

    # ---------------------------------------------------------------- loop
    def tick(self):
        with self.lock:
            now = time.monotonic()
            dt = min(now - self.last_tick, 0.1)
            self.last_tick = now
            try:
                self.present = ids_to_names(self.arm.read_positions())
                self.read_errors = 0
            except Exception as exc:  # noqa: BLE001 - one lost packet must not kill the arm
                self.read_errors += 1
                if self.read_errors == 10:
                    self.set_fault(f'lost contact with servos: {exc}')
                return
            if self.enabled and self.fault is None:
                goals = self.motion.step(now, self.present, dt)
                if self.motion.fault:
                    self.set_fault(self.motion.fault)
                    self.motion.fault = None
                    goals = {n: float(self.present[n]) for n in JOINTS}
                speeds = None if self.fault else names_to_ids(self.motion.speeds)
                self.arm.write_goals(names_to_ids(goals), speeds)
                self.update_grip(now)
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = list(JOINTS)
            msg.position = [self.calib.to_rad(n, self.present[n]) for n in JOINTS]
            self.js_pub.publish(msg)

    STATUS_BITS = {0: 'voltage', 1: 'angle sensor', 2: 'overheat', 3: 'overcurrent', 5: 'overload'}

    def slow_tick(self):
        with self.lock:
            try:
                raw = self.arm.health()
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(f'health read failed: {exc}')
                return
        # Ignore impossible values from a corrupted read (once: 93C while the real value was 35C).
        good = {i: h for i, h in raw.items() if 4.0 <= h['voltage'] <= 16.0 and 0 <= h['temp'] <= 100}
        if len(good) < len(raw):
            self.get_logger().debug(f'ignored implausible health sample: {raw}')
        self.health.update(good)
        if not self.health:
            return
        volts = [h['voltage'] for h in self.health.values()]
        temps = [h['temp'] for h in self.health.values()]
        lag = math.degrees(self.motion.max_lag)
        self.motion.max_lag = 0.0          # report the worst lag of the last second, not since start
        if min(volts) < self.min_voltage and self.enabled:
            self.get_logger().warn(f'supply dropped to {min(volts):.1f} V (12V 2A adapter is too weak; 5A recommended)')
        # Over-temperature only after 3 consecutive hot readings; while hot, new commands are refused.
        hot = [i for i, h in self.health.items() if h['temp'] >= self.max_temp]
        self.hot_count = self.hot_count + 1 if hot else 0
        if hot and self.hot_count == 1:
            self.get_logger().warn(f'servo {hot} reads {max(temps)}C (checking again)')
        if self.hot_count >= 3 and self.enabled and self.fault is None:
            self.set_fault(f'servo {hot} temperature {max(temps)}C too high, letting it cool')
        # Servo error bits, and torque the servo switched off by itself (built-in overload protection).
        errors = {i: [name for bit, name in self.STATUS_BITS.items() if h.get('status', 0) >> bit & 1]
                  for i, h in good.items()}
        errors = {i: e for i, e in errors.items() if e}
        if errors != self.last_errors:
            if errors:
                self.get_logger().warn(f'servo error bits: {errors}')
            self.last_errors = errors
        dropped = [i for i, h in good.items() if self.enabled and h.get('torque', 1) == 0]
        if dropped and self.fault is None:
            detail = f' {errors}' if errors else ''
            self.set_fault(f'servo {dropped} switched its torque OFF by itself{detail} '
                           '(overload/heat protection). Support the arm; the next command re-enables it.')
        txt = (f'torque={"ON" if self.enabled else "OFF"} voltage={min(volts):.1f}-{max(volts):.1f}V '
               f'temp_max={max(temps)}C max_lag={lag:.1f}deg '
               f'fault={self.fault or "none"} '
               f'load%={[round(self.health[i]["load"]) for i in sorted(self.health)]}')
        self.status_pub.publish(String(data=txt))

    def set_fault(self, reason):
        self.fault = reason
        self.motion.hold(self.present)
        self.get_logger().error(reason)

    def on_params(self, params):
        """Live servo tuning: ros2 param set /so100_driver servo_d "[64,64,64,32,32,32]"."""
        from rcl_interfaces.msg import SetParametersResult
        names = {'servo_p': 'p', 'servo_d': 'd', 'servo_dead_zone': 'dead_zone'}
        for prm in params:
            if prm.name == 'min_joint_speed':
                self.min_joint_speed = float(prm.value)
            if prm.name in names:
                vals = list(prm.value)
                if len(vals) != 6 or any(not 0 <= v <= 254 for v in vals):
                    return SetParametersResult(successful=False, reason='need 6 values 0..254')
                if self.sim:
                    continue
                with self.lock:
                    setattr(self.arm, names[prm.name], vals)
                    self.arm.apply_gains()
                self.get_logger().info(f'{prm.name} -> {vals}')
        return SetParametersResult(successful=True)

    # ------------------------------------------------------------ helpers
    def ensure_enabled(self):
        """Torque on at the current pose. Raises RuntimeError (command refused) while a servo is hot."""
        with self.lock:
            if self.hot_count > 0:
                hot = {i: h['temp'] for i, h in self.health.items() if h['temp'] >= self.max_temp}
                raise RuntimeError(f'servo too hot {hot} C: wait until it cools below {self.max_temp} C')
            if self.fault:
                # a new command after a fault is an explicit restart from the current pose
                self.get_logger().warn(f'clearing fault: {self.fault}')
                self.fault = None
            if self.enabled and not all(self.arm.torque().values()):
                self.enabled = False       # a servo dropped its torque: re-arm all at the current pose
            if not self.enabled:
                self.present = ids_to_names(self.arm.read_positions())
                self.motion.hold(self.present)
                self.arm.write_goals(names_to_ids(self.present))
                self.arm.set_torque(True)
                self.enabled = True
                self.get_logger().info('torque ON, holding current pose')

    def on_enable(self, req, res):
        with self.lock:
            if req.data:
                try:
                    self.ensure_enabled()
                except RuntimeError as exc:
                    res.success, res.message = False, str(exc)
                    return res
            else:
                self.motion.hold(self.present)
                self.grip = None
                self.arm.set_torque(False)
                self.enabled = False
        res.success, res.message = True, f'torque {"ON" if req.data else "OFF"}'
        return res

    def on_stop(self, _req, res):
        with self.lock:
            self.motion.freeze()
            self.grip = None
        res.success, res.message = True, 'holding current pose'
        return res

    def on_command(self, msg):
        # Slider GUIs publish all joints continuously from their own start values (zeros).
        # Only follow a joint after its value changes, so starting the GUI never jerks the arm.
        cmd = {n: p for n, p in zip(msg.name, msg.position) if n in JOINTS and math.isfinite(p)}
        if not hasattr(self, 'cmd_base'):
            self.cmd_base, self.cmd_live = cmd, set()
            return
        self.cmd_live |= {n for n, p in cmd.items() if abs(p - self.cmd_base.get(n, p)) > 1e-4}
        if not self.cmd_live & set(cmd):
            return
        try:
            self.ensure_enabled()
        except RuntimeError as exc:
            self.get_logger().warn(str(exc), throttle_duration_sec=2.0)
            return
        with self.lock:
            self.motion.traj = None
            for n in self.cmd_live & set(cmd):
                self.motion.set_target(n, cmd[n])

    # ---------------------------------------------------------- arm action
    def accept_traj(self, goal):
        names = list(goal.trajectory.joint_names)
        if not names or any(n not in JOINTS for n in names):
            self.get_logger().error(f'rejecting trajectory for joints {names}')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def exec_traj(self, handle):
        jt = handle.request.trajectory
        names = list(jt.joint_names)
        pts = [(p.time_from_start.sec + p.time_from_start.nanosec * 1e-9, list(p.positions)) for p in jt.points]
        result = FollowJointTrajectory.Result()
        try:
            self.ensure_enabled()
        except RuntimeError as exc:
            handle.abort()
            result.error_code = result.INVALID_GOAL
            result.error_string = str(exc)
            self.get_logger().error(result.error_string)
            return result
        with self.lock:
            if not pts or pts[0][0] > 0.0:
                # Start from where the arm really is, so the first segment is interpolated too.
                pts.insert(0, (0.0, [self.calib.to_rad(n, self.present[n]) for n in names]))
            # A goal with path_tolerance asks for the exact path (straight-line pick/place moves):
            # retiming a slow joint would bend that line, so the no-creep retime is skipped.
            exact = len(handle.request.path_tolerance) > 0
            traj = Trajectory(names, pts, time.monotonic(), min_speed=0.0 if exact else self.min_joint_speed)
            if traj.retimed:
                self.get_logger().info('no-creep retime: ' + ', '.join(
                    f'{n} {math.degrees(abs(b - a)):.1f}deg' for n, (a, b, _, _) in traj.retimed.items()))
            self.motion.set_trajectory(traj)
        tolerance = 0.06  # rad (~3.4 deg) final error accepted
        deadline = traj.start + traj.duration + 2.0
        while rclpy.ok():
            time.sleep(0.02)
            with self.lock:
                if self.motion.traj is not traj:
                    reason = self.fault or 'stopped / replaced by another command'
                    handle.abort()
                    result.error_code = result.PATH_TOLERANCE_VIOLATED
                    result.error_string = reason
                    return result
                if handle.is_cancel_requested:
                    self.motion.freeze()
                    handle.canceled()
                    return result
                now = time.monotonic()
                fb = FollowJointTrajectory.Feedback()
                fb.joint_names = names
                fb.actual.positions = [self.calib.to_rad(n, self.present[n]) for n in names]
                handle.publish_feedback(fb)
                if now >= traj.start + traj.duration:
                    err = self.motion.joint_error(self.present, names)
                    if max(err.values()) <= tolerance:
                        handle.succeed()
                        result.error_code = result.SUCCESSFUL
                        return result
                    if now > deadline:
                        worst = max(err, key=err.get)
                        handle.abort()
                        result.error_code = result.GOAL_TOLERANCE_VIOLATED
                        result.error_string = f'{worst} stopped {math.degrees(err[worst]):.1f} deg short'
                        self.get_logger().warn(result.error_string)
                        return result
        return result

    # ------------------------------------------------------- gripper action
    def exec_grip(self, handle):
        target = handle.request.command.position
        result = GripperCommand.Result()
        try:
            self.ensure_enabled()
        except RuntimeError as exc:
            self.get_logger().error(str(exc))
            handle.abort()
            return result
        with self.lock:
            self.motion.set_target('gripper', target)
            self.grip = {'since': time.monotonic(), 'last': self.present['gripper'], 'stalled': False}
        start = time.monotonic()
        while rclpy.ok():
            time.sleep(0.02)
            with self.lock:
                pos = self.calib.to_rad('gripper', self.present['gripper'])
                err = abs(self.present['gripper'] - self.motion.target['gripper']) / (4096 / (2 * math.pi))
                g = self.grip
                if handle.is_cancel_requested:
                    handle.canceled()
                    return result
                if g is None:
                    handle.abort()
                    return result
                result.position = pos
                result.effort = abs(self.health.get(6, {}).get('load', 0.0))
                if err < 0.05:
                    result.reached_goal = True
                    handle.succeed()
                    return result
                if g['stalled'] or time.monotonic() - start > 5.0:
                    # Holding something: keep squeezing with the limited gripper torque.
                    result.stalled = True
                    handle.succeed()
                    return result
        return result

    def update_grip(self, now):
        g = self.grip
        if g is None:
            return
        pos = self.present['gripper']
        if abs(pos - g['last']) > 6:
            g['last'], g['since'] = pos, now
        elif now - g['since'] > 0.4:
            g['stalled'] = True


def main():
    rclpy.init()
    node = Driver()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # Torque stays ON so the arm does not drop; use /so100/enable false while holding the arm.
        node.arm.close()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
