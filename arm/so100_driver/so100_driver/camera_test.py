"""Camera test for the arm: a fixed depth camera (ASUS Xtion, openni2) finds objects on the table and
the arm picks them through the /arm/* API (use hardware:=sim first).

  ros2 launch so100_moveit_config arm.launch.py hardware:=sim camera:=true

Calibration (once per camera placement, one narrow object such as a can or bottle, nothing else on the table):
  1. put it at spot A (param spot_a, default forward 20 cm, left 10 cm) -> ros2 service call /camera_test/calib_a std_srvs/srv/Trigger
  2. move it to spot B (default forward 20 cm, right 10 cm)            -> ros2 service call /camera_test/calib_b std_srvs/srv/Trigger
  The table plane gives the camera tilt/height; the two spots give its turn and position. Saved to calib_file.
Then:
  ros2 service call /camera_test/objects    std_srvs/srv/Trigger   list what the camera sees (base frame, cm)
  ros2 service call /camera_test/pick       std_srvs/srv/Trigger   pick the object nearest to the arm
  ros2 service call /camera_test/pick_place std_srvs/srv/Trigger   ... and put it at place_spot
RViz shows the point cloud (in the arm base frame once calibrated) and orange boxes for the objects.
"""
import json
import math
import os
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
import rclpy.parameter
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_srvs.srv import Trigger
from tf2_ros import StaticTransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

from .arm_client import PAN_AXIS, ArmClient, user_xy
from .tabletop import CameraPose, calibrate_2pt, find_objects, fit_plane, fuse

CALIB_MAX_HEIGHT = 0.25     # the calibration object (e.g. a can or bottle) is narrow: tells it apart from the platform
CALIB_MAX_SIZE = 0.09
CALIB_FRAMES = 8     # frames averaged per calibration spot
MIN_RANGE = 0.55   # m from the camera: closer depth from the Xtion is missing or poor
CAMERA_UP = (0.0, -1.0, 0.0)   # optical frame: y points down in the image


class CameraTest(Node):
    def __init__(self):
        super().__init__('camera_test')
        self.declare_parameter('cloud_topic', '/camera/depth_registered/points')
        self.declare_parameter('table_z', -0.075)
        self.declare_parameter('calib_file', '~/so100_ws/camera_calib.json')
        self.declare_parameter('spot_a', [0.20, 0.10])       # forward, left (m from the pan axis)
        self.declare_parameter('spot_b', [0.20, -0.10])
        self.declare_parameter('place_spot', [0.22, -0.12])
        self.declare_parameter('ignore_radius', 0.11)        # arm + its platform around the pan axis
        self.declare_parameter('max_radius', 0.45)
        # real size [a, b, height] m of the object when known (e.g. a box): the camera then only finds where
        # it is and how it is turned. All zero = measure the size from the camera (round objects).
        self.declare_parameter('object_size', [0.0, 0.0, 0.0])
        self.declare_parameter('stride', 3)                  # use every n-th pixel row/column
        p = lambda n: self.get_parameter(n).value
        self.table_z = float(p('table_z'))
        self.calib_file = os.path.expanduser(p('calib_file'))
        self.frame = ''
        self.pts = None                 # latest cloud, N x 3 in the cloud frame
        self.plane = None               # (n, d) of the table in the cloud frame
        self.pose = None                # CameraPose once calibrated
        self.spot_seen = {}             # 'a' -> object centre in the table frame
        self.history = []               # last frames of objects (base frame)
        self.lock = threading.Lock()
        self.load()
        self.size_file = os.path.join(os.path.dirname(self.calib_file), 'camera_object_size.json')
        try:                             # object_size survives restarts
            with open(self.size_file) as f:
                saved = [float(v) for v in json.load(f)]
            if p('object_size') == [0.0, 0.0, 0.0] and len(saved) == 3:
                self.set_parameters([rclpy.parameter.Parameter('object_size', value=saved)])
                self.get_logger().info(f'ขนาดของ (จากไฟล์): {saved}')
        except (OSError, ValueError):
            pass
        self.add_on_set_parameters_callback(self.on_params)

        self.tf = StaticTransformBroadcaster(self)
        self.marker_pub = self.create_publisher(MarkerArray, '/camera_test/markers', 1)
        cloud_cb = MutuallyExclusiveCallbackGroup()
        cb = ReentrantCallbackGroup()
        self.create_subscription(PointCloud2, p('cloud_topic'), self.on_cloud, 1, callback_group=cloud_cb)
        for name, fn in (('calib_a', lambda q, r: self.on_calib('a', r)), ('calib_b', lambda q, r: self.on_calib('b', r)),
                         ('objects', self.on_objects), ('pick', lambda q, r: self.on_pick(r, False)),
                         ('pick_place', lambda q, r: self.on_pick(r, True))):
            self.create_service(Trigger, f'/camera_test/{name}', fn, callback_group=cb)
        self.arm = ArmClient(self)
        self.last_info = 0.0
        self.get_logger().info('รอภาพจากกล้อง ' + p('cloud_topic') +
                               ('' if self.pose else '  (ยังไม่ได้ calibrate: ดูวิธีใน README หัวข้อกล้อง)'))

    # ------------------------------------------------------------------------------ calibration file
    def load(self):
        try:
            with open(self.calib_file) as f:
                v = json.load(f)
            self.pose = CameraPose(v['n'], v['d'], v['yaw'], v['txy'], self.table_z)
            self.get_logger().info(f'ใช้ค่า calibrate กล้องจาก {self.calib_file}')
        except (OSError, ValueError, KeyError):
            self.pose = None

    def save(self):
        os.makedirs(os.path.dirname(self.calib_file), exist_ok=True)
        with open(self.calib_file, 'w') as f:
            json.dump(self.pose.to_dict(), f, indent=2)

    def publish_tf(self):
        if not self.pose or not self.frame:
            return
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id, t.child_frame_id = 'base', self.frame
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = map(float, self.pose.t)
        q = self.pose.quaternion()
        t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = q
        self.tf.sendTransform(t)

    # ------------------------------------------------------------------------------------- cloud
    def on_cloud(self, msg):
        s = max(1, int(self.get_parameter('stride').value))
        a = point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=False)
        if msg.height > 1:
            a = a.reshape(msg.height, msg.width)[::s, ::s]
        pts = np.stack([a['x'], a['y'], a['z']], axis=-1).reshape(-1, 3).astype(float)
        pts = pts[np.isfinite(pts).all(axis=1) & (pts[:, 2] > 0.1) & (pts[:, 2] < 2.0)]
        new_frame = msg.header.frame_id != self.frame
        with self.lock:
            self.pts, self.frame = pts, msg.header.frame_id
        if new_frame:
            self.publish_tf()
        if self.pose is None:
            if time.monotonic() - self.last_info > 10:
                self.last_info = time.monotonic()
                try:
                    n, d, inl = fit_plane(pts, up=CAMERA_UP)
                    tilt = math.degrees(math.acos(min(1.0, abs(n[2]))))
                    self.get_logger().info(f'เห็นโต๊ะ: กล้องสูงจากโต๊ะ {d * 100:.1f} ซม. ก้มลง {90 - tilt:.0f}° '
                                           f'({inl.mean() * 100:.0f}% ของภาพเป็นโต๊ะ) - ยังไม่ได้ calibrate')
                except ValueError as exc:
                    self.get_logger().warn(f'ยังหาพื้นโต๊ะไม่ได้: {exc}')
            return
        objs = self.detect(pts)
        with self.lock:
            self.history = (self.history + [objs])[-5:]
        self.publish_markers(fuse(self.history))

    def level_table(self, base, area):
        """Heights from the table right in front of the arm: refit the table there (a slightly tilted
        calibration plane is centimetres off half a metre away) and measure every point from it."""
        near = area & (np.abs(base[:, 2] - self.table_z) < 0.04)
        if near.sum() < 200:
            return base
        try:
            n, d, inl = fit_plane(base[near], tol=0.006, up=(0.0, 0.0, 1.0))
        except ValueError:
            return base
        if inl.mean() < 0.5 or abs(n[2]) < math.cos(math.radians(10)):
            return base                    # not clearly the table: keep the calibration plane
        out = base.copy()
        out[:, 2] = self.table_z + base @ n + d
        return out

    def detect(self, pts):
        base = self.pose.to_base(pts)
        r = np.hypot(base[:, 0] - PAN_AXIS[0], base[:, 1] - PAN_AXIS[1])
        keep = (r > float(self.get_parameter('ignore_radius').value)) & (r < float(self.get_parameter('max_radius').value) + 0.1)
        base = self.level_table(base, keep)
        known = self.known_size()
        z_max = known[2] + 0.04 if known else 0.30   # known object: ignore what hangs above it (the arm itself)
        objs = find_objects(base, self.table_z, keep, z_max=z_max, camera=self.pose.t, known_size=known)
        rmax = float(self.get_parameter('max_radius').value)
        known = self.known_size()
        if known:          # only things that look like the known object (as tall, about as long)
            long_ = max(known[:2])
            objs = [o for o in objs if abs(o['seen'][1] - known[2]) < 0.02 and o['seen'][0] < long_ + 0.03
                    and o['seen'][0] > 0.5 * min(known[:2])]
        return [o for o in objs if math.hypot(o['x'] - PAN_AXIS[0], o['y'] - PAN_AXIS[1]) < rmax]

    def publish_markers(self, objs):
        arr = MarkerArray()
        arr.markers.append(Marker(action=Marker.DELETEALL))
        for i, o in enumerate(objs):
            m = Marker(ns='objects', id=i, type=Marker.CUBE)
            m.header.frame_id = 'base'
            m.pose.position.x, m.pose.position.y = o['x'], o['y']
            m.pose.position.z = self.table_z + o['size'][2] / 2
            m.pose.orientation.z, m.pose.orientation.w = math.sin(o['yaw'] / 2), math.cos(o['yaw'] / 2)
            m.scale.x, m.scale.y, m.scale.z = o['size']
            m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.55, 0.1, 0.5
            t = Marker(ns='labels', id=i, type=Marker.TEXT_VIEW_FACING)
            t.header.frame_id = 'base'
            t.pose.position.x, t.pose.position.y = o['x'], o['y']
            t.pose.position.z = self.table_z + o['size'][2] + 0.025
            t.scale.z = 0.02
            t.color.r = t.color.g = t.color.b = t.color.a = 1.0
            t.text = self.describe(o)
            arr.markers += [m, t]
        self.marker_pub.publish(arr)

    @staticmethod
    def describe(o):
        f = PAN_AXIS[1] - o['y']
        l_ = o['x'] - PAN_AXIS[0]
        d, w, h = (v * 100 for v in o['size'])
        turn = math.degrees(math.remainder(o['yaw'] + math.pi / 2, math.pi))   # 0 = long side points at the arm
        return f'หน้า {f * 100:.0f} ซ้าย {l_ * 100:.0f} หมุน {turn:.0f}° | {d:.1f}x{w:.1f}x{h:.1f} ซม.'

    # ---------------------------------------------------------------------------------- services
    def on_calib(self, which, res):
        with self.lock:
            pts = self.pts
        if pts is None:
            res.success, res.message = False, 'ยังไม่ได้ภาพจากกล้อง (เสียบกล้อง/เปิดไดรเวอร์หรือยัง?)'
            return res
        try:
            n, d, _ = fit_plane(pts, up=CAMERA_UP)
        except ValueError as exc:
            res.success, res.message = False, f'หาพื้นโต๊ะไม่ได้: {exc}'
            return res
        if which == 'a' or self.plane is None:
            self.plane = (n, d)               # spot B is measured in the same table frame as spot A
        n, d = self.plane
        known = self.known_size()

        def is_target(o):
            if known:      # the known object, long side to the camera: as tall, about as long, not far away
                long_ = max(known[:2])
                return (abs(o['seen'][1] - known[2]) < 0.02 and 0.5 * long_ < o['seen'][0] < long_ + 0.03
                        and math.hypot(o['x'], o['y']) < 1.8)
            return o['size'][2] < CALIB_MAX_HEIGHT and max(o['size'][:2]) < CALIB_MAX_SIZE

        def one_frame(pts):
            tab = CameraPose(n, d).to_base(pts)   # table frame, table at z = 0
            objs = [o for o in find_objects(tab, 0.0, z_max=known[2] + 0.04 if known else 0.30, camera=(0.0, 0.0, d),
                                            known_size=known) if is_target(o)]
            if known and len(objs) > 1:      # several look alike: take the one whose seen side matches clearly best
                long_ = max(known[:2])
                objs.sort(key=lambda o: abs(o['seen'][0] - long_))
                if abs(objs[1]['seen'][0] - long_) - abs(objs[0]['seen'][0] - long_) > 0.02:
                    objs = objs[:1]
            return objs

        # several frames (a far object flickers): median of the frames that saw exactly one
        seen, last, frame = [], [], None
        end = time.monotonic() + 4.0
        while time.monotonic() < end and len(seen) < CALIB_FRAMES:
            with self.lock:
                cur = self.pts
            if cur is frame:
                time.sleep(0.05)
                continue
            frame = cur
            last = one_frame(cur)
            if len(last) == 1:
                seen.append(last[0])
        if len(seen) >= CALIB_FRAMES // 2:
            spread = max(math.hypot(o['x'] - seen[0]['x'], o['y'] - seen[0]['y']) for o in seen)
            objs = [dict(seen[0], x=float(np.median([o['x'] for o in seen])),
                         y=float(np.median([o['y'] for o in seen])))]
            self.get_logger().info(f'calib {which}: {len(seen)} ภาพ กระจาย {spread * 1000:.0f} มม.')
        else:
            objs = last
        if len(objs) != 1:
            res.success = False
            res.message = (f'ต้องเห็นของที่ใช้ calibrate ชิ้นเดียว (ขนาดตาม object_size หรือของแคบกว้าง < {CALIB_MAX_SIZE * 100:.0f} ซม.) แต่เห็น {len(objs)} ชิ้น'
                           + ''.join(f'\n  {o["size"][0] * 100:.1f}x{o["size"][1] * 100:.1f}x{o["size"][2] * 100:.1f} ซม.'
                                     for o in objs))
            return res
        dist = math.hypot(objs[0]['x'], objs[0]['y'])
        if dist < MIN_RANGE:
            res.success = False
            res.message = (f'ของอยู่ห่างกล้องแค่ ~{dist * 100:.0f} ซม. กล้องวัดระยะแม่นตั้งแต่ ~{MIN_RANGE * 100:.0f} ซม.: '
                           'ถอยกล้องออกไปอีก แล้วเริ่มใหม่ที่จุด A')
            return res
        self.spot_seen[which] = (objs[0]['x'], objs[0]['y'])
        if which == 'a':
            b = self.get_parameter('spot_b').value
            res.success = True
            res.message = (f'จำจุด A แล้ว (ของห่างกล้อง {dist * 100:.0f} ซม. กล้องสูงจากโต๊ะ {d * 100:.1f} ซม.) ต่อไปย้ายของไปจุด B: '
                           f'หน้า {b[0] * 100:.0f} ซม. ซ้าย {b[1] * 100:.0f} ซม. แล้วเรียก /camera_test/calib_b')
            return res
        if 'a' not in self.spot_seen:
            res.success, res.message = False, 'ทำจุด A ก่อน (/camera_test/calib_a)'
            return res
        a_base = user_xy(*self.get_parameter('spot_a').value)
        b_base = user_xy(*self.get_parameter('spot_b').value)
        try:
            yaw, txy, err = calibrate_2pt(self.spot_seen['a'], self.spot_seen['b'], a_base, b_base)
        except ValueError as exc:
            res.success, res.message = False, str(exc)
            return res
        pose = CameraPose(n, d, yaw, txy, self.table_z)
        cam_f, cam_l = PAN_AXIS[1] - pose.t[1], pose.t[0] - PAN_AXIS[0]
        res.success = err < 0.02
        if res.success:                  # a bad pair never replaces the saved calibration
            self.pose = pose
            self.save()
            self.history = []
            self.publish_tf()
        res.message = (f'calibrate เสร็จ: กล้องอยู่ หน้า {cam_f * 100:.0f} ซ้าย {cam_l * 100:.0f} สูงจากโต๊ะ {d * 100:.1f} ซม. '
                       f'ระยะ A-B ที่กล้องเห็นต่างจากที่วัด {err * 1000:.0f} มม.'
                       + ('' if res.success else ' (เกิน 2 ซม.: วัดจุดใหม่ หรือของเลื่อน ลองใหม่)'))
        return res

    def on_params(self, params):
        for prm in params:
            if prm.name == 'object_size':
                try:
                    with open(self.size_file, 'w') as f:
                        json.dump([float(v) for v in prm.value], f)
                except OSError:
                    pass
                self.history = []
        return SetParametersResult(successful=True)

    def known_size(self):
        v = [float(x) for x in self.get_parameter('object_size').value]
        return tuple(v) if len(v) == 3 and min(v) > 0 else None

    def current(self):
        with self.lock:
            objs = fuse(self.history)
        return sorted(objs, key=lambda o: math.hypot(o['x'] - PAN_AXIS[0], o['y'] - PAN_AXIS[1]))

    def on_objects(self, _req, res):
        if not self.pose:
            res.success, res.message = False, 'ยังไม่ได้ calibrate กล้อง'
            return res
        objs = self.current()
        res.success = True
        res.message = f'เห็น {len(objs)} ชิ้น (เรียงจากใกล้แขน)' + ''.join(f'\n  {self.describe(o)}' for o in objs)
        return res

    def on_pick(self, res, place):
        if not self.pose:
            res.success, res.message = False, 'ยังไม่ได้ calibrate กล้อง'
            return res
        objs = self.current()
        if not objs:
            res.success, res.message = False, 'ไม่เห็นของบนโต๊ะในระยะแขน'
            return res
        o = objs[0]
        where, size = (o['x'], o['y']), o['size']
        self.get_logger().info(f'หยิบ: {self.describe(o)}')
        if place:
            out = self.arm.pick_place(where, user_xy(*self.get_parameter('place_spot').value), size, obj_yaw=o['yaw'])
        else:
            out = self.arm.pick(where, size, yaw=o['yaw'])
        res.success = bool(out[0])
        res.message = f'{self.describe(o)} -> {out[1]}: {out[2]}'
        return res


def main():
    rclpy.init()
    node = CameraTest()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    try:
        ex.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        ex.shutdown(timeout_sec=1.0)
        rclpy.try_shutdown()
        os._exit(0)


if __name__ == '__main__':
    main()
