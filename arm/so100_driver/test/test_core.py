import math
import unittest

from so100_driver import feetech as ft
from so100_driver.calib import COUNTS_PER_RAD, JOINTS, Calibration
from so100_driver.motion import Motion, Trajectory


class FakeServos:
    """Emulates STS servos on a serial line (optionally echoing like a half-duplex adapter)."""

    def __init__(self, echo=False):
        self.mem = {i: bytearray(80) for i in range(1, 7)}
        for i in self.mem:
            self.mem[i][56:58] = (2000 + i).to_bytes(2, 'little')
        self.rx = bytearray()
        self.echo = echo
        self.writes = []

    def reply(self, sid, params=b''):
        body = bytes([sid, len(params) + 2, 0]) + params
        self.rx += b'\xff\xff' + body + bytes([ft.checksum(body)])

    def write(self, data):
        self.writes.append(bytes(data))
        if self.echo:
            self.rx += data
        sid, instr, params = data[2], data[4], data[5:-1]
        if instr == ft.PING:
            self.reply(sid)
        elif instr == ft.READ:
            a, n = params
            self.reply(sid, bytes(self.mem[sid][a:a + n]))
        elif instr == ft.WRITE:
            self.mem[sid][params[0]:params[0] + len(params) - 1] = params[1:]
            self.reply(sid)
        elif instr == ft.SYNC_READ:
            a, n, *ids = params
            for i in ids:
                self.reply(i, bytes(self.mem[i][a:a + n]))
        elif instr == ft.SYNC_WRITE:
            a, n = params[0], params[1]
            rest = params[2:]
            for k in range(0, len(rest), n + 1):
                self.mem[rest[k]][a:a + n] = rest[k + 1:k + 1 + n]

    def read(self, n):
        out, self.rx = bytes(self.rx[:n]), self.rx[n:]
        return out

    def reset_input_buffer(self):
        self.rx.clear()

    def close(self):
        pass


def bus(echo=False):
    fake = FakeServos(echo)
    return ft.Bus('x', serial_factory=lambda *a, **k: fake), fake


class ProtocolTest(unittest.TestCase):
    def test_packet_checksum_matches_feetech_example(self):
        # Ping ID1: FF FF 01 02 01 FB (documented example)
        self.assertEqual(ft.packet(1, ft.PING), bytes([0xFF, 0xFF, 1, 2, 1, 0xFB]))

    def test_read_write_sync(self):
        for echo in (False, True):
            b, fake = bus(echo)
            self.assertTrue(b.ping(3))
            self.assertEqual(b.sync_read_positions([1, 2, 3, 4, 5, 6]), {i: 2000 + i for i in range(1, 7)})
            b.write1(2, ft.P_GAIN, 16)
            self.assertEqual(b.read1(2, ft.P_GAIN), 16)
            b.sync_write_goals({1: 1234, 6: 5000})
            self.assertEqual(int.from_bytes(fake.mem[1][42:44], 'little'), 1234)
            self.assertEqual(int.from_bytes(fake.mem[6][42:44], 'little'), 4095)  # clamped

    def test_signed_decode(self):
        self.assertEqual(ft.decode_signed(0x8005), -5)
        self.assertEqual(ft.decode_signed(5), 5)


def calib():
    c = Calibration.from_zero({'shoulder_pan': 2048, 'shoulder_lift': 900, 'elbow_flex': 3100,
                               'wrist_flex': 2400, 'wrist_roll': 1000, 'gripper': 950})
    for n in JOINTS:
        z = c.data[n]['zero']
        c.data[n]['min'], c.data[n]['max'] = max(0, z - 1500), min(4095, z + 1500)
    return c


class MotionTest(unittest.TestCase):
    def test_roundtrip(self):
        c = calib()
        for n in JOINTS:
            self.assertAlmostEqual(c.to_rad(n, c.to_raw(n, 0.7)), 0.7)

    def test_speed_is_capped_and_limits_clamped(self):
        c = calib()
        m = Motion(c, {'shoulder_lift': (0.0, 3.5)}, max_speed=1.0)
        present = {n: c.data[n]['zero'] for n in JOINTS}
        m.hold(present)
        m.set_target('shoulder_lift', 3.4)
        goals = m.step(0.0, present, 0.02)
        moved = (m.goal['shoulder_lift'] - present['shoulder_lift']) / COUNTS_PER_RAD
        self.assertAlmostEqual(moved, 0.02, places=5)  # reference: 1 rad/s * 20 ms
        lead = (goals['shoulder_lift'] - m.goal['shoulder_lift']) / COUNTS_PER_RAD
        self.assertAlmostEqual(lead, 0.1, places=5)    # sent goal leads by 0.1 s of motion
        m.set_target('shoulder_lift', -1.0)                # below URDF lower limit 0
        self.assertAlmostEqual(c.to_rad('shoulder_lift', m.target['shoulder_lift']), 0.0)
        hi = c.raw_limits('elbow_flex')[1]
        m.target['elbow_flex'] = 99999
        for k in range(2000):
            goals = m.step(k * 0.02, {**present, 'elbow_flex': goals.get('elbow_flex', present['elbow_flex'])}, 0.02)
        self.assertLessEqual(goals['elbow_flex'], hi)

    def test_trajectory_interpolation(self):
        t = Trajectory(['a'], [(0.0, [0.0]), (1.0, [1.0]), (2.0, [0.0])], start=10.0)
        self.assertAlmostEqual(t.sample(10.5)['a'], 0.5)
        self.assertAlmostEqual(t.sample(11.5)['a'], 0.5)
        self.assertAlmostEqual(t.sample(99)['a'], 0.0)

    def test_blocked_joint_faults_and_holds(self):
        c = calib()
        m = Motion(c, {}, max_speed=3.0, follow_error=0.3, follow_error_time=0.5)
        present = {n: c.data[n]['zero'] for n in JOINTS}
        m.hold(present)
        m.set_target('elbow_flex', -1.2)
        for k in range(100):          # joint never moves (blocked)
            m.step(k * 0.02, present, 0.02)
            if m.fault:
                break
        self.assertIn("elbow_flex", m.fault)
        self.assertEqual(m.goal['elbow_flex'], present['elbow_flex'])


if __name__ == '__main__':
    unittest.main()


class SagTest(unittest.TestCase):
    def test_static_sag_is_compensated_and_capped(self):
        c = calib()
        m = Motion(c, {}, sag_gain=1.5, sag_max_deg=10.0)
        present = {n: c.data[n]['zero'] for n in JOINTS}
        m.hold(present)
        sag = 40   # ~3.5 deg below the goal, like the loaded shoulder
        lift = present['shoulder_lift']
        out = {}
        for k in range(200):   # 4 s standing still; servo settles `sag` below whatever we command
            out = m.step(k * 0.02, {**present, 'shoulder_lift': out.get('shoulder_lift', lift) - sag}, 0.02)
        self.assertAlmostEqual(out['shoulder_lift'] - sag, lift, delta=3)      # real joint ends on target
        m.hold(present)
        for k in range(2000):  # impossible sag -> capped at 10 deg
            m.step(k * 0.02, {**present, 'shoulder_lift': lift - 1000}, 0.02)
            m.fault = None
        self.assertLessEqual(abs(m.offset['shoulder_lift']), math.radians(10) * COUNTS_PER_RAD + 1e-6)


class CollisionCheckTest(unittest.TestCase):
    def _run(self, lag_of_speed):
        c = calib()
        m = Motion(c, {}, max_speed=3.0, follow_error=0.14, lag_time=0.25, follow_error_time=0.5)
        present = {n: c.data[n]['zero'] for n in JOINTS}
        m.hold(present)
        pan0 = present['shoulder_pan']
        traj = Trajectory(['shoulder_pan'], [(0.0, [0.0]), (2.0, [2.0])], start=0.0)  # 1 rad/s = 57 deg/s
        m.set_trajectory(traj)
        real = dict(present)
        for k in range(100):
            m.step(k * 0.02, real, 0.02)
            real['shoulder_pan'] = m.goal['shoulder_pan'] - lag_of_speed(1.0) * COUNTS_PER_RAD
            if m.fault:
                return m.fault
        return None

    def test_normal_lag_while_fast_is_not_a_collision(self):
        self.assertIsNone(self._run(lambda v: 0.2 * v))     # 11 deg behind at 57 deg/s (like the real arm)

    def test_blocked_while_moving_is_caught(self):
        self.assertIsNotNone(self._run(lambda v: 0.6 * v))  # 34 deg behind -> something is in the way


class SpeedTest(unittest.TestCase):
    def test_goal_and_speed_packet(self):
        b, fake = bus()
        b.sync_write_goals_speed({1: 1234, 2: 99999}, {1: 250})
        m = fake.mem
        self.assertEqual(int.from_bytes(m[1][42:44], 'little'), 1234)
        self.assertEqual(int.from_bytes(m[1][44:46], 'little'), 0)      # goal time unused
        self.assertEqual(int.from_bytes(m[1][46:48], 'little'), 250)
        self.assertEqual(int.from_bytes(m[2][42:44], 'little'), 4095)   # clamped
        self.assertEqual(int.from_bytes(m[2][46:48], 'little'), 0)      # no speed given -> unlimited

    def test_servo_speed_follows_reference(self):
        self.assertEqual(Motion.servo_speed(0.0, 1), 0)          # holding: unlimited correction
        self.assertEqual(Motion.servo_speed(100.0, 3), 115)      # glide at the ref speed (+ lag term)
        self.assertEqual(Motion.servo_speed(5.0, 60), 205)       # big lag -> catch up in ~0.3 s
        self.assertEqual(Motion.servo_speed(1e6, 0), 3000)


class NoCreepTest(unittest.TestCase):
    def test_small_slow_joint_moves_smoothly_and_arrives_early(self):
        # 10 s move: shoulder 60 deg (fast enough), pan only 3 deg (would creep at 0.3 deg/s)
        pts = [(0.0, [0.0, 0.0]), (10.0, [math.radians(3), math.radians(60)])]
        t = Trajectory(['shoulder_pan', 'shoulder_lift'], pts, start=0.0, min_speed=math.radians(5))
        self.assertIn('shoulder_pan', t.retimed)
        self.assertNotIn('shoulder_lift', t.retimed)
        self.assertAlmostEqual(math.degrees(t.sample(2.0)['shoulder_pan']), 3.0, places=3)   # done early
        self.assertAlmostEqual(math.degrees(t.sample(5.0)['shoulder_lift']), 30.0, places=3)  # unchanged
        self.assertAlmostEqual(math.degrees(t.sample(10.0)['shoulder_pan']), 3.0, places=6)
        # speed never jumps: positions strictly within the move and monotonic
        xs = [t.sample(k * 0.02)['shoulder_pan'] for k in range(120)]
        self.assertTrue(all(b >= a for a, b in zip(xs, xs[1:])))

    def test_off_by_default(self):
        pts = [(0.0, [0.0]), (10.0, [0.05])]
        self.assertEqual(Trajectory(['shoulder_pan'], pts, start=0.0).retimed, {})


class OneWayRetimeTest(unittest.TestCase):
    def test_out_and_back_joint_keeps_moveit_path(self):
        # pan swings 20 deg out and back to 2 deg (obstacle detour) -> must NOT be straightened
        d = math.radians
        pts = [(0.0, [0.0]), (5.0, [d(20)]), (10.0, [d(2)])]
        t = Trajectory(['shoulder_pan'], pts, start=0.0, min_speed=d(5))
        self.assertEqual(t.retimed, {})
        self.assertAlmostEqual(math.degrees(t.sample(5.0)['shoulder_pan']), 20.0, places=6)

    def test_reversing_joint_is_not_retimed(self):
        d = math.radians
        pts = [(0.0, [0.0]), (4.0, [d(3)]), (6.0, [d(1)]), (10.0, [d(3)])]
        self.assertEqual(Trajectory(['shoulder_pan'], pts, start=0.0, min_speed=d(5)).retimed, {})
