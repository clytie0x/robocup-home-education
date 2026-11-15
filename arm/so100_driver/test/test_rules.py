"""Rules of the task server and the driver that need no ROS: error codes as values, a held bag is never
forgotten, a fault keeps the squeeze, one commander at a time, the load reading, one floor height."""
import ast
import math
import os
import re
import sys
import unittest
from pathlib import Path

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from so100_driver import task_rules as tr  # noqa: E402
from so100_driver.calib import JOINTS  # noqa: E402
from so100_driver.motion import Motion  # noqa: E402
from test_core import calib  # noqa: E402

PKG = Path(__file__).resolve().parents[1] / 'so100_driver'
ARM = Path(__file__).resolve().parents[2]


def failed_calls(path):
    """Every Failed(...) call in a source file: (line, args, keywords)."""
    tree = ast.parse(path.read_text(encoding='utf-8'))
    return [n for n in ast.walk(tree) if isinstance(n, ast.Call) and getattr(n.func, 'id', '') == 'Failed']


def messages_keys():
    tree = ast.parse((PKG / 'pick_place_node.py').read_text(encoding='utf-8'))
    for n in ast.walk(tree):
        if isinstance(n, ast.Assign) and getattr(n.targets[0], 'id', '') == 'MESSAGES':
            return {k.value for k in n.value.keys}
    raise AssertionError('no MESSAGES in pick_place_node.py')


class ErrorCodes(unittest.TestCase):
    def test_failed_needs_a_code(self):
        with self.assertRaises(TypeError):
            tr.Failed('servo temperature 61C too high')       # no code: no guessing from the words
        self.assertEqual(tr.Failed('stopped / replaced by another command', 'REPLACED').code, 'REPLACED')

    def test_every_failed_in_the_task_server_gives_a_known_code(self):
        codes = messages_keys()
        calls = failed_calls(PKG / 'pick_place_node.py')
        self.assertGreater(len(calls), 15)
        for c in calls:
            code = c.args[1] if len(c.args) > 1 else next((k.value for k in c.keywords if k.arg == 'code'), None)
            self.assertIsNotNone(code, f'line {c.lineno}: Failed without a code')
            for v in ast.walk(code):          # literal codes (also inside a conditional) must be API codes
                if isinstance(v, ast.Constant) and isinstance(v.value, str):
                    self.assertIn(v.value, codes, f'line {c.lineno}')

    def test_the_task_server_does_not_define_its_own_failed(self):
        src = (PKG / 'pick_place_node.py').read_text(encoding='utf-8')
        self.assertNotIn('class Failed', src)

    def test_driver_codes_become_api_codes(self):
        self.assertEqual(tr.api_code(tr.ARM_REPLACED), 'REPLACED')
        self.assertEqual(tr.api_code(tr.ARM_HOT), 'HOT')
        self.assertEqual(tr.api_code(tr.ARM_STOPPED), 'STOPPED')
        self.assertEqual(tr.api_code(tr.ARM_BUSY), 'BUSY')
        self.assertEqual(tr.api_code(tr.PATH_TOLERANCE_VIOLATED), 'COLLISION')
        self.assertEqual(tr.api_code(tr.GOAL_TOLERANCE_VIOLATED), 'COLLISION')
        self.assertEqual(tr.api_code(tr.ARM_FAULT), 'FAILED')
        self.assertEqual(tr.api_code(-1), 'FAILED')
        self.assertIn('REPLACED', messages_keys())

    def test_replaced_is_in_every_action_definition(self):
        for f in (ARM / 'so100_interfaces' / 'action').glob('*.action'):
            line = next(ln for ln in f.read_text().splitlines() if ln.startswith('string error_code'))
            self.assertIn('REPLACED', line, f.name)


class HeldBag(unittest.TestCase):
    def test_jaws_held_open_mean_something_is_in_them(self):
        # closed on air reads 0.031 to 0.046 rad (2026-11-15); the bag threshold is 0.06
        self.assertTrue(tr.still_held(0.12, 0.06))
        self.assertFalse(tr.still_held(0.046, 0.06))
        self.assertFalse(tr.still_held(None, 0.06))    # no joint_states: not known to hold
        self.assertTrue(tr.still_held(0.12, 0.06, 0.9))
        self.assertFalse(tr.still_held(0.88, 0.06, 0.9))   # the jaws were opened again: nothing held


class FaultKeepsSqueeze(unittest.TestCase):
    def setUp(self):
        self.c = calib()
        self.present = {n: self.c.data[n]['zero'] for n in JOINTS}

    def squeezing(self, m):
        m.hold(self.present)
        m.set_target('gripper', -0.5)              # closing on a bag: the goal lies past where the jaw stops
        for k in range(50):
            m.step(k * 0.02, self.present, 0.02)
        return m.goal['gripper']

    def test_hold_on_fault_keeps_the_gripper_goal(self):
        m = Motion(self.c, {})
        squeeze = self.squeezing(m)
        target = m.target['gripper']
        self.assertGreater(abs(squeeze - self.present['gripper']), 50)
        m.hold(self.present, keep_gripper=True)
        self.assertEqual(m.goal['gripper'], squeeze)
        self.assertEqual(m.target['gripper'], target)
        self.assertEqual(m.goal['elbow_flex'], self.present['elbow_flex'])
        m.hold(self.present)                       # torque-on hold: everything where it is
        self.assertEqual(m.goal['gripper'], self.present['gripper'])

    def test_a_blocked_joint_keeps_the_squeeze(self):
        m = Motion(self.c, {}, max_speed=3.0, follow_error=0.3, follow_error_time=0.5)
        squeeze = self.squeezing(m)
        m.set_target('elbow_flex', -1.2)
        for k in range(100):                       # elbow blocked
            m.step(1.0 + k * 0.02, self.present, 0.02)
            if m.fault:
                break
        self.assertIn('elbow_flex', m.fault)
        self.assertEqual(m.goal['gripper'], squeeze)
        self.assertEqual(m.fault_goals(self.present)['gripper'], squeeze)
        self.assertEqual(m.fault_goals(self.present)['elbow_flex'], self.present['elbow_flex'])

    def test_only_opening_the_gripper_is_allowed_while_hot(self):
        self.assertTrue(tr.allowed_while_hot(0.9, 0.3))
        self.assertFalse(tr.allowed_while_hot(0.0, 0.3))
        self.assertFalse(tr.allowed_while_hot(0.3, 0.3))


class OneCommander(unittest.TestCase):
    def test_another_program_cannot_take_a_running_move(self):
        self.assertTrue(tr.may_take_arm(None, ''))                    # nothing runs
        self.assertTrue(tr.may_take_arm('base', 'base'))              # MoveIt replaces its own goal
        self.assertFalse(tr.may_take_arm(tr.PICK_PLACE, ''))          # a script during our pick (04:33)
        self.assertFalse(tr.may_take_arm('base', ''))
        self.assertFalse(tr.may_take_arm('', 'base'))
        self.assertTrue(tr.may_take_arm('', tr.PICK_PLACE))           # the task server always may
        self.assertEqual(tr.BUSY_TEXT, 'arm busy: another program has it')


class LoadReading(unittest.TestCase):
    def test_gripper_load_from_the_driver_status(self):
        st = ('torque=ON voltage=11.9-12.1V temp_max=41C max_lag=1.2deg fault=none '
              'load%=[3, -12, 20, 5, 1, 37]')
        self.assertEqual(tr.gripper_load(st), 37.0)
        self.assertTrue(math.isnan(tr.gripper_load('torque=ON load%=[3, 4]')))   # not all six servos read
        self.assertTrue(math.isnan(tr.gripper_load('')))

    def test_fresh_gripper_load_register(self):
        from so100_driver.hardware import RealArm
        from test_core import bus
        b, fake = bus()
        fake.mem[6][60:62] = (0x400 | 321).to_bytes(2, 'little')     # 32.1 %, closing direction
        arm = RealArm.__new__(RealArm)
        arm.bus = b
        self.assertAlmostEqual(arm.gripper_load(), 32.1)

    def test_interfaces_carry_the_reading(self):
        res = (ARM / 'so100_interfaces' / 'action' / 'PickBag.action').read_text().split('---')[1]
        st = (ARM / 'so100_interfaces' / 'msg' / 'ArmState.msg').read_text()
        for text in (res, st):
            self.assertRegex(text, r'\nfloat32 gripper_pos\b')
            self.assertRegex(text, r'\nfloat32 gripper_load\b')


class FloorHeight(unittest.TestCase):
    def test_one_floor_for_the_arm_on_the_robot(self):
        launch = (ARM / 'so100_moveit_config' / 'launch' / 'arm.launch.py').read_text(encoding='utf-8')
        m = re.search(r"DeclareLaunchArgument\('floor_z', default_value='([-0-9.]+)'", launch)
        self.assertEqual(m.group(1), '-0.331')
        client = (PKG / 'arm_client.py').read_text(encoding='utf-8')
        self.assertRegex(client, r'\nFLOOR_Z = -0\.331\b')


if __name__ == '__main__':
    unittest.main()
