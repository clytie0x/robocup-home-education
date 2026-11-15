"""Rules of the task server and the driver that need no ROS: error codes as values, a held bag is never
forgotten, a fault keeps the squeeze, one commander at a time, the load reading, one floor height, the touch probe for an unknown bag top."""
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


class TouchProbe(unittest.TestCase):
    """Bag top unknown (PickBag grasp_pose z = NaN): the closed jaws go down in steps until the loads or the
    tracking error of shoulder_lift / elbow_flex leave their free-air baseline."""
    BASE, LOAD, ERR = 3, 8.0, 0.02

    @staticmethod
    def free_air(n, seed=1):
        import random
        rnd = random.Random(seed)
        return [((20 + rnd.uniform(-3, 3), -10 + rnd.uniform(-3, 3)),
                 (0.005 + rnd.uniform(-0.003, 0.003), 0.004 + rnd.uniform(-0.003, 0.003))) for _ in range(n)]

    def contact(self, samples, load=None, err=None):
        return tr.probe_contact(samples, self.BASE, self.LOAD if load is None else load,
                                self.ERR if err is None else err)

    def test_noisy_free_air_is_no_contact(self):
        for seed in range(20):
            self.assertIsNone(self.contact(self.free_air(30, seed)))

    def test_a_load_step_is_contact_at_that_step(self):
        s = self.free_air(30)
        for i in range(17, 30):                    # pushed from below: weight comes off the shoulder
            s[i] = ((s[i][0][0] - 12, s[i][0][1]), s[i][1])
        self.assertEqual(self.contact(s), 17)
        self.assertIsNone(self.contact(s[:17]))    # one step earlier nothing is seen yet
        s = self.free_air(30)
        s[12] = ((s[12][0][0], s[12][0][1] + 13), s[12][1])     # elbow, the other sign
        self.assertEqual(self.contact(s), 12)

    def test_tracking_error_growing_is_contact(self):
        s = self.free_air(30)
        for i in range(20, 30):                    # blocked: the joints stay behind the command
            s[i] = (s[i][0], (0.04, 0.035))
        self.assertEqual(self.contact(s), 20)
        self.assertIsNone(self.contact(s, err=0.0))     # 0 = that check off
        self.assertEqual(self.contact(s, load=0.0), 20)

    def test_baseline_is_the_first_steps(self):
        s = [((30.0, -10.0), (0.01, 0.01))] * 3 + [((22.0, -10.0), (0.01, 0.01))]
        self.assertIsNone(self.contact(s))         # 8 % off the baseline is not more than 8 %
        s[-1] = ((21.5, -10.0), (0.01, 0.01))
        self.assertEqual(self.contact(s), 3)
        self.assertIsNone(self.contact(s[:3]))     # only the baseline yet: nothing is judged

    def test_settling_steps_are_not_the_baseline(self):
        # real run 2026-11-15 ~11:25, robot (0.335, 0.035): steps 1-2 still settling after the MoveIt move
        sh = [13, 12, 3, 2, 4, 4, 3, 4, 6]
        el = [2, 2, 3, 2, 2, 2, 2, 5, 13]
        err = [0.004, 0.003, 0.002, 0.002, 0.003, 0.002, 0.003, 0.004, 0.011]
        s = [((a, b), (e, e)) for a, b, e in zip(sh, el, err)]
        hit = tr.probe_contact(s, self.BASE, self.LOAD, self.ERR, skip=2)
        self.assertEqual(hit, 8)                                  # step 9: elbow +13 (measured tip 27.7 cm)
        self.assertIsNone(tr.probe_contact(s[:8], self.BASE, self.LOAD, self.ERR, skip=2))
        flat_elbow = [((a, 2.0), (0.002, 0.002)) for a in sh]   # the shoulder alone gives no contact
        self.assertIsNone(tr.probe_contact(flat_elbow, self.BASE, self.LOAD, self.ERR, skip=2))
        self.assertIsNone(tr.probe_contact(s[:5], self.BASE, self.LOAD, self.ERR, skip=2))   # baseline only
        # baseline from the settling steps (skip 0): shoulder 13, 12 pull it to 9.3, a drop of 3.3 later
        # would already pass for contact with a 6 % threshold
        self.assertEqual(tr.probe_contact(flat_elbow, self.BASE, 6.0, self.ERR), 3)

    def test_held_by_weight(self):
        # 0.1 kg at the approach pose: 0.98 N x 21-28.5 cm = 7-9.5 % of 2.94 N.m on the shoulder (STS3215 30 kg.cm)
        held, change = tr.held_by_weight((3.0, 2.0), (11.0, 7.0), 5.0)
        self.assertTrue(held)
        self.assertEqual(change, (8.0, 5.0))
        self.assertFalse(tr.held_by_weight((3.0, 2.0), (6.0, 4.0), 5.0)[0])     # free-air spread of the run
        self.assertTrue(tr.held_by_weight((3.0, 2.0), (2.0, 8.0), 5.0)[0])      # the elbow alone, either sign
        self.assertFalse(tr.held_by_weight((math.nan, 2.0), (30.0, 3.0), 5.0)[0])   # no reading: not held
        self.assertFalse(tr.held_by_weight((3.0, 2.0), (30.0, 30.0), 0.0)[0])   # 0 = check off

    def test_a_weight_reading_counts_once_settled(self):
        self.assertIsNone(tr.settled_load([(14.0, 0.0), (11.0, 0.0), (8.0, 0.0)], 2.0))   # still falling
        self.assertEqual(tr.settled_load([(17.0, 0.0), (16.0, 0.0), (16.0, 0.0)], 2.0), (16.0, 0.0))
        self.assertEqual(tr.settled_load([(17.0, 1.0), (16.0, 3.0)], 2.0), (16.5, 2.0))   # mean of the pair
        self.assertIsNone(tr.settled_load([(16.0, 0.0)], 2.0))                            # one reading
        self.assertIsNone(tr.settled_load([(16.0, 0.0), (16.0, 4.0)], 2.0))               # every joint
        self.assertIsNone(tr.settled_load([(16.0, math.nan), (16.0, math.nan)], 2.0))

    def test_friction_hysteresis_is_not_a_bag(self):
        # robot 2026-11-15 13:25, nothing lifted: empty jaws arrived from above, read while settling
        # (shoulder 14, 11, 8), closed jaws arrived from below (17, 16, 16); elbow 0 in both
        above = [(14.0, 0.0), (11.0, 0.0), (8.0, 0.0)]
        closed = [(17.0, 0.0), (16.0, 0.0), (16.0, 0.0)]
        self.assertTrue(tr.held_by_weight(above[-1], closed[-1], 5.0)[0])     # the old way: false positive
        self.assertIsNone(tr.settled_load(above, 2.0))                        # now: not a reading at all
        free_below = [(17.0, 0.0), (16.0, 0.0), (16.0, 0.0)]                 # empty jaws after the dip (~16)
        free, held = tr.settled_load(free_below, 2.0), tr.settled_load(closed, 2.0)
        self.assertFalse(tr.held_by_weight(free, held, 5.0)[0])
        # a real bag: +12 % on the shoulder, settled
        bag = tr.settled_load([(29.0, 0.0), (28.0, 0.0), (28.0, 0.0)], 2.0)
        self.assertTrue(tr.held_by_weight(free, bag, 5.0)[0])
        self.assertEqual(tr.held_by_weight(free, bag, 5.0)[1], (12.0, 0.0))

    def test_missing_readings_never_count(self):
        s = self.free_air(10)
        s[5] = ((math.nan, math.nan), (math.nan, math.nan))
        self.assertIsNone(self.contact(s))

    def test_loads_of_the_descent_joints_from_the_status(self):
        st = 'torque=ON fault=none load%=[3, -12, 20, 5, 1, 37]'
        self.assertEqual(tr.descent_loads(st), (-12.0, 20.0))      # servo 2 shoulder_lift, 3 elbow_flex
        self.assertTrue(all(math.isnan(v) for v in tr.descent_loads('torque=ON load%=[3, 4]')))
        self.assertTrue(all(math.isnan(v) for v in tr.descent_loads('')))
        self.assertEqual(tr.PROBE_JOINTS, ('shoulder_lift', 'elbow_flex'))

    def test_only_a_nan_top_means_probe(self):
        self.assertTrue(tr.top_unknown(float('nan')))
        for z in (-0.061, 0.0, -0.331 + 0.26, 0.5):
            self.assertFalse(tr.top_unknown(z))

    def test_the_task_server_probes_only_for_an_unknown_top(self):
        tree = ast.parse((PKG / 'pick_place_node.py').read_text(encoding='utf-8'))
        calls = [n for n in ast.walk(tree) if isinstance(n, ast.Call) and getattr(n.func, 'attr', '') == 'probe_bag']
        self.assertEqual(len(calls), 1)
        fn = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == 'do_pick_bag')
        guard = next(n for n in ast.walk(fn) if isinstance(n, ast.If)
                     and getattr(getattr(n.test, 'func', None), 'id', '') == 'top_unknown')
        self.assertIn(calls[0], list(ast.walk(guard)))
        # a real z keeps the height check it always had
        to_bag = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == 'to_bag')
        self.assertIn('0.03 < h < 0.6', ast.unparse(to_bag))

    def test_weight_check_only_rescues_a_closed_bag_grasp(self):
        tree = ast.parse((PKG / 'pick_place_node.py').read_text(encoding='utf-8'))
        pick = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == 'pick')
        src = ast.unparse(pick)
        self.assertIn("'hang' in plan", src)                     # bags only, the tabletop pick is unchanged
        self.assertIn('r.position < miss', src)                 # bag_miss_below decides first, as before
        self.assertIn('self.weigh_bag(', src)
        # the empty-jaws reading also arrives from below: a dip down the approach and back first
        self.assertLess(src.index("'bag_weight_dip'"), src.index('self.weight_loads('))
        weigh = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == 'weigh_bag')
        self.assertIn('unmeasured', ast.unparse(weigh))
        after = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == 'after_failure')
        self.assertIn("by_weight", ast.unparse(after))         # a bag held by weight is not forgotten either


if __name__ == '__main__':
    unittest.main()
