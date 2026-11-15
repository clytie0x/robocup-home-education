import math
import random
import unittest
from pathlib import Path

import numpy as np

from so100_driver.ik import ARM, FIXED_INNER_X, GRASP_Y, Box, GraspPlanner, Kinematics, PlanError, object_offset

CONFIG = Path(__file__).resolve().parents[2] / 'so100_moveit_config'
URDF = (CONFIG / 'urdf' / 'so100.urdf').read_text()
MESHES = str(CONFIG / 'meshes')
# limits of the real arm (calibration.json 2026-11-07, with the 20-count margin)
REAL = {'shoulder_pan': (-2.11, 1.90), 'shoulder_lift': (0.0, 3.50), 'elbow_flex': (-3.34, 0.0),
        'wrist_flex': (-2.52, 0.56), 'wrist_roll': (-1.48, 4.74)}
TABLE = -0.075


def kin(meshes=True):
    return Kinematics(URDF, REAL, MESHES if meshes else None)


class IKTest(unittest.TestCase):
    def test_fk_ik_roundtrip(self):
        k = kin()
        rnd = random.Random(1)
        hits = 0
        for _ in range(300):
            q = {n: rnd.uniform(*REAL[n]) for n in ARM}
            q['wrist_roll'] = 0.0
            w = k.frames(q)
            p = k.gripper_point(q, (0, GRASP_Y, 0))
            tool = -w['wrist'][:3, 1]
            pitch = math.atan2(tool[2], math.hypot(tool[0], tool[1]))
            if math.hypot(*(p[:2] - k.pan_point)) < 0.03:
                continue
            sols = k.solve(p, pitch)
            for s in sols:
                self.assertLess(np.linalg.norm(k.gripper_point(s, (0, GRASP_Y, 0)) - p), 1e-5)
                t = -k.frames(s)['wrist'][:3, 1]
                self.assertAlmostEqual(math.atan2(t[2], math.hypot(t[0], t[1])), pitch, places=4)
            hits += bool(sols)
        self.assertGreater(hits, 150)

    def test_user_frame(self):
        k = kin()
        fwd, left = k.to_user(k.from_user(0.2, -0.05))
        self.assertAlmostEqual(fwd, 0.2)
        self.assertAlmostEqual(left, -0.05)
        ready = {'shoulder_pan': 0.0, 'shoulder_lift': 1.2, 'elbow_flex': -1.2, 'wrist_flex': 0.3, 'wrist_roll': 0}
        f, l_ = k.to_user(k.frames(ready)['tcp'][:3, 3])
        self.assertGreater(f, 0.1)
        self.assertAlmostEqual(l_, 0.0, places=4)
        # arm points along base -y, so base +x is the arm's left
        self.assertGreater(k.to_user(k.pan_point + np.array([0.1, 0.0]))[1], 0.09)


class PlannerTest(unittest.TestCase):
    def setUp(self):
        self.k = kin()
        self.p = GraspPlanner(self.k, TABLE)

    def box(self, fwd, left, yaw_deg=0.0, size=(0.03, 0.03, 0.05)):
        x, y = self.k.from_user(fwd, left)
        return Box(x, y, math.radians(yaw_deg), size)

    def test_top_down_pick_and_place(self):
        plan = self.p.plan(self.box(0.20, 0.0), self.box(0.18, 0.10))
        # (nearly) straight down; the real wrist_flex range stops exactly -90 deg at this spot
        self.assertLessEqual(math.degrees(plan['pitch']), -65)
        g = plan['steps']['approach'][-1]
        tool = -self.k.frames(g)['wrist'][:3, 1]
        self.assertLess(tool[2], -0.9)
        # approach is a straight line along the tool axis, ending at the object
        pts = [self.k.gripper_point(q, (object_offset(plan['gap']), GRASP_Y, 0)) for q in plan['steps']['approach']]
        d = pts[-1] - pts[0]
        for p in pts:
            off = (p - pts[0]) - d * ((p - pts[0]) @ d) / (d @ d)
            self.assertLess(np.linalg.norm(off), 0.001)
        self.assertLess(np.linalg.norm(pts[-1] - plan['grasp_centre']), 0.002)

    def test_jaws_line_up_with_narrow_side(self):
        obj = self.box(0.20, 0.05, yaw_deg=30, size=(0.06, 0.025, 0.05))   # narrow along own y
        plan = self.p.plan(obj, self.box(0.20, -0.08))
        close = self.k.close_axis(plan['steps']['approach'][-1])
        self.assertGreater(abs(close @ obj.axes()[1]), 0.95)
        self.assertAlmostEqual(plan['extent'], 0.025)

    def test_object_centre_between_jaws(self):
        obj = self.box(0.22, -0.04, size=(0.04, 0.04, 0.06))
        plan = self.p.plan(obj, self.box(0.16, 0.12))
        q = plan['steps']['approach'][-1]
        gap = plan['gap']
        mid = self.k.gripper_point(q, (object_offset(gap), GRASP_Y, 0))
        self.assertLess(np.linalg.norm(mid[:2] - [obj.x, obj.y]), 0.002)
        # the object sits ~6 mm from the fixed finger, the moving jaw has the larger margin
        fixed_face = self.k.gripper_point(q, (FIXED_INNER_X, GRASP_Y, 0))
        close = self.k.close_axis(q)
        to_fixed = (fixed_face - mid) @ close - 0.02       # object half-width 2 cm
        self.assertAlmostEqual(to_fixed, 0.006, places=3)

    def test_long_thin_object_lying_forward(self):
        # 15 x 4 x 2 cm bar pointing away from the arm: jaws must close across the 4 cm side
        # (left/right jaw roll ties at +-90 deg; this used to flip-flop and fail)
        size = (0.15, 0.04, 0.02)
        plan = self.p.plan(self.box(0.22, 0.0, size=size), self.box(0.20, -0.10, size=size))
        self.assertAlmostEqual(plan['extent'], 0.04)
        # unreachable target must say so, not blame the object width
        with self.assertRaises(PlanError) as cm:
            self.p.plan(self.box(0.22, 0.0, size=size), self.box(0.257, -0.271, size=size))
        self.assertNotIn('เกินก้าม', str(cm.exception))

    def test_no_link_goes_below_the_table(self):
        # 2026-11-09 on the real arm: 15x4x2 cm bar at 26.5 cm, target far right -> a 15 deg side grasp
        # put the wrist motor on the table. Every pose of every plan must keep all links above it.
        size = (0.15, 0.04, 0.02)
        obj, tgt = self.box(0.265, 0.002, size=size), self.box(0.016, -0.30, size=size)
        try:
            plan = self.p.plan(obj, tgt)
        except PlanError:
            return                                   # refusing is fine
        for name, qs in plan['steps'].items():
            for q in (qs if isinstance(qs, list) else [qs]):
                self.assertGreater(self.k.lowest(q, GraspPlanner.ARM_LINKS), TABLE + 0.01, name)

    def test_reach_map_is_collision_checked(self):
        bad = 0
        for fwd in np.arange(0.12, 0.34, 0.04):
            for left in (-0.15, 0.0, 0.15):
                try:
                    plan = self.p.plan(self.box(fwd, left, size=(0.03, 0.03, 0.02)), self.box(0.18, 0.0))
                except PlanError:
                    continue
                for q in plan['steps']['approach']:
                    bad += self.k.lowest(q, GraspPlanner.ARM_LINKS) < TABLE + 0.01
        self.assertEqual(bad, 0)

    def test_object_exactly_at_jaw_limit(self):
        # 9 x 6.5 x 2.4 cm on the real arm (2026-11-09): 6.5 cm must not be rejected by float rounding
        sz = (0.09, 0.065, 0.024)
        plan = self.p.plan(self.box(0.261, 0.0, -90, sz), self.box(0.125, -0.268, -90, sz))   # yaw -90 = as the panel starts
        self.assertAlmostEqual(plan['extent'], 0.065, places=4)

    def test_place_map_matches_full_plans(self):
        sz = (0.03, 0.03, 0.05)
        obj = self.box(0.20, 0.0, -90, sz)
        cells = [tuple(self.k.from_user(f, l)) for f in np.arange(0.04, 0.40, 0.04) for l in np.arange(-0.3, 0.31, 0.06)]
        ok = self.p.place_map(obj, obj.yaw, cells)
        self.assertTrue(any(ok))
        self.assertFalse(ok[cells.index(min(cells, key=lambda c: -np.hypot(*(np.array(c) - self.k.pan_point))))])
        agree = 0
        for (x, y), good in zip(cells, ok):
            try:
                self.p.plan(obj, Box(x, y, obj.yaw, sz))
                full = True
            except PlanError:
                full = False
            agree += full == good
        self.assertGreaterEqual(agree / len(cells), 0.9)

    def test_tall_thin_object_approach_starts_above_its_top(self):
        # 6 x 2 x 9 cm on the real arm (2026-11-09): the pre-grasp jaw overlapped the object
        sz = (0.06, 0.02, 0.09)
        obj = self.box(0.25, 0.05, -90, sz)
        plan = self.p.plan(obj, self.box(0.18, -0.12, -90, sz))
        top = TABLE + sz[2]
        q = plan['steps']['pre']
        tips = min(self.k.gripper_point(q, p)[2] for p in ((0.013, -0.106, 0), (-0.04, -0.104, 0)))
        self.assertGreater(tips, top + 0.015)

    def test_pick_then_place_separately_and_grasp_modes(self):
        sz = (0.03, 0.03, 0.05)
        obj = self.box(0.20, 0.0, -90, sz)
        held = self.p.plan_pick(obj)
        self.assertNotIn('lower', held['steps'])
        full = self.p.plan_place(obj, held, self.box(0.18, -0.10, -90, sz))
        self.assertIn('lower', full['steps'])
        self.assertLessEqual(self.p.plan_pick(obj, mode='top')['pitch'], GraspPlanner.NEAR_DOWN)
        self.assertGreater(self.p.plan_pick(self.box(0.30, 0.0, -90, sz), mode='side')['pitch'], GraspPlanner.NEAR_DOWN)
        with self.assertRaises(PlanError) as cm:
            self.p.plan_place(obj, held, self.box(0.60, 0.0, -90, sz))
        self.assertEqual(cm.exception.code, 'UNREACHABLE')
        with self.assertRaises(PlanError) as cm:
            self.p.plan_pick(self.box(0.2, 0.0, size=(0.09, 0.09, 0.05)))
        self.assertEqual(cm.exception.code, 'TOO_WIDE')

    def test_unreachable_and_too_wide(self):
        with self.assertRaises(PlanError):
            self.p.plan(self.box(0.60, 0.0), self.box(0.2, 0.0))
        with self.assertRaises(PlanError) as cm:
            self.p.plan(self.box(0.2, 0.0, size=(0.09, 0.09, 0.05)), self.box(0.2, 0.1))
        self.assertIn('เกินก้าม', str(cm.exception))

    def test_tips_stay_above_table(self):
        plan = self.p.plan(self.box(0.20, 0.0, size=(0.03, 0.03, 0.015)), self.box(0.2, 0.1))
        for q in plan['steps']['approach'] + plan['steps']['lower']:
            self.assertGreater(self.p._tips_low(q, plan['gap']), TABLE + 0.0005)

    def test_reach_map_has_a_usable_area(self):
        ok = 0
        for fwd in np.arange(0.10, 0.36, 0.02):
            for left in np.arange(-0.2, 0.21, 0.04):
                try:
                    self.p.plan(self.box(fwd, left), self.box(0.2, 0.0))
                    ok += 1
                except PlanError:
                    pass
        self.assertGreater(ok, 30)


if __name__ == '__main__':
    unittest.main()
