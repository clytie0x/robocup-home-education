"""Bag on the floor, arm on the robot: where to hold a bag of any shape, and pick / carry / put down plans."""
import math
import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, os.path.dirname(__file__))
from so100_driver.ik import JAW_G_MAX, Box, GraspPlanner, PlanError, bag_shifts, deck_box, jaw_gap  # noqa: E402
from so100_driver.tabletop import find_objects, grasp_top  # noqa: E402
from test_ik import kin  # noqa: E402

FLOOR, DECK_Z = -0.31, -0.068
PAN = np.array([0.0, -0.0452])


def bag_cloud(cx, cy, yaw, handles_up, seed=0, noise=0.002):
    """A soft shopping bag (not a box): bottom 18 x 11 cm, walls bulging out to 20 x 13 cm at mid height,
    rim 27 cm above the floor; two rope handles on the long walls, standing up (arcs to 37 cm) or fallen
    over to the outside."""
    rng = np.random.default_rng(seed)
    pts = []
    for z in np.arange(0.0, 0.27, 0.006):
        bulge = 1.0 + 0.12 * math.sin(math.pi * z / 0.27)
        a, b = 0.09 * bulge, 0.055 * bulge
        for s in np.linspace(0, 2 * math.pi, 90, endpoint=False):      # rounded-rectangle wall
            c, d = math.cos(s), math.sin(s)
            r = 1.0 / max(abs(c) / a, abs(d) / b) ** 0.85 / max(abs(c) / a, abs(d) / b) ** 0.15
            pts.append((r * c, r * d, z))
    for side in (-1, 1):                     # handles on the long walls (y = +-5.5 cm)
        for t in np.linspace(0, math.pi, 40):
            x = 0.045 * math.cos(t)
            h = 0.10 * math.sin(t)
            for w in (-0.003, 0.0, 0.003):
                if handles_up:
                    pts.append((x, side * 0.055 + w, 0.27 + h))
                else:                        # fallen outwards, hanging a bit below the rim
                    pts.append((x, side * (0.055 + 0.6 * h) + w, 0.27 - 0.4 * h))
    p = np.array(pts)
    c, s = math.cos(yaw), math.sin(yaw)
    p[:, :2] = p[:, :2] @ np.array([[c, s], [-s, c]]) + (cx, cy)
    p[:, 2] += FLOOR
    floor = np.column_stack([rng.uniform(cx - 0.3, cx + 0.3, 4000), rng.uniform(cy - 0.3, cy + 0.3, 4000),
                             np.full(4000, FLOOR)])
    return np.vstack([floor, p]) + rng.normal(0, noise, (len(p) + 4000, 3))


class BagGrasp(unittest.TestCase):
    def bag(self, handles_up, yaw=0.3):
        cx, cy = 0.02, -0.33
        objs = find_objects(bag_cloud(cx, cy, yaw, handles_up), FLOOR, z_min=0.02, z_max=0.55, min_points=60,
                            with_points=True)
        self.assertEqual(len(objs), 1)
        return grasp_top(objs[0]['pts'], FLOOR, PAN), (cx, cy, yaw)

    def near_arm(self, g, cx, cy):
        return np.linalg.norm(np.array([g['x'], g['y']]) - PAN) < np.linalg.norm(np.array([cx, cy]) - PAN) - 0.03

    def test_handles_up_holds_the_near_handle(self):
        g, (cx, cy, yaw) = self.bag(True)
        self.assertEqual(g['part'], 'side')           # two handles: too wide together, take the near one
        self.assertAlmostEqual(g['height'], 0.37, delta=0.015)
        self.assertLess(g['thickness'], 0.045)        # the jaws (max 6.5 cm) close across it
        self.assertTrue(self.near_arm(g, cx, cy))
        self.assertAlmostEqual(math.remainder(g['yaw'] - yaw, math.pi), 0.0, delta=0.25)

    def test_one_handle_up_is_held_as_it_is(self):
        cx, cy, yaw = 0.0, -0.33, -0.4
        pts = bag_cloud(cx, cy, yaw, True)
        far = (pts[:, 2] > FLOOR + 0.275) & ((pts[:, :2] - (cx, cy)) @ (-math.sin(yaw), math.cos(yaw)) > 0)
        objs = find_objects(pts[~far], FLOOR, z_min=0.02, z_max=0.55, min_points=60, with_points=True)
        g = grasp_top(objs[0]['pts'], FLOOR, PAN)
        self.assertEqual(g['part'], 'top')
        self.assertAlmostEqual(g['height'], 0.37, delta=0.015)
        self.assertLess(g['thickness'], 0.03)

    def test_handles_down_holds_the_near_rim_wall(self):
        g, (cx, cy, yaw) = self.bag(False)
        self.assertEqual(g['part'], 'side')
        self.assertAlmostEqual(g['height'], 0.27, delta=0.015)
        self.assertLess(g['thickness'], 0.045)
        self.assertTrue(self.near_arm(g, cx, cy))     # the long wall nearest the arm
        self.assertAlmostEqual(math.remainder(g['yaw'] - yaw, math.pi), 0.0, delta=0.25)


class BagPlan(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.k = kin()
        cls.body = deck_box(cls.k, 0.18, 0.22, 0.18, 0.22, DECK_Z, FLOOR)
        cls.pl = GraspPlanner(cls.k, FLOOR, [cls.body])

    def part(self, fwd, left, top, yaw=None):
        x, y = self.k.from_user(fwd, left)
        if yaw is None:
            yaw = math.atan2(y - self.k.pan_point[1], x - self.k.pan_point[0]) + math.pi / 2
        return Box(x, y, yaw, (0.05, 0.01, top))

    def test_pick_carry_put_down(self):
        obj = self.part(0.28, 0.0, 0.27)
        held = self.pl.plan_pick(obj, None, 'top')
        hang = held['grasp_centre'][2] - FLOOR
        held['carry'] = carry = self.pl.plan_carry(held, hang, math.pi / 2, 0.36)
        g = self.pl.held_point(carry['q'], held)
        self.assertGreaterEqual(g[2] - hang - FLOOR, 0.36 - 1e-6)          # bag bottom high enough
        f, l_ = self.k.to_user(g)
        self.assertGreater(l_, 0.18 + 0.05)                               # out at the left, off the deck
        self.assertLess(abs(f), 0.08)
        for q in carry['path']:
            self.assertFalse(self.pl.hits_body(q))
        place = self.pl.plan_place(obj, held, self.part(0.28, 0.06, 0.27))
        path = self.pl.plan_bag_place(held, place, hang)
        self.assertTrue(path)
        # put down: the bag bottom ends up just above the floor
        down = self.pl.held_point(place['steps']['lower'][-1], held)
        self.assertAlmostEqual(down[2] - hang - FLOOR, 0.005, delta=0.004)

    def test_bag_jaws_open_fully_and_still_carry(self):
        obj = self.part(0.28, 0.0, 0.27)
        held = self.pl.plan_pick(obj, None, 'top', wide=True)
        self.assertAlmostEqual(held['jaw_open'], JAW_G_MAX, places=6)
        self.assertAlmostEqual(held['gap'], jaw_gap(JAW_G_MAX), places=6)       # 7.5 cm, not thickness + 2.4
        self.assertLess(self.pl.plan_pick(obj, None, 'top')['gap'], 0.05)        # other picks unchanged
        hang = held['grasp_centre'][2] - FLOOR
        held['carry'] = self.pl.plan_carry(held, hang, math.pi / 2, 0.36)
        place = self.pl.plan_bag_put(obj, held, self.part(0.28, 0.06, 0.27))
        self.assertTrue(self.pl.plan_bag_place(held, place, hang))

    def test_wide_jaws_reach_as_much_in_front(self):
        ok = 0
        for fwd in (0.24, 0.27, 0.30):
            for top in (0.22, 0.27, 0.32):
                try:
                    held = self.pl.plan_pick(self.part(fwd, 0.0, top), None, 'top', wide=True)
                    self.pl.plan_carry(held, held['grasp_centre'][2] - FLOOR, math.pi / 2, 0.36)
                    ok += 1
                except PlanError:
                    pass
        self.assertGreaterEqual(ok, 8)

    def test_retries_overlap(self):
        thick = 0.02
        shifts = bag_shifts(thick, 0.05, 3)
        cover = jaw_gap(JAW_G_MAX) - thick          # where a 2 cm part fits between the open jaws: 5.5 cm
        self.assertEqual(len(shifts), 3)
        self.assertEqual(shifts[0], 0.0)
        self.assertAlmostEqual(shifts[1], -shifts[2])
        xs = sorted(shifts)
        for a, b in zip(xs, xs[1:]):
            self.assertLessEqual(b - a, cover - 0.01 + 1e-9)    # neighbouring tries overlap by 1 cm or more
        self.assertEqual(bag_shifts(thick, 0.03, 3), [0.0, 0.03, -0.03])   # a smaller step is kept
        self.assertEqual(bag_shifts(thick, 0.05, 1), [0.0])

    def test_body_blocks_reaching_under_the_deck(self):
        # 15 cm in front of the axis is above the deck: the arm cannot go down to a 22 cm top there
        with self.assertRaises(PlanError):
            self.pl.plan_pick(self.part(0.15, 0.0, 0.22), None, 'top')

    def test_mostly_reachable_in_front(self):
        ok = 0
        for fwd in (0.24, 0.27, 0.30):
            for top in (0.22, 0.27, 0.32):
                try:
                    held = self.pl.plan_pick(self.part(fwd, 0.0, top), None, 'top')
                    self.pl.plan_carry(held, held['grasp_centre'][2] - FLOOR, math.pi / 2, 0.36)
                    ok += 1
                except PlanError:
                    pass
        self.assertGreaterEqual(ok, 8)


if __name__ == '__main__':
    unittest.main()
