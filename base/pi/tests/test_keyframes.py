import math
import unittest

from keyframes import Keyframes
from test_scanmatch import CORRIDOR, P, ROOM, box, crowd, world_scan

KEY = (0.5, 0.3, 0.2)  # where the keyframe was stored, in the corrected frame


def stored(walls=ROOM, pose=KEY):
    frames = Keyframes(P)
    frames.add(pose, world_scan(pose, walls))
    return frames


def off(pose, dx, dy, dyaw):
    return (pose[0] + dx, pose[1] + dy, pose[2] + dyaw)


class Correct(unittest.TestCase):
    def test_drift_near_a_keyframe_is_corrected(self):
        truth = off(KEY, 0.3, -0.2, math.radians(8))
        believed = off(truth, 0.12, 0.08, math.radians(-4))
        pose, report = stored().correct(believed, world_scan(truth, ROOM, start_deg=0.4))
        self.assertTrue(report["used"], report)
        self.assertEqual(report["index"], 0)
        self.assertAlmostEqual(pose[0], truth[0], delta=0.02)
        self.assertAlmostEqual(pose[1], truth[1], delta=0.02)
        self.assertAlmostEqual(math.degrees(pose[2] - truth[2]), 0.0, delta=0.5)
        self.assertAlmostEqual(report["shift"], math.hypot(0.12, 0.08), delta=0.02)
        self.assertAlmostEqual(math.degrees(report["turn"]), 4.0, delta=0.5)
        self.assertGreater(report["share"], 0.9)

    def test_reversed_heading_matches_the_same_place(self):
        truth = off(KEY, -0.2, 0.3, math.pi + math.radians(10))  # driving back past it
        believed = off(truth, -0.1, 0.1, math.radians(5))
        pose, report = stored().correct(believed, world_scan(truth, ROOM, start_deg=0.4))
        self.assertTrue(report["used"], report)
        self.assertAlmostEqual(pose[0], truth[0], delta=0.02)
        self.assertAlmostEqual(pose[1], truth[1], delta=0.02)
        self.assertAlmostEqual(math.degrees(math.remainder(pose[2] - truth[2], 2 * math.pi)), 0.0, delta=0.5)
        self.assertAlmostEqual(pose[2], believed[2] - math.radians(5), delta=0.01)  # yaw keeps the believed pose's turn count

    def test_a_plain_corridor_corrects_sideways_and_heading_only(self):
        # Review Focus 1: along the corridor the match cannot tell, so that part stays on odometry
        key = (0.0, 0.0, 0.0)
        truth = (0.6, 0.15, math.radians(4))
        believed = (0.9, 0.05, 0.0)
        pose, report = stored(CORRIDOR, key).correct(believed, world_scan(truth, CORRIDOR, start_deg=0.4))
        self.assertTrue(report["used"], report)
        self.assertAlmostEqual(pose[0], believed[0], delta=1e-6)
        self.assertAlmostEqual(pose[1], truth[1], delta=0.02)
        self.assertAlmostEqual(math.degrees(pose[2]), 4.0, delta=0.5)

    def test_people_in_the_scan_lower_the_share_and_the_match_is_skipped(self):
        # Review Focus 5
        truth = off(KEY, 0.1, 0.0, 0.0)
        believed = off(truth, 0.05, 0.0, 0.0)
        pose, report = stored().correct(believed, world_scan(truth, ROOM, crowd(truth[0], truth[1]), start_deg=0.4))
        self.assertFalse(report["used"])
        self.assertEqual(report["why"], "share")
        self.assertLess(report["share"], 0.6)
        self.assertEqual(pose, believed)

    def test_a_far_keyframe_is_not_used(self):
        frames = stored()
        for believed in (off(KEY, 1.1, 0.0, 0.0), off(KEY, 0.2, 0.0, math.radians(60))):
            pose, report = frames.correct(believed, world_scan(believed, ROOM))
            self.assertFalse(report["used"])
            self.assertIsNone(report["index"])
            self.assertEqual(report["why"], "far")
            self.assertEqual(pose, believed)

    def test_the_nearest_keyframe_is_used(self):
        frames = stored()
        frames.add((2.0, 0.3, 0.2), world_scan((2.0, 0.3, 0.2), ROOM))
        truth = (1.7, 0.4, 0.2)
        _, report = frames.correct(off(truth, 0.05, 0.0, 0.0), world_scan(truth, ROOM, start_deg=0.4))
        self.assertEqual(report["index"], 1)
        self.assertTrue(report["used"], report)

    def test_a_large_correction_is_refused(self):
        truth = off(KEY, 0.1, 0.0, 0.0)
        # 25 deg off in ROOM the far walls move over 0.5 m and the match fails ("share"); a small room converges
        small = box(-0.6, -0.9, 1.6, 1.4)
        for believed, walls, why in ((off(truth, 0.0, 0.0, math.radians(25)), small, "turn"), (off(truth, 0.4, 0.4, 0.0), ROOM, "shift")):
            pose, report = stored(walls).correct(believed, world_scan(truth, walls, start_deg=0.4))
            self.assertFalse(report["used"])
            self.assertEqual(report["why"], why, report)
            self.assertEqual(pose, believed)

    def test_a_keyframe_without_a_usable_scan_gives_way_to_the_next_nearest(self):
        frames = Keyframes(P)
        frames.add(KEY, [(0.0, 1.0)] * 5)  # too few points to match
        second = off(KEY, 0.5, 0.0, 0.0)
        frames.add(second, world_scan(second, ROOM))
        truth = off(KEY, 0.1, 0.0, 0.0)  # nearer the first
        pose, report = frames.correct(off(truth, 0.05, 0.0, 0.0), world_scan(truth, ROOM, start_deg=0.4))
        self.assertTrue(report["used"], report)
        self.assertEqual(report["index"], 1)
        self.assertAlmostEqual(pose[0], truth[0], delta=0.02)

    def test_no_keyframes_and_no_match(self):
        pose, report = Keyframes(P).correct(KEY, world_scan(KEY, ROOM))
        self.assertEqual((pose, report["used"], report["why"]), (KEY, False, "far"))
        pose, report = stored().correct(KEY, [(0.0, 1.0)] * 5)  # too few points to match
        self.assertEqual((pose, report["used"], report["why"]), (KEY, False, "no match"))


if __name__ == "__main__":
    unittest.main()
