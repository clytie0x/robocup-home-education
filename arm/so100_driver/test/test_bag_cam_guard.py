"""The front camera may not move a grasp, or open the jaws, on evidence that is not a located bag.

Real frames: the blue bag (2026-11-20 22:47, two looks of one pick, nothing moved between them). The segmenter found
no bag and the floor-difference fallback took the lamp glare on the floor for it: 0.52 m and 0.73 m for one bag.
"""
import json
import os
import re
import tempfile
import unittest
from types import SimpleNamespace as NS

import cv2
import numpy as np

from so100_driver import bag_camera as bc

DATA = os.path.join(os.path.dirname(__file__), "data")
LOOKS = tuple("20261120T224703-245811-1_%s.jpg" % k for k in ("look", "look2"))
# docs/calibration/front-camera-2026-11-20-discs/front_camera_2026-11-20.json
CAM = {"calibrated": True, "fx": 522.7004622679294, "cx": 296.70882730620366, "cy": 241.09751641118947, "width": 640,
       "height": 480, "x": 0.225, "y": -0.0009974647691394832, "z": 0.83, "yaw_deg": -0.2975755040136706,
       "pitch_deg": 47.65724106386179, "roll_deg": 0.9615972633301483, "mirror": False, "dist_model": "fisheye",
       "dist": [-0.12000174022745945, 0.05392880504627527, 0.0, 0.0]}


def node_default(name):
    """The value pick_place_node.py declares (read from the source: the node needs ROS to import)."""
    path = os.path.join(os.path.dirname(__file__), "..", "so100_driver", "pick_place_node.py")
    with open(path, encoding="utf-8") as f:
        return json.loads(re.search(r'declare_parameter\("%s", (\[.*?\])\)' % name, f.read()).group(1))


PARAMS = {"bag_cam_rim_h": 0.27, "bag_cam_half_depth": 0.055, "bag_cam_width": 0.18,
          "bag_front_cam_pan": node_default("bag_front_cam_pan"), "bag_model": "coco"}
GOAL = (0.321, -0.026)


def frame(name):
    return cv2.imread(os.path.join(DATA, name))


class NoBag:
    """A segmenter that finds no bag."""

    def segment(self, _im):
        return []


class Guard(unittest.TestCase):
    def setUp(self):
        fd, self.cal = tempfile.mkstemp(suffix=".json")
        with os.fdopen(fd, "w") as f:
            json.dump(CAM, f)
        node = NS(kin=NS(heading0=-np.pi / 2, to_user=lambda xy: GOAL), joints={}, say=lambda _s: None,
                  get_logger=lambda: NS(info=lambda *_: None))
        self.cam = bc.CameraBag(node)
        self.cam.p = lambda name: self.cal if name == "bag_front_cam_file" else PARAMS[name]
        self.cam.vision_model = lambda: ("coco", None)
        self.seg, bc.bv._get_segmenter = bc.bv._get_segmenter, lambda *a: NoBag()
        self.sleep, bc.time.sleep = bc.time.sleep, lambda _s: None
        self.obj = NS(x=0.0, y=0.0, yaw=0.0, yaw_given=False, size=(0.2, 0.1, float("nan")))

    def tearDown(self):
        bc.bv._get_segmenter, bc.time.sleep = self.seg, self.sleep
        os.unlink(self.cal)

    def see(self, *frames):
        left = list(frames)
        self.cam.web = lambda: left.pop(0) if len(left) > 1 else left[0]

    def test_the_real_frames_are_the_floor_difference_fallback(self):
        """What the pick located before the guard: two very different places for one bag."""
        fwd = []
        for name in LOOKS:
            b = bc.bv.find_bag(frame(name), segmenter=NoBag())
            self.assertEqual(b["source"], "floor-diff")
            self.assertFalse(bc.trusted(b))
            loc = bc.locate(bc.FrontCamera(CAM), b, PARAMS["bag_front_cam_pan"], 0.055, 0.27)
            fwd.append(loc["centre"][0])
        self.assertGreater(abs(fwd[0] - fwd[1]), 0.15)

    def test_glare_does_not_move_the_goal_on_the_first_or_a_later_look(self):
        for first in (True, False):
            self.see(*(frame(n) for n in LOOKS))
            got = self.cam.where(self.obj, first=first)
            self.assertEqual(got["centre"], GOAL)
            self.assertEqual(got["from"], "goal")

    def test_no_bag_at_all_on_a_later_look_is_still_an_error(self):
        self.see(np.full((480, 640, 3), 120, np.uint8))
        self.assertEqual(self.cam.where(self.obj, first=True)["centre"], GOAL)
        with self.assertRaises(bc.Failed) as got:
            self.cam.where(self.obj, first=False)
        self.assertEqual(got.exception.code, "MISSED")

    def test_a_detected_bag_with_only_its_rim_keeps_the_goal(self):
        # 5 cm off the goal: inside LIDAR_DISAGREE, so only the confidence gate stops it
        edges = bc.reconcile_edges((GOAL[0] + 0.05, GOAL[1]), None)
        loc = dict(edges, width=0.18, standing=None)
        got = bc.resolve_grasp([loc], GOAL, 0.18)
        self.assertEqual((got["xy"], got["source"]), (GOAL, "request"))

    def test_a_foot_on_the_floor_still_wins_over_the_goal(self):
        edges = bc.reconcile_edges(None, (GOAL[0] + 0.05, GOAL[1]))
        loc = dict(edges, width=0.18, standing=True)
        self.assertEqual(bc.resolve_grasp([loc], GOAL, 0.18)["source"], "camera")

    def test_a_handle_does_not_make_the_rim_distance_confident(self):
        tops = np.array([[c, 300.0] for c in range(250, 350)])
        blob = {"x0": 250.0, "x1": 349.0, "y0": 300.0, "y1": 479.0, "tops": tops, "all_tops": tops,
                "handle": {"cx": 300.0, "cy": 290.0, "area": 50}}
        loc = bc.locate(bc.FrontCamera(CAM), blob, PARAMS["bag_front_cam_pan"], 0.055, 0.27)
        self.assertEqual(loc["from"], "handle")
        self.assertLess(loc["confidence"], bc.CONF_MIN)

    def test_glare_still_there_after_the_lift_never_opens_the_jaws(self):
        """The glare outline can stay in place with the bag gone. False would drop a bag that is in the jaws."""
        self.cam.in_view = lambda *a: False
        before, after = (frame(n) for n in LOOKS)
        start = bc.bv.find_bag(before, segmenter=NoBag())
        start["frame"] = before
        # the old verdict: the same outline again = "the bag is still on the floor"
        self.assertFalse(bc.spot_left(start, bc.bv.find_bag(after, segmenter=NoBag())))
        self.see(after)
        self.assertIsNone(self.cam.place_left(start))
        # detected before, only glare after, and that spot of the picture still shows the bag
        self.assertIsNone(self.cam.place_left(dict(start, source="yolo-seg")))

    def test_the_bag_taped_at_20_cm_reads_20_cm(self):
        """2026-11-24: front wall taped 20 cm from the pan axis, rim 35 to 37 cm high. The picture's front rim,
        two frames. It read 26 to 27 cm with the pan axis taken from the LiDAR."""
        pan = PARAMS["bag_front_cam_pan"]
        for (u0, v0), (u1, v1) in (((171, 388), (417, 389)), ((183, 390), (428, 386))):
            tops = np.array([[u, v0 + (v1 - v0) * (u - u0) / (u1 - u0)] for u in range(u0, u1)], float)
            blob = {"x0": float(u0), "x1": float(u1), "y0": float(min(v0, v1)), "y1": 479.0, "tops": tops,
                    "all_tops": tops}
            for rim_h in (0.35, 0.37):
                loc = bc.locate(bc.FrontCamera(CAM), blob, pan, 0.04, rim_h)
                self.assertAlmostEqual(loc["centre"][0] - 0.04, 0.20, delta=0.015)
                self.assertAlmostEqual(loc["centre"][1], 0.0, delta=0.02)

    def test_the_close_bias_only_where_it_was_measured(self):
        bias = [-0.012, -0.014]
        self.assertEqual(bc.close_bias(0.31, bias), (-0.012, -0.014))
        self.assertEqual(bc.close_bias(bc.BIAS_FROM_R, bias), (-0.012, -0.014))
        self.assertEqual(bc.close_bias(0.25, bias), (0.0, 0.0))


if __name__ == "__main__":
    unittest.main()
