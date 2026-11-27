"""The bag's distance from its foot (bag_vision.foot_edge, bag_camera.locate) on real front-camera frames.

Each frame has the bag11n-seg-v3 outline it got on this PC (OpenCV dnn on the Jetson's ONNX), saved as a PNG, so
the test needs no model. Distances: the blue bag was taped at 20 and 25 cm from the pan axis to its front wall
(owner, 2026-11-24); the pink bag was over about 27 cm (owner, 2026-11-27) and its corners in the picture
read 26.8 to 27.2 cm.
"""
import json
import os
import re
import unittest

import cv2

from so100_driver import bag_camera as bc
from so100_driver import bag_vision as bv

DATA = os.path.join(os.path.dirname(__file__), "data")
HALF = 0.055
# docs/calibration/front-camera-2026-11-20-discs/front_camera_2026-11-20.json
CAM = {"calibrated": True, "fx": 522.7004622679294, "cx": 296.70882730620366, "cy": 241.09751641118947, "width": 640,
       "height": 480, "x": 0.225, "y": -0.0009974647691394832, "z": 0.83, "yaw_deg": -0.2975755040136706,
       "pitch_deg": 47.65724106386179, "roll_deg": 0.9615972633301483, "mirror": False, "dist_model": "fisheye",
       "dist": [-0.12000174022745945, 0.05392880504627527, 0.0, 0.0]}


def node_default(name):
    path = os.path.join(os.path.dirname(__file__), "..", "so100_driver", "pick_place_node.py")
    with open(path, encoding="utf-8") as f:
        return json.loads(re.search(r'declare_parameter\("%s", (\[.*?\])\)' % name, f.read()).group(1))


def blob(name):
    im = cv2.imread(os.path.join(DATA, name + ".jpg"))
    mask = cv2.imread(os.path.join(DATA, name + "_v3mask.png"), cv2.IMREAD_GRAYSCALE) > 0
    b = bv._blob_from_mask(mask & (bv.orange_mask(im) == 0), bv.orange_mask(im), "yolo-seg")
    b["front_rim"] = bv.front_rim(im, b)
    return im, b


class FootEdge(unittest.TestCase):
    def wall(self, name, rim_h):
        im, b = blob(name)
        loc = bc.locate(bc.FrontCamera(CAM), b, node_default("bag_front_cam_pan"), HALF, rim_h, foot=bv.foot_row(im, b))
        return loc, loc["centre"][0] - HALF

    def test_the_foot_gives_the_taped_distance(self):
        for name, want, rim_h in (("20261124_blue_25cm", 0.25, 0.37), ("20261124_blue_25cm_b", 0.25, 0.37),
                                  ("20261127_pink_27cm", 0.27, 0.29)):
            loc, wall = self.wall(name, rim_h)
            self.assertEqual(loc["from"], "foot", name)
            self.assertGreaterEqual(loc["confidence"], bc.CONF_MIN, name)
            self.assertAlmostEqual(wall, want, delta=0.02, msg=name)

    def test_the_outline_bottom_alone_reads_too_near(self):
        """What foot_edge corrects: the outline runs over the bag's shadow."""
        im, b = blob("20261127_pink_27cm")
        loc = bc.locate(bc.FrontCamera(CAM), b, node_default("bag_front_cam_pan"), HALF, 0.29, foot=b["y1"])
        self.assertLess(loc["centre"][0] - HALF, 0.235)

    def test_a_foot_out_of_the_picture_is_not_found(self):
        """Taped at 20 cm: the foot is below the picture; the outline's side columns must not pass for it."""
        im, b = blob("20261124_blue_20cm")
        self.assertIsNone(bv.foot_edge(im, b))

    def test_without_the_foot_the_front_rim_of_an_open_bag(self):
        """Taped at 20 cm, rim 37 cm high (measured): the outline's top (the back rim) read 30.7 cm."""
        im, b = blob("20261124_blue_20cm")
        self.assertEqual(b["front_rim"]["kind"], "open")
        loc, wall = self.wall("20261124_blue_20cm", 0.37)
        self.assertEqual(loc["from"], "front rim")
        self.assertAlmostEqual(wall, 0.20, delta=0.015)
        self.assertGreaterEqual(bc.front_rim_trust(loc, "measured")["confidence"], bc.CONF_MIN)
        self.assertLess(bc.front_rim_trust(loc, "fallback")["confidence"], bc.CONF_MIN)
        b.pop("front_rim")
        old = bc.locate(bc.FrontCamera(CAM), b, node_default("bag_front_cam_pan"), HALF, 0.37)
        self.assertGreater(old["centre"][0] - HALF, 0.28)

    def test_the_front_rim_where_the_foot_is_seen_too(self):
        for name in ("20261124_blue_25cm", "20261124_blue_25cm_b"):
            im, b = blob(name)
            self.assertEqual(b["front_rim"]["kind"], "open", name)
            c = bc._at_height(bc.FrontCamera(CAM), b["front_rim"]["u"], b["front_rim"]["row"], 0.37)
            self.assertAlmostEqual(c[0] - node_default("bag_front_cam_pan")[0], 0.25, delta=0.015, msg=name)

    def test_the_floor_above_a_closed_bag_is_not_its_inside(self):
        """The pink bag's outline ran up over the floor: that band is not an open mouth."""
        im, b = blob("20261127_pink_27cm")
        self.assertEqual(b["front_rim"]["kind"], "floor")

    def test_a_foot_beyond_the_top_is_not_used(self):
        """A piece of the bag between the raised arm's links: its bottom is not the foot."""
        im, b = blob("20261127_pink_27cm")
        loc = bc.locate(bc.FrontCamera(CAM), b, node_default("bag_front_cam_pan"), HALF, 0.29, foot=250.0)
        self.assertEqual(loc["from"], "rim")
        self.assertIn("foot_dropped", loc)


if __name__ == "__main__":
    unittest.main()
