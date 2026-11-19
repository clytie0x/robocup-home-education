"""bag_choice.py: every bag in a front camera frame, its colour and side, and the one the operator names."""
import os
import sys
import unittest

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

import bag_choice as bcx  # noqa: E402

try:
    import pick_bag_cam as pbc  # noqa: E402

    BV, BC = pbc.arm_modules()
except ImportError:  # the arm team's tree is not on this machine
    BV = BC = None

CAM = {"calibrated": True, "fx": 805.66, "cx": 319.5, "cy": 239.5, "x": 0.33, "y": 0.0, "z": 0.89,
       "yaw_deg": 0.0, "pitch_deg": 61.37, "roll_deg": 0.0, "mirror": False}
FLOOR = (200, 190, 120)  # BGR, cyan-ish
RED, BLUE, WHITE, VIOLET = (40, 30, 210), (200, 60, 30), (235, 235, 235), (170, 40, 90)


def patch(colour, size=60):
    img = np.full((size, size, 3), colour, np.uint8)
    return img, np.ones((size, size), bool)


def frame(*bags):
    """bags: (x0, x1, y0, y1, BGR) rectangles on the floor; also returns their masks."""
    img = np.full((480, 640, 3), FLOOR, np.uint8)
    masks = []
    for x0, x1, y0, y1, colour in bags:
        img[y0:y1, x0:x1] = colour
        m = np.zeros((480, 640), np.uint8)
        m[y0:y1, x0:x1] = 1
        masks.append(m)
    return img, masks


class FakeSeg:
    def __init__(self, masks):
        self.masks = masks

    def segment(self, _bgr):
        return [(0.9, (0, 0, 1, 1), m) for m in self.masks]  # the coco head's 3-tuples


def bag(colour, side, ok=True, y=0.0):
    return {"colour": colour, "side": side, "ok": ok, "y": y, "x": 0.3}


class ColourTest(unittest.TestCase):
    def test_plain_colours(self):
        for colour, name in ((RED, "red"), (BLUE, "blue"), (WHITE, "white"), ((20, 20, 20), "black"),
                             ((40, 200, 40), "green"), ((30, 220, 230), "yellow")):
            self.assertEqual(bcx.colour_name(*patch(colour)), name, name)

    def test_a_printed_logo_does_not_change_the_colour(self):
        img, m = patch(BLUE, 80)
        img[30:45, 30:50] = WHITE  # a logo, a fifth of the bag
        self.assertEqual(bcx.colour_name(img, m), "blue")


class ChooseTest(unittest.TestCase):
    TWO = [bag("blue", "left", y=0.1), bag("red", "right", y=-0.1)]

    def test_one_bag_needs_no_words(self):
        self.assertEqual(bcx.choose([bag("red", None)]), (0, "the only bag"))

    def test_two_bags_and_no_words_ask_which_one(self):
        self.assertEqual(bcx.choose(self.TWO), (None, "which one"))

    def test_by_colour_and_by_side(self):
        self.assertEqual(bcx.choose(self.TWO, colour="red")[0], 1)
        self.assertEqual(bcx.choose(self.TWO, side="left")[0], 0)
        self.assertEqual(bcx.choose(self.TWO, colour="navy")[0], 0)  # a word for blue
        self.assertEqual(bcx.choose(self.TWO, side="centre"), (None, "no bag on the middle"))

    def test_a_close_colour_counts_only_when_no_bag_has_the_very_one(self):
        bags = [bag("purple", "left", y=0.1), bag("red", "right", y=-0.1)]
        self.assertEqual(bcx.choose(bags, colour="blue")[0], 0)
        bags = [bag("purple", "left", y=0.1), bag("blue", "right", y=-0.1)]
        self.assertEqual(bcx.choose(bags, colour="blue")[0], 1)

    def test_a_colour_nobody_has_or_two_of(self):
        self.assertEqual(bcx.choose(self.TWO, colour="green"), (None, "no green bag"))
        two_red = [bag("red", "left", y=0.1), bag("red", "right", y=-0.1)]
        self.assertEqual(bcx.choose(two_red, colour="red"), (None, "more than one red bag"))
        self.assertEqual(bcx.choose(two_red, colour="red", side="right")[0], 1)

    def test_out_of_reach_bags_are_never_chosen(self):
        self.assertEqual(bcx.choose([bag("red", None, ok=False)], colour="red"), (None, "no bag within reach"))

    def test_describe(self):
        self.assertEqual(bcx.describe(self.TWO), "a blue bag on the left and a red bag on the right")
        self.assertEqual(bcx.describe([bag("red", None)]), "a red bag")


@unittest.skipIf(BV is None, "no arm team source tree")
class ListBagsTest(unittest.TestCase):
    def setUp(self):
        self.cam = BC.FrontCamera(CAM)

    def test_two_bags_left_and_right_with_their_colours(self):
        img, masks = frame((90, 310, 330, 470, BLUE), (340, 560, 330, 470, RED))
        bags = bcx.list_bags(img, self.cam, BV, BC, segmenters=[FakeSeg(masks)])
        self.assertEqual([(b["colour"], b["side"], b["ok"]) for b in bags], [("blue", "left", True), ("red", "right", True)],
                         bags)
        self.assertGreater(bags[0]["y"], bags[1]["y"])
        self.assertEqual(bcx.choose(bags, colour="red")[0], 1)

    def test_the_same_bag_from_two_models_counts_once(self):
        img, masks = frame((240, 390, 330, 470, RED))
        bags = bcx.list_bags(img, self.cam, BV, BC, segmenters=[FakeSeg(masks), FakeSeg(masks)])
        self.assertEqual(len(bags), 1)
        self.assertIsNone(bags[0]["side"])

    def test_a_far_or_odd_sized_bag_is_listed_but_not_ok(self):
        img, masks = frame((40, 600, 300, 470, RED))
        bags = bcx.list_bags(img, self.cam, BV, BC, segmenters=[FakeSeg(masks)])
        self.assertFalse(bags[0]["ok"])
        self.assertTrue(bags[0]["why"])


if __name__ == "__main__":
    unittest.main()


@unittest.skipIf(BV is None, "no arm team source tree")
class NoiseTest(unittest.TestCase):
    def test_a_weak_detection_or_an_empty_answer_is_no_bag(self):
        cam = BC.FrontCamera(CAM)
        img, masks = frame((240, 460, 330, 470, RED))

        class Weak(FakeSeg):
            def segment(self, _bgr):
                return [(0.2, (0, 0, 1, 1), m) for m in self.masks]
        self.assertEqual(bcx.list_bags(img, cam, BV, BC, segmenters=[Weak(masks)]), [])
        self.assertEqual(bcx.list_bags(img, cam, BV, BC, segmenters=[FakeSeg([])]), [])
