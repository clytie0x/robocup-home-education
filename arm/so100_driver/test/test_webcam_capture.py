"""The AX1080P is asked for MJPG at the calibration size, and importing the viewer does not open it."""
import importlib.util
import unittest
from pathlib import Path

VIEW = Path(__file__).resolve().parents[2] / "webcam_view.py"


def load_view():
    spec = importlib.util.spec_from_file_location("webcam_view_under_test", VIEW)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


class FakeCv:
    CAP_PROP_FOURCC = 1
    CAP_PROP_FRAME_WIDTH = 2
    CAP_PROP_FRAME_HEIGHT = 3
    CAP_PROP_FPS = 4
    CAP_V4L2 = 200

    @staticmethod
    def VideoWriter_fourcc(*chars):
        return "".join(chars)


class FakeCap:
    def __init__(self):
        self.sets = []

    def set(self, prop, value):
        self.sets.append((prop, value))
        return True


class Capture(unittest.TestCase):
    def test_import_does_not_need_cv2(self):
        mod = load_view()
        self.assertEqual((mod.WIDTH, mod.HEIGHT), (640, 480))
        self.assertEqual(mod.FOURCC, "MJPG")
        self.assertEqual(mod.FPS, 30)

    def test_fourcc_is_set_before_the_size(self):
        mod = load_view()
        cap = FakeCap()
        mod.configure_capture(cap, FakeCv())
        props = [prop for prop, _value in cap.sets]
        self.assertLess(props.index(FakeCv.CAP_PROP_FOURCC), props.index(FakeCv.CAP_PROP_FRAME_WIDTH))
        self.assertLess(props.index(FakeCv.CAP_PROP_FRAME_WIDTH), props.index(FakeCv.CAP_PROP_FRAME_HEIGHT))
        self.assertEqual(cap.sets[0][1], "MJPG")
        self.assertEqual(cap.sets[1][1], 640)
        self.assertEqual(cap.sets[2][1], 480)


if __name__ == "__main__":
    unittest.main()
