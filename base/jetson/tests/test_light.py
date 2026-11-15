"""light: the frame gate, the light rule for looks, normalise, the lighting changes and the depth washout; and the
pure parts of tools/light_bench.py (no TensorRT: a fake detector and the stand-in look)."""
import os
import sys
import tempfile
import unittest

import cv2
import numpy as np

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "tools"))

import light  # noqa: E402
import light_bench  # noqa: E402


def grey(level, h=240, w=320):
    return np.full((h, w, 3), level, np.uint8)


def scene():
    """A 320 x 240 frame with some texture: a grey wall, a dark floor and a reddish 'person' box."""
    rng = np.random.default_rng(1)
    img = np.clip(rng.normal(110, 20, (240, 320, 3)), 0, 255).astype(np.uint8)
    img[160:] = (60, 70, 65)
    img[40:220, 120:180] = (60, 60, 170)
    return img


class GateTests(unittest.TestCase):
    def test_a_black_frame_is_dark_and_a_normal_one_passes(self):
        self.assertEqual(light.FrameGate().check(grey(0)), (False, "dark"))
        self.assertEqual(light.FrameGate().check(grey(int(light.DARK_MEAN) - 2)), (False, "dark"))
        self.assertEqual(light.FrameGate().check(scene()), (True, None))

    def test_a_mostly_clipped_frame_is_saturated(self):
        img = scene()
        img[:120] = 255  # half the frame blown out
        self.assertEqual(light.FrameGate().check(img), (False, "saturated"))
        img = scene()
        img[:40] = 255  # a sixth: a bright window, still fine
        self.assertEqual(light.FrameGate().check(img), (True, None))

    def test_a_sudden_jump_is_a_flash_and_a_lasting_change_becomes_normal(self):
        gate = light.FrameGate()
        for _ in range(light.FLASH_HISTORY):
            self.assertEqual(gate.check(grey(30)), (True, None))
        self.assertEqual(gate.check(grey(160)), (False, "flash"))  # 5.3 times brighter in one frame
        self.assertEqual(gate.check(grey(30)), (True, None))
        verdicts = [gate.check(grey(160)) for _ in range(light.FLASH_HISTORY)]
        self.assertEqual(verdicts[-1], (True, None))  # the lights stayed up: the new normal
        self.assertLessEqual(sum(1 for ok, _ in verdicts if not ok), light.FLASH_HISTORY // 2 + 1)

    def test_a_light_flickering_to_0_3_is_never_refused(self):
        # light_bench: frames at 0.3 of the light kept recall 1.0 and looks >= 0.80 (stand-in), so not refused
        gate = light.FrameGate()
        frames = [light.dim(scene(), g, seed=i) for i, g in enumerate(light.flicker_gains(10, 30, 60))]
        self.assertEqual([gate.check(f)[0] for f in frames], [True] * 60)

    def test_no_flash_without_enough_history(self):
        gate = light.FrameGate()
        gate.check(grey(30))
        self.assertEqual(gate.check(grey(200)), (True, None))


class LightRuleTests(unittest.TestCase):
    def test_box_light_is_the_mean_luma_in_the_box(self):
        img = grey(0)
        img[100:200, 50:100] = 200
        self.assertAlmostEqual(light.box_light(img, [50, 100, 100, 200]), 200.0, places=3)
        self.assertAlmostEqual(light.box_light(img, [0, 0, 320, 240]), 200 * 5000 / 76800, places=3)
        self.assertGreaterEqual(light.box_light(img, [400, 300, 400, 300]), 0.0)  # off the frame: never empty

    def test_light_ok_within_the_ratio_and_unknown_is_ok(self):
        self.assertTrue(light.light_ok(80.0, 80.0))
        self.assertTrue(light.light_ok(160.0, 80.0))
        self.assertTrue(light.light_ok(40.0, 80.0))
        self.assertFalse(light.light_ok(177.0, 75.0))  # light_bench "spot on body": 177.8 against 75.4
        self.assertFalse(light.light_ok(23.0, 75.0))  # dim 0.3 without auto exposure
        self.assertTrue(light.light_ok(None, 80.0))
        self.assertTrue(light.light_ok(80.0, None))


class NormaliseTests(unittest.TestCase):
    def test_grayworld_removes_a_colour_cast(self):
        warm = light.cast(scene(), 0.65, 0.9, 1.2)
        means = light.normalise(warm, "grayworld").reshape(-1, 3).mean(axis=0)
        self.assertLess(means.max() - means.min(), 3.0)

    def test_clahe_spreads_a_dim_frame_and_keeps_its_size(self):
        dark = light.dim(scene(), 0.2)
        out = light.normalise(dark, "clahe")
        self.assertEqual(out.shape, dark.shape)
        self.assertGreater(light.luma(out).std(), light.luma(dark).std())

    def test_none_is_the_same_frame_and_an_unknown_mode_raises(self):
        img = scene()
        self.assertIs(light.normalise(img, "none"), img)
        for mode in light.MODES:
            self.assertEqual(light.normalise(img, mode).shape, img.shape)
        with self.assertRaises(ValueError):
            light.normalise(img, "sepia")


class ChangesTests(unittest.TestCase):
    def test_each_change_keeps_the_frame_shape_and_type(self):
        img = scene()
        for out in [light.dim(img, 0.3), light.dim_auto(img, 0.3), light.gamma(img, 2.0), light.overexpose(img, 3),
                    light.spotlight(img, (150, 120), 80), light.shadow(img), light.cast(img, 1.2, 1.0, 0.7),
                    light.bands(img)]:
            self.assertEqual((out.shape, out.dtype), (img.shape, np.uint8))

    def test_dim_darkens_and_auto_exposure_brings_the_level_back(self):
        img = scene()
        self.assertAlmostEqual(light.dim(img, 0.3).mean(), img.mean() * 0.3, delta=1.0)
        self.assertAlmostEqual(light.dim_auto(img, 0.3).mean(), img.mean(), delta=2.0)

    def test_the_spotlight_saturates_its_centre_only(self):
        out = light.spotlight(grey(60), (160, 120), 60)
        self.assertEqual(int(out[120, 160, 0]), 255)
        self.assertEqual(int(out[0, 0, 0]), 60)

    def test_flicker_switches_at_the_asked_rate(self):
        gains = light.flicker_gains(5, 30, 30)
        switches = sum(1 for a, b in zip(gains, gains[1:]) if a != b)
        self.assertEqual(switches, 9)  # 5 Hz square wave, 1 s at 30 frames/s: about 10 edges
        self.assertEqual(set(gains), {1.0, 0.3})


class DepthTests(unittest.TestCase):
    def test_washout_makes_holes_and_keeps_the_other_readings(self):
        depth = np.full((240, 320), 1500, np.uint16)
        washed = light.wash_depth(depth, (160, 120), 30)
        self.assertEqual(int(washed[120, 160]), 0)
        self.assertEqual(int(washed[0, 0]), 1500)
        self.assertEqual(set(np.unique(washed)), {0, 1500})
        self.assertAlmostEqual(np.count_nonzero(light.wash_depth(depth, (0, 0), 0, dropout=0.5)) / depth.size, 0.5,
                               delta=0.01)

    def test_valid_share_reads_the_person_range_patch(self):
        depth = np.zeros((240, 320), np.uint16)
        box = [100, 0, 200, 200]  # patch: columns 125 to 175, rows 100 to 180
        self.assertEqual(light.person_patch(box, depth.shape), ((100, 180), (125, 175)))
        depth[100:140, 125:175] = 1200
        self.assertAlmostEqual(light.valid_share(depth, box), 0.5)
        self.assertEqual(light.valid_share(depth, [10, 10, 10, 10]), 0.0)


class FakeDetector:
    """Finds the reddish box of scene() unless the frame is nearly black."""

    def detect(self, bgr):
        return [(120.0, 40.0, 180.0, 220.0, 0.9)] if bgr.mean() > 5 else []


class BenchTests(unittest.TestCase):
    def test_load_frames_reads_images_and_npz_with_depth(self):
        with tempfile.TemporaryDirectory() as d:
            cv2.imwrite(os.path.join(d, "a.jpg"), scene())
            cv2.imwrite(os.path.join(d, "big.png"), cv2.resize(scene(), (640, 480)))
            np.savez(os.path.join(d, "c.npz"), rgb=scene()[:, :, ::-1], depth=np.ones((3, 240, 320), np.uint16))
            open(os.path.join(d, "notes.txt"), "w").close()
            frames = light_bench.load_frames(d)
        self.assertEqual([n for n, _, _ in frames], ["a.jpg", "big.png", "c.npz"])
        self.assertEqual({f[1].shape for f in frames}, {(240, 320, 3)})
        self.assertIsNone(frames[0][2])
        self.assertEqual(frames[2][2].shape, (3, 240, 320))
        self.assertTrue(np.array_equal(frames[2][1], scene()))  # RGB in the npz, BGR out

    def test_bench_rows_with_a_fake_detector(self):
        depth = np.full((1, 240, 320), 1500, np.uint16)
        rows = light_bench.bench([("s.npz", scene(), depth)], FakeDetector(), light_bench.HistLook(), ["none"],
                                 flicker=True)
        light_rows = {r["condition"]: r for r in rows if r["kind"] == "light"}
        self.assertEqual(light_rows["original"]["recall"], 1.0)
        self.assertEqual(light_rows["original"]["look"], 1.0)
        self.assertEqual(light_rows["black"]["recall"], 0.0)
        self.assertEqual(light_rows["black"]["refused"], {"dark": 1})
        self.assertEqual(light_rows["black every 5th (gate kept)"]["recall"], 1.0)
        self.assertEqual(light_rows["black every 5th"]["gate"], 0.2)
        depth_rows = [r for r in rows if r["kind"] == "depth"]
        self.assertEqual(depth_rows[0]["valid_share"], 1.0)
        self.assertEqual(depth_rows[2]["falls_back"], 1.0)  # a disc of 0.3 box heights over the patch
        self.assertEqual([r["mode"] for r in rows if r["kind"] == "timing"], ["none"])

    def test_iou(self):
        self.assertEqual(light_bench.iou([0, 0, 10, 10], [0, 0, 10, 10]), 1.0)
        self.assertAlmostEqual(light_bench.iou([0, 0, 10, 10], [5, 0, 15, 10]), 1 / 3)
        self.assertEqual(light_bench.iou([0, 0, 10, 10], [20, 20, 30, 30]), 0.0)


if __name__ == "__main__":
    unittest.main()
