"""tools/front_cam_cal.py: a session of saved synthetic frames fitted offline, written only when determined."""
import json
import math
import os
import sys
import tempfile
import unittest

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), "tools"))
sys.path.insert(0, HERE)

import cv2  # noqa: E402

import front_cam_cal  # noqa: E402
import front_camera as fc  # noqa: E402
from test_front_camera import TRUE, render_room  # noqa: E402


def write_session(folder, turns_deg, backs=(0.0, 0.66), edge=True, world_wall=True):
    frames = []
    for back in backs:
        for a in turns_deg:
            pose = (back, 0.0, math.radians(a))
            name = "f%.2f_%+.1f.png" % (back, a)
            cv2.imwrite(os.path.join(folder, name), render_room(TRUE, pose))
            fr = {"image": name, "pose": [back, 0.0, a]}
            if not world_wall:
                fr["wall"] = {"normal_deg": -a, "dist": 2.26 - back}
            q = fc._to_robot((2.26, -0.5), pose)
            u = TRUE._project(np.array([(q[0], q[1], 0.5)]))[0][0]
            if edge and 40 < u < 600:
                fr["edges"] = {"cabinet": {"u": round(float(u)), "rows": [0, 200]}}
            frames.append(fr)
    session = {"camera_guess": {"fx": 480, "x": 0.2, "y": 0.0, "z": 0.25}, "walls": [{"normal_deg": 0, "dist": 2.26}],
               "frames": frames}
    path = os.path.join(folder, "session.json")
    with open(path, "w") as f:
        json.dump(session, f)
    return path


class Tool(unittest.TestCase):
    def test_a_full_session_is_fitted_and_written(self):
        with tempfile.TemporaryDirectory() as d:
            path = write_session(d, (-15, -7, 0, 7, 15), world_wall=False)
            out = os.path.join(d, "front_camera.json")
            lines = []
            self.assertEqual(front_cam_cal.main(["fit", path, "--write", out], log=lines.append), 0, "\n".join(lines))
            cam = fc.FrontCamera.load(out)
        self.assertTrue(cam.calibrated)
        self.assertFalse(cam.mirror)
        self.assertLess(abs(cam.fx / TRUE.fx - 1), 0.01)
        for k in fc.ANGLES:
            self.assertLess(abs(math.degrees(getattr(cam, k) - getattr(TRUE, k))), 0.3, k)
        self.assertLess(abs(cam.z - TRUE.z), 0.01)
        self.assertTrue(any("NOT DETERMINED" not in s and s.strip().startswith("fx") for s in lines))

    def test_three_turns_without_an_edge_write_nothing_and_say_why(self):
        with tempfile.TemporaryDirectory() as d:
            path = write_session(d, (0.0, 11.47, 20.70), backs=(0.0,), edge=False)
            out = os.path.join(d, "front_camera.json")
            lines = []
            self.assertEqual(front_cam_cal.main(["fit", path, "--write", out], log=lines.append), 1)
            self.assertFalse(os.path.exists(out))
        text = "\n".join(lines)
        self.assertIn("NOT DETERMINED", text)
        self.assertIn("not written", text)

    def test_the_rooms_columns_alone_cannot_be_fitted(self):
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, "session.json")
            with open(path, "w") as f:
                json.dump({"frames": [{"pose": [0, 0, a], "edges": {"cabinet": [[u, None]]}}
                                      for a, u in ((0, 627.5), (11.47, 450.0), (20.70, 294.0))]}, f)
            lines = []
            self.assertEqual(front_cam_cal.main(["fit", path], log=lines.append), 1)
        self.assertIn("more frames are needed", "\n".join(lines))

    def test_detect_shows_the_line(self):
        with tempfile.TemporaryDirectory() as d:
            img = os.path.join(d, "f.png")
            cv2.imwrite(img, render_room(TRUE, (0.0, 0.0, 0.0)))
            lines = []
            self.assertEqual(front_cam_cal.main(["detect", img, "--out", os.path.join(d, "o.png")], log=lines.append), 0)
            self.assertTrue(os.path.exists(os.path.join(d, "o.png")))
        self.assertIn("wall-floor line from", lines[0])


if __name__ == "__main__":
    unittest.main()
