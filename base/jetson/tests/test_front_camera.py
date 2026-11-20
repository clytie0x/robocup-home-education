"""front_camera: the model (exact numbers), the calibration fit on synthetic room frames with known parameters,
and the bag's top and handle from synthetic frames (a box with a handle loop on a tiled floor)."""
import json
import math
import os
import socket
import sys
import tempfile
import unittest

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import front_camera as fc  # noqa: E402

RAD = math.radians
TRUE = fc.FrontCamera(fx=600.0, x=0.20, y=0.0, z=0.27, yaw=RAD(1.5), pitch=RAD(8.0), roll=RAD(-1.2), calibrated=True)
FLOOR = np.array([190.0, 205.0, 175.0])  # BGR: pale blue-green tiles
GROUT = np.array([150.0, 160.0, 150.0])
WALL = np.array([225.0, 225.0, 225.0])
BASEBOARD = np.array([25.0, 25.0, 25.0])  # black, 8 cm high
CABINET = np.array([100.0, 70.0, 50.0])
PINK = np.array([150.0, 110.0, 235.0])
HANDLE = np.array([40.0, 30.0, 160.0])


def pixel_rays(cam, ss=2):
    """Robot-frame unit rays of every sub-pixel (ss x ss per pixel): (H, W, ss*ss, 3)."""
    off = (np.arange(ss) + 0.5) / ss - 0.5
    u = np.arange(cam.width)[None, :, None, None] + off[None, None, :, None]
    v = np.arange(cam.height)[:, None, None, None] + off[None, None, None, :]
    u, v = np.broadcast_arrays(u, v)
    sign = -1.0 if cam.mirror else 1.0
    d = np.stack([sign * (u - cam.cx) / cam.fx, (v - cam.cy) / cam.fx, np.ones_like(u)], -1)
    d = d.reshape(cam.height, cam.width, ss * ss, 3) @ cam.rotation().T
    return d / np.linalg.norm(d, axis=-1, keepdims=True)


def floor_colour(q):
    gx = np.abs(((q[..., 0] + 0.15) % 0.30) - 0.15) < 0.003
    gy = np.abs(((q[..., 1] + 0.15) % 0.30) - 0.15) < 0.003
    return np.where((gx | gy)[..., None], GROUT, FLOOR)


def render_room(cam, pose, wall_x=2.26, edge_y=-0.5, ss=2):
    """The camera on the robot at world pose (x, y, yaw) before a flat wall at world x = wall_x: tiled floor,
    black baseboard, white wall, a dark cabinet front on the wall for world y < edge_y (a vertical edge)."""
    d = pixel_rays(cam, ss)
    o = cam.position
    px, py, th = pose
    c, s = math.cos(th), math.sin(th)
    # into the world frame
    dw = np.stack([c * d[..., 0] - s * d[..., 1], s * d[..., 0] + c * d[..., 1], d[..., 2]], -1)
    ow = np.array([px + c * o[0] - s * o[1], py + s * o[0] + c * o[1], o[2]])
    with np.errstate(divide="ignore", invalid="ignore"):
        t_floor = np.where(dw[..., 2] < -1e-9, -ow[2] / dw[..., 2], np.inf)
        t_wall = np.where(dw[..., 0] > 1e-9, (wall_x - ow[0]) / dw[..., 0], np.inf)
    img = np.zeros(d.shape[:-1] + (3,))
    on_floor = t_floor < t_wall
    t = np.where(on_floor, t_floor, t_wall)
    q = ow + np.where(np.isfinite(t), t, 0.0)[..., None] * dw
    wall = np.where((q[..., 1] < edge_y)[..., None], CABINET, WALL)
    wall = np.where((q[..., 2] < 0.08)[..., None], BASEBOARD, wall)
    img = np.where(on_floor[..., None], floor_colour(q), wall)
    img = np.where(np.isfinite(t)[..., None], img, WALL)
    return img.mean(axis=2).round().clip(0, 255).astype(np.uint8)


def robot_wall(pose, wall_x=2.26):
    """The world wall x = wall_x as (normal, dist) in the robot frame at pose (what the LiDAR line fit gives)."""
    px, py, th = pose
    return (-th, wall_x - px)


def frames_for(cam, poses, edge=True, wall_x=2.26, edge_y=-0.5):
    out = []
    for pose in poses:
        img = render_room(cam, pose, wall_x, edge_y)
        fl = fc.detect_floor_line(img)
        fr = {"pose": pose, "wall": robot_wall(pose, wall_x), "floor": fl["points"], "edges": {}}
        if edge:  # where the edge should be: the true camera says (the tool takes a hint from the user)
            q = fc._to_robot((wall_x, edge_y), pose)
            uv = cam._project(np.array([(q[0], q[1], 0.5)]))[0]
            if np.isfinite(uv).all() and 40 < uv[0] < cam.width - 40:
                pts = fc.detect_vertical_edge(img, uv[0], rows=(0, int(fl["points"][:, 1].min()) - 10))
                if len(pts):
                    fr["edges"]["cabinet"] = pts
        out.append(fr)
    return out


GUESS = fc.FrontCamera(fx=480.0, x=0.20, y=0.0, z=0.25)
TURNS = [RAD(a) for a in (-15, -7, 0, 7, 15)]


class Model(unittest.TestCase):
    def test_straight_ahead_lands_on_the_centre(self):
        cam = fc.FrontCamera(fx=500.0, x=0.2, y=0.0, z=0.3, calibrated=True)
        u, v = cam.project((2.2, 0.0, 0.3))
        self.assertAlmostEqual(u, 319.5, places=9)
        self.assertAlmostEqual(v, 239.5, places=9)

    def test_exact_numbers_left_and_below(self):
        cam = fc.FrontCamera(fx=500.0, x=0.2, y=0.0, z=0.3, calibrated=True)
        u, v = cam.project((1.2, 0.5, 0.1))  # 1 m ahead of the camera, 0.5 left, 0.2 below
        self.assertAlmostEqual(u, 319.5 - 250.0, places=9)  # left of the robot = left in the image
        self.assertAlmostEqual(v, 239.5 + 100.0, places=9)
        self.assertAlmostEqual(cam.copy(mirror=True).project((1.2, 0.5, 0.1))[0], 319.5 + 250.0, places=9)

    def test_pitch_down_moves_the_horizon_up(self):
        cam = fc.FrontCamera(fx=500.0, z=0.3, pitch=RAD(10), calibrated=True)
        u, v = cam.project((100.0 + 0.2, 0.0, 0.3))  # far away at camera height: the horizon
        self.assertAlmostEqual(v, 239.5 - 500.0 * math.tan(RAD(10)), delta=1e-3 * 500)

    def test_yaw_left_moves_the_scene_right(self):
        cam = fc.FrontCamera(fx=500.0, yaw=RAD(10), calibrated=True)
        u, _ = cam.project((cam.x + 1.0, cam.y, cam.z))
        self.assertAlmostEqual(u, 319.5 + 500.0 * math.tan(RAD(10)), places=6)

    def test_roll_right_side_down_makes_a_level_line_rise_to_the_right(self):
        cam = fc.FrontCamera(fx=500.0, roll=RAD(5), calibrated=True)
        _, vl = cam.project((cam.x + 2.0, cam.y + 0.5, cam.z - 0.2))
        _, vr = cam.project((cam.x + 2.0, cam.y - 0.5, cam.z - 0.2))
        self.assertLess(vr, vl)

    def test_ray_is_the_inverse_of_project(self):
        for mirror in (False, True):
            cam = TRUE.copy(mirror=mirror)
            p = np.array([1.3, -0.2, 0.05])
            u, v = cam.project(p)
            d = cam.ray(u, v)
            want = (p - cam.position) / np.linalg.norm(p - cam.position)
            self.assertTrue(np.allclose(d, want, atol=1e-9))

    def test_behind_the_camera_is_none(self):
        self.assertIsNone(TRUE.project((0.0, 0.0, 0.2)))

    def test_not_calibrated_until_a_calibration_says_so(self):
        cam = fc.FrontCamera.load(os.path.join(tempfile.gettempdir(), "no-such-front-camera.json"))
        self.assertFalse(cam.calibrated)
        with self.assertRaises(fc.NotCalibrated):
            cam.project((1.0, 0.0, 0.0))
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, "front_camera.json")
            TRUE.save(path)
            back = fc.FrontCamera.load(path)
            self.assertTrue(back.calibrated)
            self.assertAlmostEqual(back.pitch, TRUE.pitch, places=12)
            self.assertEqual(back.project((1.3, 0.1, 0.2)), TRUE.project((1.3, 0.1, 0.2)))
            with open(path, "w") as f:
                f.write("{broken")
            self.assertFalse(fc.FrontCamera.load(path).calibrated)


class Fit(unittest.TestCase):
    def check(self, got, truth=TRUE):
        cam = got["camera"]
        self.assertLess(abs(cam.fx / truth.fx - 1), 0.01, cam.fx)
        for k in fc.ANGLES:
            self.assertLess(abs(math.degrees(getattr(cam, k) - getattr(truth, k))), 0.3, (k, math.degrees(getattr(cam, k))))
        self.assertLess(abs(cam.z - truth.z), 0.01, cam.z)

    def test_the_wall_floor_line_is_found_in_a_rendered_frame(self):
        img = render_room(TRUE, (0.0, 0.0, 0.0))
        fl = fc.detect_floor_line(img)
        self.assertGreater(fl["count"], 250)
        line = fc._image_line(TRUE, *fc._floor_line_points(TRUE, robot_wall((0.0, 0.0, 0.0))))
        self.assertLess(np.abs(fc._line_dist(line, fl["points"])).max(), 0.5)

    def test_turns_at_two_wall_distances_recover_the_camera(self):
        poses = [(0.0, 0.0, a) for a in TURNS] + [(0.66, 0.0, a) for a in TURNS]  # 2.26 m and 1.60 m
        got = fc.fit(frames_for(TRUE, poses), GUESS)
        self.check(got)
        self.assertEqual(got["undetermined"], [])
        self.assertTrue(got["camera"].calibrated)
        self.assertLess(got["rms_px"], 0.5)
        self.assertLess(abs(got["edges"]["cabinet"]["y"] + 0.5), 0.01)

    def test_one_wall_distance_with_the_edge_also_recovers_it(self):
        got = fc.fit(frames_for(TRUE, [(0.0, 0.0, a) for a in TURNS]), GUESS)
        self.check(got)
        self.assertEqual(got["undetermined"], [])

    def test_the_rooms_three_turns_alone_say_what_they_cannot_tell(self):
        # the turns measured so far (0, 11.47, 20.70 deg) at one distance, the floor line only
        poses = [(0.0, 0.0, RAD(a)) for a in (0.0, 11.47, 20.70)]
        got = fc.fit(frames_for(TRUE, poses, edge=False), GUESS)
        self.assertEqual(got["undetermined"], ["fx", "yaw"])  # no edge: the focal length and the yaw stay open
        self.assertFalse(got["camera"].calibrated)
        self.assertIn("not determined", got["why"])

    def test_columns_of_one_edge_alone_are_too_few(self):
        frames = [{"pose": (0.0, 0.0, RAD(a)), "wall": None, "floor": None, "edges": {"cabinet": [[u, float("nan")]]}}
                  for a, u in ((0.0, 627.5), (11.47, 450.0), (20.70, 294.0))]
        with self.assertRaises(ValueError):
            fc.fit(frames, GUESS)

    def test_a_mirrored_image_is_told_from_the_rms(self):
        true = TRUE.copy(mirror=True)
        poses = [(0.0, 0.0, a) for a in TURNS] + [(0.66, 0.0, a) for a in TURNS]
        got = fc.fit_both_mirrors(frames_for(true, poses), GUESS)
        self.assertTrue(got["camera"].mirror)
        self.check(got, true)
        self.assertGreater(got["mirror_rms"]["false"], 5 * got["mirror_rms"]["true"])


def hit_box(o, d, centre, yaw, size):
    """t along rays d (..., 3) from o to a box standing on the floor, inf where they miss."""
    c, s = math.cos(yaw), math.sin(yaw)
    rot = np.array([[c, s, 0], [-s, c, 0], [0, 0, 1]])
    lo = np.array([-size[0] / 2, -size[1] / 2, 0.0])
    hi = np.array([size[0] / 2, size[1] / 2, size[2]])
    ol = rot @ (o - np.array([centre[0], centre[1], 0.0]))
    dl = d @ rot.T
    with np.errstate(divide="ignore", invalid="ignore"):
        t1, t2 = (lo - ol) / dl, (hi - ol) / dl
    tn = np.nanmax(np.minimum(t1, t2), axis=-1)
    tf = np.nanmin(np.maximum(t1, t2), axis=-1)
    return np.where((tf >= tn) & (tf > 0), tn, np.inf)


def hit_handle(o, d, centre, yaw, high, rise=0.07, half=0.055, thick=0.01):
    """t to an inverted-U handle loop in the bag's middle plane along its long axis, from its top up `rise`."""
    axis = np.array([math.cos(yaw), math.sin(yaw), 0.0])
    n = np.array([-math.sin(yaw), math.cos(yaw), 0.0])
    c = np.array([centre[0], centre[1], 0.0])
    with np.errstate(divide="ignore", invalid="ignore"):
        t = ((c - o) @ n) / (d @ n)
    q = o + np.where(np.isfinite(t), t, 0.0)[..., None] * d
    s_, z = (q - c) @ axis, q[..., 2]
    legs = (np.abs(np.abs(s_) - (half - thick / 2)) <= thick / 2) & (z >= high) & (z <= high + rise)
    bar = (np.abs(s_) <= half) & (z >= high + rise - thick) & (z <= high + rise)
    return np.where((legs | bar) & (t > 0), t, np.inf)


def render_bag(cam, centre, yaw, size, colour=PINK, wall_x=2.5, ss=2, handle=True):
    d = pixel_rays(cam, ss)
    o = cam.position
    with np.errstate(divide="ignore", invalid="ignore"):
        t_floor = np.where(d[..., 2] < -1e-9, -o[2] / d[..., 2], np.inf)
        t_wall = np.where(d[..., 0] > 1e-9, (wall_x - o[0]) / d[..., 0], np.inf)
    t_box = hit_box(o, d, centre, yaw, size)
    t_h = hit_handle(o, d, centre, yaw, size[2]) if handle else np.full(t_box.shape, np.inf)
    t = np.minimum.reduce([t_floor, t_wall, t_box, t_h])
    q = o + np.where(np.isfinite(t), t, 0.0)[..., None] * d
    wall = np.where((q[..., 2] < 0.08)[..., None], BASEBOARD, WALL)
    img = np.where((t == t_floor)[..., None], floor_colour(q), wall)
    img = np.where((t == t_box)[..., None], colour, img)
    img = np.where((t == t_h)[..., None], HANDLE, img)
    return img.mean(axis=2).round().clip(0, 255).astype(np.uint8)


def lidar_points(centre, yaw, size, lidar=(0.175, 0.0), step_deg=0.5):
    """Where the LiDAR's rays (every step_deg) hit the box at the LiDAR plane: the bag's points."""
    o = np.array([lidar[0], lidar[1], fc.LIDAR_Z])
    a = np.radians(np.arange(-60, 60, step_deg))
    d = np.stack([np.cos(a), np.sin(a), np.zeros_like(a)], -1)
    t = hit_box(o, d, centre, yaw, size)
    hit = np.isfinite(t)
    return (o + t[hit, None] * d[hit])[:, :2]


class Bag(unittest.TestCase):
    def measure(self, centre, yaw, size, colour=PINK, handle=True):
        img = render_bag(TRUE, centre, yaw, size, colour, handle=handle)
        return fc.measure_bag(TRUE, img, lidar_points(centre, yaw, size))

    def test_the_top_of_the_handle_and_its_place(self):
        for centre, yaw_deg, high in (((1.05, 0.02), 75, 0.28), ((1.40, -0.15), 100, 0.30), ((0.95, 0.10), 60, 0.25)):
            got = self.measure(centre, RAD(yaw_deg), (0.26, 0.11, high))
            self.assertTrue(got["ok"], got)
            self.assertEqual(got["top_by"], "range")
            self.assertLess(abs(got["top"] - (high + 0.07)), 0.01, (centre, got))
            self.assertLess(math.dist(got["handle"], centre), 0.03, (centre, got))
            self.assertGreaterEqual(got["conf"], 0.5)

    def test_a_wrong_camera_height_hardly_moves_the_range_way(self):
        img = render_bag(TRUE, (1.05, 0.02), RAD(75), (0.26, 0.11, 0.28))
        pts = lidar_points((1.05, 0.02), RAD(75), (0.26, 0.11, 0.28))
        low = fc.measure_bag(TRUE.copy(z=TRUE.z - 0.03), img, pts)  # 3 cm off
        self.assertLess(abs(low["top"] - 0.35), 0.012, low)
        self.assertAlmostEqual(low["top_height_way"], 0.35 - 0.03, delta=0.012)

    def test_without_a_handle_the_rim(self):
        got = self.measure((1.1, 0.0), RAD(80), (0.26, 0.11, 0.30), handle=False)
        self.assertTrue(got["ok"], got)
        self.assertLess(abs(got["top"] - 0.30), 0.01, got)

    def test_a_bag_like_the_floor_is_not_measured(self):
        grey = np.array([200.0, 205.0, 185.0])  # close to the tiles and the wall, no edges inside
        got = self.measure((1.1, 0.0), RAD(80), (0.26, 0.11, 0.30), colour=grey, handle=False)
        self.assertFalse(got["ok"], got)

    def test_not_calibrated_or_no_frame(self):
        pts = lidar_points((1.1, 0.0), RAD(80), (0.26, 0.11, 0.30))
        self.assertIn("not calibrated", fc.measure_bag(TRUE.copy(calibrated=False), None, pts)["why"])
        self.assertIn("no front camera frame", fc.measure_bag(TRUE, None, pts)["why"])
        self.assertFalse(fc.measure_bag(TRUE, np.zeros((480, 640, 3), np.uint8), pts[:2])["ok"])

    def test_a_learned_detector_can_replace_the_segmenter(self):
        class Nothing:
            def segment(self, image, roi, inner, seeds):
                return np.zeros((roi[3] - roi[1], roi[2] - roi[0]), bool)
        img = render_bag(TRUE, (1.1, 0.0), RAD(80), (0.26, 0.11, 0.30))
        got = fc.measure_bag(TRUE, img, lidar_points((1.1, 0.0), RAD(80), (0.26, 0.11, 0.30)), segmenter=Nothing())
        self.assertFalse(got["ok"])
        self.assertIn("not told from the background", got["why"])


class Service(unittest.TestCase):
    def test_a_request_over_tcp_gets_the_measurement(self):
        centre, yaw, size = (1.05, 0.02), RAD(75), (0.26, 0.11, 0.28)
        img = render_bag(TRUE, centre, yaw, size)
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, "front_camera.json")
            TRUE.save(path)
            got = fc.answer({"id": "7-1", "points": lidar_points(centre, yaw, size).tolist()}, path, lambda: img)
            self.assertEqual(got["id"], "7-1")
            self.assertLess(abs(got["top"] - 0.35), 0.01)
            json.dumps(got)  # the answer goes out as JSON
            saved = fc.fetch_snapshot
            fc.fetch_snapshot = lambda url=None, timeout=2.0: img
            try:
                server = fc.serve(0, path, host="127.0.0.1")
                port = server.server_address[1]
                with socket.create_connection(("127.0.0.1", port), timeout=5) as s:
                    s.sendall((json.dumps({"id": "7-2", "points": lidar_points(centre, yaw, size).tolist()}) + "\n").encode())
                    reply = json.loads(s.makefile().readline())
                with socket.create_connection(("127.0.0.1", port), timeout=5) as s:
                    s.sendall(b"not json\n")
                    bad = json.loads(s.makefile().readline())
                server.shutdown()
                server.server_close()
            finally:
                fc.fetch_snapshot = saved
        self.assertEqual(reply["id"], "7-2")
        self.assertTrue(reply["ok"], reply)
        self.assertFalse(bad["ok"])

    def test_uncalibrated_answers_without_a_top_and_takes_no_frame(self):
        taken = []
        got = fc.answer({"id": 1, "points": [[1, 0], [1, 0.1], [1, 0.2]]},
                        os.path.join(tempfile.gettempdir(), "no-such-front-camera.json"), lambda: taken.append(1))
        self.assertFalse(got["ok"])
        self.assertIsNone(got["top"])
        self.assertEqual(taken, [])


class FrameSize(unittest.TestCase):
    def test_the_calibration_size_is_unchanged(self):
        image = np.zeros((TRUE.height, TRUE.width, 3), np.uint8)
        self.assertIs(fc.for_image(TRUE, image), TRUE)

    def test_a_uniform_scale_doubles_fx_and_the_optical_centre(self):
        scaled = fc.for_image(TRUE, np.zeros((TRUE.height * 2, TRUE.width * 2, 3), np.uint8))
        self.assertAlmostEqual(scaled.fx, TRUE.fx * 2)
        self.assertAlmostEqual(scaled.cx, (TRUE.cx + 0.5) * 2 - 0.5)
        self.assertAlmostEqual(scaled.cy, (TRUE.cy + 0.5) * 2 - 0.5)

    def test_a_16_by_9_frame_is_not_measured(self):
        self.assertIsNone(fc.for_image(TRUE, np.zeros((720, 1280, 3), np.uint8)))
        got = fc.measure_bag(TRUE, np.zeros((720, 1280, 3), np.uint8), [[1.0, 0.0]])
        self.assertFalse(got["ok"])
        self.assertIn("not %dx%d" % (TRUE.width, TRUE.height), got["why"])


# The 2026-11-20 HD USB Camera candidate (docs/calibration). calibrated stays false in the file.
OLD_PINHOLE = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                           "calibration", "front-camera-2026-11-18", "front_camera.json")
CANDIDATE_FILE = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))), "docs", "calibration", "front_camera_fisheye_candidate_2026-11-20.json")
FISH_K = (-0.12000174022745945, 0.05392880504627527, 0.0, 0.0)


def candidate():
    """The candidate lens, marked calibrated so project and ray can be checked."""
    return fc.FrontCamera(fx=522.7004622679294, x=0.33, y=0.0, z=0.7933,
                          yaw=RAD(0.7606571023798528), pitch=RAD(46.954749902416516),
                          roll=RAD(0.9615972633301483), cx=296.70882730620366, cy=241.09751641118947,
                          dist_model="fisheye", dist=FISH_K, calibrated=True)


class Fisheye(unittest.TestCase):
    def test_an_unknown_lens_is_refused(self):
        with self.assertRaises(ValueError) as caught:
            fc.FrontCamera(dist_model="brown")
        self.assertIn("unknown dist_model", str(caught.exception))
        self.assertIn("brown", str(caught.exception))
        with self.assertRaises(ValueError) as caught:
            fc.FrontCamera.from_dict({"fx": 500.0, "x": 0.2, "y": 0.0, "z": 0.3, "dist_model": "brown"})
        self.assertIn("unknown dist_model", str(caught.exception))

    def test_an_old_pinhole_file_matches_the_plain_projection(self):
        with open(OLD_PINHOLE) as f:
            raw = json.load(f)
        self.assertNotIn("dist_model", raw)
        cam = fc.FrontCamera.from_dict(raw)
        plain = fc.FrontCamera(fx=raw["fx"], x=raw["x"], y=raw["y"], z=raw["z"],
                               yaw=RAD(raw["yaw_deg"]), pitch=RAD(raw["pitch_deg"]), roll=RAD(raw["roll_deg"]),
                               cx=raw["cx"], cy=raw["cy"], width=raw["width"], height=raw["height"],
                               mirror=raw["mirror"], calibrated=True)
        self.assertEqual(cam.dist_model, "pinhole")
        self.assertEqual(cam.dist, (0.0, 0.0, 0.0, 0.0))
        pts = np.array([[0.8, -0.2, 0.0], [1.2, 0.15, 0.05], [1.6, 0.0, 0.3], [1.0, 0.4, 0.1]])
        got = cam.project_many(pts)
        pc = (pts - cam.position) @ cam.rotation()
        want = np.column_stack([cam.cx + cam.fx * pc[:, 0] / pc[:, 2], cam.cy + cam.fx * pc[:, 1] / pc[:, 2]])
        self.assertTrue(np.array_equal(got, want))
        self.assertTrue(np.array_equal(got, plain.project_many(pts)))
        for u, v in ((10.0, 10.0), (320.0, 240.0), (600.0, 400.0)):
            d = np.array([(u - cam.cx) / cam.fx, (v - cam.cy) / cam.fx, 1.0])
            d = cam.rotation() @ d
            d = d / np.linalg.norm(d)
            self.assertTrue(np.array_equal(cam.ray(u, v), d))
            self.assertTrue(np.array_equal(cam.ray(u, v), plain.ray(u, v)))
        mirrored = plain.copy(mirror=True)
        u, v = 180.0, 300.0
        d = np.array([(u - mirrored.cx) / mirrored.fx * -1.0, (v - mirrored.cy) / mirrored.fx, 1.0])
        d = mirrored.rotation() @ d
        d = d / np.linalg.norm(d)
        self.assertTrue(np.array_equal(mirrored.ray(u, v), d))

    def test_stored_coefficients_do_nothing_on_a_pinhole(self):
        plain = fc.FrontCamera(fx=500.0, x=0.2, y=0.0, z=0.3, calibrated=True)
        held = plain.copy(dist=(-0.3, 0.2, 0.1, -0.05))
        self.assertEqual(held.dist_model, "pinhole")
        pts = np.array([[1.2, 0.4, 0.0], [2.0, -0.3, 0.2]])
        self.assertTrue(np.array_equal(plain.project_many(pts), held.project_many(pts)))

    def test_to_dict_round_trips_the_lens(self):
        cam = candidate()
        back = fc.FrontCamera.from_dict(cam.to_dict())
        self.assertEqual(back.dist_model, "fisheye")
        self.assertEqual(back.dist, FISH_K)
        self.assertEqual(back.to_dict()["dist"], list(FISH_K))
        full = fc.FrontCamera(fx=500.0, x=0.2, y=0.0, z=0.3, calibrated=True)
        raw = full.to_dict()
        self.assertEqual(raw["dist_model"], "pinhole")
        del raw["dist_model"], raw["dist"]
        pts = np.array([[1.5, 0.1, 0.0], [1.1, -0.2, 0.05]])
        self.assertTrue(np.array_equal(fc.FrontCamera.from_dict(raw).project_many(pts), full.project_many(pts)))

    def test_for_image_keeps_the_lens(self):
        cam = candidate()
        scaled = fc.for_image(cam, np.zeros((cam.height * 2, cam.width * 2, 3), np.uint8))
        self.assertEqual(scaled.dist_model, "fisheye")
        self.assertEqual(scaled.dist, FISH_K)
        u, v = 120.0, 360.0
        uu, vv = (u + 0.5) * 2 - 0.5, (v + 0.5) * 2 - 0.5
        self.assertTrue(np.allclose(cam.ray(u, v), scaled.ray(uu, vv), atol=1e-12))

    def test_pixels_round_trip_across_the_frame(self):
        for mirror in (False, True):
            cam = candidate().copy(mirror=mirror)
            err = 0.0
            for v in np.linspace(0, cam.height - 1, 25):
                for u in np.linspace(0, cam.width - 1, 33):
                    uv = cam.project(cam.position + cam.ray(float(u), float(v)))
                    err = max(err, abs(uv[0] - u), abs(uv[1] - v))
            self.assertLess(err, 1e-6)

    def test_the_projection_matches_opencv_fisheye(self):
        import cv2
        cam = candidate()
        pts = np.array([[x, y, z] for x in np.linspace(0.45, 2.2, 7) for y in np.linspace(-0.8, 0.8, 7)
                        for z in (0.0, 0.25)])
        uv = cam.project_many(pts)
        rvec, _ = cv2.Rodrigues(cam.rotation().T)
        tvec = -cam.rotation().T @ cam.position
        k = np.array([[cam.fx, 0.0, cam.cx], [0.0, cam.fx, cam.cy], [0.0, 0.0, 1.0]])
        cvp = cv2.fisheye.projectPoints(pts.reshape(-1, 1, 3), rvec, tvec.reshape(3, 1), k,
                                        np.array(cam.dist, float))[0].reshape(-1, 2)
        ok = np.isfinite(uv).all(axis=1)
        self.assertGreater(int(ok.sum()), 10)
        self.assertLess(float(np.abs(uv[ok] - cvp[ok]).max()), 1e-9)

    def test_the_ray_matches_opencv_undistort(self):
        import cv2
        cam = candidate()
        pixels = np.array([[0.0, 0.0], [639.0, 479.0], [320.0, 240.0], [100.0, 400.0], [500.0, 50.0]])
        k = np.array([[cam.fx, 0.0, cam.cx], [0.0, cam.fx, cam.cy], [0.0, 0.0, 1.0]])
        und = cv2.fisheye.undistortPoints(pixels.reshape(-1, 1, 2), k, np.array(cam.dist, float)).reshape(-1, 2)
        for (u, v), (x, y) in zip(pixels, und):
            d = cam.rotation().T @ cam.ray(float(u), float(v))
            self.assertAlmostEqual(d[0] / d[2], x, delta=1e-9)
            self.assertAlmostEqual(d[1] / d[2], y, delta=1e-9)

    def test_a_point_behind_the_fisheye_is_none(self):
        # above the lens: the camera looks down, so a floor point just behind its x can still be in front
        self.assertIsNone(candidate().project((0.33, 0.0, 2.0)))

    def test_a_harsh_lens_still_gives_a_finite_ray(self):
        cam = fc.FrontCamera(fx=500.0, x=0.3, y=0.0, z=0.8, pitch=RAD(45), calibrated=True,
                             dist_model="fisheye", dist=(-0.9, 0.8, -0.5, 0.2))
        for u, v in ((0.0, 0.0), (639.0, 479.0), (320.0, 240.0)):
            d = cam.ray(u, v)
            self.assertTrue(np.isfinite(d).all())
            self.assertAlmostEqual(float(np.linalg.norm(d)), 1.0, places=12)

    def test_a_short_distortion_is_padded(self):
        cam = fc.FrontCamera(dist_model="fisheye", dist=[-0.12, 0.054])
        self.assertEqual(cam.dist, (-0.12, 0.054, 0.0, 0.0))

    def test_the_fit_keeps_a_fixed_distortion(self):
        cam = candidate()
        out, _edges = fc._unpack(cam, ("fx",), (), np.array([math.log(500.0)]))
        self.assertEqual(out.dist_model, "fisheye")
        self.assertEqual(out.dist, FISH_K)
        self.assertAlmostEqual(out.fx, 500.0)
        self.assertAlmostEqual(out.pitch, cam.pitch)

    def test_the_candidate_file_stays_uncalibrated(self):
        with open(CANDIDATE_FILE) as f:
            raw = json.load(f)
        self.assertFalse(raw["calibrated"])
        self.assertEqual(raw["dist_model"], "fisheye")
        self.assertEqual(tuple(raw["dist"]), FISH_K)
        cam = fc.FrontCamera.from_dict(raw)
        self.assertFalse(cam.calibrated)
        self.assertEqual(cam.dist, FISH_K)
        with self.assertRaises(fc.NotCalibrated):
            cam.project((1.0, 0.0, 0.0))


if __name__ == "__main__":
    unittest.main()
