"""Floor bag pick: SO-100 kinematics, the pick/place sequence on a fake arm, the gripper camera
(calibration fit, handle detection, visual servo) and the CLI's dry run. No ROS needed."""
import importlib.util
import io
import json
import math
import os
import sys
import tempfile
import unittest
from contextlib import redirect_stdout

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import so100_ik  # noqa: E402
from so100_ik import ARM, Kinematics, Mount, Unreachable  # noqa: E402

HAVE_CV2 = importlib.util.find_spec("cv2") is not None


def kin(**kw):
    return Kinematics(Mount(**kw))


class MountTest(unittest.TestCase):
    def test_pan_axis_maps_to_the_lidar_position(self):
        m = Mount()
        x, y, z = m.to_robot(np.array([0.0, -0.0452, 0.0]))
        self.assertAlmostEqual(x, 0.012, places=6)
        self.assertAlmostEqual(y, 0.0145, places=6)
        self.assertAlmostEqual(z, 0.305 + 0.026, places=6)

    def test_arm_forward_is_minus_y_and_left_is_plus_x(self):
        m = Mount()
        p = m.to_robot(np.array([0.05, -0.0452 - 0.20, 0.0]))   # API.md example: 20 cm forward, 5 cm left
        self.assertAlmostEqual(p[0], 0.012 + 0.20, places=6)
        self.assertAlmostEqual(p[1], 0.0145 + 0.05, places=6)

    def test_round_trip_with_yaw_and_offset(self):
        m = Mount(yaw=math.radians(3), z_offset=0.03)
        p = np.array([0.4, -0.1, 0.25])
        np.testing.assert_allclose(m.to_robot(m.to_arm(p)), p, atol=1e-12)

    def test_z_offset_lowers_every_command(self):
        a = Mount(z_offset=0.0).to_arm(np.array([0.4, 0.0, 0.25]))
        b = Mount(z_offset=0.026).to_arm(np.array([0.4, 0.0, 0.25]))
        self.assertAlmostEqual(a[2] - b[2], 0.026, places=9)


class KinematicsTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.k = kin()

    def test_fk_matches_the_urdf_tcp_at_zero(self):
        # so100.urdf at q = 0: tcp (gripper origin + 0.09 along -y) measured with the arm team's ik.py
        f = self.k.fk({n: 0.0 for n in ARM})
        np.testing.assert_allclose(f["tcp"][:3, 3], [0.0, -0.2314, 0.1061], atol=2e-4)
        np.testing.assert_allclose(f["shoulder"][:3, 3], [0.0, -0.0452, 0.0165], atol=1e-9)

    def test_ik_round_trip_over_the_floor_area(self):
        worst, n = 0.0, 0
        for x in np.arange(0.24, 0.46, 0.02):
            for y in np.arange(-0.16, 0.17, 0.04):
                for z in (0.20, 0.22, 0.245, 0.27, 0.30):
                    for pitch in (-90, -75, -60, -45, -30):
                        try:
                            q = self.k.ik((x, y, z), math.radians(pitch))
                        except Unreachable:
                            continue
                        p = self.k.grasp_point(q)
                        worst = max(worst, float(np.linalg.norm(p - (x, y, z))))
                        self.assertAlmostEqual(self.k.pitch(q), math.radians(pitch), places=4)
                        self.k.check_limits(q)
                        n += 1
        # the band in front of the deck that the arm reaches at bag-neck heights (doc: floor-pick.md)
        self.assertGreater(n, 80)
        self.assertLess(worst, 0.002)

    def test_neck_height_band_in_front_of_the_robot(self):
        for x in (0.29, 0.32, 0.35):
            q, pitch = self.k.ik_any_pitch((x, 0.0145, 0.245))
            np.testing.assert_allclose(self.k.grasp_point(q), (x, 0.0145, 0.245), atol=0.002)

    def test_documented_reach_at_neck_height(self):
        # hardware assessment: ~26 cm forward of the pan axis straight down, ~39 cm at 30 deg down
        m = Mount()
        z = 0.245
        def reach(pitch):
            best = 0.0
            for f in np.arange(0.10, 0.55, 0.005):
                try:
                    self.k.ik((m.pan_xy[0] + f, m.pan_xy[1], z), math.radians(pitch), check=False)
                    best = f
                except Unreachable:
                    pass
            return best
        self.assertAlmostEqual(reach(-90), 0.26, delta=0.03)
        self.assertAlmostEqual(reach(-30), 0.39, delta=0.04)

    def test_unreachable_is_reported_not_clamped(self):
        with self.assertRaises(Unreachable) as cm:
            self.k.ik((1.0, 0.0, 0.25), math.radians(-90))
        self.assertIn("reach", str(cm.exception))

    def test_joint_limits_are_enforced(self):
        q = {n: 0.0 for n in ARM}
        q["elbow_flex"] = 0.5          # URDF upper limit is 0
        with self.assertRaises(Unreachable):
            self.k.check_limits(q)

    def test_floor_clearance(self):
        with self.assertRaises(Unreachable) as cm:
            self.k.ik((0.36, 0.0, 0.01), math.radians(-90))
        self.assertTrue("floor" in str(cm.exception) or "reach" in str(cm.exception))

    def test_deck_is_not_entered(self):
        # straight down just in front of the arm, low: the wrist would go into the deck
        with self.assertRaises(Unreachable) as cm:
            self.k.ik((0.12, 0.0, 0.28), math.radians(-90))
        self.assertIn("deck", str(cm.exception))

    def test_best_pitch_prefers_steep(self):
        q, pitch = self.k.ik_any_pitch((0.30, 0.0, 0.245))
        self.assertLessEqual(pitch, math.radians(-60))
        np.testing.assert_allclose(self.k.grasp_point(q), (0.30, 0.0, 0.245), atol=0.002)

    def test_calibrated_limits_narrow_the_urdf(self):
        lo, hi = self.k.limits["shoulder_lift"]
        self.assertGreaterEqual(lo, 0.0)
        self.assertLessEqual(hi, 3.5)


import floor_pick  # noqa: E402
import gripper_camera as gc  # noqa: E402
from floor_pick import CLOSED, GRIP_HELD, JAW_OPEN, REST, FloorPick, Planner  # noqa: E402

KIN = Kinematics(Mount())
PLANNER = Planner(KIN)     # the carry search takes a second: share it


class FakeArm:
    """Arm, driver and gripper in memory. `bag` = robot-frame handle point: the jaws stop on it when the
    grasp point closes within 1.5 cm of it."""

    def __init__(self, kin=KIN, bag=None, drop_on_lift=False, busy_after=None, fail_move=None, start=None,
                 gripper=0.0, camera=None):
        self.k = kin
        self.q = dict(start or REST)
        self.q["gripper"] = gripper
        self.bag = None if bag is None else np.asarray(bag, float)
        self.drop_on_lift, self.busy_after, self.fail_move = drop_on_lift, busy_after, fail_move
        self.camera = camera
        self.moves, self.grips, self.events = [], [], []
        self.stops = self.state_calls = 0
        self.others_moving = self.replaced = False
        self.holding = False

    def arm_state(self):
        self.state_calls += 1
        if self.busy_after is not None and self.state_calls > self.busy_after:
            return {"state": "busy", "task": "rest"}
        return {"state": "idle", "task": ""}

    def joints(self):
        return dict(self.q)

    def moving(self):
        return self.others_moving

    def follow(self, points, timeout):
        self.moves.append((points, timeout))
        if self.fail_move is not None and len(self.moves) == self.fail_move:
            if self.replaced:
                return False, "BUSY", "stopped / replaced by another command"
            return False, "TIMEOUT", "timed out"
        self.q.update(points[-1][1])
        if self.holding and self.drop_on_lift:
            self.holding = False
            self.q["gripper"] = 0.0
        return True, "", ""

    def grip(self, position, timeout):
        self.grips.append(position)
        if position < CLOSED:
            near = self.bag is not None and np.linalg.norm(self.k.grasp_point(self.q) - self.bag) < 0.015
            self.holding = bool(near)
            self.q["gripper"] = 0.35 if near else 0.0
            return {"ok": True, "position": self.q["gripper"], "stalled": near, "reached": not near}
        self.holding = False
        self.q["gripper"] = position
        return {"ok": True, "position": position, "stalled": False, "reached": True}

    def stop(self):
        self.stops += 1

    def event(self, text):
        self.events.append(text)

    def snapshot(self):
        return "frame" if self.camera else None

    def sleep(self, s):
        pass


def run(arm, **kw):
    return FloorPick(KIN, arm, PLANNER, log=lambda *_: None, **kw)


class PickSequenceTest(unittest.TestCase):
    def test_pick_succeeds_and_holds_in_the_carry_pose(self):
        arm = FakeArm(bag=(0.32, 0.0, 0.245))
        res = run(arm).pick(0.32, 0.0)
        self.assertTrue(res.ok, res)
        self.assertEqual(arm.events, ["grab_success"])
        self.assertEqual(arm.grips, [JAW_OPEN, 0.0])
        self.assertGreater(arm.q["gripper"], CLOSED)
        carry = PLANNER.carry_plan()
        self.assertLess(max(abs(arm.q[n] - carry["q"][n]) for n in ARM), 1e-9)
        self.assertEqual(arm.stops, 0)

    def test_speeds_stay_under_30_percent_and_every_move_has_a_timeout(self):
        arm = FakeArm(bag=(0.32, 0.0, 0.245))
        run(arm).pick(0.32, 0.0)
        q_prev, worst = dict(REST), 0.0
        for points, timeout in arm.moves:
            t_prev = 0.0
            self.assertGreater(timeout, points[-1][0])
            for t, q in points:
                for n in ARM:
                    worst = max(worst, abs(q[n] - q_prev[n]) / (t - t_prev))
                t_prev, q_prev = t, q
        self.assertLessEqual(worst, 0.3 * 1.0 + 1e-6)     # URDF velocity 1 rad/s, scaled to 30 %

    def test_descent_is_a_straight_line_onto_the_neck(self):
        arm = FakeArm(bag=(0.32, 0.0, 0.245))
        run(arm).pick(0.32, 0.0)
        pts = np.array([KIN.grasp_point(q) for _t, q in arm.moves[1][0]])
        np.testing.assert_allclose(pts[-1], (0.32, 0.0, 0.245), atol=0.002)
        d = pts[-1] - pts[0]
        off = np.linalg.norm(np.cross(pts - pts[0], d / np.linalg.norm(d)), axis=1)
        self.assertLess(off.max(), 0.002)
        self.assertLess(pts[0][2] - pts[-1][2], 0.08 + 1e-6)

    def test_closed_jaws_mean_missed_and_the_arm_backs_up(self):
        arm = FakeArm(bag=None)
        res = run(arm).pick(0.32, 0.0)
        self.assertEqual(res.code, "MISSED")
        self.assertEqual(len(arm.events), 1)
        self.assertTrue(arm.events[0].startswith("grab_failed|MISSED|"))
        self.assertEqual(arm.grips, [JAW_OPEN, 0.0, JAW_OPEN])
        self.assertGreater(KIN.grasp_point(arm.q)[2], 0.245 + 0.03)    # back above the neck

    def test_bag_slipping_out_during_the_lift_is_reported(self):
        arm = FakeArm(bag=(0.32, 0.0, 0.245), drop_on_lift=True)
        res = run(arm).pick(0.32, 0.0)
        self.assertEqual(res.code, "DROPPED")
        self.assertTrue(arm.events[0].startswith("grab_failed|DROPPED|"))
        self.assertEqual(arm.stops, 1)

    def test_unreachable_target_moves_nothing(self):
        arm = FakeArm()
        res = run(arm).pick(0.60, 0.0)
        self.assertEqual(res.code, "UNREACHABLE")
        self.assertEqual(arm.moves, [])
        self.assertTrue(arm.events[0].startswith("grab_failed|UNREACHABLE|"))

    def test_busy_arm_team_node_refuses_the_start(self):
        arm = FakeArm(busy_after=0)
        res = run(arm).pick(0.32, 0.0)
        self.assertEqual(res.code, "BUSY")
        self.assertEqual(arm.moves, [])

    def test_another_program_moving_the_arm_refuses_without_stopping_it(self):
        arm = FakeArm(bag=(0.32, 0.0, 0.245))
        arm.others_moving = True
        res = run(arm).pick(0.32, 0.0)
        self.assertEqual(res.code, "BUSY")
        self.assertEqual(arm.moves, [])
        self.assertEqual(arm.stops, 0)

    def test_a_move_replaced_by_another_program_is_not_stopped(self):
        arm = FakeArm(bag=(0.32, 0.0, 0.245), fail_move=1)
        arm.replaced = True
        res = run(arm).pick(0.32, 0.0)
        self.assertEqual(res.code, "BUSY")
        self.assertEqual(arm.stops, 0)

    def test_becoming_busy_mid_run_stops_the_arm(self):
        arm = FakeArm(bag=(0.32, 0.0, 0.245), busy_after=3)
        res = run(arm).pick(0.32, 0.0)
        self.assertEqual(res.code, "BUSY")
        self.assertEqual(arm.stops, 1)
        self.assertTrue(arm.events[0].startswith("grab_failed|BUSY|"))

    def test_a_failed_move_calls_arm_stop(self):
        arm = FakeArm(bag=(0.32, 0.0, 0.245), fail_move=2)
        res = run(arm).pick(0.32, 0.0)
        self.assertEqual(res.code, "TIMEOUT")
        self.assertEqual(arm.stops, 1)

    def test_hover_only_stops_above_with_open_jaws(self):
        arm = FakeArm(bag=(0.32, 0.0, 0.245))
        res = run(arm).pick(0.32, 0.0, hover_only=True)
        self.assertEqual(res.code, "HOVER")
        self.assertEqual(arm.events, [])
        self.assertEqual(arm.grips, [JAW_OPEN])
        self.assertGreater(KIN.grasp_point(arm.q)[2], 0.245 + 0.05)

    def test_place_lowers_opens_and_backs_off(self):
        carry = PLANNER.carry_plan()
        arm = FakeArm(start=carry["q"], gripper=GRIP_HELD)
        res = run(arm).place(0.32, 0.0)
        self.assertTrue(res.ok, res)
        self.assertEqual(arm.events, ["drop_done"])
        self.assertEqual(arm.grips, [JAW_OPEN, 0.0])
        lowest = min(KIN.grasp_point(q)[2] for pts, _ in arm.moves for _t, q in pts)
        self.assertAlmostEqual(lowest, 0.255, delta=0.002)

    def test_place_without_a_bag(self):
        arm = FakeArm()
        res = run(arm).place(0.32, 0.0)
        self.assertEqual(res.code, "NOT_HOLDING")
        self.assertTrue(arm.events[0].startswith("drop_failed|NOT_HOLDING|"))


class CarryPoseTest(unittest.TestCase):
    def test_default_bag_fits_inside_the_footprint_above_the_lidar(self):
        c = PLANNER.carry_plan()
        self.assertTrue(c["inside_footprint"], c["note"])
        self.assertTrue(c["clears_lidar"])
        self.assertGreaterEqual(c["bag_bottom"], so100_ik.DECK_TOP + 0.02)
        self.assertGreater(c["grip"][2] - floor_pick.BAG_HANG, so100_ik.LIDAR_Z)

    def test_a_deep_bag_cannot_stay_inside_and_says_so(self):
        c = Planner(KIN, bag_half=(0.12, 0.15)).carry_plan()
        self.assertFalse(c["inside_footprint"])
        self.assertIn("IMPOSSIBLE", c["note"])


class DryRunTest(unittest.TestCase):
    def test_dry_run_prints_the_trajectories(self):
        out = io.StringIO()
        with redirect_stdout(out):
            code = floor_pick.main(["--dry-run", "--pick", "0.32", "0.0"])
        text = out.getvalue()
        self.assertEqual(code, 0)
        self.assertIn("shoulder_pan", text)
        self.assertIn("/robot/event: grab_success", text)
        self.assertIn("t=", text)

    def test_dry_run_place(self):
        out = io.StringIO()
        with redirect_stdout(out):
            code = floor_pick.main(["--dry-run", "--place", "0.32", "0.0"])
        self.assertEqual(code, 0, out.getvalue()[-500:])
        self.assertIn("/robot/event: drop_done", out.getvalue())


def true_camera():
    """A camera 'under the jaws': 4 cm down the tool axis, 3.5 cm off it, tilted 20 deg, f = 520 px."""
    T = np.eye(4)
    tilt = gc.rotvec_to_matrix(np.array([math.radians(20), 0, 0]))
    T[:3, :3] = gc.nominal_rotation(math.radians(10)) @ tilt
    T[:3, 3] = (0.004, -0.04, 0.035)
    return gc.CameraModel(520.0, T_gc=T)


class ProjectingDetector:
    """Sees `point` through `camera` from the fake arm's current joints (plus pixel noise)."""

    def __init__(self, arm, camera, point, noise=0.0, seed=1):
        self.arm, self.camera, self.point = arm, camera, np.asarray(point, float)
        self.rng = np.random.default_rng(seed)
        self.noise = noise

    def detect(self, _img):
        uv = self.camera.project(KIN, self.arm.q, self.point)
        if not self.camera.in_view(uv, 1.0):
            return None
        uv = uv + self.rng.normal(0, self.noise, 2)
        return gc.Detection(uv[0], uv[1], how="test")


class CameraGeometryTest(unittest.TestCase):
    def test_pixel_to_plane_inverts_project(self):
        cam = true_camera()
        q = KIN.ik((0.32, 0.0, 0.30), math.radians(-75))
        p = np.array([0.36, 0.02, 0.245])
        uv = cam.project(KIN, q, p)
        np.testing.assert_allclose(cam.pixel_to_plane(KIN, q, uv[0], uv[1], 0.245), p, atol=1e-9)

    def test_fit_recovers_the_camera_from_marker_sightings(self):
        cam = true_camera()
        obs = []
        for m in ((0.36, 0.0, 0.0), (0.38, 0.08, 0.0)):
            for q in PLANNER.calibration_poses(np.array(m)):
                uv = cam.project(KIN, q, m)
                if cam.in_view(uv, 1.0):
                    obs.append((q, np.array(m), uv + np.random.default_rng(len(obs)).normal(0, 0.5, 2)))
        self.assertGreaterEqual(len(obs), 6)
        fit, rms = gc.fit_camera(KIN, obs)
        self.assertLess(rms, 1.5)
        self.assertLess(np.linalg.norm(fit.T_gc[:3, 3] - cam.T_gc[:3, 3]), 0.005)
        ang = np.linalg.norm(gc.matrix_to_rotvec(fit.T_gc[:3, :3].T @ cam.T_gc[:3, :3]))
        self.assertLess(math.degrees(ang), 1.5)

    def test_fit_with_an_unmeasured_floor_cross_recovers_the_camera(self):
        cam = true_camera()
        m = np.array([0.34, -0.03, 0.0])
        obs = []
        for q in PLANNER.self_calibration_poses(m):
            uv = cam.project(KIN, q, m)
            if cam.in_view(uv, 1.0):
                obs.append((q, "a", uv + np.random.default_rng(len(obs)).normal(0, 0.5, 2)))
        self.assertGreaterEqual(len(obs), 6)
        fit, rms, where = gc.fit_camera_free(KIN, obs)
        self.assertLess(rms, 1.5)
        self.assertLess(np.linalg.norm(where["a"] - m[:2]), 0.005)
        self.assertLess(np.linalg.norm(fit.T_gc[:3, 3] - cam.T_gc[:3, 3]), 0.006)
        ang = np.linalg.norm(gc.matrix_to_rotvec(fit.T_gc[:3, :3].T @ cam.T_gc[:3, :3]))
        self.assertLess(math.degrees(ang), 2.0)

    def test_triangulate_finds_a_point_seen_from_two_poses(self):
        cam = true_camera()
        p = np.array([0.33, -0.10, 0.31])
        views = []
        for g in ((0.22, -0.04, 0.42), (0.22, -0.16, 0.42), (0.18, -0.10, 0.44), (0.24, -0.10, 0.40)):
            q = KIN.ik(np.array(g), math.radians(-60))
            views.append((q, cam.project(KIN, q, p) + np.random.default_rng(len(views)).normal(0, 1.0, 2)))
        est, spread = gc.triangulate(KIN, cam, views)
        self.assertLess(np.linalg.norm(est - p), 0.01)
        self.assertLess(spread, 0.01)

    def test_self_calibrate_finds_the_cross_moves_fits_and_saves(self):
        cam = true_camera()
        arm = FakeArm()
        cross = np.array([0.34, -0.03, 0.0])
        det = ProjectingDetector(arm, cam, cross, noise=0.3)
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, "gripper_camera.json")
            fit = run(arm).calibrate_self(det, path, start=cross[:2])
            self.assertIsNotNone(fit)
        self.assertLess(np.linalg.norm(np.array(fit.info["markers"][0][:2]) - cross[:2]), 0.005)

    def test_self_calibrate_from_nothing_tracks_the_cross_first(self):
        cam = true_camera()
        cross = np.array([0.34, -0.03, 0.0])
        q0 = KIN.ik(np.array([0.24, -0.02, 0.40]), math.radians(-50))
        arm = FakeArm(start=q0)
        self.assertIsNotNone(cam.project(KIN, q0, cross))
        det = ProjectingDetector(arm, cam, cross, noise=0.3)
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, "gripper_camera.json")
            fit = run(arm).calibrate_self(det, path)
            self.assertIsNotNone(fit)
            with open(path) as f:
                saved = json.load(f)
            with open(path) as f:
                saved = json.load(f)
        self.assertLess(saved["rms_px"], 1.5)
        self.assertEqual(saved["method"], "floor cross, position fitted")
        self.assertLess(np.linalg.norm(np.array(saved["T_gripper_camera"])[:3, 3] - cam.T_gc[:3, 3]), 0.01)

    def test_calibrate_moves_fits_and_saves(self):
        cam = true_camera()
        arm = FakeArm()
        markers = [(0.36, 0.0, 0.0), (0.38, 0.08, 0.0)]
        state = {"i": 0}
        det = ProjectingDetector(arm, cam, markers[0], noise=0.3)

        def prompt(_text):          # the owner moves the marker
            det.point = np.asarray(markers[state["i"]], float)
            state["i"] += 1
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, "gripper_camera.json")
            fit = run(arm).calibrate(markers, det, prompt, path)
            self.assertIsNotNone(fit)
            with open(path) as f:
                saved = json.load(f)
        self.assertIn("T_gripper_camera", saved)
        self.assertLess(saved["rms_px"], 1.5)
        self.assertLess(np.linalg.norm(np.array(saved["T_gripper_camera"])[:3, 3] - cam.T_gc[:3, 3]), 0.006)
        self.assertEqual(state["i"], 2)

    def test_visual_servo_corrects_a_3_cm_error(self):
        cam = true_camera()
        handle = np.array([0.335, 0.025, 0.245])
        arm = FakeArm(bag=handle, camera=True)
        det = ProjectingDetector(arm, cam, handle, noise=0.3)
        res = run(arm).pick(0.32, 0.0, camera=cam, detector=det)
        self.assertTrue(res.ok, res)
        self.assertEqual(arm.events, ["grab_success"])

    def test_servo_gives_up_when_nothing_is_seen(self):
        arm = FakeArm(bag=(0.32, 0.0, 0.245), camera=True)

        class Blind:
            def detect(self, _img):
                return None
        res = run(arm).pick(0.32, 0.0, camera=true_camera(), detector=Blind())
        self.assertEqual(res.code, "NOT_FOUND")
        self.assertTrue(arm.events[0].startswith("grab_failed|NOT_FOUND|"))


def floor_image():
    img = np.zeros((480, 640, 3), np.uint8)
    img[:] = (170, 160, 60)                    # teal floor (BGR), like the owner's frame
    img[392:] = (150, 170, 180)                # beige jaw at the bottom
    return img


@unittest.skipUnless(HAVE_CV2, "OpenCV is not installed")
class DetectorTest(unittest.TestCase):
    def test_pink_bag_is_found(self):
        img = floor_image()
        img[150:260, 250:400] = (150, 60, 230)      # pink (BGR)
        d = gc.BagHandleDetector().detect(img)
        self.assertEqual(d.how, "colour")
        self.assertAlmostEqual(d.u, 324.5, delta=2)
        self.assertAlmostEqual(d.v, 204.5, delta=2)

    def test_pink_under_the_jaw_is_ignored(self):
        img = floor_image()
        img[420:470, 250:400] = (150, 60, 230)
        self.assertIsNone(gc.ColourBlobDetector().detect(img))

    def test_fallback_finds_a_compact_blob_near_the_centre(self):
        img = floor_image()
        img[170:250, 290:360] = (40, 40, 40)        # dark bag, no pink
        img[20:40, 20:40] = (40, 40, 40)            # small far blob
        d = gc.BagHandleDetector().detect(img)
        self.assertEqual(d.how, "compact")
        self.assertAlmostEqual(d.u, 324.5, delta=3)

    def test_empty_floor_gives_nothing(self):
        self.assertIsNone(gc.BagHandleDetector().detect(floor_image()))

    def test_grout_crossing(self):
        import cv2
        img = floor_image()
        cv2.line(img, (335, 0), (330, 392), (40, 40, 30), 2)
        cv2.line(img, (0, 90), (640, 95), (40, 40, 30), 2)
        d = gc.GroutCrossDetector().detect(img)
        self.assertIsNotNone(d)
        self.assertAlmostEqual(d.u, 334, delta=4)
        self.assertAlmostEqual(d.v, 92, delta=4)

    def test_plugin_detector_is_loaded_by_name(self):
        det = gc.load_detector("gripper_camera:ColourBlobDetector")
        self.assertIsInstance(det, gc.ColourBlobDetector)


if __name__ == "__main__":
    unittest.main()
