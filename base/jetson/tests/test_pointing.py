"""Pointing at a bag: synthetic keypoints made by projecting a 3D stick person through the camera model."""

import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import pointing  # noqa: E402

P = {"camera_x": 0.205, "camera_y": 0.045, "camera_height": 0.07, "camera_pitch_deg": 12.3, "camera_hfov_deg": 58.59}
W, H = 320, 240


def unit(v):
    n = math.sqrt(sum(c * c for c in v))
    return tuple(c / n for c in v)


def body(person, target=None, arm="right", p=P, vis=0.9, mirrored=True):
    """17 COCO keypoints [u, v, vis] of a person at `person` (x, y) facing the robot.

    `arm` is the operator's own arm. It points straight at the floor spot `target` (x, y), or hangs
    down when target is None. In a mirrored image the pose model sees a mirrored person, so it names
    the operator's right arm "left" (COCO 5, 7, 9) and the left arm "right" (6, 8, 10).
    """
    px, py = person
    f = unit((-px, -py))  # facing the robot
    left = (-f[1], f[0])  # the operator's own left
    pts = {}

    def at(side, along, h, fwd=0.0):
        s = 1 if side == "left" else -1
        return (px + s * along * left[0] + fwd * f[0], py + s * along * left[1] + fwd * f[1], h)

    for side in ("left", "right"):
        shoulder = at(side, 0.20, 1.40)
        if side == arm and target is not None:
            d = unit((target[0] - shoulder[0], target[1] - shoulder[1], 0.0 - shoulder[2]))
            elbow = tuple(s + 0.30 * c for s, c in zip(shoulder, d))
            wrist = tuple(s + 0.58 * c for s, c in zip(shoulder, d))
        else:
            elbow = at(side, 0.22, 1.10)
            wrist = at(side, 0.23, 0.82)
        pts[side] = [shoulder, elbow, wrist, at(side, 0.12, 0.95), at(side, 0.11, 0.50), at(side, 0.11, 0.08)]
    head = [at("left", 0.0, 1.60, 0.10), at("left", 0.03, 1.64, 0.08), at("right", 0.03, 1.64, 0.08),
            at("left", 0.07, 1.62), at("right", 0.07, 1.62)]
    model_left, model_right = ("right", "left") if mirrored else ("left", "right")
    order = head[:]  # nose, eyes (l, r), ears (l, r): the head is symmetric enough here
    for i in range(6):  # shoulders, elbows, wrists, hips, knees, ankles
        order += [pts[model_left][i], pts[model_right][i]]
    return [[*pointing.project(q, p, W, H, mirrored=mirrored), vis] for q in order]


def polar(xy):
    return math.hypot(*xy), math.degrees(math.atan2(xy[1], xy[0]))


def dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


class CameraModelTests(unittest.TestCase):
    def test_project_then_ray_comes_back_to_the_point(self):
        for q in [(2.0, 0.3, 1.2), (1.0, -0.4, 0.0), (3.0, 1.0, 0.5)]:
            u, v = pointing.project(q, P, W, H)
            origin, d = pointing.ray(u, v, P, W, H)
            t = (q[0] - origin[0]) / d[0]
            for a, b in zip(q, [o + t * c for o, c in zip(origin, d)]):
                self.assertAlmostEqual(a, b, places=6)

    def test_image_right_is_robot_left_when_mirrored(self):
        u_left, _ = pointing.project((2.0, 0.5, 0.5), P, W, H)
        self.assertGreater(u_left, W / 2)
        u_plain, _ = pointing.project((2.0, 0.5, 0.5), P, W, H, mirrored=False)
        self.assertLess(u_plain, W / 2)

    def test_shoulders_of_a_standing_person_need_about_2_m(self):
        # Top image row at 1.5 m is about 1.1 m above the floor: 1.4 m shoulders are out of view.
        self.assertLess(pointing.top_visible_height(1.5, P, H), 1.2)
        self.assertGreater(pointing.top_visible_height(2.5, P, H), 1.6)


class PointingTests(unittest.TestCase):
    def check(self, person, target, arm, tol=0.25):
        r, b = polar(person)
        found = pointing.pointing(body(person, target, arm), r, b, P, W, H)
        self.assertIsNotNone(found)
        self.assertLess(dist(found.point, target), tol, found.point)
        self.assertEqual(found.arm, arm)
        self.assertGreater(found.conf, 0.3)
        return found

    def test_standing_1_5_m_away_pointing_1_m_to_the_side(self):
        # The geometry alone (at 1.5 m the shoulders are above the top row, see the camera test).
        self.check((1.5, 0.0), (1.5, 1.0), "right")  # facing the robot, the right arm is on the robot's left
        self.check((1.5, 0.0), (1.5, -1.0), "left")

    def test_2_5_m_away_off_centre_with_every_keypoint_in_view(self):
        person = (2.4, 0.6)
        f = unit((-person[0], -person[1]))
        target = (person[0] - f[1], person[1] + f[0])  # 1 m to the operator's own left
        kp = body(person, target, "left")
        self.assertTrue(all(0 <= u < W and 0 <= v < H for u, v, _ in kp[5:13]), kp)  # shoulders to hips
        self.check(person, target, "left")

    def test_pointing_toward_the_robot_is_the_known_weak_case(self):
        # All body points are put at the operator's range, so an arm aimed at the camera loses its depth.
        person, target = (2.5, 0.0), (1.7, 0.8)
        r, b = polar(person)
        found = pointing.pointing(body(person, target, "right"), r, b, P, W, H)
        self.assertGreater(dist(found.point, target), 0.25)

    def test_two_pixel_noise_keeps_the_point_within_0_25_m(self):
        person, target = (2.5, 0.0), (2.5, 1.0)
        r, b = polar(person)
        kp = body(person, target, "right")
        for i, (du, dv) in zip((5, 7, 9), [(2, -2), (-2, 2), (2, 2)]):
            kp[i] = [kp[i][0] + du, kp[i][1] + dv, kp[i][2]]
        found = pointing.pointing(kp, r, b, P, W, H)
        self.assertLess(dist(found.point, target), 0.25, found.point)

    def test_arms_hanging_is_not_pointing(self):
        r, b = polar((2.5, 0.0))
        self.assertIsNone(pointing.pointing(body((2.5, 0.0)), r, b, P, W, H))

    def test_hand_on_the_chest_is_not_pointing(self):
        person = (2.5, 0.0)
        kp = body(person)
        sh_l, sh_r = kp[5], kp[6]
        kp[7] = [sh_l[0] + 0.6 * (sh_l[0] - sh_r[0]), sh_l[1] + 20, 0.9]  # elbow out to the side
        kp[9] = [(sh_l[0] + sh_r[0]) / 2, sh_l[1] + 15, 0.9]  # wrist back at the middle of the chest
        r, b = polar(person)
        self.assertIsNone(pointing.pointing(kp, r, b, P, W, H))

    def test_hidden_wrist_is_not_pointing(self):
        person, target = (2.5, 0.0), (2.5, 1.0)
        kp = body(person, target, "right")
        kp[9][2] = 0.3
        r, b = polar(person)
        self.assertIsNone(pointing.pointing(kp, r, b, P, W, H))

    def test_arm_raised_above_the_shoulder_never_reaches_the_floor(self):
        person = (2.5, 0.0)
        r, b = polar(person)
        kp = body(person, (2.5, 1.0), "right")
        sh = kp[5]
        kp[7] = [sh[0] + 25, sh[1] - 12, 0.9]
        kp[9] = [sh[0] + 50, sh[1] - 25, 0.9]
        self.assertIsNone(pointing.pointing(kp, r, b, P, W, H))

    def test_mirrored_image_labels_and_sides(self):
        person, target = (2.5, 0.0), (2.5, 1.0)  # a spot on the robot's left
        kp = body(person, target, "right")
        self.assertGreater(kp[9][0], kp[5][0])  # the wrist is to the image right of the shoulder
        r, b = polar(person)
        found = pointing.pointing(kp, r, b, P, W, H)
        self.assertGreater(found.point[1], 0.5)  # robot left
        self.assertEqual(found.arm, "right")  # model label "left" (9) is the operator's right arm
        plain = body(person, target, "right", mirrored=False)
        found = pointing.pointing(plain, r, b, P, W, H, mirrored=False)
        self.assertLess(dist(found.point, target), 0.25)
        self.assertEqual(found.arm, "right")


class SteadyTests(unittest.TestCase):
    def found(self, xy):
        return pointing.Pointing(point=xy, conf=0.9, arm="right", shoulder=(2.5, -0.2, 1.4), operator=(2.5, 0.0))

    def test_reports_the_median_only_after_one_steady_second(self):
        steady = pointing.Steady()
        out = None
        for i in range(12):
            jitter = 0.05 if i % 2 else -0.05
            out = steady.add(i * 0.1, self.found((2.5 + jitter, 1.0)))
            if i < 9:
                self.assertIsNone(out, i)
        self.assertIsNotNone(out)
        self.assertAlmostEqual(out.point[1], 1.0)
        self.assertLess(abs(out.point[0] - 2.5), 0.06)

    def test_a_moving_point_is_not_reported(self):
        steady = pointing.Steady()
        outs = [steady.add(i * 0.1, self.found((2.5, 0.5 + 0.05 * i))) for i in range(15)]
        self.assertTrue(all(o is None for o in outs))

    def test_a_missed_frame_does_not_reset_and_old_points_expire(self):
        steady = pointing.Steady()
        for i in range(12):
            out = steady.add(i * 0.1, None if i == 5 else self.found((2.5, 1.0)))
        self.assertIsNotNone(out)
        self.assertIsNone(steady.add(5.0, self.found((2.5, 1.0))))  # everything before 4 s is gone


class ChooseBagTests(unittest.TestCase):
    A, B = (2.0, 0.5), (2.0, -0.3)  # 0.8 m apart; the operator faces the robot, so A is on their right

    def aim(self, xy, shoulder=(2.8, 0.1, 1.4), operator=(2.8, 0.1)):
        return pointing.Pointing(point=xy, conf=0.9, arm="right", shoulder=shoulder, operator=operator)

    def test_points_at_each_bag(self):
        got = pointing.choose_bag(self.aim((2.05, 0.45)), [self.A, self.B])
        self.assertEqual((got.index, got.side), (0, "right"))
        self.assertIn("right", got.say)
        got = pointing.choose_bag(self.aim((1.9, -0.2)), [self.A, self.B])
        self.assertEqual((got.index, got.side), (1, "left"))

    def test_nothing_within_0_6_m(self):
        self.assertIsNone(pointing.choose_bag(self.aim((3.0, 1.5)), [self.A, self.B]))

    def test_between_the_bags_the_ray_decides(self):
        # The point lands short, nearer to B, but the shoulder -> point ray goes on through A.
        here = (2.25, 0.05)
        self.assertLess(dist(here, self.B), dist(here, self.A))
        shoulder = (2.75, -0.85, 1.4)
        got = pointing.choose_bag(self.aim(here, shoulder=shoulder, operator=shoulder[:2]), [self.A, self.B])
        self.assertEqual(got.index, 0)

    def test_one_bag_is_just_the_bag(self):
        got = pointing.choose_bag(self.aim((2.0, 0.45)), [self.A])
        self.assertEqual((got.index, got.side), (0, None))

    def test_end_to_end_from_keypoints(self):
        # Bags beside the operator (on their right, 0.8 m apart), as the plane assumption wants.
        operator, near, far = (2.8, 0.1), (2.8, 1.1), (2.8, 1.9)
        r, b = polar(operator)
        for target, index, side in [(near, 0, "left"), (far, 1, "right")]:
            steady, out = pointing.Steady(), None
            for i in range(12):
                out = steady.add(i * 0.1, pointing.pointing(body(operator, target, "right"), r, b, P, W, H))
            got = pointing.choose_bag(out, [near, far])
            self.assertEqual((got.index, got.side), (index, side))


class CandidateTests(unittest.TestCase):
    def box_at(self, xy, size=0.3):
        """Box of a bag standing on the floor at xy, 0.3 m wide and high."""
        corners = [pointing.project((xy[0], xy[1] + dy, h), P, W, H) for dy in (-size / 2, size / 2) for h in (0.0, size)]
        us, vs = [c[0] for c in corners], [c[1] for c in corners]
        return [min(us), min(vs), max(us), max(vs)]

    def test_range_from_the_lidar_cluster_on_the_box_bearing(self):
        boxes = [self.box_at((2.0, 0.5)), self.box_at((2.0, -0.3))]
        clusters = [(2.05, -0.3), (2.05, 0.5), (1.0, -1.5)]
        got = pointing.bag_candidates(boxes, P, W, H, clusters)
        self.assertEqual(got, [(2.05, 0.5), (2.05, -0.3)])

    def test_without_lidar_the_box_bottom_on_the_floor(self):
        got = pointing.bag_candidates([self.box_at((2.0, 0.5))], P, W, H)
        self.assertLess(dist(got[0], (2.0, 0.5)), 0.15)  # the near bottom edge, so a little short


if __name__ == "__main__":
    unittest.main()
