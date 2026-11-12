import math
import unittest

import params
from link import Answer, Seen
from legs import candidates
from senses import RealSenses
from test_legs import person, scan_of

P = params.load()
OPERATOR = (1.0, 0.0)
OTHER = (0.0, 1.0)
STILL = (0.0, 0.0, 0.0)  # pose: the robot at the odometry origin, facing +x


def score(look):
    return 1.0 if look == OPERATOR else 0.0


def camera_bearing(x, y):
    """Bearing of a lidar-frame point as the camera sees it (the camera is not at the laser)."""
    bx, by = x + P["lidar_x"], y
    return math.degrees(math.atan2(by - P["camera_y"], bx - P["camera_x"]))


def at_bearing(deg, metres):
    """The lidar-frame point at this camera bearing and distance from the camera."""
    rad = math.radians(deg)
    return (P["camera_x"] + metres * math.cos(rad) - P["lidar_x"], P["camera_y"] + metres * math.sin(rad))


def seen(x, y, look):
    return Seen(camera_bearing(x, y), 0.9, look, [0, 0, 1, 1])


def read(s, now, scan, answer=None, pose=STILL, link_alive=True, depth=None):
    return s.read(now, pose, 0.0, 0.0, 0.1, scan, depth, answer, link_alive, score)


def ready():
    """Senses after the brain has learned the operator's 20 looks: the camera may re-lock."""
    s = RealSenses(P)
    s.learned = 20
    return s


def enrolled(x=1.5, y=0.0):
    s = RealSenses(P)
    got = s.enrol(0.0, scan_of(person(x, y)), Answer(0.0, [seen(x, y, OPERATOR)]))
    assert not isinstance(got, str), got
    return s


class NamedTests(unittest.TestCase):
    def test_holding_close_with_no_camera_look_stays_locked(self):
        s = enrolled(0.8, 0.0)
        for i in range(300):  # 30 s at 10 Hz, the operator stands 0.8 m in front, no answers
            got = read(s, 0.1 * (i + 1), scan_of(person(0.8, 0.0)), Answer(0.0, []))
            self.assertTrue(s.locked)
            self.assertIsNotNone(got.operator)
            self.assertIsNone(got.operator_look)

    def test_stale_answer_is_ignored(self):
        scan = scan_of(person(1.5, 0.0))
        s = ready()
        got = read(s, 10.0, scan, Answer(9.0, [seen(1.5, 0.0, OPERATOR)]))  # 1 s late
        self.assertIsNone(got.operator)
        self.assertFalse(s.locked)
        got = read(s, 10.1, scan, Answer(9.7, [seen(1.5, 0.0, OPERATOR)]))  # 0.4 s: used
        self.assertIsNotNone(got.operator)

    def test_no_new_lock_while_the_jetson_is_silent(self):
        s = ready()
        got = read(s, 0.0, scan_of(person(1.5, 0.0)), Answer(0.0, [seen(1.5, 0.0, OPERATOR)]), link_alive=False)
        self.assertIsNone(got.operator)
        self.assertFalse(s.locked)

    def test_relock_needs_a_matching_look(self):
        scan = scan_of(person(1.5, 0.0))
        s = ready()
        got = read(s, 0.0, scan, Answer(0.0, [seen(1.5, 0.0, OTHER)]))
        self.assertIsNone(got.operator)
        self.assertFalse(s.locked)
        got = read(s, 0.1, scan, Answer(0.1, [seen(1.5, 0.0, OPERATOR)]))
        self.assertTrue(s.locked)
        self.assertAlmostEqual(got.operator[0], 1.5 + P["lidar_x"], delta=0.1)
        self.assertAlmostEqual(got.operator[1], 0.0, delta=0.1)
        self.assertEqual(got.operator_look, OPERATOR)

    def test_enrol_refuses_two_people_inside_2_5_m(self):
        s = RealSenses(P)
        scan = scan_of(person(1.5, 0.6) + person(1.5, -0.6))
        answer = Answer(0.0, [seen(1.5, 0.6, OPERATOR), seen(1.5, -0.6, OTHER)])
        self.assertEqual(s.enrol(0.0, scan, answer), "more than one person within 2.5 m")
        self.assertFalse(s.locked)
        self.assertEqual(s.enrol(0.0, scan_of(), Answer(0.0, [])), "nobody in view")

    def test_enrol_refuses_a_stale_answer(self):
        s = RealSenses(P)
        scan = scan_of(person(1.5, 0.0))
        self.assertEqual(s.enrol(1.0, scan, Answer(0.0, [seen(1.5, 0.0, OPERATOR)])), "camera answer too old")
        self.assertFalse(s.locked)
        self.assertNotIsInstance(s.enrol(1.0, scan, Answer(0.6, [seen(1.5, 0.0, OPERATOR)])), str)

    def test_no_camera_lock_before_enrolment_has_learned_20_looks(self):
        scan = scan_of(person(1.5, 0.0))
        s = RealSenses(P)
        s.learned = 19
        self.assertIsNone(read(s, 0.0, scan, Answer(0.0, [seen(1.5, 0.0, OPERATOR)])).operator)
        self.assertFalse(s.locked)
        s.learned = 20
        self.assertIsNotNone(read(s, 0.1, scan, Answer(0.1, [seen(1.5, 0.0, OPERATOR)])).operator)

    def test_bearing_is_compared_from_the_camera_position(self):
        scan = scan_of(person(0.3, 0.0))
        (c,) = candidates(scan)
        bearing = camera_bearing(*c)  # straight ahead of the laser, but 11 deg right of the camera
        self.assertGreater(abs(bearing - math.degrees(math.atan2(c[1], c[0]))), 8.0)
        s = RealSenses(P)
        point, look = s.enrol(0.0, scan, Answer(0.0, [Seen(bearing, 0.9, OPERATOR, [0, 0, 1, 1])]))
        self.assertAlmostEqual(s._camera_bearing(c), bearing, delta=0.2)
        self.assertEqual(look, OPERATOR)


class Rules(unittest.TestCase):
    def test_rule1_bearings_are_corrected_by_the_turn_since_capture(self):
        s = ready()
        read(s, 0.0, scan_of(), pose=(0.0, 0.0, 0.0))  # capture time: facing +x
        real = at_bearing(-10.0, 2.0)  # where the operator is now, after a 20 degree left turn
        decoy = at_bearing(10.0, 2.0)  # where the uncorrected bearing points now
        answer = Answer(0.0, [Seen(10.0, 0.9, OPERATOR, [0, 0, 1, 1])])
        read(s, 0.3, scan_of(person(*real) + person(*decoy)), answer, pose=(0.0, 0.0, math.radians(20)))
        self.assertTrue(s.locked)
        self.assertLess(s.tracker.pos[1], 0.0)  # the real one, on the right

    def test_rule2_several_within_8_degrees_takes_the_one_nearest_the_track_range(self):
        near, far = at_bearing(-6.0, 1.0), at_bearing(2.0, 2.5)
        s = RealSenses(P)
        s.enrol(0.0, scan_of([near]), Answer(0.0, [seen(*near, OPERATOR)]))
        self.assertTrue(s.locked)
        # The camera says -1.5 deg: nearer in bearing to `far` (3.5) than to the track (4.5).
        got = read(s, 0.1, scan_of([near, far]), Answer(0.1, [Seen(-1.5, 0.9, OPERATOR, [0, 0, 1, 1])]))
        self.assertEqual(got.operator_look, OPERATOR)
        self.assertLess(math.hypot(*got.operator), 1.5)

    def test_rule3_operator_is_the_track_in_the_odometry_frame(self):
        s = enrolled()
        pose = (1.0, 2.0, math.radians(90))
        got = read(s, 0.1, scan_of(person(1.5, 0.0)), pose=pose)
        self.assertAlmostEqual(got.operator[0], 1.0, delta=0.1)
        self.assertAlmostEqual(got.operator[1], 2.0 + 1.5 + P["lidar_x"], delta=0.1)
        self.assertIsNone(got.operator_look)
        got = read(s, 0.2, scan_of(person(1.5, 0.0) + person(1.5, -1.2)), Answer(0.2, [seen(1.5, -1.2, OPERATOR)]), pose=pose)
        self.assertIsNone(got.operator_look)  # the camera person is not the track
        got = read(s, 0.3, scan_of(person(1.5, 0.0)), Answer(0.3, [seen(1.5, 0.0, OPERATOR)]), pose=pose)
        self.assertEqual(got.operator_look, OPERATOR)

    def test_rule4_lock_is_reported_and_silence_never_locks(self):
        scan = scan_of(person(1.5, 0.0))
        s = ready()
        self.assertIsNone(read(s, 0.0, scan).operator)  # no camera, no lock
        self.assertIsNone(read(s, 0.1, scan, Answer(0.1, [seen(1.5, 0.0, OPERATOR)]), link_alive=False).operator)
        self.assertIsNotNone(read(s, 0.2, scan, Answer(0.2, [seen(1.5, 0.0, OPERATOR)])).operator)

    def test_rule5_people_keep_their_ids_and_obstacles_leave_people_out(self):
        s = enrolled()
        first = read(s, 0.1, scan_of(person(1.5, 0.0) + person(1.0, 1.5), wall=2.5))
        self.assertEqual(len(first.people), 1)
        (pid, where), = first.people.items()
        second = read(s, 0.2, scan_of(person(1.5, 0.0) + person(1.1, 1.5) + person(-1.0, -1.0), wall=2.5))
        self.assertEqual(len(second.people), 2)
        self.assertAlmostEqual(second.people[pid][0], where[0] + 0.1, delta=0.05)
        for everyone in [second.operator] + list(second.people.values()):
            self.assertFalse(any(math.dist((c.x, c.y), everyone) < 0.3 for c in second.obstacles))
        self.assertGreater(len(second.obstacles), 10)  # the wall is still there


class Ambiguity(unittest.TestCase):
    def test_a_second_person_at_the_track_latches_ambiguous_for_1_s(self):
        s = enrolled()
        got = read(s, 0.1, scan_of(person(1.5, 0.0) + person(1.5, -0.35)))
        self.assertTrue(s.ambiguous)
        self.assertAlmostEqual(got.operator[1], 0.0, delta=0.05)  # still the track
        for i in range(2, 12):  # clear from 0.2 s on: still ambiguous until 1 s after 0.1 s
            got = read(s, round(0.1 * i, 1), scan_of(person(1.5, 0.0) + person(1.5, -0.4)))
            self.assertTrue(s.ambiguous)
            self.assertIsNotNone(got.operator)
        read(s, 1.2, scan_of(person(1.5, 0.0) + person(1.5, -0.4)))
        self.assertFalse(s.ambiguous)

    def test_no_operator_look_while_ambiguous(self):
        s = enrolled()
        both = scan_of(person(1.5, 0.0) + person(1.5, -0.35))
        got = read(s, 0.1, both, Answer(0.1, [seen(1.46, 0.0, OPERATOR)]))  # the camera points at the track
        self.assertTrue(s.ambiguous)
        self.assertIsNone(got.operator_look)
        alone = scan_of(person(1.5, 0.0))
        got = read(s, 0.2, alone, Answer(0.2, [seen(1.46, 0.0, OPERATOR)]))
        self.assertIsNone(got.operator_look)  # latched
        for i in range(3, 12):
            read(s, round(0.1 * i, 1), alone)
        got = read(s, 1.2, alone, Answer(1.2, [seen(1.46, 0.0, OPERATOR)]))
        self.assertEqual(got.operator_look, OPERATOR)  # clear for 1 s: the look counts

    def test_two_camera_people_at_the_track_bearing_are_ambiguous(self):
        s = enrolled()
        answer = Answer(0.1, [seen(1.46, 0.0, OPERATOR), seen(3.0, 0.25, OTHER)])  # a stranger behind
        for legs in (person(1.5, 0.0) + person(3.0, 0.25), person(1.5, 0.0)):
            got = read(s, 0.1, scan_of(legs), answer)
            self.assertTrue(s.ambiguous)
            self.assertIsNone(got.operator_look)

    def test_no_camera_lock_while_ambiguous(self):
        s = ready()
        got = read(s, 0.0, scan_of(person(1.5, 0.0) + person(1.5, -0.35)), Answer(0.0, [seen(1.46, 0.0, OPERATOR)]))
        self.assertTrue(s.ambiguous)
        self.assertFalse(s.locked)
        self.assertIsNone(got.operator)


class Gaps(unittest.TestCase):
    def test_one_and_three_scan_gaps_keep_the_lock(self):
        for gap in (1, 3):
            with self.subTest(gap=gap):
                s = enrolled()
                t = 0.0
                for _ in range(gap):
                    t = round(t + 0.1, 1)
                    got = read(s, t, scan_of())  # legs scissor out of sight
                    self.assertTrue(s.locked)
                    self.assertIsNone(got.operator)  # no fresh fix, no operator
                got = read(s, round(t + 0.1, 1), scan_of(person(1.5, 0.0)))
                self.assertTrue(s.locked)
                self.assertAlmostEqual(got.operator[1], 0.0, delta=0.05)

    def test_coasting_ends_after_track_coast_max(self):
        s = enrolled()
        for i in range(1, 21):
            read(s, round(0.1 * i, 1), scan_of())
        self.assertTrue(s.locked)  # 2.0 s
        read(s, 2.1, scan_of())
        self.assertFalse(s.locked)

    def test_hidden_operator_never_ends_on_the_stranger(self):
        # Review probe C: the operator is hidden 1 s, a stranger stands 0.8 m to the right.
        s = enrolled()
        frames = [[(1.5, 0.0), (1.5, -0.8)]] * 3 + [[(1.5, -0.8)]] * 10 + [[(1.5, 0.0), (1.5, -0.8)]] * 3
        for i, people in enumerate(frames):
            got = read(s, round(0.1 * (i + 1), 1), scan_of(sum((person(*p) for p in people), [])))
            if got.operator is not None:
                self.assertGreater(got.operator[1], -0.4)  # never the stranger
            if s.locked:
                self.assertGreater(s.tracker.pos[1], -0.4)

    def test_a_jump_drops_the_track(self):
        s = enrolled()
        got = read(s, 0.1, scan_of(person(1.5, -0.45)))  # the operator vanished, someone 0.45 m away
        self.assertFalse(s.locked)
        self.assertIsNone(got.operator)

    def test_a_contradicting_look_after_a_gap_drops_the_track(self):
        s = enrolled()
        read(s, 0.1, scan_of())  # gap
        stranger = (1.5, -0.25)  # nearest to the prediction, inside track_jump
        got = read(s, 0.2, scan_of(person(*stranger)), Answer(0.2, [seen(*stranger, OTHER)]))
        self.assertFalse(s.locked)
        self.assertIsNone(got.operator)
        s = enrolled()
        read(s, 0.1, scan_of())
        got = read(s, 0.2, scan_of(person(*stranger)), Answer(0.2, [seen(*stranger, OPERATOR)]))
        self.assertTrue(s.locked)  # the look agrees: keep it


if __name__ == "__main__":
    unittest.main()
