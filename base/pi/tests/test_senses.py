import math
import unittest
from types import SimpleNamespace

import numpy as np

import params
from link import Answer, Seen
from legs import candidates
from senses import RealSenses, person_range
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


def read(s, now, scan, answer=None, pose=STILL, link_alive=True, depth=None, frame=None):
    return s.read(now, pose, 0.0, 0.0, 0.1, scan, depth, answer, link_alive, score, frame)


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


W, H = 320, 240  # the Xtion frames the Jetson answers for
FOCAL = (W / 2) / math.tan(math.radians(P["camera_hfov_deg"]) / 2)
PITCH = math.radians(P["camera_pitch_deg"])


def camera_range(x, y):
    return math.hypot(x + P["lidar_x"] - P["camera_x"], y - P["camera_y"])


def pixel(ahead, left, up):
    """Image pixel of a point given from the camera (metres ahead, left, up). Mirrored: robot left is image right."""
    z = ahead * math.cos(PITCH) + up * math.sin(PITCH)
    down = ahead * math.sin(PITCH) - up * math.cos(PITCH)
    return (W - 1) / 2 + FOCAL * left / z, (H - 1) / 2 + FOCAL * down / z


def view(t, people, width=0.4, tall=1.7):
    """A camera answer and its frame. Each person, (lidar-frame point, look), is an upright board width x tall
    facing the camera at their LiDAR range; a nearer board hides a farther one in the depth image."""
    depth = np.zeros((H, W), np.uint16)
    found, boards = [], []
    for (x, y), look in people:
        ahead, left = x + P["lidar_x"] - P["camera_x"], y - P["camera_y"]
        x1, _ = pixel(ahead, left - width / 2, 0.0)
        x2, _ = pixel(ahead, left + width / 2, 0.0)
        _, top = pixel(ahead, left, tall - P["camera_height"])
        _, bottom = pixel(ahead, left, -P["camera_height"])
        box = [max(0.0, x1), max(0.0, top), min(float(W), x2), min(float(H), bottom)]
        found.append(Seen(camera_bearing(x, y), 0.9, look, box))
        boards.append((ahead, box))
    for ahead, box in sorted(boards, key=lambda b: -b[0]):  # the far ones first: the near ones paint over them
        for v in range(int(box[1]), int(math.ceil(box[3]))):
            down = (v - (H - 1) / 2) / FOCAL
            depth[v, int(box[0]):int(math.ceil(box[2]))] = round(1000 * ahead / (math.cos(PITCH) + down * math.sin(PITCH)))
    return Answer(t, found), SimpleNamespace(time=t, depth=depth)


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
        first = read(s, 0.1, scan_of(person(1.5, 0.0) + person(1.0, 1.5), wall=2.5),
                     Answer(0.1, [seen(1.0, 1.5, OTHER)]))  # seen by the camera: a person, not furniture
        self.assertEqual(len(first.people), 1)
        (pid, where), = first.people.items()
        second = read(s, 0.2, scan_of(person(1.5, 0.0) + person(1.1, 1.5) + person(-1.0, -1.0), wall=2.5))
        self.assertEqual(len(second.people), 1)  # the one at (-1, -1) never moved and no camera saw them: furniture
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


class Merge(unittest.TestCase):
    """Spec criterion 4: a second person stands next to the operator, then they walk apart.
    The LiDAR may see the two as one object meanwhile; at the split the track would take
    whichever piece is nearer the blob's centre, a matter of centimetres."""

    STRANGER = (1.5, -0.45)  # stands 0.45 m to the operator's right

    def frames(self, split):
        approach = [[(1.5, 0.0), (1.5, -0.8 + 0.035 * i)] for i in range(10)] + [[(1.5, 0.0), self.STRANGER]]
        merged = [[(1.5, -0.225)]] * 20  # 2 s seen as one object between them
        return approach + merged + split

    def splits(self):
        return {
            "operator walks on, stranger stays": [[(1.5, 0.05 * i), self.STRANGER] for i in range(1, 21)],
            "stranger walks off, operator stays": [[(1.5, 0.0), (1.5, -0.45 - 0.05 * i)] for i in range(1, 21)],
            "both walk apart": [[(1.5, 0.03 * i), (1.5, -0.45 - 0.03 * i)] for i in range(1, 21)],
        }

    def check_never_on_the_stranger(self, s, frames, answer_for):
        for i, people in enumerate(frames):
            t = round(0.1 * (i + 1), 1)
            got = read(s, t, scan_of(sum((person(*p) for p in people), [])), answer_for(t, people),
                       link_alive=answer_for(t, people) is not None)
            stranger_y = people[1][1] if len(people) > 1 else None
            if stranger_y is not None and s.locked and got.operator is not None and not s.ambiguous:  # driving
                self.assertLess(abs(s.tracker.pos[1] - people[0][1]), abs(s.tracker.pos[1] - stranger_y),
                                "following the stranger at %.1f s" % t)

    def test_link_down_follows_the_operator_or_stops(self):
        for name, split in self.splits().items():
            with self.subTest(name):
                self.check_never_on_the_stranger(enrolled(), self.frames(split), lambda t, people: None)

    def test_the_camera_locks_the_operator_again_after_the_split(self):
        # The camera sees both people where they really are, also while the LiDAR sees one object.
        for gap in (0.45, 0.30):
            with self.subTest(gap=gap):
                s = enrolled()
                s.learned = 20
                op, st = (1.5, 0.0), (1.5, -gap)
                phases = [([op, (1.5, -0.8)], [op, (1.5, -0.8)])] * 3 + [([op, st], [op, st])] * 5
                phases += [([(1.5, -gap / 2)], [op, st])] * 20  # merged in the LiDAR, two in the camera
                phases += [([(1.5, 0.05 * i), st], [(1.5, 0.05 * i), st]) for i in range(1, 31)]  # operator walks on
                for i, (lidar_people, real) in enumerate(phases):
                    t = round(0.1 * (i + 1), 1)
                    cam = Answer(t, [seen(*real[0], OPERATOR), seen(*real[1], OTHER)])
                    got = read(s, t, scan_of(sum((person(*p) for p in lidar_people), [])), cam)
                    if got.operator is not None and len(lidar_people) > 1 and not s.ambiguous:  # driving, not holding
                        self.assertLess(abs(s.tracker.pos[1] - real[0][1]), abs(s.tracker.pos[1] - real[1][1]),
                                        "following the stranger at %.1f s" % t)
                self.assertTrue(s.locked)
                self.assertAlmostEqual(s.tracker.pos[1], phases[-1][1][0][1], delta=0.1)  # the operator

    def test_a_stranger_who_walks_up_fast_is_not_forgotten(self):
        # Review finding: counted from entering the 0.6 m zone, someone arriving at 0.75 m/s merged
        # before the count was full, and the robot rode the stranger. Now the id's age counts.
        for step in (0.075,):
            with self.subTest(speed=step * 10):
                ys, y = [], -1.2
                while y < -0.45 - 1e-9:  # walks in at constant speed, the last step onto -0.45
                    ys.append(y)
                    y += step
                ys.append(-0.45)
                approach = [[(1.5, 0.0), (1.5, y)] for y in ys]
                split = self.splits()["operator walks on, stranger stays"]
                frames = approach + [[(1.5, -0.225)]] * 20 + split
                self.check_never_on_the_stranger(enrolled(), frames, lambda t, people: None)

    def test_a_stranger_missed_for_one_scan_just_before_the_merge_still_counts(self):
        # Scratch fuzz 2026-11-15, scene 115: the stranger walked up and stood beside the operator, missed one
        # scan, the camera locked the operator again, and they merged 0.2 s later. Counted from the missed
        # scan, the stranger was too young to count, and the robot rode them after the split.
        s = ready()
        approach = [[(1.5, 0.0), (1.5, -1.2 + 0.05 * i)] for i in range(16)]  # walks in from 1.2 m away
        beside = [[(1.5, 0.0), self.STRANGER]] * 5 + [[(1.5, 0.0)]] + [[(1.5, 0.0), self.STRANGER]] * 2
        frames = approach + beside + [[(1.5, -0.225)]] * 20 + self.splits()["operator walks on, stranger stays"]
        relock = round(0.1 * (len(approach) + 7), 1)  # the cycle after the missed scan; the camera looks only then
        self.check_never_on_the_stranger(
            s, frames, lambda t, people: Answer(t, [seen(*people[0], OPERATOR)]) if t == relock else None)

    def test_enrol_waits_while_someone_is_right_at_the_operator(self):
        s = enrolled()
        read(s, 0.1, scan_of(person(1.5, 0.0) + person(1.5, -0.35)))  # crowded: ambiguous latches
        s.tracker = s._new_tracker()  # lost meanwhile
        self.assertEqual(s.enrol(0.2, scan_of(person(1.5, -0.35)), Answer(0.2, [seen(1.5, -0.35, OTHER)])),
                         "someone is right at the operator: wait")

    def test_someone_walking_past_does_not_drop_the_track(self):
        s = enrolled()
        frames = [[(1.5, 0.0), (1.5 - 1.0 + 0.1 * i, -0.5)] for i in range(21)]  # passes 0.5 m to the side
        for i, people in enumerate(frames):
            read(s, round(0.1 * (i + 1), 1), scan_of(sum((person(*p) for p in people), [])), link_alive=False)
            self.assertTrue(s.locked, i)

    def test_link_up_with_the_stranger_out_of_camera_view(self):
        # The camera sees only the operator (the stranger is outside its 58 degrees).
        for name, split in self.splits().items():
            with self.subTest(name):
                self.check_never_on_the_stranger(
                    enrolled(), self.frames(split),
                    lambda t, people: Answer(t, [seen(*people[0], OPERATOR)]) if len(people) > 1 else Answer(t, []))


class MergedRelock(unittest.TestCase):
    """Scratch fuzz 2026-11-15 (senses only, a model, not real data), camera on, the stranger not in its
    answer: the track was dropped when the stranger merged with the operator, and once the latch ran out
    the camera re-locked onto the merged object (the operator's bearing was within 8 deg of it). At the
    split the track took the stranger's piece: 42 of 200 scenes drove on the stranger."""

    OP, STRANGER = (1.5, 0.0), (1.5, -0.4)
    BLOB = (1.5, -0.1)  # the merged object nearer the operator: the stranger is 11 deg from it, outside MATCH_DEG

    def run_merge(self, answer_for, merged=30, hidden=0):
        """The stranger walks up and stands beside the operator, the operator's legs drop out of `hidden` scans,
        they merge into one LiDAR object for `merged` scans, then the operator walks on. Yields (time, what
        the LiDAR sees, senses, output)."""
        s = enrolled(*self.OP)
        s.learned = 20
        frames = [[self.OP, (1.5, -1.2 + 0.05 * i)] for i in range(16)] + [[self.OP, self.STRANGER]] * 5
        frames += [[self.STRANGER]] * hidden + [[self.BLOB]] * merged + [[(1.5, 0.05 * i), self.STRANGER] for i in range(1, 31)]
        for i, lidar in enumerate(frames):
            t = round(0.1 * (i + 1), 1)
            real = [lidar[0], self.STRANGER] if len(lidar) > 1 else [self.OP, self.STRANGER]
            answer, frame = answer_for(t, real)
            got = read(s, t, scan_of(sum((person(*p) for p in lidar), [])), answer, frame=frame)
            yield t, lidar, s, got

    def check(self, answer_for, hidden=0):
        for t, lidar, s, got in self.run_merge(answer_for, hidden=hidden):
            if len(lidar) == 1:
                self.assertFalse(s.locked, "re-locked onto the merged object at %.1f s" % t)
            elif s.locked and got.operator is not None and not s.ambiguous:  # driving
                self.assertLess(abs(s.tracker.pos[1] - lidar[0][1]), abs(s.tracker.pos[1] - lidar[1][1]),
                                "following the stranger at %.1f s" % t)
        self.assertTrue(s.locked)  # the operator walked away from the stranger: the camera locks them again
        self.assertAlmostEqual(s.tracker.pos[1], lidar[0][1], delta=0.1)

    def test_no_re_lock_onto_the_merged_object_by_bearing_alone(self):
        self.check(lambda t, real: (Answer(t, [seen(*real[0], OPERATOR)]), None))  # no depth: no range

    def test_no_re_lock_while_the_depth_sees_the_stranger_inside_the_object(self):
        self.check(lambda t, real: view(t, [(real[0], OPERATOR), (real[1], OTHER)]))

    def test_a_fix_rejected_next_to_the_stranger_counts_too(self):
        # Fuzz scene 4: the operator's legs drop out with the stranger 0.4 m away, beyond track_jump: the fix
        # is refused and the track dropped (no company rule), then the two merge.
        self.check(lambda t, real: (Answer(t, [seen(*real[0], OPERATOR)]), None), hidden=1)

    def test_a_re_enrol_waits_too(self):
        # While the brain still learns (fewer than 20 looks) the loop enrols again whenever unlocked.
        s = enrolled(*self.OP)
        frames = [[self.OP, (1.5, -1.2 + 0.05 * i)] for i in range(16)] + [[self.OP, self.STRANGER]] * 5
        frames += [[self.BLOB]] * 30
        said = set()
        for i, lidar in enumerate(frames):
            t = round(0.1 * (i + 1), 1)
            scan, answer = scan_of(sum((person(*p) for p in lidar), [])), Answer(t, [seen(*self.OP, OPERATOR)])
            if not s.locked:
                said.add(s.enrol(t, scan, answer, lambda look: score(look) >= 0.65))
            read(s, t, scan, answer)
            if len(lidar) == 1:
                self.assertFalse(s.locked, "enrolled onto the merged object at %.1f s" % t)
        self.assertIn("two people may be one object there: wait", said)

    def test_the_depth_seeing_only_the_operator_there_re_locks(self):
        # The stranger stepped behind the operator (gone from the scan) and walked off unseen: the LiDAR
        # sees one object where the two merged, the camera's depth one person at its range, the operator.
        s = enrolled(*self.OP)
        s.learned = 20
        frames = [[self.OP, (1.5, -1.2 + 0.05 * i)] for i in range(16)] + [[self.OP, self.STRANGER]] * 5
        frames += [[self.OP]] * 30
        for i, lidar in enumerate(frames):
            t = round(0.1 * (i + 1), 1)
            answer, frame = view(t, [(self.OP, OPERATOR)])
            read(s, t, scan_of(sum((person(*p) for p in lidar), [])), answer, frame=frame)
        self.assertTrue(s.locked)
        self.assertAlmostEqual(s.tracker.pos[1], 0.0, delta=0.1)

    def test_without_the_depth_one_person_there_is_not_enough(self):
        s = enrolled(*self.OP)
        s.learned = 20
        frames = [[self.OP, (1.5, -1.2 + 0.05 * i)] for i in range(16)] + [[self.OP, self.STRANGER]] * 5
        frames += [[self.OP]] * 30
        whys = set()
        for i, lidar in enumerate(frames):
            t = round(0.1 * (i + 1), 1)
            read(s, t, scan_of(sum((person(*p) for p in lidar), [])), Answer(t, [seen(*self.OP, OPERATOR)]))
            whys.add(s.why)
            if i >= 21:
                self.assertFalse(s.locked, t)  # the stranger may still be in that object: hold
        self.assertTrue(s.ambiguous)
        self.assertIn("no re-lock: merged", whys)


class CloseStranger(unittest.TestCase):
    """Scratch fuzz 2026-11-15, link down: 5 of 200 scenes drove on a stranger first seen 0.64 to 0.99 m
    from the operator. Closer than WALKED_IN, their vanishing counted only from their last missed scan."""

    STRANGER = (1.5, -0.45)

    def check_never_on_the_stranger(self, frames):
        s = enrolled()
        for i, people in enumerate(frames):
            t = round(0.1 * (i + 1), 1)
            got = read(s, t, scan_of(sum((person(*p) for p in people), [])), link_alive=False)
            if len(people) > 1 and s.locked and got.operator is not None and not s.ambiguous:  # driving
                self.assertLess(abs(s.tracker.pos[1] - people[0][1]), abs(s.tracker.pos[1] - people[1][1]),
                                "following the stranger at %.1f s" % t)

    def test_a_stranger_from_0_85_m_missed_once_on_the_way_in_still_counts(self):
        # Missed at 0.7 m (outside COMPANY), then merged 0.4 s later: too young counted from the missed scan.
        ys = [round(-0.85 + 0.05 * i, 2) for i in range(9)]  # -0.85 to -0.45
        approach = [[(1.5, 0.0)] + ([] if y == -0.7 else [(1.5, y)]) for y in ys]
        split = [[(1.5, 0.05 * i), self.STRANGER] for i in range(1, 21)]  # the operator walks on
        self.check_never_on_the_stranger(approach + [[(1.5, -0.225)]] * 20 + split)

    def test_a_fast_stranger_the_track_slides_onto_is_not_followed(self):
        # Fuzz scene 25: first seen 0.65 m away, at 1 m/s; right next to the operator (crowded) the operator's
        # legs drop out of one scan and the track takes the stranger, 0.33 m away, inside track_jump. The
        # stranger walks off, the operator stays and flickers (no unbroken run for the split rule).
        strangers = [-0.65, -0.55, -0.45, -0.33, -0.33, -0.33] + [-0.33 - 0.05 * i for i in range(1, 30)]
        s = enrolled()
        for i, y in enumerate(strangers):
            t = round(0.1 * (i + 1), 1)
            operator_seen = i != 5 and (i < 6 or (i - 6) % 3)
            lidar = ([(1.5, 0.0)] if operator_seen else []) + [(1.5, y)]
            got = read(s, t, scan_of(sum((person(*p) for p in lidar), [])), link_alive=False)
            if s.locked and got.operator is not None and not s.ambiguous:  # driving
                self.assertLess(abs(s.tracker.pos[1]), abs(s.tracker.pos[1] - y), "following the stranger at %.1f s" % t)


class Furniture(unittest.TestCase):
    """A chair leg is a LiDAR candidate that never moves: it must not count as a person (Task 6)."""

    CHAIR = (2.4, -0.3)  # one leg, 0.3 m beside the operator's path

    def walk_past(self, s, extra=lambda i: [], answer=lambda t, op: None):
        """The operator walks from 1.5 m to 2.8 m along y = 0 at 0.5 m/s, past the chair (room radius 3 m)."""
        frames = [(1.5 + 0.05 * i, 0.0) for i in range(27)]
        for i, op in enumerate(frames):
            t = round(0.1 * (i + 1), 1)
            got = read(s, t, scan_of(person(*op) + [self.CHAIR] + extra(i)), answer(t, op),
                       link_alive=answer(t, op) is not None)
            yield t, op, got

    def test_walking_past_a_chair_leg_keeps_the_track(self):
        s = enrolled()
        for t, op, got in self.walk_past(s):
            self.assertTrue(s.locked, t)
            self.assertFalse(s.ambiguous, (t, s.why))
            self.assertAlmostEqual(got.operator[0], op[0] + P["lidar_x"], delta=0.1)
        self.assertNotIn(True, [math.dist(pt, (self.CHAIR[0] + P["lidar_x"], self.CHAIR[1])) < 0.1
                                for pt in got.people.values()])  # an obstacle, not a person

    def test_a_still_stranger_seen_by_the_camera_is_a_person(self):
        s = enrolled()
        cam = lambda t, op: Answer(t, [seen(*op, OPERATOR), seen(*self.CHAIR, OTHER)])
        ambiguous = [s.ambiguous for _, _, _ in self.walk_past(s, answer=cam)]
        self.assertIn(True, ambiguous)  # the camera says someone stands there: crowded

    def test_a_still_stranger_never_takes_the_track_when_the_operator_is_hidden(self):
        # Review 2026-11-14: a stranger standing still from the first moment (furniture by the rule)
        # beside where the operator stops. The operator's legs drop out for k scans while the stranger
        # walks off, then the operator is seen again. Link down.
        for side in (0.25, 0.34):
            for k in (3, 6, 8, 12):
                with self.subTest(side=side, hidden=k):
                    s = enrolled(1.5, 0.0)
                    st = lambda n: (2.4, -side - 0.05 * n)
                    frames = [([(1.5 + 0.05 * i, 0.0), st(0)], st(0)) for i in range(19)]  # walks up to (2.4, 0)
                    frames += [([(2.4, 0.0), st(0)], st(0))] * 10  # stands beside the stranger
                    frames += [([st(n)], st(n)) for n in range(k)]  # hidden; the stranger walks off
                    frames += [([(2.4, 0.0), st(n)], st(n)) for n in range(k, k + 15)]  # seen again
                    for i, (people, stranger) in enumerate(frames):
                        t = round(0.1 * (i + 1), 1)
                        got = read(s, t, scan_of(sum((person(*p) for p in people), [])), link_alive=False)
                        if s.locked and got.operator is not None and not s.ambiguous:  # driving
                            here = s.tracker.pos
                            self.assertGreater(math.dist(here, stranger), math.dist(here, (2.4, 0.0)) - 1e-9,
                                               "following the stranger at %.1f s" % t)

    def test_a_stranger_who_stops_next_to_a_chair_still_counts(self):
        # Review Focus 3: the robot has seen the chair for a while; a stranger walks up and stops
        # right beside it, then the operator walks past them both. Link down.
        s = enrolled()
        for i in range(30):  # 3 s: operator still, chair seen all along
            read(s, round(0.1 * (i + 1), 1), scan_of(person(1.5, 0.0) + [self.CHAIR]), link_alive=False)
        stranger = lambda i: person(2.15, -0.3 - 0.05 * max(0, 10 - i))  # walks in 0.5 m, stops 0.25 m from the chair
        s2 = s
        ambiguous = []
        for i in range(37):  # the operator ends at 2.75 m, inside the 3 m room
            t = round(3.1 + 0.1 * i, 1)
            op = (1.5 + 0.05 * (i - 12), 0.0) if i >= 12 else (1.5, 0.0)  # starts walking once the stranger stands
            read(s2, t, scan_of(person(*op) + [self.CHAIR] + stranger(i)), link_alive=False)
            ambiguous.append(s2.ambiguous or not s2.locked)
        self.assertIn(True, ambiguous)  # held or dropped next to the stranger, never drove on blind


class SetDown(unittest.TestCase):
    """Run 20261115-032329 at 172.9 s: the operator stood beside the gift bag they had put down, 0.31 m
    away. The bag flickered in the scan; each new id was 'first seen within COMPANY of the operator',
    so a person for good, and the robot held 'crowded' for 13.7 s while the camera saw one person."""

    OP, BAG = (2.0, 0.0), (2.0, -0.31)

    def run_set_down(self, link, frames=60, flicker=3):
        s = enrolled(1.5, 0.0)
        held = []
        for i in range(frames):
            t = round(0.1 * (i + 1), 1)
            op = (1.5 + 0.05 * i, 0.0) if i < 10 else self.OP  # walks up with the bag, stops, puts it down
            things = person(*op) + ([self.BAG] if i >= 10 and i % flicker else [])
            answer = Answer(t, [seen(*op, OPERATOR)]) if link else None  # the camera sees one person
            read(s, t, scan_of(things), answer, link_alive=link)
            held.append(s.ambiguous or not s.locked)
        return s, held

    def test_a_still_bag_the_camera_does_not_see_as_a_person_stops_crowding(self):
        s, held = self.run_set_down(link=True)
        self.assertTrue(s.locked)
        self.assertEqual(held[-15:], [False] * 15, "still holding 3.5 s after the bag was put down")

    # A bag that stays in every scan from the moment it is put down sets off the split rule (_mixed_up,
    # 0.6 s) before the SET_DOWN_S evidence is in: that only holds while the camera is looking, it no
    # longer drops the track.
    def test_a_bag_that_never_flickers_too(self):
        s, held = self.run_set_down(link=True, flicker=1000)
        self.assertEqual(held[-15:], [False] * 15)

    def test_with_the_link_down_it_stays_careful(self):
        _, held = self.run_set_down(link=False)
        self.assertIn(True, held[10:])  # no camera to say it is not a person: the old rule holds

    def test_a_person_who_stepped_out_and_stands_still_keeps_crowding(self):
        # the camera does see a second person there: never treated as a bag
        s = enrolled(1.5, 0.0)
        held = []
        for i in range(60):
            t = round(0.1 * (i + 1), 1)
            op = (1.5 + 0.05 * i, 0.0) if i < 10 else self.OP
            st = (2.0, -0.31)
            things = person(*op) + (person(*st) if i >= 10 else [])
            answer = Answer(t, [seen(*op, OPERATOR)] + ([seen(*st, OTHER)] if i >= 10 else []))
            read(s, t, scan_of(things), answer)
            held.append(s.ambiguous or not s.locked)
        self.assertEqual(held[-10:], [True] * 10)


class Range(unittest.TestCase):
    """The range of a camera person from the depth image of their frame (senses.person_range)."""

    def test_the_depth_in_the_box_gives_the_range_from_the_camera(self):
        for metres in (1.0, 1.5, 2.5, 3.5):
            with self.subTest(metres=metres):
                at = (metres, 0.3)
                answer, frame = view(0.0, [(at, OTHER)])
                (who,) = answer.people
                self.assertGreater(who.box[0], W / 2 - 10)  # on the robot's left: the image's right
                self.assertAlmostEqual(person_range(who.box, frame.depth, P), camera_range(*at), delta=0.05)

    def test_no_depth_no_range(self):
        answer, frame = view(0.0, [((2.0, 0.0), OTHER)])
        self.assertIsNone(person_range(answer.people[0].box, np.zeros((H, W), np.uint16), P))
        self.assertIsNone(person_range([0, 0, 1, 1], frame.depth, P))  # a box too small to read

    def test_someone_hidden_behind_reads_the_range_of_who_hides_them(self):
        front, back = (1.5, 0.0), (3.0, 0.1)
        answer, frame = view(0.0, [(front, OPERATOR), (back, OTHER)])
        ranges = [person_range(who.box, frame.depth, P) for who in answer.people]
        self.assertAlmostEqual(ranges[1], camera_range(*front), delta=0.05)  # never farther apart than they look


class TwoAtTheTrack(unittest.TestCase):
    """Run 20261115-032329: 44 cycles 'two seen at the track', two camera people within MATCH_DEG of the
    track bearing, e.g. a second person behind the operator at a different range."""

    OP = (2.2, 0.0)

    def check(self, stranger, with_frame):
        s = enrolled(*self.OP)
        scan = scan_of(person(*self.OP) + person(*stranger), wall=4.5)
        answer, frame = view(0.1, [(self.OP, OPERATOR), (stranger, OTHER)])
        self.assertLessEqual(abs(camera_bearing(*self.OP) - camera_bearing(*stranger)), 8.0)  # within MATCH_DEG
        got = read(s, 0.1, scan, answer, frame=frame if with_frame else None)
        return s, got

    def test_someone_clearly_behind_the_operator_is_not_ambiguous(self):
        s, got = self.check((3.6, 0.45), with_frame=True)
        self.assertFalse(s.ambiguous, s.why)
        self.assertEqual(got.operator_look, OPERATOR)  # the look whose range is the track's
        self.assertEqual(len(got.people), 1)  # the camera saw a person behind: a person, not furniture

    def test_without_the_depth_it_stays_ambiguous(self):
        s, got = self.check((3.6, 0.45), with_frame=False)
        self.assertTrue(s.ambiguous)
        self.assertEqual(s.why, "two seen at the track")
        self.assertIsNone(got.operator_look)

    def test_someone_in_front_of_the_operator_keeps_it_ambiguous(self):
        s, got = self.check((1.2, 0.05), with_frame=True)  # nearer: they hide the operator from the depth
        self.assertTrue(s.ambiguous)
        self.assertIsNone(got.operator_look)

    def test_the_depth_never_re_locks_onto_a_candidate_at_another_range(self):
        # The camera sees the operator 2.8 m away; the LiDAR has only a chair leg at that bearing, 1.5 m away.
        chair, operator = (1.5, 0.0), (2.8, 0.0)
        answer, frame = view(0.0, [(operator, OPERATOR)])
        s = ready()
        read(s, 0.0, scan_of([chair]), answer, frame=frame)
        self.assertFalse(s.locked)
        s = ready()
        read(s, 0.0, scan_of([chair]), answer)  # without the depth, the bearing alone locks onto the chair
        self.assertTrue(s.locked)


class Ids(unittest.TestCase):
    def test_a_person_who_drops_out_of_a_few_scans_keeps_their_id(self):
        s = enrolled()
        ids = set()
        for i in range(25):  # walks along y = 1.5; the LiDAR misses them on 3 scans
            t = round(0.1 * (i + 1), 1)
            walker = (0.5 + 0.06 * i, 1.5)
            scan = scan_of(person(1.5, 0.0) + ([] if 10 <= i < 13 else person(*walker)), wall=2.5)
            got = read(s, t, scan, link_alive=False)
            ids |= set(got.people)
        self.assertEqual(len(ids), 1)


class Objects(unittest.TestCase):
    """Task 3 of plan Milestone 3: a candidate the camera never confirms as a person and that stays still is an
    object, whether or not it flickers or appeared next to the operator."""

    CHAIR = [(2.3, -0.5), (2.3, -0.7)]  # two legs, one candidate at (2.3, -0.6)
    START, BESIDE = (1.5, 0.3), (2.1, -0.1)  # the chair is 0.54 m from the operator beside it

    def beside_the_chair(self, link_for, drop_out=(40,)):
        """The operator stands 2.5 s at START (the camera has the chair in clear view), walks beside it in
        1 s and stands there; the chair drops out of the scans in drop_out. Returns the senses and, per
        cycle, whether the robot holds (ambiguous or unlocked)."""
        s = enrolled(*self.START)
        held = []
        for i in range(55):
            t = round(0.1 * (i + 1), 1)
            k = min(max(i - 25, 0), 10) / 10
            op = (self.START[0] + k * (self.BESIDE[0] - self.START[0]), self.START[1] + k * (self.BESIDE[1] - self.START[1]))
            things = person(*op) + ([] if i in drop_out else self.CHAIR)
            link = link_for(i)
            read(s, t, scan_of(things), Answer(t, [seen(*op, OPERATOR)]) if link else None, link_alive=link)
            held.append(s.ambiguous or not s.locked)
        return s, held

    def test_a_watched_chair_that_drops_out_beside_the_operator_keeps_the_track(self):
        s, held = self.beside_the_chair(lambda i: True)
        self.assertTrue(s.locked)
        self.assertEqual(held[36:], [False] * 19, s.why)

    def test_with_the_link_down_it_drops_the_track(self):
        _, held = self.beside_the_chair(lambda i: i < 38)  # an object while the camera answered; then silence
        self.assertIn(True, held[40:])
        _, held = self.beside_the_chair(lambda i: False)
        self.assertIn(True, held[40:])

    def test_an_object_never_takes_the_track(self):
        # The bag (to the operator's left, its own LiDAR candidate) is an object; then the operator's legs
        # drop out of the scan. The coasting track reaches the bag, inside track_jump: the fix is refused.
        s = enrolled(1.5, 0.0)
        op, bag = (2.0, 0.0), (2.0, 0.33)
        for i in range(70):
            t = round(0.1 * (i + 1), 1)
            here = (1.5 + 0.05 * i, 0.0) if i < 10 else op
            hidden = i >= 55
            things = ([] if hidden else person(*here)) + ([bag] if i >= 10 else [])
            got = read(s, t, scan_of(things), Answer(t, [seen(*here, OPERATOR)]))
            if i == 54:
                self.assertFalse(s.ambiguous or not s.locked, s.why)  # the bag stopped crowding
            if s.locked:
                self.assertGreater(math.dist(s.tracker.pos, bag), 0.1, "on the bag at %.1f s" % t)
            if got.operator is not None:
                self.assertGreater(math.dist(got.operator, (bag[0] + P["lidar_x"], bag[1])), 0.15, t)

    def test_clear_view_is_where_a_person_standing_there_would_be_seen(self):
        s = RealSenses(P)
        s._camera_fresh = True
        operator = at_bearing(-8.6, 1.14)  # run 20261115-032329 at 180 s: the operator, and the gift bag
        bag = at_bearing(3.0, 1.33)  # 0.31 m from them, 0.19 m farther from the camera
        self.assertTrue(s._in_clear_view(bag, [operator, bag]))
        behind = at_bearing(-5.6, 1.29)  # 0.15 m behind the operator, 3 deg off: hidden by them
        self.assertFalse(s._in_clear_view(behind, [operator, behind]))
        beside = at_bearing(10.0, 1.2)  # 0.4 m to their left
        self.assertTrue(s._in_clear_view(beside, [operator, beside]))
        self.assertFalse(s._in_clear_view(at_bearing(0.0, 1.0), []))  # too near: only the legs below 0.77 m are in the picture
        self.assertFalse(s._in_clear_view(at_bearing(25.0, 2.0), []))  # half of them out of the picture
        s._camera_fresh = False
        self.assertFalse(s._in_clear_view(bag, [operator, bag]))  # no camera answer: nobody watches

    def test_a_still_stranger_the_camera_sees_stays_a_person(self):
        s = enrolled(*self.START)
        stranger = (2.3, -0.6)
        for i in range(40):
            t = round(0.1 * (i + 1), 1)
            got = read(s, t, scan_of(person(*self.START) + person(*stranger)),
                       Answer(t, [seen(*self.START, OPERATOR), seen(*stranger, OTHER)]))
        self.assertEqual(len(got.people), 1)
        self.assertFalse(any(track.object for track in s.tracks.tracks.values()))


class Looks(unittest.TestCase):
    LEFT, RIGHT = (1.5, 0.8), (1.5, -0.8)
    A, B = (0.1, 0.2), (0.3, 0.4)

    def two(self, s):
        scan = scan_of(person(*self.LEFT) + person(*self.RIGHT))
        return scan, read(s, 0.0, scan, Answer(0.0, [seen(*self.LEFT, self.A), seen(*self.RIGHT, self.B)]))

    def test_two_people_in_view_give_two_looks_under_their_ids(self):
        _, got = self.two(RealSenses(P))
        self.assertEqual(len(got.people), 2)
        self.assertEqual(set(got.looks), set(got.people))
        for pid, look in got.looks.items():
            self.assertEqual(look, self.A if got.people[pid][1] > 0 else self.B)

    def test_a_person_seen_only_by_the_lidar_has_no_look(self):
        s = RealSenses(P)
        scan, _ = self.two(s)
        got = read(s, 0.1, scan, Answer(0.1, [seen(*self.LEFT, self.A)]))
        self.assertEqual(len(got.people), 2)  # the right one still counts: the camera saw them 0.1 s ago
        (pid,) = got.looks
        self.assertGreater(got.people[pid][1], 0)
        self.assertEqual(got.looks[pid], self.A)

    def test_a_stale_answer_gives_no_looks(self):
        s = RealSenses(P)
        scan, _ = self.two(s)
        both = [seen(*self.LEFT, self.A), seen(*self.RIGHT, self.B)]
        got = read(s, 1.0, scan, Answer(0.0, both))  # 1 s late
        self.assertEqual(len(got.people), 2)
        self.assertEqual(got.looks, {})
        got = read(s, 1.1, scan, Answer(1.1, both), link_alive=False)
        self.assertEqual(got.looks, {})

    def test_looks_leave_the_operator_choice_unchanged(self):
        s = enrolled()  # the operator at (1.5, 0)
        scan = scan_of(person(1.5, 0.0) + person(*self.LEFT))
        got = read(s, 0.1, scan, Answer(0.1, [seen(1.5, 0.0, OPERATOR), seen(*self.LEFT, self.A)]))
        self.assertTrue(s.locked)
        self.assertAlmostEqual(got.operator[1], 0.0, delta=0.1)
        self.assertEqual(got.operator_look, OPERATOR)
        self.assertEqual(set(got.looks), set(got.people))
        self.assertEqual(list(got.looks.values()), [self.A])  # the operator's own look is not among them

    def test_two_camera_people_on_one_track_give_no_look(self):
        s = RealSenses(P)
        scan = scan_of(person(*self.LEFT) + person(*self.RIGHT))
        near_left = (self.LEFT[0] + 0.1, self.LEFT[1] + 0.05)  # within the bearing rule of the left track
        got = read(s, 0.0, scan, Answer(0.0, [seen(*self.LEFT, self.A), seen(*near_left, self.B), seen(*self.RIGHT, self.B)]))
        self.assertEqual(len(got.looks), 1)  # the right one only: whose look the left track has, nobody knows
        (pid,) = got.looks
        self.assertLess(got.people[pid][1], 0)

    def test_a_look_is_not_handed_to_whoever_stands_at_the_operator_range(self):
        # Fable review: locked on the operator at 2.3 m; a person at 2.3 m and another at 2.8 m, both
        # within 8 deg of the camera bearing of the far one. The camera sees only the far one.
        s = enrolled(2.2, -0.6)
        front, back = (2.3, -0.05), (2.8, 0.25)  # camera bearings about -2.6 and +4.5 deg
        scan = scan_of(person(2.2, -0.6) + person(*front) + person(*back))
        got = read(s, 0.1, scan, Answer(0.1, [seen(2.2, -0.6, OPERATOR), seen(*back, self.B)]))
        self.assertTrue(s.locked)
        self.assertEqual(got.looks, {})  # two candidates in the bearing window: nobody gets B

    def test_the_operators_own_look_never_goes_to_another_id(self):
        s = enrolled()  # at (1.5, 0)
        for i in range(5):  # the operator jumps 0.9 m sideways: the track coasts, the operator is a "person"
            got = read(s, round(0.1 * (i + 1), 1), scan_of(person(1.5, 0.9)), Answer(round(0.1 * (i + 1), 1), [seen(1.5, 0.9, OPERATOR)]))
            self.assertNotIn(OPERATOR, got.looks.values())


if __name__ == "__main__":
    unittest.main()
