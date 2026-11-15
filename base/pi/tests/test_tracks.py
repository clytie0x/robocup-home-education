import math
import random
import unittest

from tracks import KEEP, Tracks


def run(frames, dt=0.1):
    """frames: a list of point lists, one per scan. Returns the {id: point} of every scan."""
    tracks, out = Tracks(), []
    for i, points in enumerate(frames):
        out.append(tracks.update(round(dt * i, 3), points))
    return tracks, out


class Ids(unittest.TestCase):
    def test_a_walker_keeps_one_id(self):
        _, out = run([[(1.0 + 0.1 * i, 0.5)] for i in range(30)])  # 1 m/s
        self.assertEqual({tid for ids in out for tid in ids}, {1})

    def test_a_flicker_of_a_few_scans_keeps_the_id(self):
        for missed in (1, 3, 5):
            with self.subTest(missed=missed):
                frames = [[(2.0, -0.3)]] * 10 + [[]] * missed + [[(2.0, -0.3)]] * 5
                _, out = run(frames)
                self.assertEqual({tid for ids in out for tid in ids}, {1})

    def test_a_longer_gap_is_a_new_id(self):
        missed = round(KEEP / 0.1) + 2
        _, out = run([[(2.0, -0.3)]] * 10 + [[]] * missed + [[(2.0, -0.3)]] * 5)
        self.assertEqual({tid for ids in out for tid in ids}, {1, 2})

    def test_a_walker_who_drops_out_comes_back_ahead_under_the_same_id(self):
        # walking 1 m/s, unseen 4 scans: 0.5 m from the last fix, outside the plain 0.4 m gate of one scan
        frames = [[(1.0 + 0.1 * i, 0.0)] for i in range(10)] + [[]] * 4 + [[(1.0 + 0.1 * i, 0.0)] for i in range(14, 20)]
        _, out = run(frames)
        self.assertEqual({tid for ids in out for tid in ids}, {1})

    def test_two_people_passing_half_a_metre_apart_keep_their_ids(self):
        frames = [[(0.5 + 0.1 * i, 0.0), (3.0 - 0.1 * i, 0.35)] for i in range(26)]
        _, out = run(frames)
        for ids in out:
            by_y = {round(pt[1], 2): tid for tid, pt in ids.items()}
            self.assertEqual(by_y, {0.0: 1, 0.35: 2})

    def test_ids_come_back_in_the_order_of_the_points(self):
        tracks = Tracks()
        tracks.update(0.0, [(1.0, 0.0), (2.0, 0.0)])
        ids = tracks.update(0.1, [(2.0, 0.0), (1.0, 0.0)])
        self.assertEqual(list(ids.values()), [(2.0, 0.0), (1.0, 0.0)])
        self.assertEqual(list(ids), [2, 1])


class Motion(unittest.TestCase):
    def test_a_still_thing_stays_still_and_a_walker_moves(self):
        rnd = random.Random(1)
        frames = [[(2.0 + rnd.gauss(0, 0.02), -0.3 + rnd.gauss(0, 0.02)), (1.0 + 0.1 * i, 1.0)] for i in range(30)]
        tracks, out = run(frames)
        still, walker = tracks[1], tracks[2]
        self.assertLess(still.speed, 0.2)
        self.assertLess(still.far, 0.10)
        self.assertAlmostEqual(walker.speed, 1.0, delta=0.1)
        self.assertGreater(walker.far, 2.5)

    def test_a_run_restarts_after_a_miss_but_the_birth_stays(self):
        tracks = Tracks()
        for i in range(5):
            tracks.update(0.1 * i, [(2.0, 0.0)])
        tracks.update(0.5, [])
        tracks.update(0.6, [(2.0, 0.0)])
        self.assertEqual((tracks[1].born, tracks[1].run_since), (0.0, 0.6))

    def test_an_object_takes_only_a_point_right_where_it_stands(self):
        tracks = Tracks()
        tracks.update(0.0, [(2.0, 0.0)])
        tracks[1].object = True
        self.assertEqual(list(tracks.update(0.1, [(2.3, 0.0)])), [2])  # someone else, 0.3 m away
        self.assertEqual(list(tracks.update(0.2, [(2.05, 0.0)])), [1])


if __name__ == "__main__":
    unittest.main()
