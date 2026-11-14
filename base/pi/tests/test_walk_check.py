import math
import sys
import unittest
from pathlib import Path

import params
from test_legs import person, scan_of

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "tools"))
import walk_check  # noqa: E402

P = params.load()


def loop(seconds=20.0, radius=1.2, speed=0.6):
    """A person walking circles round the standing robot (lidar frame), one scan every 0.1 s."""
    out = []
    for i in range(int(seconds / 0.1)):
        a = speed * 0.1 * i / radius
        out.append((0.1 * i, scan_of(person(radius * math.cos(a), radius * math.sin(a)))))
    return out


class WalkCheck(unittest.TestCase):
    def test_a_lone_walker_is_held(self):
        records, summary = walk_check.walk(loop(), P)
        self.assertEqual(summary["drops"], 0)
        self.assertGreaterEqual(summary["held_share"], 0.95)

    def test_nobody_in_front_never_starts(self):
        records, summary = walk_check.walk([(0.1 * i, scan_of()) for i in range(10)], P)
        self.assertEqual(summary["scans"], 0)

    def test_a_drop_is_counted_and_locked_again(self):
        stand = (1.0, 0.0)
        scans = [(0.1 * i, scan_of(person(*stand))) for i in range(10)]
        scans += [(0.1 * i, scan_of()) for i in range(10, 35)]  # gone 2.5 s, past track_coast_max: lost
        scans += [(0.1 * i, scan_of(person(*stand))) for i in range(35, 45)]  # back on the same spot
        records, summary = walk_check.walk(scans, P)
        self.assertEqual(summary["drops"], 1)
        self.assertEqual(summary["relocks"], 1)


if __name__ == "__main__":
    unittest.main()
