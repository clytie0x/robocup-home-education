import json
import tempfile
import unittest
from pathlib import Path

import replay


def cycle(t, locked, stop=None, nearest=1.0, cycle_time=0.1):
    return {"time": t, "pose": [0, 0, 0], "locked": locked, "stop": stop, "nearest": nearest, "cycle_time": cycle_time}


class Replay(unittest.TestCase):
    def test_replay_summary_counts(self):
        records = [
            cycle(0.0, True, cycle_time=None),
            cycle(0.1, False, stop="ambiguous"),
            {"time": 0.15, "say": "Please wait for me."},  # speech is not a cycle
            cycle(0.2, False, stop="obstacle", nearest=0.4),
            cycle(0.3, False, stop="obstacle", cycle_time=0.3),
            cycle(0.4, True, nearest=None),
            cycle(0.5, True),
            {"time": 0.6, "stop": "odometry", "end": "odometry: bad odom 'x'"},
        ]
        with tempfile.TemporaryDirectory() as folder:
            (Path(folder) / "log.jsonl").write_text("".join(json.dumps(r) + "\n" for r in records))
            s = replay.summary(replay.read_log(folder))
        self.assertEqual(s["cycles"], 6)
        self.assertAlmostEqual(s["locked_share"], 3 / 6)
        self.assertAlmostEqual(s["longest_gap"], 0.3)  # unlocked from 0.1 to 0.4
        self.assertEqual(s["closest"], 0.4)
        self.assertEqual(s["stops"], {"ambiguous": 1, "obstacle": 2, "odometry": 1})
        self.assertAlmostEqual(s["cycle_median"], 0.1)
        self.assertAlmostEqual(s["cycle_worst"], 0.3)

    def test_a_run_that_ends_unlocked_counts_the_gap_to_the_end(self):
        s = replay.summary([cycle(0.0, True), cycle(0.1, False), cycle(1.1, False)])
        self.assertAlmostEqual(s["longest_gap"], 1.0)


if __name__ == "__main__":
    unittest.main()
