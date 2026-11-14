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

    def test_the_way_back_is_summed_up(self):
        def back(t, v, keyframe=None, queue=None, breadcrumbs=9):
            return dict(cycle(t, True), phase="return", v=v, w=0.0, period=0.1, keyframe=keyframe, queue=queue,
                        breadcrumbs=breadcrumbs)

        def match(why, shift):
            return {"used": why == "used", "index": 3, "share": 0.9, "shift": shift, "turn": 0.0, "why": why}

        records = [
            dict(cycle(0.0, True), phase="follow", v=0.1, w=0.0, period=None, keyframe=None, queue=None, breadcrumbs=1),
            dict(cycle(0.1, True), phase="follow", v=0.1, w=0.0, period=0.1, keyframe=None, queue=None, breadcrumbs=9),
            back(0.2, 0.1, match("far", 0.0)),
            back(0.3, 0.1, match("used", 0.04)),
            back(0.4, 0.0, match("used", 0.21)),  # standing: blocked
            back(0.5, 0.0, match("share", 0.6)),  # blocked; a refused match is not a correction
            back(0.6, 0.1, match("no match", 0.0), queue=7),
            back(0.7, 0.1, queue=7, breadcrumbs=8),  # a loop cut while... never on the way back, but the most counts
        ]
        s = replay.summary(records)
        self.assertEqual(s["breadcrumbs"], 9)
        self.assertEqual(s["keyframes"], {"used": 2, "share": 1, "no match": 1})
        self.assertAlmostEqual(s["largest_correction"], 0.21)
        self.assertAlmostEqual(s["queue_seconds"], 0.2)
        self.assertAlmostEqual(s["blocked_seconds"], 0.2)

    def test_an_old_log_without_the_way_back(self):
        s = replay.summary([cycle(0.0, True), cycle(0.1, True)])
        self.assertEqual((s["breadcrumbs"], s["keyframes"], s["largest_correction"]), (None, {}, None))
        self.assertEqual((s["queue_seconds"], s["blocked_seconds"]), (0.0, 0.0))

    def test_a_run_that_ends_unlocked_counts_the_gap_to_the_end(self):
        s = replay.summary([cycle(0.0, True), cycle(0.1, False), cycle(1.1, False)])
        self.assertAlmostEqual(s["longest_gap"], 1.0)


if __name__ == "__main__":
    unittest.main()
