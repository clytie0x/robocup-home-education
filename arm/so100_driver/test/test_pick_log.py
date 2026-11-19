"""Per-pick phase timings."""
import json
import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from so100_driver.pick_log import PhaseLog, write_pick  # noqa: E402


class PhaseLogTest(unittest.TestCase):
    def test_a_phase_ends_when_the_next_one_starts(self):
        times = iter([0.0, 1.0, 2.5])
        log = PhaseLog(clock=lambda: next(times))
        log.mark("home")
        log.mark("pick_bag")
        body = log.finish(ok=True, code="SUCCESS")
        self.assertEqual(body["phases"], [{"phase": "home", "s": 1.0}, {"phase": "pick_bag", "s": 1.5}])
        self.assertEqual(body["total_s"], 2.5)
        self.assertTrue(body["ok"])
        self.assertEqual(body["code"], "SUCCESS")

    def test_write_pick_names_the_file_from_the_task(self):
        with tempfile.TemporaryDirectory() as folder:
            path = write_pick(folder, {"phases": []}, "20261119-080000", "pick bag")
            self.assertTrue(path.endswith("20261119-080000-pick_bag.json"))
            with open(path) as f:
                self.assertEqual(json.load(f)["phases"], [])
            self.assertTrue(os.path.isfile(path))


if __name__ == "__main__":
    unittest.main()
