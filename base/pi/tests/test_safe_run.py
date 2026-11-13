import signal
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "tools"))
import safe_run  # noqa: E402


class FakeWheels:
    def __init__(self, sent):
        self.sent = sent
        sent.append("open (S# x3)")

    def stop(self):
        self.sent.append("S#")

    def close(self):
        self.sent.append("close (S# x3)")


class SafeRunTest(unittest.TestCase):
    def setUp(self):
        self.saved = {s: signal.getsignal(s) for s in safe_run.STOP_SIGNALS}

    def tearDown(self):
        for s, handler in self.saved.items():
            signal.signal(s, handler)

    def check(self, code_line, expect_code):
        sent = []
        code = safe_run.run([sys.executable, "-c", code_line], open_wheels=lambda: FakeWheels(sent))
        self.assertEqual(sent, ["open (S# x3)", "S#", "close (S# x3)"])
        if expect_code is not None:
            self.assertEqual(code, expect_code)

    def test_a_normal_end_still_sends_stop(self):
        self.check("pass", 0)

    def test_a_crash_sends_stop_and_keeps_the_exit_code(self):
        self.check("raise SystemExit(3)", 3)

    def test_a_killed_program_sends_stop(self):
        # kill -9 on the Pi; on Windows os.kill ends the process the same hard way
        self.check("import os, signal; os.kill(os.getpid(), getattr(signal, 'SIGKILL', signal.SIGTERM))", None)

    def test_a_dead_port_is_reported(self):
        def broken():
            raise OSError("no such device")

        code = safe_run.run([sys.executable, "-c", "pass"], open_wheels=broken)
        self.assertEqual(code, 1)


if __name__ == "__main__":
    unittest.main()
