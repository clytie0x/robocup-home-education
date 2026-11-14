import sys
import threading
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "tools"))
import wheel_test  # noqa: E402

# The probe of 2026-11-14 23:5x (motor supply dead): PWM ramps 14 -> 76 % on both wheels, both counts
# stay put, flags 00, then 03 (both encoder faults) after 0.3 s above 30 %, then 07 once the drive stopped.
TONIGHT_E = [
    "Eleft:+0017983,right:+0000502,pwm:+000,+000,flags:04#",
    "Eleft:+0017983,right:+0000502,pwm:+014,+014,flags:00#",
    "Eleft:+0017983,right:+0000502,pwm:+031,+031,flags:00#",
    "Eleft:+0017983,right:+0000502,pwm:+052,+052,flags:00#",
    "Eleft:+0017983,right:+0000502,pwm:+076,+076,flags:00#",
    "Eleft:+0017983,right:+0000502,pwm:+000,+000,flags:03#",
    "Eleft:+0017983,right:+0000502,pwm:+000,+000,flags:03#",
    "Eleft:+0017983,right:+0000502,pwm:+000,+000,flags:07#",
]
TONIGHT_D = [
    "Dtarget:+0050,+0050,speed:+0000,+0000,pwm:+014,+014#",
    "Dtarget:+0050,+0050,speed:+0000,+0000,pwm:+076,+076#",
    "Dtarget:+0050,+0050,speed:+0000,+0000,pwm:+000,+000#",
]


def readings(lines):
    return [wheel_test.parse_e(line) for line in lines]


class ParseTest(unittest.TestCase):
    def test_e(self):
        self.assertEqual(wheel_test.parse_e("Eleft:+0017983,right:-0000502,pwm:+076,-012,flags:03#\r\n"),
                         {"left": 17983, "right": -502, "pwm": [76, -12], "flags": 3})
        self.assertEqual(wheel_test.parse_e("Eleft:+0000000,right:+0000000,pwm:+000,+000,flags:0A#")["flags"], 10)

    def test_d(self):
        self.assertEqual(wheel_test.parse_d(TONIGHT_D[1]), {"target": [50, 50], "speed": [0, 0], "pwm": [76, 76]})

    def test_garbage_is_refused(self):
        for line in ("", "Ox:+000.00,y:+000.00,th:+000.00#", "Eleft:+1,right:+2,pwm:+3,flags:00#",
                     "Eleft:x,right:+2,pwm:+3,+4,flags:00#", "Eleft:+1,right:+2,pwm:+3,+4,flags:00",
                     "Dtarget:+1,+2,speed:+3,+4,pwm:+5#"):
            with self.assertRaises(ValueError, msg=line):
                (wheel_test.parse_d if line.startswith("D") else wheel_test.parse_e)(line)


class VerdictTest(unittest.TestCase):
    def test_tonight_is_no_power(self):
        self.assertEqual(wheel_test.verdict(readings(TONIGHT_E)), "no_power")
        self.assertIn("21V motor battery", wheel_test.VERDICTS["no_power"][1])
        self.assertNotIn("power bank", wheel_test.VERDICTS["no_power"][1])
        self.assertEqual(wheel_test.VERDICTS["no_power"][0], 2)

    def test_fault_flags_alone_say_no_power(self):
        lines = ["Eleft:+0000100,right:+0000100,pwm:+000,+000,flags:00#",
                 "Eleft:+0000100,right:+0000100,pwm:+012,+012,flags:03#"]
        self.assertEqual(wheel_test.verdict(readings(lines)), "no_power")

    def test_both_wheels_counting_is_ok(self):
        lines = ["Eleft:+0000100,right:+0000200,pwm:+000,+000,flags:04#",
                 "Eleft:+0000180,right:+0000290,pwm:+025,+024,flags:00#",
                 "Eleft:+0000260,right:+0000371,pwm:+000,+000,flags:04#"]
        self.assertEqual(wheel_test.verdict(readings(lines)), "ok")

    def test_one_dead_encoder(self):
        lines = ["Eleft:+0000100,right:+0000200,pwm:+000,+000,flags:00#",
                 "Eleft:+0000260,right:+0000200,pwm:+030,+080,flags:02#"]
        self.assertEqual(wheel_test.verdict(readings(lines)), "one_encoder")

    def test_no_answer_and_unclear(self):
        self.assertEqual(wheel_test.verdict([]), "no_answer")
        lines = ["Eleft:+0000100,right:+0000200,pwm:+000,+000,flags:04#"] * 3
        self.assertEqual(wheel_test.verdict(readings(lines)), "unclear")

    def test_exit_codes_differ(self):
        codes = [code for code, _ in wheel_test.VERDICTS.values()]
        self.assertEqual(len(codes), len(set(codes)))
        self.assertEqual(wheel_test.VERDICTS["ok"][0], 0)


class FakePort:
    def __init__(self, e_lines, d_lines):
        self.e, self.d = list(e_lines), list(d_lines)
        self.written, self.timeout, self.query = [], 0.2, None

    def reset_input_buffer(self):
        pass

    def write(self, data):
        self.written.append(data)
        self.query = data

    def read_until(self, end):
        source = self.e if self.query == b"E#" else self.d
        line = source.pop(0) if len(source) > 1 else (source[0] if source else "")
        return line.encode()


class FakeWheels:
    def __init__(self, port):
        self.port, self.lock, self.sent = port, threading.Lock(), []

    def send(self, v, w):
        self.sent.append((v, w))

    def stop(self):
        self.sent.append("S#")


class DriveTest(unittest.TestCase):
    def run_drive(self, port):
        now = [0.0]
        w = FakeWheels(port)
        result = wheel_test.drive_test(w, clock=lambda: now[0], sleep=lambda s: now.__setitem__(0, now[0] + s),
                                       log=lambda *a: None)
        return w, result

    def test_drives_reads_and_stops(self):
        w, (e, d) = self.run_drive(FakePort(TONIGHT_E, TONIGHT_D))
        drives = [s for s in w.sent if s != "S#"]
        self.assertEqual(w.sent[-1], "S#")
        self.assertTrue(all(s == (wheel_test.SPEED, 0.0) for s in drives))
        self.assertAlmostEqual(len(drives) * wheel_test.READ_EVERY, wheel_test.DISTANCE / wheel_test.SPEED, delta=0.1)
        self.assertEqual(len(e), len(drives) + 2)  # before, one per step, after S#
        self.assertTrue(d)
        self.assertEqual(wheel_test.verdict(e), "no_power")

    def test_silent_base_is_not_driven(self):
        w, (e, d) = self.run_drive(FakePort([""], [""]))
        self.assertEqual((w.sent, e), ([], []))
        self.assertEqual(wheel_test.verdict(e), "no_answer")

    def test_small_constants(self):
        self.assertLessEqual(wheel_test.SPEED, 0.05)
        self.assertLessEqual(wheel_test.DISTANCE, 0.05)
        self.assertLess(wheel_test.READ_EVERY, 0.3)  # under the firmware's command timeout


if __name__ == "__main__":
    unittest.main()
