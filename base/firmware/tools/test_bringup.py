"""Laptop check of bringup.py's reply parsing and pass rules, against a fake port.

  python -m unittest discover -s base/firmware/tools -v
"""
import unittest

import bringup


class FakePort:
    """Answers E# with the next canned reply; records everything written."""

    def __init__(self, e_replies):
        self.e_replies = list(e_replies)
        self.written = []

    def reset_input_buffer(self):
        pass

    def write(self, data):
        self.written.append(data)

    def read_until(self, _end):
        last = self.written[-1]
        if last == b"E#":
            return self.e_replies.pop(0).encode()
        if last == b"O#":
            return b"Ox:+000.00,y:+000.00,th:+000.00#"
        return b"Px:+0000000,y:+0000000,th:+0000000,t:0000012345#"


def e(left, right, pwm_l=0, pwm_r=0, flags=0):
    return f"Eleft:{left:+08d},right:{right:+08d},pwm:{pwm_l:+04d},{pwm_r:+04d},flags:{flags:02X}#\r\n"


class Parse(unittest.TestCase):
    def test_parses_the_firmware_format(self):
        self.assertEqual(bringup.parse_e("Eleft:+0001320,right:-0000045,pwm:+037,-100,flags:05#\r\n"),
                         (1320, -45, 37, -100, 5))

    def test_broken_reply_is_an_error(self):
        for bad in ("", "Eleft:+1320,right:-45,pwm:+37,-100,flags:05#", "junk#"):
            with self.assertRaises(ValueError):
                bringup.parse_e(bad)


class Rules(unittest.TestCase):
    def setUp(self):
        sleep, drive = bringup.time.sleep, bringup.Base.drive
        self.addCleanup(setattr, bringup.time, "sleep", sleep)
        self.addCleanup(setattr, bringup.Base, "drive", drive)
        bringup.time.sleep = lambda _s: None  # the fake port answers at once
        bringup.Base.drive = lambda self, packet, seconds: self.port.write(packet)

    def test_turn_left_needs_left_back_and_right_forward(self):
        base = bringup.Base(FakePort([e(0, 0), e(-4000, 4000, -40, 40)]))
        self.assertTrue(bringup.driven(base, "turn", b"F000|+030#", -1, +1))
        base = bringup.Base(FakePort([e(0, 0), e(4000, 4000, 40, 40)]))
        self.assertFalse(bringup.driven(base, "turn", b"F000|+030#", -1, +1))

    def test_stop_needs_still_wheels_and_zero_pwm(self):
        base = bringup.Base(FakePort([e(5000, 5000), e(5003, 5001)]))
        self.assertTrue(bringup.stops(base, "stop", base.stop))
        base = bringup.Base(FakePort([e(5000, 5000, 40, 40), e(5400, 5400, 40, 40)]))
        self.assertFalse(bringup.stops(base, "stop", base.stop))


if __name__ == "__main__":
    unittest.main()
