"""Laptop check of image.py and update.py against a fake serial port and a fake stm32flash.

  python -m unittest discover -s base/firmware/tools -v
"""
import os
import struct
import tempfile
import unittest
import zlib

import image
import update


def raw_app(body=b"\x00" * 0x300, build=b"Vversion:1,build:abc1234#"):
    """An unsealed application laid out like the real one."""
    raw = bytearray(0x400) + body
    struct.pack_into("<II", raw, 0, 0x20005000, image.APP_BASE + 0x201)
    struct.pack_into("<II", raw, 0x100, image.APP_MAGIC, 0)
    raw[0x300:0x300 + len(build)] = build
    return bytes(raw)


class Image(unittest.TestCase):
    def test_seal_then_check(self):
        sealed = image.seal(raw_app() + b"\x01")  # odd length: padded to whole words
        self.assertIsNone(image.check(sealed))
        self.assertEqual(len(sealed) % 4, 0)
        length = struct.unpack_from("<I", sealed, 0x104)[0]
        self.assertEqual(struct.unpack_from("<I", sealed, length)[0], zlib.crc32(sealed[:length]))
        self.assertEqual(image.version(sealed), "Vversion:1,build:abc1234#")

    def test_check_refuses_broken_images(self):
        sealed = image.seal(raw_app())
        flipped = bytearray(sealed)
        flipped[0x350] ^= 1
        self.assertIn("CRC", image.check(bytes(flipped)))
        self.assertIn("length", image.check(sealed[:-8]))
        self.assertIn("header", image.check(b"\xff" * 0x1000 + sealed))  # full.bin is not an app
        self.assertIn("header", image.check(b"\xff" * 0x200))  # erased / old firmware

    def test_combine_puts_the_app_at_4_kib(self):
        sealed = image.seal(raw_app())
        full = image.combine(b"\x11" * 0x2a0, sealed)
        self.assertEqual(full[0x1000:], sealed)
        self.assertEqual(set(full[0x2a0:0x1000]), {0xFF})
        with self.assertRaises(ValueError):
            image.combine(b"\x11" * 0x1001, sealed)


class Proc:
    def __init__(self, code, out=""):
        self.returncode, self.stdout, self.stderr = code, out, ""


class FakeChip:
    """Stands in for fuser, stm32flash and the serial link; records what was run."""

    def __init__(self, app, running="Vversion:1,build:old0000#", u_reply="Uok#", fail=None, busy=False):
        self.app, self.running, self.u_reply = app, running, u_reply
        self.fail, self.busy = fail, busy
        self.flash = None
        self.commands, self.written = [], []

    def run(self, cmd, **_kw):
        self.commands.append(cmd)
        if cmd[0] == "fuser":
            return Proc(0 if self.busy else 1, "1234" if self.busy else "")
        action = cmd[3]
        if action == self.fail:
            return Proc(1)
        if action == "-w":
            self.flash = open(cmd[4], "rb").read()
        elif action == "-r":
            open(cmd[4], "wb").write(self.flash)
        elif action == "-g":
            self.running = image.version(self.flash)
        return Proc(0)

    def connect(self, _port):
        chip = self

        class Link:
            def reset_input_buffer(self):
                pass

            def write(self, data):
                chip.written.append(data)

            def read_until(self, _end):
                last = chip.written[-1]
                if last == b"V#":
                    return chip.running.encode()
                if last in (b"U#", b"Uforce#"):
                    return (chip.u_reply + "\r\n").encode()
                return b""

            def close(self):
                pass

        return Link()


class Update(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.path = os.path.join(self.tmp.name, "app.bin")
        self.sealed = image.seal(raw_app())
        open(self.path, "wb").write(self.sealed)

    def go(self, chip, rescue=False):
        update.update(self.path, "/dev/null", rescue, run=chip.run, connect=chip.connect, sleep=lambda s: None)

    def actions(self, chip):
        return [c[3] if c[0] == "stm32flash" else c[0] for c in chip.commands]

    def test_happy_path(self):
        chip = FakeChip(self.sealed)
        self.go(chip)
        self.assertEqual(self.actions(chip), ["fuser", "-w", "-r", "-g"])
        write = chip.commands[1]
        self.assertIn("-v", write)
        self.assertEqual(write[write.index("-S") + 1], f"0x08001000:{len(self.sealed)}")  # never page 0
        self.assertEqual(chip.commands[3][4], "0x08000000")  # Go through stage1
        self.assertLess(chip.written.index(b"S#"), chip.written.index(b"U#"))
        self.assertEqual(chip.running, "Vversion:1,build:abc1234#")

    def test_refuses_a_broken_file_before_touching_anything(self):
        open(self.path, "wb").write(self.sealed[:-1])
        chip = FakeChip(self.sealed)
        with self.assertRaisesRegex(update.Failed, "Nothing was written"):
            self.go(chip)
        self.assertEqual(chip.commands, [])

    def test_busy_port(self):
        chip = FakeChip(self.sealed, busy=True)
        with self.assertRaisesRegex(update.Failed, "in use"):
            self.go(chip)
        self.assertEqual(self.actions(chip), ["fuser"])

    def test_old_firmware_or_moving_wheels_stop_before_stm32flash(self):
        for chip, why in ((FakeChip(self.sealed, running=""), "no V# reply"),
                          (FakeChip(self.sealed, u_reply="U!busy#"), "U# refused")):
            with self.assertRaisesRegex(update.Failed, why):
                self.go(chip)
            self.assertEqual(self.actions(chip), ["fuser"])

    def test_failures_name_the_chip_state(self):
        for step, state in (("-w", "does not answer"), ("-r", "waits in the ST ROM bootloader"),
                            ("-g", "Power the base off and on")):
            chip = FakeChip(self.sealed, fail=step)
            with self.assertRaisesRegex(update.Failed, state):
                self.go(chip)

    def test_readback_mismatch(self):
        chip = FakeChip(self.sealed)
        real_run = chip.run

        def run(cmd, **kw):
            result = real_run(cmd, **kw)
            if cmd[0] == "stm32flash" and cmd[3] == "-r":
                open(cmd[4], "wb").write(b"\xaa" + self.sealed[1:])
            return result

        chip.run = run
        with self.assertRaisesRegex(update.Failed, "read-back differs"):
            self.go(chip)
        self.assertNotIn("-g", self.actions(chip))

    def test_force_sends_uforce(self):
        chip = FakeChip(self.sealed)
        update.update(self.path, "/dev/null", run=chip.run, connect=chip.connect, sleep=lambda s: None,
                      force=True)
        self.assertIn(b"Uforce#", chip.written)
        self.assertNotIn(b"U#", chip.written)

    def test_break_escape_sends_only_the_pattern(self):
        chip = FakeChip(self.sealed, running="")
        slow = []

        class SlowLink:
            def write(self, data):
                slow.append(data)

            def close(self):
                slow.append("closed")

        asked = []
        update.update(self.path, "/dev/null", run=chip.run, connect=chip.connect, sleep=lambda s: None,
                      escape=True, open_slow=lambda port: SlowLink(), wait_for_owner=asked.append)
        self.assertEqual(len(asked), 1)
        self.assertIn("Power the base OFF", asked[0])
        self.assertTrue(slow and slow[-1] == "closed")
        self.assertTrue(all(set(b) == {0} for b in slow[:-1]))  # zero bytes only
        self.assertEqual(chip.written[:-1], [])  # nothing at 115200 before stm32flash
        self.assertEqual(self.actions(chip), ["fuser", "-w", "-r", "-g"])

    def test_break_pattern_keeps_the_line_fed(self):
        written, now = [], [0.0]

        class Link:
            def write(self, data):
                written.append((now[0], len(data)))

        def sleep(s):
            now[0] += s

        update.break_pattern(Link(), lambda: now[0] >= 1.9, sleep=sleep, clock=lambda: now[0])
        self.assertEqual(written[0], (0.0, 3))  # three bytes ahead
        self.assertEqual(sum(n for _, n in written), 3 + 10)  # then one per 0.2 s
        self.assertTrue(all(abs(t - 0.2 * i) < 1e-9 for i, (t, _) in enumerate(written[1:])))

    def test_rescue_skips_the_serial_handshake(self):
        chip = FakeChip(self.sealed, running="")
        self.go(chip, rescue=True)
        self.assertNotIn(b"U#", chip.written)
        self.assertEqual(self.actions(chip), ["fuser", "-w", "-r", "-g"])


if __name__ == "__main__":
    unittest.main()
