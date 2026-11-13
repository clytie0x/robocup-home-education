"""Bring-up test for the new base firmware, run on the Pi right after flashing.

Wheels lifted off the floor, red button within reach. Needs only pyserial:

  python tools/safe_run.py .venv/bin/python ~/firmware-new/bringup.py

Checks, in order: S# and the replies, each encoder by hand, both wheels forward, a left
turn, S# stops, and the 0.3 s timeout stops. Prints PASS or FAIL for each and sends S#
however it ends.
"""
import argparse
import re
import sys
import time

PORT = "/dev/rct_ahr_robot"
PERIOD = 0.04  # the Pi resends drive packets every 40 ms
E_REPLY = re.compile(r"Eleft:([+-]\d{7}),right:([+-]\d{7}),pwm:([+-]\d{3}),([+-]\d{3}),flags:([0-9A-F]{2})#")
O_REPLY = re.compile(r"Ox:[+-]\d{3}\.\d{2},y:[+-]\d{3}\.\d{2},th:[+-]\d{3}\.\d{2}#")
P_REPLY = re.compile(r"Px:[+-]\d{7},y:[+-]\d{7},th:[+-]\d{7},t:\d{10}#")
TIMEOUT_FLAG = 0x04
QUARTER_TURN = 9600  # counts, about 38 600 per wheel turn (config.h)


def parse_e(text):
    """'Eleft:+0001320,right:-0000045,pwm:+037,-100,flags:05#' to (left, right, pwm_l, pwm_r, flags)."""
    m = E_REPLY.search(text)
    if not m:
        raise ValueError(f"bad E# reply {text!r}")
    left, right, pwm_l, pwm_r = (int(g) for g in m.groups()[:4])
    return left, right, pwm_l, pwm_r, int(m.group(5), 16)


class Base:
    def __init__(self, port):
        self.port = port

    def ask(self, request):
        self.port.reset_input_buffer()
        self.port.write(request)
        return self.port.read_until(b"#").decode("ascii", "replace")

    def encoders(self):
        return parse_e(self.ask(b"E#"))

    def stop(self):
        for _ in range(3):
            self.port.write(b"S#")

    def drive(self, packet, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.port.write(packet)
            time.sleep(PERIOD)


def report(name, ok, detail):
    print(f"{'PASS' if ok else 'FAIL'}  {name}: {detail}")
    return ok


def replies(base):
    base.stop()
    time.sleep(0.2)
    left, right, pwm_l, pwm_r, flags = base.encoders()
    o, p = base.ask(b"O#"), base.ask(b"P#")
    ok = bool(O_REPLY.search(o)) and bool(P_REPLY.search(p)) and pwm_l == pwm_r == 0
    return report("replies", ok, f"E flags {flags:02X} pwm {pwm_l},{pwm_r}; {o.strip()}; {p.strip()}")


def by_hand(base, wheel):
    before = base.encoders()
    input(f"Turn the {wheel.upper()} wheel FORWARD about a quarter turn by hand, then press Enter ")
    after = base.encoders()
    mine, other = (0, 1) if wheel == "left" else (1, 0)
    moved, still = after[mine] - before[mine], after[other] - before[other]
    ok = moved > QUARTER_TURN // 4 and abs(still) < QUARTER_TURN // 20
    return report(f"{wheel} encoder by hand", ok, f"{wheel} {moved:+d} counts, other wheel {still:+d}")


def driven(base, name, packet, want_left, want_right):
    before = base.encoders()
    base.drive(packet, 1.5)
    during = base.encoders()
    base.stop()
    time.sleep(0.5)
    dl, dr = during[0] - before[0], during[1] - before[1]
    ok = (dl > 1000 if want_left > 0 else dl < -1000) and (dr > 1000 if want_right > 0 else dr < -1000)
    return report(name, ok, f"left {dl:+d}, right {dr:+d} counts, pwm {during[2]},{during[3]}")


def stops(base, name, after_drive):
    base.drive(b"F005|+000#", 1.0)
    after_drive()
    time.sleep(0.6)  # 0.3 s timeout + 0.1 s ramp from 0.05 m/s, with margin
    a = base.encoders()
    time.sleep(0.3)
    b = base.encoders()
    moved = abs(b[0] - a[0]) + abs(b[1] - a[1])
    ok = moved < 50 and b[2] == b[3] == 0
    return report(name, ok, f"{moved} counts in 0.3 s after the stop, pwm {b[2]},{b[3]}, flags {b[4]:02X}")


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--port", default=PORT)
    args = ap.parse_args()
    input("Wheels lifted off the floor and the red button within reach? Press Enter to start ")
    import serial

    port = serial.Serial(args.port, 115200, parity=serial.PARITY_EVEN, timeout=0.3, write_timeout=0.1,
                         dsrdtr=False, rtscts=False)
    base = Base(port)
    results = []
    try:
        results.append(replies(base))
        results.append(by_hand(base, "left"))
        results.append(by_hand(base, "right"))
        results.append(driven(base, "both wheels forward", b"F005|+000#", +1, +1))
        ok = input("Did both wheels roll FORWARD? [y/n] ").strip().lower() == "y"
        results.append(report("forward by eye", ok, "owner"))
        results.append(driven(base, "turn left on the spot", b"F000|+030#", -1, +1))
        results.append(stops(base, "S# stops", base.stop))
        results.append(stops(base, "0.3 s timeout stops", lambda: None))
    finally:
        base.stop()
        port.close()
    print(f"{sum(results)} of {len(results)} passed")
    return 0 if all(results) else 1


if __name__ == "__main__":
    sys.exit(main())
