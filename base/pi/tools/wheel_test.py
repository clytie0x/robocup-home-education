"""Wheel test: drive about 4 cm forward slowly and say whether the motors and encoders work.

  python tools/safe_run.py .venv/bin/python tools/wheel_test.py

The base moves DISTANCE forward at SPEED (both small constants below). Every READ_EVERY s it sends
the drive packet and reads D# (target, speed, PWM) and E# (encoder counts, PWM, fault flags) from
the base firmware (0c020e4 or later; the old firmware has no D#/E# and gets "no answer").
Then S#, and one more E# for the final counts. It prints every reading and one verdict line,
"VERDICT: <key>: <explanation>", writes logs/wheel-test/last.json (the console's Overview reads
it) and exits with the verdict's code:

  0 ok            both wheels turned
  2 no_power      PWM went up, neither encoder counted (2026-11-14: no motor power; flags 03)
  3 one_encoder   one wheel counted, the other did not
  4 no_answer     the base did not answer E# (port gone, old firmware, board off)
  5 unclear       the firmware never raised the PWM, or the readings do not fit a verdict
  6 refused       not started through tools/safe_run.py
"""
import json
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))  # params.py, wheels.py

SPEED = 0.05  # m/s
DISTANCE = 0.04  # m, about 4 cm: DISTANCE / SPEED = 0.8 s of driving, as the probe of 2026-11-14
READ_EVERY = 0.07  # s; under the firmware's 0.3 s command timeout, so the drive never stops by itself
REPLY_TIMEOUT = 0.1  # s per D# or E# answer
SETTLE = 0.4  # s after S# before the last E#
PWM_UP = 20  # %, PWM at least this on a wheel means the firmware tried to turn it
MIN_COUNTS = 5  # encoder counts a turning wheel moves at least, over 4 cm
FAULT_LEFT, FAULT_RIGHT, NO_COMMAND = 0x01, 0x02, 0x04
OUT = Path(__file__).resolve().parent.parent / "logs" / "wheel-test"

VERDICTS = {  # key: (exit code, plain explanation)
    "ok": (0, "both wheels turned and both encoders counted"),
    "no_power": (2, "the wheels get no power: check that the 21V motor battery is plugged in and charged"),
    "one_encoder": (3, "one wheel gave no encoder counts: its encoder or its motor cable"),
    "no_answer": (4, "no answer from the base: check the USB cable to the motor board and that the board is on"),
    "unclear": (5, "the firmware did not raise the PWM; run it again, then look at the readings"),
    "refused": (6, "start it through tools/safe_run.py"),
}


def _fields(line, letter, names):
    """'<letter>name:a,b,name:c#' to a flat list of ints, checked against names."""
    line = line.strip()
    if not (line.startswith(letter) and line.endswith("#")):
        raise ValueError("bad %s# reply %r" % (letter, line))
    values, seen = [], []
    for part in line[1:-1].split(","):
        if ":" in part:
            name, part = part.split(":", 1)
            seen.append(name)
        values.append(part)
    if seen != names:
        raise ValueError("bad %s# reply %r" % (letter, line))
    return values


def parse_e(line):
    """'Eleft:+0017983,right:+0000502,pwm:+076,+076,flags:03#' to a dict."""
    v = _fields(line, "E", ["left", "right", "pwm", "flags"])
    if len(v) != 5:
        raise ValueError("bad E# reply %r" % line)
    try:
        return {"left": int(v[0]), "right": int(v[1]), "pwm": [int(v[2]), int(v[3])], "flags": int(v[4], 16)}
    except ValueError:
        raise ValueError("bad E# reply %r" % line) from None


def parse_d(line):
    """'Dtarget:+0050,+0050,speed:+0000,+0000,pwm:+076,+076#' to a dict (mm/s, %)."""
    v = _fields(line, "D", ["target", "speed", "pwm"])
    if len(v) != 6:
        raise ValueError("bad D# reply %r" % line)
    try:
        n = [int(x) for x in v]
    except ValueError:
        raise ValueError("bad D# reply %r" % line) from None
    return {"target": n[0:2], "speed": n[2:4], "pwm": n[4:6]}


def verdict(e_readings):
    """The verdict key from the E# readings in order (the first before driving, the last after S#)."""
    if not e_readings:
        return "no_answer"
    first, last = e_readings[0], e_readings[-1]
    moved = [abs(last["left"] - first["left"]) >= MIN_COUNTS, abs(last["right"] - first["right"]) >= MIN_COUNTS]
    pwm = [max(abs(e["pwm"][i]) for e in e_readings) for i in (0, 1)]
    flags = 0
    for e in e_readings:
        flags |= e["flags"]
    if all(moved):
        return "ok"
    if any(moved):
        return "one_encoder"
    if all(p >= PWM_UP for p in pwm) or flags & (FAULT_LEFT | FAULT_RIGHT) == FAULT_LEFT | FAULT_RIGHT:
        return "no_power"
    if any(p >= PWM_UP for p in pwm):
        return "one_encoder"  # one side driven hard and no counts anywhere: the side that never rose is odd too
    return "unclear"


def ask(port, lock, query, timeout=REPLY_TIMEOUT):
    """One query (b"E#" or b"D#") and its answer as text, '' when nothing came."""
    with lock:
        port.reset_input_buffer()
        old, port.timeout = port.timeout, timeout
        port.write(query)
    try:
        return port.read_until(b"#").decode("ascii", "replace")
    finally:
        with lock:
            port.timeout = old


def read_e(port, lock, log):
    line = ask(port, lock, b"E#")
    log("E# %s" % line.strip())
    try:
        return parse_e(line)
    except ValueError:
        return None


def drive_test(w, clock=time.monotonic, sleep=time.sleep, log=print):
    """Drive DISTANCE at SPEED reading D# and E#; returns (E readings, D readings). S# at the end."""
    port, lock = w.port, w.lock
    readings, diags = [], []
    first = read_e(port, lock, log)
    if first is None:
        return readings, diags
    readings.append(first)
    start = clock()
    try:
        while clock() - start < DISTANCE / SPEED:
            t = clock()
            w.send(SPEED, 0.0)
            d = ask(port, lock, b"D#")
            log("%.2f s  D# %s" % (t - start, d.strip()))
            try:
                diags.append(parse_d(d))
            except ValueError:
                pass
            e = read_e(port, lock, log)
            if e is not None:
                readings.append(e)
            sleep(max(0.0, READ_EVERY - (clock() - t)))
    finally:
        w.stop()
    sleep(SETTLE)
    e = read_e(port, lock, log)
    if e is not None:
        readings.append(e)
    return readings, diags


def save(key, readings, diags, out=OUT):
    out.mkdir(parents=True, exist_ok=True)
    code, text = VERDICTS[key]
    record = {"time": time.time(), "verdict": key, "code": code, "text": text, "speed": SPEED,
              "distance": DISTANCE, "e": readings, "d": diags}
    (out / "last.json").write_text(json.dumps(record))
    (out / time.strftime("%Y%m%d-%H%M%S.json")).write_text(json.dumps(record))


def main():
    import params
    import wheels

    if not wheels.under_safe_run():
        print("VERDICT: refused: " + VERDICTS["refused"][1])
        return VERDICTS["refused"][0]
    try:
        w = wheels.Wheels(params.load(), start_thread=False)  # opening sends S#, which also clears held faults
    except Exception as error:
        print("cannot open the wheel port: %s" % error)
        key, readings, diags = "no_answer", [], []
    else:
        try:
            time.sleep(0.2)
            readings, diags = drive_test(w)
        finally:
            w.close()
        key = verdict(readings)
    save(key, readings, diags)
    code, text = VERDICTS[key]
    if readings:
        print("counts left %+d, right %+d; most PWM %d %%, %d %%; flags seen %02X" % (
            readings[-1]["left"] - readings[0]["left"], readings[-1]["right"] - readings[0]["right"],
            max(abs(e["pwm"][0]) for e in readings), max(abs(e["pwm"][1]) for e in readings),
            _all_flags(readings)))
    print("VERDICT: %s: %s" % (key, text), flush=True)
    return code


def _all_flags(readings):
    flags = 0
    for e in readings:
        flags |= e["flags"]
    return flags


if __name__ == "__main__":
    sys.exit(main())
