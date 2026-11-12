"""Calibrate the SO-100 the LeRobot way: torque off, then move every joint by hand.

  ros2 run so100_driver calibrate status        # read-only: raw counts, angles, torque, voltage
  ros2 run so100_driver calibrate release       # torque OFF (hold the arm first!)
  ros2 run so100_driver calibrate zero          # arm in the folded rest pose = URDF zero
  ros2 run so100_driver calibrate sweep [secs] [joint ...]  # move joints to both mechanical ends
  ros2 run so100_driver calibrate flip <joint>  # reverse one joint direction

Stop the driver first: only one program may own the serial port.
"""
import math
import sys
import time

from . import feetech as ft
from .calib import DEFAULT_PATH, IDS, JOINTS, Calibration

PORT = '/dev/serial/by-id/usb-1a86_USB_Single_Serial_58FA093250-if00'


def out(*a):
    print(*a, flush=True)


def load():
    try:
        return Calibration.load(DEFAULT_PATH)
    except FileNotFoundError:
        return None


def read_all(bus):
    raw = bus.sync_read_positions(list(range(1, 7)))
    return {n: raw[IDS[n]] for n in JOINTS}


def torque_on(bus):
    return [n for n in JOINTS if bus.read1(IDS[n], ft.TORQUE_ENABLE)]


def status(bus):
    cal = load()
    raw = read_all(bus)
    on = torque_on(bus)
    volts = [bus.read1(i, ft.PRESENT_VOLTAGE) / 10 for i in range(1, 7)]
    out(f'torque ON: {on or "none"}   supply {min(volts):.1f}-{max(volts):.1f} V   calibration: {DEFAULT_PATH if cal else "none"}')
    for n in JOINTS:
        line = f'  {n:14s} raw {raw[n]:5d}'
        if cal:
            j = cal.data[n]
            line += f'  angle {math.degrees(cal.to_rad(n, raw[n])):7.1f} deg'
            if j['min'] is not None:
                lo, hi = sorted(math.degrees(cal.to_rad(n, r)) for r in (j['min'], j['max']))
                line += f'  range {lo:6.1f} .. {hi:6.1f} deg (raw {j["min"]}..{j["max"]})'
        out(line)


def require_limp(bus):
    on = torque_on(bus)
    if on:
        sys.exit(f'Torque is ON for {on}. Hold the arm, then run: calibrate release')


def average(bus, n=20):
    acc = {k: 0 for k in JOINTS}
    for _ in range(n):
        for k, v in read_all(bus).items():
            acc[k] += v
        time.sleep(0.01)
    return {k: round(v / n) for k, v in acc.items()}


def zero(bus):
    require_limp(bus)
    old = load()
    raw = average(bus)
    signs = {n: old.data[n]['sign'] for n in JOINTS} if old else None
    cal = Calibration.from_zero(raw, signs)
    cal.save(DEFAULT_PATH)
    out('Zero pose saved:', raw)
    out('Next: calibrate sweep   (ranges were reset)')


SHORT = {'shoulder_pan': 'pan', 'shoulder_lift': 'lift', 'elbow_flex': 'elbow',
         'wrist_flex': 'wflex', 'wrist_roll': 'wroll', 'gripper': 'grip'}


def sweep(bus, seconds, joints=JOINTS):
    """Record min/max of `joints`; every joint that moved >= 20 deg without wrapping is saved."""
    require_limp(bus)
    cal = load()
    if cal is None:
        sys.exit('Run `calibrate zero` first.')
    lo = read_all(bus)
    hi = dict(lo)
    prev = dict(lo)
    wrapped = set()
    end = time.monotonic() + seconds
    last_print = 0
    out(f'Recording {seconds:.0f}s for {", ".join(SHORT[n] for n in joints)}: move each slowly to both ends.')
    while time.monotonic() < end:
        try:
            raw = read_all(bus)
        except ft.FeetechError:
            continue
        for n in joints:
            if abs(raw[n] - prev[n]) > 2048:
                wrapped.add(n)
            lo[n], hi[n] = min(lo[n], raw[n]), max(hi[n], raw[n])
        prev = raw
        if time.monotonic() - last_print > 2:
            last_print = time.monotonic()
            out(f'  {end - last_print:4.0f}s left | ' + ' | '.join(
                f'{SHORT[n]} {(hi[n] - lo[n]) * 360 / 4096:4.0f}' for n in joints))
        time.sleep(0.02)
    saved, skipped = [], []
    for n in joints:
        if n in wrapped:
            if n != 'wrist_roll':
                skipped.append(f'{n}: encoder wrapped (impossible for this joint, check wiring)')
                continue
            # Wrist roll turns a full circle; usable = one encoder turn, never across raw 0/4095.
            lo[n], hi[n] = 0, 4095
        if hi[n] - lo[n] < 20 * 4096 / 360:
            skipped.append(f'{n}: moved only {(hi[n] - lo[n]) * 360 / 4096:.0f} deg, not saved')
            continue
        cal.data[n]['min'], cal.data[n]['max'] = lo[n], hi[n]
        saved.append(n)
    cal.save(DEFAULT_PATH)
    out('Saved:' if saved else 'Nothing saved.')
    for n in saved:
        a, b = sorted(math.degrees(cal.to_rad(n, r)) for r in (lo[n], hi[n]))
        out(f'  {n:14s} {a:7.1f} .. {b:7.1f} deg   ({(hi[n] - lo[n]) * 360 / 4096:.0f} deg of travel)')
    for s in skipped:
        out('  NOT saved ->', s)
    missing = [n for n in JOINTS if cal.data[n]['min'] is None]
    out(f'Still missing: {missing}' if missing else 'All six joints calibrated.')


def main():
    args = sys.argv[1:]
    if not args or args[0] not in ('status', 'release', 'zero', 'sweep', 'flip'):
        sys.exit(__doc__)
    if args[0] == 'flip':
        cal = load()
        name = args[1] if len(args) > 1 else ''
        if cal is None or name not in JOINTS:
            sys.exit(f'usage: calibrate flip <{"|".join(JOINTS)}>')
        cal.data[name]['sign'] *= -1
        cal.save(DEFAULT_PATH)
        out(f'{name} direction is now {cal.data[name]["sign"]:+d}')
        return
    try:
        bus = ft.Bus(PORT)
    except Exception as exc:  # noqa: BLE001
        sys.exit(f'Cannot open {PORT}: {exc}\nIs the driver (or old app) still running?')
    try:
        if args[0] == 'status':
            status(bus)
        elif args[0] == 'release':
            for i in range(1, 7):
                bus.write1(i, ft.TORQUE_ENABLE, 0)
            out('Torque OFF on all joints.')
        elif args[0] == 'zero':
            zero(bus)
        elif args[0] == 'sweep':
            joints = [j for j in args[2:] if j in JOINTS] or JOINTS
            if len(joints) != len(args[2:]) and args[2:]:
                sys.exit(f'unknown joint in {args[2:]}; use {JOINTS}')
            sweep(bus, float(args[1]) if len(args) > 1 else 60, joints)
    finally:
        bus.close()


if __name__ == '__main__':
    main()
