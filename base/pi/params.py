"""Load the base's measured numbers from params.json.

JSON holds no comments, so where a number was measured is written here (the rest: base/docs/hardware-assessment-
2026-11-13.md).
- decel 0.10 m/s^2: tools/base_check.py's "distance after S#" (the 2026-11-13 runs) runs from S# to standstill, so
  it holds the stop delay too; decel 0.064 = v^2 / (2 x that distance) of the 0.08 m/s run counted the delay a second
  time, because safety.braking_distance adds v x stop_delay itself. Without it that run gives 0.09 to 0.11 (0.11 with
  stop_delay 0.27). The online estimates (calib.BaseEstimates, trusted in all four runs of 2026-11-15) measure the
  90 % to 10 % slope: 0.135 to 0.157. 0.10 is the cautious end; a stop test on the robot is to confirm it.
- command_latency 0.40 s: the online estimates gave 0.393 to 0.421 s in the same four runs (the first value, 0.17,
  was far off).
  stop_clearance stays 0.10 m, and calib.safe_braking still brakes with the smaller of an estimate and these.
"""
import json
from pathlib import Path


def load(path=None):
    """Read params.json next to this file, else in the parent folder."""
    if path is None:
        here = Path(__file__).resolve().parent
        path = next(p for p in (here / "params.json", here.parent / "params.json") if p.exists())
    return json.loads(Path(path).read_text())


def _all_measured(p, keys):
    return all(p[k] is not None for k in keys)


def speed_gates_open(p):
    return _all_measured(p, ("linear_cmd_scale", "decel", "stop_delay"))


def depth_gates_open(p):
    return _all_measured(p, ("camera_x", "camera_height", "camera_pitch_deg", "robot_height"))
