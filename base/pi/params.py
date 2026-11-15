"""Load the base's measured numbers from params.json.

JSON holds no comments, so where a number was measured is written here (the rest: base/docs/hardware-assessment-
2026-11-13.md).
- decel 0.14 m/s^2 and command_latency 0.40 s: the online estimates (calib.BaseEstimates, logged as "estimates"),
  trusted in all four runs of 2026-11-15, gave decel 0.135 to 0.157 and latency 0.393 to 0.421 s. The first
  values, 0.064 (the slowest coast of the 0.08 m/s run, 2026-11-13) and 0.17, were far off. A larger decel
  shortens the braking strip (safety.braking_distance) and lets the brain brake later (hmc.world.MAX_DECEL);
  stop_clearance stays 0.10 m, and calib.safe_braking still brakes with the smaller of an estimate and this.
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
