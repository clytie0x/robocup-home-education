"""Load the base's measured numbers from params.json."""
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
