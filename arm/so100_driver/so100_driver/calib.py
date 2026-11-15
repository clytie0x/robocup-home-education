"""Calibration: raw encoder counts <-> URDF joint angles (radians).

angle = sign * (raw - zero) * 2*pi / 4096
`zero` is the raw reading when the real arm sits in the URDF zero pose (folded rest pose).
`min`/`max` are raw extremes measured by moving each joint by hand to its mechanical end.
"""
import json
import math
from pathlib import Path

JOINTS = ('shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper')
ARM_JOINTS = JOINTS[:5]
IDS = {name: i for i, name in enumerate(JOINTS, 1)}
COUNTS_PER_RAD = 4096 / (2 * math.pi)
# Directions found by the earlier two-pose calibration on this arm; re-checked visually in RViz.
DEFAULT_SIGN = {'shoulder_pan': -1, 'shoulder_lift': 1, 'elbow_flex': 1,
                'wrist_flex': 1, 'wrist_roll': 1, 'gripper': 1}
MARGIN_COUNTS = 20          # ~1.8 deg kept away from each hand-measured mechanical end
DEFAULT_PATH = Path.home() / 'so100_ws' / 'calibration.json'


class Calibration:
    def __init__(self, data):
        self.data = data
        for name in JOINTS:
            j = data[name]
            if j['sign'] not in (-1, 1) or not 0 <= j['zero'] <= 4095:
                raise ValueError(f'{name}: invalid sign/zero')

    @classmethod
    def load(cls, path=DEFAULT_PATH):
        return cls(json.loads(Path(path).read_text()))

    def save(self, path=DEFAULT_PATH):
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        tmp = path.with_suffix('.tmp')
        tmp.write_text(json.dumps(self.data, indent=2) + '\n')
        if path.exists():
            path.with_suffix('.bak').write_bytes(path.read_bytes())
        tmp.replace(path)

    @classmethod
    def from_zero(cls, raw_by_name, signs=None):
        signs = signs or DEFAULT_SIGN
        return cls({n: {'id': IDS[n], 'sign': signs[n], 'zero': int(raw_by_name[n]),
                        'min': None, 'max': None} for n in JOINTS})

    def complete(self):
        return all(self.data[n]['min'] is not None and self.data[n]['max'] is not None for n in JOINTS)

    def to_rad(self, name, raw):
        j = self.data[name]
        return j['sign'] * (raw - j['zero']) / COUNTS_PER_RAD

    def to_raw(self, name, rad):
        j = self.data[name]
        return j['zero'] + j['sign'] * rad * COUNTS_PER_RAD

    def raw_limits(self, name):
        j = self.data[name]
        if j['min'] is None:
            raise ValueError(f'{name}: range not calibrated yet')
        lo, hi = j['min'] + MARGIN_COUNTS, j['max'] - MARGIN_COUNTS
        if name == 'gripper':
            lo = j['min']  # closed jaws touching is the real closed position
        return lo, hi

    def rad_limits(self, name, urdf_limits=None):
        lo, hi = sorted(self.to_rad(name, r) for r in self.raw_limits(name))
        if urdf_limits is not None:
            lo, hi = max(lo, urdf_limits[0]), min(hi, urdf_limits[1])
        return lo, hi

    def clamp_raw(self, name, raw):
        lo, hi = self.raw_limits(name)
        return min(max(raw, lo), hi)
