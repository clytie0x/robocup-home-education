# base

Wheeled base and person following on the real robot (Raspberry Pi, STM32, LiDAR, camera).

- Entry point: `base/pi/run_follow.py` (planned, not written yet)
- Lifecycle state: `active`
- Measured numbers: `base/params.json`, loaded by `base/pi/params.py`. A `null` value means not measured yet.

Docs:
- [Hardware assessment](docs/hardware-assessment-2026-11-13.md)
- [Person-follow design](docs/superpowers/specs/2026-11-13-person-follow-design.md)
- [Person-follow plan](docs/superpowers/plans/2026-11-13-person-follow.md)
- [New base firmware, design draft](docs/firmware-design.md); code in [firmware/](firmware/README.md), not on the board yet

Run the tests from the repo root (Windows, `;` separates paths):

```
PYTHONPATH="hmc/src;base/pi" python -m unittest discover -s base/pi/tests -v
```
