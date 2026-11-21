# base

Wheeled base and person following on the real robot (Raspberry Pi, STM32, LiDAR, camera).

- Entry points: the console on the Pi (`base/pi/console.py`, kept running by
  `base/pi/tools/run_console.sh`) and the agent on the Jetson (`base/jetson/agent.py`). The
  console starts `base/pi/run_follow.py` for a task.
- Lifecycle state: `active`
- Measured numbers: `base/params.json`, loaded by `base/pi/params.py`. A `null` value means not measured yet.

| Entry point | Machine | What it does |
|---|---|---|
| `base/pi/console.py` | Pi | Web console on :8081: starts `run_follow.py` and the checks, relays the camera and the LiDAR, shows the Jetson |
| `base/jetson/agent.py` | Jetson | Agent on :8090: starts, stops and restarts the eight programs, Start all in their order |
| `base/tools/deploy.sh` | laptop | Deploys a git commit to both hosts, writes `VERSION.json`, backs up, `--rollback`, `--voice`, `--arm`, `--params` |

Docs:
- [Robot console](docs/robot-console.md)
- [Venue checklist](docs/venue-checklist.md)
- [Hardware assessment](docs/hardware-assessment-2026-11-13.md)
- [Person-follow design](docs/superpowers/specs/2026-11-13-person-follow-design.md)
- [Person-follow plan](docs/superpowers/plans/2026-11-13-person-follow.md)
- [Systemd user units](systemd/README.md)
- [New base firmware, design](docs/firmware-design.md); code in [firmware/](firmware/README.md), flashed 2026-11-14 (0c020e4)

Run the tests with pytest from each suite's folder, as CI does (`.github/workflows/ci.yml`, job `robot`):

```
pip install -r base/pi/requirements.txt -r base/jetson/requirements.txt pytest
cd base/pi && python -m pytest -q
```

The same in base/jetson, base/firmware/tools, base/pc/virtual_operator and voice/voice_assistant,
`python -m pytest -q test` in arm/so100_driver (with scipy), `make test` in base/firmware for the C code. On the Pi, without pytest:
`.venv/bin/python -m unittest discover -s tests`.
