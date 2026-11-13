# Robot software inventory (Pi, Jetson, repository), draft

Status: **draft, no owner decisions yet. Nothing has been moved.** Every proposed move goes
to an archive folder on the same machine (never a deletion) and waits for the owner's yes.

**Survey state:** the Pi was offline when this draft was written (2026-11-13: Tailscale
"last seen 52 m ago", no SSH, the Jetson could not ping 192.168.31.21). The Pi rows below
come from earlier records, not from a live listing:
- `robocup-home/hardware-handoff-2026-11-07/robot-hardware-inventory-2026-11-07/report.md` (read-only survey, 2026-11-07)
- `robocup-home/hardware-handoff-2026-11-07/pi-jetson-bridge-2026-11-13.md` (bridge handover)
- `base/docs/hardware-assessment-2026-11-13.md` (files changed on 2026-11-13, archive move)

Column "Verified": `live` = listed on the machine in this survey, `record` = from those
documents only. Before any action a live listing (`ls -la ~`, `du -sh`, `docker ps -a`,
`crontab -l`, enabled systemd units, a grep of every script for the paths it calls) must
replace every `record`.

Roles: **follower** = used by the new follower stack (`base/` on the Pi as `~/hmc-base`,
`base/jetson` on the Jetson as `~/hmc-follow`). **needed** = still used by something else.
**superseded** = replaced by the follower stack. **preserve** = backup or evidence.

## Raspberry Pi (`xohm@100.73.128.36`, home `~`)

| Item | Role | What it is / who uses it | Proposed action | Verified |
|---|---|---|---|---|
| `~/hmc-base/` | follower | Deployed by `base/tools/deploy.sh`: `base/pi`, `hmc/src/hmc` as `hmc/`, `params.json`, `jetson/server.py` (for the Pi tests), own `.venv` (system site packages), `logs/` | Keep. It is the entry point (`run_follow.py`) | record |
| `~/hmc-base/logs/` | preserve | Run logs, incl. `base-check/` speed ladder of 2026-11-13 | Keep. Copy to the laptop before any SD card work | record |
| `~/pi-jetson-bridge/` | needed | ROS1 to ROS2 bridge (`pi-jetson-bridge.sh`, `bridge.yaml`, `scan_relay.py`, `odom_relay.py`): gives the Jetson teams `/scan`, `/base_odom_tf`, `/rct_driver_status` on domain 42 | Keep | record |
| Docker `rct-noetic` | needed | ROS1 Noetic master, base driver from `~/catkin_ws`, `odom_relay.py` | Keep | record |
| Docker `rct-lds03` | needed | ROS2 LDS-03 node, domain 0 `/scan`, `ros1_bridge`, `scan_relay.py` | Keep | record |
| `~/robot-pi` | needed | Starts and stops the ROS sessions (`lidar-start`, `nav`, `stop`). The bridge needs `lidar-start`; the follower needs `stop` before it opens the ports | Keep | record |
| `~/catkin_ws/` | needed, partly superseded | ROS1 workspace, 23 packages (`rct_ahr_*`, navigation, `rct_person_follow_vm`). `rct_ahr_driver` (`driver.py`, `rct_driver_watchdog.py`) is the only ROS odometry source for the Jetson and the only code that documents the firmware protocol. Navigation and `rct_person_follow_vm` are superseded by the follower | Keep the whole workspace (it is built inside the container, moving packages breaks the build). Mark read-only in its README | record |
| `~/robot-deploy-kit/` | superseded (to check) | Old task layer: `config/robot.json`, `navigation_guard.py`, `command_guard.py`, `task_tree.py`, edited 2026-11-13 for 0.30 m/s | Archive, **only if** a grep shows `~/robot-pi` and the containers do not read it | record |
| `~/pi-drive/` | superseded | Old Python follower (`main.py`, `follow.py`, `decide.py`, `wheels.py`, `lidar.py`, `person.py`, `slam.py`, new `legs.py`, `chase.py`). Its useful parts were ported to `base/pi` (`wheels`, `lidar`, `legs`, `is_body`). `base/pi/tools` and the plan read it as a source | Archive after two checks: `~/hmc-base/.venv` does not depend on `~/pi-drive/.venv`, and nothing else imports it. Keep a copy of `pi-drive/README.md` measurements (already mirrored in `robocup-home/pi-drive`) | record |
| `~/backup-driver-20261113/` | preserve | Originals of every robot file edited on 2026-11-13 | Keep | record |
| `~/archive-xohm-20261113/` | preserve | 205 unused items moved on 2026-11-13, 6.88 GB | Keep. Proposed archive target for the moves above (as a new dated subfolder) | record |
| `~/real-sensors-check-*` | unknown | Seen 2026-11-07, contents never read. May already be in the archive | Read, then decide | record |
| Anything else in `~` | unknown | Not listed | Live survey | none |

## Jetson (`robotpsu@100.108.202.48`, shared with the arm and voice teams)

Listed live on 2026-11-13 (`ls ~`). Only one folder belongs to the base.

| Item | Role | Proposed action | Verified |
|---|---|---|---|
| `~/hmc-follow/` (`server.py`, `detector.py`, `engine.py`, `reid.py`, `bench.py`, `models/`, `tests/`, `logs/`) | follower | Keep | live |
| Everything else (`robot_voice*`, `ros2_ws`, `so100_ws`, `piper*`, `llama.cpp`, `models`, sherpa-onnx, ...) | other teams | Do not touch | live |

## Repository `robocup-home-education`

| Item | Role | Proposed action |
|---|---|---|
| `base/pi/run_follow.py`, `body.py`, `camera.py`, `depth_obstacles.py`, `geometry.py`, `legs.py`, `lidar.py`, `link.py`, `params.py`, `safety.py`, `senses.py`, `shapes.py`, `wheels.py` | follower | Keep (module changes: see below) |
| `base/pi/scanmatch.py` | follower tools | Only `tools/base_check.py` imports it. Could move to `tools/`; low value, leave |
| `base/pi/replay.py` | follower tools | Offline log summary. Keep |
| `base/pi/tools/base_check.py`, `sensor_check.py` | calibration | Keep. They are how every number in `params.json` was measured |
| `base/jetson/*` | follower | Keep |
| `base/tools/deploy.sh` | follower | Keep |
| `base/params.json` | follower | Keep. Must change together with the firmware (`firmware-design.md`, section 2) |
| `hmc/src/hmc/` | follower brain + simulator | `follower.py` and what it imports (`hri`, `navigation`, `perception`, `appearance`, `world`) run on the robot; the rest is the simulator. Keep as one package |
| `hmc/out/` | generated | Ignored by `hmc/.gitignore`. Keep |
| `hmc/pytest-of-teera/` (388 kB), `pip-build-env-*`, `pip-build-tracker-*`, `pip-ephem-wheel-cache-*`, `pip-install-*`, `pip-metadata-*`, `pip-modern-metadata-*`, `pip-unpack-*` at the repo root (about 6 MB) | leftovers | Untracked temp folders from a pytest and a pip run (not from this survey). Propose: remove, with the owner's yes |
| `voice/`, `arm/` | other work | Out of scope, not surveyed |

## Proposed module changes in `base/pi`

Ordered by expected effect. None is done; `run_follow.py`, `wheels.py` and the tests are
on branch `feature/person-follow`, so each change needs a word with the session that owns
that branch first.

1. **Move odometry polling out of the cycle, into the `Wheels` thread (safety and speed).**
   Today `run_follow.cycle` calls `wheels.read_odom()` inline: `reset_input_buffer`,
   write `O#`, `read_until(b"#")` with a 0.2 s timeout, up to 4 tries. A bad or missing
   reply can stall the 10 Hz cycle for up to 0.8 s, and during that time no new wheel
   command is computed (the watchdog keeps repeating the last packet for up to 0.3 s,
   then stops). The thread that already writes every 40 ms can poll `O#` at 25 Hz and keep
   the latest pose with its time; the cycle reads the cached pose and treats a pose older
   than 0.2 s as a stop reason. Effect: no serial wait in the cycle, a bounded worst case,
   and a fresher pose. Expected cost: about 40 lines in `wheels.py`, tests in
   `test_wheels.py`.
2. **One place for the base's units.** `wheels.packet` and `parse_odom` hold the firmware
   scales (via `params.json`), and the ROS driver holds its own copy. With new firmware
   both must go to 1.0 on the same day. Keep the scales only in `params.json` and make
   `wheels.py` refuse to start if the firmware reports a version (new `E#`/version reply)
   that does not match the scales. Effect: safety, the base can never run with the wrong
   scales after a flash. Depends on the firmware.
3. **Curvature-keeping clamp in `wheels.packet`.** It clamps `v` and `w` separately, so a
   fast turn asked together with speed changes the path. Scale both by the same factor
   instead. Effect: safety in tight spaces. Small change, but it changes driving, so it
   needs a test run.
4. **Leave the rest as it is.** The files are small (23 to 330 lines), each has one job and
   its own tests; merging them would not change speed or safety.

## Decisions needed from the owner

For each row with a proposed action: yes, no, or later. Most of the Pi rows can only be
decided after the live survey.
