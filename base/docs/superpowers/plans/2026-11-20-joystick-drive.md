# Joystick drive Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** The DualShock 4 on the Pi (`/dev/input/js0`) drives the base wheels, started as a console check, with a deadman and LiDAR guards.

**Architecture:** One new tool `base/pi/tools/joystick_drive.py` (pure `command()` mapping, a `Pad` reader, a guarded `drive()` loop, `main()`), added to the console's `TOOL_CHECKS` like `back_up`, plus a panel on the Checks tab. Forward and turning use `safety.must_stop`, backward uses `safety.rear_clear` and `Wheels.send_reverse` exactly as `tools/back_up.py` does.

**Tech Stack:** Python 3 (Pi `.venv`, laptop 3.13), numpy, pytest/unittest as in `base/pi/tests`; the console page is plain HTML and JS.

**Spec:** `base/docs/superpowers/specs/2026-11-20-joystick-drive-design.md`

## Global Constraints

- Wheels open only under `tools/safe_run.py` (`wheels.under_safe_run()`); LiDAR and wheels only inside a console job (`console_gate`).
- Never above `params.json` `max_speed` (0.12 m/s) or `max_turn_rate` (0.49 rad/s). Backward only `Wheels.send_reverse`, speed `REAR_SPEED` 0.04 m/s (`wheels.REVERSE_MAX` is 0.06), turn 0.0, only after `safety.rear_clear`.
- `--yes-owner-at-robot` is accepted only from the console page; `--dry` and `--device` are not allowed through the console.
- No new package; the joystick is read as raw `js_event` records.
- Run tests from `base/pi`: `python -m pytest tests/<file> -q` (baseline today: `test_console_control.py`, `test_back_up.py`, `test_wheels.py` give 76 passed).
- The owner has not authorized a commit. Leave every change uncommitted; Task 5 needs the owner's yes to commit because `deploy.sh` deploys a commit, never the working tree.

## Review Focus

- The deadman is already held when the tool starts (a stuck button or a held start-up event): nothing moves until it has been seen released once.
- The controller is unplugged mid-drive: S# at once, the tool ends with `"controller gone"`.
- An empty or stale LiDAR scan: `safety.must_stop` returns False for no points, so the tool must stop itself (as `back_up.blocked` does), never drive blind.
- The stick at full deflection on both axes, and backward with the turn stick pushed: v and w stay inside the limits (`wheels.clamp_curvature`), backward ignores the turn.
- `js0` sends events only on change: a held stick with no new event is not silence and must keep driving.

---

### Task 1: `parse_event` and `command` (pure, no hardware)

**Files:**
- Create: `base/pi/tools/joystick_drive.py`
- Test: `base/pi/tests/test_joystick_drive.py`

**Interfaces:**
- Produces (module constants, js numbering of the DS4 under the kernel's `hid-playstation`, to be confirmed with `--dry` in Task 5): `DEADMAN = 5` (R1), `AXIS_FWD = 1` (left stick Y, up is negative), `AXIS_TURN = 3` (right stick X, right is positive), `DEAD_ZONE = 0.15`, `REAR_SPEED = 0.04`, `AXIS_FULL = 32767`.
- Produces `parse_event(data: bytes) -> tuple[int, int, int]`: `(type, number, value)` from one 8-byte `struct "IhBB"` record, the init flag `0x80` removed from the type (`1` button, `2` axis).
- Produces `command(axes: dict[int, int], buttons: dict[int, int], p: dict, max_speed: float) -> tuple[str, float, float]`: `(kind, v, w)`, kind one of `"stop"`, `"forward"`, `"reverse"`; a missing axis or button counts as 0.

- [ ] **Step 1: Write the failing tests** in `tests/test_joystick_drive.py` (class `CommandTest`, `P = params.load()`, held = `{5: 1}`):
  - `parse_event(struct.pack("IhBB", 1000, -32767, 0x82, 1)) == (2, 1, -32767)`.
  - Left stick fully up (`{1: -32767}`), deadman held: `command(...)` is `("forward", 0.08, 0.0)` for `max_speed=0.08`; with `max_speed=0.5` v equals `P["max_speed"]`.
  - Deadman not held, any sticks: `("stop", 0.0, 0.0)`.
  - Stick inside the dead zone (`{1: -3000}`): stop. Half stick (`{1: -16384}`): `0 < v < 0.08`.
  - Stick fully down (`{1: 32767}`): `("reverse", 0.04, 0.0)`; the same with `{3: 32767}` pushed too: still `("reverse", 0.04, 0.0)`.
  - Right stick fully right alone (`{3: 32767}`): kind `"forward"`, `v == 0.0`, `w == -P["max_turn_rate"]` (right is a clockwise, negative turn).
  - Both sticks at full deflection: `v <= P["max_speed"]` and `abs(w) <= P["max_turn_rate"]`, and `v / P["max_speed"]` plus `abs(w) / P["max_turn_rate"]` scaled by the same factor (use `wheels.clamp_curvature`).
- [ ] **Step 2: Run** `python -m pytest tests/test_joystick_drive.py -q`. Expected: FAIL (module not found).
- [ ] **Step 3: Implement** `parse_event` and `command` in `tools/joystick_drive.py` (module header with `sys.path.insert(0, parent.parent)` like `back_up.py`). Stick fraction is `(abs(axis) / AXIS_FULL - DEAD_ZONE) / (1 - DEAD_ZONE)` clipped to `[0, 1]`; `v = min(max_speed, P["max_speed"]) * fraction`; `w = -turn_fraction_signed * p["max_turn_rate"]`; apply `wheels.least` then `wheels.clamp_curvature`; reverse returns `("reverse", REAR_SPEED, 0.0)`.
- [ ] **Step 4: Run** the same command. Expected: PASS.

### Task 2: `Pad`, the guarded `drive()` loop and `main()`

**Files:**
- Modify: `base/pi/tools/joystick_drive.py`
- Test: `base/pi/tests/test_joystick_drive.py`

**Interfaces:**
- Consumes: Task 1's `command`, `parse_event`, constants; `run_follow.scan_points(scan, p)`, `safety.must_stop(points, v, w, p)`, `safety.rear_clear(points, speed, p, distance)`, `lidar.LidarSession()` with `.latest()` and `.close()`, `base_check.first_full_scan(session, {"scans": []}, clock, sleep)`, `wheels.Wheels(p)` with `send`, `send_reverse`, `stop`, `close`, `wheels.least`.
- Produces `class Pad(path: str)` with `.axes: dict[int, int]`, `.buttons: dict[int, int]`, `.poll() -> None` (applies every event waiting, never blocks; raises `OSError` when the device is gone or reads empty) and `.close()`.
- Produces `blocked(kind: str, v: float, w: float, points, p) -> str | None`: `"no LiDAR points"` for no points, `"front blocked"` when `must_stop` is True for forward or turning, `"rear blocked"` when `not rear_clear(points, REAR_SPEED, p, REAR_LOOK)` for reverse, else None. Constants `REAR_LOOK = 0.30`, `PERIOD = 0.05`, `SECONDS = 300.0`, `SECONDS_MAX = 600.0`.
- Produces `drive(pad, w, session, p, max_speed, seconds, clock, sleep, log) -> dict`: loops every `PERIOD`; result `{"stop": <why>, "seconds": <float>}` with `stop` one of `"time"`, `"controller gone"`, `"lidar error"`. `w.stop()` in a `finally`; `log(str)` on each change of kind or block reason.
- Produces `main()`: `argparse` options `--max-speed` (default 0.08), `--seconds` (default 300), `--device` (default `/dev/input/js0`), `--dry`, `--yes-owner-at-robot`; prints one JSON line.

- [ ] **Step 1: Write the failing tests** (fakes as in `test_back_up.py`: a `Clock` with `sleep`, a `FakeWheels` that records `("send", v, w)`, `("reverse", speed, w)`, `"stop"`, a `FakeSession` whose `latest()` returns `room_scan` output, and a `FakePad` that applies one scripted `(axes, buttons)` state per `poll()` and raises `OSError` when the script ends in `"gone"`). Each is a test method in `DriveTest`:
  - `test_deadman_held_at_start_moves_nothing_until_released`: script holds `{5: 1}` with the stick up for 10 ticks, then releases for 2, then holds again: no `"send"` call before the first release, a `"send"` after it.
  - `test_forward_then_release_stops`: open room, stick up, then release: `("send", ...)` calls with `v == 0.08` then the last call is `"stop"`; the result stop is `"time"` after `seconds`.
  - `test_front_wall_stops_forward`: front wall `gap = 0.35` m (`room_scan(((2.0 - 0.20 - gap), 0.0, 0.0), half=2.0)`): no `"send"` call, log has `"front blocked"`.
  - `test_rear_wall_blocks_reverse`: wall 0.10 m behind the back edge (as `rear_wall_at` in `test_back_up.py`): no `"reverse"` call; open room: `("reverse", 0.04, 0.0)`.
  - `test_empty_scan_never_drives`: `session.latest()` returns `[]`: no `"send"`, log has `"no LiDAR points"`.
  - `test_controller_gone_stops`: script ends in `"gone"` mid-drive: result `stop == "controller gone"`, `"stop"` is the last call.
  - `test_held_stick_keeps_driving_without_new_events`: one scripted state repeated (`poll()` applies no new event): `"send"` calls continue every tick.
  - `MainTest`: `--yes-owner-at-robot` missing, or not under `safe_run`, or `--seconds 601`, or `--max-speed 0.2` each exit with `REFUSED` before any device opens; `--dry` with a fake `Pad` opens neither wheels nor LiDAR (an `opened` list stays empty) and prints the `command()` result when it changes.
- [ ] **Step 2: Run** `python -m pytest tests/test_joystick_drive.py -q`. Expected: the new tests FAIL (names missing).
- [ ] **Step 3: Implement** `Pad`, `blocked`, `drive`, `main`. `Pad.poll` reads with `select` at zero timeout and `os.read(fd, 8 * 32)`; `drive` follows `back_up.drive`: an `armed` flag set only once `command` has returned `"stop"` because the deadman was seen released; per tick `kind, v, w = command(...)`, `reason = blocked(...)` for a non-stop kind (a blocked kind becomes stop), send forward with `w.send(*wheels.least(v, w))`, reverse with `w.send_reverse(REAR_SPEED, 0.0)`, and `w.stop()` only when the previous tick was not already stop; the scan is `run_follow.scan_points(session.latest(), p)`; `main` follows `back_up.main` (open LiDAR, `first_full_scan`, then `Wheels(p)`; `finally` stops, closes wheels, session and pad). `--dry` opens only the `Pad`.
- [ ] **Step 4: Run** `python -m pytest tests/test_joystick_drive.py -q`. Expected: PASS.

### Task 3: The console runs it as a check

**Files:**
- Modify: `base/pi/console.py:167-174` (the `TOOL_CHECKS` dict)
- Test: `base/pi/tests/test_console_control.py` (`CheckArgsTest.test_allowed`, `test_refused`, and `test_checks_by_name_from_a_program_and_from_the_page`)

**Interfaces:**
- Consumes: `console.check_command(name, py, args, max_speed, page)`.
- Produces: `TOOL_CHECKS["joystick_drive"] = (True, [], {"--max-speed": "speed", "--seconds": "number", "--yes-owner-at-robot": "flag"})`.

- [ ] **Step 1: Write the failing tests:** allowed: `check("joystick_drive", ["--max-speed", "0.08", "--seconds", "120", "--yes-owner-at-robot"], page=True)` is not None and its command runs `job_guard.py`, `safe_run.py`, `joystick_drive.py` in that order with `moves` True; refused (`ValueError`): `["--dry"]`, `["--device", "/dev/input/js1"]`, `["--max-speed", "0.3"]`, `["--seconds", "1e9"]`, and `["--yes-owner-at-robot"]` with `page=False`.
- [ ] **Step 2: Run** `python -m pytest tests/test_console_control.py -q`. Expected: FAIL (`joystick_drive` unknown, returns None).
- [ ] **Step 3: Add the `TOOL_CHECKS` entry** and a one-line comment saying `--dry` and `--device` stay out on purpose.
- [ ] **Step 4: Run** `python -m pytest tests/test_console_control.py tests/test_console.py tests/test_joystick_drive.py -q`. Expected: all PASS.

### Task 4: The Checks-tab panel and the docs

**Files:**
- Modify: `base/pi/console/index.html:320` (a new `.panel` after "ตรวจมุมกล้อง"), `base/pi/console/app.js:1113-1120` (the `[data-check]` click handler), `base/docs/robot-console.md` (Checks bullet, line 30, and the Overview of tools)

- [ ] **Step 1: Add the panel** with `<button type="button" class="btn big" data-check="joystick_drive">ขับด้วยจอย</button>` and a short Thai text: hold R1 to move, left stick forward and back, right stick turns, the LiDAR stops the base, the operator watches the power cable.
- [ ] **Step 2: In the click handler**, for `check === "joystick_drive"` ask `confirmBox("ขับด้วยจอยไหม", "หุ่นจะขยับตามจอย มีคนอยู่ข้างหุ่น พื้นที่โล่ง และสายไฟปลอดภัยแล้วใช่ไหม", "ใช่ เริ่มเลย", true)` and post `{ check: "joystick_drive", args: ["--max-speed", "0.08", "--yes-owner-at-robot"] }`.
- [ ] **Step 3: Document** the check in `robot-console.md` (what it does, the deadman, the limits, that `robotctl check joystick_drive` needs the page for the presence flag).
- [ ] **Step 4: Verify** with `python -m pytest tests/test_console.py tests/test_console_control.py -q` (the static files are served by existing tests), then load the page after Task 5's deploy and check the panel and the confirm box render; do not press Start without the owner.

### Task 5: Deploy and the live test (owner at the robot)

**Files:** none (needs a commit).

- [ ] **Step 1: Ask the owner for a commit** of the joystick change (`deploy.sh` deploys a commit). After a yes: commit only the files of Tasks 1 to 4.
- [ ] **Step 2: Deploy** `base/tools/deploy.sh --dry-run`, then `base/tools/deploy.sh`; the script asks the console for a hold first and refuses while a job runs. The console reads page files on each request, but `console.py` itself needs `--restart` to pick up `TOOL_CHECKS`.
- [ ] **Step 3: Dry run with the owner pressing buttons:** `ssh xohm@192.168.31.21 "cd ~/hmc-base && .venv/bin/python tools/joystick_drive.py --dry"`; confirm R1, the left stick up and down, the right stick right, and fix the constants if any number differs.
- [ ] **Step 4: Check the Pi before driving:** power bank estimate above 30 % on `/api/bank`, a fresh wheel test, the power cable clear.
- [ ] **Step 5: Drive from the page** at `--max-speed 0.05` for a few seconds with the base clear (right stick right must turn the base clockwise, else flip the sign in `command` and re-test), then 0.08 and 0.12 on open floor: forward, turn, backward, release R1, unplug the controller mid-drive. Expected: every release or unplug stops the base within the firmware's 0.3 s.
