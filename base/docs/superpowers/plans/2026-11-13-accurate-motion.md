# Accurate Motion Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** The real base follows the operator through 0.65 m gaps without touching anything, drives what the brain asks, and knows its pose to 0.10 m, with the base's numbers measured while it drives.

**Architecture:** Group B changes the shared brain (`hmc/`) and the Pi's last-resort stop to use the robot's real outline and the measured base limits. Group A measures and calibrates the base, then replaces the firmware and adds an IMU. Group C runs slam_toolbox on the Jetson, fed over TCP by the Pi, and sends the map correction back.

**Tech Stack:** Python 3 standard library in `hmc/` (pytest); Python 3.13 + numpy + pyserial on the Pi (`unittest`); ROS2 Jazzy, rclpy, slam_toolbox on the Jetson.

**Spec:** `base/docs/superpowers/specs/2026-11-13-accurate-motion-design.md`

## Global Constraints

- Branch `sim/robustness-ladder`. Commit locally, never push or open a PR without asking.
- `hmc/` stays standard library only, ROS-free, simulation only (`hmc/AGENTS.md`). Before calling an `hmc` change done: `pytest`, `python -m hmc.run` ends in `done`, `python -m hmc.ladder --seeds 10 --speed 0.5` scores 1115 on every map, and `hmc/docs/evidence/help-me-carry-slice2/` plus its `SHA256SUMS` are regenerated.
- Every drive goes through `tools/safe_run.py`; the drive tools refuse otherwise. Announce each run in one line; report danger at once.
- Changes to `safety.py`, `wheels.py` or `run_follow.py` get one Fable review before the next drive; changes to the senses identity rules get one Fable review before they are deployed.
- Measured values (hardware assessment): footprint 0.40 m square with 0.04 m corners cut (`world.FOOTPRINT`, circumscribed radius 0.256 m), inscribed half-width 0.20 m, decel 0.064 m/s^2, stop delay up to 0.32 s, top speed 0.12 m/s, turn rate 0.49 rad/s, odometry step 0.01 m.
- No number is reported without its source (measured today, read from config, not measured).

## Review Focus

1. The operator walks through a gap that fits the body only when it drives straight: the robot must not turn inside the gap (test in Task 1).
2. A point appears inside the swept outline after the command was sent (a person steps in): `must_stop` must catch it at the next cycle (test in Task 2).
3. A chair leg the robot has stood next to for minutes, then a stranger stops beside it: the stranger must still count as a person once they moved (test in Task 6).
4. The Jetson link drops mid-route: the Pi keeps the last `map -> odom` correction and keeps driving on odometry, never jumps the pose (test in Task 10).
5. The SLAM correction jumps by more than 0.5 m or 20 degrees in one message: it is refused and logged (test in Task 10).

---

## File structure

```
hmc/src/hmc/navigation.py        modify  plan_path: inscribed inflation + outline check per segment and corner
hmc/src/hmc/world.py             modify  INSCRIBED, MAX_DECEL, STOP_DELAY, HEADING_DEADBAND (defaults keep the sim)
hmc/src/hmc/run.py               modify  BASES["real"] gains decel, stop_delay, deadband
hmc/tests/test_footprint_planning.py  new
base/pi/footprint.py             new     outline gap and swept-outline check (numpy)
base/pi/safety.py                modify  must_stop sweeps the outline
base/pi/body.py                  modify  sets the new world constants from params
base/pi/senses.py                modify  static candidates are not people
base/pi/run_follow.py            modify  pose prediction for latency, search turning
base/pi/motion.py                new     fused motion from wheel odometry and LiDAR scan matching
base/pi/calib.py                 new     online estimates of gains, drift, latency, braking, with bounds
base/pi/velocity.py              new     velocity loop on the Pi
base/pi/slam_link.py             new     Pi side of the SLAM stream (port 5602)
base/pi/tools/lap_check.py       new     wall-following lap for gap and map runs (from the scratch lap.py)
base/pi/tools/latency.py         new     command-to-motion latency from base_check logs
base/jetson/slam_bridge.py       new     TCP <-> ROS2 bridge for slam_toolbox
base/jetson/slam/params.yaml     new     slam_toolbox settings
base/params.json                 modify  robot_radius 0.26; command_latency as a starting value
```

---

## Order of work (owner, 2026-11-13: bottom layer first, values measured while driving)

M1, M2, M3 first (layers 2 and 3 of the spec), then Task 2 (safety), Task 1, 3, 4, 5, 6, 7, 8.
Firmware (Task 10) and the SLAM stream (Tasks 11 and 12) run in parallel sessions; they do
not touch the files of M1 to Task 8.

## Layers 2 and 3: measured motion

### Task M1: Motion estimator, wheel odometry plus LiDAR scan matching

**Files:** Create `base/pi/motion.py`; Test `base/pi/tests/test_motion.py`.

**Interfaces:**
- Consumes: `scanmatch.match(before, after, p, guess) -> (dx, dy, dyaw, agree)`; `wheels.Wheels.read_odom() -> (x, y, yaw)`.
- Produces: `MotionEstimator(p, match=scanmatch.match)`; `update(t, odom_pose, scan, command) -> MotionState`; `MotionState(t, pose, v, w, trusted: bool, agree: float)` where `pose` is the fused pose in the odometry frame and `v, w` the measured speeds over the last revolution.

- [ ] **Step 1: Failing tests:** with a fake `match` that reports 0.012 m per revolution while odometry reports 0.010 m, the fused step follows the scan match (within 1 mm) and `trusted` is True; with `agree` 0.4 the step follows odometry and `trusted` is False; a scan-match step that disagrees with odometry by more than 0.05 m or 5 degrees is refused (odometry used, counted in `refused`); timestamps that go backward raise.
- [ ] **Step 2: Run** `PYTHONPATH="hmc/src;base/pi" python -m unittest base.pi.tests.test_motion` — FAIL.
- [ ] **Step 3: Implement.** Odometry delta as the match guess; downsample each scan to at most 150 points before matching (time budget below).
- [ ] **Step 4: Measure on the Pi:** match time per revolution pair, median and worst of 200 from tonight's saved scans. Must stay under 40 ms median, 80 ms worst; if not, lower the point count and say what was lost.
- [ ] **Step 5:** Replay the three follow logs and the two lap logs of 2026-11-13 through the estimator; report the fused path against odometry (end-point difference) and the share of trusted revolutions. Commit.

### Task M2: Online estimates of the base, with bounds

**Files:** Create `base/pi/calib.py`; Test `base/pi/tests/test_calib.py`.

**Interfaces:**
- Produces: `BaseEstimates(p)`; `observe(state: MotionState, command: (v, w), sent_at)`; read-only properties `speed_gain`, `turn_gain`, `drift_per_m`, `latency`, `decel`, `stop_delay`, each `(value, samples, trusted)`; `safe_braking()` returns the larger of the estimate and the `params.json` floor.

- [ ] **Step 1: Failing tests:** a synthetic run where the base drives 0.8 times the command converges to `speed_gain` 0.80 within 0.03 after 30 s of samples; a stop event (command to 0, speed decays) gives `decel` and `stop_delay` within 10 % of the synthetic truth; a lag of 0.4 s between command and measured turn rate gives `latency` 0.40 within 0.1; `safe_braking()` never returns less than the floor; with fewer than the minimum samples every estimate reports `trusted` False and the `params.json` value.
- [ ] **Step 2: Run, FAIL. Step 3: Implement** (exponential averages with a sample count; latency by the lag of best correlation over a 10 s window; bounds: gains 0.5 to 1.5, latency 0 to 1.0 s, decel 0.02 to 1.0 m/s^2).
- [ ] **Step 4:** Replay tonight's logs; report each estimate. Compare with the one-off measurements in the hardware assessment. Commit.

### Task M3: Velocity loop on the Pi

**Files:** Create `base/pi/velocity.py`; Modify `base/pi/run_follow.py` (wiring only); Test `base/pi/tests/test_velocity.py`.

**Interfaces:**
- Produces: `VelocityLoop(p, estimates)`; `command(v_wanted, w_wanted, state) -> (v_send, w_send)`: feed-forward divides by the trusted gains, a PI term on the measured error, a turn bias for the estimated drift; clamps to `max_speed` and `max_turn_rate`; no integral wind-up while the base is at its cap (measured speed flat while the command rises).

- [ ] **Step 1: Failing tests:** against a simulated base with gain 0.8 and a 0.12 m/s cap, a wanted 0.10 m/s settles within 5 % in 3 s; a wanted 0.20 m/s never sends above `max_speed` and the integral stays bounded; a drift of 2 cm per metre is turned out within 0.5 cm per metre; `v_wanted = 0` sends exactly 0 (the stop path is never delayed by the loop).
- [ ] **Step 2: Run, FAIL. Step 3: Implement.** `run_follow.cycle` passes the brain's `(v, w)` through `VelocityLoop.command` after `must_stop`, so a stop still sends 0 at once.
- [ ] **Step 4:** Fable review of `velocity.py` and the `run_follow.py` wiring. Then on the robot: `lap_check` (Task 8 tool) one lap with the loop; pass when the driven speed is within 10 % of the wanted speed in steady 1 s windows and straight segments drift under 0.5 cm per metre. Commit.

## Group B: the brain

### Task 1: Plan with the outline, not a 0.29 m circle

**Files:** Modify `hmc/src/hmc/navigation.py`, `hmc/src/hmc/world.py`; Test `hmc/tests/test_footprint_planning.py`.

**Interfaces:**
- Produces: `world.INSCRIBED = 0.20`; `navigation.outline_gap(pos, heading, shapes) -> float` (smallest `shape.clearance` over `world.outline(*pos, heading)`); `navigation.path_clear(path, start, start_heading, shapes, margin) -> int | None` (index of the first segment or corner that fails, else None); `plan_path(start, goal, shapes, near_side=False, heading=None)` unchanged return type.

- [ ] **Step 1: Write the failing tests**

```python
def test_a_0_65_m_gap_is_planned_through():        # two walls of Circle(r=0.05) points, gap 0.65 m between surfaces
    assert plan_path((0, 0), (3, 0), walls(gap=0.65)) is not None
def test_a_0_55_m_gap_is_not():
    assert plan_path((0, 0), (3, 0), walls(gap=0.55)) is None
def test_no_corner_turn_inside_a_tight_gap():       # an L-shaped 0.65 m corridor
    path = plan_path((0, 0), (2, 2), l_corridor(gap=0.65), heading=0.0)
    assert path is not None and path_clear(path, (0, 0), 0.0, l_corridor(gap=0.65), SAFETY) is None
```

- [ ] **Step 2: Run** `cd hmc && pytest tests/test_footprint_planning.py -v` — expected FAIL (0.65 m gap gives None today).
- [ ] **Step 3: Implement.** Inflate by `INSCRIBED + SAFETY`; after A*, `path_clear` checks the outline at every 0.05 m along each segment facing the segment, and the outline swept from the incoming to the outgoing heading at each corner (5 degree steps) with `margin = SAFETY`; block the cells of a failing segment's middle and plan again, at most 5 rounds, then return None. `Navigator.step` passes `robot.heading` as `heading`. `free_ahead` keeps `ROBOT_RADIUS`.
- [ ] **Step 4: Run** the new tests, then the whole `pytest`, `python -m hmc.run`, the ladder (1115 on every map at 0.5 m/s). If a map loses points, report the map and the failing scenario before changing anything else.
- [ ] **Step 5:** Set `robot_radius` to 0.26 in `base/params.json` (the real outline's circumscribed radius). Regenerate the evidence folder and `SHA256SUMS`. Commit.

### Task 2: The last-resort stop sweeps the outline

**Files:** Create `base/pi/footprint.py`; Modify `base/pi/safety.py`; Test `base/pi/tests/test_footprint.py`, `base/pi/tests/test_safety.py`.

**Interfaces:**
- Consumes: `hmc.world.FOOTPRINT`.
- Produces: `footprint.body_gap(points, x, y, th) -> float` (distance from each point to the octagon, negative inside; numpy); `footprint.sweep_clear(points, v, w, seconds, dt=0.05, margin=0.03) -> bool`; `safety.must_stop(points, v, w, p)` unchanged signature.

- [ ] **Step 1: Write the failing tests**

```python
def test_turning_away_from_a_wall_3_cm_away_is_allowed():   # wall points at y = -0.23, x -0.4..0.6
    assert not must_stop(wall, 0.0, +0.3, P)
def test_turning_into_it_is_stopped():
    assert must_stop(wall, 0.0, -0.3, P)
def test_a_point_entering_the_swept_outline_stops():
    assert must_stop([(0.35, 0.10)], 0.10, 0.4, P)
def test_body_gap_of_a_point_at_the_cut_corner():
    assert abs(body_gap(np.array([[0.25, 0.25]]), 0, 0, 0) - math.dist((0.25, 0.25), (0.18, 0.18))) < 0.01
```
and keep every existing `test_safety` assertion (straight-ahead clearance 0.30 m, NaN means stop, backward raises).

- [ ] **Step 2: Run** `PYTHONPATH="hmc/src;base/pi" python -m unittest base.pi.tests.test_footprint base.pi.tests.test_safety` — FAIL.
- [ ] **Step 3: Implement.** `must_stop`: NaN rule and backward rule first; then the forward clearance strip as today; then `not sweep_clear(points, v, w, run_on)` with `run_on = stop_delay + max(v / decel, abs(w) / turn_accel)`. Remove `SPIN_RADIUS`.
- [ ] **Step 4: Run** the full Pi suite. Expected OK.
- [ ] **Step 5:** Fable review of `safety.py`, `footprint.py` (Global Constraints). Fix what it confirms. Commit.

### Task 3: Brake like the real base

**Files:** Modify `hmc/src/hmc/world.py`, `hmc/src/hmc/run.py`, `base/pi/body.py`; Test `hmc/tests/test_world.py` (add), `base/pi/tests/test_body.py` (add).

**Interfaces:**
- Produces: `world.MAX_DECEL: float | None = None` (None means `MAX_ACCEL`), `world.STOP_DELAY = 0.0`; `braking_speed(room, dt)` uses them: the speed from which the robot stops within `room` after `STOP_DELAY`.

On the robot `Body` sets `MAX_DECEL` and `STOP_DELAY` every cycle from `BaseEstimates.safe_braking()` (M2), so the brain brakes with what the base did today, never better than the floor.

- [ ] **Step 1: Failing tests:** `test_braking_speed_with_the_real_base()` — with `MAX_DECEL = 0.064`, `STOP_DELAY = 0.27`, `braking_speed(0.30, 0.1)` is at most 0.12 and `v * 0.27 + v * v / (2 * 0.064) <= 0.30`; `test_body_sets_decel_and_delay()` — after `Body(P)`, `world.MAX_DECEL == P["decel"]`.
- [ ] **Step 2: Run, expect FAIL. Step 3: Implement** (`Robot.brake` uses `MAX_DECEL`; `BASES["real"]` gains `decel=0.064, stop_delay=0.27`; `use_base` sets them).
- [ ] **Step 4: Run** `pytest`, `python -m hmc.run`, the ladder. Regenerate evidence. Commit.

### Task 4: No turning back and forth while standing

**Files:** Modify `hmc/src/hmc/world.py`, `base/pi/body.py`, `hmc/src/hmc/run.py`; Test `hmc/tests/test_world.py`.

- [ ] **Step 1: Failing test:** `test_no_turn_inside_the_deadband()` — with `world.HEADING_DEADBAND = math.radians(4)`, a standing robot whose target is 3 degrees off returns from `turn_toward` with `turn_speed == 0` and heading unchanged; at 6 degrees it turns.
- [ ] **Step 2: Run, FAIL. Step 3: Implement** in `Robot._turn` (only when `speed == 0`); default 0 keeps the sim; `Body` and `BASES["real"]` set 4 degrees.
- [ ] **Step 4: Run** tests, sim, ladder; regenerate evidence. Commit.

### Task 5: Command latency, measured, then predicted

**Files:** Create `base/pi/tools/latency.py`; Modify `base/pi/run_follow.py`, `base/params.json`; Test `base/pi/tests/test_latency.py`.

**Interfaces:**
- Produces: `latency.first_motion(log) -> float` (seconds from `drive_start` to the first odometry or scan-matched yaw change over 1 degree, from a `base_check` JSON log); `run_follow.predict(pose, v, w, seconds) -> pose`; param `command_latency`.

- [ ] **Step 1: Failing tests:** `first_motion` on a synthetic log that starts moving 0.30 s after `drive_start` returns 0.30 within 0.05; `predict((0,0,0), 0.1, 0.0, 0.5) == (0.05, 0, 0)`.
- [ ] **Step 2: Run, FAIL. Step 3: Implement.**
- [ ] **Step 4: Measure:** run `latency.py` on tonight's 13 turn and line logs; report median and worst, and compare with `BaseEstimates.latency` (M2). `command_latency` in `params.json` becomes the starting value only.
- [ ] **Step 5:** Call `predict` on the pose before `body.sync` in `run_follow.cycle` with `BaseEstimates.latency` (the `params.json` value until it is trusted) and the commands sent in that window. Test with the fake base. Fable review of `run_follow.py` with Task 7. Commit.

### Task 6: Furniture is not a person

**Files:** Modify `base/pi/senses.py`; Test `base/pi/tests/test_senses.py`.

**Interfaces:**
- Produces: `RealSenses._is_person(pid, now) -> bool` (moved 0.25 m from `_first_seen[pid]`, or a camera person within `MATCH_DEG` in the last 2 s). `people` in `Senses`, `_crowded`, `_mixed_up` use only persons.

- [ ] **Step 1: Failing tests:** `test_walking_past_a_chair_leg_keeps_the_track()` (static candidate 0.3 m beside the operator's path, link down, lock held in every cycle, `ambiguous` never set); `test_a_still_stranger_seen_by_the_camera_is_a_person()`; `test_a_stranger_who_stops_next_to_a_chair_still_counts()` (Review Focus 3). All `Merge` tests keep passing.
- [ ] **Step 2: Run, FAIL. Step 3: Implement. Step 4: Run** the suite, then replay the three follow logs of 2026-11-13 through `RealSenses` from their saved scans and report `ambiguous` cycles before and after.
- [ ] **Step 5:** Fable review. Commit.

### Task 7: Search by turning, inside the follow loop

**Files:** Modify `base/pi/run_follow.py`; Test `base/pi/tests/test_run_follow.py`.

- [ ] **Step 1: Failing tests:** with no lock and no camera person, `cycle` sends `w = +0.3` (or toward the side the operator was last seen) and `v = 0`; with a camera person at bearing 5 degrees it sends `w = 0` and calls `enrol`; a LiDAR point inside the swept outline still stops the turn (Task 2's `must_stop`).
- [ ] **Step 2: Run, FAIL. Step 3: Implement** (replaces the scratch `search_turn.py`). **Step 4: Run** the suite.
- [ ] **Step 5:** Fable review of `run_follow.py` (with Task 5). Commit and deploy.

### Task 8: Field check of group B

**Files:** Create `base/pi/tools/lap_check.py` (from the scratch `lap.py`, using `footprint.sweep_clear` for its escapes); Test `base/pi/tests/test_lap_check.py`.

- [ ] **Step 1:** Tests for `lap_check.control` and `choose` on the four cases of the scratch dry check (wall ahead, corridor, open, wall 3 cm right).
- [ ] **Step 2:** `lap_check` one lap: report the narrowest gap passed and the closest body gap (now logged with `body_gap`, not the side strips).
- [ ] **Step 3:** One follow run with the owner walking through the room's gaps. Pass: path found in at least 95 % of following cycles with the operator over 1 m away; locked at least 95 %; closest body gap at least 0.03 m; no hold longer than 2 s; no turning while standing inside the deadband; driven speed within 10 % of the command in steady 1 s windows (spec goal 2; if only new firmware can reach it, say so and move it to Task 10). Record in the hardware assessment. Commit.

## Group A: the base

### Task 9: Check the drift the velocity loop leaves

The fixed `drift_w_per_v` of the first draft is replaced by M2's online drift estimate and
M3's turn bias. This task only checks it: on the open lane, `base_check --line 0.12
--metres 2.5` out, turn 180, back, three times each way, with the loop on. Pass when drift
is under 0.5 cm per metre in every run; record the numbers in the hardware assessment.

### Task 10: New firmware and an IMU (owner hardware)

- [ ] **Step 1 (owner):** open the base, photograph the controller board (both sides, the MCU marking, any SWD or BOOT0 header) and the motor labels; measure the motor supply at the driver at `F015` and `F037` with the wheels raised.
- [ ] **Step 2:** From the photos, finish `base/docs/firmware-design.md` section 0; get the owner's approval of the design; buy an ST-Link if the board has SWD.
- [ ] **Step 3:** Back up the old firmware (design section 1) before any write. Then the firmware work runs as its own plan.
- [ ] **Step 4 (owner):** choose and buy an IMU for the Pi's I2C; its driver and an EKF with the wheel odometry run as their own plan.

## Group C: where the robot is

### Task 11: The SLAM stream, Pi and Jetson sides

**Files:** Create `base/pi/slam_link.py`, `base/jetson/slam_bridge.py`; Test `base/pi/tests/test_slam_link.py`, `base/jetson/tests/test_slam_bridge.py`.

**Interfaces:**
- Produces: wire format, both directions length-prefixed JSON like the camera link: Pi to Jetson `{"t", "pose": [x, y, th], "scan": [[deg, m], ...]}` per revolution; Jetson to Pi `{"t", "map_to_odom": [x, y, th]}`. `SlamLink(host, port=5602).send(t, pose, scan)`, `SlamLink.correction() -> (x, y, th)` (the last accepted, identity until the first); `slam_bridge.to_messages(record) -> (LaserScan fields, Odometry fields)` testable without ROS.

- [ ] **Step 1: Failing tests:** round trip of one record; a correction that jumps more than 0.5 m or 20 degrees from the last one is refused (Review Focus 5); with the link down `correction()` keeps the last value and never raises (Review Focus 4).
- [ ] **Step 2: Run, FAIL. Step 3: Implement** (bridge stamps messages with the Jetson's receive time; static `base_link -> laser` at x 0.175; publishes `/scan`, `/odom`, the `odom -> base_link` transform; looks up `map -> odom` at 2 Hz and sends it back; subscribes `/map` and writes `logs/map-<time>.png` every 10 s).
- [ ] **Step 4: Run** both suites locally (ROS parts skipped without rclpy). Commit.

### Task 12: slam_toolbox on the Jetson

**Files:** Create `base/jetson/slam/params.yaml` (online async, resolution 0.05, max laser range 8.0, minimum travel 0.10 m and 0.10 rad, loop closure on).

- [ ] **Step 1 (owner):** `sudo apt install ros-jazzy-slam-toolbox` on the Jetson; tell the other teams.
- [ ] **Step 2:** Start the bridge and slam_toolbox; with the robot still, check `/scan` and `/odom` rates (at least 9 Hz) and `map -> odom` coming back.
- [ ] **Step 3:** `lap_check` one lap with `slam_link` on. Pass: the map PNG has single, sharp walls; back at the start the corrected pose is within 0.10 m of the owner's tape mark.
- [ ] **Step 4:** Measure the Jetson RAM with the detector, SLAM, voice and arm running. If it does not fit, report the numbers and fall back to the return-trip keyframes.
- [ ] **Step 5:** `run_follow` logs the corrected pose each cycle. Record results in the hardware assessment. Commit.

### Task 13: Re-measure

- [ ] **Step 1:** Speed and turn ladders again (`base_check`), the follow criteria of `person-follow-design.md` that fit the room (1, 2, 3, 5, 6, 8), five runs each.
- [ ] **Step 2:** Update `params.json`, `run.BASES["real"]` and the hardware assessment with the new numbers; rerun the ladder in the simulation with them. Commit.
