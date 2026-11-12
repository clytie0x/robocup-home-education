# Person Following Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** The real robot follows one operator through the arena and outside it, without a break, without touching anything, and without ever following someone else, using the same decision code as the `hmc` simulation.

**Architecture:** The decision code (operator tracking by look, A* planning, search) is extracted from `hmc/src/hmc/task.py` into a pure `Follower` that takes a body and a `Senses` record. The simulation fills `Senses` from its world; the robot fills it on the Pi from the LiDAR leg tracker, the depth image and the Jetson's person detections with look vectors. The Pi owns the wheels and a last-resort stop and keeps following on the LiDAR track when the Jetson is unreachable.

**Tech Stack:** Python 3.10+ standard library in `hmc/`; Python 3.13, numpy, OpenCV, pyserial, OpenNI2 on the Pi (`unittest`, no pytest there); TensorRT 10.16 (apt), numpy, OpenCV on the Jetson; plain TCP between them.

**Spec:** `base/docs/superpowers/specs/2026-11-13-person-follow-design.md`
**Hardware facts:** `base/docs/hardware-assessment-2026-11-13.md`

## Global Constraints

- Repository root is `C:\Users\teera\dev\robocup-home-education`. Work on a new branch `feature/person-follow` cut from `feature/help-me-carry-slice2`. Commit locally only. Do not push, merge, force-push or delete files or branches.
- All repository content is in English. Reply to the owner in Thai.
- `hmc/` stays simulation only and standard library only (`hmc/AGENTS.md`). No serial, socket, numpy or hardware code there. Hardware code goes in `base/`.
- Do not touch `voice/`, `arm/`, or on the Jetson anything outside `~/hmc-follow`. Do not stop, restart or reconfigure other teams' processes. Do not reboot either machine or change the Jetson power mode.
- On the Pi, change nothing outside `~/hmc-base`. `~/pi-drive`, `~/catkin_ws` and `~/robot-deploy-kit` are read-only sources. Stop a ROS session with `~/robot-pi stop` before opening the serial ports, because both use `/dev/rct_ahr_robot` and `/dev/rct_ahr_lidar`.
- **The wheels may turn only when the owner has said in chat, for that run, that they are at the robot.** Before every driving run: state the direction, distance and top speed, read the LiDAR in the base frame for the whole swept area, and wait for the owner's answer. Never confirm a pose, a position or a clearance on the owner's behalf.
- Top speed is 0.15 m/s until Task 2 has filled `linear_cmd_scale`, `decel` and `stop_delay` in `base/params.json`. The depth obstacle layer is off until Task 3 has filled `camera_x`, `camera_height`, `camera_pitch_deg` and `robot_height`.
- The follower never drives backward.
- Values fixed by the spec: follow gap 0.6 m from the robot's front edge; robot planned as a circle of radius 0.29 m; depth obstacles from 5 cm above the floor up to the robot's height, inside 3 m, remembered 10 s; Jetson silence limit 0.5 s; braking distance = v²/(2·decel) + v·stop_delay + 0.15 m; speed ladder 0.3, 0.4, 0.5, 0.6 m/s with pass limits of 0.4 m braking and 5 cm drift per metre; enrolment 20 frames, one person inside 2.5 m; detector at least 15 frames per second on the Jetson under the other teams' load.
- Report only what was run. Paste the real test summary line and the real measured numbers. A gate that could not be measured is reported as not measured.

## Review Focus

1. **The operator stands still 0.6 m in front of the robot for a long time.** The camera sees only legs there, so no look arrives. Expected: the robot holds and stays locked on the unbroken LiDAR track. Test in Task 9.
2. **A second person stands shoulder to shoulder with the operator, then walks away.** The LiDAR sees one merged object, then two. Expected: the track stays on the operator or the robot stops; it never leaves with the other person. Test in Task 9.
3. **Wi-Fi stalls and a camera answer arrives one second late.** Expected: the stale answer is discarded, not matched against the present scan. Test in Task 8.
4. **Sunlight blinds the depth image (all zeros).** Expected: the top speed drops to the LiDAR-only limit; the frame is never read as "nothing there". Test in Task 7.
5. **The firmware returns a broken odometry line, or none.** Expected: `S#` is sent and the run ends with a clear message; the wheels never keep the last command. Test in Task 10.

---

## File structure

```
hmc/src/hmc/follower.py          new   Senses, Follower: who the operator is, follow, search
hmc/src/hmc/task.py              edit  FOLLOW and SEARCH delegate to Follower
hmc/src/hmc/navigation.py        edit  robot radius read at call time
hmc/src/hmc/appearance.py        edit  LookMemory sized by the first look
hmc/tests/test_follower.py       new

base/README.md                   new   purpose, entry points, lifecycle state
base/params.json                 new   every measured value; null means "gate not passed"
base/tools/deploy.sh             new   copies hmc/src/hmc and base/pi to the Pi, base/jetson to the Jetson
base/pi/params.py                new   loads params.json, says which gates are open
base/pi/lidar.py                 copy  from Pi ~/pi-drive/lidar.py, with its fixtures and test
base/pi/wheels.py                new   serial commands and odometry in true units
base/pi/camera.py                new   colour and depth frames in a thread
base/pi/geometry.py              new   lidar frame, base frame, odometry frame
base/pi/legs.py                  copy  from Pi ~/pi-drive/legs.py (people in the scan, LegTracker)
base/pi/depth_obstacles.py       new   depth image to obstacle points
base/pi/shapes.py                new   obstacle circles for the planner, with memory
base/pi/safety.py                new   braking distance and the last-resort stop
base/pi/link.py                  new   TCP client to the Jetson, newest frame only
base/pi/senses.py                new   fuses LiDAR track and camera people into hmc Senses
base/pi/run_follow.py            new   the control loop and its log
base/pi/replay.py                new   reruns a log through the brain
base/pi/tools/base_check.py      new   speed ladder (Task 2)
base/pi/tools/sensor_check.py    new   camera pose, LiDAR yaw (Task 3)
base/pi/tests/                   new   unittest files, one per module
base/jetson/detector.py          new   people in an image (TensorRT)
base/jetson/reid.py              new   look vector per person (TensorRT)
base/jetson/server.py            new   TCP service
base/jetson/bench.py             new   frames per second under load
base/jetson/tests/               new
base/docs/field-results.md       new   the scenario table (Tasks 13 to 15)
```

Tests in `base/` use `unittest` so they run on the laptop and on the Pi:
`PYTHONPATH=hmc/src:base/pi python -m unittest discover -s base/pi/tests -v`.

---

### Task 0: Branch, pending simulation work, and the `base/` folder

**Files:**
- Commit as they are: `hmc/src/hmc/run.py`, `hmc/src/hmc/odometry.py`, `hmc/src/hmc/robustness.py`, `hmc/tests/test_speed_and_pace.py` (uncommitted `--base real` and the `calibrated` odometry preset from 2026-11-13).
- Create: `base/README.md`, `base/params.json`, `base/pi/params.py`, `base/pi/tests/test_params.py`
- Modify: `README.md` (root table gets a `base/` row, owner Teerapat), `C:/Users/teera/dev/README.md` if it lists projects (register `base/` as part of `robocup-home-education`).

**Interfaces:**
- Produces: `params.load(path=None) -> dict`, `params.speed_gates_open(p) -> bool` (true when `linear_cmd_scale`, `decel`, `stop_delay` are not null), `params.depth_gates_open(p) -> bool` (true when `camera_x`, `camera_height`, `camera_pitch_deg`, `robot_height` are not null).

- [x] **Step 1:** `git switch -c feature/person-follow`. Run `cd hmc && pytest -q`. Expected: `109 passed, 2 xfailed, 1 xpassed`. Commit the four pending `hmc` files with message `hmc: real-base limits (--base real) and calibrated odometry preset`. Leave the untracked `pip-*` folders alone.
- [x] **Step 2:** Write `base/params.json` with exactly these keys and values:

```json
{
  "linear_cmd_scale": null,
  "angular_cmd_scale": 0.862,
  "odom_distance_scale": 0.902,
  "odom_yaw_scale": 0.862,
  "max_speed": 0.15,
  "max_turn_rate": 0.49,
  "accel": 0.25,
  "turn_accel": 0.7,
  "decel": null,
  "stop_delay": null,
  "robot_radius": 0.29,
  "robot_half_width": 0.20,
  "robot_height": null,
  "lidar_x": 0.175,
  "lidar_front_deg": 178.7,
  "camera_x": null,
  "camera_height": null,
  "camera_pitch_deg": null,
  "camera_hfov_deg": 58.59,
  "follow_gap": 0.6,
  "jetson_host": "192.168.31.137",
  "jetson_port": 5601,
  "jetson_silence": 0.5
}
```

- [x] **Step 3:** Write the failing test `base/pi/tests/test_params.py`:

```python
class Gates(unittest.TestCase):
    def test_gates_are_closed_in_the_committed_file(self):
        p = params.load()
        self.assertFalse(params.speed_gates_open(p))
        self.assertFalse(params.depth_gates_open(p))

    def test_speed_gate_opens_when_the_three_values_are_measured(self):
        p = dict(params.load(), linear_cmd_scale=0.9, decel=0.5, stop_delay=0.2)
        self.assertTrue(params.speed_gates_open(p))
```

- [x] **Step 4:** Run it, see it fail, implement `base/pi/params.py`, run it, see it pass.
- [x] **Step 5:** Write `base/README.md` (purpose: wheeled base and person following on the real robot; entry point `base/pi/run_follow.py`; lifecycle state `active`; the two docs). Add the root `README.md` row. Commit.

### Task 1: Drivers on the Pi under version control, and deployment

**Files:**
- Create: `base/pi/lidar.py`, `base/pi/tests/test_lidar.py`, `base/pi/tests/fixtures/` (copied from Pi `~/pi-drive`, unchanged, with a first line comment naming the source and date), `base/pi/wheels.py`, `base/pi/tests/test_wheels.py`, `base/pi/geometry.py`, `base/pi/tests/test_geometry.py`, `base/tools/deploy.sh`

**Interfaces:**
- Consumes: `params.load()`.
- Produces:
  - `lidar.LidarSession().latest() -> list[tuple[float, float]]` (degrees in the lidar frame, 0 = front, positive left; metres), `.stream.updated_at`, `.close()`; `lidar.usable(scan)` (move `is_body` and `usable` here from `pi-drive/decide.py`).
  - `wheels.packet(v: float, w: float, p: dict) -> bytes`: true m/s and rad/s in, firmware packet out. `v` is multiplied by `p["linear_cmd_scale"] or 1.0`, `w` by `p["angular_cmd_scale"]`; both clamped to `p["max_speed"]` and `p["max_turn_rate"]` before scaling; zero gives `b"S#"`; negative `v` is refused with `ValueError` (never backward).
  - `wheels.parse_odom(line: str, p: dict) -> tuple[float, float, float]`: `"Ox:+001.00,y:+000.50,th:+000.86#"` to true x, y, yaw (distance / `odom_distance_scale`, yaw / `odom_yaw_scale`); raises `ValueError` on anything else.
  - `wheels.Wheels(p, port=None)`: `.send(v, w) -> bytes`, `.read_odom() -> tuple[float, float, float]`, `.stop()`, `.close()`. A daemon thread resends the last packet every 0.04 s and switches to `S#` after 0.3 s without a new `send` (as `pi-drive/wheels.py` does).
  - `geometry.lidar_to_base(p_xy, params)`, `geometry.base_to_odom(p_xy, pose)`, `geometry.odom_to_base(p_xy, pose)`, `geometry.polar(p_xy) -> (deg, metres)`; `pose` is `(x, y, yaw)`.

- [x] **Step 1:** Copy `lidar.py`, `test_lidar.py` and `fixtures/` from the Pi (`scp xohm@100.73.128.36:pi-drive/...`). Run `python -m unittest base/pi/tests/test_lidar.py`. Expected: OK.
- [x] **Step 2:** Write failing tests for `wheels` and `geometry`:

```python
def test_packet_scales_and_formats(self):
    p = dict(P, linear_cmd_scale=0.9, max_speed=0.6)
    self.assertEqual(wheels.packet(0.30, 0.0, p), b"F027|+000#")
    self.assertEqual(wheels.packet(0.0, -0.40, p), b"F000|-034#")   # 0.40 * 0.862
    self.assertEqual(wheels.packet(0.0, 0.0, p), b"S#")

def test_packet_is_capped_and_never_backward(self):
    self.assertEqual(wheels.packet(2.0, 0.0, P), b"F015|+000#")      # max_speed 0.15, scale null
    with self.assertRaises(ValueError):
        wheels.packet(-0.1, 0.0, P)

def test_parse_odom_applies_the_measured_scales(self):
    x, y, yaw = wheels.parse_odom("Ox:+000.90,y:+000.00,th:+000.86#", P)
    self.assertAlmostEqual(x, 0.90 / 0.902, places=3)
    self.assertAlmostEqual(yaw, 0.86 / 0.862, places=3)

def test_broken_odom_line_is_an_error(self):
    for line in ("", "Ox:+000.90,y:+000.00#", "garbage"):
        with self.assertRaises(ValueError):
            wheels.parse_odom(line, P)

def test_silence_becomes_a_stop(self):            # fake serial port, fake clock
    w = wheels.Wheels(P, port=fake); w.send(0.1, 0.0); clock.advance(0.35); w.tick()
    self.assertEqual(fake.written[-1], b"S#")

def test_base_and_odom_frames_round_trip(self):
    pose = (1.0, 2.0, math.pi / 2)
    p = geometry.base_to_odom((1.0, 0.0), pose)
    self.assertAlmostEqual(p[0], 1.0); self.assertAlmostEqual(p[1], 3.0)
    self.assertAlmostEqual(geometry.odom_to_base(p, pose)[0], 1.0)
```

- [x] **Step 3:** Implement, run, see them pass.
- [x] **Step 4:** Write `base/tools/deploy.sh`: `rsync` (or `scp -r`) `hmc/src/hmc/` to `xohm@100.73.128.36:hmc-base/hmc/`, `base/pi/` and `base/params.json` to `hmc-base/`, and `base/jetson/` to `robotpsu@100.108.202.48:hmc-follow/`. On the Pi create `~/hmc-base/.venv` with `python3 -m venv --system-site-packages` and install numpy, opencv-python-headless and pyserial only if `import` fails. Nothing is deleted on either machine.
- [x] **Step 5:** Run `deploy.sh`, then on the Pi `cd ~/hmc-base && .venv/bin/python -m unittest discover -s tests`. Expected: OK with the same test count as on the laptop. Commit.

### Task 2 (GATE, robot moves): Speed ladder

**Files:**
- Create: `base/pi/tools/base_check.py`, `base/pi/tests/test_base_check.py`
- Modify: `base/params.json`, `base/docs/hardware-assessment-2026-11-13.md` (replace the GATE rows with the numbers)

**Interfaces:**
- Consumes: `Wheels`, `LidarSession`, `params`.
- Produces: `base_check.scan_shift(before, after) -> (dx, dy, dyaw)` (rigid motion between two scans, by point-to-point ICP on `usable` points, numpy); a JSON result file per run in `~/hmc-base/logs/base-check/`.

- [x] **Step 1:** Failing test: `scan_shift` on a synthetic room (a 4 m square of points) shifted by 0.30 m and turned by 10° returns those values within 1 cm and 0.5°. Implement, pass.
- [x] **Step 2:** `base_check.py --line V --metres 1.5`: read a scan standing still, drive straight at true `V` (with `linear_cmd_scale` 1.0 for the first rung), send `S#` after 1.5 m of odometry, wait 2 s, read a scan. Log firmware odometry at 20 Hz and every scan. Report: real distance (scan), odometry distance, real speed over the middle second (scan matching between successive scans), distance travelled after `S#`, sideways drift per metre, yaw change.
- [x] **Step 3:** `base_check.py --turn W --degrees 180`: same for turning on the spot; report real rate, odometry yaw against scan yaw, and coast angle after `S#`. This settles the open question from the hardware assessment (command `-026` with almost no yaw change).
- [ ] **Step 4 (owner at the robot, 3 m clear in front):** Run the line test at 0.15, 0.3, 0.4, 0.5, 0.6 m/s, three runs each, raising `max_speed` in a local copy of the params for the run only. Stop climbing at the first rung where braking exceeds 0.4 m or drift exceeds 5 cm per metre. Run the turn test at 0.26 and 0.49 rad/s, three runs each.
- [x] **Step 5:** Write into `base/params.json`: `linear_cmd_scale` (asked / real), `max_speed` (highest passing rung), `decel` (v² / (2 × braking distance), the lowest of the runs), `stop_delay` (time from `S#` to the first drop in scan-matched speed, the highest of the runs), and corrected odometry scales if they differ from 0.902 and 0.862 by more than 2 %. Paste the table of all runs into the hardware assessment. If no rung above 0.15 passes, leave `max_speed` at 0.15 and say so. Commit.

### Task 3 (GATE, robot stands still): Sensor geometry

**Files:**
- Create: `base/pi/camera.py`, `base/pi/tests/test_camera.py`, `base/pi/tools/sensor_check.py`, `base/pi/tests/test_sensor_check.py`
- Modify: `base/params.json`, the hardware assessment

**Interfaces:**
- Produces:
  - `camera.Frame` (dataclass): `time: float` (monotonic at capture), `width`, `height`, `rgb: bytes`, `depth: numpy.ndarray` (uint16 millimetres, shape `(height, width)`).
  - `camera.Camera()`: `.start()`, `.latest() -> Frame | None` (newest, at most once each), `.error`, `.stop()`. The OpenNI2 ctypes capture is taken from `~/pi-drive/person.py:_capture_color_and_depth`, with the depth buffer read by `numpy.frombuffer` instead of `struct.unpack`.
  - `sensor_check.fit_floor(depth, hfov_deg) -> (camera_height, pitch_deg, inlier_share)`.

- [x] **Step 1:** Failing test: a synthetic depth image of a flat floor seen from 0.20 m with 8° of down pitch gives `fit_floor` within 1 cm and 0.5°. Implement with a RANSAC plane fit on the lower half of the image, pass.
- [x] **Step 2:** `sensor_check.py --floor`: 30 frames on an empty floor, median height and pitch, inlier share. `sensor_check.py --yaw`: the owner puts a bottle 1.5 m straight ahead on a floor-tile line that runs through the middle of the robot; report the bottle's bearing in the scan (its centre from `legs.clusters`) and in the image (darkest-depth blob centre). The scan bearing error is the correction to `lidar_front_deg`; the image bearing must then agree within 2°.
- [x] **Step 3 (owner at the robot, wheels off):** Run both. The owner measures with a tape: camera lens to the turning centre along x (`camera_x`), robot height to its highest point (`robot_height`), and the battery voltage and type for the assessment.
- [ ] **Step 4:** Fill `camera_x`, `camera_height`, `camera_pitch_deg`, `robot_height`, `lidar_front_deg` in `base/params.json`. Record the numbers in the hardware assessment. Commit.

### Task 4: One brain: extract `Follower` in `hmc`

**Files:**
- Create: `hmc/src/hmc/follower.py`, `hmc/tests/test_follower.py`
- Modify: `hmc/src/hmc/task.py` (FOLLOW and SEARCH), `hmc/src/hmc/navigation.py`, `hmc/src/hmc/appearance.py`, `hmc/docs/tasks/5.1-help-me-carry.md` (one line: where the follow logic lives), `hmc/WORKLOG.md`

**Interfaces:**
- Produces:

```python
@dataclass
class Senses:
    time: float
    operator: Point | None                    # the tracked operator this cycle, belief frame
    operator_look: tuple[float, ...] | None   # their look, if the camera saw them this cycle
    people: dict[int, Point]                  # everyone else, by tracker id
    obstacles: list[Shape]

class Mode(Enum): LEARN, FOLLOW, SEARCH

class Follower:
    def __init__(self, body, speech, follow_distance: float = navigation.FOLLOW_DISTANCE,
                 search_again_every: float | None = None): ...
    # search_again_every: after asking the operator to come back, turn on the spot again
    # every this many seconds. None (the simulation) keeps today's behaviour: stand still.
    # The robot passes 10.0.
    mode: Mode
    last_operator_pos: Point | None
    tracker: perception.MotionTracker
    operator_look: LookMemory
    navigator: navigation.Navigator
    def score(self, look) -> float            # similarity to the learned operator
    def watch(self, senses: Senses) -> Point | None   # today's Task._watch_operator
    def shapes(self, senses: Senses) -> list[Shape]   # today's Task._shapes
    def step(self, senses: Senses, dt: float) -> None # follow or search, drives the body
```

  `body` needs `pos`, `heading`, `speed`, `drive_toward`, `turn_toward`, `brake` (`RobotView` and `world.Robot` both have them). `speech` needs `say(time, text)`.
- `navigation.py`: read the robot radius as `hmc.world.ROBOT_RADIUS` at call time (the way `run.use_base` already overrides `MAX_ACCEL`), so the robot can set 0.29. `follow_step` gains a `follow_distance` parameter with the present default.
- `appearance.LookMemory`: sized by the first look added, so a 512-number look works. `LOOK_SIZE` stays the simulation's size.

- [x] **Step 1:** Record the baseline before touching anything: `cd hmc && pytest -q` (expect `109 passed, 2 xfailed, 1 xpassed`) and `for m in default bangkok2022 bordeaux2023 eindhoven2024 germanopen2024 portugalopen2024; do python -m hmc.run --map $m; done > out/before.txt`.
- [x] **Step 2:** Write failing tests in `hmc/tests/test_follower.py`:

```python
def test_follows_to_the_follow_distance_and_holds(): ...
    # operator 4 m ahead, no obstacles, 200 steps: distance ends within 0.1 m of follow_distance
def test_a_look_that_does_not_match_is_not_the_operator(): ...
    # after 20 learning frames, a detection with an unrelated look leaves last_operator_pos unchanged
def test_search_after_two_seconds_without_the_operator(): ...
    # senses.operator None for LOST_AFTER + dt: mode is SEARCH; a matching look brings FOLLOW back
def test_a_stranger_does_not_end_the_search(): ...
def test_remembered_obstacle_still_blocks_after_it_leaves_the_view(): ...
    # seen once, absent for 5 s: still in shapes(); absent for MEMORY_KEEP + 1 s: gone
def test_look_memory_takes_a_512_number_look(): ...
    # LookMemory().add(look512); similarity(look512) == pytest.approx(1.0)
def test_follow_distance_is_a_parameter(): ...
def test_search_turns_again_every_ten_seconds_when_asked_to(): ...
    # search_again_every=10.0: 12 s after the come-back line the body is turning again;
    # with None it stands still
```

- [x] **Step 3:** Move the code. `Task` keeps its states and builds a `Senses` each step from `perception.see_operator`, `see_operator_look`, `see_people`, `see_obstacles`; `_follow` and `_search` call `follower.step`. Nothing about bags, queue or crowd moves.
- [x] **Step 4:** Run `pytest -q`: `117 passed, 2 xfailed, 1 xpassed`. Rerun the six maps into `out/after.txt`; `diff out/before.txt out/after.txt` must be empty (a refactor, no behaviour change). If it is not empty, find the cause; do not regenerate evidence to hide it.
- [x] **Step 5:** Add a WORKLOG entry. Commit.

### Task 5: People in the LiDAR scan

**Files:**
- Create: `base/pi/legs.py`, `base/pi/tests/test_legs.py` (copy both from Pi `~/pi-drive`, written and tested 2026-11-13; drop `chase` and its tests, the brain replaces it; import `usable` from `lidar`)

**Interfaces:**
- Produces: `legs.clusters(scan) -> list[Point]`, `legs.candidates(scan) -> list[Point]` (lidar frame, legs within 0.5 m merged), `legs.moved(point, v, w, dt) -> Point`, `legs.LegTracker` with `.pos`, `.seen_at`, `.predict(v, w, dt)`, `.update(scan, now, lock_at: Point | None = None) -> Point | None`. `update` takes the lock point as a lidar-frame point instead of a camera dict.

- [x] **Step 1:** Copy, adapt the signature, run the copied tests. Expected: the 7 candidate and tracker tests pass.
- [x] **Step 2:** Add the Review Focus 2 test and make it pass:

```python
def test_two_people_side_by_side_then_apart_does_not_switch(self):
    t = LegTracker(); t.update(scan_of(person(1.5, 0.0)), 0.0, lock_at=(1.5, 0.0))
    t.update(scan_of(person(1.5, 0.0) + person(1.5, -0.45)), 0.1)        # shoulder to shoulder
    for i in range(1, 15):                                                # the other walks off
        pos = t.update(scan_of(person(1.5, 0.0) + person(1.5, -0.45 - 0.1 * i)), 0.1 + 0.1 * i)
    self.assertIsNotNone(pos)
    self.assertAlmostEqual(pos[1], 0.0, delta=0.15)
```

- [ ] **Step 3:** On the Pi, with the owner walking a loop around the standing robot (wheels closed), log `candidates` and the track for 60 s after a manual lock. Report how many scans held the track. Expected: at least 95 %. If a table or chair leg captured the track, add that scan to the fixtures as a regression test before fixing. Commit.

### Task 6: Obstacles from the depth image and for the planner

**Files:**
- Create: `base/pi/depth_obstacles.py`, `base/pi/shapes.py`, tests for both

**Interfaces:**
- Consumes: `camera.Frame`, `params`, `geometry`, `hmc.world.Circle`.
- Produces:
  - `depth_obstacles.points(depth, p) -> numpy.ndarray | None`: `(N, 2)` base-frame x, y of pixels whose height above the floor is between 0.05 m and `p["robot_height"]` and whose range is under 3 m, thinned to one point per 5 cm cell. Returns `None` when fewer than 5 % of the pixels have a depth (the sunlight case): `None` means "no depth", an empty array means "nothing there".
  - `shapes.ObstacleMemory(p)`: `.update(now, pose, scan, depth_points, people: list[Point]) -> list[Circle]`, in the odometry frame. LiDAR points (after `usable`) are circles of radius 0.05 m, one per 0.1 m cell, valid for this scan only. Depth points are circles of radius 0.05 m kept for 10 s after last seen. Points within 0.45 m of any point in `people` are left out (people are added by the brain as person circles).

- [x] **Step 1:** Failing tests:

```python
def test_floor_is_not_an_obstacle_and_a_box_is(self): ...
    # synthetic depth: flat floor plus a 0.12 m high box at 1.2 m: points only on the box, x within 5 cm
def test_things_above_the_robot_are_ignored(self): ...
def test_a_blind_depth_image_is_none_not_empty(self):
    self.assertIsNone(depth_obstacles.points(numpy.zeros((240, 320), numpy.uint16), P))
def test_depth_obstacle_is_remembered_ten_seconds(self): ...
def test_lidar_points_near_a_person_are_left_out(self): ...
def test_shapes_are_in_the_odometry_frame(self): ...
    # the same wall seen from two poses gives circles at the same odometry coordinates within 5 cm
```

- [x] **Step 2:** Implement with numpy, run, pass.
- [ ] **Step 3:** On the Pi (wheels closed): a box 0.12 m high at 1.0 m and a chair at 1.5 m. Save a top-down PNG of LiDAR circles, depth circles and the robot to `~/hmc-base/logs/`. Report whether the box appears and whether the bare floor is clear. Report `hmc.navigation.plan_path` time on the Pi for that scene (median and worst of 50). If the worst is over 150 ms, say so; Task 10 then plans in a worker thread. Commit.

### Task 7: Body and last-resort stop

**Files:**
- Create: `base/pi/body.py`, `base/pi/safety.py`, tests for both

**Interfaces:**
- Consumes: `hmc.world.Robot`, `hmc.world` module limits, `params`, `Wheels`.
- Produces:
  - `body.Body(p)`: `.robot` is an `hmc.world.Robot` with `max_speed` and `max_turn_rate` from the params; the constructor also sets `hmc.world.MAX_ACCEL`, `MAX_TURN_ACCEL` and `ROBOT_RADIUS` from `accel`, `turn_accel`, `robot_radius`. `.sync(pose)` overwrites `robot.x, y, heading` with odometry and leaves `speed` and `turn_speed`. `.command() -> (v, w)` returns `robot.speed, robot.turn_speed` after the brain's step. `.limit(max_speed)` lowers the cap for this cycle.
  - `safety.braking_distance(v, p) -> float` = `v*v / (2*p["decel"]) + v*p["stop_delay"] + 0.15`.
  - `safety.must_stop(points_base, v, w, p) -> bool`: true when any point lies in the strip the robot will sweep within the braking distance: `0 < x - 0.20 < braking_distance(v)` and `|y| < robot_half_width + 0.05` for forward motion; for any turning (`w != 0`, with or without `v`), any point inside radius 0.33 m on either side; NaN input means stop.
  - `safety.speed_cap(depth_points, p) -> float`: `p["max_speed"]` with depth, `min(p["max_speed"], 0.3)` when `depth_points is None`.

- [x] **Step 1:** Failing tests:

```python
def test_braking_distance_formula(self):
    p = dict(P, decel=0.5, stop_delay=0.2)
    self.assertAlmostEqual(safety.braking_distance(0.4, p), 0.16 + 0.08 + 0.15)
def test_stops_for_a_point_inside_the_braking_strip(self): ...
def test_ignores_a_point_beside_the_strip_or_behind(self): ...
def test_no_depth_lowers_the_speed_cap(self):            # Review Focus 4
    self.assertEqual(safety.speed_cap(None, dict(P, max_speed=0.5)), 0.3)
    self.assertEqual(safety.speed_cap(numpy.empty((0, 2)), dict(P, max_speed=0.5)), 0.5)
def test_command_comes_from_the_brain_step(self): ...
    # Body.sync((0,0,0)); body.robot.drive_toward((2,0), 0.1); v > 0 and v <= accel * 0.1 + 1e-9
def test_sync_keeps_speed_and_takes_the_measured_pose(self): ...
```

- [x] **Step 2:** Implement, run, pass. Commit.

### Task 8: Link to the Jetson

**Files:**
- Create: `base/pi/link.py`, `base/pi/tests/test_link.py`, `base/jetson/server.py` (protocol part and a `--fake` mode), `base/jetson/tests/test_server.py`

**Interfaces:**
- Wire format, both directions: a 4-byte big-endian length, then a UTF-8 JSON header, then for requests the JPEG bytes. Request header: `{"seq": int, "t": float, "width": int, "height": int, "jpeg": int}`. Reply: `{"seq": int, "t": float, "people": [{"box": [x1, y1, x2, y2], "conf": float, "look": [float, ...]}]}`. `t` is the Pi's monotonic capture time, echoed back.
- Produces:
  - `link.Seen` (dataclass): `bearing_deg: float`, `conf: float`, `look: tuple[float, ...]`, `box`.
  - `link.Answer` (dataclass): `t: float`, `people: list[Seen]`.
  - `link.bearing_deg(box, width, hfov_deg) -> float`: `(centre_x - width/2) / (width/2) * hfov_deg / 2`, positive to the robot's left (the image is mirrored, so image right is robot left).
  - `link.JetsonLink(host, port, hfov_deg)`: `.start()`, `.send(frame)` (never blocks; replaces a frame still waiting), `.latest() -> Answer | None` (each answer once), `.alive(now, silence) -> bool`, `.stop()`. Reconnects by itself every 1 s.
  - `server.py --fake` answers every frame with one person in the middle and a fixed look, for tests.

- [x] **Step 1:** Failing tests, against a server thread on `127.0.0.1`:

```python
def test_round_trip_keeps_the_capture_time(self): ...
def test_only_the_newest_frame_is_sent(self): ...      # send 5 frames while the server sleeps: it gets 1 or 2
def test_alive_goes_false_after_the_silence_limit(self): ...
def test_link_reconnects_after_the_server_restarts(self): ...
def test_bearing_sign_image_right_is_robot_left(self):
    self.assertGreater(link.bearing_deg([300, 0, 320, 240], 320, 58.59), 20)
def test_send_never_blocks_without_a_server(self): ...   # 100 sends in under 0.1 s
```

- [x] **Step 2:** Implement, run, pass.
- [ ] **Step 3:** Deploy. Run `server.py --fake` on the Jetson and a 60 s stream from the Pi camera. Report frames sent per second, answers per second, and round-trip time median and 95th percentile over Wi-Fi. Commit.

### Task 9: Fusion into `Senses`

**Files:**
- Create: `base/pi/senses.py`, `base/pi/tests/test_senses.py`

**Interfaces:**
- Consumes: `LegTracker`, `candidates`, `ObstacleMemory`, `Answer`, `geometry`, `hmc.follower.Senses`.
- Produces `senses.RealSenses(p)`:
  - `.locked -> bool`: the LiDAR track on the operator is alive.
  - `.enrol(scan, answer) -> tuple[Point, look] | str`: the person nearest the image centre within 2.5 m, or the refusal text `"more than one person within 2.5 m"` / `"nobody in view"`.
  - `.read(now, pose, v, w, dt, scan, depth_points, answer, link_alive, score) -> Senses`, where `score(look) -> float` is `Follower.score`.
- Rules, each one a test:
  1. An answer is used only if `now - answer.t <= 0.5`; older ones are dropped (Review Focus 3). A used answer's bearings are corrected by the robot's turn since `answer.t`.
  2. A camera person is matched to the LiDAR candidate nearest its bearing within 8° and, if the candidate set is ambiguous, nearest in range to the previous track.
  3. While `locked`, `Senses.operator` is the track position in the odometry frame, with or without a camera answer (Review Focus 1). `operator_look` is set only when a camera person matched the track this cycle.
  4. When not locked: if the link is alive and a camera person has `score(look) >= 0.65` and a LiDAR candidate, lock the tracker there and report it. If the link is not alive, never lock: `operator` stays `None`.
  5. `Senses.people` are the other LiDAR candidates between 0.3 m and 4 m (ids by nearest-neighbour continuity). `Senses.obstacles` come from `ObstacleMemory.update` with those people and the operator left out.

- [ ] **Step 1:** Failing tests, one per rule, plus:

```python
def test_holding_close_with_no_camera_look_stays_locked(self): ...        # Review Focus 1: 30 s at 0.8 m, answers empty
def test_stale_answer_is_ignored(self): ...                               # Review Focus 3
def test_no_new_lock_while_the_jetson_is_silent(self): ...
def test_relock_needs_a_matching_look(self): ...
def test_enrol_refuses_two_people_inside_2_5_m(self): ...
```

- [ ] **Step 2:** Implement, run, pass. Commit.

### Task 10: The control loop, its log, and replay

**Files:**
- Create: `base/pi/run_follow.py`, `base/pi/replay.py`, `base/pi/tests/test_run_follow.py`, `base/pi/tests/test_replay.py`

**Interfaces:**
- Consumes everything above and `hmc.follower.Follower`.
- `run_follow.py --seconds N [--drive] [--max-speed V] [--no-jetson]`. Without `--drive` the serial wheel port is never opened and odometry is taken as zero motion.
- Produces `run_follow.cycle(state, now) -> dict`: one control cycle as a pure function of injected devices, so tests run it with fakes. The returned dict is the log record.
- One cycle, at the LiDAR rate (10 Hz):
  1. Read odometry (`ValueError` or a serial error: stop, end the run: Review Focus 5), scan, newest frame, newest answer.
  2. `depth_points`; `body.limit(safety.speed_cap(depth_points, p))`; `body.sync(pose)`.
  3. `senses.read(...)`; `follower.watch(senses)`; `follower.step(senses, dt)` (watch first, every cycle).
  4. `v, w = body.command()`; if `safety.must_stop(scan and depth points, v, w, p)`: `v = w = 0` and `body.robot.speed = 0`.
  5. `wheels.send(v, w)`; send the frame to the Jetson; append the record to `logs/<start time>/log.jsonl`: time, pose, v, w, mode, locked, link alive, operator position, people, obstacle count, nearest obstacle distance, stop reason, cycle time. Every 10th scan and its JPEG are saved too.
- A cycle longer than 0.4 s stops the wheels and ends the run. Ctrl+C and SIGTERM send `S#` first, and a second signal is ignored while stopping.
- Start: wait for the owner to press Enter, then `senses.enrol` until 20 looks are learned (`Follower` in `LEARN`), then follow.
- `--drive` refuses to start above 0.15 m/s unless `params.speed_gates_open`. The depth layer is skipped unless `params.depth_gates_open`.
- `speech` is an object whose `say(time, text)` prints the line and logs it; the voice work plugs in there later.
- `replay.py LOGDIR` prints: share of cycles locked, longest gap, closest obstacle distance, count of stops by reason, cycle time median and worst.

- [ ] **Step 1:** Failing tests with fake devices:

```python
def test_dry_run_never_opens_the_wheel_port(self): ...
def test_broken_odometry_stops_and_ends_the_run(self): ...      # Review Focus 5: last packet written is b"S#"
def test_slow_cycle_stops_the_wheels(self): ...
def test_obstacle_in_the_braking_strip_overrides_the_brain(self): ...
def test_drive_is_refused_above_0_15_with_closed_gates(self): ...
def test_follows_a_simulated_walker_with_fakes(self): ...       # 30 s: locked >= 95 % of cycles, min gap >= 0.3 m
def test_replay_summary_counts(self): ...
```

- [ ] **Step 2:** Implement, run, pass. Full suite on the laptop and on the Pi. Commit.

### Task 11: Jetson: TensorRT, detector, look vectors

**Files:**
- Create: `base/jetson/detector.py`, `base/jetson/reid.py`, `base/jetson/bench.py`, `base/jetson/models/README.md` (where each model file came from, its licence, its SHA-256, how the engine was built), tests; complete `base/jetson/server.py`.

**Interfaces:**
- `detector.Detector(engine_path)`: `.detect(bgr) -> list[tuple[x1, y1, x2, y2, conf]]`, people only, confidence at least 0.4.
- `reid.ReId(engine_path)`: `.looks(bgr, boxes) -> list[tuple[float, ...]]`, unit-length vectors.
- `server.py` without `--fake` uses both.

- [ ] **Step 1:** Before installing, tell the owner the exact command and what it adds: `sudo apt-get install -y tensorrt python3-libnvinfer` (TensorRT 10.16.2.10-1+cuda13.2 from the NVIDIA Jetson repository). The owner approved this install on 2026-11-13. Record the package list before and after with `dpkg -l | grep -i -E "nvinfer|tensorrt"`. If `sudo` asks for a password, stop and ask the owner to run it.
- [ ] **Step 2:** Create `~/hmc-follow/.venv` with `--system-site-packages`. Download a person detector and a re-identification model as ONNX from their official sources, record source, licence and SHA-256 in `models/README.md`, and build FP16 engines with `trtexec`. Tell the owner the file names, sources and sizes before downloading. Note for the owner: Ultralytics YOLO weights are AGPL-3.0; a permissively licensed detector (for example YOLOX, Apache-2.0) avoids that question.
- [ ] **Step 3:** `bench.py`: 300 frames of 320 x 240 with two people, detector plus look vectors, while the other teams' processes run as found. Report frames per second (median, 5th percentile), GPU memory and RAM used. Pass: 5th percentile at least 15 per second and under 1.5 GB of RAM. If it fails, try the next smaller input size, then report.
- [ ] **Step 4:** Tests: `detect` finds the person in a saved frame from the Pi camera and nothing in an empty-room frame; two crops of the same person from one walk score above 0.65 and crops of two different people score below it (use frames recorded in Task 5 step 3). If the 0.65 threshold does not separate them, report the two score distributions and set `OPERATOR_MATCH` for the robot from them in `base/params.json` (key `operator_match`), leaving the simulation's value alone.
- [ ] **Step 5:** Run the real server; from the Pi, stream 60 s with the owner in view. Report answers per second and round-trip time. Commit.

### Task 12: First drives (robot moves)

- [ ] **Step 1:** Dry run, 60 s, owner walks a loop around the robot: `run_follow.py --seconds 60`. `replay.py` must show locked at least 95 % and no cycle over 0.4 s.
- [ ] **Step 2:** `--drive --max-speed 0.15 --seconds 30`, owner standing 2 m ahead in clear space, then walking slowly. Watch for: stops 0.6 m from the owner, never closer than 0.3 m to anything, stops when the owner stops.
- [ ] **Step 3:** Cut the run with Ctrl+C mid-drive, twice: the robot stops both times.
- [ ] **Step 4:** Repeat step 2 at each speed rung that passed Task 2, up to `max_speed`.
- [ ] **Step 5:** Write the numbers from `replay.py` for every run into `base/docs/field-results.md`. Commit.

### Task 13: Scenarios inside the arena

Run each of the nine scenarios of the spec five times. For each run record in `base/docs/field-results.md`: date, place, top speed, locked share, longest gap, closest distance, switches to another person (the owner says who was followed), stops by reason, and the log folder name.

- [ ] **Step 1:** Scenarios 1 to 5 (walk, side and behind, crossing person, two people apart, hide and return). A second person is needed for 3 and 4.
- [ ] **Step 2:** Scenarios 6 and 7 (door and table corner, overhanging table top). For 7 the owner stands ready at the power switch.
- [ ] **Step 3:** Scenario 9 (pull the Jetson's network during a run).
- [ ] **Step 4:** For every run that failed a criterion: save the log as a replay fixture, write the failing test first, then fix. Rerun that scenario five times.

### Task 14: Scenarios outside the arena

- [ ] **Step 1:** Before driving outside, report: Wi-Fi reach from the Jetson's position, or mount the Jetson on the robot with a LAN cable if the owner has arranged power; sunlight on the route (does `depth_obstacles.points` return `None`?); the floor (thresholds, slopes, gratings).
- [ ] **Step 2:** Run scenarios 1 to 6 and 9 outside, five runs each, same table. Top speed is `safety.speed_cap` when the depth layer is blind.
- [ ] **Step 3:** Same failing-run rule as Task 13.

### Task 15: Readiness report

- [ ] **Step 1:** In `base/docs/field-results.md` add a table: one row per criterion of the spec, inside and outside, with pass or fail and the runs that prove it. A criterion with no runs is "not run", not a pass.
- [ ] **Step 2:** Update `hmc`'s real-base limits (`run.BASES["real"]`) with the measured `max_speed`, `accel`, turn rate and turn acceleration, rerun `python -m hmc.robustness` for all maps and record the result in `hmc/WORKLOG.md`. Say plainly if the simulation with the real numbers still fails maps.
- [ ] **Step 3:** List what is still open for the owner: battery and Jetson mounting, the voice hook for "wait" and "come back", bags and arm, the way back.
