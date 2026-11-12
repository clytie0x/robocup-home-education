# Return Trip Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** After following, the real robot drives back to within 0.5 m of where it started, without a map made beforehand, and joins the queue of crowd people on the way, using the same decision code as the `hmc` simulation.

**Architecture:** Teach and repeat. The brain in `hmc/` drops a breadcrumb every 0.5 m while following and drives the breadcrumbs in reverse on the way back, with the same A* planner and obstacle handling. The queue logic already in `hmc/src/hmc/task.py` moves behind an interface like `Follower`. On the robot, each breadcrumb also stores a LiDAR scan; on the way back the current scan is matched against it to correct odometry drift.

**Tech Stack:** As the person-follow plan: standard-library Python in `hmc/`; numpy on the Pi; `unittest` in `base/`.

**Spec:** `base/docs/superpowers/specs/2026-11-13-return-trip-design.md`
**Depends on:** `base/docs/superpowers/plans/2026-11-13-person-follow.md`, Tasks 0 to 12 finished. This plan uses `hmc.follower.Follower` and `Senses`, `base/pi/body.py`, `senses.py`, `shapes.py`, `safety.py`, `run_follow.py`, `replay.py` and the scan matcher from that plan's Task 2.

## Global Constraints

- Every Global Constraint of the person-follow plan applies unchanged: branch `feature/person-follow` (or a branch cut from it), local commits only, English in the repository, Thai to the owner, `hmc/` simulation only and standard library only, nothing outside `~/hmc-base` on the Pi and `~/hmc-follow` on the Jetson, never backward, and **the wheels turn only when the owner has said in chat, for that run, that they are at the robot**.
- Values fixed by the spec: breadcrumb every 0.5 m; loop cut when a new breadcrumb is within 0.5 m of an older one that is not among the last three; breadcrumb reached within 0.4 m; skip ahead after 5 s, at most three breadcrumbs; ask to pass after 20 s; stop at the start within 0.3 m of the believed position and turn to the start heading; keyframe used within 1.0 m and 30° (original or reversed heading); a match is used only with at least 60 % of points agreeing within 5 cm and a correction under 0.5 m and 20°; queue gap 1.0 m; leave the queue 2 m from the start, when the followed person stops, or after 3 s without a queue member; crowd = stood still at least 6 s, in a group within 1.5 m, within 4 m of the robot.
- The simulation must keep 1115 of 1115 on all six maps and all three bags after its way back changes. If it does not, the brain is wrong; do not loosen the test.
- The robot carries no bag and has no voice command yet. The way back starts when the owner presses Enter. Spoken lines are printed and logged through the same `speech.say(time, text)` object as in following.

## Review Focus

1. **A long plain corridor.** A scan of two parallel walls matches equally well anywhere along the corridor. Expected: a match there corrects sideways position and heading only, never position along the corridor. Test in Task 3.
2. **A pose correction while driving.** The believed pose jumps by up to 0.5 m. Expected: remembered depth obstacles are dropped (they are now in the wrong place), the path is replanned, and the wheels do not jerk: speed stays within the acceleration limit. Test in Task 5.
3. **The owner presses Enter before the robot has moved, or while it is searching.** Expected: with one breadcrumb the robot reports it is at the start at once; from a search it returns from where it stands. Test in Task 2.
4. **The queue arrives and stops while the robot is behind it.** Expected: the robot leaves the queue and finishes along the breadcrumbs; it does not wait behind standing people for ever. Test in Task 1.
5. **A crowded keyframe.** People stood in the stored scan and are gone now, or the other way round. Expected: the match fails the 60 % rule and is skipped; the robot continues on odometry and logs it. Test in Task 3.

---

## File structure

```
hmc/src/hmc/follower.py        edit  Senses gains `looks`; Follower drops breadcrumbs
hmc/src/hmc/queue.py           new   QueueFollower: crowd memory, tail, joining
hmc/src/hmc/route.py           new   Route (breadcrumbs), Returner (drive them back)
hmc/src/hmc/task.py            edit  RETURN and JOIN_QUEUE delegate
hmc/tests/test_queue.py        new
hmc/tests/test_route.py        new
hmc/docs/evidence/help-me-carry-slice2/   regenerated in Task 2

base/pi/scanmatch.py           new or moved   scan_match with a quality share
base/pi/keyframes.py           new   stored scans and the pose correction
base/pi/senses.py              edit  looks for every person
base/pi/run_follow.py          edit  phases FOLLOW, RETURN, DONE; Enter key
base/pi/replay.py              edit  return summary
base/pi/tests/test_scanmatch.py, test_keyframes.py   new
base/docs/field-results.md     edit  return scenarios
```

---

### Task 0: Check what this plan stands on

- [ ] **Step 1:** Confirm the person-follow plan's Tasks 0 to 12 are ticked in its file and that `base/docs/field-results.md` has the first-drive numbers. If not, stop and tell the owner which tasks are missing.
- [ ] **Step 2:** Run both suites and record the summary lines as the baseline: `cd hmc && .venv/Scripts/python -m pytest -q` and `PYTHONPATH=hmc/src:base/pi python -m unittest discover -s base/pi/tests`.
- [ ] **Step 3:** Record the simulation baseline: the six-map output of `python -m hmc.run --map <map>` into `out/return-before.txt` (scratch, not committed).

### Task 1: Queue logic behind an interface

**Files:**
- Create: `hmc/src/hmc/queue.py`, `hmc/tests/test_queue.py`
- Modify: `hmc/src/hmc/follower.py` (`Senses`), `hmc/src/hmc/task.py`

**Interfaces:**
- `Senses` gains `looks: dict[int, tuple[float, ...]]` (default empty): the look of each person in `people` that the camera saw this step, by the same id.
- Produces:

```python
class QueueFollower:
    def __init__(self, body, start: Point, people_trackers: dict[int, MotionTracker]): ...
    crowd: list[LookMemory]
    done: bool                                    # the queue part is over; never joined again
    def remember_crowd(self, senses: Senses, standing: dict[int, tuple[float, Point]]) -> None
    def tail(self, senses: Senses) -> int | None  # today's Task._queue_tail
    def step(self, senses: Senses, shapes: list[Shape], navigator, dt: float) -> bool
        # drives 1.0 m behind the tail; returns False when the robot must leave the queue
```

  The constants `QUEUE_GAP`, `QUEUE_SPOT`, `CROWD_*`, `SAME_PERSON`, `CROWD_MATCH`, `LOST_TAIL_WAIT`, `LEAVE_QUEUE_NEAR_START` move with the code and keep their values.

- [ ] **Step 1:** Failing tests in `hmc/tests/test_queue.py`:

```python
def test_people_standing_in_a_group_are_remembered_as_the_crowd(): ...
def test_a_single_standing_person_is_not_the_crowd(): ...
def test_tail_is_the_nearest_walking_crowd_member_closer_to_the_start(): ...
def test_a_passer_by_with_another_look_is_not_the_queue(): ...
def test_robot_stands_one_metre_behind_the_tail_in_their_walking_direction(): ...
def test_leaves_when_the_followed_person_stops(): ...          # Review Focus 4: step() returns False, done is True
def test_leaves_two_metres_from_the_start(): ...
def test_keeps_going_three_seconds_after_losing_sight_of_the_tail(): ...
def test_no_looks_means_no_queue(): ...                        # senses.looks empty: tail() is None
```

- [ ] **Step 2:** Move the code out of `task.py`; `Task` fills `Senses.looks` from `perception.see_looks` and its JOIN_QUEUE state calls `QueueFollower.step`.
- [ ] **Step 3:** `pytest -q` passes with nine more tests than the baseline. The six-map output is byte-identical to `out/return-before.txt` (this task is a refactor). Commit.

### Task 2: Breadcrumbs and the drive back, in the brain

**Files:**
- Create: `hmc/src/hmc/route.py`, `hmc/tests/test_route.py`
- Modify: `hmc/src/hmc/follower.py` (drops breadcrumbs), `hmc/src/hmc/task.py` (RETURN), `hmc/docs/tasks/5.1-help-me-carry.md` (the notes on the way back and the acceptance table), `hmc/WORKLOG.md`, `hmc/docs/evidence/help-me-carry-slice2/` and its `SHA256SUMS`

**Interfaces:**
- Produces:

```python
SPACING, LOOP, REACHED, ARRIVE = 0.5, 0.5, 0.4, 0.3
SKIP_AFTER, MAX_SKIP, ASK_AFTER = 5.0, 3, 20.0

class Route:
    points: list[Point]                       # oldest first; points[0] is the start
    def __init__(self, start: Point): ...
    def add(self, pos: Point) -> bool         # True when a breadcrumb was dropped or a loop was cut
    def back(self) -> list[Point]             # newest first, ending at the start

class Returner:
    def __init__(self, body, speech, navigator, route: Route, start_heading: float): ...
    done: bool
    target: Point | None                      # the breadcrumb it is driving to
    def step(self, shapes: list[Shape], now: float, dt: float) -> None
```

- `Follower.route: Route`, created with the body's position at construction; `Follower.step` calls `route.add(body.pos)` every step.
- `hri.PASS_TEXT = "Please let me pass."`, `hri.RETURN_TEXT = "I will go back to the start now."`, `hri.ARRIVED_TEXT = "I am back at the start."`.

- [ ] **Step 1:** Failing tests in `hmc/tests/test_route.py`:

```python
def test_a_breadcrumb_every_half_metre():
    r = Route((0.0, 0.0))
    for i in range(1, 21): r.add((0.1 * i, 0.0))
    assert [round(x, 1) for x, _ in r.points] == [0.0, 0.5, 1.0, 1.5, 2.0]

def test_a_loop_is_cut_out(): ...
    # out 3 m, a 2 m circle back onto the line, then on: no point of the circle is left in back()
def test_standing_still_adds_nothing(): ...
def test_drives_the_breadcrumbs_in_reverse_and_stops_at_the_start(): ...
    # L-shaped route, wall across the straight line to the start: ends within ARRIVE of points[0], never inside the wall
def test_turns_to_the_start_heading_at_the_end(): ...
def test_one_breadcrumb_means_already_there(): ...             # Review Focus 3: done after the first step
def test_return_from_a_search_starts_where_the_robot_stands(): ...   # Review Focus 3
def test_skips_a_blocked_breadcrumb_after_five_seconds(): ...
def test_asks_to_pass_after_twenty_seconds_blocked_and_goes_on_when_free(): ...
def test_goes_around_a_chair_put_on_the_route(): ...
```

- [ ] **Step 2:** Implement `Route` and `Returner`. `Task._return` drives with `Returner` instead of `_drive_to(self.start, ...)`; the dropped-bag branch and the queue branch stay.
- [ ] **Step 3:** `pytest -q`: every test passes, including `AT-38` (all maps, all bags, 1115, nothing touched) and the lost-operator and noise tests. If a map now fails, fix the brain. Rerun `python -m hmc.robustness` for `--odometry wheel` and `--odometry slam` and record the run counts next to the old ones (15 of 18 and 16 of 18) in `hmc/WORKLOG.md`.
- [ ] **Step 4:** The behaviour changed, so follow `hmc/AGENTS.md`: update the notes and the acceptance table in the task spec, regenerate the evidence folder and `SHA256SUMS`. Commit.

### Task 3: Scan matching with a quality score, and keyframes

**Files:**
- Create: `base/pi/scanmatch.py` (move `scan_shift` here from `base/pi/tools/base_check.py` if the follow plan put it there; `base_check.py` then imports it), `base/pi/keyframes.py`, `base/pi/tests/test_scanmatch.py`, `base/pi/tests/test_keyframes.py`

**Interfaces:**
- `scanmatch.scan_match(reference, scan, guess=(0.0, 0.0, 0.0)) -> tuple[float, float, float, float, float]`: `dx, dy, dyaw` of the pose the `scan` was taken from, in the reference scan's frame, plus `share` (points agreeing within 5 cm) and `slide` (0 to 1: how freely the match can slide in one direction, from the spread of the point directions; 1 for two parallel walls, near 0 for a room with corners).
- `keyframes.Keyframes(p)`:
  - `.add(pose, scan) -> None`: stored with every breadcrumb, pose in the corrected frame.
  - `.correct(pose, scan) -> tuple[Pose, dict]`: the corrected pose and a report `{"used": bool, "index": int | None, "share": float, "shift": float, "turn": float, "why": str}`.
  - Rules: nearest keyframe within 1.0 m and within 30° of its heading or its reverse; used only if `share >= 0.6`, shift `< 0.5`, turn `< 20°`; when `slide > 0.8` only the heading and the sideways part of the shift are applied (Review Focus 1).

- [ ] **Step 1:** Failing tests on synthetic rooms:

```python
def test_recovers_a_known_shift_in_a_room_with_corners(self): ...      # 0.20 m and 5 deg, within 2 cm and 0.5 deg, share > 0.9
def test_reversed_heading_matches_the_same_place(self): ...            # scan taken facing the other way
def test_a_plain_corridor_corrects_sideways_and_heading_only(self): ... # Review Focus 1: along-corridor part of the correction is 0
def test_people_in_the_scan_lower_the_share_and_the_match_is_skipped(self): ...   # Review Focus 5: used is False, why == "share"
def test_a_far_keyframe_is_not_used(self): ...
def test_a_large_correction_is_refused(self): ...
```

- [ ] **Step 2:** Implement with numpy, pass.
- [ ] **Step 3:** On the Pi, with the wheel port closed: the owner places the robot by hand on three marked spots in turn and a scan is stored at each. Then the owner puts it back on spot one, about 10 cm and 10° off. Report the correction against the tape. Commit.

### Task 4: Looks for every person

**Files:**
- Modify: `base/pi/senses.py`, `base/pi/tests/test_senses.py`

**Interfaces:**
- `RealSenses.read(...)` fills `Senses.looks` for every person in `Senses.people` whose LiDAR candidate was matched to a camera person this cycle (the same 8° bearing rule as for the operator). No answer or a stale answer: `looks` is empty.

- [ ] **Step 1:** Failing tests: two people in view give two looks under the ids used in `people`; a person seen only by the LiDAR has no look; a stale answer gives no looks.
- [ ] **Step 2:** Implement, pass. Commit.

### Task 5: Phases in the control loop

**Files:**
- Modify: `base/pi/run_follow.py`, `base/pi/replay.py`, their tests

**Interfaces:**
- Phases `FOLLOW`, `RETURN`, `DONE`. In `FOLLOW` each cycle also calls `queue.remember_crowd` and, when the route gained a breadcrumb, `keyframes.add(pose, scan)`. Enter on the notebook (read without blocking) switches to `RETURN` and says `hri.RETURN_TEXT`.
- In `RETURN` each cycle: `pose, report = keyframes.correct(odometry pose, scan)`; if `report["used"]` and the shift is over 0.1 m, drop the remembered depth obstacles and force a replan (Review Focus 2); then `QueueFollower.step` while a queue is joined and not done, else `Returner.step`. The safety stop, the speed cap, the slow-cycle stop and the log are unchanged from following.
- `DONE`: wheels stopped, `hri.ARRIVED_TEXT` said, the run ends.
- The log record gains: phase, breadcrumb count, target breadcrumb, keyframe report, queue tail id.
- `replay.py` adds: breadcrumbs dropped, keyframe matches used and skipped by reason, largest correction, time in the queue, seconds blocked.
- `run_follow.py --return-after N` (test aid): switches to `RETURN` by itself after N seconds of following, so a return can be tested without someone at the keyboard.

- [ ] **Step 1:** Failing tests with fake devices:

```python
def test_enter_switches_to_return_and_says_so(self): ...
def test_a_keyframe_is_stored_with_each_breadcrumb(self): ...
def test_a_pose_correction_clears_depth_memory_and_keeps_the_speed_limits(self): ...   # Review Focus 2
def test_return_with_fakes_ends_within_arrive_of_the_start(self): ...   # simulated 12 m L route, 2 % odometry error, corrections on
def test_without_a_link_the_queue_is_never_joined(self): ...
def test_done_stops_the_wheels_and_ends_the_run(self): ...
```

- [ ] **Step 2:** Implement, pass, run both full suites on the laptop and on the Pi. Commit.

### Task 6: First returns (robot moves)

- [ ] **Step 1:** Dry run: the owner walks a 6 m L-shaped route with the robot standing; nothing to return, but check that looks, crowd memory and keyframes are logged without errors and no cycle is over 0.4 s.
- [ ] **Step 2:** The owner marks the start on the floor under the turning centre. Follow for about 6 m at 0.15 m/s, Enter, return. Measure the error at the start with a tape. Three runs.
- [ ] **Step 3:** The same at the top speed from the follow plan. Three runs.
- [ ] **Step 4:** Write the tape errors and the `replay.py` summaries into `base/docs/field-results.md`. Commit.

### Task 7: Return scenarios

Each of the eight criteria of the spec, five runs, in the place the criterion names (inside the arena or the corridor outside the room). Record per run: date, place, route length, top speed, error at the start by tape, closest distance to anything, keyframe matches used and skipped, seconds blocked, and for the queue runs whether the robot joined within 2 m, its largest gap, and who it followed according to the owner.

- [ ] **Step 1:** Criteria 1 to 3 (inside 10 m, corridor 30 m, corridor with the correction off).
- [ ] **Step 2:** Criteria 4 and 5 (a chair on the route, a full block for 15 s). The owner stands ready at the power switch.
- [ ] **Step 3:** Criteria 6 and 7 with the owner's teammates: three people stand as a group near the route for at least 6 s on the way out and walk back in a line; then a passer-by run. Also record the queue's walking pace against the robot's top speed.
- [ ] **Step 4:** Criterion 8 (Jetson link cut before the way back).
- [ ] **Step 5:** For every failed run: keep the log as a replay fixture, write the failing test first, fix, rerun that criterion five times.
- [ ] **Step 6:** Add a pass table to `base/docs/field-results.md`: one row per criterion, with the runs that prove it. A criterion with no runs is "not run". List what stays open for the owner: the bag and the arm, the voice command that starts the way back, and the queue pace if the robot is too slow for it. Commit.
