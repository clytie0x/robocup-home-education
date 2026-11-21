# Field results (Help Me Carry on the real robot)

The record of what the robot really did. Every plan since 2026-11-13 points here. Each entry gives the date,
the commit the robot ran (`~/hmc-base/VERSION.json`), what was run, and the numbers. Claims without a log, an
image or a measured number stay out.

## 2026-11-21: review of the robot's logs since 2026-11-14

Read only, from the Pi's `~/hmc-base/logs/` (run folders and `console/`) and the Jetson's `~/hmc-follow/runs/`
and `~/hmc-follow/logs/`. 80 console jobs (81 run folders).

What happened, counted from the logs:

| Capability | Count | Notes |
|---|---|---|
| Full mission, start to end | 0 | The closest, 20261116-150749: follow 1.8 m, pointing failed, back to 0.58 m from the start, no bag |
| Followed a walking person more than 2 m | 9 runs | Furthest 6.96 m from the start, longest path 17.29 m (20261114-025427, -030057) |
| `bag_near` reached the bag | 7 of 33 tasks | 18 ended BLOCKED, 7 NO_BAG |
| Point-bag task reached the bag | 2 of 57 tasks | 30 ended NOT_POINTED |
| Arm pick marked success | 3 of 8 trials | Not proven: the gripper reading of a real lift (11-19 17:29) equals an empty close, and the Xtion check called 11-19 15:17 lifted while the bag stayed on the floor |
| Drove back to the start | 2 of 4 | Both without a bag |
| Obstacles of the task, the queue | never tried | |

Root causes, from replayed frames, scans and logs:

- **NOT_POINTED.** The first gate that refused, over 538 frames of the 30 failed tasks: no LiDAR operator 54 %
  (13 tasks began with the follower already unlocked), arm out under 30 deg 17 %, the LiDAR operator not
  within 10 deg of a camera person 12 % (once the LiDAR "operator" was the bag), keypoint visibility 11 %.
  Field procedure: operator 2.2 to 3 m away, arm clearly out, phone down while pointing.
- **`bag_near` BLOCKED.** 14 of 18 were the robot's own rear-left body point (153 deg, 0.21 m) in the stop
  check, on the first cycle, before the robot moved. The self mask deployed later (943b84b) clears 14 of 18
  on the same scans.
- **Stale odometry** (12 runs on 2026-11-19 12:03 to 15:06): the 25 Hz odometry thread (4bb1437) read one
  byte per call under the GIL and lost line tails. Fixed by 44e309d (deployed 15:07). No odometry end in
  the 20 runs since.
- **Lost operator** ("I lost you" 42 times in 20 runs): the LiDAR rules dropped the lock (`_mixed_up` on a
  non-person beside the operator, track jumps), not the re-id threshold. Re-id `LOOK_MATCH` 0.65 separates the
  operator at 1 to 2 m (367 operator crops, median 0.83) but refuses them closer than 1 m. The stranger side
  rests on 6 crops only.
- **bag_only on 2026-11-20 22:59:** the console sent `bag_near` 0.3 s into the run, before the LiDAR's first
  scan at 0.6 s: NO_BAG at once. The scan at 1 s holds the bag at (0.77, -0.04) m. Fixed by bcba098.
- **The new front camera's bag model.** `bag11n-seg-v3` (the old camera's) found the white paper bag in 0 of 4
  frames of 2026-11-20 15:48 that no model was trained on. The research checkpoint n-v4 found it in 4 of 4.

## 2026-11-21: deployed

| Time | Commit | What | Checked |
|---|---|---|---|
| 12:21 | bcba098 | `bag_near` waits for the first scan | VERSION.json on both hosts, `NO_SCAN_S` in the Pi's `bag.py` |
| 13:16 | 87ace05 | The executive's chain (PR #29), the detector names its re-id model | VERSION.json on both hosts; console restarted with the new code; detector restarted; one frame sent to port 5601 answered `"reid": "tensorrt osnet_ain_x1_0_msmt17_256x128.engine"` |
| 13:27 | 154ffc8 | main after PR #24, #27, #28, #29 (the Jetson's bag dataset tools; no Pi file changed) | VERSION.json on both hosts; console untouched (same process); Jetson programs all running |
| 13:59 | 30cd478 | PR #32: pointing without a LiDAR lock, BLOCKED after 1 s, the rear-left self mask, the narrow senses hold; `--params` (the Pi's params.json equalled git at 154ffc8 byte for byte) | VERSION.json; `rear_left_body` in the Pi's params.json; the new code in `bag.py` and `senses.py` |
| 14:12 | 4934b53 | PR #33: an overhang out of view close by stays in the obstacle memory | VERSION.json on both hosts; on the Pi with its own params and Python, a belt 0.6 m ahead was still kept after 60 s |

## Next robot checks (owner and helpers present)

In this order; each one is written up here with its logs.

1. One full run ending at the start: the bag is dropped at `so100_ik.reachable_drop`, "Here is your bag." is
   heard, and the outcome is SUCCESS (`events.jsonl`).
2. A run with the base bridge stopped: BASE_LINK_DOWN within about 7 s, and the wheels never move.
3. While following, walk out of view: "Please wait for me." and the come-back line, each heard once.
4. The pick (PRs #21, #26, #30): 5 picks per bag, Xtion before and after images saved. Label each pick lifted or
   not from the images. Record every put-down: did the bag stand, did it hit COLLISION?
5. Calibration: the Xtion lens height (`camera_height`, 0.07 in `params.json` against about 0.105 in the docs),
   the LiDAR plane height, and the front camera's file (`~/hmc-follow/front_camera.json`).
6. 30 to 40 front camera frames of every bag (standing, lying, several distances, a few empty floors) for the
   bag model v5 (`base/jetson/tools/capture_front.py`).
7. The staged tests of the plan: follow 10 m, pointing 10 times, approach and pick 10 times, the way back with
   the bag 5 times, each obstacle 3 times, the queue 3 times, then the full task 3 times on the 5:00 clock.
