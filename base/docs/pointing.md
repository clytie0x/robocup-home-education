# Pointing at the bag (Help Me Carry)

Rule (hmc/docs/reference/hmc-2025.txt, Procedure 1 and 3): at the car the operator points at one of the bags on
the floor; the robot says which bag it understood (perceive bag, 15 points), picks it (100 points) and goes back.

Code: `base/jetson/pointing.py` (pure functions), `base/jetson/server.py` (pose and bags in the answer),
`base/jetson/detector.py` (bag classes), `base/pi/link.py` (parsing, `send(frame, pose=True)`), `base/pi/bag.py`
(the BAG phase of `run_follow.py`, which uses all of it).

## In the task

It runs on the Pi, which has the LiDAR clusters, the operator's LiDAR track and the camera frames. `bag.py` loads
`base/jetson/pointing.py` by its path (`jetson/pointing.py` next to the Pi code, which `tools/deploy.sh` copies, or
`../jetson/pointing.py` on the laptop), so there is one source.

1. `hmc_flow.py` at the car: wait, then "Please point at the bag." and `find_bag`; `base_bridge.py` sends Pi `bag`.
   The Pi takes it only in FOLLOW held by that wait, and enters BAG. Without pointing, the console's `bag_near`
   (`robotctl run bag_near`, also from IDLE) skips steps 2 and 3: the nearest bag-sized LiDAR cluster within
   `NEAR_BAG_R` 2.0 m and `NEAR_BAG_DEG` 40 deg of straight ahead, else NO_BAG (robot-console.md, step commands).
   `bag_near` and `bag_again` first wait up to `NO_SCAN_S` 3 s for a LiDAR scan (2026-11-20: the console's
   `bag_only` sent `bag_near` 0.3 s into the run, before the first scan at 0.6 s, and got NO_BAG at once).
2. BAG, pointing: the robot stands; every frame goes to the Jetson with `pose=True` (only now). The camera person
   nearest the operator's LiDAR bearing (within 10 deg) gives `pointing()`; `Steady` needs about 1 s. With
   `--virtual-operator` the PC's pointed spot stands in for it (no camera). No steady pointing in 20 s: NOT_POINTED.
3. Candidates: the LiDAR clusters (`legs.clusters` of the scan, the operator's legs left out), and
   `bag_candidates` of the detector's bag boxes for a bag under the LiDAR plane. A cluster is the side of the bag
   facing the laser; the centre is taken 0.05 m behind it (`bag.BAG_HALF_DEPTH`, an assumption to measure).
   `choose_bag`; none within 0.6 m of the point for 2 s: NO_BAG. The choice and its sentence go to the status line,
   and the bridge sends `bag_chosen|SIDE|text` for the flow to say.
4. The drive: turn on the spot until the bag lies on the robot's line y = 0.0145 (the arm's pan axis), straight on
   at 0.08 m/s to 0.25 m before the goal, then creep at 0.03 m/s until the bag centre is at robot x = 0.31.
   Every point counts for the stop, except while creeping: then only the locked bag (its points within 0.15 m of
   its centre) may come closer than the normal clearance. Anything else in the front strip, the cable on the
   floor across the way, a wound cable, more than 4 m driven or 120 s: BLOCKED or TIMEOUT. The LiDAR sees nothing
   within 0.12 m of the laser (`lidar.is_body`), so the last few centimetres go by odometry from the last
   sighting (`odometry_m` in the result says how far). A bag the LiDAR had seen and lost for 1.5 s: LOST_BAG.
5. Reached: `bag_reached|OK|x,y` (ESTIMATED when only the camera found the bag), robot frame; the flow sends
   `pick_bag` with it. The drive counts as path for the way back.

Not tested on the robot: all of BAG. Tested with fakes (`base/pi/tests/test_bag.py`): the approach in a simulated
room with virtual bags, the side words with two bags, a bag behind, the stops, the timeouts, the run_follow phase
over `--remote` with the virtual operator.

## What the Jetson answers

- `objects`: COCO backpack (24), handbag (26), suitcase (28) at score >= 0.25, from the same detector run as the
  people (no extra cost), NMS across the three classes. On the Pi: `Answer.objects`, a `Thing(cls, bearing_deg,
  conf, box)` each.
- `kp` per person: only when the request header has `"pose": true` (`JetsonLink.send(frame, pose=True)`) or the
  server runs with `--pose-every N`. yolo11n-pose (`models/yolo11n-pose_256x320.engine`) output 1x56x1680 is
  decoded to boxes and 17 x [x, y, visibility] in the 320 x 240 frame, and each pose box goes to the person box it
  overlaps best (IoU >= 0.3). On the Pi: `Seen.kp`, None on frames without pose.
- Cost, not measured: yolo11n-pose is about 1.2 times yolo11n in parameters and FLOPs. Detector plus OSNet ran at
  94.7 frames/s (10.6 ms) on 2026-11-13, so pose should add about 5 to 8 ms. Measure with pose on before relying
  on it at 10 Hz.

## Geometry

Robot base frame: x ahead, y left, z up, floor z = 0. Camera as in `pi/depth_obstacles.py`: pinhole with focal
`f = 160 / tan(hfov / 2)` (285 px for 58.59 deg), at (`camera_x`, `camera_y`, `camera_height`), tilted up by
`camera_pitch_deg`. The image is mirrored: image right is robot left.

1. Pointing check, in image pixels, per arm: shoulder, elbow, wrist and both hips have visibility > 0.5; the angle
   at the shoulder between the upper arm and the torso side (shoulder to hip of the same side) is > 30 deg; the
   wrist is farther from the body axis (mid-hip to mid-shoulder) than the elbow. Both arms out: the larger angle.
2. Depth: the operator's LiDAR position (range, bearing) gives the plane. Each keypoint goes where its camera ray
   meets the vertical plane through that position that faces the camera. Approximation: the whole body is at the
   operator's range, so the arm is taken to lie in that plane.
3. The 3D ray shoulder -> wrist meets the floor at `s = z_shoulder / (z_shoulder - z_wrist)`. No point if the wrist
   is not below the shoulder or the point is more than 3 m from the operator.
4. Confidence: lowest visibility of shoulder, elbow, wrist, times `min(1, sin(depression) / sin(30 deg))`.
5. `Steady`: keeps the points of the last 1 s (a frame without pointing is skipped, not a reset); reports the
   median of the last 5 only when at least 5 points span 0.9 s or more and no two are 0.25 m or more apart.
6. `bag_candidates`: bearing from the box centre; range from the LiDAR cluster nearest in bearing (seen from the
   camera) within 5 deg, else from the box bottom row on the floor (fallback only: it leans on camera height and
   pitch).
7. `choose_bag`: candidates within 0.6 m of the point; the one nearest the floor ray from the shoulder through
   the point, plus a quarter of its distance along the ray from the point (two bags in line with the arm go to
   the nearer). Side relative to the operator, who is taken to face the robot: "left", "right", "middle", or
   none for a single bag. `Choice.say` is the sentence to speak or show.
8. Arm name: in the mirrored image the model sees a mirrored person and calls the operator's right arm "left", so
   `Pointing.arm` swaps the label. Assumed, check it (below).

Synthetic results (`tests/test_pointing.py`): operator 1.5 m ahead pointing at the floor 1 m to the side, error
0.04 m; at 2.5 m, 0.02 m; with 2 px noise on shoulder, elbow and wrist, < 0.25 m. Error from the calibration at
2.5 m: camera height 0.07 used while 0.105 is true 0.02 m, pitch off by 2 deg 0.04 to 0.06 m, hfov off by 2 deg
0.04 m, LiDAR range off by 0.2 m 0.19 to 0.24 m. The range is the one that matters.

## Assumptions and failure modes

- The camera sees the shoulders only from about 2.2 m. The top image row is 0.98 m above the floor at 1.5 m,
  1.33 m at 2 m, 1.68 m at 2.5 m (camera 0.07 m high, pitched 12.3 deg up). At the follow gap (0.6 m) the robot
  sees legs only: it must stand 2.3 to 3 m from the operator for this step.
- Arm aimed toward or away from the camera: its depth part is lost (the plane). A spot 0.8 m toward the robot and
  0.8 m to the side gave an error of 0.9 m (test `test_pointing_toward_the_robot_is_the_known_weak_case`). Ask the
  operator to stand beside the bags, side-on to the robot, or turn the robot so the bags are to the operator's
  side as the robot sees it.
- Bag at the operator's feet: the arm hangs within 30 deg of the torso and is not taken as pointing. A bag 1 m to
  the side gives about 40 deg.
- Occlusion: the pointing arm behind the body, or another person or the car in front: low visibility, no point.
  Keep asking; `Steady` needs a full second.
- Range: the leg tracker measures the legs, the shoulders are about 0.1 m behind; a range error moves the point by
  about the same amount.
- Bags the LiDAR cannot see (a handbag lower than the scan plane) fall back to the box bottom, which is weak.
- Person facing away: the side words ("your left") assume the operator faces the robot.

## How to test on the robot

Nothing here has run on the robot or the Jetson.

1. Jetson: `python3 server.py` with `models/yolo11n-pose_256x320.engine` present (it prints if missing). Time a
   frame with pose on and off; at 10 Hz pose + detector + OSNet + link must stay well under 100 ms.
2. Check the label swap: the operator, facing the robot at 2.5 m, lifts the right arm; with `pose=True`, `kp[9]`
   (model "left wrist") must be at the image right (robot left) and `Pointing.arm` must say "right".
3. Floor point: tape marks on the floor 1 m to each side of the operator, at 2.5 and 3 m from the robot. Point at
   each for 3 s; log `Pointing.point` and the `Steady` result; error against the tape, 10 tries per mark.
4. Bags: two bags 0.8 m apart beside the operator; check `bag_candidates` against tape and `choose_bag` side words.
5. Measure first: camera_height (params says 0.07, measured about 0.105), camera_pitch_deg, camera_x/camera_y, the
   leg tracker's range offset to the body centre, whether the LiDAR sees each bag type, and pose time on the Jetson.
