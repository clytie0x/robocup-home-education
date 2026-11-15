# Bag sensing: the bag's top height and yaw, measured from a distance

## Why

The first real `/arm/pick_bag` calls all closed on air (hardware-assessment-2026-11-13.md, last section). At the
pick distance nothing sees the bag. The gripper camera is now a fixed front camera that looks over a bag this
close. The Xtion gives no depth nearer than about 0.5 m. The LiDAR is blind within 0.12 m of the laser and cuts
the bag only at 0.245 m. So the bag's top height and its long-axis yaw are measured once, while the robot still
stands 0.7 to 1.5 m away. They then go with the bag's x, y to the arm.

## Where it runs and how it flows

```
Pi  bag.py far stage ──"look" at a settle──> bag_depth.measure_top(depth frame)   (1) Xtion depth, on the Pi
                                    └─ no good depth ─> Look ─TCP 5606─> Jetson front_camera.serve  (2) RGB
                                                                          (started by bag_arm.py, frames
                                                                           from webcam_view :8082/snap.jpg)
    bag.py reached result {"x","y","top","top_by","yaw"} ─status─> base_bridge
        bag_reached|OK|x,y,top=T,yaw=Y ─> hmc_flow ─ pick_bag {"x","y","top","yaw"} ─> bag_arm ─> /arm/pick_bag
```

- **The look** (`bag.py`). It happens once per bag task, at a settle (the robot standing for 0.8 s) with the bag
  centre `LOOK_RANGE` 0.7 to 1.5 m from the cameras. Ranges are counted from `LOOK_CAM_X` 0.205 m ahead of the
  turning centre, since both cameras are about there. A far drive that starts farther away stops at `LOOK_AT`
  1.1 m to look, then goes on. A bag already nearer than 0.7 m at the first stop is not looked at. The status
  says so in `look.why`. While looking the robot stands. It waits `LOOK_DEPTH_S` 1 s for a depth frame and
  `LOOK_RGB_S` 3 s for the Jetson's answer.
- **The order.** (1) Depth on the Pi, since it works in a dark room. (2) Only when depth fails and the Pi has a
  Jetson link: the front camera on the Jetson, which needs the room lit. (3) Neither: no top, and bag_arm uses
  its `DEFAULT_TOP` 0.26 m as before. A virtual bag (`--virtual-operator`) is never looked at.
- **The yaw** always comes from the bag's LiDAR points at the look (`bag_depth.cluster_yaw`), on the Pi. It is
  passed on only with a confidence of at least `YAW_OK` 0.5. The look stores it in the odometry frame, and the
  result gives it in the base frame of the final stop, as a line in [-90, 90) deg.
- **run_follow** changed in four small places, all for the look (no following logic). `State.bag_look` is a
  `bag.Look` when the Jetson link exists. The bag task is made with `look=True, rgb=state.bag_look is not None`.
  The bag task's `View` gets the frame's depth. `bag_look.serve(task, now)` runs every BAG cycle on a thread,
  so it never blocks the cycle.
- **The Jetson service** is `front_camera.serve` on TCP 5606: one JSON line `{"id", "points"}` in, one JSON line
  out, then the connection closes. It only reads (a frame, the calibration file). bag_arm.py starts it next to
  its ROS node, so `start-all` brings it up with the arm. It can also run by hand with
  `python3 front_camera.py`. If the port is taken, bag_arm logs it and the arm jobs still run.
- **The fields.** `bag_reached|CODE|x,y[,top=T][,yaw=Y]` (base_bridge). hmc_flow puts `top` and `yaw` into
  `pick_bag` when they are present. After a miss, the remeasure (`bag_again`) stands too near to look again.
  hmc_flow then keeps the earlier top, since the height does not change, but drops the yaw, since the miss may
  have turned the bag. A new pointing forgets both.
- **bag_arm.** `pick_bag {"x", "y", "top", "yaw"}`. A given top skips the gripper triangulation, as before. The
  yaw goes through `arm_yaw`. `bag_miss_below` and every arm threshold are unchanged.

## (1) Depth path (Pi, `base/pi/bag_depth.py`)

- The cloud comes from the depth frame with params.json's pose (`camera_x`, `camera_y`, `camera_height`,
  `camera_pitch_deg` 12.3 up, mirrored image), the same conversion as depth_obstacles.py.
- The **footprint** is the points within `FOOT_R` 0.15 m of the bag centre (the LiDAR centre from bag.py). The
  **floor ring** is the points 0.20 to 0.40 m from it. A plane `z = a x + b y + c` is fitted to the ring,
  robust to things standing on it (median start, 3 sigma cuts, sigma at least 4 mm).
- **Top** = the 3rd highest footprint point's distance above that plane, measured square to the plane (one flying
  pixel does not set it). Footprint points 0.03 to 0.80 m above the plane are the bag.
- **Why the camera pose errors cancel.** `camera_height` 0.07 is known to be about 0.105. A wrong height shifts
  every point of the frame by the same 3.5 cm. A wrong pitch rotates every point about the camera. Both are rigid
  moves of the whole cloud, and the local floor plane moves with it, so the distance from that plane to the bag's
  highest point is unchanged. A wrong field of view is not rigid and does not cancel.
- **Confidence** is the lowest of four parts. The share of valid depth pixels in the bag's image box (floor to
  the LiDAR plane, where the LiDAR saw the bag): full at 0.6, zero at 0.2. The bag points: full at 40. The floor
  points: full at 100. The floor fit rms: full under 1 cm, zero at 2 cm. If bag pixels reach the image's top
  rows, the top may be out of view, and the confidence is 0. The measurement is used only with a confidence of
  at least 0.5 and a top of 0.10 to 0.60 m. The answer always carries `valid_share`, `bag_points`,
  `floor_points`, `floor_rms` and `why`.
- **Dark, light-absorbing bags** give no depth on the bag. The bag points then drop and the path falls back.

Tests (`base/pi/tests/test_bag_depth.py`) use synthetic depth images: a box on the floor, the camera truly at
0.105 m tilted up 12 deg, measured with params' 0.07 m and 12.3 deg. Tops of 0.26, 0.30 and 0.36 m at 0.7, 1.0
and 1.4 m came out with 0.000 m error (3-decimal rounding). A 2.3 deg pitch error and 3 mm depth noise also gave
0.000 m. The noise is along the rays, and those meet the floor and the bag top at a grazing angle. With the
params pose, the floor itself comes out at -0.035 m, which is the error the local plane removes. A dark bag, a
bag too tall for the image and bare floor are all refused.

## (2) RGB path (Jetson, `base/jetson/front_camera.py`)

**Model.** Pinhole with fx = fy and cx, cy at the image centre ((W-1)/2, (H-1)/2). The camera sits at (x, y, z)
in the robot frame and is turned by yaw (+ left), pitch (+ down) and roll (+ right side down). `mirror` is set
when the image is flipped left-right. The model is stored in `~/hmc-follow/front_camera.json`. Without that file,
or with `"calibrated": false`, `project` and `ray` raise `NotCalibrated`, and the service answers without a top.

**Top from the image.**

1. The region comes from the bag's LiDAR points projected at the floor and at 0.70 m, widened by 0.15 m to each
   side and 0.10 m toward the robot.
2. The segmenter (`ColourEdgeSegmenter`) compares each pixel with the background of its own row. That background
   is the median of the strips left and right of the bag, which see the floor, the baseboard and the wall at
   that row. A pixel is the bag when its colour is more than 40 BGR levels, or 3 times the row's spread, away
   from that background. When colour finds little (a clear plastic bag), strong edges count too. The bag is
   the part connected to the LiDAR points seen at 0.245 m.
3. The segmenter is pluggable. Anything with `segment(image, roi, inner, seeds) -> bool mask` can replace it,
   for example a learned model.
4. The top row is the first bag row. The handle column is the middle of that row. The bottom is the last bag row
   in the same column (where the bag meets the floor).

**Why the height needs no camera height.** Let the range r be the horizontal distance from the camera to the
bag's side in that column (the ray against the line through the LiDAR points). With camera height h:

```
bottom on the floor:   0     = h + r tan(e_bottom)
top:                   z_top = h + r tan(e_top)
so                     z_top = r (tan(e_top) - tan(e_bottom))     (h drops out)
```

A pitch error shifts both elevations by nearly the same angle, so it mostly cancels too. The camera height way,
`h + r tan(e_top)`, is the check. The confidence is full when the two agree within 3 cm and zero at 6 cm. With
no bottom found, the height way is used alone at confidence 0.5.

A handle at the bag's middle is about 5 cm behind the side the range is taken to. That adds about
0.05 x tan(e_top), which is 4 mm for a top 8 cm above the camera at 1 m. The handle (x, y) is the top's ray
taken out to the range plus 0.05 m (bag.py's BAG_HALF_DEPTH). It is reported and logged but not used for the
grasp yet.

Tests (`tests/test_front_camera.py`) render a pink box with an inverted-U handle loop on a tiled floor before a
wall with a black baseboard. The LiDAR points are ray cast at 0.245 m. For tops 0.32 to 0.37 m at 0.95 to 1.4 m
the error was -0.003 to +0.009 m, and the handle place error 5 to 22 mm. With the camera height 3 cm wrong, the
range way stays within 1.2 cm while the height way is 3 cm off. A bag coloured like the floor and wall, a
segmenter that finds nothing, a missing frame and a missing calibration are all refused.

## Yaw and the arm frame

- `bag_depth.cluster_yaw`. With one straight side, the yaw is its direction. With two sides (an L, points more
  than 1 cm off one line), the points are split at the corner and the longer side is taken. The confidence is
  `min(1, extent / 0.20 m)` times the straightness.
- **End on.** A bag seen end-on shows only its short side, whose direction is 90 deg off. So one straight side
  shorter than `ONE_SIDE_MIN` 0.18 m, or an L whose longer side is not 1.3 times the other (`SIDE_RATIO`), gives
  no yaw, and the arm keeps its default. A narrow end longer than 0.18 m (a bag deeper than that) still fools it.
- The handles of a gift bag run along its long axis. The arm team's yaw is "the part's long direction, the jaws
  close across it" (PickBag.action).
- `bag_arm.arm_yaw(yaw, mount) = yaw - mount.yaw - 90 deg`, wrapped to [-90, 90). The arm's forward is -y and its
  left is +x (API.md), so robot forward becomes -90 deg, and across the robot (+90 deg) becomes 0. That is also
  their default for a part straight ahead. Tests check it against `Mount.to_arm` for 13 directions and 3 mount
  yaws.

## Calibrating the front camera (`base/jetson/tools/front_cam_cal.py`, room lit)

**Data so far** (this room). The front wall is 2.26 m ahead (LiDAR line fit). The wall-floor line, the bottom
edge of a black baseboard, is seen in the image. On-the-spot turns of 0, 11.47 and 20.70 deg (LiDAR) moved a
vertical cabinet edge to columns 627.5, 450.0 and 294.0.

- Those three columns alone are 3 numbers for at least 7 unknowns (fx, yaw, the camera's position, the edge's
  x, y). The tool refuses them ("more frames are needed").
- The direction of the turns must be recorded with its sign. Turning left moves a fixed edge right in an
  unmirrored image, and the columns here went down. So either the turns were to the right, or the image is
  mirrored. The fit tries both and the rms tells them apart (a test checks this), but only when the poses carry
  the true sign.
- The camera is about 0.2 m ahead of the turning centre, so it moves when the robot turns. The fit models the
  camera position. Its x, y are fixed from the guess (tape) unless `--free` adds them.

**Procedure** (main session, with the robot, the room lit):

1. Stand square to a flat wall with a black baseboard, the floor in front clear. Best with a vertical edge (a
   cabinet side) in view. Measure the camera's x, y from the turning centre with a tape, and put them in the
   session's `camera_guess`.
2. Turn on the spot to about -15, -7, 0, +7, +15 deg (LiDAR-measured, signed, + left). After each turn, with the
   robot standing:
   `python3 tools/front_cam_cal.py capture --session cal/session.json --name a0 --pose 0,0,YAW_DEG --wall NORMAL_DEG,DIST --edge cabinet=U --url http://<jetson>:8082/snap.jpg`
   - `--pose` is the robot in the world (the first frame's robot frame).
   - `--wall` is the LiDAR wall line fit in this frame's robot frame, from the turning centre. Without it, the
     session's world wall is moved by the pose.
   - `--edge` gives the column near which the vertical edge is.
3. Drive straight back about 0.6 m (LiDAR-measured), then turn and capture the same again. Two wall distances
   tie down the height and the pitch best.
4. `python3 tools/front_cam_cal.py fit cal/session.json --write ~/hmc-follow/front_camera.json`
   - The fit finds fx, yaw, pitch, roll and the height. cx, cy stay at the image centre. The edges' positions are
     helper unknowns.
   - Each standard error is taken at the measured pixel scatter, at least 0.5 px. A parameter whose error is over
     1 % (fx), 0.3 deg or 1 cm is printed NOT DETERMINED, and then nothing is written.
   - An rms over 1.5 px is not written either.
5. `python3 tools/front_cam_cal.py detect FRAME.jpg --out o.png` shows the line the tool found in one frame.

**Fit results on synthetic frames** (true fx 600, z 0.27 m, yaw 1.5, pitch 8, roll -1.2 deg; detection on the
rendered images, start fx 480, z 0.25, angles 0):

| Frames | rms px | fx error | angle errors | z error | Not determined |
| --- | --- | --- | --- | --- | --- |
| 2 distances x 5 turns + edge | 0.18 | +0.02 % | under 0.005 deg | under 0.1 mm | none |
| 1 distance x 5 turns + edge | 0.18 | +0.07 % | under 0.005 deg | 0.1 mm | none |
| 2 distances x 5 turns, no edge | 0.14 | -0.02 % | under 0.005 deg | 0.1 mm | none |
| the room's 3 turns (0 to 20.7 deg), floor line only | 0.14 | -0.44 % | yaw +0.05 deg | 0.1 mm | fx (std 15.7 px), yaw (std 0.33 deg) |
| the room's 3 turns + edge | 0.16 | -0.46 % | yaw +0.06 deg | 0.1 mm | fx (14.3 px), yaw (0.31 deg) |

So the three turns measured so far would not be enough even with clean floor-line pixels. Turns to both sides,
or a second distance, are needed.

**Not modelled.** Lens distortion is not modelled. The cabinet edge sat at column 627.5, at the image's edge,
where a webcam distorts most. A high rms after a fit (over 1.5 px) is the sign of it, and then a radial term
must be added.

## What still needs the robot and the lit room

- The front camera calibration (above). No front_camera.json exists yet, so the RGB path answers "not
  calibrated" until it does.
- The depth path on the real bags:
  - the pink gift bag, the clear plastic bag (it may give little depth), and a dark bag;
  - the valid share and the confidence each one gets;
  - whether the handles show in depth at 0.7 to 1.5 m.
- The RGB segmentation on the real floor tiles, the pink bag and the clear bag (only synthetic so far).
- Whether the look stop at 1.1 m (bag centre from the cameras) puts the whole bag in both images. The Xtion's
  58.6 deg field of view and its 12.3 deg tilt up decide that.
- The yaw on a real bag seen side-on and end-on, and the arm's grasp with it.
- The Jetson's port 5606 open to the Pi (cable 10.0.0.x, Wi-Fi, Tailscale), and bag_arm's service in the
  agent's `arm` program after a deploy.
