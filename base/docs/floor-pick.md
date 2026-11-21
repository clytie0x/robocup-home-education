# Floor bag pick with the SO-100 arm

`base/jetson/floor_pick.py` picks the operator's gift bag from the floor, carries it, and puts it
down. The arm team's `/arm/pick` assumes the object stands on a table (it ignores the object's z),
so it cannot pick from the floor. Files:

| File | What it holds |
|---|---|
| `base/jetson/so100_ik.py` | FK/IK of the SO-100 in the robot base frame, joint limits, floor/deck clearance |
| `base/jetson/gripper_camera.py` | gripper camera model, calibration fit, handle/marker detectors |
| `base/jetson/floor_pick.py` | planner, pick/place/carry/calibrate sequence, ROS 2 transport, CLI |
| `base/jetson/tests/test_floor_pick.py` | tests (no ROS needed) |

## How it talks to the arm

Owner's decision: use the arm team's running driver through its standard ROS 2 interfaces on
domain 42. Never change, restart or reconfigure their nodes.

- `/arm_controller/follow_joint_trajectory` (FollowJointTrajectory, joints shoulder_pan,
  shoulder_lift, elbow_flex, wrist_flex, wrist_roll). Every goal carries `path_tolerance`, as their
  own straight-line moves do, so the driver does not retime it.
- `/gripper_controller/gripper_cmd` (GripperCommand). Their driver squeezes with its own
  `gripper_torque_limit` (500). It does not read our `max_effort`.
- `/arm/state` is read before every step. The pick starts only in `idle` or `resting`. `holding`
  (their pick holds something) refuses a pick. `busy` before or during a move cancels our goal,
  calls `/arm/stop` and reports `BUSY`.
- `/joint_states` gives the current angles, including the gripper.
- `/arm/stop` is called on any error, except when another program has the arm: a move is refused
  (`BUSY`, nothing sent) while the joints moved in the last 1.5 s without a goal of ours, and a move the
  driver ended with "replaced by another command" reports `BUSY` without `/arm/stop`, so the other
  program's move is not frozen (2026-11-15: a camera rig script of the arm team ran while we
  calibrated). `/arm/torque` is never called.
- Events go out on `/robot/event` with the names hmc_flow.py parses: `grab_success`,
  `grab_failed|CODE|message`, `drop_done`, `drop_failed|CODE|message`. The codes are UNREACHABLE,
  COLLISION, BUSY, HOLDING, NOT_HOLDING, MISSED, DROPPED, NOT_FOUND, NOT_CENTRED, TIMEOUT and FAILED.
  hmc_flow does not call floor_pick's pick: its `pick_bag` goes to `bag_arm.py` (below), which uses
  floor_pick only for the camera look moves.
- Camera frames come only from the arm team's `webcam_view.py` at
  `http://<jetson>:8082/snap.jpg`. `/dev/video0` is never opened, because that would fight their program.

## Task use: `bag_arm.py` (the arm team's pick_bag)

For the task the grasp is the arm team's `/arm/pick_bag` and `/arm/place` (API.md, 2026-11-15), not our
own trajectories: their planner knows the deck, carries the bag on the left at its bottom 0.36 m up and
retries a miss shifted across the handle. `base/jetson/bag_arm.py` gives them the input:
- `pick_bag {"x", "y"[, "top"]}` on /robot/command (robot frame): `/arm/home` first (restarts their
  120 s idle timer, see the hardware assessment 2026-11-15), then with a calibrated gripper camera 2 to 3
  views from above (`look_poses`, the grasp point at least 8 cm over the assumed top) and
  `gripper_camera.triangulate` of the detector's point; the point is used only with a spread under 2 cm,
  a height of 0.15 to 0.50 m and within 8 cm of the Pi's x, y. Else the top is `DEFAULT_TOP` 0.26 m (just
  above the LiDAR plane the bag crossed: too low only grips lower, too high closes on air).
- The goal goes in the arm base frame through `Mount.to_arm`, so `z = top - 0.331` (base_z 0.305 plus our
  measured z_offset 0.026, the same as their launch `floor_z` -0.331 for mount:=robot and `FLOOR_Z`
  in their arm_client.py): their jaws then reach the real top.
- `drop_bag {"x", "y"}`: `/arm/place` at that floor point.
- Events: `grab_success`, `grab_failed|CODE|message`, `drop_done`, `drop_failed|CODE|message`.
  `stop` during a job calls `/arm/stop`.


`floor_pick.py --calibrate-self` uses the floor-tile grout crossing in view as the marker. Its place is
not measured: `gripper_camera.fit_camera_free` fits it with the camera pose and focal length.
1. Coarse: the crossing is seen from the current pose and from up to 9 small joint moves round it
   (`TRACK_MOVES`), tracked as the crossing nearest its first pixel. The fit starts from 160 start
   rotations, tilted up to 60 deg off the tool axis (the real camera looks well below it).
2. Fine: 12 poses from `Planner.self_calibration_poses` (heights 0.22 to 0.40 m, pitch -90 to -30,
   wrist roll -0.8, 0, +0.8 rad) around the coarse place; a sighting more than 250 px from where the
   coarse model expects it is skipped.
Tests (0.5 px noise): the camera within 4 to 6 mm, the crossing within 5 mm, a point 0.31 m up
triangulated within 1 cm (`gripper_camera.triangulate`). The jaw stays as it is during the run.


**Arm frame.** This is the URDF `base` link (`arm/so100_moveit_config/urdf/so100.urdf` on
`origin/arm-system`). Its origin is the bottom of the arm's base plate, since the Base.stl vertices
start at z = 0. z points up. The pan axis is at (0, -0.0452) (URDF line 194, API.md). The arm's
"forward" is -y and its "left" is +x (API.md, `arm_client.user_xy`).

**Robot frame.** This is the wheeled base's frame (x forward, y left, z up from the floor, origin at
the base centre). The LiDAR found the pan axis at (0.012, 0.0145), about 1.5 cm uncertain
(hardware-assessment-2026-11-13.md, line 537).

```
forward = -(y_arm + 0.0452)          left = x_arm
x_robot = 0.012  + cos(yaw)*forward - sin(yaw)*left
y_robot = 0.0145 + sin(yaw)*forward + cos(yaw)*left
z_robot = z_arm + base_z + z_offset          base_z = 0.305, z_offset = +0.026, yaw = 0
```

So, with yaw = 0, `x_robot = -y_arm - 0.0332` and `y_robot = x_arm + 0.0145`.

- `base_z` is the arm base above the floor (tape, 30 to 31 cm).
- `z_offset` is how much higher the real arm sits than its model (1.9 to 3.4 cm, line 581). Their
  calibration does not correct it. It is applied to every target, so a command for a robot-frame
  height h goes to the model's h - base_z - z_offset.
- `yaw` is the arm's turn on the deck. It is assumed 0 (sideways error was within about 1 cm).

All four are CLI parameters (`--pan-xy`, `--base-z`, `--z-offset`, `--yaw`).

**Target.** A target is the point mid-way between the open jaws: 0.092 m down the tool axis, and
0.0295 m towards the moving jaw from the axis, the middle of a 7.5 cm opening at 0.9 rad. Along with
it goes the tool pitch, the angle of the tool axis below the horizontal (-90 deg = straight down).
wrist_roll stays 0. At roll 0 the jaws close front-to-back in the arm's vertical plane, which pinches
both handles of a bag whose wide side faces the robot. A bag turned 90 deg would need roll ±pi/2.
That is not exposed yet.

## Kinematics

- FK chains the URDF joint origins (lines 191 to 250, numbers in `so100_ik.URDF_JOINTS`).
- IK uses a closed-form planar solution (pan, then a 2-link triangle for lift/elbow, then wrist_flex
  for the pitch), both elbow branches. Newton refinement on the full FK follows, so the off-axis
  grasp point lands exactly.
- Unreachable targets raise `Unreachable` with the reason. Nothing is clamped.
- Joint limits are the URDF limits narrowed by the calibrated range: the robot's own
  `~/so100_ws/calibration.json` if present, else the copy of `arm/calibration.json` in the code,
  with calib.py's 20-count margin.
- Accuracy: over a 1 cm grid (x 0.24 to 0.46, y ±0.16, z 0.18 to 0.32, pitch -90 to -20 in 5 deg
  steps), 6373 targets are reachable. The worst FK(IK(p)) error is 1e-7 m.

**Clearance model.** These are assumptions, to be measured.

- Arm links: the bounding box of each link's STL meshes (upper arm, lower arm, wrist, gripper, jaw
  at its commanded opening). The checks use the 8 corners of each box.
- Camera: a box around the gripper body (gripper frame x -0.035..0.04, y -0.075..-0.02,
  z ±0.055), because the camera's exact place is not measured yet.
- Floor: every corner at least 0.02 m above the floor.
- Robot body: the base outline (a 0.40 m square, hmc `FOOTPRINT`) up to a top deck 0.30 m above the
  floor, plus a 1 cm margin. Arm parts must not enter it. The arm's own base and shoulder are exempt.
- The checks use the model height (a positive z_offset is left out), so they still hold if the
  offset is wrong.

**Reach at bag-neck height (0.245 m), measured with this model.** Straight down reaches to 0.27 m
forward of the pan axis (the hardware assessment says 26 cm). At 30 deg down the IK reaches
0.35 m (assessment: 39 cm), but wrist_flex's calibrated limit then refuses it. With all checks,
picks work for robot x from about 0.28 to 0.37 at y near 0.0145, pitch -90 to -45. Nothing below
about 0.18 m above the floor is reachable. **Creep the base until the bag neck is about 0.32 m ahead
of the base centre** (`tools/approach.py`).

## Sequence (`FloorPick.pick`)

Every pose is planned and checked before anything moves.

1. Check `/arm/state`.
2. Joint-space move to the pre-grasp: 8 cm back along the tool axis, with the steepest pitch that
   works. The path is checked every 0.03 rad and goes via the arm team's READY pose if the direct
   path hits something.
3. Open the jaws to 0.9 rad.
4. With `--camera`, run the visual servo (below).
5. Descend in a straight line (5 mm steps) to the neck.
6. Close. If the driver reports `reached_goal`, or the position is below 0.05 rad (as their
   pick_place_node.py:745 does), the result is **MISSED**: reopen, back up along the line, send
   `grab_failed|MISSED|...`.
7. Lift straight up as high as the arm goes there, up to the carry height. The pitch turns towards
   the carry pitch on the way.
8. Wait 0.5 s and read the gripper. Below 0.05 rad means **DROPPED**. Otherwise send `grab_success`,
   as their bridge does once the object is up.
9. Joint-space move across to the carry pose.

`--hover-only` stops after step 3 (above the bag, jaws open, no event).

**Place.** From the carry pose: move above the spot, lower until the grip is 1 cm above the pick
height (the bag stands again), open, back off 8 cm along the tool axis, send `drop_done`, close the
jaws. If the jaws read closed, the result is NOT_HOLDING.

**Speeds.** The URDF gives every joint 1 rad/s; we use 30 % (`--speed`). The grasp point moves at
most 0.05 m/s on average on straight lines. Moves use min-jerk timing, with the peak inside those
limits. Every move's timeout is its duration + 5 s, every gripper command 10 s. In the dry run from
the rest pose, a full pick takes about 41 s of motion: above the bag 14 s, descend 3.8 s, lift
15.7 s, across 7.7 s, plus the gripper. grab_success comes about 35 s after the start; hmc_flow's
grab_timeout default is 90 s and the agent starts it with `-p grab_timeout:=240.0`
(`base/jetson/agent.py`, `FLOW_ARGS`).

**Carry pose.** The planner searches for a pose that keeps the hanging bag inside the footprint and
above the deck. The bag is modelled hanging 0.27 m below the grip (assessment line 559), half size
0.06 m along x and 0.12 m along y (not measured). Result with the defaults: the grip at
(0.14, 0.0145, 0.64), tool level. The bag spans x 0.08 to 0.20, which is inside the footprint, and
its bottom is at 0.37 m: 7 cm above the deck and clear of the LiDAR plane. It hangs over the front of
the top deck, so **anything on the deck in x 0.08 to 0.20 is in the way.** A bag deeper than 12 cm
cannot stay inside the footprint. The planner then reports `IMPOSSIBLE ...` and carries it in front.
For a 24 x 30 cm bag that means a grip at (0.24, 0.0145, 0.62), bag bottom 0.35 m, still above the
LiDAR plane.

## Gripper camera

**What the owner's first frame shows (2026-11-15).** The AX1080P sits behind or under the jaws,
looking along them and down. The jaw fills the bottom ~15 % of the 640x480 image, so detectors
ignore the bottom 18 % (`JAW_ROI`). The floor there is teal with dark grout lines on 0.60 m tiles.

**Model.** Pinhole with fx = fy. The principal point is the image centre (not fitted), and lens
distortion is not modelled. The camera pose is the camera-to-gripper transform `T_gripper_camera`
(OpenCV axes).

**Run the calibration.** On the Jetson, from `~/hmc-follow`, with the arm stack up and `webcam_view.py`
on port 8082:

```bash
python3 tools/gripper_cam_cal.py
```

That is `floor_pick.py --calibrate-self`: no taped marker, the floor joint in view is the target.
It moves the arm and writes `~/hmc-follow/gripper_camera.json`. Add `--dry-run` to print the moves
and write nothing. With a red square at taped spots (metres, robot frame):

```bash
python3 tools/gripper_cam_cal.py --marker 0.36 0.0 --marker 0.38 0.08
```

`bag_arm` uses the file to measure the bag top. A measurement inside the spread, height and shift
limits skips the touch probe. No file, or a rejected measurement, and the touch probe still runs.
The live pick (`bag_method` `camera`) does not touch-probe: it closes at `bag_cam_close_h` and
ignores the grasp z. The ~10 s added on 19 Nov was two Xtion reads of the Wi-Fi address, 5 s each.
That URL now defaults to the LAN, `http://10.0.0.1:8081/stream`.
The model is the camera on the gripper (`T_gripper_camera`). Since 2026-11-18 the webcam used for
the pick is fixed under the deck (`front_camera.json`, `tools/front_cam_cal.py`). A
`gripper_camera.json` fitted to that fixed camera will not pass the gate, and a planner pick falls
back to the touch probe. On the camera pick, `bag_arm` still runs `measure_top` when the file
exists, then the arm pick ignores that z.

**Calibration** (`--calibrate`):

1. Put a marker on the floor at a measured robot-frame position. It can be a red/pink printed square
   (`--marker-kind red`) or a grout-line crossing measured with a tape (`--marker-kind grout`).
   Markers between x 0.34 and 0.40 work best: the arm looks forward and down at them.
2. Run `python3 floor_pick.py --calibrate --marker 0.36 0.0 --marker 0.38 0.08` (`--marker-z` if
   the marker stands on something).
3. For each marker the tool prompts you to place it, then moves to up to 4 poses. Each pose's tool
   axis passes within 20 deg of the marker, at heights 0.22 to 0.30 m, pitch -90 to -45, with side
   offsets. It takes a frame from webcam_view.py at each pose.
4. It fits the 6-DOF camera pose and the focal length (Levenberg-Marquardt, 32 starts covering every
   turn about the tool axis) and saves `~/hmc-follow/gripper_camera.json` with the rms in pixels.
   It needs at least 4 sightings.

Check the rms: a few pixels is fine; 10 or more means a marker was misplaced or misdetected. The fit
absorbs part of a wrong z_offset, so **measure z_offset first.** In tests the fit recovers a known
camera to within 5 mm and 1.5 deg at 0.5 px noise.

**Handle detection.** The default `BagHandleDetector` finds the largest pink/red blob (HSV hue 0-12
or 150-180, saturation ≥ 70, value ≥ 50), else the largest compact blob that differs from the floor
colour, scored by closeness to the centre. The neck point is the blob centroid (`point="top"` for
side views). A learned detector can replace it with `--detector module:Class` (any class with
`.detect(bgr) -> Detection or None`).

**Visual servo** (`--pick X Y --camera`). The arm waits at the pre-grasp with open jaws and takes a
frame. It intersects the handle pixel's ray with the plane at the neck height and moves the target
towards that point in steps of at most 3 cm. It repeats until the correction is under 4 mm (8 tries
at most). Then it descends. No sighting twice gives NOT_FOUND. Still more than 1.5 cm off after 8
tries gives NOT_CENTRED. In tests a 3 cm error is corrected and the pick succeeds. The log prints the
pixel where the grasp point should appear, next to the detected handle.

## Testing on the real robot, step by step

Run on the Jetson:

```
source /opt/ros/jazzy/setup.bash; source ~/so100_ws/install/setup.bash; export ROS_DOMAIN_ID=42
cd ~/hmc-follow   # wherever base/jetson is deployed
export HMC_MAINTENANCE="floor pick bench"   # floor_pick.py moves the arm only for a logged repair (console_gate.py)
```

0. `ros2 topic echo /arm/state --once`: the state must be idle or resting, with no fault.
1. `python3 floor_pick.py --dry-run --pick 0.32 0.0`. Read the plan line (pitch, carry note) and
   the trajectories. The grip column is the robot-frame point between the jaws.
2. Bag in place, base crept so the neck is about 0.32 m ahead: `python3 floor_pick.py --pick X Y
   --hover-only`. Look from the side. The open jaws should be 8 cm above and in line with the neck.
   Measure the miss and correct `--z-offset` / `--pan-xy`.
3. Height check without the bag: `--pick X Y --z 0.30 --hover-only`, then compare the tip height
   with a tape. This is the best z_offset measurement.
4. Full pick `--pick X Y`. Watch the gripper value in `/joint_states` after the close.
5. `--place X Y` puts the bag back down. `--carry` returns to the carry pose.
6. Calibrate the camera (above), then repeat 2 to 4 with `--camera`.

## Safety

- **Power cable.** The Jetson's 10+ m cable leaves the front-left corner. The bag hangs at x 0.08 to
  0.20 during the carry and in front of the robot during the pick. Keep the cable clear of the front
  before a pick. The arm never drives the base. Turning with the bag follows the cable rules in
  memory (never wind it, never run over it).
- **Hands.** The arm moves at 30 % speed but it does not sense people. Keep hands out of the area
  in front of the robot from step 2 on. Have `ros2 service call /arm/stop std_srvs/srv/Trigger` (or
  the arm team's panel) ready. Ctrl-C on floor_pick cancels our goal and calls `/arm/stop`.
  Killing the process any other way leaves the current trajectory running.
- **Their auto-rest.** The arm team's node folds the arm to rest `rest_after_s` (120 s) after its
  last own task. It does not count our moves, and it closes the gripper first. While we hold the bag
  in the carry pose, that fold would swing the bag back over the robot. `floor_pick` aborts with BUSY
  when it sees that start, but only while it runs. A held bag needs `/arm/stop` before the timer runs
  out, or the arm team's agreement on `rest_after_s`. We do not change their parameter.
- **Torque.** We never switch torque. A `torque_off` or `fault` state refuses to start.

## What must be measured on the robot

1. `z_offset` (step 3), and `base_z` if it differs from 0.305.
2. Pan axis xy (±1.5 cm) and yaw. Use the hover test.
3. Top deck height (0.30 assumed) and its front edge (0.20 assumed). Also what stands on the deck
   in x 0.08 to 0.20 (the carry zone) and near the front edge.
4. The bag: hang length below the grip (0.27), depth and width (12 x 24 cm assumed), neck height
   (0.245).
5. The gripper angle when it stops on the handles (0.3 rad assumed for planning). Confirm that
   0.05 rad still means "missed".
6. The camera box on the gripper, for the clearance check. Then run the camera calibration.
7. Whether `~/so100_ws/calibration.json` on the Jetson matches the branch copy (the code prefers
   the robot's file).
