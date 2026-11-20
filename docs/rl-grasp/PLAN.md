# RL bag grasp for the SO-100 arm in MuJoCo: phase A plan

Update, 20 Nov 2026: tier 1 now lives in `sim/rl_grasp` (see `docs/rl-grasp/README.md` and `WORKLOG.md`). The research below is the 20 Nov plan, kept as written, with three corrections the code had to follow:

- MuJoCo 4.0.1 rejects the legacy `<composite>` element (`unrecognized element`). Rope and ribbon handles are `flexcomp dim="1"` rods. This plan already calls that the cable equivalent of composite.
- The sim's position-actuator kp, kv and forcerange are the bench stand-ins. kp is 40 and kv is 2, up from the bench's 20 and 1, so the -60 deg reach holds against gravity. They are not an STS3215 stall torque and not a `mujoco.sysid` fit. `mujoco.sysid` was not run.
- Training uses MuJoCo, CPU PyTorch and numpy only. PPO and SAC are in this repo. Gymnasium, stable-baselines3, Menagerie, MJX, Warp, Isaac, ManiSkill and Brax are not used.
- The default camera and dynamics draws are a narrow band around the placeholder (a few centimetres, 2% of focal length, mass and friction ±20%, actuators ±10%). `--hard` is the wide draw in section 5.3 and 5.7. The wide draw made the scripted pick miss or collide, because the close only works when the estimated wall is about 0.29–0.32 m.

The sentence below, that nothing was committed, describes the research session. The tier-1 package is a later change.


Written 2026-11-20 12:15 ICT for Teerapat (RoboCup@Home Help Me Carry robot). This is research and a plan only. No training pipeline was built. Nothing was committed, deployed or moved on the robot. The only thing installed was `mujoco` in `/workspace/rl-grasp-2026-11-20/venv`.

Approval used: Teerapat, 20 Nov 12:06 ICT, "ทดสอบหยิบ 10 รอบและทำ RL ใน MuJoCo อนุญาตใช้ MuJoCo". Everything else new is listed under "Proposals" and is **not** used.

Correction, 20 Nov 12:10 ICT: the front (bag) camera is now a wide fisheye "HD USB Camera" (USB 32e4:9320, 640x480 MJPG). The 18 Nov pinhole calibration in `front_camera.json` is **invalid** for it, and this plan does not depend on those numbers. The sim camera is wide-FOV with lens distortion, and its intrinsics and pose are randomised.

---

## 1. Sources

**Repo** (`clytie0x/robocup-home-education`, `origin/main` = `cc8a70b`, read with `git show origin/main:<path>`; the local checkout has an unrelated uncommitted change in `base/tools/deploy.sh`, which was left alone)
- `arm/so100_moveit_config/urdf/so100.urdf`: links, joints, limits, masses, TCP.
- `arm/so100_moveit_config/config/joint_limits.yaml`: velocity and acceleration limits.
- `arm/so100_moveit_config/meshes/*.stl`: 13 binary STL meshes.
- `arm/so100_driver/so100_driver/bag_camera.py`: the camera bag pick (`CameraBag.pick`, the docstring describes steps 1–4, constants `TRIES`, `REACH_WALL`, `LIFT_SPEED` and others).
- `arm/so100_driver/so100_driver/pick_place_node.py` lines 113–221 (`bag_*` parameters, `bag_model` coco|tuned) and 1003–1066 (`do_pick_bag`).
- `arm/so100_driver/so100_driver/ik.py`: gripper geometry constants.
- `arm/so100_driver/so100_driver/motion_profile.py`: speed profiles (safe and fast).
- `arm/so100_driver/so100_driver/driver_node.py`: 50 Hz `/joint_states`, `max_temp` 60.
- `arm/so100_driver/so100_driver/feetech.py`: STS3215 bus, 1 Mbaud, goal position plus speed.
- `arm/README.md`, `arm/API.md`: console gate (`HMC_CONSOLE_JOB`) and the `/arm/pick_bag` API.

**Jetson** (`robotpsu@jetson-orin.tail09b237.ts.net`, read-only: `ls`, `cat`, `scp` *from* the Jetson. No process, GPU, camera or param was touched.)
- `~/so100_ws/src/so100_moveit_config/{urdf,meshes}` was copied to `jetson-copy/`. The URDF and all 13 STLs are **byte-identical** to the repo (sha256 compared; the URDF sha256 starts `f12e252a37339a6d`).
- `~/so100_ws/src/VERSION.json`: deployed commit `cc8a70b`, 2026-11-19 22:06 ICT.
- `~/hmc-follow/models/`: `bag11n-seg-v3_480x640.{engine,onnx}` is present.
- `~/hmc-follow/runs/*.json`: 3 real camera-pick trials from 19 Nov, with phase timings (see section 2.5).
- `~/hmc-follow/front_camera.json`: the old pinhole calibration. It was read but **is not used** (it is invalid after the camera swap).
- No PyTorch on the Jetson (`import torch` fails). No training on the Jetson is planned.

**Box measurements** (this folder)
- `loadtest/load_urdf.py`: loads the team URDF in MuJoCo 4.0.1 and steps it.
- `loadtest/so100_from_urdf.xml`: the MJCF that MuJoCo saved from it.
- `loadtest/bench.py` and `loadtest/bench.txt`: speed of three scene variants on one core.
- `loadtest/par.py` and `loadtest/par.txt`: speed with 1, 4 and 8 processes.
- `loadtest/torchbench.py` and `loadtest/torchbench.txt`: PyTorch CPU update cost.
- `refs/modeling.txt`, `refs/XMLreference.txt`, `refs/changelog.html`: MuJoCo docs as fetched on 20 Nov 2026.

**MuJoCo docs**
- Modeling, "Deformable objects": https://mujoco.readthedocs.io/en/latest/modeling.html#deformable-objects
- Modeling, "URDF extensions": https://mujoco.readthedocs.io/en/latest/modeling.html#urdf-extensions
- Modeling, "Deformables" tuning guide: same page, `#deformables`
- XML reference, `flexcomp`: https://mujoco.readthedocs.io/en/latest/XMLreference.html#body-flexcomp
- Changelog: https://mujoco.readthedocs.io/en/latest/changelog.html (latest is 4.0.1, 11 Nov 2026; flex has existed since 3.0)

**Low-cost-arm sim-to-real RL (published)**
- [P1] Squint, arXiv 2602.21203 (https://arxiv.org/html/2602.21203v1). An SO-101 trained in ManiSkill3 with heavy domain randomisation, zero-shot to the real arm: 91.3% (73/80) across 8 tasks, Lift Cube 10/10 (SAC 10/10, PPO 6/10). Removing colour jitter dropped it to 58/80. About 15 min on one RTX 3090.
- [P2] ManiSkill3, arXiv 2410.00425 (https://arxiv.org/abs/2410.00425). A Koch low-cost arm, PPO from RGB plus joints, delta joint-position control, domain-randomised: 22/24 = 91.6% real cube pick. 15M samples, 256 envs, about 1 h on an RTX 4090.
- [P3] lerobot-sim2real tutorial, SO-100 `SO100GraspCube-v1` (https://github.com/StoneT2000/lerobot-sim2real/blob/main/docs/zero_shot_rgb_sim2real.md): 25–40M steps, about 1 h on a 4090.
- [P4] State representations for sim2real grasping, arXiv 2309.11984. Numerical-state RL policies matched non-learning baselines on the real robot (50/50 picks). End-to-end image RL dropped to 24%.
- [P5] Force-control RL in MuJoCo with domain randomisation, zero-shot to a real gripper, arXiv 2311.07245. Domain randomisation was "crucial".
- Practitioner evidence (not peer-reviewed, cautionary):
  - [N1] https://github.com/lachlanhurst/so100-mujoco-rl: SO-100 MuJoCo PPO reach tasks; projecting the object into the camera analytically was "much faster" than rendering plus YOLO.
  - [N2] https://ggando.com/blog/so101-hil-serl/: SO-101 state-based SAC reached 100% in MuJoCo, but RGB and segmentation-plus-depth policies **did not transfer zero-shot**.
  - [N3] https://github.com/twu3202/soarm101-vla: SO-101 MuJoCo/ManiSkill PPO/SAC, about 69% single-cube in sim, **zero real transfer**; blamed on the sim2real dynamics gap.
  - Every positive result above is for **rigid cubes or cans**. I found no published SO-100 RL result for a **soft bag with handles**. Its transfer is unknown.

## 2. Real arm facts

### 2.1 Kinematics (URDF, identical on the Jetson and in the repo)
| joint | URDF range (rad) | MoveIt max vel / acc |
|---|---|---|
| shoulder_pan | -2 .. 2 | 1.5 rad/s / 6 rad/s² |
| shoulder_lift | 0 .. 3.5 | 1.5 / 6 |
| elbow_flex | -3.14158 .. 0 | 1.5 / 6 |
| wrist_flex | -2.5 .. 1.2 | 1.5 / 6 |
| wrist_roll | -3.14158 .. 3.14158 | 1.5 / 6 |
| gripper (jaw) | -0.2 .. 2.0 | 3.0 / 12 |

- URDF `effort="35"` and `velocity="1"` on every joint are placeholders, not motor data.
- The STS3215 stall torque is **unknown here**. It must be measured or taken from the Feetech datasheet before actuator limits are set in sim.
- Link masses come from the URDF: shoulder 0.119, upper_arm 0.162, lower_arm 0.148, wrist 0.066, gripper 0.093, jaw 0.020, base 1.0 kg. MuJoCo total: 1.609 kg.
- TCP: fixed, 0.09 m along the gripper -y axis.

### 2.2 Gripper (`ik.py`)
- One moving jaw.
- Fixed finger inner face at x = 0.008 m (gripper frame). Finger tips at y = -0.106. Grasp centre at y = -0.092.
- Widest useful jaw angle `JAW_G_MAX` 0.9 rad, which gives a 7.5 cm gap. Finger length about 3 cm (`GRIP_DEPTH` 0.028).
- Planner object limit `MAX_OBJECT` 0.065 m (rigid picks).

### 2.3 Control interface
- The driver publishes `/joint_states` at 50 Hz (`rate` 50).
- Servos take goal position plus goal speed (`sync_write_goals_speed`, speed 0..3400 raw).
- The driver refuses moves at or above 60 °C. Picks wait at or above 55 °C (`PICK_TEMP_C`).
- Motion programs run only when started by the console (`HMC_CONSOLE_JOB`), otherwise `REFUSED`.

### 2.4 How the current bag pick works (`bag_method=camera`, `bag_camera.py`, PR16/PR17)
1. **Look.** The arm goes to `LOOK`. The bag is segmented in the fixed front webcam image (`bag_model`: `coco` by default, or `tuned` with `bag_tuned_engine`, n-v3 built on the Jetson). Two edge estimates, the rim at `bag_cam_rim_h` (0.27 m) and the foot on the floor, are reconciled. The camera centre is used if confidence ≥ 0.55, otherwise the Pi LiDAR goal. A LiDAR disagreement over 0.10 m gets a re-look.
2. **What to grasp.** The bag's front wall, `bag_cam_half_depth` (0.055 m) in front of the centre. With the tuned model, the grasp shifts sideways onto the **handle centroid** and keeps the rim's forward distance. Tips close at `bag_cam_close_h` (0.19 m above the floor), jaws pitched -60°, and the wrist roll lines the closing axis up with the wall normal. If the wall is more than `REACH_WALL` (0.32 m) from the pan axis, the pick fails with "move closer".
3. **Move.** One planned trajectory: above, down, close (the only pause), then straight up at `LIFT_SPEED` (0.25 rad/s). On a miss it opens, backs off, looks again, and tries the next of `TRIES` (+0, +1.5 cm, -1.5 cm at -60°; +3 cm at -50°).
4. **Raise and swing to the side** (`CARRY_THETA` 75°). The Xtion and the front camera must confirm the bag left the floor. Otherwise the result is `SUCCESS_UNVERIFIED`.

Speeds: free 0.8, held 0.4, put 0.6 rad/s on the fastest joint. Profile `safe` is the default; `fast` is only for a supervised bench test.

### 2.5 Real outcomes so far (`~/hmc-follow/runs`, 19 Nov)
| trial | result | look | grasp | lift | raise | carry |
|---|---|---|---|---|---|---|
| 151738-1 | success | 4.6 s | 11.7 s | 17.6 s | 17.7 s | 8.0 s |
| 164855-1 | COLLISION (shoulder_lift stopped 5.9° short) | 4.6 | 13.5 | – | – | – |
| 172917-2 | success | 4.2 | 13.9 | 16.2 | 1.1 | 18.6 |

- That is 2 of 3. The look-to-lift-done time is about 34–36 s in the two successes. The per-pick time to beat is therefore about 35 s to lifted, about 52–60 s including carry.
- The sample is tiny. The 10-trial test (step 21) is the real baseline.

### 2.6 Bags and scene
- The front camera frame `/workspace/robot-status-2026-11-20/front-new-cam.jpg` shows two bags: a **paper shopping bag with rope handles** and a **soft printed bag with flat ribbon handles**. The arm's orange gripper is visible at the bottom of the frame.
- Owner tape: handle top 0.36–0.37 m.
- Parameter defaults: rim 0.27 m, bag width 0.18 m, depth 0.11 m.
- The bag's mass and the handles' stiffness are **unknown** and must be measured: a kitchen scale, and the handle sag under its own weight.

### 2.7 Cameras
- **Front (bag) camera:** now a wide fisheye at 640x480. Its intrinsics, distortion and pose are **unknown until recalibrated**. Mounting info from the owner on 18 Nov (lens about 0.89 m above the floor, looking down) may also have changed; treat it as unknown.
- **Xtion:** depth only from about 0.6 m (arm README), used to confirm lift-off.

## 3. Sim model choice

**Recommendation: use the team's own URDF, converted once to MJCF.** Verified on the box:
- A raw `MjModel.from_xml_string(urdf)` **fails** because `package://` mesh URIs can't be resolved.
- Fix: add `<mujoco><compiler meshdir="…/meshes" balanceinertia="true" discardvisual="false" fusestatic="false"/></mujoco>` under `<robot>` and strip the `package://so100_moveit_config/meshes/` prefix. This is the documented URDF extension.
- Result: it **loads**, with 9 bodies, 6 joints, 20 geoms, 13 meshes, and the correct joint ranges. Passive stepping runs at about 99k steps/s (1 core).
- Saved as `loadtest/so100_from_urdf.xml`. Following the MuJoCo docs, convert once and then edit the MJCF.

Fixes still needed in the MJCF:
1. **Actuators:** URDF has none. Add `position` actuators per joint with kp, kv and forcerange fitted to the real servos. Gains, stall torque and backlash are unknown; fit them with logged real moves. The MuJoCo docs point to `mujoco.sysid`, which ships inside the approved `mujoco` package.
2. **Collision geometry:** MuJoCo convexifies mesh geoms. The jaw needs simple box or capsule pads on the finger insides for stable pinch contact. Proper fingertip friction (`condim` 4 or 6) is also needed.
3. **Effort and velocity:** replace the placeholder 35 Nm and 1 rad/s with measured values, and cap at the MoveIt velocities.
4. **Gripper joint mapping:** URDF jaw radians must map to the driver's jaw convention, already used in `ik.jaw_gap`.
5. **Mounting:** put the arm at its deck pose on the robot (`deck_z` -0.068 m below the arm base, `mount:=robot`). The base pose relative to the floor and camera must be measured. Front, back, left and right deck offsets default to 0 in the params; the true values are unknown.

An outside model (MuJoCo Menagerie `trs_so_arm100`, Apache-2.0) exists, but it is a **proposal only** (see section 8). Our URDF loads fine, so it is not needed.

## 4. Bag model

MuJoCo options (docs: Modeling → Deformable objects / Deformables):

| option | what | pros | cons |
|---|---|---|---|
| **A. Rigid body + handle chains** | Bag body as a box with a free joint. Each handle is a chain of small capsules with ball joints (or a `dim=1` flex rod), ends welded to the rim. | Fast (measured 24.9k steps/s on 1 core with arm and one 6-link handle, nv=30), stable, easy to randomise. | The wall doesn't crush when pinched; the paper wall pinch is approximated. |
| **B. Shell flex (`flexcomp dim=2`)** | Bag walls as a thin shell, continuum `young/poisson/thickness/elastic2d` or edge equality. Handles as `dim=1` rods (DER bending/torsion since MuJoCo 3.x). | Real pinch and crumple of a soft bag; contact force spread over vertices. | Slower: a flat 8x6 test grid already ran at 12.8k steps/s (nv=150), and a closed bag will be more. The docs say flex elasticity needs the `discrete` integrator (implicit/implicitfast give a runtime error for continuum elasticity), and PGS and noslip are unsupported. Frictionless `passive`/`ipc` contact modes must stay **off** for grip. Tuning is harder. |
| C. Composite (legacy) | `composite` macros (cable, etc.). | – | Superseded by `flexcomp` for deformables; use only the cable equivalent via flex. |

**Recommended staged approach:**
1. **Stage 1, option A.** A rigid bag box with a compliant handle: a capsule chain or flex rod with randomised stiffness, plus a thin compliant "rim strip" (a few hinged plates) for the paper-wall pinch. This is the policy-learning workhorse.
2. **Stage 2, option B for evaluation only.** A shell-flex paper bag and a soft fabric bag with ribbon handles, used to stress-test the stage-1 policy before any real trial. Train on B only if A-trained policies fail in B.

Physical numbers to measure on the real bags (unknown now): mass empty and with typical contents, handle length and sag, wall stiffness (deflection under a known push), and jaw-paper friction.

Note on what was measured: the flex numbers above come from a **flat grid**, not a bag, and no arm-to-flex grasp contact was tested. These are throughput indications only.

## 5. RL design

### 5.1 Two tiers, smallest first

**Tier 1: "grasp-parameter policy" (one decision per try).**
- Observation: bag estimate (see 5.2).
- Action, bounded and continuous: forward offset, sideways offset, close height, pitch, roll offset, and approach speed scale.
- The *existing* scripted trajectory (`CameraBag` steps 2–4, re-implemented in sim) executes it.
- This replaces the hand-picked `TRIES` and offsets with a learned, observation-dependent choice. It is a contextual bandit, so it trains fast and keeps the proven safety checks (reach, IK feasibility, the `tip()` dry run, miss detection) on the real robot.
- Algorithm: own small PPO (or CEM as a sanity baseline) in PyTorch.

**Tier 2: closed-loop end-effector-delta policy at 10 Hz.**
- Starts from the tier-1 pre-grasp pose and handles the last 10–15 cm: descend, close, lift.
- This is where speed and robustness to bag sway can be learned.
- Algorithm: own SAC (sample-efficient; in [P1], SAC beat PPO on real Lift Cube 10/10 vs 6/10) or PPO.

Why this split: [N2] and [N3] show that end-to-end policies on these arms often fail to transfer. [P4] shows numeric-state policies transfer best. Tier 1 changes the least on the real robot.

### 5.2 Observations (only what the real robot has)
- Joint positions for 6 joints (from `/joint_states`, 50 Hz) and joint velocities (finite-differenced).
- Gripper state: jaw angle, plus the gripper load % that the driver already reads.
- Bag estimate from the detector pipeline, in the robot frame:
  - bag centre (x, y), handle point (x, y, z) when `tuned` gives a handle centroid, otherwise NaN mask plus flag
  - rim width, standing flag, detector confidence
  - source flag (camera or LiDAR fallback)
- Tier 2 only: the relative vector from the TCP to the handle point (estimate refreshed at the detector rate, held between updates), and the time since the last detection.
- **No images into the policy.** The sim gives the policy what the detector would give: true pose, then degraded by the camera model below. [N1] reports that projecting analytically is much faster than rendering, and it avoids the RGB gap ([N2], [N3]).

### 5.3 Camera model in sim (new fisheye)
- Project the true handle and rim points through a **wide-FOV fisheye model**: an equidistant or Kannala-Brandt-style polynomial with k1..k4, 640x480.
- Add pixel noise and occasional dropouts, then back-project with the *nominal* model. This reproduces the calibration error the real pipeline will have.
- Randomise intrinsics, distortion, and camera pose (position, pitch, yaw, roll) **around the new calibration once it exists**. Until then, use wide uniform ranges.
- Nothing depends on the old pinhole numbers (fx 805.7 is invalid).
- Optional later, for evaluation only: MuJoCo's own offscreen render through a wide camera to check that the segmenter sees the sim bag. That would need the trained YOLO model in the loop, which is slow and not needed for training.

### 5.4 Action space
- Tier 1: 6 bounded scalars (above). The executor clamps them to the reachable set and rejects infeasible ones, which counts as a fail with no motion.
- Tier 2: TCP delta (dx, dy, dz) ≤ 1 cm per 0.1 s step, plus d-pitch and d-roll ≤ 3°, plus a gripper open/close command. These go through our own damped-least-squares IK to joint targets and are sent as position goals (matching the real `goal position + speed` interface). Joint targets are clamped to URDF ranges and MoveIt velocity limits.
- The alternative, joint-delta actions as in [P2], is simpler to transfer but harder to keep safe near the deck.

### 5.5 Reward (tier 2; tier 1 uses the episode outcome)
- **+10 on success** (below). **-5 on collision** with the robot body, deck, floor or bag wall (as opposed to the handle) above a force threshold.
- **-2 on dropping the bag** after a close.
- Shaping: -‖TCP − handle point‖ before the close, and +lift height while the bag bottom is off the floor.
- Time: -0.01 per step, which rewards speed.
- Action smoothness: -c·‖Δa‖².
- Joint limit proximity penalty. Motor effort penalty, as a proxy for servo heat (the real arm waits at 55 °C).

### 5.6 Episode
- Tier 2: at most 150 steps at 10 Hz (15 s) from the pre-grasp pose.
- Tier 1: one try, the scripted move to "lifted 3 s", at most about 30 s sim time. The real look-to-lift is about 34–36 s; the target is a lower sim and real time.

### 5.7 Domain randomisation (per episode)
- **Bag:** pose (x, y within the reachable band in front of the robot, yaw ±30° around facing), size (width, depth, rim height around the measured bags), handle length, stiffness and droop, mass (empty to loaded; range to be set from measurements), and the friction of jaw-paper and bag-floor contacts.
- **Arm:** actuator kp, kv and forcerange (±30% around the sysid fit), joint damping, frictionloss, backlash (a dead-band on the jaw and wrist), and control latency (0–2 control steps).
- **Camera and detector:** intrinsics, distortion, pose (5.3), pixel noise, handle-centroid bias, missed detections, and confidence. Detector latency is 0–300 ms; the real value is unknown and should be logged.
- **Floor:** height offset ±1 cm.
- Colour and texture: irrelevant, because there are no images in the policy.
- Use Automatic Domain Randomisation (start narrow, widen as success grows; described in arXiv 2509.16434 and OpenAI's work) only if fixed ranges stall.

### 5.8 Success in sim (mirrors the real test)
- Jaws closed on the **handle or rim** (contact with the handle or rim geoms on both fingers), the bag bottom ≥ 2 cm off the floor, held **3 s** without slip over 1 cm, no collision, and done within a time limit.
- Sim pass bar to consider a real trial: **≥ 95%** over 500 randomised held-out episodes, and ≥ 90% in the stage-2 flex bag. Sim is optimistic: [P1] dropped from 96.1% in sim to 91.3% on hardware on rigid objects.
- Median time to "lifted" must also be **below the current scripted pick in the same sim** (measured, not assumed).

## 6. Compute and time estimate

**Box (measured 20 Nov):**
- 8 vCPU Intel Xeon, 15 GB RAM, **no GPU** (no `nvidia-smi`), Python 3.13.5, 90 GB free disk.
- `mujoco` 4.0.1 was installed in `venv/`; it imports and steps.
- PyTorch is **already on the box**: `torch 2.15.0+cpu` in `/workspace/bag-dataset/.venv`, used for the yolo11 fine-tune.

Measured MuJoCo throughput (2 ms timestep, our arm with position actuators):
| scene | 1 core | 8 processes |
|---|---|---|
| arm only | 66.7k steps/s | – |
| arm + rigid bag + 6-link rope handle | 24.9k steps/s | **148k steps/s total** |
| arm + flat 8x6 flex shell (implicitfast, edge equality) | 12.8k steps/s | not measured |

PyTorch CPU: 100 Adam updates, batch 4096, 2x256 MLP = 0.62 s, so learner cost is small next to simulation.

**Estimate (assumptions marked):**
- Control at 10 Hz with 2 ms physics gives 50 physics steps per policy step. 148k/50 ≈ **3k policy steps/s** at best on the box for the rigid bag.
- Grasp contacts and Python env overhead will lower that. Assumption: **1–2k steps/s**, to be measured in phase B.
- Tier 2 sample budget: published rigid-object PPO used 15M ([P2]) to 25–40M ([P3]) samples. SAC usually needs fewer, but no number was measured for our task.
  - **PPO, 15–40M steps:** about 2–11 h on the box at 1–2k steps/s.
  - **SAC, guess 1–5M steps:** a few hours on the box, limited by update cost.
- Tier 1 (bandit, about 15–30 s sim per try, which is 7.5–15k physics steps): 8 cores give about 10–20 tries/s. 50k tries ≈ **1–2 h**.
- Flex-bag evaluation is 2x or more slower per step than rigid (measured on a flat grid only); fine for 500-episode evaluations.
- **Laptop:** specs (CPU, GPU) are **unknown**. MuJoCo CPU sim would scale with its cores. A CUDA GPU would only speed up the network updates, not CPU MuJoCo, so do not expect a big gain unless MJX or Warp is used (a proposal).
- **Jetson:** not allowed now (shared with teammates). It is the deployment target only.

## 7. Phases with pass/fail tests

| phase | work | pass | fail → action |
|---|---|---|---|
| **A** (this doc) | research, URDF load test, benchmarks | the plan is accepted by Teerapat | revise |
| **B1** sim arm twin | MJCF from URDF, actuators, finger pads, mount on deck; replay a logged real joint trajectory | sim joint tracking error vs the real log ≤ a threshold set after the first fit (target ≤ 2° RMS), using `mujoco.sysid` | refit gains and backlash |
| **B2** bag model | rigid bag + compliant handles (option A), measured mass and sizes | a scripted pick in sim reproduces the real behaviour: the 19 Nov success geometry lifts, and an offset over the jaw gap misses | adjust contact and stiffness |
| **B3** scripted baseline in sim | port `CameraBag` steps 2–4 and `TRIES` as the baseline policy | baseline sim success and time measured over 500 episodes (the number we must beat) | – |
| **C1** tier-1 RL | own PPO or CEM over grasp parameters | beats the baseline success by a clear margin, or matches it with a lower time, over 500 held-out episodes; ≥ 95% | more domain randomisation or features |
| **C2** stage-2 check | evaluate in the flex bag (option B) | ≥ 90% | train with B in the mix |
| **D** real dry runs | (with Teerapat at the robot) tier-1 outputs shown with `bag_hover_only` = jaws open at the close height, **no grasp**; compare the asked and real gap middle with a plumb line | gap-middle error within ±1.5 cm (a 7.5 cm gap leaves that margin; see `TRIES`) | fix the camera calibration or sim mount |
| **E** real slow trials | 10 picks at the `safe` profile, the same protocol as step 21 | ≥ 8/10 lifted and held 3 s, no collision, median time ≤ the scripted baseline's | log, add to domain randomisation, retrain |
| **F** tier 2 | only after E passes; same B–E gates for the closed-loop policy | same as E, faster | stay on tier 1 |

## 8. Sim-to-real plan (later; nothing here is run now)

1. **Preconditions:**
   - Teerapat at the robot with a hand on the motor switch, battery charged, console idle, no teammate job running on the arm (check first, never kill others' jobs).
   - Back up before any deploy.
   - Front fisheye camera recalibrated with distortion.
2. **Policy as a parameter file, not new motion code:**
   - Tier 1 runs on the Jetson CPU as a tiny MLP. Running it as plain numpy matrix maths needs no PyTorch on the Jetson.
   - It only returns grasp parameters. The **existing** `CameraBag` pipeline executes them, with all its checks: reach, dry run, miss detection, temperature waits, and the console gate.
   - It is enabled behind a new param (for example `bag_grasp_policy`: off|tier1, default off), switched on only for the supervised session and back off after.
3. **Ladder:**
   1. Hover only (`bag_hover_only`), no close.
   2. Close, no lift.
   3. Lift 2 cm.
   4. Full lift and hold 3 s.
   5. Carry.
   - All at the `safe` profile first. The `fast` profile only after 10 safe successes, matching the standing rule on faster arm motion.
4. **Logging:** every trial writes the existing `runs/*.json` plus the policy inputs and outputs, so each failure can be replayed in sim and added to domain randomisation.
5. **Abort rules:** any collision code, servo ≥ 55 °C, or detector confidence below `CONF_MIN` means fall back to the scripted grasp or stop.

## 9. Proposals needing Teerapat's permission (none installed or used)

| # | proposal | why | alternative without it |
|---|---|---|---|
| 1 | **PyTorch (CPU) in the RL venv** `/workspace/rl-grasp-2026-11-20/venv` | Own small PPO, SAC and CEM. PyTorch is already used for the yolo11 fine-tune; it is a reinstall in a new venv, not a new library. Flagged anyway because it would be a new install. | Reuse the existing `/workspace/bag-dataset/.venv` interpreter (torch 2.15.0+cpu) with `mujoco` added there, or write PPO/CEM in numpy |
| 2 | Gymnasium (API only) | Standard env interface for tools and tests | Our own minimal `reset()/step()` class (recommended; no need) |
| 3 | stable-baselines3 | Tested PPO and SAC, fewer bugs | Our own PPO and SAC in PyTorch (recommended) |
| 4 | MuJoCo Menagerie `trs_so_arm100` MJCF (Apache-2.0) | Hand-tuned collision geoms and actuators to compare against | Our own URDF (loads fine; recommended) |
| 5 | MJX / MuJoCo Warp + JAX or Warp, GPU | Massively parallel sim (thousands of envs) | CPU multiprocessing on the box (enough for tier 1, slower for tier 2) |
| 6 | Cloud or laptop GPU time | Faster tier-2 training | The box CPU; no spending |
| 7 | OpenCV (fisheye calibration on the box) | Fisheye model and calibration fitting for the new camera | Our own numpy fisheye projection in sim. Real recalibration is a separate task with its own tools. |
| 8 | Isaac Lab / ManiSkill3 / Brax | The published SO-100/101 sim2real results ([P1], [P2]) used ManiSkill3 | Stay on MuJoCo (approved) |

`mujoco.sysid` ships inside the approved `mujoco` package, but it is a new module. I will mention it before using it.

## 10. Risks

- **Soft-bag transfer is unproven.** All published low-cost-arm sim-to-real RL is on rigid cubes and cans ([P1]–[P3]). A paper wall pinched by a 7.5 cm one-jaw gripper may behave unlike the sim. Mitigation: tier 1 on the existing executor, the flex evaluation, and the hover-first ladder.
- **Detection is the bottleneck, not motion.** If step 21 shows misses come from the bag pose, the camera calibration and data matter more than RL. Step 21 should come first.
- **The new fisheye camera is uncalibrated.** Until recalibrated, the real handle estimate's error is unknown, and domain-randomisation ranges are guesses.
- **Unknown actuator dynamics:** STS3215 torque, backlash and latency. Mitigation: sysid from logged moves (phase B1).
- **Practitioner reports show zero transfer** with end-to-end policies ([N2], [N3]). Hence numeric states and a bounded tier 1.
- **Servo heat:** fast, repeated real trials hit 55–60 °C limits. Mitigation: an effort penalty and the existing temperature waits.
- **Shared Jetson:** no training and no long tests there. Check others' jobs first.
- **Throughput figures are first measurements** on toy scenes; the real env may be several times slower.
