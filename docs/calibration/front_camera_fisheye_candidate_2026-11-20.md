# Candidate only. Not a deployed calibration.

`front_camera_fisheye_candidate_2026-11-20.json` keeps `"calibrated": false`. This note is how those numbers were estimated on 2026-11-20. It is not the robot's `front_camera.json`, and nothing on the robot was changed. The geometry code can read `dist_model` and `dist`. Copying this file onto the robot still waits until the owner checks the lens height and the tile size and sets `calibrated` true.

The estimate follows.

# New front (bag) camera: calibration estimated without a board — 2026-11-20

**Status: CANDIDATE.** `front_camera.candidate.json` keeps `"calibrated": false`. The owner hasn't checked it, and nothing on the Jetson was changed.
Times are ICT (UTC+7).

## What was done (read-only)
- Grabbed 12 frames at 12:14–12:17 with `curl http://127.0.0.1:8082/snap.jpg` over SSH (`frames/f01..f12.jpg`), plus the 12:10 frame (`frames/f00_1210.jpg`).
  /dev/video0 was never opened, no process was started or stopped, and nothing moved. The 12 frames are static: phase-correlation shift ≤ 0.06 px.
  The 12:10 frame is shifted 1.24 px sideways against them, so something moved slightly between 12:10 and 12:14. The fit uses the 12-frame median (`work_median.png`).
- Lens model: OpenCV `cv2.fisheye` (equidistant + k1, k2), fy = fx, no skew.
  Pose convention is the same as `front_camera.py`: R = Rz(yaw) Ry(pitch) Rx(roll) R0.
- **Fit (plumb-line + square tile grid in one step):** grout ridges were found on a high-pass image.
  An ICP-style loop projected the model grid, searched ±2.5–10 px perpendicular for the ridge, and ran Levenberg–Marquardt on point-to-curve distances (Huber).
  Unknowns: f, cx, cy, k1, k2, pitch, roll, yaw (relative to the grout), height (in tiles) and the grid offsets.
  Bags, tape, gripper, chair, cabinet and frame corners were masked (`evidence/00_mask_boxes.jpg`). Code is in `tools/`.
- **Scale:** the tile size comes from the 18 Nov frames. A homography from the 13 taped red discs (d1 + d2; leave-one-out 0.3–1.5 cm) was mapped onto 3 grout spacings: 0.309 / 0.284 / 0.295 m.
  → **tile = 0.296 ± 0.007 m**. This is consistent with 0.30 m nominal tiles, which nobody has confirmed. The old calibration alone gives 0.29–0.315.

## Result (model B: fisheye k1,k2 + free principal point)
| | new camera (candidate) | 1σ | old AX1080P (18 Nov) |
|---|---|---|---|
| f (px, 640×480) | **522.7** | ±7 | 805.7 (pinhole) |
| cx, cy (px) | **296.7, 241.1** | ±2.2, ±3.6 | 319.5, 239.5 (fixed) |
| k1, k2 (fisheye) | **−0.120, +0.054** | ±0.013 each (correlated) | none |
| FOV H × V | 73.0° × 53.8° | | 43.3° × 33.2° |
| pitch (down) | **46.95°** | ±0.48° | 61.37° |
| roll | **0.96°** | ±0.27° | 0 (fixed) |
| yaw, relative to the floor grout | **0.76°** | ±0.39° | 1.40° (relative to the robot) |
| lens height z | **0.793 m** (2.680 tiles) | ±0.020 m | 0.89 m (owner tape) |
| x, y | **not measurable here**: 0.33 / 0.0 copied from the old file | unknown | 0.33 / 0.0 |

1σ = leave-one-line-out jackknife combined with the spread over distortion models (k1 only / k1,k2 / k1–k3: f 514–523, pitch 46.9–47.8°, h 2.656–2.680 tiles). The formal covariance is 3–10× smaller and is not quoted.
- Floor coverage along the centre column (from the lens's ground point): bottom row 0.23 m, image centre 0.74 m, top row 2.19 m. One pixel ≈ 2.2 mm on the floor at 0.52 m.

## Validation
- **Fit:** 456 ridge points on 12 lines (5 lateral, 7 forward). RMS 0.49 px, median 0.26 px, p90 0.68 px.
- **Hold-out:** fit on the even lines, test on the odd lines, and the reverse: RMS **0.62 / 0.55 px** (median 0.35 / 0.27). With cx, cy fixed at the image centre, hold-out was 2.8–3.4 px and lines were lost, so that model was rejected.
- **Independent near-field line X = 0** (not in the fit, 0.6–0.9 m from the lens): median 0.6 px on the left, 0.4 px on the right.
  - The bottom-right forward line at 0.48–0.64 m: median 1.8 px, with an RMS of 7.4 px from a few bad ridge hits.
  - The pick zone (bottom third) has few visible grout ridges, so it is the least-constrained part. A board would fix this.
- **Tape centre line (independent object):**
  - Maps to a straight floor line: RMS 1.8 mm over 0.55–0.98 m (the raw image curve is removed).
  - It runs −0.36° to the grout; on 18 Nov it was −0.33°.
  - Tape-to-next-grout distance is 0.102 m; the 18 Nov disc homography gave 0.109 m. This checks scale and lateral geometry within 7 mm.
- **cv2 equivalence:** the model matches `cv2.fisheye.projectPoints` to 2e-13 px.
- **Bag cross-check (weak):**
  - Red gift bag: foot width 0.171 m, rim 0.29–0.31 m. That depends on rim-pixel choice: −5 px gives −1 cm. The code assumes rim 0.27 m (`bag_cam_rim_h`, not an owner measurement).
  - The handles are folded down, so the owner's 0.36–0.37 m handle-top check can't be used.
  - If the lens were at 0.89 m, every length would grow by 12% (rim ≈ 0.33–0.35 m). So the bag check does not support 0.89 m.
  - White UM bag, for a future tape check: foot width 0.245 m, front-rim height 0.36 m, handle top ≈ 0.40 m.
- **Evidence images:**
  - `evidence/01_grid_overlay_fit.jpg`: fitted grid (red) over the median frame, with the ridge points used (yellow).
  - `02_undistorted_pinhole_f287.jpg`: chair leg and grout straight.
  - `03_birdseye_metric.jpg`: square tiles in a metric top view.
  - `05_birdseye_grout_vs_grid.jpg`: grout ridges remapped onto the fitted grid.
  - `04_tape_and_bag_picks.jpg`: tape points and the bag corner picks.

## Did the camera move?
- **Orientation: yes, clearly.** Pitch went from 61.4° to 47.0° (≈ 14° less steep), and the view now reaches 2.2 m instead of about 1.2 m. Roll is ≈ 1°.
- **Height: probably lower,** 0.79 ± 0.02 m against 0.89 m. That is 10 cm, about 5σ, if the 0.296 m tile scale is right. Two things settle it: a tape measure on the lens height and on one tile.
  - If the lens really is at 0.89 m, the tile would have to be 0.332 m, which contradicts the 18 Nov disc layout.
- **x, y:** unknown. Nothing robot-fixed lies on the floor in both views, and the robot itself was moved: the grid phase under the robot changed by ≈ 6 cm.
  - The tape is 1.3 cm left of the new lens's ground point. On 18 Nov it was 0.2 cm left of the robot centre, so the robot is roughly centred on the tape again, but its placement isn't known.
- **Yaw relative to the robot:** 0.76° is relative to the grout. It only holds if the robot stands square to the tiles (it was within ≈ 0.4° on 18 Nov).

## Code changes needed (minimal; proposed, NOT committed)
The deployed code matches origin/main `cc8a70b`. The patch and its tests are in `proposed-patch/` (`front_camera.py.diff`, `bag_camera.py.diff`, `test_patch.py`).
Idea: detection keeps running on the raw (distorted) frame. Only pixel ↔ ray conversion learns the lens. No image remap is needed and no detector change.
1. `base/jetson/front_camera.py` (→ `~/hmc-follow/front_camera.py`):
   - New helpers `_fish_r`, `_fish_theta` (Newton inverse).
   - `FrontCamera.__init__`: new args `dist_model="pinhole"` and `dist=(k1..k4)`. `copy()` and `to_dict()` / `from_dict()` carry them, defaulting to pinhole, so the old file is unchanged.
   - `_project()`: θ = atan2(r, z), scale = f·θd/r.
   - `_ray()`: undistort the pixel (θd → θ, scale by tan θ/θd) before rotating.
   - Unchanged but now correct, because they call `_ray`/`_project`: `measure_bag()`, the fit in `fit()`, `tools/front_cam_cal.py` and `tools/front_cam_bag_cal.py` (the distortion stays fixed from the file), and `for_image()` (k is dimensionless under uniform scaling).
   - Doc only: `WIDTH, HEIGHT` / `F_GUESS` comments still say AX1080P. `measure_bag`'s `band = cam.fx*0.03/r_col` is ≤ 10% off near the frame edge (cosmetic).
2. `arm/so100_driver/so100_driver/bag_camera.py` (→ `~/so100_ws/.../bag_camera.py`):
   - `FrontCamera.__init__` reads `dist_model` / `dist`.
   - `ray()` and `project()` get the same fisheye math.
   - `for_image()` (copy.copy) keeps them.
   - `side_at`, `_at_height`, `locate` and the `cam.project` at line 927 all go through these.
3. Not changed but needs re-checking with the new geometry:
   - `bag_vision.fixed_lifted` pixel thresholds (`min_up=12`, "10 cm ≈ 40 px").
   - `bag_cam_rim_h` / `bag_cam_width` gates.
   - The bag detector, which was trained on pinhole frames (parallel task; `distortion.json` plus `tools/warp_old_to_fisheye.py`).
   - Doc: `base/docs/bag-sensing.md` "Model. Pinhole…".
4. **Tests:**
   - `test_patch.py`: patched classes vs `cv2.fisheye` 2e-13 px; the two classes agree; `ray(project(p))` 1e-15 m; the old pinhole JSON gives bit-identical results; `to_dict` / `for_image` keep `dist`.
   - Existing suites on the patched copies: `base/jetson` front_camera / cal tests 33/38 pass, and the same 5 fail on unpatched code (no scipy on the box). `arm/so100_driver` bag_camera_scale + bag_vision 14/14 OK.
- **Deploying the candidate JSON without the patch would be wrong:** old code ignores `dist`. Deploy the patch first, then the JSON with `calibrated: true`, after the owner checks it.

## What a later session with a person would add
1. Tape-measure the lens height above the floor, one tile edge, and the lens x from F / LiDAR. That is 2 minutes, settles z and scale, and the fit then gives z directly.
2. Discs at taped points in the pick zone (0.3–0.7 m), the old `front_cam_bag_cal.py discs` with the patched model and distortion fixed. That fixes x and yaw relative to the robot and checks the pick zone, where grout is sparse.
3. A checkerboard or ArUco board in 15–20 poses over the whole frame, including the corners, with `cv2.fisheye.calibrate`. That gives independent k1–k4, cx and cy (now ±2–4 px) and an independent f, instead of a single-plane estimate.
4. A bag of known size with the handles up, to repeat the 0.36–0.37 m handle check.

## Files
`frames/` (f00_1210, f01–f12), `work_median.png`, `fit_results.json` (all fits, observations, jackknife, hold-out), `nearfield_check.json`, `old_tiles_lines.json`, `old-2026-11-18/` (d1, discs.json, overlays), `evidence/`, `tools/` (model, fit, overlay, tape, bags, warp), `proposed-patch/`, `distortion.json`, `front_camera.candidate.json`.
