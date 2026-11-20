# Front camera (HD USB Camera 32e4:9320), red-disc calibration, 2026-11-20

`front_camera_2026-11-20.json` is the new front camera's calibration (`calibrated: true`). It is not yet on the robot:
it uses `dist_model: fisheye`, which the front camera code reads only from 77a8cc4 on.

- 19 red discs at taped floor points, 3 frames (`d1.jpg`..`d3.jpg`, layouts in `layouts.json`, pixels in
  `discs.json`, `tools/front_cam_bag_cal.py discs` format). Distances from F (floor under the front edge of the
  lowest plate, on the centre line); left positive.
- Tape (owner): lens height 0.83 m, lens 0.15 m behind F, so x = 0.375 - 0.15 = 0.225 m.
- Held from the tile fit (`front_camera_fisheye_candidate_2026-11-20.json` on main): fx 522.7, cx/cy 296.7/241.1,
  fisheye k1/k2 -0.120/0.054, roll 0.96 deg.
- Fitted: y -0.001 m, yaw -0.30 deg, pitch 47.66 deg. rms 2.30 px (worst 5.7); floor points back-projected within
  0.62 cm median, 1.86 cm worst.
- Check: with x and z also free, the discs alone give z 0.821 m and the lens 0.157 m behind F, matching the tape.
  The tile candidate (x copied from the old camera) put the same discs 9-10 cm off.
- Far discs (80-100 cm) are only 12-22 px on this lens; the gripper's red parts were left out of the detections.
