# bagpick: automatic bag pick with the arm camera

Test scripts (not part of the ROS package) that pick the bag up by its top and raise it high
enough to keep the robot's front clear. They run on the Jetson next to the arm stack.

## Setup

- Arm stack running in robot mode: `~/so100_ws/start_arm.sh mount:=robot rviz:=false`
- Webcam on the forearm, served by `webcam_view.py` on port 8082 (`/snap.jpg`)
- Xtion stream from the base Pi at `http://192.168.31.21:8081/stream`
- Copy the three `.py` files into `~/so100_ws` (same folder)

## Run

```bash
cd ~/so100_ws
python3 bagpick.py --putback      # pick, raise, then put the bag back (testing)
python3 bagpick.py                # pick and keep holding the bag high
```

Each step saves `bp_*.jpg` frames in the current folder. The last line printed starts with
`RESULT` (holding / missed / too far / no bag).

## What it does

1. **Find:** from a viewing pose (forearm pointing at the floor in front), look at three pan
   angles, centre the bag in the image.
2. **Ridge:** read the bag's top edge: level = broad side facing (jaws open front-back),
   peak = narrow side facing (jaws open left-right), slanted = diagonal (jaws turned +-45 deg).
3. **Grasp:** along the bearing the camera saw the bag on, jaws pitched -60 deg, come down in
   2 cm steps and close near the top. The SO-100 has one moving jaw, so the middle of the open
   gap (not the tool axis) is aimed at the bag.
4. **Check:** lift 10 cm; in the Xtion, the bag-coloured columns at the bottom must now show floor.
   On a miss: open in place, look for the bag again, try further out (up to 4 tries).
5. **Raise:** tilt the jaws up in small steps while rising (tips about 65 cm above the floor),
   check again that the bag is still held.

## Known limits

- The bag is slippery: it can slip out of the jaws (gripper torque is limited to 50 %).
- Distance is not measured from the image: tries go from ~33 cm out to ~37 cm from the pan axis.
- Bags far to the side are outside the Xtion view: the lift is then reported as not verified.

## Files

- `bagpick.py`: the pick sequence
- `bagvision.py`: image helpers (find the bag, read the ridge, Xtion lift check)
- `gcam.py`: small arm helper (move to a pose, gripper, webcam snapshot), also usable alone:
  `python3 gcam.py state`, `python3 gcam.py look FWD LEFT Z [ROLL] --pitch DEG`, `python3 gcam.py grip 0.9`
