"""Calibrate the gripper camera and write ~/hmc-follow/gripper_camera.json.

bag_arm uses that file to measure the bag top from the camera. A measurement that
passes the confidence gate skips the arm's touch probe. Without the file, or when
the measurement is rejected, the touch probe still runs.

The live pick is bag_method camera (bag_camera.py). That path does not touch-probe:
it closes at bag_cam_close_h and ignores the grasp z. A missing gripper_camera.json
is not what added the ~10 s. Two Xtion reads to the old Wi-Fi address did (each
urlopen waits 5 s). The default is now the LAN, http://10.0.0.1:8081/stream.

This is floor_pick.py --calibrate-self / --calibrate, with the flags filled in.
Run it on the Jetson from ~/hmc-follow, with the arm stack up, webcam_view.py
serving port 8082, and the console holding the arm (or HMC_MAINTENANCE set):

    python3 tools/gripper_cam_cal.py

That is the grout-crossing calibration: no taped marker, the floor joint in view
is the target (floor_pick.py --calibrate-self). It moves the arm. --dry-run
prints the moves and does not write the file.

With a red square at taped spots (metres, robot frame):

    python3 tools/gripper_cam_cal.py --marker 0.36 0.0 --marker 0.38 0.08

The camera model is the one in gripper_camera.py: it is the camera on the gripper.
The fixed front camera under the deck is a different file (front_camera.json,
tools/front_cam_cal.py). A gripper_camera.json from the fixed camera will not
pass the gate, and the pick falls back to the touch probe.

Any other floor_pick.py flag (--snap-url, --camera-file, --dry-run, --marker-kind)
is passed through.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def argv_for(argv):
    """floor_pick.py arguments. No mode yet: --calibrate-self. A --marker: --calibrate."""
    argv = list(argv)
    if "--calibrate" in argv or "--calibrate-self" in argv:
        return argv
    if "--marker" in argv:
        return ["--calibrate", *argv]
    return ["--calibrate-self", *argv]


def main(argv=None):
    from floor_pick import main as floor_main  # noqa: PLC0415 (ROS only when the tool runs)
    return floor_main(argv_for(sys.argv[1:] if argv is None else argv))


if __name__ == "__main__":
    sys.exit(main())
