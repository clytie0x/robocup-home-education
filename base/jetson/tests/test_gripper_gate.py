"""The gripper-camera confidence gate and the calibration tool's default flags."""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "tools"))

import bag_arm  # noqa: E402
import gripper_cam_cal  # noqa: E402


def test_confidence_is_one_at_no_spread_and_the_limit_at_max_spread():
    assert bag_arm.gripper_gate(0.0, 0.26, 0.0) == 1.0
    assert bag_arm.gripper_gate(bag_arm.MAX_SPREAD, 0.26, 0.0) == bag_arm.GRIPPER_CONF_AT_LIMIT


def test_a_point_outside_a_hard_limit_is_rejected():
    assert bag_arm.gripper_gate(0.021, 0.26, 0.0) == 0.0
    assert bag_arm.gripper_gate(0.0, 0.14, 0.0) == 0.0
    assert bag_arm.gripper_gate(0.0, 0.51, 0.0) == 0.0
    assert bag_arm.gripper_gate(0.0, 0.26, 0.09) == 0.0


def test_top_source():
    assert bag_arm.top_source(True, True) == "camera"
    assert bag_arm.top_source(False, True) == "probe"
    assert bag_arm.top_source(False, False) == "default"


def test_the_calibration_tool_defaults_to_the_grout_cross():
    assert gripper_cam_cal.argv_for([]) == ["--calibrate-self"]
    assert gripper_cam_cal.argv_for(["--dry-run"]) == ["--calibrate-self", "--dry-run"]


def test_a_marker_selects_marker_calibration_and_an_explicit_mode_is_kept():
    got = gripper_cam_cal.argv_for(["--marker", "0.36", "0.0"])
    assert got[:1] == ["--calibrate"]
    assert "--marker" in got
    assert gripper_cam_cal.argv_for(["--calibrate", "--marker", "0.36", "0.0"])[0] == "--calibrate"
    assert gripper_cam_cal.argv_for(["--calibrate-self", "--dry-run"])[0] == "--calibrate-self"
