"""The sim copies the arm stack's numbers. If those change, this fails on purpose."""

import xml.etree.ElementTree as ET

from rl_grasp import constants as C
from rl_grasp.paths import repo_root


def test_tries_and_speeds_match_bag_camera():
    text = (repo_root() / "arm/so100_driver/so100_driver/bag_camera.py").read_text()
    assert "TRIES = ((0.0, -60), (0.015, -60), (-0.015, -60), (0.03, -50))" in text
    assert C.TRIES[0] == (0.0, -60.0)
    assert C.TRIES[3] == (0.03, -50.0)
    assert "REACH_WALL = 0.32" in text
    assert C.REACH_WALL == 0.32
    assert "LIFT_SPEED = 0.25" in text
    assert C.LIFT_SPEED == 0.25
    assert "FREE_SPEED, HELD_SPEED, PUT_SPEED = 0.8, 0.4, 0.6" in text
    assert C.FREE_SPEED == 0.8


def test_close_height_and_bias_match_the_node():
    text = (repo_root() / "arm/so100_driver/so100_driver/pick_place_node.py").read_text()
    assert 'self.declare_parameter("bag_cam_close_h", 0.19)' in text
    assert 'self.declare_parameter("bag_cam_close_bias", [-0.012, -0.014])' in text
    assert 'self.declare_parameter("bag_cam_rim_h", 0.27)' in text
    assert 'self.declare_parameter("bag_cam_width", 0.18)' in text
    assert 'self.declare_parameter("bag_cam_half_depth", 0.055)' in text
    assert C.CLOSE_H == 0.19
    assert C.BIAS_FORWARD == -0.012
    assert C.BIAS_UP == -0.014


def test_floor_and_joint_speed_match_the_launch_and_moveit():
    launch = (repo_root() / "arm/so100_moveit_config/launch/arm.launch.py").read_text()
    assert 'DeclareLaunchArgument("floor_z", default_value="-0.331"' in launch
    assert 'DeclareLaunchArgument("deck_z", default_value="-0.068"' in launch
    assert C.FLOOR_Z == -0.331
    assert C.DECK_Z == -0.068
    limits = (repo_root() / "arm/so100_moveit_config/config/joint_limits.yaml").read_text()
    assert "max_velocity: 1.5" in limits
    assert "gripper:" in limits and "max_velocity: 3.0" in limits
    assert C.ARM_VEL == 1.5
    assert C.GRIPPER_VEL == 3.0


def test_model_joint_ranges_match_the_urdf():
    import mujoco

    from rl_grasp.scene import joint_id, load_scene

    model, _kin = load_scene("paper")
    root = ET.parse(repo_root() / "arm/so100_moveit_config/urdf/so100.urdf")
    checked = 0
    for joint in root.findall("joint"):
        if joint.get("type") != "revolute":
            continue
        limit = joint.find("limit")
        low, high = float(limit.get("lower")), float(limit.get("upper"))
        index = joint_id(model, joint.get("name"))
        assert model.jnt_range[index, 0] == low
        assert abs(model.jnt_range[index, 1] - high) < 1e-5
        checked += 1
    assert checked == 6
    # mujoco is imported for the load. The range compare is the point.
    assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "gripper") >= 0
