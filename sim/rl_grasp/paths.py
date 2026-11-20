"""Locations of the SO-100 URDF inside this repo."""

import sys
from pathlib import Path


def repo_root():
    return Path(__file__).resolve().parents[2]


def urdf_path():
    return repo_root() / "arm" / "so100_moveit_config" / "urdf" / "so100.urdf"


def mesh_dir():
    return repo_root() / "arm" / "so100_moveit_config" / "meshes"


def kinematics_type():
    """so100_driver.ik.Kinematics. That module does not import ROS."""
    driver = str(repo_root() / "arm" / "so100_driver")
    if driver not in sys.path:
        sys.path.insert(0, driver)
    from so100_driver.ik import Kinematics

    return Kinematics
