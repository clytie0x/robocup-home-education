"""The fisheye round trip, and that a mismatched nominal model moves the point."""

import numpy as np

from rl_grasp.camera import hit_plane, nominal_camera, project
from rl_grasp.paths import kinematics_type, urdf_path


def _kin():
    return kinematics_type()(urdf_path().read_text())


def test_project_and_backproject_agree_on_the_same_camera():
    kin = _kin()
    cam = nominal_camera(kin)
    point = np.array(kin.from_user(0.32, 0.02))
    point = np.array((point[0], point[1], -0.331 + 0.27))
    pixel = project(point, cam)
    assert pixel is not None
    back = hit_plane(cam, pixel[0], pixel[1], point[2])
    assert back is not None
    assert np.allclose(back[:2], point[:2], atol=1e-3)


def test_a_different_focal_length_moves_the_back_projection():
    kin = _kin()
    true = nominal_camera(kin)
    nominal = nominal_camera(kin)
    nominal["f"] = true["f"] * 1.4
    point = np.array(kin.from_user(0.32, 0.04))
    point = np.array((point[0], point[1], -0.331 + 0.27))
    pixel = project(point, true)
    back = hit_plane(nominal, pixel[0], pixel[1], point[2])
    assert np.linalg.norm(back[:2] - point[:2]) > 0.005
