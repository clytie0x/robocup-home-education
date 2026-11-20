"""Wide-FOV fisheye model for the bag detector.

The front camera is a 640x480 fisheye. Its intrinsics and pose are not calibrated,
and the old pinhole file is not read. A point is projected with a Kannala-Brandt
polynomial, noise is added, and the pixel is back-projected with a fixed nominal
model. The difference is the calibration error the policy will see.
"""

import math

import numpy as np

from . import constants as C


def nominal_camera(kin):
    pos = np.array(kin.from_user(C.NOMINAL_CAM_FWD, 0.0))
    pos = np.array((pos[0], pos[1], C.FLOOR_Z + C.NOMINAL_CAM_H))
    target = np.array(kin.from_user(0.35, 0.0))
    target = np.array((target[0], target[1], C.FLOOR_Z + C.RIM_H))
    return {
        "pos": pos,
        "rotation": look_rotation(pos, target),
        "f": C.NOMINAL_F_PX,
        "cx": C.IMAGE_W / 2.0,
        "cy": C.IMAGE_H / 2.0,
        "k": np.zeros(4),
    }


def sample_camera(kin, rng):
    """A wide draw around the nominal mount. Every range is uncalibrated."""
    forward = float(rng.uniform(*C.CAM_FWD_RANGE))
    left = float(rng.uniform(*C.CAM_LEFT_RANGE))
    height = float(rng.uniform(*C.CAM_H_RANGE))
    pos = np.array(kin.from_user(forward, left))
    pos = np.array((pos[0], pos[1], C.FLOOR_Z + height))
    aim_h = C.RIM_H + float(rng.uniform(-0.08, 0.08))
    target = np.array(kin.from_user(0.35, float(rng.uniform(-0.05, 0.05))))
    target = np.array((target[0], target[1], C.FLOOR_Z + aim_h))
    k = rng.uniform(-0.08, 0.08, size=4)
    return {
        "pos": pos,
        "rotation": look_rotation(pos, target),
        "f": float(rng.uniform(*C.F_RANGE)),
        "cx": C.IMAGE_W / 2.0 + float(rng.uniform(-40.0, 40.0)),
        "cy": C.IMAGE_H / 2.0 + float(rng.uniform(-30.0, 30.0)),
        "k": np.asarray(k, float),
    }


def look_rotation(pos, target):
    """Columns are camera x (right), y (down), z (forward), in the arm frame."""
    forward = np.asarray(target, float) - np.asarray(pos, float)
    forward = forward / np.linalg.norm(forward)
    right = np.cross(forward, np.array((0.0, 0.0, 1.0)))
    if np.linalg.norm(right) < 1e-8:
        right = np.array((1.0, 0.0, 0.0))
    right = right / np.linalg.norm(right)
    down = np.cross(forward, right)
    return np.column_stack((right, down, forward))


def _poly(theta, k):
    t2 = theta * theta
    return 1.0 + k[0] * t2 + k[1] * t2 * t2 + k[2] * t2 ** 3 + k[3] * t2 ** 4


def project(point, cam):
    """Pixel of a world point, or None when it is behind the camera or off the sensor."""
    relative = np.asarray(point, float) - cam["pos"]
    local = cam["rotation"].T @ relative
    if local[2] <= 1e-6:
        return None
    radius = math.hypot(local[0], local[1])
    theta = math.atan2(radius, local[2])
    distorted = theta * _poly(theta, cam["k"])
    scale = cam["f"] if radius < 1e-8 else cam["f"] * distorted / radius
    u = cam["cx"] + scale * local[0]
    v = cam["cy"] + scale * local[1]
    if u < 0 or v < 0 or u >= C.IMAGE_W or v >= C.IMAGE_H:
        return None
    return u, v


def _theta_of(distorted, k):
    """Invert theta * poly(theta) = distorted. distorted is the angle, not pixels."""
    theta = float(distorted)
    for _ in range(8):
        value = theta * _poly(theta, k)
        t2 = theta * theta
        derivative = (
            1.0 + 3.0 * k[0] * t2 + 5.0 * k[1] * t2 * t2
            + 7.0 * k[2] * t2 ** 3 + 9.0 * k[3] * t2 ** 4
        )
        if abs(derivative) < 1e-8:
            break
        theta -= (value - distorted) / derivative
    return theta


def pixel_ray(u, v, cam):
    x = (u - cam["cx"]) / cam["f"]
    y = (v - cam["cy"]) / cam["f"]
    distorted = math.hypot(x, y)
    theta = 0.0 if distorted < 1e-8 else _theta_of(distorted, cam["k"])
    if distorted < 1e-8:
        local = np.array((0.0, 0.0, 1.0))
    else:
        sine, cosine = math.sin(theta), math.cos(theta)
        local = np.array((sine * x / distorted, sine * y / distorted, cosine))
    return cam["rotation"] @ local


def hit_plane(cam, u, v, world_z):
    """Where the back-projected ray crosses a horizontal plane, or None."""
    direction = pixel_ray(u, v, cam)
    if abs(direction[2]) < 1e-8:
        return None
    step = (world_z - cam["pos"][2]) / direction[2]
    if step <= 0:
        return None
    return cam["pos"] + step * direction


def detect(truth, true_cam, nominal_cam, rng, kin, noise_px, dropout_p):
    """Detector reading in the arm user frame (forward, left, height above the floor).

    `truth` has centre, handle, rim_left, rim_right as world points, and standing.
    A dropout, or a point that misses the sensor, falls back to the true centre
    with source "lidar", which is what bag_camera does below CONF_MIN.
    """
    true_forward, true_left = kin.to_user(truth["centre"])
    fallback = {
        "forward": float(true_forward), "left": float(true_left),
        "handle": None, "width": float(truth["width"]),
        "standing": bool(truth["standing"]), "confidence": 0.0, "source": "lidar",
    }
    if rng.random() < dropout_p:
        return fallback
    centre_px = project(truth["centre"], true_cam)
    if centre_px is None:
        return fallback
    noisy = (
        centre_px[0] + float(rng.normal(0.0, noise_px)),
        centre_px[1] + float(rng.normal(0.0, noise_px)),
    )
    plane_z = C.FLOOR_Z + C.RIM_H
    centre = hit_plane(nominal_cam, noisy[0], noisy[1], plane_z)
    if centre is None:
        return fallback
    forward, left = kin.to_user(centre)
    handle = _handle(truth, true_cam, nominal_cam, rng, kin, noise_px)
    width = _width(truth, true_cam, nominal_cam, kin)
    return {
        "forward": float(forward), "left": float(left),
        "handle": handle, "width": width,
        "standing": bool(truth["standing"]), "confidence": 0.85, "source": "camera",
    }


def _handle(truth, true_cam, nominal_cam, rng, kin, noise_px):
    if truth.get("handle") is None:
        return None
    pixel = project(truth["handle"], true_cam)
    if pixel is None:
        return None
    noisy = (
        pixel[0] + float(rng.normal(0.0, noise_px)),
        pixel[1] + float(rng.normal(0.0, noise_px)),
    )
    point = hit_plane(nominal_cam, noisy[0], noisy[1], C.FLOOR_Z + C.HANDLE_TOP)
    if point is None:
        return None
    forward, left = kin.to_user(point)
    return (float(forward), float(left), C.HANDLE_TOP)


def _width(truth, true_cam, nominal_cam, kin):
    left = project(truth["rim_left"], true_cam)
    right = project(truth["rim_right"], true_cam)
    if left is None or right is None:
        return float(truth["width"])
    a = hit_plane(nominal_cam, left[0], left[1], C.FLOOR_Z + C.RIM_H)
    b = hit_plane(nominal_cam, right[0], right[1], C.FLOOR_Z + C.RIM_H)
    if a is None or b is None:
        return float(truth["width"])
    return float(np.linalg.norm(a[:2] - b[:2]))
