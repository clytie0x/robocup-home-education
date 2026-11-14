"""Frame maths for the base. Base frame: x forward, y left, origin at the turning centre."""
import math


def lidar_to_base(p_xy, params):
    # lidar.py already rotates its angles so 0 is the front; only the mounting offset is left.
    return (p_xy[0] + params["lidar_x"], p_xy[1])


def polar(p_xy):
    """(degrees, metres), 0 = front, positive left."""
    return (math.degrees(math.atan2(p_xy[1], p_xy[0])), math.hypot(p_xy[0], p_xy[1]))


def base_to_odom(p_xy, pose):
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    return (x + c * p_xy[0] - s * p_xy[1], y + s * p_xy[0] + c * p_xy[1])


def odom_to_base(p_xy, pose):
    x, y, yaw = pose
    dx, dy = p_xy[0] - x, p_xy[1] - y
    c, s = math.cos(yaw), math.sin(yaw)
    return (c * dx + s * dy, -s * dx + c * dy)
