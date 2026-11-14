"""Obstacle circles for the planner, in the odometry frame."""
import math

from hmc.world import Circle

import geometry
import lidar

RADIUS = 0.05  # m
LIDAR_CELL = 0.1  # m, one circle per cell
DEPTH_CELL = 0.05
DEPTH_KEEP = 10.0  # s a depth point lives after it was last seen
PERSON_CLEAR = 0.45  # m around a person that is not an obstacle (the brain adds a person circle)


class ObstacleMemory:
    def __init__(self, p):
        self.p = p
        self._depth = {}  # cell -> (x, y, time last seen), odometry frame

    def update(self, now, pose, scan, depth_points, people):
        """Circles for this moment. people are (x, y) in the odometry frame.

        scan: (degrees, metres) as from lidar.py. depth_points: (N, 2) base-frame array, or
        None when there was no usable depth image (the remembered points still count).
        """
        def near_person(x, y):
            return any(math.dist((x, y), person) < PERSON_CLEAR for person in people)

        for xy in () if depth_points is None else depth_points:
            x, y = geometry.base_to_odom((float(xy[0]), float(xy[1])), pose)
            if not near_person(x, y):
                self._depth[(math.floor(x / DEPTH_CELL), math.floor(y / DEPTH_CELL))] = (x, y, now)
        self._depth = {cell: seen for cell, seen in self._depth.items() if now - seen[2] <= DEPTH_KEEP}

        lidar_cells = {}
        for deg, metres in lidar.usable(scan):
            rad = math.radians(deg)
            base = geometry.lidar_to_base((metres * math.cos(rad), metres * math.sin(rad)), self.p)
            x, y = geometry.base_to_odom(base, pose)
            if not near_person(x, y):
                lidar_cells.setdefault((math.floor(x / LIDAR_CELL), math.floor(y / LIDAR_CELL)), (x, y))

        return [Circle(x, y, RADIUS) for x, y in lidar_cells.values()] + [
            Circle(x, y, RADIUS) for x, y, _ in self._depth.values() if not near_person(x, y)
        ]  # memory stays whole: a person passing by only hides points while near
