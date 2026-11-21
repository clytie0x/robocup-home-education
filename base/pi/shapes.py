"""Obstacle circles for the planner, in the odometry frame."""
import math

from hmc.world import Circle

import footprint
import geometry
import lidar

RADIUS = 0.05  # m
LIDAR_CELL = 0.1  # m, one circle per cell
DEPTH_CELL = 0.05
DEPTH_KEEP = 10.0  # s a depth point lives after it was last seen
PERSON_CLEAR = 0.45  # m around a person that is not an obstacle (the brain adds a person circle)
# A belt or table top above the LiDAR plane leaves the Xtion's view close by (above the image's top edge, and beside
# the robot after a turn), and the robot (robot_height) is taller than a barrier belt: such a remembered point this
# near is kept while the camera cannot see it. Lower things (a bag, a small object) age DEPTH_KEEP as before.
OVERHANG_M = 0.5  # m above the floor: well above the LiDAR plane and a bag's top
FREEZE_R = 1.5  # m from the turning centre
DEPTH_SIZE = (320, 240)  # px, the Xtion's depth image (camera.py)
VIEW_MARGIN = math.radians(2.0)  # this close to the image's edge counts as out of view


class ObstacleMemory:
    def __init__(self, p):
        self.p = p
        self._depth = {}  # cell -> (x, y, time last seen or kept, highest point above the floor or None), odometry frame

    def update(self, now, pose, scan, depth_points, people, heights=None):
        """Circles for this moment. people are (x, y) in the odometry frame.

        scan: (degrees, metres) as from lidar.py. depth_points: (N, 2) base-frame array, or
        None when there was no usable depth image (the remembered points still count). heights: each depth
        point's height above the floor (m), or None: an overhang (OVERHANG_M or higher) within FREEZE_R is kept
        while the camera cannot see it; everything else ages DEPTH_KEEP as before.
        """
        def near_person(x, y):
            return any(math.dist((x, y), person) < PERSON_CLEAR for person in people)

        seen = set()
        for i, xy in enumerate(() if depth_points is None else depth_points):
            x, y = geometry.base_to_odom((float(xy[0]), float(xy[1])), pose)
            if not near_person(x, y):
                cell = (math.floor(x / DEPTH_CELL), math.floor(y / DEPTH_CELL))
                z = None if heights is None else float(heights[i])
                old = self._depth.get(cell)
                if z is not None and old is not None and old[3] is not None:
                    z = max(z, old[3])
                self._depth[cell] = (x, y, now, z)
                seen.add(cell)
        for cell, (x, y, t, z) in list(self._depth.items()):
            if cell not in seen and z is not None and self._unseen_overhang(geometry.odom_to_base((x, y), pose), z):
                self._depth[cell] = (x, y, now, z)  # out of view close by: kept until the camera can check it
        self._depth = {cell: kept for cell, kept in self._depth.items() if now - kept[2] <= DEPTH_KEEP}

        lidar_cells = {}
        bases = []
        for deg, metres in lidar.usable(scan):
            rad = math.radians(deg)
            bases.append(geometry.lidar_to_base((metres * math.cos(rad), metres * math.sin(rad)), self.p))
        if bases:
            own = footprint.own_mask(bases, self.p)  # the arm, cable and bag are not obstacles for the planner
            for base, is_self in zip(bases, own):
                if is_self:
                    continue
                x, y = geometry.base_to_odom(base, pose)
                if not near_person(x, y):
                    lidar_cells.setdefault((math.floor(x / LIDAR_CELL), math.floor(y / LIDAR_CELL)), (x, y))

        return [Circle(x, y, RADIUS) for x, y in lidar_cells.values()] + [
            Circle(x, y, RADIUS) for x, y, _, _ in self._depth.values() if not near_person(x, y)
        ]  # memory stays whole: a person passing by only hides points while near

    def _unseen_overhang(self, base, z):
        """True for an overhang (z of OVERHANG_M or more) within FREEZE_R that the depth camera cannot see now: above
        the top edge of its image (pinhole, tilted up by camera_pitch_deg) or outside its width."""
        p = self.p
        if z < OVERHANG_M or math.hypot(*base) > FREEZE_R:
            return False
        dx, dy, dz = base[0] - p["camera_x"], base[1] - p.get("camera_y", 0.0), z - p["camera_height"]
        width, height = DEPTH_SIZE
        half = math.radians(p["camera_hfov_deg"]) / 2
        top = math.atan(((height - 1) / 2) / ((width / 2) / math.tan(half)))  # the top edge, above the optical axis
        up = math.atan2(dz, math.hypot(dx, dy)) - math.radians(p["camera_pitch_deg"])
        return abs(math.atan2(dy, dx)) > half - VIEW_MARGIN or up > top - VIEW_MARGIN
