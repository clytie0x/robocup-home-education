"""The bag at the car (bag.py): the pure task in a simulated room, then run_follow's BAG phase over --remote."""
import json
import math
import os
import sys
import unittest

import numpy

import bag
import geometry
import params
import run_follow
import safety
import virtual_operator as vo
from link import Answer, Seen
from test_run_follow import Camera, FakeBase, Logged, Scans
from test_virtual_operator import room

P = params.load()
PERIOD = 0.1


def scan_at(pose, bags_odom, wall=3.0, radius=vo.BAG_RADIUS):
    """A round room around the robot with bag circles at odometry points, as the LiDAR sees it from pose."""
    saved, vo.BAG_RADIUS = vo.BAG_RADIUS, radius
    try:
        return vo.inject(room(wall=wall), [], P, [geometry.odom_to_base(b, pose) for b in bags_odom])[0]
    finally:
        vo.BAG_RADIUS = saved


class World:
    """A perfect base in a round room with bags; the operator stands at `operator` and points at `point`."""

    def __init__(self, bags, operator, point, pose=(0.0, 0.0, 0.0), lay=(), seen_bags=None):
        self.bags, self.operator, self.point = list(bags), operator, point
        self.pose, self.t = pose, 0.0
        self.lay = list(lay)  # cable circles, odometry frame
        self.seen_bags = self.bags if seen_bags is None else seen_bags  # the bags the LiDAR sees
        self.extra = lambda world: []  # more odometry points to block with, per step
        self.sent = []
        self.depth_of = None  # (world, task) -> a depth image for the cycle, or None
        self.look = None  # a bag.Look (or a fake) served every cycle

    def view(self, boxes=(), frame=(320, 240)):
        scan = scan_at(self.pose, self.seen_bags)
        pts = run_follow.scan_points(scan, P)
        extra = [geometry.odom_to_base(q, self.pose) for q in self.extra(self)]
        if extra:
            pts = numpy.vstack([pts, numpy.array(extra)])
        op = geometry.odom_to_base(self.operator, self.pose)
        found = None if self.point is None else bag.virtual_pointing(geometry.odom_to_base(self.point, self.pose), op)
        return bag.View(self.t, self.pose, seen=pts, clusters=bag.lidar_clusters(scan, P), operator=op, found=found,
                        boxes=boxes, frame=frame, lay=[geometry.odom_to_base(c, self.pose) for c in self.lay],
                        speed_cap=0.12)

    def run(self, task, seconds=200.0, boxes=lambda world: ()):
        """Cycle the task like run_follow: its command, the stop on its safety points, the base moves."""
        while self.t < seconds and not task.done:
            view = self.view(boxes(self))
            if self.depth_of is not None:
                view.depth = self.depth_of(self, task)
            if self.look is not None:
                self.look.serve(task, self.t)
            v, w, _ = task.step(view)
            stop = "obstacle" if (v or w) and safety.must_stop(task.safety_points(view.real), v, w, P) else None
            if stop:
                v = w = 0.0
            task.after(self.t, stop, (v, w))
            self.sent.append((v, w))
            x, y, th = self.pose
            self.pose = (x + v * PERIOD * math.cos(th), y + v * PERIOD * math.sin(th), th + w * PERIOD)
            self.t += PERIOD
        return task

    def truth(self, i=0):
        return geometry.odom_to_base(self.bags[i], self.pose)


def new_task(mode="point", last=None, **kw):
    return bag.BagTask(P, 0.0, 1, mode, last=last, **kw)


class Geometry(unittest.TestCase):
    def test_the_turn_puts_the_bag_on_the_arm_line(self):
        for b in [(2.0, 0.8), (1.0, -0.5), (-1.5, 0.5), (0.5, 0.0145), (0.4, -0.3)]:
            th = bag.turn_for(b)
            c, s = math.cos(th), math.sin(th)
            self.assertAlmostEqual(-s * b[0] + c * b[1], bag.ARM_Y, places=9, msg=b)  # y after turning by th
            self.assertGreater(c * b[0] + s * b[1], 0.0, b)  # and ahead

    def test_the_stop_point_leaves_the_bag_at_the_target(self):
        b = (2.0, 0.8)
        (ex, ey), d = bag.stop_point(b)
        th = bag.turn_for(b)
        dx, dy = b[0] - ex, b[1] - ey
        self.assertAlmostEqual(math.cos(th) * dx + math.sin(th) * dy, bag.TARGET_X, places=9)
        self.assertAlmostEqual(-math.sin(th) * dx + math.cos(th) * dy, bag.ARM_Y, places=9)
        self.assertAlmostEqual(d, math.hypot(ex, ey), places=9)

    def test_the_centre_is_behind_the_side_the_laser_sees(self):
        face = (1.175, 0.0)  # 1 m straight ahead of the laser
        self.assertEqual(bag.centre_of(face, P), (1.175 + bag.BAG_HALF_DEPTH, 0.0))

    def test_a_cable_beside_the_drive_line_blocks_it(self):
        self.assertTrue(bag.crosses([(1.0, 0.2)], (2.0, 0.0), P))
        self.assertFalse(bag.crosses([(1.0, 0.3)], (2.0, 0.0), P))
        self.assertFalse(bag.crosses([(-0.5, 0.0)], (2.0, 0.0), P))  # behind the robot: not on the way

    def test_the_target_is_inside_the_arms_reach(self):
        ahead = bag.TARGET_X - 0.012  # bag centre ahead of the arm's pan axis at robot (0.012, 0.0145)
        wall = ahead - bag.BAG_HALF_DEPTH  # front wall grasped by the fixed-camera bag pick
        self.assertTrue(0.24 <= wall - bag.X_TOL and wall + bag.X_TOL <= 0.32)
        self.assertGreater(bag.TARGET_X - bag.BAG_HALF_DEPTH, safety.FRONT)  # the bag's side stays off the body


class Approach(unittest.TestCase):
    def assert_reached(self, task, world, i=0):
        self.assertEqual(task.state, bag.REACHED, task.result)
        x, y = task.result["x"], task.result["y"]
        self.assertLessEqual(abs(x - bag.TARGET_X), bag.X_TOL)
        self.assertLessEqual(abs(y - bag.ARM_Y), bag.Y_TOL)
        truth = world.truth(i)  # the reported centre is where the bag really is
        self.assertLess(math.dist((x, y), truth), 0.02, (x, y, truth))
        self.assertLessEqual(abs(truth[0] - bag.TARGET_X), bag.X_TOL)

    def test_the_pointed_bag_is_chosen_turned_to_and_reached(self):
        world = World([(2.0, 0.8)], operator=(2.6, 0.2), point=(2.0, 0.8))
        task = world.run(new_task())
        self.assert_reached(task, world)
        self.assertEqual(task.result["code"], "OK")
        self.assertLess(task.result["odometry_m"], 0.15)  # the last LiDAR sighting was a few cm before the stop
        self.assertEqual((task.chosen["side"], task.chosen["say"]), (None, "I will take the bag."))
        self.assertLess(math.dist((task.chosen["x"], task.chosen["y"]), (2.0, 0.8)), 0.03)
        speeds = [v for v, _ in world.sent]
        self.assertLessEqual(max(speeds), bag.FAR_SPEED + 1e-9)
        self.assertFalse(any(v < 0 for v in speeds))  # never backward

    def test_it_creeps_at_most_0_03_m_s_near_the_bag(self):
        world = World([(1.0, 0.0145)], operator=(1.2, 0.8), point=(1.0, 0.0145))
        task = new_task()
        near = []
        while not task.done and world.t < 100:
            view = world.view()
            v, w, _ = task.step(view)
            task.after(world.t, None, (v, w))
            if world.truth()[0] - bag.BAG_HALF_DEPTH - safety.FRONT < safety.clearance(P) + 0.05:
                near.append(v)
            x, y, th = world.pose
            world.pose = (x + v * PERIOD * math.cos(th), y + v * PERIOD * math.sin(th), th + w * PERIOD)
            world.t += PERIOD
        self.assert_reached(task, world)
        self.assertTrue(near)
        self.assertLessEqual(max(near), bag.CREEP + 1e-9)

    def test_of_two_bags_the_pointed_one_and_its_side(self):
        bags = [(2.0, 0.8), (2.0, -0.2)]
        for i, side in ((0, "right"), (1, "left")):  # the operator faces the robot: robot left is their right
            world = World(bags, operator=(2.6, 0.3), point=bags[i])
            task = world.run(new_task())
            self.assertEqual(task.chosen["side"], side)
            self.assertEqual(task.chosen["say"], "I will take the %s bag, on your %s." % (side, side))
            self.assert_reached(task, world, i)

    def test_a_bag_behind_or_beside_is_turned_to_first(self):
        for b, operator in (((-1.5, 0.5), (-1.5, -0.4)), ((0.6, -0.9), (1.4, -0.9))):
            world = World([b], operator=operator, point=b)
            task = world.run(new_task())
            self.assert_reached(task, world)

    def test_no_pointing_is_not_pointed_after_20_s(self):
        world = World([(2.0, 0.8)], operator=(2.6, 0.2), point=None)
        task = world.run(new_task())
        self.assertEqual(task.result["code"], "NOT_POINTED")
        self.assertAlmostEqual(world.t, bag.POINTING_S + PERIOD, delta=0.25)
        self.assertEqual(set(world.sent), {(0.0, 0.0)})  # it never moved

    def test_pointing_at_bare_floor_is_no_bag(self):
        world = World([(2.0, 0.8)], operator=(2.6, 0.2), point=(1.5, -1.0))
        task = world.run(new_task())
        self.assertEqual(task.result["code"], "NO_BAG")
        self.assertLess(world.t, 5.0)  # steady pointing (1 s), then NO_BAG_S
        self.assertEqual(set(world.sent), {(0.0, 0.0)})

    def test_the_operators_legs_are_never_the_bag(self):
        task = new_task()
        view = bag.View(0.0, (0.0, 0.0, 0.0), clusters=[(2.0, 0.0), (2.0, 0.15), (1.6, 0.9)], operator=(2.05, 0.07))
        self.assertEqual(task._bags(view), [bag.centre_of((1.6, 0.9), P)])

    def test_the_bag_alone_is_left_out_of_the_stop_and_only_while_creeping(self):
        task = new_task()
        task.bag, task.last_pose = (0.5, 0.0), (0.0, 0.0, 0.0)
        pts = numpy.array([[0.45, 0.0], [0.47, 0.05], [0.33, -0.18], [1.5, 0.0]])
        self.assertIs(task.safety_points(pts), pts)  # pointing: everything counts
        task.state = bag.CREEP_STATE
        self.assertEqual(task.safety_points(pts).tolist(), [[0.33, -0.18], [1.5, 0.0]])
        task.virtual = True  # a virtual bag: nothing real to come close to, so nothing is left out
        self.assertIs(task.safety_points(pts), pts)

    def test_something_else_in_front_while_creeping_stops_it(self):
        world = World([(1.5, 0.0145)], operator=(1.7, 0.9), point=(1.5, 0.0145))
        # a box comes into the front strip beside the bag (0.25 m from it) once the robot creeps
        world.extra = lambda w: [geometry.base_to_odom((0.28, -0.17), w.pose)] if task.state == bag.CREEP_STATE else []
        task = new_task()
        world.run(task)
        self.assertEqual(task.result["code"], "BLOCKED", task.result)

    def test_an_obstacle_on_the_way_far_out_is_blocked(self):
        world = World([(2.0, 0.0145)], operator=(2.2, 0.9), point=(2.0, 0.0145))
        world.extra = lambda w: [(1.0, 0.05)]  # a pole on the drive line
        task = world.run(new_task())
        self.assertEqual(task.result["code"], "BLOCKED")
        self.assertLess(world.pose[0], 1.0 - safety.FRONT - safety.clearance(P) + 0.02)

    def test_a_bag_that_vanishes_is_lost(self):
        world = World([(2.0, 0.8)], operator=(2.6, 0.2), point=(2.0, 0.8))
        task = new_task()
        while task.state in (bag.POINTING,) and world.t < 10:
            world.run(task, seconds=world.t + PERIOD)
        world.seen_bags = []  # someone took it
        world.run(task)
        self.assertEqual(task.result["code"], "LOST_BAG")

    def test_a_cable_on_the_way_is_blocked_before_moving(self):
        world = World([(2.0, 0.0145)], operator=(2.2, 0.9), point=(2.0, 0.0145), lay=[(1.0, 0.1), (1.1, 0.1)])
        task = world.run(new_task())
        self.assertEqual(task.result, {"ok": False, "code": "BLOCKED", "message": "my cable lies on the way to the bag"})
        self.assertEqual(set(world.sent), {(0.0, 0.0)})

    def test_a_wound_cable_stops_the_task(self):
        world = World([(-1.5, 0.5)], operator=(-1.5, -0.4), point=(-1.5, 0.5))
        task = new_task()
        while task.state == bag.POINTING:
            world.run(task, seconds=world.t + PERIOD)
        view = world.view()
        view.cable_holding = True
        task.step(view)
        self.assertEqual(task.result["code"], "BLOCKED")

    def test_too_far_is_blocked(self):
        world = World([(4.5, 0.0)], operator=(4.5, 0.8), point=(4.5, 0.0))
        task = new_task()
        view = world.view()
        view.clusters = [(4.45, 0.0)]  # the room is only 3 m: put a cluster there by hand
        view.found = bag.virtual_pointing((4.5, 0.0), (4.5, 0.8))
        for k in range(15):
            view.now = k * PERIOD
            task.step(view)
        self.assertEqual(task.result["code"], "BLOCKED")
        self.assertIn("more than", task.result["message"])

    def test_the_approach_times_out(self):
        world = World([(2.0, 0.8)], operator=(2.6, 0.2), point=(2.0, 0.8))
        task = new_task()
        while task.state == bag.POINTING:
            world.run(task, seconds=world.t + PERIOD)
        view = world.view()
        view.now = task.chosen_at + bag.APPROACH_S + 0.1
        task.step(view)
        self.assertEqual(task.result["code"], "TIMEOUT")

    def test_a_cancel_fails_it_and_it_stands(self):
        task = new_task()
        task.cancel(1.0, "wait")
        self.assertEqual(task.result, {"ok": False, "code": "CANCELLED", "message": "wait came"})
        self.assertEqual(task.step(bag.View(1.1, (0.0, 0.0, 0.0))), (0.0, 0.0, None))
        self.assertFalse(task.wants_pose)

    def test_bag_again_measures_and_approaches_the_last_bag(self):
        world = World([(2.0, 0.8)], operator=(2.6, 0.2), point=None)
        task = world.run(new_task("again", last=(2.03, 0.78)))
        self.assert_reached(task, world)
        self.assertFalse(world.sent[0] == (0.0, 0.0) and len(world.sent) > 200)
        task = World([(2.0, 0.8)], operator=(2.6, 0.2), point=None).run(new_task("again", last=(1.0, -1.0)))
        self.assertEqual(task.result["code"], "NO_BAG")
        task = World([(2.0, 0.8)], operator=(2.6, 0.2), point=None).run(new_task("again"))
        self.assertEqual(task.result["code"], "NO_BAG")

    def test_bag_near_takes_the_nearest_bag_in_front_without_pointing(self):
        world = World([(1.6, 0.0), (1.0, -0.3)], operator=(-1.0, 1.0), point=None)
        task = new_task("near")
        self.assertFalse(task.wants_pose)
        world.run(task)
        self.assert_reached(task, world, 1)
        self.assertEqual((task.chosen["say"], task.chosen["candidates"]), ("I will go to the bag in front of me.", 2))
        for bags in ([(1.0, 1.0)], [(2.3, 0.0)], []):  # 45 deg off, 2.3 m away, none
            task = World(bags, operator=(-1.0, 1.0), point=None).run(new_task("near"))
            self.assertEqual(task.result["code"], "NO_BAG", bags)

    def test_bag_near_waits_for_the_first_lidar_scan(self):
        # 2026-11-20 22:59: the console sent bag_near 0.3 s into the run, the LiDAR's first scan came at 0.6 s
        world = World([(1.0, 0.0)], operator=(-1.0, 1.0), point=None)
        task = new_task("near")
        for t in (0.0, 0.1, 0.2, 0.3, 0.4, 0.5):
            task.step(bag.View(t, world.pose))  # no scan yet
            self.assertFalse(task.done, task.result)
        world.t = 0.6
        world.run(task)
        self.assert_reached(task, world, 0)
        task, t = new_task("near"), 0.0
        while not task.done and t < 10.0:  # a LiDAR that never answers
            task.step(bag.View(t, (0.0, 0.0, 0.0)))
            t += 0.1
        self.assertEqual(task.result["code"], "NO_BAG")
        self.assertIn("LiDAR", task.result["message"])
        self.assertAlmostEqual(t, bag.NO_SCAN_S, delta=0.25)

    def test_a_bag_only_the_camera_sees_is_reached_by_odometry(self):
        # The clear plastic bag stays under the LiDAR plane: only the detector's box finds it.
        world = World([(1.8, 0.4)], operator=(2.3, -0.3), point=(1.8, 0.4), seen_bags=[])

        def boxes(w):
            if task.state != bag.POINTING:
                return ()
            front = geometry.odom_to_base((1.8 - 0.05, 0.4), w.pose)  # the bag's near bottom edge, on the floor
            u, v = bag.pointing.project((front[0], front[1], 0.0), P, 320, 240)
            return [[u - 12, v - 30, u + 12, v]]

        task = new_task()
        world.run(task, boxes=boxes)
        self.assertEqual(task.state, bag.REACHED, task.result)
        self.assertEqual(task.result["code"], "ESTIMATED")
        self.assertIsNone(task.result["odometry_m"])
        self.assertLess(math.dist((task.result["x"], task.result["y"]), world.truth()), 0.1)

    def test_pose_is_asked_only_while_looking_for_the_pointing(self):
        world = World([(2.0, 0.8)], operator=(2.6, 0.2), point=(2.0, 0.8))
        task = new_task()
        self.assertTrue(task.wants_pose)
        world.run(task, seconds=3.0)
        self.assertNotEqual(task.state, bag.POINTING)
        self.assertFalse(task.wants_pose)
        self.assertFalse(new_task("again").wants_pose)


TESTS = os.path.join(os.path.dirname(__file__), "..", "..", "jetson", "tests")  # the laptop; not deployed to the Pi


@unittest.skipUnless(os.path.exists(os.path.join(TESTS, "test_pointing.py")), "base/jetson/tests is not here")
class CameraPointing(unittest.TestCase):
    def test_the_operators_keypoints_give_the_pointed_floor_spot(self):
        sys.path.insert(0, TESTS)
        try:
            import test_pointing
        finally:
            sys.path.remove(TESTS)
        operator, target = (2.5, 0.0), (2.5, 1.0)
        kp = test_pointing.body(operator, target, p=P)
        bearing = math.degrees(math.atan2(operator[1] - P["camera_y"], operator[0] - P["camera_x"]))
        answer = Answer(0.0, [Seen(bearing + 30.0, 0.9, (1.0,), [0, 0, 1, 1], kp),  # someone else, far off
                              Seen(bearing, 0.9, (1.0,), [0, 0, 1, 1], kp)])
        found = bag.camera_pointing(answer, operator, (320, 240), P)
        self.assertLess(math.dist(found.point, target), 0.15)
        self.assertIsNone(bag.camera_pointing(Answer(0.0, [Seen(bearing, 0.9, (1.0,), [0, 0, 1, 1])]), operator,
                                              (320, 240), P))  # no keypoints on this frame
        self.assertIsNone(bag.camera_pointing(answer, None, (320, 240), P))


class FakeLook:
    """Answers a task's front camera request at once with `answer` (None: never answers)."""

    def __init__(self, answer):
        self.answer, self.requests = answer, []

    def serve(self, task, now):
        req = task.look_request
        if req is None:
            return
        if not self.requests or self.requests[-1]["id"] != req["id"]:
            self.requests.append(req)
        if self.answer is not None:
            task.looked(dict(self.answer, id=req["id"]))


class Look(unittest.TestCase):
    """The far stage measures the bag (base/docs/bag-sensing.md): depth first, the Jetson's front camera next."""

    BAG = (2.6, 0.3)  # odometry: chosen from about 2.6 m, so the far drive must stop to look
    HIGH, YAW = 0.30, math.radians(25)

    def world(self, depth=True, dark=False):
        from test_bag_depth import render
        world = World([self.BAG], operator=(3.0, -0.4), point=self.BAG)
        if depth:
            def depth_of(w, task):
                centre = geometry.odom_to_base(self.BAG, w.pose)
                box = {"centre": centre, "yaw": self.YAW - w.pose[2], "size": (0.26, 0.11, self.HIGH)}
                return render(box, dark=dark) if task.wants_depth else None
            world.depth_of = depth_of
        return world

    def test_the_far_drive_stops_to_look_and_the_depth_top_reaches_the_result(self):
        world = self.world()
        task = world.run(new_task(look=True))
        self.assertEqual(task.state, bag.REACHED, task.result)
        self.assertEqual(task.result["top_by"], "depth")
        self.assertLess(abs(task.result["top"] - self.HIGH), 0.015, task.result)
        m = task.status()["look"]
        self.assertTrue(bag.LOOK_RANGE[0] <= m["range"] <= bag.LOOK_RANGE[1], m)
        self.assertLess(abs(m["range"] - bag.LOOK_AT), 0.1, m)  # stopped there on the way, not at the first stop
        self.assertTrue(m["depth"]["ok"])

    def test_the_yaw_is_given_in_the_base_frame_at_the_end(self):
        from unittest import mock
        with mock.patch.object(bag.bag_depth, "cluster_yaw", lambda pts: (0.4, 1.0, "test")):
            world = self.world()
            world.pose = (0.0, 0.0, 0.7)  # odometry yaw far from zero: an odometry-frame yaw would show
            world.bags = world.seen_bags = [geometry.base_to_odom(self.BAG, world.pose)]
            world.point = world.bags[0]
            world.operator = geometry.base_to_odom((3.0, -0.4), world.pose)
            self.BAG = world.bags[0]
            task = world.run(new_task(look=True))
        self.assertEqual(task.state, bag.REACHED, task.result)
        look_yaw = task.status()["look"]["pose"][2]
        want = bag.bag_depth.wrap_line(0.4 + look_yaw - world.pose[2])
        self.assertAlmostEqual(task.result["yaw"], want, places=3)
        self.assertLess(abs(task.result["yaw"] - 0.4), 0.05)  # about the look's base-frame yaw, not 0.4 + 0.7

    def test_a_low_confidence_yaw_is_left_out(self):
        from unittest import mock
        with mock.patch.object(bag.bag_depth, "cluster_yaw", lambda pts: (0.4, 0.2, "short")):
            task = self.world().run(new_task(look=True))
        self.assertNotIn("yaw", task.result)

    def test_no_depth_on_the_bag_asks_the_front_camera(self):
        world = self.world(dark=True)
        world.look = FakeLook({"ok": True, "top": 0.33, "conf": 0.8, "top_by": "range"})
        task = world.run(new_task(look=True, rgb=True))
        self.assertEqual(task.state, bag.REACHED, task.result)
        self.assertEqual((task.result["top"], task.result["top_by"]), (0.33, "rgb"))
        self.assertEqual(len(world.look.requests), 1)
        pts = numpy.array(world.look.requests[0]["points"])
        self.assertGreaterEqual(len(pts), bag.MIN_POINTS)
        self.assertFalse(task.status()["look"]["depth"]["ok"])

    def test_no_answer_goes_on_without_a_top(self):
        world = self.world(depth=False)
        world.look = FakeLook(None)
        task = world.run(new_task(look=True, rgb=True))
        self.assertEqual(task.state, bag.REACHED, task.result)
        self.assertNotIn("top", task.result)
        self.assertIn("no answer", task.status()["look"]["why"])

    def test_a_refused_front_camera_answer_is_not_used(self):
        world = self.world(dark=True)
        world.look = FakeLook({"ok": False, "top": 0.5, "why": "the two ways disagree"})
        task = world.run(new_task(look=True, rgb=True))
        self.assertNotIn("top", task.result)

    def test_without_look_nothing_is_asked(self):
        world = self.world()
        world.look = FakeLook({"ok": True, "top": 0.33})
        task = world.run(new_task())
        self.assertEqual(task.state, bag.REACHED, task.result)
        self.assertNotIn("top", task.result)
        self.assertEqual(world.look.requests, [])

    def test_a_bag_first_seen_too_near_is_not_looked_at(self):
        world = World([(0.85, 0.0145)], operator=(1.2, 0.8), point=(0.85, 0.0145))
        task = world.run(new_task(look=True))
        self.assertEqual(task.state, bag.REACHED, task.result)
        self.assertNotIn("top", task.result)
        self.assertIn("nearer than", task.status()["look"]["why"])


class LookClient(unittest.TestCase):
    def ask(self, port, request):
        import time

        class Task:
            look_request = request
            answer = None

            def looked(self, a):
                self.answer = a
        task, look = Task(), bag.Look(["127.0.0.1"], port)
        for _ in range(200):
            look.serve(task, 0.0)
            if task.answer:
                break
            time.sleep(0.02)
        return task.answer

    def test_it_asks_the_service_on_a_thread_and_hands_the_answer_over(self):
        import socket
        import threading
        srv = socket.socket()
        srv.bind(("127.0.0.1", 0))
        srv.listen(1)
        got = []

        def answer():
            c, _ = srv.accept()
            req = json.loads(c.makefile().readline())
            got.append(req)
            c.sendall((json.dumps({"id": req["id"], "ok": True, "top": 0.31}) + "\n").encode())
            c.close()
        threading.Thread(target=answer, daemon=True).start()
        a = self.ask(srv.getsockname()[1], {"id": "1-1", "points": [[1.0, 0.0]]})
        srv.close()
        self.assertEqual(a["top"], 0.31)
        self.assertEqual(got[0]["id"], "1-1")

    def test_nobody_listening_is_an_answer_too(self):
        import socket
        s = socket.socket()
        s.bind(("127.0.0.1", 0))
        port = s.getsockname()[1]
        s.close()  # nothing listens there now
        a = self.ask(port, {"id": "2-1", "points": []})
        self.assertFalse(a["ok"])
        self.assertEqual(a["id"], "2-1")

class Phase(Logged):
    """run_follow's BAG phase over --remote, with the virtual operator and a virtual bag."""

    def setUp(self):
        super().setUp()
        from unittest import mock
        from command_server import CommandServer
        from test_command_server import Client
        planner = mock.patch.object(run_follow, "REMOTE", None)
        planner.start()
        self.addCleanup(planner.stop)
        self.server = CommandServer("127.0.0.1", 0)
        self.server.start()
        self.addCleanup(self.server.stop)
        self.jetson = Client(self.server.port)
        self.addCleanup(self.jetson.close)
        self.base = FakeBase()
        self.state = run_follow.State(P, self.base, Scans(room()), Camera(), None, self.log, clock=lambda: self.now,
                                      remote=self.server)
        self.state.virtual = vo.VirtualOperator(P, port=0, controls=self.state.controls, clock=lambda: self.now, seed=3)
        self.t = 0.0
        self.person, self.point, self.bags, self.word = (2.5, -0.6), None, [[2.0, 0.7]], None

    def tick(self, n=1):
        """n cycles: the PC's datagram, a ping, the cycle, the base moves."""
        record = None
        for _ in range(n):
            self.now = self.t
            msg = {"t": self.t, "x": self.person[0], "y": self.person[1], "th": math.pi, "speaking": self.word,
                   "pointing": self.point, "bags": self.bags}
            if self.state.virtual is not None:
                self.state.virtual.receive(json.dumps(msg).encode(), ("127.0.0.1", 1))
            self.server.heard = self.t  # the bridge's pings, without a socket round trip each cycle
            record = run_follow.cycle(self.state, self.t)
            self.base.advance(PERIOD)
            self.t = round(self.t + PERIOD, 3)
        return record

    def at_the_car(self):
        self.jetson.cmd("follow")
        self.tick(3)
        self.jetson.cmd("wait")
        self.tick(2)

    def statuses(self):
        return self.jetson.read()

    def test_bag_is_refused_unless_waiting_at_the_car(self):
        self.jetson.cmd("bag")
        self.tick()  # IDLE
        self.jetson.cmd("follow")
        self.tick()
        self.jetson.cmd("bag")  # following, not waiting
        record = self.tick()
        errors = [s for s in self.statuses() if "error" in s]
        self.assertEqual([e["refused"] for e in errors], ["bag", "bag"])
        self.assertIn("IDLE", errors[0]["error"])
        self.assertEqual(record["phase"], "follow")
        self.assertIsNone(record["bag_task"])

    def test_the_pointed_virtual_bag_is_found_approached_and_the_way_back_drives_it(self):
        self.at_the_car()
        crumbs = len(self.state.follower.route.points)
        self.point = self.bags[0]
        self.jetson.cmd("bag")
        phases = set()
        for _ in range(900):
            record = self.tick()
            phases.add(record["phase"])
            if record["bag_task"] and record["bag_task"]["result"]:
                break
        for _ in range(5):  # 0.5 s more: the 5 Hz status line catches up with the result (record stays the one above)
            self.tick()
        result = record["bag_task"]["result"]
        self.assertEqual(result["code"], "OK", record["bag_task"])
        truth = geometry.odom_to_base(self.bags[0], self.base.pose)
        self.assertLess(math.dist((result["x"], result["y"]), truth), 0.02)
        self.assertLessEqual(abs(truth[0] - bag.TARGET_X), bag.X_TOL)
        self.assertLessEqual(abs(truth[1] - bag.ARM_Y), bag.Y_TOL)
        self.assertIn("bag", phases)
        self.assertEqual((record["phase"], record["remote"]["held"]), ("follow", "wait"))  # at the car, waiting
        self.assertLess(math.dist(record["bag"], self.bags[0]), 0.03)  # odometry frame, for the PC app
        self.assertGreater(len(self.state.follower.route.points), crumbs)  # the drive counts as path
        statuses = [s for s in self.statuses() if "bag" in s]
        self.assertIn("BAG", {s["phase"] for s in statuses})
        self.assertTrue(all(s["held"] is None for s in statuses if s["phase"] == "BAG"))
        # the newest status that carries the result (statuses go out at 5 Hz: the very last one may predate it)
        last = next(s["bag"] for s in reversed(statuses) if s["bag"] and s["bag"].get("result"))
        self.assertEqual((last["result"]["code"], last["chosen"]["say"]), ("OK", "I will take the bag."))
        self.assertTrue(all(abs(v) <= bag.FAR_SPEED + 1e-9 for v, _ in self.base.sent))
        self.jetson.cmd("return")  # the arm picked it: the way back
        record = self.tick()
        self.assertEqual(record["phase"], "return")

    def test_the_virtual_operators_words_stop_then_bag_run_it_without_the_jetson(self):
        self.jetson.close()  # nobody on --remote: the PC leads (its datagrams are the page heartbeat)
        for word, n in (("follow", 3), (None, 2), ("stop", 3), (None, 2)):
            self.word = word
            self.tick(n)
        self.assertTrue(self.state.page_wait)  # the word stop is the page's Wait
        self.point, self.word = self.bags[0], "bag"
        for _ in range(900):
            record = self.tick()
            self.word = None
            if record["bag_task"] and record["bag_task"]["result"]:
                break
        self.assertEqual(record["bag_task"]["result"]["code"], "OK", record["bag_task"])
        self.assertIn({"time": 1.1, "page_cmd": "bag"}, [r for r in self.records() if "page_cmd" in r])
        self.assertEqual((record["phase"], record["remote"]["held"]), ("follow", "wait"))

    def test_bag_near_runs_from_idle_and_goes_back_to_idle(self):
        self.bags = [[1.5, 0.3]]
        self.tick(2)
        self.state.silent = True  # no Jetson for a while: the latched silence hold
        self.jetson.cmd("bag_near")
        record = self.tick()
        self.assertEqual(record["phase"], "bag")
        self.assertFalse(self.state.silent)
        self.jetson.cmd("return")  # IDLE's answer: a taken return would end the next follow at once
        for _ in range(900):
            record = self.tick()
            if record["bag_task"] and record["bag_task"]["result"]:
                break
        self.assertEqual(record["bag_task"]["result"]["code"], "OK", record["bag_task"])
        self.assertEqual(record["bag_task"]["mode"], "near")
        self.assertEqual(record["phase"], "idle")
        self.assertEqual(self.tick()["stop"], "idle")  # the next cycle stands as IDLE does
        self.assertFalse(self.state.go_back.is_set())
        self.assertEqual([s["error"] for s in self.statuses() if "error" in s], ["not following yet"])
        self.jetson.cmd("follow")
        self.tick(2)
        self.assertEqual(self.state.phase, "follow")

    def test_bag_near_is_refused_while_following_not_held(self):
        self.jetson.cmd("follow")
        self.tick(2)
        self.jetson.cmd("bag_near")
        record = self.tick()
        errors = [s for s in self.statuses() if "error" in s]
        self.assertEqual([e["refused"] for e in errors], ["bag_near"])
        self.assertIn("or in IDLE", errors[0]["error"])
        self.assertEqual(record["phase"], "follow")

    def test_a_follow_cancels_a_bag_near_from_idle_and_follows(self):
        self.bags = [[1.5, 0.3]]
        self.jetson.cmd("bag_near")
        self.tick(3)
        self.jetson.cmd("follow")
        record = self.tick()
        self.assertEqual(record["bag_task"]["result"]["code"], "CANCELLED")
        self.assertEqual(record["phase"], "follow")

    def test_wait_cancels_the_bag_and_holds(self):
        self.at_the_car()
        self.point = self.bags[0]
        self.jetson.cmd("bag")
        for _ in range(25):
            record = self.tick()
        self.assertEqual(record["phase"], "bag")
        self.jetson.cmd("wait")
        record = self.tick()
        self.assertEqual((record["phase"], record["stop"]), ("follow", "wait"))
        self.assertEqual(record["bag_task"]["result"]["code"], "CANCELLED")
        self.assertEqual(self.base.command, (0.0, 0.0))

    def test_the_page_wait_holds_the_bag_task_still(self):
        self.at_the_car()
        self.point = self.bags[0]
        self.jetson.cmd("bag")
        self.tick(25)
        self.state.page_wait = True
        record = self.tick()
        self.assertEqual((record["phase"], record["stop"], record["v"], record["w"]), ("bag", "page", 0.0, 0.0))

    def test_keypoints_are_asked_only_while_the_bag_task_looks_for_the_pointing(self):
        class Link:
            def __init__(self):
                self.poses = []

            def send(self, frame, pose=False):
                self.poses.append(pose)

            def latest(self):
                return None

            def alive(self, now, silence):
                return True

        self.state.link = link = Link()
        self.state.virtual = None
        self.at_the_car()
        self.assertFalse(any(link.poses))
        self.jetson.cmd("bag")
        self.tick(3)
        self.assertEqual(link.poses[-3:], [True, True, True])


    def test_the_bag_task_looks_with_the_frames_depth_and_the_jetson_only_with_a_look_client(self):
        self.assertIsNone(self.state.bag_look)
        self.at_the_car()
        self.jetson.cmd("bag")
        self.tick(2)
        self.assertTrue(self.state.bag.look)
        self.assertFalse(self.state.bag.rgb)  # no Jetson: depth only
        views = []
        step = self.state.bag.step
        self.state.bag.step = lambda view: (views.append(view), step(view))[1]
        self.tick(2)
        self.assertTrue(views and all(v.depth is not None for v in views))  # the Camera's frames carry depth
        self.jetson.cmd("wait")
        self.tick(2)

        served = []

        class Look:
            def serve(self, task, now):
                served.append(task)
        self.state.bag_look = Look()
        self.jetson.cmd("bag")
        self.tick(3)
        self.assertTrue(self.state.bag.rgb)
        self.assertIs(served[-1], self.state.bag)

if __name__ == "__main__":
    unittest.main()
