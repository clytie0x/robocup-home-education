"""The virtual operator: a simulated person put into the real LiDAR scan and a made-up camera answer."""
import json
import math
import socket
import time
import unittest

import numpy

import dashboard
import params
import virtual_operator as vo
from legs import candidates
from link import Answer, Seen
from test_legs import ray_circle

P = params.load()
STEP = 0.5  # degrees between samples, like the LDS-03


def room(wall=3.0, step=STEP, box=None):
    """A round room of radius `wall`; box: (deg_from, deg_to, metres) a real object nearer on those bearings."""
    scan, deg = [], -180.0
    while deg < 180.0:
        m = wall
        if box is not None and box[0] <= deg <= box[1]:
            m = box[2]
        scan.append((round(deg, 3), m))
        deg += step
    return scan


def at_robot(x, y, th=math.pi):
    """A packet for a person at (x, y) of the odometry frame, the robot standing at the origin."""
    return {"t": 0.0, "x": x, "y": y, "th": th, "speaking": None}


class Operator(unittest.TestCase):
    def make(self, allow=(), controls=None):
        self.now = 0.0
        return vo.VirtualOperator(P, port=0, allow=allow, controls=controls, clock=lambda: self.now, seed=3)

    def feed(self, op, packet, now=None, addr="127.0.0.1"):
        if now is not None:
            self.now = now
        return op.receive(json.dumps(packet).encode(), (addr, 40000))


class Legs(Operator):
    def test_two_legs_018_apart_across_the_heading_while_standing(self):
        left, right = vo.legs((2.0, 0.0, math.pi), walked=0.0, speed=0.0)
        self.assertAlmostEqual(math.dist(left, right), vo.LEG_APART, places=6)
        self.assertAlmostEqual(left[0], 2.0, places=6)  # side by side: nothing ahead or behind
        self.assertAlmostEqual(right[0], 2.0, places=6)

    def test_legs_swing_against_each_other_while_walking(self):
        quarter = vo.STRIDE / 4  # a quarter of a gait cycle: the widest swing
        left, right = vo.legs((0.0, 0.0, 0.0), walked=quarter, speed=1.0)
        self.assertAlmostEqual(left[0], vo.SWING, places=6)
        self.assertAlmostEqual(right[0], -vo.SWING, places=6)
        slow_left, _ = vo.legs((0.0, 0.0, 0.0), walked=quarter, speed=vo.FULL_SWING_SPEED / 2)
        self.assertAlmostEqual(slow_left[0], vo.SWING / 2, places=6)  # a slower walk swings less

    def test_the_legs_swing_as_the_packets_walk_the_person(self):
        op = self.make()
        for i in range(10):  # 0.5 m/s along x
            self.feed(op, dict(at_robot(1.0 + 0.05 * i, 0.0, 0.0), t=0.1 * i), now=0.1 * i)
        self.assertAlmostEqual(op.walked, 0.45, places=6)
        self.assertAlmostEqual(op.speed, 0.5, delta=0.05)


class Injection(Operator):
    def test_the_person_is_one_lidar_candidate_at_its_place_sampled_at_the_scan_step(self):
        legs = [vo.legs((2.0, 0.5, math.pi), 0.0, 0.0)]
        sensed, shown, hidden = vo.inject(room(), [l for pair in legs for l in pair], P)
        self.assertEqual(len(sensed), len(room()))  # the same bearings, nothing added between them
        self.assertEqual([d for d, _ in sensed], [d for d, _ in room()])
        found = candidates(sensed)
        self.assertEqual(len(found), 1)
        self.assertAlmostEqual(found[0][0], 2.0 - P["lidar_x"], delta=0.06)  # lidar frame
        self.assertAlmostEqual(found[0][1], 0.5, delta=0.06)
        self.assertEqual(hidden, 0)
        # two 0.12 m legs at about 1.9 m from the laser: each about 3.6 deg wide, so 6 to 8 samples each
        self.assertTrue(10 <= shown <= 18, shown)

    def test_the_ranges_are_the_ray_hits_on_the_leg_circles(self):
        left, right = (2.0, 0.09), (2.0, -0.09)
        sensed, _, _ = vo.inject(room(), [left, right], P)
        for (deg, m), (_, real) in zip(sensed, room()):
            hits = [h for c in (left, right) if (h := ray_circle(deg, c[0] - P["lidar_x"], c[1], vo.LEG_RADIUS))]
            self.assertAlmostEqual(m, min(hits) if hits else real, places=6)

    def test_a_virtual_leg_hides_the_real_wall_behind_it(self):
        sensed, shown, _ = vo.inject(room(wall=3.0), [(2.0, 0.09), (2.0, -0.09)], P)
        leg = math.degrees(math.atan2(0.09, 2.0 - P["lidar_x"]))  # the left leg's bearing from the laser
        at_leg = [m for deg, m in sensed if abs(deg - leg) <= 1.0]
        self.assertTrue(at_leg and all(m < 2.0 for m in at_leg))
        between = [m for deg, m in sensed if abs(deg) <= 0.5]
        self.assertTrue(between and all(m == 3.0 for m in between))  # the wall still shows between the legs

    def test_a_real_return_in_front_drops_the_virtual_returns_behind_it(self):
        real = room(box=(-10.0, 10.0, 1.0))  # a real box 1 m ahead of the laser
        sensed, shown, hidden = vo.inject(real, [(2.0, 0.09), (2.0, -0.09)], P)
        self.assertEqual(sensed, real)  # nothing of the person shows: the box is nearer on every bearing
        self.assertEqual(shown, 0)
        self.assertGreater(hidden, 0)

    def test_a_no_return_sample_takes_the_virtual_leg(self):
        real = [(deg, 0.0) for deg, _ in room()]  # nothing back from anywhere (open hall)
        sensed, shown, _ = vo.inject(real, [(2.0, 0.09), (2.0, -0.09)], P)
        self.assertGreater(shown, 0)
        self.assertTrue(all(m == 0.0 or 1.7 < m < 2.0 for _, m in sensed))

    def test_an_empty_scan_stays_empty(self):
        self.assertEqual(vo.inject([], [(2.0, 0.09)], P), ([], 0, 0))


class Camera(Operator):
    def answer(self, x, y, scan=None, real=None):
        op = self.make()
        self.feed(op, at_robot(x, y), now=0.0)
        return op.apply(0.05, (0.0, 0.0, 0.0), scan if scan is not None else room(), real, False)

    def test_one_person_with_conf_09_inside_the_field_of_view(self):
        _, answer, alive = self.answer(2.0, 0.5)
        self.assertTrue(alive)
        self.assertEqual(len(answer.people), 1)
        seen = answer.people[0]
        bearing = math.degrees(math.atan2(0.5 - P["camera_y"], 2.0 - P["camera_x"]))  # from the camera
        self.assertAlmostEqual(seen.bearing_deg, bearing, places=6)
        self.assertEqual(seen.conf, 0.9)
        self.assertEqual(answer.t, 0.05)
        x1, y1, x2, y2 = seen.box
        self.assertTrue(0 <= x1 < x2 <= vo.IMAGE[0] and 0 <= y1 < y2 <= vo.IMAGE[1])

    def test_the_box_centre_gives_back_the_bearing(self):
        from link import bearing_deg
        _, answer, _ = self.answer(2.0, -0.6)
        seen = answer.people[0]
        self.assertAlmostEqual(bearing_deg(seen.box, vo.IMAGE[0], P["camera_hfov_deg"]), seen.bearing_deg, places=3)

    def test_nobody_outside_the_field_of_view_or_behind(self):
        for x, y in ((1.0, 1.5), (-2.0, 0.0), (0.5, -2.0)):
            _, answer, alive = self.answer(x, y)
            self.assertTrue(alive)  # the camera works, it sees nobody
            self.assertEqual(answer.people, [], (x, y))

    def test_nobody_when_a_real_object_hides_the_person(self):
        _, answer, _ = self.answer(2.0, 0.0, scan=room(box=(-15.0, 15.0, 1.0)))
        self.assertEqual(answer.people, [])

    def test_nobody_beyond_the_camera_range(self):
        _, answer, _ = self.answer(vo.CAMERA_RANGE + 0.5, 0.0, scan=room(wall=9.0))
        self.assertEqual(answer.people, [])

    def test_real_camera_people_are_dropped(self):
        real = Answer(0.0, [Seen(0.0, 0.95, (1.0, 0.0), [0, 0, 1, 1]), Seen(10.0, 0.9, (0.0, 1.0), [0, 0, 1, 1])])
        _, answer, _ = self.answer(2.0, 1.4, real=real)  # the virtual person out of view
        self.assertEqual(answer.people, [])
        _, answer, _ = self.answer(2.0, 0.0, real=real)
        self.assertEqual(len(answer.people), 1)
        self.assertNotIn(answer.people[0].look, [(1.0, 0.0), (0.0, 1.0)])

    def test_the_look_is_the_same_every_cycle_a_512_unit_vector(self):
        op = self.make()
        looks = []
        for i in range(5):
            self.feed(op, dict(at_robot(2.0, 0.0), t=0.1 * i), now=0.1 * i)
            looks.append(op.apply(0.1 * i, (0.0, 0.0, 0.0), room(), None, True)[1].people[0].look)
        self.assertTrue(all(look == looks[0] for look in looks))
        self.assertEqual(len(looks[0]), 512)
        self.assertAlmostEqual(sum(v * v for v in looks[0]), 1.0, places=6)
        other = vo.VirtualOperator(P, port=0, seed=4).look
        self.assertLess(abs(sum(a * b for a, b in zip(other, looks[0]))), 0.3)  # another person looks different

    def test_the_person_moves_with_the_robot_pose(self):
        op = self.make()
        self.feed(op, at_robot(3.0, 1.0))
        _, answer, _ = op.apply(0.05, (1.0, 1.0, 0.0), room(), None, True)  # the robot drove to (1, 1)
        self.assertAlmostEqual(answer.people[0].bearing_deg,
                               math.degrees(math.atan2(-P["camera_y"], 2.0 - P["camera_x"])), places=6)


class Timeout(Operator):
    def test_without_a_state_for_half_a_second_the_person_vanishes(self):
        op = self.make()
        self.feed(op, at_robot(2.0, 0.0), now=0.0)
        sensed, answer, alive = op.apply(0.45, (0.0, 0.0, 0.0), room(), None, True)
        self.assertNotEqual(sensed, room())
        sensed, answer, alive = op.apply(0.55, (0.0, 0.0, 0.0), room(), Answer(0.5, [Seen(0.0, 0.9, (1.0,), [0, 0, 1, 1])]), True)
        self.assertEqual(sensed, room())  # the real scan only
        self.assertIsNone(answer)  # and no camera answer at all: never a real person in this mode
        self.assertFalse(alive)
        self.assertIsNone(op.report["person"])

    def test_nothing_before_the_first_state(self):
        op = self.make()
        self.assertEqual(op.apply(0.0, (0.0, 0.0, 0.0), room(), None, True), (room(), None, False))

    def test_the_report_goes_in_the_record(self):
        op = self.make()
        self.feed(op, at_robot(2.0, 0.0), now=1.0)
        op.apply(1.1, (0.0, 0.0, 0.0), room(), None, True)
        report = json.loads(json.dumps(op.report))  # it must go into log.jsonl as it is
        self.assertEqual(report["person"], [2.0, 0.0, round(math.pi, 3)])
        self.assertTrue(report["seen"])
        self.assertGreater(report["returns"], 0)
        self.assertEqual(report["from"], "127.0.0.1")
        self.assertAlmostEqual(report["age"], 0.1, places=3)
        self.assertTrue(report["jetson_link"])


class Words(Operator):
    def test_spoken_words_are_the_voice_flows_commands(self):
        self.assertEqual(vo.COMMANDS, {"follow": "follow", "stop": "wait", "done": "return", "back": "return",
                                       "bag": "bag"})
        for word in vo.COMMANDS:
            self.assertIn(vo.COMMANDS[word], dashboard.COMMANDS)

    def test_a_word_said_over_several_packets_is_one_command(self):
        controls = dashboard.Controls(0.12, clock=lambda: self.now)
        op = self.make(controls=controls)
        said = [None, "follow", "follow", "follow", None, "stop", "stop", "back", None, "follow"]
        for i, word in enumerate(said):
            self.feed(op, dict(at_robot(2.0, 0.0), t=0.1 * i, speaking=word), now=0.1 * i)
            op.apply(0.1 * i + 0.01, (0.0, 0.0, 0.0), room(), None, True)
        self.assertEqual(controls.take_commands(), ["follow", "wait", "return", "follow"])

    def test_a_fresh_state_is_the_pages_heartbeat(self):
        controls = dashboard.Controls(0.12, clock=lambda: self.now)
        op = self.make(controls=controls)
        self.feed(op, at_robot(2.0, 0.0), now=5.0)
        op.apply(5.05, (0.0, 0.0, 0.0), room(), None, True)
        self.assertTrue(controls.page_alive(5.1))
        op.apply(5.6, (0.0, 0.0, 0.0), room(), None, True)  # the PC went quiet
        self.assertFalse(controls.page_alive(7.1))

    def test_no_command_from_a_stale_state(self):
        controls = dashboard.Controls(0.12, clock=lambda: self.now)
        op = self.make(controls=controls)
        self.feed(op, dict(at_robot(2.0, 0.0), speaking="follow"), now=0.0)
        op.apply(1.0, (0.0, 0.0, 0.0), room(), None, True)
        self.assertEqual(controls.take_commands(), [])


class Pointing(Operator):
    def test_the_floor_spot_pointed_at_goes_in_the_report(self):
        op = self.make()
        self.feed(op, dict(at_robot(2.0, 0.0), pointing=[1.5, -0.75]), now=0.0)
        op.apply(0.1, (0.0, 0.0, 0.0), room(), None, True)
        self.assertEqual(op.report["pointing"], [1.5, -0.75])  # odometry frame, as sent
        self.feed(op, dict(at_robot(2.0, 0.0), t=0.2, pointing=None), now=0.2)
        op.apply(0.3, (0.0, 0.0, 0.0), room(), None, True)
        self.assertIsNone(op.report["pointing"])

    def test_no_pointing_without_a_fresh_state(self):
        op = self.make()
        self.feed(op, dict(at_robot(2.0, 0.0), pointing=[1.0, 1.0]), now=0.0)
        op.apply(0.6, (0.0, 0.0, 0.0), room(), None, True)
        self.assertIsNone(op.report["pointing"])

    def test_a_bad_pointing_is_refused(self):
        op = self.make()
        for bad in ([1.0], [1.0, "x"], [1.0, float("inf")], "here", {"x": 1}, [True, 1.0]):
            self.assertFalse(op.receive(json.dumps(dict(at_robot(2.0, 0.0), pointing=bad)).encode(),
                                        ("127.0.0.1", 1)), bad)


class Bags(Operator):
    def test_a_virtual_bag_is_a_circle_in_the_scan_and_in_the_report(self):
        op = self.make()
        self.feed(op, dict(at_robot(2.5, -1.0), bags=[[1.5, 0.0]]), now=0.0)
        sensed, _, _ = op.apply(0.1, (0.0, 0.0, 0.0), room(), None, True)
        ahead = [m for deg, m in sensed if abs(deg) < 0.3]
        self.assertAlmostEqual(ahead[0], 1.5 - P["lidar_x"] - vo.BAG_RADIUS, places=3)  # its near side
        self.assertEqual(op.report["bags"], [[1.5, 0.0]])  # odometry frame, as sent
        self.assertEqual([round(m, 3) for deg, m in room() if abs(deg) < 0.3], [3.0])  # the real scan is untouched

    def test_a_real_return_nearer_on_the_same_bearing_wins(self):
        sensed, shown, hidden = vo.inject(room(box=(-5.0, 5.0, 0.8)), [], P, [(1.5, 0.0)])
        self.assertEqual({m for deg, m in sensed if abs(deg) < 5.0}, {0.8})
        self.assertEqual((shown, hidden), (0, 0))  # bags are not counted as legs

    def test_bags_go_with_the_person(self):
        op = self.make()
        self.feed(op, dict(at_robot(2.5, -1.0), bags=[[1.5, 0.0]]), now=0.0)
        sensed, _, _ = op.apply(0.6, (0.0, 0.0, 0.0), room(), None, True)  # the PC went quiet
        self.assertEqual(sensed, room())
        self.assertEqual(op.report["bags"], [])

    def test_bad_bags_are_refused(self):
        op = self.make()
        for bad in ([1.0, 2.0], [[1.0]], [[1.0, "x"]], [[1.0, float("nan")]], "here",
                    [[0.0, 0.0]] * (vo.MAX_BAGS + 1)):
            self.assertFalse(op.receive(json.dumps(dict(at_robot(2.0, 0.0), bags=bad)).encode(),
                                        ("127.0.0.1", 1)), bad)
        self.assertTrue(op.receive(json.dumps(dict(at_robot(2.0, 0.0), bags=[[1.0, 2.0]])).encode(), ("127.0.0.1", 1)))


class Packets(Operator):
    def test_the_allow_list(self):
        op = self.make(allow=["192.168.31.50"])
        for addr, ok in (("127.0.0.1", True), ("192.168.31.50", True), ("100.73.128.36", True),
                         ("100.127.255.254", True), ("192.168.31.51", False), ("100.128.0.1", False),
                         ("10.0.0.2", False), ("not an address", False)):
            self.assertEqual(op.allowed(addr), ok, addr)

    def test_refused_packets_change_nothing(self):
        op = self.make()
        bad = [b"not json", b"[1, 2]", json.dumps(dict(at_robot(2.0, 0.0), x=float("nan"))).encode(),
               json.dumps(dict(at_robot(2.0, 0.0), speaking="dance")).encode(),
               json.dumps({"t": 0.0, "x": 1.0}).encode(), json.dumps(dict(at_robot(2.0, 0.0), x=True)).encode(),
               b"{" + b" " * 5000 + b"}"]
        for data in bad:
            self.assertFalse(op.receive(data, ("127.0.0.1", 1)), data[:40])
        self.assertFalse(self.feed(op, at_robot(2.0, 0.0), addr="192.168.31.99"))  # not on the list
        self.assertIsNone(op.state)
        self.assertEqual(op.refused, len(bad) + 1)

    def test_an_older_packet_than_the_last_is_dropped(self):
        op = self.make()
        self.assertTrue(self.feed(op, dict(at_robot(2.0, 0.0), t=5.0)))
        self.assertFalse(self.feed(op, dict(at_robot(9.0, 0.0), t=4.9)))
        self.assertEqual(op.state[:2], (2.0, 0.0))

    def test_a_new_sender_clock_is_taken(self):
        op = self.make()  # the PC app restarted: its t starts again from 0
        self.feed(op, dict(at_robot(2.0, 0.0), t=500.0), now=0.0)
        self.assertTrue(self.feed(op, dict(at_robot(3.0, 0.0), t=1.0), now=vo.TIMEOUT + 0.1))

    def test_udp_on_the_loopback(self):
        op = vo.VirtualOperator(P, port=0, host="127.0.0.1", seed=1)
        op.start()
        self.addCleanup(op.stop)
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.sendto(json.dumps(at_robot(2.0, 0.25)).encode(), ("127.0.0.1", op.port))
        deadline = time.monotonic() + 2.0
        while op.state is None and time.monotonic() < deadline:
            time.sleep(0.01)
        self.assertEqual(op.state[:2], (2.0, 0.25))


class Safety(unittest.TestCase):
    def test_must_stop_on_the_real_points_ignores_the_injected_legs(self):
        # Why: the virtual person is a test aid for the brain. The real LiDAR's stop must decide on the real
        # room only: never made weaker (no real return is taken away) and never stopped by a person who is
        # not there. The brain still keeps its gap to the person, as to a real one.
        import run_follow
        import safety
        real = room(wall=3.0)
        sensed, shown, _ = vo.inject(real, [(0.45, 0.09), (0.45, -0.09)], P)  # 0.25 m ahead of the front edge
        self.assertGreater(shown, 0)
        self.assertTrue(safety.must_stop(run_follow.scan_points(sensed, P), 0.1, 0.0, P))
        self.assertFalse(safety.must_stop(run_follow.scan_points(real, P), 0.1, 0.0, P))


if __name__ == "__main__":
    unittest.main()
