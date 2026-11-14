import http.client
import json
import time
import unittest
import urllib.error
import urllib.request

import numpy

import dashboard
import live_feed


class ControlsTest(unittest.TestCase):
    def test_switches_take_only_sane_values(self):
        c = dashboard.Controls(0.12)
        self.assertIsNone(c.apply({"pause": True}))
        self.assertTrue(c.snapshot()["paused"])
        self.assertIsNone(c.apply({"max_speed": 0.05}))
        self.assertEqual(c.snapshot()["max_speed"], 0.05)
        for bad in ({"max_speed": 0.2}, {"max_speed": -0.1}, {"max_speed": True}, {"max_speed": "fast"},
                    {"pause": "yes"}, {"end": False}, {"drive": True}, {"pause": True, "search": False}, [1]):
            self.assertIsNotNone(c.apply(bad), bad)
        self.assertEqual(c.snapshot()["max_speed"], 0.05)  # the refused ones changed nothing

    def test_task_commands_are_queued_in_order_and_taken_once(self):
        c = dashboard.Controls(0.12)
        for name in ("follow", "wait", "resume", "return"):
            self.assertIsNone(c.command(name))
        for bad in ("stop", "ping", "dance", None, 1, ["follow"]):  # STOP stays the page's end switch
            self.assertIsNotNone(c.command(bad), bad)
        self.assertEqual(c.take_commands(), ["follow", "wait", "resume", "return"])
        self.assertEqual(c.take_commands(), [])

    def test_only_the_last_ten_presses_are_kept(self):
        c = dashboard.Controls(0.12)
        for _ in range(30):  # pressed while run_follow waits for Enter and takes nothing
            c.command("wait")
        c.command("resume")
        taken = c.take_commands()
        self.assertEqual(len(taken), dashboard.COMMANDS_KEPT)
        self.assertEqual(taken[-1], "resume")

    def test_a_status_poll_is_the_pages_heartbeat(self):
        now = [0.0]
        c = dashboard.Controls(0.12, clock=lambda: now[0])
        self.assertFalse(c.page_alive(0.0))  # never polled
        now[0] = 1.0
        c.polled()
        self.assertTrue(c.page_alive(2.9))
        self.assertFalse(c.page_alive(3.0))  # dashboard.PAGE_ALIVE without a poll


class ServerTest(unittest.TestCase):
    def setUp(self):
        self.controls = dashboard.Controls(0.12)
        self.page = dashboard.Dashboard(self.controls, {"drive": False, "seconds": 30}, port=0, host="127.0.0.1")
        self.page.start()
        self.addCleanup(self.page.stop)
        self.base = "http://127.0.0.1:%d" % self.page.server.server_address[1]

    def post(self, change, path="/control", headers=None):
        headers = {"Content-Type": "application/json", "X-HMC-Console": "1"} if headers is None else headers
        request = urllib.request.Request(self.base + path, data=json.dumps(change).encode(), method="POST",
                                         headers=headers)
        try:
            with urllib.request.urlopen(request, timeout=5) as r:
                return r.status, json.loads(r.read())
        except urllib.error.HTTPError as error:
            return error.code, json.loads(error.read())

    def test_command_takes_the_task_buttons_and_refuses_the_rest(self):
        self.assertEqual(self.post({"command": "follow"}, "/command"), (200, {"error": None, "command": "follow"}))
        self.assertEqual(self.post({"command": "return"}, "/command")[0], 200)
        for bad in ({"command": "stop"}, {"cmd": "follow"}, {"command": "wait", "extra": 1}, ["wait"], "wait"):
            code, got = self.post(bad, "/command")
            self.assertEqual(code, 400, bad)
            self.assertTrue(got["error"], bad)
        self.assertEqual(self.controls.take_commands(), ["follow", "return"])

    def test_the_json_api_answers_other_origins_for_reading_only(self):
        with urllib.request.urlopen(self.base + "/status", timeout=5) as r:
            self.assertEqual(r.headers["Access-Control-Allow-Origin"], "*")
            self.assertIs(json.loads(r.read())["remote"], False)
        self.assertFalse(self.controls.page_alive(time.monotonic()))  # a plain status read is no heartbeat
        with urllib.request.urlopen(self.base + "/status?heartbeat=1", timeout=5) as r:
            self.assertIs(json.loads(r.read())["remote"], False)
        self.assertTrue(self.controls.page_alive(time.monotonic()))  # that GET was a heartbeat
        request = urllib.request.Request(self.base + "/command", method="OPTIONS")
        with urllib.request.urlopen(request, timeout=5) as r:
            self.assertEqual(r.status, 204)
            self.assertNotIn("POST", r.headers.get("Access-Control-Allow-Methods", ""))
            self.assertIsNone(r.headers.get("Access-Control-Allow-Headers"))  # so no other site's page may send X-HMC-Console
        remote = dashboard.Dashboard(self.controls, {"drive": False, "seconds": 30}, port=0, host="127.0.0.1", remote=True)
        self.assertIs(remote.status()["remote"], True)
        remote.server.server_close()

    def test_a_post_needs_json_and_the_console_header_but_stop_never_does(self):
        json_only = {"Content-Type": "application/json"}
        self.assertEqual(self.post({"command": "follow"}, "/command", json_only)[0], 403)
        self.assertEqual(self.post({"pause": True}, "/control", json_only)[0], 403)
        plain = {"Content-Type": "text/plain", "X-HMC-Console": "1"}  # what a form on another site can send
        self.assertEqual(self.post({"command": "follow"}, "/command", plain)[0], 415)
        self.assertEqual(self.controls.take_commands(), [])
        self.assertFalse(self.controls.snapshot()["paused"])
        request = urllib.request.Request(self.base + "/command", data=json.dumps({"command": "wait"}).encode(),
                                         method="POST", headers={"Content-Type": "application/json", "X-HMC-Console": "1"})
        with urllib.request.urlopen(request, timeout=5) as r:
            self.assertIsNone(r.headers.get("Access-Control-Allow-Origin"))  # no CORS on what moves the robot
        code, got = self.post({"end": True}, "/control", {"Content-Type": "text/plain"})  # STOP: from anywhere
        self.assertEqual(code, 200)
        self.assertTrue(self.controls.snapshot()["end"])

    def test_the_page_has_the_task_buttons(self):
        with urllib.request.urlopen(self.base + "/", timeout=5) as r:
            page = r.read()
        for button in (b'id="follow"', b'id="wait"', b'id="resume"', b'id="goback"', b'"/command"', b'"X-HMC-Console": "1"'):
            self.assertIn(button, page)

    def test_the_page_shows_the_cable(self):
        with urllib.request.urlopen(self.base + "/", timeout=5) as r:
            page = r.read()
        for part in (b'id="cable"', b"r.cable_deg"):
            self.assertIn(part, page)

    def test_status_page_and_switches(self):
        self.page.latest = {"locked": True, "v": 0.1}
        with urllib.request.urlopen(self.base + "/status", timeout=5) as r:
            status = json.loads(r.read())
        self.assertEqual(status["record"], {"locked": True, "v": 0.1})
        self.assertFalse(status["controls"]["paused"])
        with urllib.request.urlopen(self.base + "/", timeout=5) as r:
            self.assertIn(b"Robot dashboard", r.read())
        self.assertEqual(self.post({"pause": True})[0], 200)
        self.assertTrue(self.controls.snapshot()["paused"])
        code, got = self.post({"max_speed": 1.0})
        self.assertEqual(code, 400)
        self.assertIn("0.12", got["error"])

    def get(self, path):
        try:
            with urllib.request.urlopen(self.base + path, timeout=5) as r:
                return r.status, r.headers, r.read()
        except urllib.error.HTTPError as error:
            return error.code, error.headers, error.read()

    def test_no_live_view_without_a_feed(self):
        for path in ("/scan", "/stream"):
            code, headers, body = self.get(path)
            self.assertEqual(code, 503, path)
            self.assertTrue(json.loads(body)["error"], path)
            self.assertEqual(headers["Access-Control-Allow-Origin"], "*")

    def test_scan_and_stream_from_the_feed(self):
        self.page.feed = live_feed.LiveFeed(encode=lambda frame: b"JPEG-" + frame)
        self.assertEqual(self.get("/stream")[0], 503)  # no frame yet
        self.page.feed.put(b"one", numpy.array([[1.0, 0.0]]), {"phase": "idle", "stop": "idle"}, time.monotonic())
        code, headers, body = self.get("/scan")
        self.assertEqual(code, 200)
        self.assertEqual(headers["Access-Control-Allow-Origin"], "*")
        scan = json.loads(body)
        self.assertEqual((scan["points"], scan["phase"], scan["stop"]), ([[1.0, 0.0]], "idle", "idle"))
        with urllib.request.urlopen(self.base + "/stream?t=1", timeout=5) as r:
            self.assertIn("multipart/x-mixed-replace", r.headers["Content-Type"])
            first = b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: 8\r\n\r\nJPEG-one"
            self.assertEqual(r.read(len(first)), first)

    def test_at_most_three_camera_viewers_at_once(self):
        class Endless:
            """A feed whose stream never ends: a viewer stays until it goes away."""

            def has_frame(self):
                return True

            def stream(self):
                while True:
                    yield b"JPEG"
                    time.sleep(0.01)

        self.page.feed = Endless()
        port = self.page.server.server_address[1]
        viewers = []  # (connection, response): a response let go closes its socket

        def view():
            viewer = http.client.HTTPConnection("127.0.0.1", port, timeout=5)
            viewer.request("GET", "/stream")
            viewers.append((viewer, viewer.getresponse()))
            return viewers[-1][1]

        for _ in range(dashboard.MAX_STREAMS):
            self.assertEqual(view().status, 200)
        refused = view()
        self.assertEqual(refused.status, 503)
        self.assertIn("viewers", json.loads(refused.read())["error"])
        self.assertEqual(self.get("/status")[0], 200)  # the rest of the page still answers
        gone, response = viewers.pop(0)  # one goes away: its place is free once its next frame fails
        response.close()
        gone.close()
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline and self.page.streams >= dashboard.MAX_STREAMS:
            time.sleep(0.02)
        self.assertEqual(view().status, 200)
        for viewer, response in viewers:
            response.close()
            viewer.close()

    def test_the_scan_json_is_made_once_for_every_viewer(self):
        made = []

        class Counting:
            at, clock = 1.0, staticmethod(time.monotonic)

            def scan_json(self, now):
                made.append(now)
                return b'{"points": []}'

        self.page.feed = Counting()
        for _ in range(5):  # five viewers asking within the same scan
            self.assertEqual(self.get("/scan")[0], 200)
        self.assertEqual(len(made), 1)
        self.page.feed.at = 1.1  # the next scan
        self.get("/scan")
        self.assertEqual(len(made), 2)

    def test_a_feed_that_cannot_encode_answers_503(self):
        def no_cv2(frame):
            raise ImportError("No module named 'cv2'")
        self.page.feed = live_feed.LiveFeed(encode=no_cv2)
        self.page.feed.put(b"one", None, {}, time.monotonic())
        code, _, body = self.get("/stream")
        self.assertEqual(code, 503)
        self.assertIn("cv2", json.loads(body)["error"])
        self.assertEqual(self.get("/status")[0], 200)  # the page goes on


if __name__ == "__main__":
    unittest.main()
