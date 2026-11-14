import json
import unittest
import urllib.error
import urllib.request

import dashboard


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


class ServerTest(unittest.TestCase):
    def setUp(self):
        self.controls = dashboard.Controls(0.12)
        self.page = dashboard.Dashboard(self.controls, {"drive": False, "seconds": 30}, port=0, host="127.0.0.1")
        self.page.start()
        self.addCleanup(self.page.stop)
        self.base = "http://127.0.0.1:%d" % self.page.server.server_address[1]

    def post(self, change):
        request = urllib.request.Request(self.base + "/control", data=json.dumps(change).encode(), method="POST",
                                         headers={"Content-Type": "application/json"})
        try:
            with urllib.request.urlopen(request, timeout=5) as r:
                return r.status, json.loads(r.read())
        except urllib.error.HTTPError as error:
            return error.code, json.loads(error.read())

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


if __name__ == "__main__":
    unittest.main()
