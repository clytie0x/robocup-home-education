import json
import os
import re
import tempfile
import unittest
from pathlib import Path
from unittest import mock

import camera
import console_gate
import lidar
import params
import wheels

REPO = Path(__file__).resolve().parents[3]
# a call that opens a device: a serial port, a camera, the servo bus
OPENS = re.compile(r"serial\.Serial\(|VideoCapture\(|\bRealArm\(|OpenNI2Driver")
NOT_OPENERS = {  # files that open a device but are no program of their own, or are gated elsewhere
    "arm/so100_driver/so100_driver/feetech.py": "the servo bus class; driver_node and calibrate_cli gate before it",
    "arm/so100_driver/so100_driver/hardware.py": "RealArm itself; driver_node gates before it",
    "arm/so100_moveit_config/launch/arm.launch.py": "the launch file; its driver node gates",
    "arm/so100_moveit_config/launch/camera_nodes.launch.py": "the Xtion on the Jetson; armstack, started by the agent",
}


class GateTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.log = Path(self.tmp.name) / "logs" / "console" / "commands.jsonl"

    def tearDown(self):
        self.tmp.cleanup()

    def require(self, environ):
        console_gate.require("lidar", environ=environ, clock=lambda: 1234.5, log=self.log)

    def test_refuses_outside_the_console(self):
        for environ in ({}, {console_gate.ENV: ""}, {console_gate.MAINTENANCE_ENV: "  "}, {console_gate.CONSOLE_ENV: "0"}):
            with self.assertRaises(RuntimeError) as caught:
                self.require(environ)
            self.assertIn("lidar", str(caught.exception))
            self.assertIn("through the console", str(caught.exception))
            self.assertIn("HMC_MAINTENANCE=<reason>", str(caught.exception))
        self.assertFalse(self.log.exists())

    def test_passes_in_a_console_job(self):
        self.require({console_gate.ENV: "20261116-101500-check-wheel"})
        self.assertFalse(self.log.exists())

    def test_passes_in_the_console_itself(self):
        self.require({console_gate.CONSOLE_ENV: "1"})
        self.assertFalse(self.log.exists())

    def test_maintenance_passes_and_logs_one_line(self):
        self.require({console_gate.MAINTENANCE_ENV: "new LiDAR cable", "USER": "pi",
                      "SSH_CLIENT": "192.168.31.5 51234 22", "HMC_CONSOLE_PIN": "4321", "SECRET_TOKEN": "abc"})
        lines = self.log.read_text(encoding="utf-8").splitlines()
        self.assertEqual(len(lines), 1)
        entry = json.loads(lines[0])
        self.assertEqual(set(entry), {"time", "what", "reason", "pid", "user", "ssh"})
        self.assertEqual((entry["time"], entry["what"], entry["reason"], entry["user"], entry["ssh"]),
                         (1234.5, "lidar", "new LiDAR cable", "pi", "192.168.31.5 51234 22"))
        self.assertNotIn("4321", lines[0])
        self.assertNotIn("abc", lines[0])

    def test_the_pi_logs_in_the_console_log(self):  # the jetson/ gate with the Pi's own log
        self.assertEqual(console_gate.LOG, Path(console_gate.__file__).resolve().parent / "logs" / "console" / "commands.jsonl")
        self.assertEqual(console_gate.ENV, "HMC_CONSOLE_JOB")

    def test_the_devices_refuse_to_open_outside_the_console(self):
        with mock.patch.dict(os.environ):
            for name in (console_gate.ENV, console_gate.CONSOLE_ENV, console_gate.MAINTENANCE_ENV):
                os.environ.pop(name, None)
            for open_device in (lidar.LidarSession, lambda: lidar.read_scan(seconds=0), camera.Camera,
                                lambda: wheels.Wheels(params.load(), start_thread=False)):
                with self.assertRaisesRegex(RuntimeError, "REFUSED"):
                    open_device()

    @unittest.skipUnless((REPO / "arm").exists(), "arm/ is not here (the Pi has only hmc-base)")
    def test_every_program_that_opens_a_device_calls_the_gate(self):
        left = []
        for folder in ("base", "arm"):
            for path in sorted((REPO / folder).rglob("*.py")):
                rel = path.relative_to(REPO).as_posix()
                if "tests" in path.parts or "test" in path.parts or path.name.startswith("test_") or rel in NOT_OPENERS:
                    continue
                text = path.read_text(encoding="utf-8", errors="replace")
                if OPENS.search(text) and "console_gate" not in text:
                    left.append(rel)
        self.assertEqual(left, [], "call console_gate.require(...) before the device opens")


if __name__ == "__main__":
    unittest.main()
