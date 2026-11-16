import json
import os
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import console_gate  # noqa: E402

ARM_COPY = Path(__file__).resolve().parents[3] / "arm" / "so100_driver" / "so100_driver" / "console_gate.py"


class GateTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.log = Path(self.tmp.name) / "logs" / "commands.jsonl"

    def tearDown(self):
        self.tmp.cleanup()

    def require(self, environ):
        console_gate.require("arm", environ=environ, clock=lambda: 1234.5, log=self.log)

    def test_refuses_outside_the_console(self):
        for environ in ({}, {console_gate.ENV: ""}, {console_gate.MAINTENANCE_ENV: "  "}, {console_gate.CONSOLE_ENV: "0"}):
            with self.assertRaisesRegex(RuntimeError, "REFUSED: arm: .*HMC_MAINTENANCE=<reason>"):
                self.require(environ)
        self.assertFalse(self.log.exists())

    def test_passes_in_a_program_the_agent_started(self):
        self.require({console_gate.ENV: "agent:arm"})
        self.assertFalse(self.log.exists())

    def test_maintenance_passes_and_logs_one_line(self):
        self.require({console_gate.MAINTENANCE_ENV: "gripper servo swap", "USER": "robotpsu",
                      "SSH_CLIENT": "192.168.31.5 51234 22", "SECRET_TOKEN": "abc"})
        lines = self.log.read_text(encoding="utf-8").splitlines()
        self.assertEqual(len(lines), 1)
        entry = json.loads(lines[0])
        self.assertEqual(set(entry), {"time", "what", "reason", "pid", "user", "ssh"})
        self.assertEqual((entry["time"], entry["what"], entry["reason"], entry["user"]),
                         (1234.5, "arm", "gripper servo swap", "robotpsu"))
        self.assertNotIn("abc", lines[0])

    def test_the_log_is_in_the_jetson_folder(self):  # base/jetson is ~/hmc-follow on the Jetson
        self.assertEqual(console_gate.LOG, Path.home() / "hmc-follow" / "logs" / "commands.jsonl")

    @unittest.skipUnless(ARM_COPY.exists(), "no arm/ next to base/ (the Jetson's ~/hmc-follow)")
    def test_the_arm_package_has_a_byte_copy(self):  # one gate: change this file, then copy it there
        self.assertEqual(ARM_COPY.read_bytes(), Path(console_gate.__file__).read_bytes())


if __name__ == "__main__":
    unittest.main()
