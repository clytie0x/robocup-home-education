import json
import os
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import console_gate  # noqa: E402


class GateTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.log = Path(self.tmp.name) / "logs" / "commands.jsonl"

    def tearDown(self):
        self.tmp.cleanup()

    def require(self, environ):
        console_gate.require("arm", environ=environ, clock=lambda: 1234.5, log=self.log)

    def test_refuses_outside_the_console(self):
        for environ in ({}, {console_gate.ENV: ""}, {console_gate.MAINTENANCE_ENV: "  "}, {"HMC_CONSOLE": "1"}):
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

    def test_the_log_is_in_the_jetson_folder(self):
        self.assertEqual(console_gate.LOG, Path(console_gate.__file__).resolve().parent / "logs" / "commands.jsonl")


if __name__ == "__main__":
    unittest.main()
