"""Tools that move the arm need the console or a logged HMC_MAINTENANCE repair (design 2026-11-16). No ROS."""
import io
import json
import os
import sys
import tempfile
import unittest
from contextlib import redirect_stderr
from pathlib import Path

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from so100_driver import console_gate  # noqa: E402

ARM = Path(__file__).resolve().parents[2]


class GateTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.log = Path(self.tmp.name) / 'hmc-follow' / 'logs' / 'commands.jsonl'

    def tearDown(self):
        self.tmp.cleanup()

    def require(self, environ, log=None):
        console_gate.require('arm_cli', environ=environ, clock=lambda: 1234.5, log=log)

    def test_refuses_outside_the_console(self):
        for environ in ({}, {console_gate.ENV: ''}, {console_gate.MAINTENANCE_ENV: '  '}, {console_gate.CONSOLE_ENV: '0'}):
            with self.assertRaises(RuntimeError) as caught:
                self.require(dict(environ, HMC_COMMAND_LOG=str(self.log)))
            self.assertEqual(str(caught.exception),
                             'REFUSED: arm_cli: start this through the console (robotctl on the Pi / the console page),'
                             ' or set HMC_MAINTENANCE=<reason> for a logged repair')
        self.assertFalse(self.log.exists())

    def test_passes_in_a_console_job(self):
        self.require({console_gate.ENV: '20261116-101500-arm', 'HMC_COMMAND_LOG': str(self.log)})
        self.assertFalse(self.log.exists())

    def test_passes_in_the_console_itself(self):
        self.require({console_gate.CONSOLE_ENV: '1', 'HMC_COMMAND_LOG': str(self.log)})
        self.assertFalse(self.log.exists())

    def test_maintenance_passes_and_logs_one_line(self):
        self.require({console_gate.MAINTENANCE_ENV: 'new gripper servo', 'USER': 'jetson',
                      'SSH_CLIENT': '192.168.31.5 51234 22', 'HMC_COMMAND_LOG': str(self.log), 'SECRET_TOKEN': 'abc'})
        lines = self.log.read_text(encoding='utf-8').splitlines()
        self.assertEqual(len(lines), 1)
        entry = json.loads(lines[0])
        self.assertEqual(set(entry), {'time', 'what', 'reason', 'pid', 'user', 'ssh'})
        self.assertEqual((entry['time'], entry['what'], entry['reason'], entry['pid'], entry['user'], entry['ssh']),
                         (1234.5, 'arm_cli', 'new gripper servo', os.getpid(), 'jetson', '192.168.31.5 51234 22'))
        self.assertNotIn('abc', lines[0])

    def test_default_log_is_in_hmc_follow(self):
        self.assertEqual(console_gate.LOG, Path.home() / 'hmc-follow' / 'logs' / 'commands.jsonl')

    def test_unwritable_log_warns_and_passes(self):
        blocker = Path(self.tmp.name) / 'a_file'
        blocker.write_text('x')
        err = io.StringIO()
        with redirect_stderr(err):
            self.require({console_gate.MAINTENANCE_ENV: 'bench'}, log=blocker / 'commands.jsonl')
        self.assertIn('WARNING', err.getvalue())

    def test_moving_tools_call_the_gate(self):
        pkg = ARM / 'so100_driver' / 'so100_driver'
        for path in (pkg / 'arm_client.py', pkg / 'calibrate_cli.py', pkg / 'motion_test.py', pkg / 'control_panel.py',
                     ARM / 'gcam.py', ARM / 'cam_calib_run.py', ARM / 'tools' / 'bagpick' / 'bagpick.py',
                     ARM / 'tools' / 'bagpick' / 'gcam.py'):
            self.assertRegex(path.read_text(encoding='utf-8'), r'console_gate(\.require)?\(', path.name)


if __name__ == '__main__':
    unittest.main()
