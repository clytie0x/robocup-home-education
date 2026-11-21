import json
import os
import shutil
import signal
import subprocess
import sys
import tempfile
import threading
import time
import unittest
import urllib.error
import urllib.request
from http.server import ThreadingHTTPServer
from pathlib import Path

HERE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE / "tools"))
import console  # noqa: E402
import job_guard  # noqa: E402

P = {"max_speed": 0.12}
SLEEP = [sys.executable, "-c", "import time; print('job up', flush=True); time.sleep(30)"]
HEART = "import sys, time\nwhile True:\n    open(sys.argv[1], 'a').write('.')\n    time.sleep(0.05)\n"
# a job that ignores the polite stops and has a child of its own (as safe_run has run_follow)
STUBBORN = ("import signal, subprocess, sys, time\n"
            "for name in ('SIGTERM', 'SIGBREAK'):\n"
            "    if hasattr(signal, name): signal.signal(getattr(signal, name), signal.SIG_IGN)\n"
            "subprocess.Popen([sys.executable, '-c', sys.argv[1], sys.argv[2]])\n"
            "print('stubborn up', flush=True)\n"
            "time.sleep(60)\n")


def wait_for(check, timeout=5.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if check():
            return True
        time.sleep(0.05)
    return check()


class JobsTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.dir = Path(self.tmp.name)
        self.events = []
        self.jobs = console.Jobs(self.dir / "console", release=lambda: self.events.append("release"),
                                 reclaim=lambda: self.events.append("reclaim"),
                                 ask_end=lambda: self.events.append("ask_end"),
                                 stop_wheels=lambda: self.events.append("S#"), end_grace=0.3, term_grace=0.5)

    def tearDown(self):
        self.jobs.shutdown()
        self.tmp.cleanup()

    def popen_recording(self):
        def popen(*args, **kwargs):
            self.events.append("popen")
            return subprocess.Popen(*args, **kwargs)
        return popen

    def test_one_job_at_a_time_and_devices_released_first(self):
        self.jobs.popen = self.popen_recording()
        self.assertIsNone(self.jobs.start("task", "Dry-run task", SLEEP))
        self.assertEqual(self.events, ["release", "popen"])
        error = self.jobs.start("check-wheel", "Wheel test", SLEEP)
        self.assertIn("Dry-run task is running", error)
        self.assertEqual(self.events, ["release", "popen"])  # the second job touched nothing
        self.assertTrue(wait_for(lambda: "job up" in self.jobs.status()["tail"]))
        self.assertTrue(self.jobs.status()["running"])

    def test_devices_come_back_after_the_job(self):
        self.assertIsNone(self.jobs.start("check-floor", "Camera pitch check", [sys.executable, "-c", "print('done')"]))
        self.jobs.watcher.join(10)
        status = self.jobs.status()
        self.assertFalse(status["running"])
        self.assertEqual(status["job"]["code"], 0)
        self.assertEqual(self.events, ["release", "reclaim"])
        self.assertIn("done", status["tail"])
        self.assertTrue(status["tail"][0].startswith("$ "))  # the command line heads the job's log file
        self.assertIsNone(self.jobs.start("check-floor", "again", [sys.executable, "-c", "pass"]))  # free again

    def test_every_job_gets_the_console_job_variable(self):
        # console_gate lets the job open the LiDAR, the camera and the wheel port
        self.assertIsNone(self.jobs.start("check-floor", "Camera pitch check", [
            sys.executable, "-c", "import os; print('job=' + os.environ.get('HMC_CONSOLE_JOB', 'none'))"]))
        self.jobs.watcher.join(10)
        name = Path(self.jobs.status()["job"]["log"]).stem
        self.assertTrue(name.endswith("-check-floor"))
        self.assertIn("job=" + name, self.jobs.status()["tail"])

    def test_devices_that_do_not_close_refuse_the_job(self):
        def stuck():
            raise RuntimeError("the LiDAR reader did not close")
        self.jobs.release = stuck
        error = self.jobs.start("task", "Drive task", SLEEP)
        self.assertIn("could not free", error)
        self.assertEqual(self.events, ["reclaim"])
        self.assertFalse(self.jobs.status()["running"])

    def test_stop_asks_a_task_to_end_first(self):
        self.jobs.start("task", "Drive task", SLEEP)
        proc = self.jobs.proc
        self.jobs.ask_end = lambda: (self.events.append("ask_end"), proc.terminate())  # the run ends itself
        self.assertIsNone(self.jobs.stop(wait=True))
        self.jobs.watcher.join(5)
        self.assertEqual(self.events, ["release", "ask_end", "reclaim"])
        self.assertFalse(any("SIGTERM" in n or "SIGKILL" in n for n in self.jobs.status()["job"]["notes"]))

    def test_stop_kills_the_whole_group(self):
        beat = self.dir / "heartbeat"
        self.jobs.start("task", "Drive task", [sys.executable, "-c", STUBBORN, HEART, str(beat)])
        self.assertTrue(wait_for(lambda: beat.exists() and beat.stat().st_size > 3))
        self.assertIsNone(self.jobs.stop(wait=True))
        self.jobs.watcher.join(5)
        self.assertFalse(self.jobs.status()["running"])
        time.sleep(0.3)
        size = beat.stat().st_size
        time.sleep(0.5)
        self.assertEqual(beat.stat().st_size, size, "the job's child is still running")
        notes = " ".join(self.jobs.status()["job"]["notes"])
        self.assertIn("asked the run to end", notes)
        self.assertIn("SIGKILL", notes)
        self.assertIn("S#", self.events)  # after a SIGKILL the console sends S# itself
        self.assertEqual(self.events[-1], "reclaim")

    def test_stop_with_nothing_running_still_sweeps(self):
        # a run started by hand over ssh, or a guard that died: STOP still asks the run's page to end and sends S#
        note = self.jobs.stop(wait=True)
        self.assertIn("nothing was running", note)
        self.assertEqual(self.events, ["ask_end", "S#"])

    def test_stop_during_the_start_cancels_it(self):
        # Wheel test, then STOP at once while the camera and LiDAR are still closing: the job must never start
        self.jobs.release = lambda: (self.events.append("release"), self.jobs.stop(wait=True))
        self.jobs.popen = self.popen_recording()
        error = self.jobs.start("check-wheel", "Wheel test", SLEEP)
        self.assertIn("STOP", error)
        self.assertNotIn("popen", self.events)
        self.assertEqual(self.events[-1], "reclaim")
        self.assertFalse(self.jobs.status()["running"])
        self.assertFalse(self.jobs.busy)

    def test_a_start_right_after_stop_is_refused(self):
        self.jobs.stop(wait=True)
        self.jobs.popen = self.popen_recording()
        self.assertIn("STOP", self.jobs.start("task", "Drive task", SLEEP))
        self.assertNotIn("popen", self.events)

    def test_stop_ends_what_is_left_of_the_last_jobs_group(self):
        # job_guard (the group's leader) died, its child still drives: STOP signals the whole group anyway
        class Dead:
            pid = 4321

            def poll(self):
                return -9
        alive = [True]
        signals = []

        def signal(proc, hard):
            signals.append((proc.pid, hard))
            alive[0] = False
        self.jobs.proc, self.jobs.signal = Dead(), signal
        self.jobs.group_alive = lambda pgid: alive[0]
        self.jobs.stop(wait=True)
        self.assertEqual(signals, [(4321, False)])
        self.assertEqual(self.events, ["ask_end", "S#"])

    def test_a_run_page_that_does_not_answer_skips_the_grace(self):
        def no_page():
            raise OSError("connection refused")
        self.jobs.ask_end, self.jobs.end_grace = no_page, 5.0
        self.jobs.start("task", "Drive task", SLEEP)
        self.assertTrue(wait_for(lambda: "job up" in self.jobs.status()["tail"]))
        t = time.monotonic()
        self.jobs.stop(wait=True)
        self.assertLess(time.monotonic() - t, 4.0)  # not END_GRACE first
        self.assertIn("SIGTERM", " ".join(self.jobs.status()["job"]["notes"]))

    def test_a_second_stop_hurries_the_one_sequence(self):
        beat = self.dir / "heartbeat"
        self.jobs.end_grace, self.jobs.term_grace = 5.0, 5.0
        self.jobs.start("task", "Drive task", [sys.executable, "-c", STUBBORN, HEART, str(beat)])
        self.assertTrue(wait_for(lambda: beat.exists() and beat.stat().st_size > 3))
        t = time.monotonic()
        self.assertIsNone(self.jobs.stop())
        time.sleep(0.3)
        self.assertIsNone(self.jobs.stop())  # pressed again: the same sequence goes to SIGKILL now
        self.assertTrue(self.jobs.stopped.wait(8))
        self.assertLess(time.monotonic() - t, 4.0)
        self.jobs.watcher.join(5)
        self.assertEqual(self.events.count("S#"), 1)  # one sequence, one S#, before the devices come back
        self.assertEqual(self.events[-1], "reclaim")
        self.assertIn("SIGKILL", " ".join(self.jobs.status()["job"]["notes"]))

    def test_a_quick_second_stop_still_gives_the_run_time_to_end_itself(self):
        # run_follow sends its own S# when asked to end: a second STOP 0.1 s later must not kill it before that
        asked, signals = [], []
        self.jobs.ask_end = lambda: asked.append(time.monotonic())

        def signal(proc, hard):
            signals.append(time.monotonic())
            console.signal_group(proc, hard)
        self.jobs.signal = signal
        self.jobs.start("task", "Drive task", SLEEP)
        self.assertTrue(wait_for(lambda: "job up" in self.jobs.status()["tail"]))
        self.jobs.stop()
        time.sleep(0.05)
        self.jobs.stop()
        self.assertTrue(self.jobs.stopped.wait(8))
        self.assertEqual(len(asked), 1)
        self.assertGreaterEqual(signals[0] - asked[0], console.HURRY_MIN - 0.02)

    def test_job_timing_uses_the_monotonic_clock(self):
        self.assertIs(console.Jobs().clock, time.monotonic)  # an NTP step must not stretch SIGTERM or SIGKILL

    def test_ask_end_fails_on_an_http_error(self):
        with self.assertRaises(OSError):
            console.ask_run_end(fetch=lambda url, body, timeout: (403, {"error": "refused"}))
        with self.assertRaises(OSError):
            console.ask_run_end(fetch=lambda url, body, timeout: (500, None))
        seen = []
        console.ask_run_end(fetch=lambda url, body, timeout: seen.append((url, body)) or (200, {"error": None}))
        self.assertEqual(seen, [(console.RUN_URL + "/control", {"end": True})])

    def test_a_reused_group_number_is_not_ours(self):
        proc = self.dir / "proc"

        def task(pid, pgid, cmd):
            (proc / str(pid)).mkdir(parents=True)
            (proc / str(pid) / "stat").write_text("%d (py thon) S 1 %d %d 0 -1" % (pid, pgid, pgid))
            (proc / str(pid) / "cmdline").write_bytes(b"\0".join(c.encode() for c in cmd) + b"\0")
        task(100, 4321, ["/usr/bin/python3", "/home/hmc/hmc-base/run_follow.py", "--drive"])
        task(200, 7777, ["bash"])
        task(300, 5555, ["sleep", "100"])  # pid 5555's group number, now someone else's
        (proc / "self").mkdir()
        self.assertTrue(console.job_group_alive(4321, proc))
        self.assertFalse(console.job_group_alive(7777, proc))
        self.assertFalse(console.job_group_alive(5555, proc))
        self.assertFalse(console.job_group_alive(9999, proc))
        self.assertFalse(console.job_group_alive(4321, self.dir / "no-proc"))  # not Linux: never signal

    def test_a_job_log_that_cannot_be_made_frees_everything(self):
        blocker = self.dir / "console-file"
        blocker.write_text("not a folder")
        self.jobs.folder = blocker  # mkdir fails, as on a full disk
        error = self.jobs.start("task", "Drive task", SLEEP)
        self.assertIn("could not start", error)
        self.assertEqual(self.events, ["release", "reclaim"])
        self.assertFalse(self.jobs.busy)


class CommandTest(unittest.TestCase):
    def test_drive_goes_through_guard_and_safe_run(self):
        cmd, error = console.task_command(True, 0.12, 5, P, py="py")
        self.assertIsNone(error)
        self.assertEqual(Path(cmd[1]).name, "job_guard.py")
        self.assertEqual(Path(cmd[3]).name, "safe_run.py")
        self.assertEqual(Path(cmd[5]).name, "run_follow.py")
        self.assertEqual(cmd[6:], ["--drive", "--seconds", "480", "--max-speed", "0.12", "--remote",
                                   "--dashboard-host", "127.0.0.1"])  # 5:00 and the executive's way back

    def test_the_runs_page_is_only_for_the_console(self):
        # run_follow's switches and task buttons (port 8080) skip the PIN: only this computer may reach them,
        # and the console reaches them there (STOP, the commands, the live view)
        for drive in (True, False):
            cmd, _ = console.task_command(drive, 0.08, 2, P, py="py")
            self.assertEqual(cmd[cmd.index("--dashboard-host") + 1], "127.0.0.1")
        self.assertTrue(console.RUN_URL.startswith("http://127.0.0.1:"))

    def test_dry_run_has_no_drive(self):
        cmd, error = console.task_command(False, 0.08, 2, P, py="py")
        self.assertIsNone(error)
        self.assertNotIn("--drive", cmd)
        self.assertFalse(any(Path(a).name == "safe_run.py" for a in cmd))

    def test_refusals(self):
        self.assertIn("above max_speed", console.task_command(True, 0.12, 5, {"max_speed": 0.08})[1])
        for speed in (0.1, 0.3, True, None, "0.12"):
            self.assertIsNotNone(console.task_command(True, speed, 5, P)[1], speed)
        for minutes in (0, -1, console.MINUTES_MAX + 1, "5", None):
            self.assertIsNotNone(console.task_command(True, 0.08, minutes, P)[1], minutes)
        self.assertIsNotNone(console.task_command("yes", 0.08, 5, P)[1])

    def test_wheel_test_goes_through_safe_run(self):
        cmd, label, moves = console.check_command("wheel", py="py")
        self.assertTrue(moves)
        self.assertEqual([Path(a).name for a in cmd[1::2]], ["job_guard.py", "safe_run.py", "wheel_test.py"])
        self.assertIsNone(console.check_command("rm"))


def write_run(root, name, records, tail=""):
    d = root / name
    d.mkdir(parents=True)
    (d / "log.jsonl").write_text("".join(json.dumps(r) + "\n" for r in records) + tail)
    return d


class LogsTest(unittest.TestCase):
    def test_newest_first_with_length_and_end(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            write_run(root, "20261114-231000", [{"time": 100.0, "phase": "idle"}, {"time": 160.5, "phase": "follow"},
                                                {"time": 161.0, "stop": "signal", "end": "signal: Ctrl+C"}])
            write_run(root, "20261115-090000", [{"time": 5.0}, {"time": 25.0, "pose": [0, 0, 0]}],
                      tail='{"time": 26.0, "po')  # cut off by a crash
            (root / "20261115-100000").mkdir()  # no log at all
            (root / "console").mkdir()  # not a run
            (root / "base-check").mkdir()
            runs = console.list_runs(root)
        self.assertEqual([r["name"] for r in runs], ["20261115-100000", "20261115-090000", "20261114-231000"])
        self.assertFalse(runs[0]["has_log"])
        self.assertEqual((runs[1]["seconds"], runs[1]["end"]), (20.0, None))
        self.assertEqual((runs[2]["seconds"], runs[2]["end"]), (61.0, "signal: Ctrl+C"))
        self.assertEqual(runs[2]["start"], "2026-11-14 23:10:00")

    def test_last_record_of_a_big_file(self):
        with tempfile.TemporaryDirectory() as tmp:
            records = [{"time": float(i), "pad": "x" * 500} for i in range(400)] + [{"time": 400.0, "end": "done: back at the start"}]
            write_run(Path(tmp), "20261115-120000", records)
            run, = console.list_runs(Path(tmp))
        self.assertEqual((run["seconds"], run["end"]), (400.0, "done: back at the start"))

    def test_missing_root(self):
        self.assertEqual(console.list_runs(Path("does-not-exist-here")), [])


class FakeCamera:
    def __init__(self):
        self.error, self.stopped, self.n = None, False, 0

    def start(self):
        pass

    def latest(self):
        self.n += 1
        return "frame" if self.n % 3 == 0 else None

    def stop(self):
        self.stopped = True


class FakeLidar:
    def __init__(self):
        self.closed, self.scan = False, object()

    def latest(self):
        return self.scan

    def close(self):
        self.closed = True


class DevicesTest(unittest.TestCase):
    def test_absent_devices_do_not_crash(self):
        def missing():
            raise OSError("could not open port /dev/rct_ahr_lidar")
        d = console.Devices(P, open_camera=missing, open_lidar=missing, retry=0.05)
        d.start()
        self.assertTrue(wait_for(lambda: "could not open" in (d.status()["lidar"]["error"] or "")))
        s = d.status()
        self.assertEqual((s["camera"]["state"], s["lidar"]["state"]), ("off", "off"))
        d.stop()
        self.assertTrue(d.status()["held"])

    def test_live_then_released(self):
        cam, lid = FakeCamera(), FakeLidar()
        d = console.Devices(P, open_camera=lambda: cam, open_lidar=lambda: lid, encode=lambda f: b"jpeg",
                            points=lambda s: [(1.0, 0.5)], retry=0.05)
        d.start()
        self.assertTrue(wait_for(lambda: d.status()["camera"]["state"] == "live" and d.status()["lidar"]["state"] == "live"))
        self.assertEqual(json.loads(d.scan.json(d.clock()))["points"], [[1.0, 0.5]])
        d.stop()  # returns only when both are closed
        self.assertTrue(cam.stopped and lid.closed)
        s = d.status()
        self.assertEqual((s["held"], s["camera"]["state"], s["lidar"]["state"]), (True, "held", "held"))

    def test_skeleton_sends_with_pose_only_while_on_and_closes_with_the_devices(self):
        from link import Answer, Seen
        kp = [[1.0, 2.0, 0.9]] * 17

        class FakeLink:
            def __init__(self):
                self.sent, self.started, self.stopped, self.error = [], False, False, None

            def start(self):
                self.started = True

            def send(self, frame, pose=False):
                self.sent.append((frame.jpeg, frame.width, pose))

            def latest(self):
                return Answer(0.0, [Seen(0.0, 0.9, (), [0, 0, 10, 20], kp)]) if self.sent else None

            def stop(self):
                self.stopped = True

        links = []
        d = console.Devices(P, open_camera=lambda: FakeCamera(), open_lidar=lambda: FakeLidar(), encode=lambda f: b"jpeg",
                            points=lambda s: [], retry=0.05, size_of=lambda j: (640, 480),
                            open_link=lambda p: links.append(FakeLink()) or links[-1])
        d.start()
        self.assertTrue(wait_for(lambda: d.status()["camera"]["state"] == "live"))
        self.assertEqual(links, [])  # off: no link, the detector stays free
        d.skeleton_on = True
        self.assertTrue(wait_for(lambda: d.skeleton_status()["people"]))
        s = d.skeleton_status()
        self.assertEqual((s["w"], s["h"], s["people"][0]["kp"]), (640, 480, kp))
        self.assertEqual(links[0].sent[0], (b"jpeg", 640, True))
        d.stop()  # a job takes the camera: the detector link closes with it
        self.assertTrue(links[0].stopped)
        self.assertEqual(len(links), 1)


class VersionTest(unittest.TestCase):
    def test_the_deployed_commit_is_read_and_bad_files_give_none(self):
        d = tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, d, True)
        path = os.path.join(d, "VERSION.json")
        self.assertIsNone(console.deployed_version(path))  # never deployed with deploy.sh
        with open(path, "w") as f:
            json.dump({"commit": "b22c5b2" + "0" * 33, "describe": "b22c5b2", "branch": "sim", "deployed": "x"}, f)
        self.assertEqual(console.deployed_version(path)["describe"], "b22c5b2")
        for bad in ("{", "[]", '{"commit": 5}'):
            with open(path, "w") as f:
                f.write(bad)
            self.assertIsNone(console.deployed_version(path), bad)


class ChildCameraTest(unittest.TestCase):
    """console.ChildCamera over a stand-in child that writes framed JPEGs like tools/camera_pipe.py."""

    CHILD = ("import struct, sys, time\n"
             "for i in range(3):\n"
             "    b = b'jpeg%d' % i\n"
             "    sys.stdout.buffer.write(struct.pack('>I', len(b)) + b); sys.stdout.buffer.flush()\n"
             "    time.sleep(0.05)\n")

    def err(self):
        d = tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, d, True)
        return os.path.join(d, "camera_pipe.err")

    def test_frames_come_through_then_the_end_is_an_error(self):
        cam = console.ChildCamera([sys.executable, "-c", self.CHILD + "sys.stderr.write('no device\\n'); sys.exit(1)\n"],
                                  err_path=self.err())
        cam.start()
        got = []
        self.assertTrue(wait_for(lambda: got.append(cam.latest()) or b"jpeg2" in got or cam.error is not None))
        self.assertIn(b"jpeg2", [g for g in got if g])
        self.assertTrue(wait_for(lambda: cam.error is not None))
        self.assertIn("code 1", str(cam.error))
        self.assertIn("no device", str(cam.error))
        cam.stop()

    def test_stop_ends_a_child_that_keeps_running(self):
        cam = console.ChildCamera([sys.executable, "-c", "import time\nwhile True: time.sleep(0.1)\n"], term_s=1.0,
                                  err_path=self.err())
        cam.start()
        started = time.monotonic()
        cam.stop()
        self.assertLess(time.monotonic() - started, 3.0)
        self.assertIsNotNone(cam.proc.poll())

    def test_a_child_that_ignores_term_is_killed(self):
        child = "import signal, time\nsignal.signal(signal.SIGTERM, signal.SIG_IGN)\nwhile True: time.sleep(0.1)\n"
        cam = console.ChildCamera([sys.executable, "-c", child], term_s=0.5, err_path=self.err())
        cam.start()
        time.sleep(0.3)
        cam.stop()
        self.assertIsNotNone(cam.proc.poll())

    def test_devices_use_the_child_by_default(self):
        d = console.Devices(P)
        self.assertIs(d.open_camera, console._open_child_camera)
        self.assertEqual(d.encode(b"x"), b"x")


class CardsTest(unittest.TestCase):
    PI = {"temp_c": 51.0, "load": [0.4, 0.3, 0.2], "disk_free_gb": 20.0, "uptime_s": 4000}
    DEVICES = {"camera": {"state": "live", "age": 0.1, "error": None}, "lidar": {"state": "off", "age": None, "error": "x"}}
    JETSON = {"links": [{"host": "10.0.0.2", "ok": True, "ms": 0.6}, {"host": "192.168.31.137", "ok": False, "ms": None}],
              "host": "10.0.0.2", "agent": {"programs": {"detector": {"running": True}, "bridge": {"running": False},
                                                         "flow": {"running": True}}},
              "agent_error": None, "webcam": False}

    def cards(self, wheel=None, pi=None):
        return {c["id"]: c for c in console.cards(pi or self.PI, self.DEVICES, wheel, self.JETSON, 1000.0)}

    def test_levels(self):
        c = self.cards()
        self.assertEqual(c["pi"]["level"], "ok")
        self.assertEqual(c["camera"]["level"], "ok")
        self.assertEqual(c["lidar"]["level"], "bad")
        self.assertEqual(c["wheels"]["level"], "warn")
        self.assertEqual(c["link"]["level"], "ok")
        self.assertEqual(c["services"]["level"], "warn")
        self.assertIn("bridge", c["services"]["text"])
        self.assertEqual(c["webcam"]["level"], "warn")

    def test_no_power_is_a_fault_with_the_fix(self):
        c = self.cards({"time": 900.0, "verdict": "no_power", "code": 2,
                        "text": "the wheels get no power: check that the 21V motor battery is plugged in and charged"})
        self.assertEqual(c["wheels"]["level"], "bad")
        self.assertIn("ตรวจแบต 21V ว่าเสียบอยู่และมีไฟ", c["wheels"]["text"])  # the page is in Thai

    def test_wheel_fault_texts_name_the_21v_battery_not_the_power_bank(self):
        self.assertIn("21V", console.WHEEL_TEXT["no_power"])
        for text in console.WHEEL_TEXT.values():
            self.assertNotIn("power bank", text)
            self.assertNotIn("USB-C", text)
        page = (Path(console.__file__).resolve().parent / "console" / "index.html").read_text(encoding="utf-8")
        self.assertIn("แบต 21V", page)
        self.assertNotIn("USB-C", page)

    def test_hot_pi(self):
        self.assertEqual(self.cards(pi=dict(self.PI, temp_c=82.0))["pi"]["level"], "bad")
        self.assertEqual(self.cards(pi=dict(self.PI, temp_c=72.0))["pi"]["level"], "warn")


class FakeJobs:
    def __init__(self):
        self.stopped = 0
        self.started = []  # the kinds of the jobs started
        self.running = None  # {"kind", "label"} of a running job, or None

    def stop(self):
        self.stopped += 1
        return "nothing was running: asked the run's page to end, and sent S#"

    def start(self, *args):
        self.started.append(args[0])
        return None

    def status(self):
        if self.running:
            return {"running": True, "job": dict(self.running), "tail": []}
        return {"running": False, "job": None, "tail": []}


class FakeJetson:
    hosts = ("10.0.0.2",)

    def snapshot(self):
        return {"host": None, "links": [{"host": "10.0.0.2", "ok": None, "ms": None}], "agent": None,
                "agent_error": "not asked yet", "agent_at": None, "webcam": None, "checked": None}

    def act(self, action, name=None):
        return 502, {"error": "the Jetson does not answer"}


class HttpTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.jobs = FakeJobs()
        devices = console.Devices(P)  # never started: nothing is opened
        self.console = console.Console(P, devices, self.jobs, FakeJetson(), run_url="http://127.0.0.1:9",
                                       pin="2468", log_root=Path(self.tmp.name))
        self.server = ThreadingHTTPServer(("127.0.0.1", 0), console.handler_for(self.console))
        threading.Thread(target=self.server.serve_forever, daemon=True).start()
        self.base = "http://127.0.0.1:%d" % self.server.server_address[1]

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()
        self.tmp.cleanup()

    def request(self, path, body=None, pin=None, page=True, kind="application/json", headers=None):
        """page: send X-HMC-Console as the console's own page does."""
        headers = dict({"Content-Type": kind}, **(headers or {}))
        if page:
            headers["X-HMC-Console"] = "1"
        if pin:
            headers["X-Console-Pin"] = pin
        req = urllib.request.Request(self.base + path, None if body is None else json.dumps(body).encode(), headers)
        try:
            with urllib.request.urlopen(req, timeout=5) as r:
                return r.status, r.read()
        except urllib.error.HTTPError as e:
            return e.code, e.read()

    def test_page_and_state(self):
        code, page = self.request("/")
        self.assertEqual(code, 200)
        self.assertIn(b"Robot Console", page)
        code, body = self.request("/api/state")
        state = json.loads(body)
        self.assertEqual(code, 200)
        self.assertEqual(state["speeds"], [0.08, 0.12])
        self.assertTrue(state["pin"])
        self.assertIn("lidar", {c["id"] for c in state["cards"]})

    def test_pin_needed_to_start_but_never_to_stop(self):
        code, body = self.request("/api/task/start", {"drive": True, "speed": 0.08, "minutes": 1})
        self.assertEqual(code, 401)
        self.assertTrue(json.loads(body)["pin"])
        self.assertEqual(self.request("/api/task/start", {"drive": True, "speed": 0.08, "minutes": 1}, pin="1111")[0], 401)
        self.assertEqual(self.request("/api/task/start", {"drive": True, "speed": 0.08, "minutes": 1}, pin="2468")[0], 200)
        code, body = self.request("/api/stop", {})
        self.assertEqual((code, json.loads(body)["error"]), (200, None))
        self.assertEqual(self.jobs.stopped, 1)

    def test_posts_need_json_and_the_console_header_but_stop_does_not(self):
        # another site's page on the same Wi-Fi can send text/plain without a preflight; never a start or a command
        dry = {"drive": False, "speed": 0.08, "minutes": 1}
        self.assertEqual(self.request("/api/task/start", dry, pin="2468", kind="text/plain")[0], 403)
        self.assertEqual(self.request("/api/task/start", dry, pin="2468", page=False)[0], 403)
        self.assertEqual(self.request("/api/run/command", {"command": "follow"}, pin="2468", page=False)[0], 403)
        self.assertEqual(self.request("/api/bank/full", {}, page=False)[0], 403)
        self.assertEqual(self.jobs.started, [])
        self.assertEqual(self.request("/api/stop", {}, kind="text/plain", page=False)[0], 200)  # STOP always works
        self.assertEqual(self.jobs.stopped, 1)

    def test_without_a_pin_nothing_that_drives_starts(self):
        self.console.pin = None
        code, body = self.request("/api/task/start", {"drive": True, "speed": 0.08, "minutes": 1})
        self.assertEqual(code, 403)
        self.assertIn("ตั้ง PIN ก่อน", json.loads(body)["error"])
        code, body = self.request("/api/check/start", {"check": "wheel"})
        self.assertEqual(code, 403)
        self.assertIn("ตั้ง PIN ก่อน", json.loads(body)["error"])
        self.assertEqual(self.jobs.started, [])
        self.assertEqual(self.request("/api/task/start", {"drive": False, "speed": 0.08, "minutes": 1})[0], 200)
        self.assertEqual(self.request("/api/check/start", {"check": "floor"})[0], 200)
        self.assertEqual(self.jobs.started, ["task", "check-floor"])

    def test_wheel_test_needs_the_pin_a_clear_front_and_10_s_between(self):
        now = [100.0]
        self.console.clock = lambda: now[0]
        devices = self.console.devices
        devices.held = False  # the console holds the LiDAR
        wheel = {"check": "wheel"}
        self.assertEqual(self.request("/api/check/start", wheel)[0], 401)
        devices.scan.put([(0.30, 0.10), (2.0, 0.0)], devices.clock())  # something 30 cm ahead
        code, body = self.request("/api/check/start", wheel, pin="2468")
        self.assertEqual(code, 409)
        self.assertIn("หน้าหุ่น", json.loads(body)["error"])
        devices.scan.put([(0.30, 0.40), (-0.30, 0.0), (1.0, 0.0)], devices.clock())  # beside and behind: clear
        self.assertEqual(self.request("/api/check/start", wheel, pin="2468")[0], 200)
        now[0] += 5.0
        code, body = self.request("/api/check/start", wheel, pin="2468")
        self.assertEqual(code, 429)
        self.assertIn("10 วินาที", json.loads(body)["error"])
        now[0] += 6.0
        devices.scan.put([(1.0, 0.0)], devices.clock())
        self.assertEqual(self.request("/api/check/start", wheel, pin="2468")[0], 200)
        self.assertEqual(self.jobs.started, ["check-wheel", "check-wheel"])

    def test_wheel_test_refused_without_a_fresh_clear_scan(self):
        devices, wheel = self.console.devices, {"check": "wheel"}
        devices.scan.put([(1.0, 0.0)], devices.clock())
        devices.held = True  # a job, --no-devices, or the LiDAR not back yet: the console cannot look
        code, body = self.request("/api/check/start", wheel, pin="2468")
        self.assertEqual(code, 409)
        self.assertIn("LiDAR", json.loads(body)["error"])
        devices.held = False
        devices.scan.put([(1.0, 0.0)], devices.clock() - 0.8)  # older than 0.5 s
        self.assertEqual(self.request("/api/check/start", wheel, pin="2468")[0], 409)
        devices.scan = console.camera_view.Scan()  # never a scan at all
        self.assertEqual(self.request("/api/check/start", wheel, pin="2468")[0], 409)
        self.assertEqual(self.jobs.started, [])
        devices.scan.put([(1.0, 0.0)], devices.clock())
        self.assertEqual(self.request("/api/check/start", wheel, pin="2468")[0], 200)

    def test_wrong_pins_lock_the_pin_out_but_never_stop(self):
        now = [1000.0]
        self.console.clock = lambda: now[0]
        dry = {"drive": False, "speed": 0.08, "minutes": 1}
        for _ in range(console.PIN_TRIES):
            self.assertEqual(self.request("/api/task/start", dry, pin="0000")[0], 401)
        code, body = self.request("/api/task/start", dry, pin="2468")  # even the right one, for now
        self.assertEqual(code, 429)
        self.assertIn("PIN", json.loads(body)["error"])
        self.assertEqual(self.request("/api/stop", {})[0], 200)  # STOP is never locked out
        self.assertEqual(self.jobs.started, [])
        now[0] += console.PIN_LOCK_S + 1
        self.assertEqual(self.request("/api/task/start", dry, pin="2468")[0], 200)
        for _ in range(console.PIN_TRIES):
            self.request("/api/task/start", dry, pin="1111")
        now[0] += console.PIN_LOCK_S + 1  # the second lockout lasts twice as long
        self.assertEqual(self.request("/api/task/start", dry, pin="2468")[0], 429)
        now[0] += console.PIN_LOCK_S
        self.assertEqual(self.request("/api/task/start", dry, pin="2468")[0], 200)

    def test_data_queries_are_limited_and_tighter_while_driving(self):
        self.console.data_running = 2
        self.assertEqual(self.request("/api/data/runs")[0], 429)
        self.console.data_running = 1
        self.assertEqual(self.request("/api/data/runs")[0], 200)
        self.jobs.running = {"kind": "task", "label": "Drive task", "info": {"drive": True}}
        self.assertEqual(self.request("/api/data/runs")[0], 429)  # a drive is running: one at a time
        self.console.data_running = 0
        self.assertEqual(self.request("/api/data/runs")[0], 200)
        self.assertEqual(self.console.data_running, 0)  # every query gives its slot back

    def test_commands_need_a_running_task(self):
        code, body = self.request("/api/run/command", {"command": "follow"}, pin="2468")
        self.assertEqual(code, 409)
        self.assertEqual(self.request("/api/run/command", {"command": "self-destruct"}, pin="2468")[0], 400)

    def test_log_download_only_for_run_folders(self):
        write_run(Path(self.tmp.name), "20261115-090000", [{"time": 1.0}])
        code, body = self.request("/logs/20261115-090000/log.jsonl")
        self.assertEqual((code, json.loads(body)), (200, {"time": 1.0}))
        self.assertEqual(self.request("/logs/..%2F..%2Fparams.json/log.jsonl")[0], 404)
        self.assertEqual(self.request("/logs/console/log.jsonl")[0], 404)

    def test_stream_refused_while_devices_held(self):
        self.assertEqual(self.request("/stream")[0], 503)

    def test_own_devices_serve_the_live_view_when_no_run_is_up(self):
        self.console.devices.scan.put([(1.0, 0.5)], self.console.devices.clock())
        code, body = self.request("/scan")
        self.assertEqual((code, json.loads(body)["points"]), (200, [[1.0, 0.5]]))
        self.jobs.running = {"kind": "check-wheel", "label": "Wheel test"}  # a check: nothing to relay
        self.assertEqual(self.request("/stream")[0], 503)
        self.assertEqual(json.loads(self.request("/scan")[1])["points"], [[1.0, 0.5]])


class FakeRun:
    """run_follow's dashboard on this computer: /scan and one MJPEG image on /stream."""

    SCAN = {"points": [[1.0, 0.0]], "older": [], "nearest": 1.0, "age": 0.05, "operator": [1.5, 0.2],
            "people": [[2.0, -1.0]], "stop": "obstacle", "phase": "follow"}
    PART = b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: 4\r\n\r\nJPEG\r\n"

    def __init__(self):
        from http.server import BaseHTTPRequestHandler
        asked = self.asked = []

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *args):
                pass

            def do_GET(self):
                asked.append(self.path)
                if self.path == "/scan":
                    body = json.dumps(FakeRun.SCAN).encode()
                    self.send_response(200)
                    self.send_header("Content-Type", "application/json")
                    self.send_header("Content-Length", str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                elif self.path.split("?")[0] == "/status":
                    body = json.dumps({"record": None, "run": {"drive": False, "seconds": 60}}).encode()
                    self.send_response(200)
                    self.send_header("Content-Type", "application/json")
                    self.send_header("Content-Length", str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                elif self.path == "/stream":
                    self.send_response(200)
                    self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                    self.end_headers()
                    self.wfile.write(FakeRun.PART)
                else:
                    self.send_error(404)

            def do_POST(self):
                length = int(self.headers.get("Content-Length", "0"))
                command = json.loads(self.rfile.read(length))["command"]
                asked.append("POST %s %s" % (self.path, command))
                body = json.dumps({"error": None, "command": command}).encode()
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
        self.server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
        threading.Thread(target=self.server.serve_forever, daemon=True).start()
        self.url = "http://127.0.0.1:%d" % self.server.server_address[1]

    def close(self):
        self.server.shutdown()
        self.server.server_close()


class LiveRelayTest(unittest.TestCase):
    """While run_follow holds the camera and the LiDAR, the console's /stream and /scan come from its page."""

    tearDown, request = HttpTest.tearDown, HttpTest.request

    def setUp(self):
        HttpTest.setUp(self)
        self.run = FakeRun()
        self.addCleanup(self.run.close)
        self.jobs.running = {"kind": "task", "label": "Dry-run task"}

    def test_the_run_feeds_the_live_view(self):
        self.console.run_url = self.run.url
        code, body = self.request("/scan")
        self.assertEqual((code, json.loads(body)), (200, FakeRun.SCAN))
        code, body = self.request("/stream?t=5")
        self.assertEqual((code, body), (200, FakeRun.PART))
        self.assertEqual(self.run.asked, ["/scan", "/stream"])

    def test_only_a_read_the_browser_asked_for_is_the_runs_heartbeat(self):
        # run_follow counts GET /status?heartbeat=1 as the page being there (it then leads the task): the console
        # must not keep that alive by itself once the phone has gone
        self.console.run_url = self.run.url
        code, body = self.request("/api/run/command", {"command": "follow"}, pin="2468")
        token = json.loads(body)["session"]
        self.run.asked.clear()
        self.assertEqual(self.console.run_status()[0], 200)  # the console's own read (a thread, a timer)
        self.assertEqual(self.request("/api/run", headers={"X-HMC-Session": token})[0], 200)  # not the page in view
        code, body = self.request("/api/run?heartbeat=1", headers={"X-HMC-Session": token})  # the page, in view
        self.assertEqual((code, json.loads(body)["run"]["seconds"]), (200, 60))
        self.assertEqual(self.run.asked, ["/status", "/status", "/status?heartbeat=1"])

    def test_only_the_tab_that_commands_keeps_the_heartbeat(self):
        # a console tab left open on a desk laptop must not keep the run's page alive for the operator's phone
        self.console.run_url = self.run.url
        self.request("/api/run?heartbeat=1")  # no command yet: nobody leads
        code, body = self.request("/api/run/command", {"command": "follow"}, pin="2468",
                                  headers={"X-HMC-Session": "a" * 32})  # the phone
        self.assertEqual((code, json.loads(body)["session"]), (200, "a" * 32))
        self.request("/api/run?heartbeat=1", headers={"X-HMC-Session": "b" * 32})  # the laptop's tab
        self.request("/api/run?heartbeat=1")  # a tab with no token
        self.request("/api/run?heartbeat=1", headers={"X-HMC-Session": "a" * 32})  # the phone
        self.request("/api/run?heartbeat=1&session=" + "a" * 32)  # the same, as a query parameter
        self.request("/api/run/command", {"command": "wait"}, pin="2468", headers={"X-HMC-Session": "b" * 32})
        self.request("/api/run?heartbeat=1", headers={"X-HMC-Session": "a" * 32})  # the laptop commands now
        self.assertEqual(self.run.asked, ["/status", "POST /command follow", "/status?heartbeat=1", "/status", "/status",
                                          "/status?heartbeat=1", "/status?heartbeat=1", "POST /command wait",
                                          "/status?heartbeat=1", "/status"])

    def test_a_command_is_followed_at_once_by_the_commanders_heartbeat(self):
        # with no Jetson, the run holds still unless the page is alive: the first Follow must not latch "silent"
        # for want of a heartbeat in the very next cycle
        self.console.run_url = self.run.url
        self.request("/api/run/command", {"command": "follow"}, pin="2468")
        self.assertEqual(self.run.asked, ["POST /command follow", "/status?heartbeat=1"])

    def test_a_run_whose_page_is_not_up_yet_answers_503(self):
        # run_url is port 9: nothing answers there (the run still starting)
        self.assertEqual(self.request("/stream")[0], 503)
        code, body = self.request("/scan")
        self.assertEqual(code, 503)
        self.assertIn("does not answer", json.loads(body)["error"])
        self.assertEqual(self.run.asked, [])


class DataTest(unittest.TestCase):
    """The Data tab's read-only API on telemetry's database, and the power bank marks (no PIN)."""

    tearDown, request = HttpTest.tearDown, HttpTest.request

    def setUp(self):
        HttpTest.setUp(self)
        self.db_path = Path(self.tmp.name) / "data" / "robot.db"
        self.console.db_path = self.db_path

    def fill(self):
        db = console.telemetry.open_db(self.db_path)
        rows = [(1000.0 + i, "pi", "pi_temp_c", float(i)) for i in range(100)]
        rows += [(1000.0 + i, "jetson", "jetson_vdd_in_mw", 5000.0) for i in range(0, 100, 10)]
        rows += [(1050.5, "pi", "pi_temp_c", None), (2000.0, "pi", "pi_temp_c", 99.0)]  # a null, and outside
        db.executemany("INSERT INTO samples VALUES (?, ?, ?, ?)", rows)
        for n, (run_id, started, distance) in enumerate((("20261114-100000", 500.0, 3.0),
                                                         ("20261115-090000", 900.0, 4.5))):
            db.execute("INSERT INTO runs (run_id, started, seconds, end_reason, distance_m) VALUES (?, ?, 60, ?, ?)",
                       (run_id, started, "done" if n else "signal", distance))
        db.executemany("INSERT INTO run_samples (run_id, t, v_cmd, v_meas) VALUES (?, ?, ?, ?)",
                       [("20261115-090000", i * 0.2, 0.1, 0.09) for i in range(1000)])
        db.commit()
        return db

    def get(self, path):
        code, body = self.request(path)
        return code, json.loads(body)

    def test_series_are_bucket_averages_within_the_window(self):
        self.fill().close()
        code, r = self.get("/api/data/series?metrics=pi_temp_c,jetson_vdd_in_mw,nothing_here&since=1000&until=1100"
                           "&points=10")
        self.assertEqual(code, 200)
        temp = r["series"]["pi_temp_c"]
        self.assertEqual(len(temp), 10)
        self.assertEqual(temp[0], [1004.5, 4.5])  # the mean time and value of 1000..1009; the null is skipped
        self.assertEqual(temp[-1], [1094.5, 94.5])
        self.assertEqual(len(r["series"]["jetson_vdd_in_mw"]), 10)
        self.assertEqual(r["series"]["nothing_here"], [])
        self.assertTrue(r["db"])

    def test_series_limits(self):
        self.fill().close()
        self.assertEqual(self.get("/api/data/series?metrics=pi_temp_c&since=1000&until=1100&points=99999")[1]["points"],
                         console.POINTS_MAX)
        code, r = self.get("/api/data/series?metrics=pi_temp_c&since=1000&until=1100&points=1")
        self.assertEqual((code, r["points"], len(r["series"]["pi_temp_c"])), (200, 2, 2))
        too_many = ",".join("m%d" % i for i in range(console.METRICS_MAX + 1))
        for query in ("metrics=" + too_many, "metrics=", "metrics=pi_temp_c;DROP", "metrics=pi_temp_c&since=x",
                      "metrics=pi_temp_c&since=10&until=10", "metrics=pi_temp_c&since=nan"):
            self.assertEqual(self.request("/api/data/series?" + query)[0], 400, query)

    def test_no_database_yet_gives_empty_answers(self):
        code, r = self.get("/api/data/series?metrics=pi_temp_c,bank_est_pct")
        self.assertEqual((code, r["series"], r["db"]), (200, {"pi_temp_c": [], "bank_est_pct": []}, False))
        self.assertEqual(self.get("/api/data/runs")[1]["runs"], [])
        code, r = self.get("/api/bank")
        self.assertEqual((code, r["percent"], r["full_ts"]), (200, None, None))
        self.assertEqual(r["motor"], {"since": None, "wh_used": None, "drive_minutes": None})
        code, body = self.request("/api/data/export.csv?metrics=pi_temp_c")
        self.assertEqual((code, body.splitlines()), (200, [b"ts,time,pi_temp_c"]))
        self.assertEqual(self.request("/api/data/run/20261115-090000")[0], 404)
        self.assertFalse(self.db_path.exists())  # reading never makes the file

    def test_runs_newest_first_and_one_run_downsampled(self):
        self.fill().close()
        code, r = self.get("/api/data/runs?limit=1")
        self.assertEqual(code, 200)
        self.assertEqual([x["run_id"] for x in r["runs"]], ["20261115-090000"])
        self.assertEqual(r["total"], {"runs": 2, "distance_m": 7.5})
        code, r = self.get("/api/data/run/20261115-090000?points=100")
        self.assertEqual((code, r["rows"], r["step"], len(r["samples"]["t"])), (200, 1000, 10, 100))
        self.assertEqual((r["samples"]["v_cmd"][0], r["samples"]["v_meas"][0]), (0.1, 0.09))
        self.assertEqual(r["run"]["end_reason"], "done")
        self.assertEqual(self.request("/api/data/run/20261115-999999")[0], 404)
        self.assertEqual(self.request("/api/data/run/..%2Frobot.db")[0], 404)

    def test_export_csv(self):
        self.fill().close()
        code, body = self.request("/api/data/export.csv?metrics=pi_temp_c,jetson_vdd_in_mw&since=1000&until=1003")
        self.assertEqual(code, 200)
        lines = body.decode().splitlines()
        self.assertEqual(lines[0], "ts,time,pi_temp_c,jetson_vdd_in_mw")
        self.assertEqual(len(lines), 4)  # 1000, 1001, 1002; until is exclusive
        self.assertEqual(lines[1].split(",")[2:], ["0.0", "5000.0"])
        self.assertEqual(lines[2].split(",")[2:], ["1.0", ""])

    def test_reads_while_the_recorder_holds_the_database(self):
        db = self.fill()  # open, in WAL mode, as the recorder keeps it
        try:
            db.execute("INSERT INTO samples VALUES (1099.5, 'pi', 'bank_est_pct', 40.0)")
            db.commit()
            r = self.get("/api/data/series?metrics=bank_est_pct&since=1000&until=1100")[1]
            self.assertEqual(r["series"]["bank_est_pct"], [[1099.5, 40.0]])
        finally:
            db.close()

    def test_bank_marks_need_no_pin_and_are_checked(self):
        for bad in (150, -1, "50", True, None):
            self.assertEqual(self.request("/api/bank/reading", {"percent": bad})[0], 400, bad)
        code, body = self.request("/api/bank/reading", {"percent": 62})  # no PIN given, and the console has one
        r = json.loads(body)
        self.assertEqual((code, r["bank"]["mark_kind"], r["bank"]["mark_percent"]), (200, "reading", 62.0))
        code, body = self.request("/api/bank/full", {})
        r = json.loads(body)
        self.assertEqual((code, r["bank"]["mark_percent"]), (200, 100.0))
        self.assertIsNotNone(r["bank"]["full_ts"])
        self.assertEqual(self.get("/api/bank")[1]["percent"], 100.0)
        # the PIN still guards starts and commands, and STOP still never needs it
        self.assertEqual(self.request("/api/task/start", {"drive": False, "speed": 0.08, "minutes": 1})[0], 401)
        self.assertEqual(self.request("/api/stop", {})[0], 200)
        self.assertEqual(self.jobs.stopped, 1)

    def test_motor_battery_full_mark_is_checked_and_kept_apart(self):
        for bad in ("jetson", "", None, 1, True, ["motor"], {"x": 1}):
            self.assertEqual(self.request("/api/bank/full", {"battery": bad})[0], 400, bad)
        self.assertFalse(self.db_path.exists())  # a refused mark writes nothing
        code, body = self.request("/api/bank/full", {"battery": "motor"})  # no PIN, as for the power bank
        r = json.loads(body)
        self.assertEqual(code, 200)
        self.assertIsNotNone(r["bank"]["motor"]["since"])
        self.assertEqual(r["bank"]["motor"]["wh_used"], 0.0)
        self.assertIsNone(r["bank"]["percent"])  # the power bank has no mark yet
        self.assertIsNone(r["bank"]["full_ts"])
        code, body = self.request("/api/bank/full", {"battery": "bank"})
        r = json.loads(body)
        self.assertEqual((code, r["bank"]["percent"]), (200, 100.0))
        self.assertIsNotNone(r["bank"]["full_ts"])

    def test_without_telemetry_the_data_tab_says_so(self):
        saved, console.telemetry = console.telemetry, None
        try:
            code, r = self.get("/api/data/series?metrics=pi_temp_c")
            self.assertEqual((code, r["telemetry"]), (503, False))
            self.assertEqual(self.request("/api/bank/full", {})[0], 503)
        finally:
            console.telemetry = saved


class GuardTest(unittest.TestCase):
    def setUp(self):
        self.saved = {s: signal.getsignal(s) for s in (signal.SIGINT, signal.SIGTERM)}

    def tearDown(self):
        for s, handler in self.saved.items():
            signal.signal(s, handler)

    def test_passes_the_exit_code_on(self):
        code = job_guard.guard([sys.executable, "-c", "raise SystemExit(3)"], parent=1, getppid=lambda: 1,
                               sleep=lambda s: time.sleep(0.01), log=lambda *a, **k: None)
        self.assertEqual(code, 3)

    def test_stops_the_job_when_the_console_is_gone(self):
        parents = iter([10, 10, 10] + [1] * 1000)
        stopped = []

        def stop(child, sleep):
            stopped.append(child.pid)
            child.terminate()
        code = job_guard.guard(SLEEP, parent=10, getppid=lambda: next(parents), sleep=lambda s: time.sleep(0.01),
                               log=lambda *a, **k: None, stop=stop)
        self.assertEqual(len(stopped), 1)
        self.assertNotEqual(code, 0)


class MissionModesTest(unittest.TestCase):
    def test_only_the_full_mission_gets_the_way_back_after_the_clock(self):
        # the executive goes home for RETURN_BUDGET after 5:00: run_follow must still run then
        flow = sys.modules[console._load_mission().__module__]
        self.assertEqual(console.RETURN_S, flow.RETURN_BUDGET)
        p = {"max_speed": 0.12}
        for mission, seconds in (("full", "300"), ("bag_only", "120"), ("follow_only", "120")):
            cmd, err = console.task_command(False, 0.08, 2, p, mission=mission)
            self.assertIsNone(err)
            self.assertEqual(cmd[cmd.index("--seconds") + 1], seconds, mission)

    def test_follow_only_omits_remote_while_full_and_bag_only_keep_it(self):
        p = {"max_speed": 0.12}
        cmd_full, err = console.task_command(False, 0.08, 5, p, mission="full")
        self.assertIsNone(err)
        self.assertIn("--remote", cmd_full)

        cmd_bag, err = console.task_command(False, 0.08, 5, p, mission="bag_only")
        self.assertIsNone(err)
        self.assertIn("--remote", cmd_bag)

        cmd_follow, err = console.task_command(False, 0.08, 5, p, mission="follow_only")
        self.assertIsNone(err)
        self.assertNotIn("--remote", cmd_follow)

        cmd_bad, err = console.task_command(False, 0.08, 5, p, mission="invalid")
        self.assertIsNone(cmd_bad)
        self.assertIn("mission", err)


class RobotPageTest(unittest.TestCase):
    """The Robot page's state, an idle run, the measured bag, and a finished log on the Data tab."""

    setUp, tearDown, request = HttpTest.setUp, HttpTest.tearDown, HttpTest.request

    def test_idle_run_is_200_and_does_not_ask_the_run(self):
        asked = []

        def fetch(url, body=None, timeout=None):
            asked.append(url)
            raise OSError("the run must not be asked while idle")

        self.console.fetch = fetch
        code, body = self.request("/api/run")
        self.assertEqual(code, 200)
        data = json.loads(body)
        self.assertEqual(data.get("state"), "idle")
        self.assertNotIn("error", data)
        code, body = self.request("/api/state")
        state = json.loads(body)
        self.assertEqual(state["run"], {"state": "idle"})
        self.assertEqual(asked, [])
        self.assertIsInstance(state["events"], list)
        drop = state["drop"]
        self.assertIn("y", drop)
        if drop.get("x") is not None:
            self.assertGreaterEqual(drop["x"], drop["x_min"])
            self.assertLessEqual(drop["x"], drop["x_max"])
            self.assertNotEqual(drop["x"], 0.31)

    def test_the_drop_default_is_found_where_the_pi_keeps_so100_ik(self):
        # deploy.sh puts so100_ik.py in ~/hmc-base/jetson, beside console.py; in the repo it is ../jetson
        here = console.HERE
        with tempfile.TemporaryDirectory() as tmp:
            (Path(tmp) / "jetson").mkdir()
            (Path(tmp) / "jetson" / "so100_ik.py").write_text(
                "def reachable_drop():\n    return {'x': 0.3, 'y': 0.0}\n", encoding="utf-8")
            self.console._drop = None
            console.HERE = Path(tmp)
            try:
                self.assertEqual(self.console.drop_default(), {"x": 0.3, "y": 0.0})
            finally:
                console.HERE = here

    def test_the_page_serves_the_robot_tab_and_one_local_library(self):
        code, page = self.request("/")
        self.assertEqual(code, 200)
        self.assertIn(b'id="tab-robot"', page)
        self.assertIn(b"/preact-htm.js", page)
        self.assertNotIn(b"unpkg.com", page)
        self.assertNotIn(b"jsdelivr", page)
        code, lib = self.request("/preact-htm.js")
        self.assertEqual(code, 200)
        self.assertIn(b"preact", lib)
        self.assertIn(b"htm", lib)
        code, app = self.request("/app.js")
        self.assertEqual(code, 200)
        self.assertNotIn(b"x: 0.36", app)
        self.assertNotIn(b"x: 0.31", app)

    def test_stop_is_kept_on_the_event_list(self):
        self.assertEqual(self.request("/api/stop", {})[0], 200)
        events = json.loads(self.request("/api/state")[1])["events"]
        self.assertTrue(any(e["text"] == "/api/stop ok" and e["level"] == "info" for e in events))

    def test_a_front_camera_handle_is_the_bag_not_a_fixed_x(self):
        self.console.devices.held = False
        self.console.devices.scan.put([(0.44, 0.02), (0.45, 0.03)], self.console.devices.clock())
        self.console._ask_front_camera = lambda points: {"ok": True, "handle": [0.44, 0.02], "top": 0.25, "yaw": 0.1}
        self.console.probe_bag()
        self.assertEqual(self.console.bag["source"], "front-camera")
        self.assertEqual(self.console.bag["x"], 0.44)
        self.assertNotEqual(self.console.bag["x"], 0.36)
        bag = json.loads(self.request("/api/state")[1])["bag"]
        self.assertEqual((bag["source"], bag["x"], bag["top"]), ("front-camera", 0.44, 0.25))

    def test_lidar_is_used_only_when_the_camera_has_no_handle(self):
        self.console.devices.held = False
        self.console.devices.scan.put([(0.50, 0.0)], self.console.devices.clock())
        self.console._ask_front_camera = lambda points: {"ok": False, "why": "no bag"}
        self.console.probe_bag()
        self.assertEqual(self.console.bag["source"], "lidar")
        self.assertAlmostEqual(self.console.bag["x"], 0.5, places=2)
        self.assertNotEqual(self.console.bag["x"], 0.36)
        self.console._set_bag(0.44, 0.02, 0.25, None, "front-camera", time.time())
        self.console.probe_bag()
        self.assertEqual(self.console.bag["source"], "front-camera")
        self.assertEqual(self.console.bag["x"], 0.44)

    def test_usb_snap_is_one_jpeg(self):
        self.assertEqual(self.request("/usb-snap")[0], 503)
        self.console.jetson.webcam_snap = lambda: (200, b"\xff\xd8\xff", "image/jpeg")
        code, body = self.request("/usb-snap")
        self.assertEqual((code, body[:2]), (200, b"\xff\xd8"))

    def test_data_tab_imports_a_finished_run_after_14_nov(self):
        self.console.db_path = Path(self.tmp.name) / "robot.db"
        name = "20261119-132600"
        folder = Path(self.tmp.name) / name
        folder.mkdir()
        lines = [
            json.dumps({"time": 1.0, "pose": [0.0, 0.0, 0.0], "v": 0.0, "w": 0.0}),
            json.dumps({"time": 2.0, "pose": [0.1, 0.0, 0.0], "v": 0.1, "w": 0.0}),
            json.dumps({"time": 3.0, "end": "done: stopped"}),
        ]
        (folder / "log.jsonl").write_text("\n".join(lines) + "\n")
        code, body = self.request("/api/data/runs")
        self.assertEqual(code, 200)
        self.assertIn(name, [r["run_id"] for r in json.loads(body)["runs"]])


if __name__ == "__main__":
    unittest.main()
