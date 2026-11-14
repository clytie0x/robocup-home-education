"""telemetry.py with fake readers and temporary folders: no network, no vcgencmd, no real home folder."""
import csv
import json
import math
import os
import socket
import sqlite3
import sys
import tempfile
import time
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "tools"))
import runs_to_db  # noqa: E402
import telemetry  # noqa: E402

DAY = 86400.0
FULL_STATUS = {
    "thermal": {"cpu-thermal": 45.2, "gpu-thermal": 44.0, "soc0-thermal": 43.1, "tj-thermal": 46.0},
    "power": {"VDD_IN": {"mV": 4984, "mA": 1208, "mW": 6020},
              "VDD_CPU_GPU_CV": {"mV": 4976, "mA": 300, "mW": 1493}},
    "gpu_load_pct": 37.5, "cpu_pct": 22.0, "load1": 1.25, "mem_used_mb": 3100, "mem_total_mb": 7620,
    "disk_free_gb": 12.5, "uptime_s": 3600,
    "programs": {"detector": {"running": True, "pid": 12}, "flow": {"running": False}},
}


class FakePi:
    def __init__(self, values=None, error=None):
        self.values, self.error = dict(values or {}), error

    def read(self):
        if self.error:
            raise self.error
        return dict(self.values)


class Net:
    """Fake probe and fetch: which hosts answer, and what /status says."""

    def __init__(self, up=(), status=None):
        self.up, self.status = set(up), status

    def probe(self, host):
        return 1.5 if host in self.up else None

    def fetch(self, host):
        return self.status if host in self.up else None


class Temp(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.dir = Path(self.tmp.name)
        self.db_path = self.dir / "data" / "robot.db"
        self.opened = []  # closed before the folder goes: Windows cannot delete an open database

    def tearDown(self):
        for db in self.opened:
            db.close()
        self.tmp.cleanup()

    def db(self):
        db = telemetry.open_db(self.db_path)
        self.opened.append(db)
        return db

    def recorder(self, net=None, pi=None, **kw):
        net = net or Net()
        kw.setdefault("sources", [])
        rec = telemetry.Recorder(self.db_path, ("10.0.0.2", "192.168.31.137"), pi=pi or FakePi(),
                                 fetch=net.fetch, probe=net.probe, clock=lambda: 0.0, **kw)
        self.opened.insert(0, rec)
        return rec

    def events(self, kind=None):
        rows = self.db().execute("SELECT ts, kind, detail FROM events ORDER BY ts, rowid").fetchall()
        return [r for r in rows if kind is None or r[1] == kind]


class SchemaTest(Temp):
    def test_new_db_has_the_tables_wal_and_version(self):
        db = self.db()
        tables = {r[0] for r in db.execute("SELECT name FROM sqlite_master WHERE type='table'")}
        for name in ("samples", "events", "runs", "run_samples", "schema_version", "charge_marks", "bank_fits"):
            self.assertIn(name, tables)
        self.assertEqual(db.execute("SELECT version FROM schema_version").fetchone()[0], telemetry.SCHEMA_VERSION)
        self.assertEqual(db.execute("PRAGMA journal_mode").fetchone()[0], "wal")
        indexes = {r[0] for r in db.execute("SELECT name FROM sqlite_master WHERE type='index'")}
        self.assertIn("samples_metric_ts", indexes)

    def test_reopen_keeps_data_and_one_version_row(self):
        db = self.db()
        db.execute("INSERT INTO samples VALUES (1, 'pi', 'pi_temp_c', 40)")
        db.commit()
        db2 = self.db()
        self.assertEqual(db2.execute("SELECT count(*) FROM samples").fetchone()[0], 1)
        self.assertEqual(db2.execute("SELECT count(*) FROM schema_version").fetchone()[0], 1)

    def test_migration_runs_only_the_newer_steps(self):
        steps = ["CREATE TABLE a (x)", "CREATE TABLE b (y); INSERT INTO a VALUES (1)"]
        db = sqlite3.connect(str(self.dir / "m.db"))
        self.opened.append(db)
        self.assertEqual(telemetry.migrate(db, steps[:1]), 1)
        self.assertEqual(telemetry.migrate(db, steps), 2)
        self.assertEqual(telemetry.migrate(db, steps), 2)
        self.assertEqual(db.execute("SELECT count(*) FROM a").fetchone()[0], 1)  # step 2 ran once

    def test_migration_v1_to_v2_keeps_old_marks_as_bank(self):
        db = sqlite3.connect(str(self.dir / "v1.db"))
        self.opened.append(db)
        self.assertEqual(telemetry.migrate(db, telemetry.MIGRATIONS[:1]), 1)
        db.execute("INSERT INTO charge_marks VALUES (10.0, 'full', 100.0, NULL)")  # a v1 row: no battery column
        db.execute("INSERT INTO charge_marks VALUES (20.0, 'reading', 80.0, 'display')")
        db.commit()
        self.assertEqual(telemetry.migrate(db), 2)
        self.assertEqual(db.execute("SELECT ts, kind, percent, battery FROM charge_marks ORDER BY ts").fetchall(),
                         [(10.0, "full", 100.0, "bank"), (20.0, "reading", 80.0, "bank")])
        telemetry.mark_full(db, ts=30.0, battery="motor")
        self.assertEqual((telemetry.estimate(db, now=40.0)["mark_ts"], telemetry.motor_usage(db, now=40.0)["since"]),
                         (20.0, 30.0))


class PiReaderTest(Temp):
    def test_nothing_readable_gives_nulls(self):
        def broken_disk(path):
            raise OSError("no disk")
        reader = telemetry.PiReader(root=self.dir / "empty", vcgencmd=lambda *a: None, disk=broken_disk)
        values = reader.read()
        self.assertEqual(set(values), set(telemetry.PI_METRICS))
        self.assertTrue(all(v is None for v in values.values()), values)

    def test_missing_vcgencmd_binary_is_none(self):
        self.assertIsNone(telemetry.vcgencmd("measure_temp", binary="no-such-vcgencmd-here"))

    def test_reads_fake_files_and_vcgencmd(self):
        root = self.dir / "root"
        files = {
            "sys/class/thermal/thermal_zone0/temp": "38400\n",
            "proc/loadavg": "0.52 0.40 0.30 1/200 999\n",
            "proc/meminfo": "MemTotal:  3884000 kB\nMemFree: 100 kB\nMemAvailable:  2860000 kB\n",
            "proc/stat": "cpu  100 0 100 800 0 0 0 0 0 0\ncpu0 1 2 3 4\n",
            "proc/uptime": "1234.5 4000.0\n",
            "proc/net/wireless": ("Inter-| sta-|   Quality        |   Discarded packets               | Missed | WE\n"
                                  " face | tus | link level noise |  nwid  crypt   frag  retry   misc | beacon | 22\n"
                                  " wlan0: 0000   58.  -52.  -256        0      0      0      0     0        0\n"),
        }
        for name, text in files.items():
            (root / name).parent.mkdir(parents=True, exist_ok=True)
            (root / name).write_text(text)
        answers = {"measure_temp": "temp=38.4'C\n", "get_throttled": "throttled=0x50005\n",
                   "measure_clock": "frequency(48)=1500398464\n"}
        reader = telemetry.PiReader(root=root, vcgencmd=lambda *a: answers.get(a[0]),
                                    disk=lambda path: (100 * 2**30, 70 * 2**30, 30 * 2**30))
        v = reader.read()
        self.assertEqual(v["pi_temp_c"], 38.4)
        self.assertEqual(v["pi_cpu_thermal_c"], 38.4)
        self.assertEqual(v["pi_throttled"], 0x50005)
        self.assertEqual((v["pi_undervolt"], v["pi_undervolt_boot"]), (1, 1))
        self.assertAlmostEqual(v["pi_arm_mhz"], 1500.4, places=1)
        self.assertEqual(v["pi_load1"], 0.52)
        self.assertAlmostEqual(v["pi_mem_used_mb"], (3884000 - 2860000) / 1024, places=3)
        self.assertEqual(v["pi_disk_free_gb"], 30.0)
        self.assertEqual((v["pi_wifi_link"], v["pi_wifi_level_dbm"]), (58.0, -52.0))
        self.assertEqual(v["pi_uptime_s"], 1234.5)
        self.assertIsNone(v["pi_cpu_pct"])  # needs two reads
        (root / "proc/stat").write_text("cpu  150 0 150 900 0 0 0 0 0 0\n")
        self.assertAlmostEqual(reader.read()["pi_cpu_pct"], 50.0)


class JetsonParseTest(unittest.TestCase):
    def test_full_status(self):
        m = telemetry.parse_jetson(FULL_STATUS)
        self.assertEqual(m["jetson_temp_cpu_c"], 45.2)
        self.assertEqual(m["jetson_temp_tj_c"], 46.0)
        self.assertEqual(m["jetson_vdd_in_mv"], 4984)
        self.assertEqual(m["jetson_vdd_in_ma"], 1208)
        self.assertEqual(m["jetson_vdd_in_mw"], 6020)
        self.assertEqual(m["jetson_vdd_cpu_gpu_cv_mw"], 1493)
        self.assertEqual(m["jetson_gpu_load_pct"], 37.5)
        self.assertEqual(m["jetson_mem_used_mb"], 3100)
        self.assertEqual(m["jetson_prog_detector_running"], 1)
        self.assertEqual(m["jetson_prog_flow_running"], 0)

    def test_missing_and_odd_fields_are_left_out(self):
        odd = {"thermal": [1, 2], "power": {"VDD_IN": {"mV": "4984", "mA": None, "mW": float("nan")},
                                            "VDD_SOC": 7, 5: {"mW": 1}},
               "gpu_load_pct": "high", "cpu_pct": True, "load1": [1], "programs": {"flow": "yes", "x": {}},
               "uptime_s": float("inf")}
        self.assertEqual(telemetry.parse_jetson(odd), {})
        for junk in (None, [], "text", 3, {"power": None, "thermal": None, "programs": None}):
            self.assertEqual(telemetry.parse_jetson(junk), {})

    def test_power_without_mw_is_computed_from_mv_and_ma(self):
        m = telemetry.parse_jetson({"power": {"VDD_IN": {"mV": 5000, "mA": 1000}}})
        self.assertEqual(m["jetson_vdd_in_mw"], 5000.0)

    def test_old_agent_system_block(self):
        m = telemetry.parse_jetson({"system": {"temp_c": 50.5, "ram_used_mb": 2000, "ram_total_mb": 7620,
                                               "load": [0.5, 0.4, 0.3], "uptime_s": 99}})
        self.assertEqual((m["jetson_temp_max_c"], m["jetson_mem_used_mb"], m["jetson_load1"], m["jetson_uptime_s"]),
                         (50.5, 2000, 0.5, 99))


class ProbeTest(unittest.TestCase):
    def test_refused_counts_as_a_path_and_timeout_as_none(self):
        def refused(address, timeout):
            raise ConnectionRefusedError()
        def silent(address, timeout):
            raise socket.timeout()
        calls = []
        def unreachable_then_open(address, timeout):
            calls.append(address)
            if address[1] == 8090:
                raise OSError("no route")
            return FakeSocket()
        self.assertIsNotNone(telemetry.probe("h", connect=refused))
        self.assertIsNone(telemetry.probe("h", connect=silent))
        self.assertIsNotNone(telemetry.probe("h", connect=unreachable_then_open))
        self.assertEqual(calls, [("h", 8090), ("h", 5601)])

    def test_fetch_failure_is_none(self):
        def opener(url, timeout):
            raise OSError("refused")
        self.assertIsNone(telemetry.fetch_status("h", opener=opener))

    def test_fetch_reads_json(self):
        class Reply:
            def __enter__(self):
                return self
            def __exit__(self, *a):
                return False
            def read(self, n):
                return b'{"cpu_pct": 5}'
        seen = []
        def opener(url, timeout):
            seen.append(url)
            return Reply()
        self.assertEqual(telemetry.fetch_status("10.0.0.2", opener=opener), {"cpu_pct": 5})
        self.assertEqual(seen, ["http://10.0.0.2:8090/status"])


class FakeSocket:
    def close(self):
        pass


class RecorderTest(Temp):
    def test_everything_missing_records_nulls_and_no_crash(self):
        rec = self.recorder()
        data = rec.tick(0.0)
        self.assertIsNone(data["link"]["link_cable_ms"])
        self.assertIsNone(data["link"]["link_wifi_ms"])
        self.assertEqual(data["jetson"], {"jetson_reachable": 0})
        rec.close()
        rows = self.db().execute("SELECT host, metric, value FROM samples").fetchall()
        self.assertIn(("jetson", "jetson_reachable", 0.0), rows)
        self.assertNotIn(None, [r[2] for r in rows])  # nulls are not stored
        kinds = [e[1] for e in self.events()]
        self.assertIn("jetson", kinds)
        self.assertIn("link", kinds)

    def test_jetson_values_are_stored(self):
        rec = self.recorder(Net(up={"10.0.0.2"}, status=FULL_STATUS))
        data = rec.tick(0.0)
        self.assertEqual(data["jetson"]["jetson_reachable"], 1)
        self.assertEqual(data["jetson"]["jetson_path"], 0)
        rec.close()
        row = self.db().execute("SELECT host, value FROM samples WHERE metric='jetson_vdd_in_mw'").fetchone()
        self.assertEqual(row, ("jetson", 6020.0))

    def test_reader_errors_become_events_at_most_once_a_minute(self):
        rec = self.recorder(pi=FakePi(error=RuntimeError("boom")))
        for now in (0.0, 5.0, 30.0, 59.0, 61.0):
            rec.tick(now)
        rec.close()
        errors = self.events("error")
        self.assertEqual([e[0] for e in errors], [0.0, 61.0])
        self.assertIn("RuntimeError: boom", errors[0][2])

    def test_undervolt_event_from_a_throttled_change(self):
        pi = FakePi({"pi_throttled": 0x0})
        rec = self.recorder(pi=pi)
        rec.tick(0.0)
        pi.values["pi_throttled"] = 0x50005
        rec.tick(5.0)
        pi.values["pi_throttled"] = 0x50000  # cleared now, still "since boot"
        rec.tick(10.0)
        rec.tick(15.0)  # no change: no event
        rec.close()
        changes = [e for e in self.events() if e[1] in ("undervolt", "throttled")]
        self.assertEqual([(e[0], e[1]) for e in changes], [(5.0, "undervolt"), (10.0, "throttled")])
        self.assertIn("0x0 -> 0x50005", changes[0][2])
        self.assertIn("under-voltage now", changes[0][2])

    def test_link_path_change_events(self):
        net = Net(up={"10.0.0.2", "192.168.31.137"})
        rec = self.recorder(net)
        rec.tick(0.0)
        net.up = {"192.168.31.137"}  # cable unplugged
        rec.tick(5.0)
        rec.tick(10.0)
        net.up = set()
        rec.tick(15.0)
        net.up = {"10.0.0.2"}
        rec.tick(20.0)
        rec.close()
        self.assertEqual([(e[0], e[2]) for e in self.events("link")],
                         [(0.0, "start: cable"), (5.0, "cable -> wifi"), (15.0, "wifi -> none"),
                          (20.0, "none -> cable")])

    def test_jetson_unreachable_and_back(self):
        net = Net(up={"10.0.0.2"}, status={"cpu_pct": 3})
        rec = self.recorder(net)
        rec.tick(0.0)
        net.up = set()
        rec.tick(5.0)
        net.up = {"192.168.31.137"}
        rec.tick(10.0)
        rec.close()
        self.assertEqual([(e[0], e[2]) for e in self.events("jetson")],
                         [(0.0, "reachable via cable"), (5.0, "unreachable"), (10.0, "reachable again via wifi")])

    def test_batched_commit_every_10_s(self):
        rec = self.recorder()
        reader = self.db()
        count = lambda: reader.execute("SELECT count(*) FROM samples WHERE metric='jetson_reachable'").fetchone()[0]
        rec.tick(0.0)
        rec.tick(5.0)
        self.assertEqual(count(), 0)
        rec.tick(10.0)
        self.assertEqual(count(), 3)  # jetson_reachable at 0, 5 and 10 s
        rec.tick(15.0)
        rec.close()  # flushes the rest
        self.assertEqual(count(), 4)

    def test_busy_samples_every_second(self):
        rec = self.recorder(period_s=5.0)
        self.assertEqual(rec.period(), 5.0)
        rec.busy = True
        self.assertEqual(rec.period(), 1.0)

    def test_busy_from_a_run_log_being_written(self):
        logs = self.dir / "logs"
        (logs / "20261115-200000").mkdir(parents=True)
        log = logs / "20261115-200000" / "log.jsonl"
        log.write_text("{}\n")
        os.utime(log, (1000.0, 1000.0))
        self.assertTrue(telemetry.run_active(logs, now=1002.0))
        self.assertFalse(telemetry.run_active(logs, now=1100.0))
        self.assertFalse(telemetry.run_active(self.dir / "nothing", now=1000.0))

    def test_retention_once_a_day(self):
        db = self.db()
        now = 100 * DAY
        db.execute("INSERT INTO samples VALUES (?, 'pi', 'old', 1)", (now - 61 * DAY,))
        db.execute("INSERT INTO samples VALUES (?, 'pi', 'new', 1)", (now - 59 * DAY,))
        db.commit()
        rec = self.recorder(keep_days=60)
        rec.tick(now)
        metrics = lambda: [r[0] for r in db.execute("SELECT DISTINCT metric FROM samples WHERE metric IN ('old', 'new') ORDER BY metric")]
        self.assertEqual(metrics(), ["new"])
        db.execute("INSERT INTO samples VALUES (?, 'pi', 'old', 1)", (now - 61 * DAY,))
        db.commit()
        rec.tick(now + 3600)
        self.assertIn("old", metrics())  # pruned at most once a day
        rec.tick(now + DAY)
        rec.close()
        self.assertNotIn("old", metrics())

    def test_thread_start_stop(self):
        net = Net()
        rec = telemetry.Recorder(self.db_path, ("10.0.0.2",), period_s=0.02, commit_s=0.05, pi=FakePi(),
                                 fetch=net.fetch, probe=net.probe, sources=[])
        rec.start()
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline and rec.ticks < 3:
            time.sleep(0.01)
        rec.stop()
        self.assertFalse(rec.thread.is_alive())
        self.assertGreaterEqual(self.db().execute("SELECT count(*) FROM samples").fetchone()[0], 3)
        details = [e[2] for e in self.events("recorder")]
        self.assertEqual(details[-1], "stopped")
        self.assertTrue(details[0].startswith("started"))

    def test_motor_source_placeholder_and_a_plugged_in_source(self):
        self.assertEqual(telemetry.motor_bus(), {"motor_bus_mv": None, "motor_bus_ma": None})
        rec = self.recorder(sources=[("motor", telemetry.motor_bus), ("motor", lambda: {"motor_bus_mv": 12100})])
        data = rec.tick(0.0)
        self.assertEqual(data["motor"]["motor_bus_mv"], 12100)
        rec.close()
        self.assertEqual(self.db().execute("SELECT host, value FROM samples WHERE metric='motor_bus_mv'").fetchall(),
                         [("motor", 12100.0)])

    def test_once_is_json_with_nulls(self):
        rec = self.recorder()
        data = json.loads(telemetry.once(rec))
        self.assertIn("ts", data)
        self.assertIsNone(data["link"]["link_cable_ms"])


def cycle(t, x, v=0.5, w=0.0, **extra):
    record = {"time": t, "pose": [x, 0.0, 0.0], "v": v, "w": w, "measured": None, "phase": "follow",
              "mode": "follow", "locked": True, "link": True, "operator": [1.0, 0.0], "nearest": 0.8,
              "stop": None, "why": None, "enrol": None, "cycle_time": 0.03, "wheels": None}
    record.update(extra)
    return record


def write_run(logs, name, records, tail="", mtime=1000.0):
    folder = logs / name
    folder.mkdir(parents=True)
    lines = [json.dumps(r) for r in records]
    (folder / "log.jsonl").write_text("\n".join(lines) + "\n" + tail, encoding="utf-8")
    os.utime(folder / "log.jsonl", (mtime, mtime))
    return folder


class ImportTest(Temp):
    def setUp(self):
        super().setUp()
        self.logs = self.dir / "logs"
        records = [{"time": 99.0, "say": "hello"}]
        for i in range(31):  # 3 s at 10 Hz, 0.5 m/s along x
            records.append(cycle(100.0 + i * 0.1, 0.05 * i, locked=i % 2 == 0,
                                 stop="no operator" if i in (3, 4) else None,
                                 measured=[0.49, 0.0, True, 0.9, 0.5] if i > 0 else None))
            if i == 10:
                records.append({"time": 101.0, "remote": "client connected"})
                records.append({"time": 101.0, "wheels": "not turning", "episode": 1, "firmware": None})
        records.append({"time": 103.1, "stop": "wheels", "end": "wheels: not turning, 3 times: is the motor power on?"})
        write_run(self.logs, "20261115-201500", records)

    def run_row(self, run_id):
        db = self.db()
        db.row_factory = sqlite3.Row
        return db.execute("SELECT * FROM runs WHERE run_id=?", (run_id,)).fetchone()

    def test_summary_numbers(self):
        self.assertEqual(telemetry.import_runs(self.db(), self.logs, now=5000.0), ["20261115-201500"])
        r = self.run_row("20261115-201500")
        self.assertEqual(r["started"], time.mktime(time.strptime("20261115-201500", "%Y%m%d-%H%M%S")))
        self.assertAlmostEqual(r["seconds"], 3.1)
        self.assertAlmostEqual(r["ended"], r["started"] + 3.1)
        self.assertEqual(r["end_reason"], "wheels")
        self.assertIn("not turning", r["end_detail"])
        self.assertEqual(r["drive"], 1)  # measured motion seen
        self.assertEqual(r["cycles"], 31)
        self.assertAlmostEqual(r["distance_m"], 1.5)
        self.assertEqual(r["max_v"], 0.5)
        self.assertEqual(r["mean_v_moving"], 0.5)
        self.assertAlmostEqual(r["locked_pct"], 100.0 * 16 / 31)
        self.assertEqual(json.loads(r["stop_counts"]), {"no operator": 2})

    def test_run_samples_at_most_5_hz(self):
        db = self.db()
        telemetry.import_runs(db, self.logs, now=5000.0)
        rows = db.execute("SELECT t, x, v_cmd, v_meas, locked, operator_m, phase, link, cycle_ms FROM run_samples"
                          " WHERE run_id='20261115-201500' ORDER BY t").fetchall()
        self.assertEqual(len(rows), 16)  # 0, 0.2, ... 3.0 s
        self.assertTrue(all(b[0] - a[0] >= 0.2 - 1e-6 for a, b in zip(rows, rows[1:])))
        first, second = rows[0], rows[1]
        self.assertEqual(first[:3], (0.0, 0.0, 0.5))
        self.assertIsNone(first[3])
        self.assertEqual(second[3], 0.49)
        self.assertEqual((first[4], first[5], first[6], first[7]), (1, 1.0, "follow", 1))
        self.assertAlmostEqual(first[8], 30.0)

    def test_reimport_is_idempotent(self):
        db = self.db()
        telemetry.import_runs(db, self.logs, now=5000.0)
        self.assertEqual(telemetry.import_runs(db, self.logs, now=5000.0), [])
        self.assertEqual(db.execute("SELECT count(*) FROM runs").fetchone()[0], 1)
        self.assertEqual(db.execute("SELECT count(*) FROM run_samples").fetchone()[0], 16)

    def test_truncated_last_line_and_old_log_without_phase(self):
        old = [cycle(10.0 + i * 0.2, 0.0, v=0.0, w=0.3) for i in range(5)]
        for r in old:
            for name in ("phase", "measured", "wheels", "why", "enrol"):
                del r[name]
        write_run(self.logs, "20261101-090000", old, tail='{"time": 11.0, "pose": [0.0, 0.')
        db = self.db()
        self.assertIn("20261101-090000", telemetry.import_runs(db, self.logs, now=5000.0))
        db.row_factory = sqlite3.Row
        r = db.execute("SELECT * FROM runs WHERE run_id='20261101-090000'").fetchone()
        self.assertIsNone(r["end_reason"])  # no end record: cut short
        self.assertIsNone(r["drive"])  # nothing says whether the wheels were on
        self.assertEqual(r["cycles"], 5)
        self.assertEqual(r["distance_m"], 0.0)
        self.assertIsNone(r["mean_v_moving"])
        phases = db.execute("SELECT phase FROM run_samples WHERE run_id='20261101-090000'").fetchall()
        self.assertEqual([tuple(r) for r in phases], [(None,)] * 5)

    def test_run_still_being_written_waits(self):
        write_run(self.logs, "20261115-220000", [cycle(1.0, 0.0)], mtime=4990.0)
        db = self.db()
        self.assertEqual(telemetry.import_runs(db, self.logs, now=5000.0), ["20261115-201500"])
        self.assertEqual(telemetry.import_runs(db, self.logs, now=5100.0), ["20261115-220000"])

    def test_measured_as_a_dict_and_folders_that_are_not_runs(self):
        write_run(self.logs, "20261115-230000", [cycle(1.0, 0.0, measured={"v": 0.1, "w": -0.2}),
                                                 {"time": 1.1, "end": "done: back at the start", "stop": None}])
        (self.logs / "notes").mkdir()
        (self.logs / "20261115-235959").mkdir()  # no log.jsonl
        write_run(self.logs, "20261116-000000", [], tail="garbage")  # no cycle at all
        db = self.db()
        self.assertEqual(sorted(telemetry.import_runs(db, self.logs, now=5000.0)), ["20261115-201500", "20261115-230000"])
        self.assertEqual(db.execute("SELECT v_meas, w_meas FROM run_samples WHERE run_id='20261115-230000'").fetchone(),
                         (0.1, -0.2))
        self.assertEqual(db.execute("SELECT end_reason FROM runs WHERE run_id='20261115-230000'").fetchone()[0], "done")
        self.assertEqual(telemetry.import_runs(db, self.dir / "no-logs-here", now=5000.0), [])

    def test_runs_to_db_cli(self):
        self.assertEqual(runs_to_db.main(["--logs", str(self.logs), "--db", str(self.db_path)]), 0)
        self.assertEqual(self.db().execute("SELECT run_id FROM runs").fetchall(), [("20261115-201500",)])


class ExportTest(Temp):
    def setUp(self):
        super().setUp()
        db = self.db()
        db.executemany("INSERT INTO samples VALUES (?, ?, ?, ?)", [
            (100.0, "pi", "pi_temp_c", 40.0), (100.0, "jetson", "jetson_vdd_in_mw", 6000.0),
            (105.0, "pi", "pi_temp_c", 41.0), (110.0, "pi", "pi_temp_c", 42.0), (105.0, "pi", "pi_load1", 9.0)])
        db.execute("INSERT INTO events VALUES (101.0, 'pi', 'undervolt', '0x0 -> 0x50005')")
        db.commit()
        self.out = self.dir / "out.csv"

    def read(self):
        with open(self.out, newline="", encoding="utf-8") as f:
            return list(csv.reader(f))

    def test_metrics_wide_one_row_per_time(self):
        n = telemetry.export_csv(self.db(), ["pi_temp_c", "jetson_vdd_in_mw"], None, 108.0, self.out)
        rows = self.read()
        self.assertEqual(n, 2)
        self.assertEqual(rows[0], ["ts", "time", "pi_temp_c", "jetson_vdd_in_mw"])
        self.assertEqual([rows[1][0], rows[1][2], rows[1][3]], ["100.0", "40.0", "6000.0"])
        self.assertEqual([rows[2][0], rows[2][2], rows[2][3]], ["105.0", "41.0", ""])
        self.assertEqual(rows[1][1], time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(100.0)))

    def test_a_table_with_since(self):
        self.assertEqual(telemetry.export_csv(self.db(), "events", 100.5, None, self.out), 1)
        rows = self.read()
        self.assertEqual(rows[0], ["ts", "time", "host", "kind", "detail"])
        self.assertEqual(rows[1][3:], ["undervolt", "0x0 -> 0x50005"])

    def test_cli_export(self):
        self.assertEqual(telemetry.main(["--db", str(self.db_path), "--export", "pi_temp_c,pi_load1",
                                         "--since", "104", "--out", str(self.out)]), 0)
        self.assertEqual(len(self.read()), 3)

    def test_when(self):
        self.assertEqual(telemetry.when("2h", now=10000.0), 10000.0 - 7200)
        self.assertEqual(telemetry.when("3d", now=DAY * 10), DAY * 7)
        self.assertEqual(telemetry.when("1234.5"), 1234.5)
        self.assertEqual(telemetry.when("2026-11-15 20:00"),
                         time.mktime(time.strptime("2026-11-15 20:00", "%Y-%m-%d %H:%M")))


class BankTest(Temp):
    def watts(self, db, start, end, step, pi_w, motor_w=0.0):
        t = start
        while t <= end + 1e-9:
            db.execute("INSERT INTO samples VALUES (?, 'bank', 'pi_est_w', ?)", (t, pi_w))
            db.execute("INSERT INTO samples VALUES (?, 'bank', 'motor_est_w', ?)", (t, motor_w))
            t += step
        db.commit()

    def drop(self, watts, hours, scale=1.0):
        return 100.0 * scale * watts * hours / telemetry.BANK_EFFICIENCY / telemetry.BANK_WH

    def test_full_then_known_watts(self):
        db = self.db()
        telemetry.mark_full(db, ts=0.0)
        self.watts(db, 0.0, 3600.0, 60.0, 8.0, 2.0)
        e = telemetry.estimate(db, now=3600.0)
        self.assertAlmostEqual(e["wh_used"], 8.0 / telemetry.BANK_EFFICIENCY)  # the motors' 2 W are not the bank's
        self.assertAlmostEqual(e["percent"], 100.0 - self.drop(8.0, 1.0))
        self.assertAlmostEqual(e["low"], 100.0 - 1.1 * self.drop(8.0, 1.0))
        self.assertAlmostEqual(e["high"], 100.0 - 0.9 * self.drop(8.0, 1.0))
        self.assertAlmostEqual(e["hours_on"], 1.0)
        self.assertAlmostEqual(e["driving_min"], 60.0)
        self.assertEqual((e["mark_kind"], e["mark_percent"]), ("full", 100.0))
        self.assertIsNone(e["warning"])

    def test_bank_estimate_ignores_motor_watts(self):
        db = self.db()
        telemetry.mark_full(db, ts=0.0)
        self.watts(db, 0.0, 3600.0, 60.0, 5.0, 12.0)
        with_motors = telemetry.estimate(db, now=3600.0)
        db2 = sqlite3.connect(":memory:")
        telemetry.migrate(db2)
        telemetry.mark_full(db2, ts=0.0)
        self.watts(db2, 0.0, 3600.0, 60.0, 5.0, 0.0)
        without = telemetry.estimate(db2, now=3600.0)
        db2.close()
        self.assertAlmostEqual(with_motors["wh_used"], 5.0 / telemetry.BANK_EFFICIENCY)
        self.assertAlmostEqual(with_motors["percent"], without["percent"])
        self.assertAlmostEqual(with_motors["driving_min"], 60.0)  # still reported, but costs the bank nothing

    def test_motor_usage_sums_only_since_the_last_motor_mark(self):
        db = self.db()
        self.assertEqual(telemetry.motor_usage(db, now=0.0), {"since": None, "wh_used": None, "drive_minutes": None})
        for t in range(0, 7201, 60):  # 12 W at the motors until 5400 s, then standing still
            db.execute("INSERT INTO samples VALUES (?, 'bank', 'pi_est_w', 5.0)", (float(t),))
            db.execute("INSERT INTO samples VALUES (?, 'bank', 'motor_est_w', ?)", (float(t), 12.0 if t < 5400 else 0.0))
        db.commit()
        telemetry.mark_full(db, ts=0.0, battery="motor")
        telemetry.mark_full(db, ts=3600.0, battery="motor")  # charged again: the first hour no longer counts
        telemetry.mark_full(db, ts=4000.0)  # a power bank mark changes nothing here
        telemetry.add_reading(db, 90, ts=4500.0)
        m = telemetry.motor_usage(db, now=7200.0)
        self.assertEqual(m["since"], 3600.0)
        self.assertAlmostEqual(m["wh_used"], 12.0 * 0.5)
        self.assertAlmostEqual(m["drive_minutes"], 30.0)
        self.assertEqual(telemetry.estimate(db, now=7200.0)["mark_ts"], 4500.0)  # motor marks are not bank marks
        self.assertEqual(telemetry.motor_usage(db, now=1800.0)["since"], 0.0)
        with self.assertRaises(ValueError):
            telemetry.mark_full(db, battery="jetson")

    def test_gaps_longer_than_a_minute_count_as_off(self):
        db = self.db()
        telemetry.mark_full(db, ts=0.0)
        self.watts(db, 0.0, 0.0, 1.0, 10.0)
        self.watts(db, 3600.0, 3600.0, 1.0, 10.0)
        e = telemetry.estimate(db, now=3600.0)
        self.assertAlmostEqual(e["hours_on"], telemetry.MAX_GAP_S / 3600.0)

    def test_a_reading_resets_the_baseline(self):
        db = self.db()
        telemetry.mark_full(db, ts=0.0)
        self.watts(db, 0.0, 7200.0, 60.0, 10.0)
        telemetry.add_reading(db, 50, ts=3600.0)
        e = telemetry.estimate(db, now=7200.0)
        self.assertEqual((e["mark_kind"], e["mark_percent"], e["mark_ts"]), ("reading", 50.0, 3600.0))
        self.assertAlmostEqual(e["percent"], 50.0 - self.drop(10.0, 1.0))

    def test_clamped_at_zero_with_a_warning(self):
        db = self.db()
        telemetry.mark_full(db, ts=0.0)
        self.watts(db, 0.0, 36000.0, 60.0, 50.0)
        e = telemetry.estimate(db, now=36000.0)
        self.assertEqual((e["percent"], e["low"]), (0.0, 0.0))
        self.assertIn("Pi จะดับ", e["warning"])  # the bank feeds only the Pi now: when it runs out the Pi goes off
        self.assertNotIn("15-20", e["warning"])

    def test_warning_below_30(self):
        db = self.db()
        telemetry.add_reading(db, 31, ts=0.0)
        self.assertIsNone(telemetry.estimate(db, now=0.0)["warning"])
        telemetry.add_reading(db, 29, ts=1.0)
        self.assertIn("ต่ำกว่า 30% แล้ว ชาร์จ power bank ก่อน", telemetry.estimate(db, now=1.0)["warning"])

    def test_no_mark_yet(self):
        e = telemetry.estimate(self.db(), now=0.0)
        self.assertIsNone(e["percent"])
        self.assertIn("--charged", e["warning"])

    def test_readings_outside_0_100_are_refused(self):
        with self.assertRaises(ValueError):
            telemetry.add_reading(self.db(), 120)

    def two_pairs(self, db, scale):
        telemetry.mark_full(db, ts=0.0)
        self.watts(db, 0.0, 7200.0, 60.0, 10.0)
        telemetry.add_reading(db, 100.0 - self.drop(10.0, 1.0, scale), ts=3600.0)
        telemetry.add_reading(db, 100.0 - self.drop(10.0, 2.0, scale), ts=7200.0)

    def test_fit_recovers_the_scale_and_estimate_uses_it(self):
        db = self.db()
        self.two_pairs(db, 1.5)
        fit = telemetry.fit_coefficients(db, now=7200.0)
        self.assertEqual((fit["pairs"], fit["applied"]), (2, True))
        self.assertAlmostEqual(fit["scale"], 1.5)
        self.watts(db, 7260.0, 10800.0, 60.0, 10.0)
        e = telemetry.estimate(db, now=10800.0)
        self.assertAlmostEqual(e["scale"], 1.5)
        self.assertAlmostEqual(e["percent"], 100.0 - self.drop(10.0, 3.0, 1.5))

    def test_fit_bounds_and_too_few_pairs(self):
        db = self.db()
        self.two_pairs(db, 2.6)
        self.assertEqual(telemetry.fit_coefficients(db, now=7200.0)["scale"], 2.0)
        db2 = sqlite3.connect(":memory:")
        telemetry.migrate(db2)
        self.watts(db2, 0.0, 3600.0, 60.0, 10.0)
        telemetry.mark_full(db2, ts=0.0)
        telemetry.add_reading(db2, 80, ts=3600.0)
        fit = telemetry.fit_coefficients(db2, now=3600.0)
        self.assertEqual((fit["pairs"], fit["applied"]), (1, False))
        self.assertEqual(telemetry.estimate(db2, now=3600.0)["scale"], 1.0)
        db2.close()

    def test_pair_across_a_charge_is_not_used(self):
        db = self.db()
        telemetry.add_reading(db, 40, ts=0.0)
        self.watts(db, 0.0, 3600.0, 60.0, 10.0)
        telemetry.add_reading(db, 90, ts=3600.0)  # charged in between
        self.assertEqual(telemetry.fit_coefficients(db, now=3600.0)["pairs"], 0)

    def test_model_watts(self):
        self.assertEqual(telemetry.motor_watts(0.0, 0.0), 0.0)
        self.assertEqual(telemetry.motor_watts(10.0, 0.0), telemetry.MOTOR_W_AT_FULL)
        self.assertEqual(telemetry.motor_watts(0.1, 0.0, wheels="not turning"), 0.0)
        self.assertGreater(telemetry.motor_watts(0.1, 0.0), 0.0)
        self.assertEqual(telemetry.pi_watts(None), telemetry.pi_watts(telemetry.DEFAULT_CPU_PCT))
        self.assertAlmostEqual(telemetry.pi_watts(100.0),
                               telemetry.PI_BASE_W + telemetry.PI_PER_CPU_W + telemetry.USB_DEVICES_W)

    def test_latest_command_from_a_run_log(self):
        logs = self.dir / "logs"
        write_run(logs, "20261115-200000", [cycle(1.0, 0.0, v=0.1, w=0.2), {"time": 1.1, "remote": "x"}],
                  tail='{"time": 1.2, "pose"')
        self.assertEqual(telemetry.latest_command(logs), (0.1, 0.2, None))
        self.assertIsNone(telemetry.latest_command(self.dir / "none"))

    def test_recorder_stores_the_estimate(self):
        db = self.db()
        telemetry.mark_full(db, ts=0.0)
        rec = self.recorder(pi=FakePi({"pi_cpu_pct": 50.0}))
        for now in range(0, 61, 5):
            data = rec.tick(float(now))
        rec.close()
        w = telemetry.pi_watts(50.0)
        self.assertAlmostEqual(data["bank"]["pi_est_w"], w)
        self.assertEqual(data["bank"]["motor_est_w"], 0.0)
        self.assertAlmostEqual(data["bank"]["bank_est_wh_used"], w * 60 / 3600 / telemetry.BANK_EFFICIENCY)
        self.assertAlmostEqual(data["bank"]["bank_est_pct"], telemetry.estimate(db, now=60.0)["percent"])
        stored = db.execute("SELECT value FROM samples WHERE metric='bank_est_pct' AND ts=60").fetchone()[0]
        self.assertAlmostEqual(stored, data["bank"]["bank_est_pct"])

    def test_recorder_counts_motor_wh_apart_from_the_bank(self):
        db = self.db()
        telemetry.mark_full(db, ts=0.0)
        telemetry.mark_full(db, ts=0.0, battery="motor")
        rec = self.recorder(pi=FakePi({"pi_cpu_pct": 50.0}))
        read_all = rec.read_all
        def driving():
            data = read_all()
            data["bank"]["motor_est_w"] = 12.0
            return data
        rec.read_all = driving
        for now in range(0, 61, 5):
            data = rec.tick(float(now))
        rec.close()
        w = telemetry.pi_watts(50.0)
        self.assertAlmostEqual(data["bank"]["bank_est_wh_used"], w * 60 / 3600 / telemetry.BANK_EFFICIENCY)
        self.assertAlmostEqual(data["bank"]["motor_est_wh_used"], 12.0 * 60 / 3600)
        self.assertAlmostEqual(data["bank"]["motor_est_wh_used"], telemetry.motor_usage(db, now=60.0)["wh_used"])
        stored = db.execute("SELECT value FROM samples WHERE metric='motor_est_wh_used' AND ts=60").fetchone()[0]
        self.assertAlmostEqual(stored, 0.2)

    def test_recorder_picks_up_a_new_mark(self):
        db = self.db()
        rec = self.recorder(pi=FakePi({"pi_cpu_pct": 0.0}))
        self.assertNotIn("bank_est_pct", rec.tick(0.0)["bank"])  # no mark yet
        telemetry.add_reading(db, 70, ts=2.0)
        data = rec.tick(5.0)
        self.assertAlmostEqual(data["bank"]["bank_est_pct"], telemetry.estimate(db, now=5.0)["percent"])
        self.assertLess(data["bank"]["bank_est_pct"], 70.0)

    def test_cli_marks(self):
        args = ["--db", str(self.db_path)]
        self.assertEqual(telemetry.main(args + ["--charged"]), 0)
        self.assertEqual(telemetry.main(args + ["--reading", "62"]), 0)
        self.assertEqual(telemetry.main(args + ["--charged", "motor"]), 0)
        self.assertEqual(telemetry.main(args + ["--estimate"]), 0)
        rows = self.db().execute("SELECT kind, percent, battery FROM charge_marks ORDER BY ts, rowid").fetchall()
        self.assertEqual(rows, [("full", 100.0, "bank"), ("reading", 62.0, "bank"), ("full", 100.0, "motor")])
        with self.assertRaises(SystemExit):
            telemetry.main(args + ["--charged", "jetson"])


if __name__ == "__main__":
    unittest.main()
