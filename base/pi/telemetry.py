"""The robot's telemetry in one SQLite file: the Pi, the Jetson and the link between them over time, the
run logs, an estimate of the power bank's charge, and the energy drawn from the motor battery. See
docs/telemetry.md for what is recorded and how to query it.

  python telemetry.py --once                 one sample as JSON, nothing written (a check on the robot)
  python telemetry.py --run                  record until Ctrl+C (every 5 s, every 1 s while a run is logging)
  python telemetry.py --export pi_temp_c,jetson_vdd_in_mw --since 2d --out temps.csv
  python telemetry.py --export runs --out runs.csv     (a table: samples, events, runs, run_samples, charge_marks)
  python telemetry.py --charged              the power bank was just charged to 100 %
  python telemetry.py --charged motor        the 21V motor battery was just charged (its energy counts from now)
  python telemetry.py --reading 62           the bank's display shows 62 %
  python telemetry.py --estimate             the bank's estimated charge now, and the motor battery's use, as JSON
  python telemetry.py --fit                  fit the energy model's scale to the readings so far
  (all take --db FILE; the default is data/robot.db next to this file: ~/hmc-base/data/robot.db on the Pi)

Plain Python standard library. Nothing read can stop the recorder: a reader that fails gives nulls, and its
error goes to the events table at most once a minute. Samples are written in one transaction every
COMMIT_S (SD card wear), so a crash loses at most that much.
"""
import argparse
import csv
import json
import math
import re
import shutil
import socket
import sqlite3
import subprocess
import sys
import threading
import time
import urllib.request
from datetime import datetime
from pathlib import Path

import ports

HERE = Path(__file__).resolve().parent
DB_PATH = HERE / "data" / "robot.db"  # on the Pi: ~/hmc-base/data/robot.db
LOGS = HERE / "logs"  # run_follow.py's run logs (its LOG_ROOT)
JETSON_HOSTS = ("10.0.0.2", "192.168.31.137")  # in the order tried: the LAN cable, then Wi-Fi
HOST_NAMES = {"10.0.0.2": "cable", "192.168.31.137": "wifi", "100.108.202.48": "tailscale"}
AGENT_PORT = ports.AGENT  # jetson/agent.py: GET /status
LINK_PORTS = (ports.AGENT, ports.DETECTOR)  # the agent, then the person detector
PROBE_TIMEOUT_S = 0.3
FETCH_TIMEOUT_S = 1.0
MAX_STATUS = 1_000_000  # bytes; a bigger /status reply is broken
MAX_KEYS = 32  # zones, rails or programs taken from one reply: a broken reply must not add thousands of metrics

PERIOD_S = 5.0
BUSY_PERIOD_S = 1.0  # while a run is active
COMMIT_S = 10.0
KEEP_DAYS = 60  # samples older than this are deleted, once a day
PRUNE_EVERY_S = 86400.0
IMPORT_EVERY_S = 600.0  # --run: finished run logs are imported this often, when no run is active
ERROR_EVERY_S = 60.0  # at most one error event per this many seconds
ACTIVE_S = 5.0  # a run log written within this many seconds means a run is active
RUN_DONE_S = 60.0  # a log without an end record, untouched this long, is a finished (cut short) run
RUN_HZ = 5.0  # run_samples keeps at most this many cycles per second
MAX_PENDING = 200_000  # rows kept in memory while the database cannot be written; the oldest go first

# The UGREEN power bank (145 W, 25,000 mAh) feeds only the Pi and its USB devices (Xtion, LiDAR, the base
# board's logic); the wheel motors run off a separate "21V MAX LITHIUM" power-tool battery through a DC-DC
# converter, and the Jetson is on mains (owner and a measurement, 2026-11-15). Neither battery has a data link,
# so the bank's charge is estimated from a model of the Pi-side power. Every coefficient below is a guess
# to be fitted (fit_coefficients scales the whole model from the owner's display readings).
BANK_WH = 25.0 * 3.7  # 92.5 Wh: 25,000 mAh at the 3.7 V cell voltage, computed from the label, not measured
BANK_EFFICIENCY = 0.85  # guess: the bank's output conversion losses
PI_BASE_W = 3.0  # guess: the Pi idle
PI_PER_CPU_W = 3.0  # guess: added at 100 % CPU
USB_DEVICES_W = 2.5  # guess: Xtion ~2 W + LiDAR ~0.5 W, assumed always on
MOTOR_W_AT_FULL = 12.0  # guess: both motors at 100 % PWM (L298N), drawn from the 21V tool battery, not the bank
MOTOR_FULL_V = 0.5  # guess: m/s the base would drive at 100 % PWM (the logs carry the command, not the PWM)
MOTOR_FULL_W = 3.0  # guess: rad/s of turning at 100 % PWM
DEFAULT_CPU_PCT = 25.0  # used while the CPU load is not known yet
MAX_GAP_S = 60.0  # a sample's power counts until the next one, but never longer: a longer gap is the Pi off
BAND = 0.10  # guess: the energy used is known to +-10 %
WARN_PCT = 30.0  # 2026-11-14: the display read 15 % when the motors had already lost power
MIN_DROP_PCT = 5.0  # a pair of readings closer than this is too coarse (the display shows whole %) to fit
MIN_PAIRS = 2
SCALE_BOUNDS = (0.5, 2.0)
# The motor battery's capacity is not known, so only its energy used is counted (motor_usage), not a percent.
# When the owner gives the battery's Ah (printed on it), set this to Ah * its nominal V and the percent can be
# shown.
MOTOR_BATTERY_WH = None
BATTERIES = ("bank", "motor")  # charge_marks.battery: the UGREEN power bank, the 21V motor battery

SCHEMA_VERSION = 2
MIGRATIONS = [  # MIGRATIONS[n] takes the database from version n to n + 1; never edit a step once shipped
    """
    CREATE TABLE samples (ts REAL NOT NULL, host TEXT NOT NULL, metric TEXT NOT NULL, value REAL);
    CREATE INDEX samples_metric_ts ON samples (metric, ts);
    CREATE INDEX samples_ts ON samples (ts);
    CREATE TABLE events (ts REAL NOT NULL, host TEXT, kind TEXT NOT NULL, detail TEXT);
    CREATE INDEX events_ts ON events (ts);
    CREATE TABLE runs (run_id TEXT PRIMARY KEY, started REAL, ended REAL, seconds REAL, end_reason TEXT,
                       end_detail TEXT, drive INTEGER, cycles INTEGER, distance_m REAL, max_v REAL,
                       mean_v_moving REAL, locked_pct REAL, stop_counts TEXT);
    CREATE TABLE run_samples (run_id TEXT NOT NULL, t REAL, x REAL, y REAL, th REAL, v_cmd REAL, w_cmd REAL,
                              v_meas REAL, w_meas REAL, locked INTEGER, operator_m REAL, nearest_m REAL,
                              stop TEXT, phase TEXT, mode TEXT, link INTEGER, cycle_ms REAL, wheels TEXT);
    CREATE INDEX run_samples_run_t ON run_samples (run_id, t);
    CREATE TABLE charge_marks (ts REAL NOT NULL, kind TEXT NOT NULL, percent REAL NOT NULL, note TEXT);
    CREATE TABLE bank_fits (ts REAL NOT NULL, scale REAL NOT NULL, pairs INTEGER NOT NULL);
    """,
    """
    ALTER TABLE charge_marks ADD COLUMN battery TEXT NOT NULL DEFAULT 'bank';
    """,
]
TABLES = {"samples": "ts", "events": "ts", "runs": "started", "run_samples": None, "charge_marks": "ts",
          "bank_fits": "ts"}  # table -> its time column, for --since and --until

PI_METRICS = ("pi_temp_c", "pi_cpu_thermal_c", "pi_throttled", "pi_undervolt", "pi_undervolt_boot", "pi_arm_mhz",
              "pi_load1", "pi_cpu_pct", "pi_mem_used_mb", "pi_disk_free_gb", "pi_wifi_link", "pi_wifi_level_dbm",
              "pi_uptime_s")
THROTTLE_BITS = {0: "under-voltage now", 1: "frequency capped now", 2: "throttled now", 3: "soft temperature limit now",
                 16: "under-voltage since boot", 17: "frequency capped since boot", 18: "throttled since boot",
                 19: "soft temperature limit since boot"}
UNDERVOLT = (1 << 0) | (1 << 16)


# --- the database ---------------------------------------------------------------------------------------------

def migrate(db, migrations=MIGRATIONS):
    """Bring db up to len(migrations); returns the version. Each step runs once, in its own transaction."""
    db.execute("CREATE TABLE IF NOT EXISTS schema_version (version INTEGER NOT NULL)")
    row = db.execute("SELECT version FROM schema_version").fetchone()
    if row is None:
        db.execute("INSERT INTO schema_version VALUES (0)")
        db.commit()
        row = (0,)
    version = row[0]
    for step in range(version, len(migrations)):
        db.executescript("BEGIN;\n%s;\nUPDATE schema_version SET version = %d;\nCOMMIT;" % (migrations[step], step + 1))
        version = step + 1
    return version


def open_db(path=DB_PATH):
    """The database at path (made, with its folder, when missing), in WAL mode and up to SCHEMA_VERSION."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    db = sqlite3.connect(str(path), timeout=10.0, check_same_thread=False)
    db.execute("PRAGMA journal_mode=WAL")
    db.execute("PRAGMA synchronous=NORMAL")  # WAL: a power cut loses the last commits at worst, never the file
    migrate(db)
    return db


# --- readers: each gives None for what it cannot read, and never raises for a missing file or tool -------------

def _number(value):
    """value as a float when it is a real, finite number (not a bool, not text), else None."""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    value = float(value)
    return value if math.isfinite(value) else None


def _name(text):
    return re.sub(r"[^a-z0-9]+", "_", str(text).lower()).strip("_")


def vcgencmd(*args, binary="vcgencmd"):
    """vcgencmd's answer, or None where it is missing or fails (not a Pi, no permission)."""
    try:
        done = subprocess.run([binary, *args], capture_output=True, text=True, timeout=2.0)
    except (OSError, subprocess.SubprocessError):
        return None
    return done.stdout if done.returncode == 0 else None


def _match(pattern, text, convert=float):
    found = re.search(pattern, text or "")
    if not found:
        return None
    try:
        return convert(found.group(1))
    except ValueError:
        return None


class PiReader:
    """The Pi's own health, without sudo. root and vcgencmd are for tests."""

    def __init__(self, root=Path("/"), vcgencmd=vcgencmd, disk=shutil.disk_usage, disk_path=HERE):
        self.root, self.vcgencmd, self.disk, self.disk_path = Path(root), vcgencmd, disk, disk_path
        self.stat = None  # the previous /proc/stat (busy, total), for the CPU load between two reads

    def _text(self, name):
        try:
            return (self.root / name).read_text()
        except (OSError, UnicodeDecodeError):
            return None

    def read(self):
        out = dict.fromkeys(PI_METRICS)
        out["pi_temp_c"] = _match(r"temp=([-\d.]+)", self.vcgencmd("measure_temp"))
        bits = _match(r"throttled=(0x[0-9a-fA-F]+)", self.vcgencmd("get_throttled"), lambda s: int(s, 16))
        if bits is not None:
            out["pi_throttled"], out["pi_undervolt"], out["pi_undervolt_boot"] = bits, bits & 1, (bits >> 16) & 1
        hz = _match(r"=(\d+)", self.vcgencmd("measure_clock", "arm"))
        out["pi_arm_mhz"] = None if hz is None else hz / 1e6
        milli = _match(r"(-?\d+)", self._text("sys/class/thermal/thermal_zone0/temp"))
        out["pi_cpu_thermal_c"] = None if milli is None else milli / 1000.0
        out["pi_load1"] = _match(r"^\s*([\d.]+)", self._text("proc/loadavg"))
        out["pi_uptime_s"] = _match(r"^\s*([\d.]+)", self._text("proc/uptime"))
        meminfo = self._text("proc/meminfo") or ""
        total = _match(r"MemTotal:\s*(\d+)", meminfo)
        avail = _match(r"MemAvailable:\s*(\d+)", meminfo)
        if total is not None and avail is not None:
            out["pi_mem_used_mb"] = (total - avail) / 1024.0
        out["pi_cpu_pct"] = self._cpu()
        try:
            out["pi_disk_free_gb"] = self.disk(self.disk_path)[2] / 2**30
        except (OSError, TypeError, IndexError):
            pass
        out["pi_wifi_link"], out["pi_wifi_level_dbm"] = self._wifi()
        return out

    def _cpu(self):
        line = (self._text("proc/stat") or "").partition("\n")[0].split()
        try:
            ticks = [int(x) for x in line[1:]]
            now = (sum(ticks) - ticks[3] - (ticks[4] if len(ticks) > 4 else 0), sum(ticks))
        except (ValueError, IndexError):
            return None
        before, self.stat = self.stat, now
        if before is None or now[1] <= before[1]:
            return None
        return 100.0 * (now[0] - before[0]) / (now[1] - before[1])

    def _wifi(self):
        for line in (self._text("proc/net/wireless") or "").splitlines()[2:]:
            parts = line.split()
            if len(parts) >= 4:
                try:
                    return float(parts[2].rstrip(".")), float(parts[3].rstrip("."))
                except ValueError:
                    pass
        return None, None


def probe(host, ports=LINK_PORTS, timeout=PROBE_TIMEOUT_S, connect=socket.create_connection):
    """ms to open (and close) a TCP connection to host, or None. A refused connection still proves the path,
    so it counts; a timeout means no path, and the next port is not tried."""
    for port in ports:
        start = time.monotonic()
        try:
            connect((host, port), timeout=timeout).close()
        except ConnectionRefusedError:
            pass
        except (socket.timeout, TimeoutError):
            return None
        except OSError:
            continue
        return (time.monotonic() - start) * 1000.0
    return None


def fetch_status(host, port=AGENT_PORT, timeout=FETCH_TIMEOUT_S, opener=urllib.request.urlopen):
    """The Jetson agent's /status as a dict, or None for anything else (no answer, not JSON, not an object)."""
    try:
        with opener("http://%s:%d/status" % (host, port), timeout=timeout) as reply:
            status = json.loads(reply.read(MAX_STATUS + 1)[:MAX_STATUS])
    except Exception:  # the network, HTTP errors, bad JSON: all just mean no status
        return None
    return status if isinstance(status, dict) else None


def parse_jetson(status):
    """{metric: value} from the agent's /status. Anything of a shape not expected is left out, never an error."""
    out = {}
    if not isinstance(status, dict):
        return out

    def put(metric, value):
        value = _number(value)
        if value is not None:
            out[metric] = value

    thermal = status.get("thermal")
    if isinstance(thermal, dict):
        for zone, value in list(thermal.items())[:MAX_KEYS]:
            if isinstance(zone, str):
                put("jetson_temp_%s_c" % _name(zone.replace("-thermal", "")), value)
    power = status.get("power")
    if isinstance(power, dict):
        for rail, values in list(power.items())[:MAX_KEYS]:
            if not (isinstance(rail, str) and isinstance(values, dict)):
                continue
            rail = _name(rail)
            mv, ma, mw = (_number(values.get(k)) for k in ("mV", "mA", "mW"))
            put("jetson_%s_mv" % rail, mv)
            put("jetson_%s_ma" % rail, ma)
            put("jetson_%s_mw" % rail, mw if mw is not None else (mv * ma / 1000.0 if None not in (mv, ma) else None))
    for key in ("gpu_load_pct", "cpu_pct", "load1", "mem_used_mb", "mem_total_mb", "disk_free_gb", "uptime_s"):
        put("jetson_" + key, status.get(key))
    programs = status.get("programs")
    if isinstance(programs, dict):
        for name, program in list(programs.items())[:MAX_KEYS]:
            if isinstance(name, str) and isinstance(program, dict) and isinstance(program.get("running"), bool):
                out["jetson_prog_%s_running" % _name(name)] = int(program["running"])
    system = status.get("system")  # the agent's older summary block
    if isinstance(system, dict):
        put("jetson_temp_max_c", system.get("temp_c"))
        load = system.get("load")
        for metric, value in (("jetson_mem_used_mb", system.get("ram_used_mb")),
                              ("jetson_mem_total_mb", system.get("ram_total_mb")),
                              ("jetson_load1", load[0] if isinstance(load, list) and load else None),
                              ("jetson_uptime_s", system.get("uptime_s"))):
            if metric not in out:
                put(metric, value)
    return out


def motor_bus():
    """The wheel motors' supply: nothing can read it yet (the 21V tool battery has no data link), so null. A sensor
    plugs in here, e.g. an INA219 on the motor line over I2C: return its mV and mA (docs/telemetry.md)."""
    return {"motor_bus_mv": None, "motor_bus_ma": None}


SOURCES = (("motor", motor_bus),)  # (host, function returning {metric: value or None}): add sensors here


def run_active(logs_dir, now=None, within=ACTIVE_S):
    """True when the newest run log under logs_dir was written within the last `within` seconds."""
    log = _newest_log(logs_dir)
    try:
        return log is not None and (time.time() if now is None else now) - log.stat().st_mtime <= within
    except OSError:
        return False


def _newest_log(logs_dir):
    try:
        folders = sorted(f for f in Path(logs_dir).iterdir() if _run_start(f.name) is not None)
    except OSError:
        return None
    for folder in reversed(folders):
        if (folder / "log.jsonl").is_file():
            return folder / "log.jsonl"
    return None


def latest_command(logs_dir, tail=16384):
    """(v, w, wheels) of the newest cycle record in the newest run log, or None."""
    log = _newest_log(logs_dir)
    if log is None:
        return None
    try:
        with open(log, "rb") as f:
            f.seek(0, 2)
            f.seek(max(0, f.tell() - tail))
            lines = f.read().decode("utf-8", "replace").splitlines()
    except OSError:
        return None
    for line in reversed(lines):
        record = _record(line)
        if _is_cycle(record):
            return _number(record.get("v")) or 0.0, _number(record.get("w")) or 0.0, record.get("wheels")
    return None


# --- the power bank's estimated charge, and the motor battery's energy used ------------------------------------

def pi_watts(cpu_pct):
    """Model watts drawn by the Pi and its USB devices at this CPU load (DEFAULT_CPU_PCT when unknown)."""
    cpu = DEFAULT_CPU_PCT if cpu_pct is None else max(0.0, min(100.0, cpu_pct))
    return PI_BASE_W + PI_PER_CPU_W * cpu / 100.0 + USB_DEVICES_W


def motor_watts(v, w, wheels=None):
    """Model watts the wheel motors draw (from the 21V battery) for the command v (m/s), w (rad/s). Wheels that
    do not turn have lost their power (the 2026-11-14 symptom), so they draw nothing."""
    if wheels:
        return 0.0
    return MOTOR_W_AT_FULL * min(1.0, abs(v) / MOTOR_FULL_V + abs(w) / MOTOR_FULL_W)


def mark_full(db, ts=None, note=None, battery="bank"):
    """The owner charged a battery full: "bank" (the power bank, to 100 %) or "motor" (the 21V tool battery)."""
    if battery not in BATTERIES:
        raise ValueError("battery is one of %s" % ", ".join(BATTERIES))
    db.execute("INSERT INTO charge_marks (ts, kind, percent, note, battery) VALUES (?, 'full', 100.0, ?, ?)",
               (time.time() if ts is None else ts, note, battery))
    db.commit()


def add_reading(db, percent, ts=None, note=None):
    """The owner read percent off the bank's display."""
    percent = _number(percent)
    if percent is None or not 0.0 <= percent <= 100.0:
        raise ValueError("a reading is a percent from 0 to 100")
    db.execute("INSERT INTO charge_marks (ts, kind, percent, note, battery) VALUES (?, 'reading', ?, ?, 'bank')",
               (time.time() if ts is None else ts, percent, note))
    db.commit()


def _marks(db):
    return db.execute("SELECT ts, kind, percent FROM charge_marks WHERE battery = 'bank' ORDER BY ts, rowid").fetchall()


def _scale(db):
    row = db.execute("SELECT scale FROM bank_fits ORDER BY ts DESC, rowid DESC LIMIT 1").fetchone()
    return 1.0 if row is None else row[0]


def _usage(db, start, end, metric="pi_est_w", efficiency=BANK_EFFICIENCY):
    """(model Wh of metric's watts / efficiency, seconds on, seconds driving) in [start, end], unscaled. The
    default is the Wh drawn from the bank's cells by the Pi side (pi_est_w only: the motors are on the other
    battery). Each sample's watts hold until the next sample, at most MAX_GAP_S; driving is motor_est_w > 0."""
    before = db.execute("SELECT max(ts) FROM samples WHERE metric = ? AND ts < ?", (metric, start)).fetchone()[0]
    rows = db.execute("SELECT ts, max(CASE WHEN metric = ? THEN value END),"
                      " max(CASE WHEN metric = 'motor_est_w' THEN value END) FROM samples"
                      " WHERE metric IN (?, 'motor_est_w') AND ts >= ? AND ts <= ? GROUP BY ts ORDER BY ts",
                      (metric, metric, start if before is None else before, end)).fetchall()
    wh = on = driving = 0.0
    for (ts, watts, motor), following in zip(rows, [r[0] for r in rows[1:]] + [end]):
        seconds = max(0.0, min(following, ts + MAX_GAP_S, end) - max(ts, start))
        wh += (watts or 0.0) * seconds / 3600.0 / efficiency
        on += seconds
        if motor:
            driving += seconds
    return wh, on, driving


def _percent(mark_percent, wh):
    return max(0.0, min(100.0, mark_percent - 100.0 * wh / BANK_WH))


def estimate(db, now=None):
    """The bank's estimated charge now, from the last mark and the model energy since."""
    now = time.time() if now is None else now
    scale = _scale(db)
    marks = [m for m in _marks(db) if m[0] <= now]
    if not marks:
        return {"percent": None, "low": None, "high": None, "wh_used": None, "mark_ts": None, "mark_kind": None,
                "mark_percent": None, "hours_on": None, "driving_min": None, "scale": scale,
                "warning": "no charge mark yet: after a full charge run python telemetry.py --charged"}
    mark_ts, kind, mark_percent = marks[-1]
    wh, on, driving = _usage(db, mark_ts, now)
    wh *= scale
    percent = _percent(mark_percent, wh)
    warning = None
    if percent < WARN_PCT:
        warning = ("เหลือประมาณ %.0f%% (ค่าประมาณ) ต่ำกว่า 30%% แล้ว ชาร์จ power bank ก่อน ถ้าแบตหมด Pi จะดับ"
                   % percent)  # the bank feeds only the Pi
    return {"percent": percent, "low": _percent(mark_percent, wh * (1 + BAND)),
            "high": _percent(mark_percent, wh * (1 - BAND)), "wh_used": wh, "mark_ts": mark_ts, "mark_kind": kind,
            "mark_percent": mark_percent, "hours_on": on / 3600.0, "driving_min": driving / 60.0, "scale": scale,
            "warning": warning}


def _motor_mark(db, now):
    """ts of the last motor battery full mark at or before now, or None."""
    return db.execute("SELECT max(ts) FROM charge_marks WHERE battery = 'motor' AND kind = 'full' AND ts <= ?",
                      (now,)).fetchone()[0]


def motor_usage(db, now=None):
    """The 21V motor battery since its last full mark: {"since": that mark's ts or None, "wh_used": model Wh at
    the motors from the motor_est_w samples (the DC-DC converter's loss not included), "drive_minutes": minutes
    with motor_est_w > 0}. Energy, not percent: its capacity is unknown (MOTOR_BATTERY_WH)."""
    now = time.time() if now is None else now
    since = _motor_mark(db, now)
    if since is None:
        return {"since": None, "wh_used": None, "drive_minutes": None}
    wh, _, driving = _usage(db, since, now, "motor_est_w", 1.0)
    return {"since": since, "wh_used": wh, "drive_minutes": driving / 60.0}


def fit_coefficients(db, now=None):
    """One scale on the whole model, from consecutive marks where the display dropped (no charge between):
    least squares of scale * model drop against the displayed drop. Stored and used from then on when at
    least MIN_PAIRS pairs are usable; bounded to SCALE_BOUNDS."""
    pairs = []
    marks = _marks(db)
    for (t0, _, p0), (t1, kind1, p1) in zip(marks, marks[1:]):
        if kind1 != "reading" or p0 - p1 < MIN_DROP_PCT:
            continue  # a full mark means a charge came between; a rise or a tiny drop says nothing
        model = 100.0 * _usage(db, t0, t1)[0] / BANK_WH
        if model > 0.0:
            pairs.append((model, p0 - p1))
    if not pairs:
        return {"scale": None, "pairs": 0, "applied": False}
    scale = sum(m * d for m, d in pairs) / sum(m * m for m, _ in pairs)
    scale = max(SCALE_BOUNDS[0], min(SCALE_BOUNDS[1], scale))
    applied = len(pairs) >= MIN_PAIRS
    if applied:
        db.execute("INSERT INTO bank_fits VALUES (?, ?, ?)", (time.time() if now is None else now, scale, len(pairs)))
        db.commit()
    return {"scale": scale, "pairs": len(pairs), "applied": applied}


# --- the recorder -----------------------------------------------------------------------------------------------

class Recorder:
    """Samples the Pi, the Jetson, the link and SOURCES every period_s (busy_period_s while busy), into db_path.

    busy: set it True while a run is active; with logs_dir, a run log being written also counts. start() runs
    it on a daemon thread, stop() ends it; run() does the same in the foreground. tick(now) is one sample."""

    def __init__(self, db_path=DB_PATH, jetson_hosts=JETSON_HOSTS, period_s=PERIOD_S, busy_period_s=BUSY_PERIOD_S,
                 commit_s=COMMIT_S, keep_days=KEEP_DAYS, prune_every_s=PRUNE_EVERY_S, logs_dir=None,
                 import_every_s=IMPORT_EVERY_S, pi=None, fetch=fetch_status, probe=probe, sources=SOURCES,
                 clock=time.time):
        self.db_path, self.hosts = db_path, list(jetson_hosts)
        self.period_s, self.busy_period_s, self.commit_s = period_s, busy_period_s, commit_s
        self.keep_days, self.prune_every_s, self.logs_dir, self.import_every_s = keep_days, prune_every_s, logs_dir, import_every_s
        self.pi = pi if pi is not None else PiReader()
        self.fetch, self.probe, self.sources, self.clock = fetch, probe, list(sources), clock
        self.busy = False
        self.db = self.thread = None
        self.stopping = threading.Event()
        self.pending, self.ticks, self.now = [], 0, 0.0
        self.committed = self.pruned = self.imported = self.error_at = None
        self.throttled = None
        self.jetson = self.path = _UNSET
        self.bank = None  # [mark ts, scale, mark percent, Wh used since the mark] while a bank mark exists
        self.motor = None  # [mark ts, motor Wh used since the mark] while a motor battery mark exists
        self.last = None  # (ts, Pi-side watts, motor watts) of the previous sample, for the energy since

    def label(self, host):
        return HOST_NAMES.get(host, host)

    def is_busy(self):
        if self.busy:
            return True
        try:
            return self.logs_dir is not None and run_active(self.logs_dir, self.clock())
        except Exception:
            return False

    def period(self):
        return self.busy_period_s if self.is_busy() else self.period_s

    def _read(self, name, read):
        try:
            values = read()
        except Exception as error:
            self._error(name, error)
            return {}
        return values if isinstance(values, dict) else {}

    def read_all(self):
        """{host: {metric: value or None}} now. Opens no database."""
        data = {"pi": self._read("pi", self.pi.read)}
        data["link"] = {"link_%s_ms" % _name(self.label(h)): self._read_value("probe " + h, self.probe, h)
                        for h in self.hosts}
        jetson = {"jetson_reachable": 0}
        for index, host in enumerate(self.hosts):
            if data["link"]["link_%s_ms" % _name(self.label(host))] is None:
                continue
            start = time.monotonic()
            status = self._read_value("fetch " + host, self.fetch, host)
            if isinstance(status, dict):
                jetson = parse_jetson(status)
                jetson.update(jetson_reachable=1, jetson_path=index,
                              jetson_status_ms=(time.monotonic() - start) * 1000.0)
                break
        data["jetson"] = jetson
        for host, read in self.sources:
            data.setdefault(host, {})
            for metric, value in self._read(host, read).items():
                if value is not None or metric not in data[host]:
                    data[host][metric] = value
        command = None
        if self.logs_dir is not None and self.is_busy():
            command = self._read_value("run log", latest_command, self.logs_dir)
        data["bank"] = {"pi_est_w": pi_watts(_number(data["pi"].get("pi_cpu_pct"))),
                        "motor_est_w": 0.0 if not command else motor_watts(*command)}
        return data

    def _read_value(self, name, read, *args):
        try:
            return read(*args)
        except Exception as error:
            self._error(name, error)
            return None

    def tick(self, now=None):
        """One sample: read everything, note the events, keep the rows; commit every commit_s, prune once a day."""
        self.now = now = self.clock() if now is None else now
        self.ticks += 1
        data = self.read_all()
        try:
            self._open()
            self._bank(now, data["bank"])
        except Exception as error:
            self._error("bank estimate", error)
        self._events(now, data)
        for host, values in data.items():
            for metric, value in values.items():
                value = _number(value)
                if value is not None:
                    self.pending.append(("s", now, host, metric, value))
        if self.committed is None:
            self.committed = now
        if now - self.committed >= self.commit_s:
            self.flush()
        if self.pruned is None or now - self.pruned >= self.prune_every_s:
            self.prune(now)
        if self.logs_dir is not None and not self.is_busy() and (
                self.imported is None or now - self.imported >= self.import_every_s):
            self.imported = now
            try:
                self._open()
                import_runs(self.db, self.logs_dir, now)
            except Exception as error:
                self._error("import runs", error)
        return data

    def _bank(self, now, bank):
        marks = self.db.execute("SELECT ts, percent FROM charge_marks WHERE battery = 'bank' AND ts <= ?"
                                " ORDER BY ts DESC, rowid DESC LIMIT 1", (now,)).fetchone()
        if marks is None:
            self.bank = None
        else:
            scale = _scale(self.db)
            if self.bank is None or self.bank[:3] != [marks[0], scale, marks[1]]:
                self.flush()  # the samples so far must be in the database to count them
                self.bank = [marks[0], scale, marks[1], scale * _usage(self.db, marks[0], now)[0]]
            elif self.last is not None:
                seconds = max(0.0, min(now - self.last[0], MAX_GAP_S))
                self.bank[3] += scale * self.last[1] * seconds / 3600.0 / BANK_EFFICIENCY
            bank["bank_est_wh_used"] = self.bank[3]
            bank["bank_est_pct"] = _percent(self.bank[2], self.bank[3])
        motor_mark = _motor_mark(self.db, now)
        if motor_mark is None:
            self.motor = None
        else:
            if self.motor is None or self.motor[0] != motor_mark:
                self.flush()  # as for the bank: the samples so far must be in the database to count them
                self.motor = [motor_mark, _usage(self.db, motor_mark, now, "motor_est_w", 1.0)[0]]
            elif self.last is not None:
                self.motor[1] += self.last[2] * max(0.0, min(now - self.last[0], MAX_GAP_S)) / 3600.0
            bank["motor_est_wh_used"] = self.motor[1]
        self.last = (now, bank["pi_est_w"], bank["motor_est_w"])

    def _events(self, now, data):
        bits = _number(data["pi"].get("pi_throttled"))
        if bits is not None:
            bits, old = int(bits), self.throttled or 0
            if bits != old:
                kind = "undervolt" if bits & ~old & UNDERVOLT else "throttled"
                names = [text for bit, text in THROTTLE_BITS.items() if bits >> bit & 1] or ["all clear"]
                self.event(now, "pi", kind, "%s%s -> %s: %s" % ("" if self.throttled is not None else "at start: ",
                                                               hex(old), hex(bits), ", ".join(names)))
            self.throttled = bits
        jetson = data["jetson"]
        state = self.label(self.hosts[int(jetson["jetson_path"])]) if jetson.get("jetson_reachable") else None
        if state != self.jetson:
            if state is None:
                detail = "unreachable"
            elif self.jetson is _UNSET:
                detail = "reachable via %s" % state
            elif self.jetson is None:
                detail = "reachable again via %s" % state
            else:
                detail = "now via %s (was %s)" % (state, self.jetson)
            self.event(now, "jetson", "jetson", detail)
            self.jetson = state
        path = next((self.label(h) for h in self.hosts
                     if data["link"].get("link_%s_ms" % _name(self.label(h))) is not None), "none")
        if path != self.path:
            self.event(now, "link", "link", "start: %s" % path if self.path is _UNSET else "%s -> %s" % (self.path, path))
            self.path = path

    def event(self, ts, host, kind, detail):
        self.pending.append(("e", ts, host, kind, detail))

    def _error(self, where, error):
        if self.error_at is None or self.now - self.error_at >= ERROR_EVERY_S:
            self.error_at = self.now
            self.event(self.now, "pi", "error", "%s: %s: %s" % (where, type(error).__name__, error))

    def _open(self):
        if self.db is None:
            self.db = open_db(self.db_path)

    def flush(self):
        """Write the kept rows in one transaction. On failure they stay (up to MAX_PENDING) for the next try."""
        self.committed = self.now
        if not self.pending:
            return
        rows, self.pending = self.pending, []
        try:
            self._open()
            with self.db:
                self.db.executemany("INSERT INTO samples VALUES (?, ?, ?, ?)", [r[1:] for r in rows if r[0] == "s"])
                self.db.executemany("INSERT INTO events VALUES (?, ?, ?, ?)", [r[1:] for r in rows if r[0] == "e"])
        except Exception as error:
            self.pending = (rows + self.pending)[-MAX_PENDING:]
            self._error("write", error)
            print("telemetry: cannot write %s: %s" % (self.db_path, error), file=sys.stderr)
            return
        try:  # a read-only connection sees the latest rows once the WAL is checkpointed
            self.db.execute("PRAGMA wal_checkpoint(PASSIVE)")
        except Exception:
            pass

    def prune(self, now):
        self.pruned = now
        try:
            self._open()
            with self.db:
                self.db.execute("DELETE FROM samples WHERE ts < ?", (now - self.keep_days * 86400.0,))
        except Exception as error:
            self._error("prune", error)

    def close(self):
        """Write what is kept and close the database. Safe to call twice."""
        self.flush()
        if self.db is not None:
            self.db.close()
            self.db = None

    def run(self):
        """Sample until stop() (or Ctrl+C in the foreground), then write the rest."""
        self.now = self.clock()
        self.event(self.now, "pi", "recorder", "started (every %g s, %g s while busy)" % (self.period_s, self.busy_period_s))
        try:
            while not self.stopping.is_set():
                start = time.monotonic()
                try:
                    self.tick()
                except Exception as error:  # tick catches its parts; this is the last guard of the thread
                    self._error("tick", error)
                self.stopping.wait(max(0.0, self.period() - (time.monotonic() - start)))
        finally:
            self.now = self.clock()
            self.event(self.now, "pi", "recorder", "stopped")
            try:
                self.close()
            except Exception as error:
                print("telemetry: close failed: %s" % error, file=sys.stderr)

    def start(self):
        self.thread = threading.Thread(target=self.run, name="telemetry", daemon=True)
        self.thread.start()

    def stop(self, timeout=10.0):
        self.stopping.set()
        if self.thread is not None:
            self.thread.join(timeout)


_UNSET = object()  # no state seen yet: the first one is logged as the start


def once(recorder):
    """One sample as JSON text, nothing written."""
    data = recorder.read_all()
    data["ts"] = time.time()
    return json.dumps(data, indent=1, sort_keys=True)


# --- run logs ---------------------------------------------------------------------------------------------------

def _run_start(name):
    """Unix time of a run folder named <YYYYMMDD-HHMMSS> (local time), or None for any other name."""
    try:
        return time.mktime(time.strptime(name, "%Y%m%d-%H%M%S"))
    except (ValueError, OverflowError):
        return None


def _record(line):
    try:
        record = json.loads(line)
    except ValueError:  # a truncated last line, or not JSON
        return None
    return record if isinstance(record, dict) else None


def _is_cycle(record):
    return (isinstance(record, dict) and _number(record.get("time")) is not None and "v" in record
            and isinstance(record.get("pose"), list))


def _measured(measured):
    """(v, w) from a record's "measured": [v, w, trusted, agree, span], {"v", "w"}, or None."""
    if isinstance(measured, list) and len(measured) >= 2:
        return _number(measured[0]), _number(measured[1])
    if isinstance(measured, dict):
        return _number(measured.get("v")), _number(measured.get("w"))
    return None, None


def _flag(value):
    return int(value) if isinstance(value, bool) else None


def _text(value):
    return None if value is None else str(value)


def _summary(run_id, lines):
    """(runs row, run_samples rows) of one log's lines, or None when it holds no cycle."""
    cycles, end, drive, times = [], None, None, []
    for line in lines:
        record = _record(line)
        if record is None:
            continue
        if _number(record.get("time")) is not None:
            times.append(record["time"])
        if "end" in record:
            end = record
        if isinstance(record.get("drive"), bool):
            drive = int(record["drive"])
        if _is_cycle(record):
            cycles.append(record)
    if not cycles:
        return None
    t0 = cycles[0]["time"]
    seconds = max(times) - t0
    started = _run_start(run_id)
    distance, prev, speeds, locked, stops, samples, kept = 0.0, None, [], 0, {}, [], None
    for c in cycles:
        pose = c["pose"] + [None] * 3
        x, y, th = (_number(p) for p in pose[:3])
        if x is not None and y is not None:
            if prev is not None:
                distance += math.hypot(x - prev[0], y - prev[1])
            prev = (x, y)
        v, w = _number(c.get("v")), _number(c.get("w"))
        if v is not None:
            speeds.append(abs(v))
        locked += c.get("locked") is True
        if c.get("stop") is not None:
            stops[str(c["stop"])] = stops.get(str(c["stop"]), 0) + 1
        v_meas, w_meas = _measured(c.get("measured"))
        if v_meas is not None and drive is None:
            drive = 1  # measured motion exists only when the wheels are driven
        t = c["time"] - t0
        if kept is not None and t - kept < 1.0 / RUN_HZ - 1e-6:
            continue
        kept = t
        operator = c.get("operator")
        op = [_number(p) for p in operator[:2]] if isinstance(operator, list) and len(operator) >= 2 else [None]
        cycle_time = _number(c.get("cycle_time"))
        samples.append((run_id, t, x, y, th, v, w, v_meas, w_meas, _flag(c.get("locked")),
                        math.hypot(*op) if None not in op else None, _number(c.get("nearest")), _text(c.get("stop")),
                        _text(c.get("phase")), _text(c.get("mode")), _flag(c.get("link")),
                        None if cycle_time is None else cycle_time * 1000.0, _text(c.get("wheels"))))
    moving = [s for s in speeds if s > 0.01]
    end_text = None if end is None else str(end.get("end"))
    run = (run_id, started, None if started is None else started + seconds, seconds,
           None if end_text is None else end_text.partition(": ")[0], end_text, drive, len(cycles), distance,
           max(speeds) if speeds else None, sum(moving) / len(moving) if moving else None,
           100.0 * locked / len(cycles), json.dumps(stops, sort_keys=True))
    return run, samples


def import_runs(db, logs_dir=LOGS, now=None):
    """Import every run folder under logs_dir not yet in runs; returns the run ids imported. A log without an
    end record waits until untouched for RUN_DONE_S (it may still be running); a log without any cycle is skipped."""
    now = time.time() if now is None else now
    try:
        folders = sorted(f for f in Path(logs_dir).iterdir() if f.is_dir() and _run_start(f.name) is not None)
    except OSError:
        return []
    known = {r[0] for r in db.execute("SELECT run_id FROM runs")}
    imported = []
    for folder in folders:
        log = folder / "log.jsonl"
        if folder.name in known or not log.is_file():
            continue
        try:
            text = log.read_text(encoding="utf-8", errors="replace")
            mtime = log.stat().st_mtime
        except OSError:
            continue
        summary = _summary(folder.name, text.splitlines())
        if summary is None:
            continue
        run, samples = summary
        if run[4] is None and now - mtime < RUN_DONE_S:
            continue  # no end record yet and still being written
        with db:
            db.execute("INSERT INTO runs VALUES (%s)" % ",".join("?" * len(run)), run)
            db.executemany("INSERT INTO run_samples VALUES (%s)" % ",".join("?" * 18), samples)
        imported.append(folder.name)
    return imported


# --- export -----------------------------------------------------------------------------------------------------

def _local(ts):
    return time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(ts))


def export_csv(db, what, since, until, out_path):
    """Write CSV to out_path; returns the data rows written. what: a table name (all its columns, plus a local
    time after ts), or a list of metric names (one row per sample time, one column per metric). since is
    inclusive, until exclusive; None for no limit."""
    lo = -math.inf if since is None else since
    hi = math.inf if until is None else until
    with open(out_path, "w", newline="", encoding="utf-8") as f:
        out = csv.writer(f)
        if isinstance(what, str) and what in TABLES:
            column = TABLES[what]
            if column is None:  # run_samples: by its run's start
                cursor = db.execute("SELECT s.* FROM run_samples s JOIN runs r USING (run_id)"
                                    " WHERE r.started >= ? AND r.started < ? ORDER BY s.run_id, s.t", (lo, hi))
            else:
                cursor = db.execute("SELECT * FROM %s WHERE %s >= ? AND %s < ? ORDER BY %s, rowid"
                                    % (what, column, column, column), (lo, hi))
            names = [d[0] for d in cursor.description]
            at = names.index("ts") + 1 if "ts" in names else None
            out.writerow(names if at is None else names[:at] + ["time"] + names[at:])
            n = 0
            for row in cursor:
                row = list(row)
                out.writerow(row if at is None else row[:at] + [_local(row[at - 1])] + row[at:])
                n += 1
            return n
        metrics = [what] if isinstance(what, str) else list(what)
        out.writerow(["ts", "time"] + metrics)
        cursor = db.execute("SELECT ts, metric, value FROM samples WHERE metric IN (%s) AND ts >= ? AND ts < ?"
                            " ORDER BY ts" % ",".join("?" * len(metrics)), metrics + [lo, hi])
        n, row, at = 0, None, None
        for ts, metric, value in cursor:
            if ts != at:
                if row is not None:
                    out.writerow(row)
                    n += 1
                at, row = ts, [ts, _local(ts)] + [""] * len(metrics)
            row[2 + metrics.index(metric)] = value
        if row is not None:
            out.writerow(row)
            n += 1
        return n


def when(text, now=None):
    """A time from the command line: unix seconds, a local date or date and time, or an age like 30m, 2h, 7d."""
    found = re.fullmatch(r"(\d+(?:\.\d+)?)([smhd])", text.strip())
    if found:
        unit = {"s": 1, "m": 60, "h": 3600, "d": 86400}[found.group(2)]
        return (time.time() if now is None else now) - float(found.group(1)) * unit
    try:
        return float(text)
    except ValueError:
        return time.mktime(datetime.fromisoformat(text.strip()).timetuple())


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.partition("\n")[0])
    parser.add_argument("--db", default=str(DB_PATH))
    parser.add_argument("--logs", default=str(LOGS), help="run logs, for --run (busy while one is written, imported after)")
    parser.add_argument("--period", type=float, default=PERIOD_S)
    action = parser.add_mutually_exclusive_group(required=True)
    action.add_argument("--once", action="store_true", help="print one sample as JSON, write nothing")
    action.add_argument("--run", action="store_true", help="record until Ctrl+C")
    action.add_argument("--export", metavar="TABLE|METRIC,...", help="CSV of a table or of metrics")
    action.add_argument("--charged", nargs="?", const="bank", choices=BATTERIES,
                        help="a battery was just charged full: bank (the power bank, the default) or motor (21V)")
    action.add_argument("--reading", type=float, metavar="PERCENT", help="the bank's display shows PERCENT")
    action.add_argument("--estimate", action="store_true", help="print the bank's estimated charge")
    action.add_argument("--fit", action="store_true", help="fit the energy model to the readings")
    parser.add_argument("--since", help="unix s, a local date/time, or an age like 2h or 7d")
    parser.add_argument("--until")
    parser.add_argument("--out", default="telemetry.csv")
    args = parser.parse_args(argv)
    if args.once:
        print(once(Recorder(args.db, sources=SOURCES, logs_dir=Path(args.logs))))
        return 0
    if args.run:
        recorder = Recorder(args.db, period_s=args.period, logs_dir=Path(args.logs))
        print("recording to %s every %g s (1 s while a run log is written); Ctrl+C stops" % (args.db, args.period))
        try:
            recorder.run()
        except KeyboardInterrupt:
            pass
        print("stopped")
        return 0
    db = open_db(args.db)
    try:
        if args.export:
            what = args.export if args.export in TABLES else [m for m in args.export.split(",") if m]
            n = export_csv(db, what, None if args.since is None else when(args.since),
                           None if args.until is None else when(args.until), args.out)
            print("%d rows to %s" % (n, args.out))
        elif args.charged is not None:
            mark_full(db, battery=args.charged)
            print("marked: the power bank charged to 100 %" if args.charged == "bank"
                  else "marked: the 21V motor battery charged full (its energy counts from now)")
        elif args.reading is not None:
            add_reading(db, args.reading)
            print("marked: the display shows %g %%" % args.reading)
        elif args.estimate:
            print(json.dumps(dict(estimate(db), motor=motor_usage(db)), indent=1))
        elif args.fit:
            print(json.dumps(fit_coefficients(db), indent=1))
    finally:
        db.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
