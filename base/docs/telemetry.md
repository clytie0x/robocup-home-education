# Telemetry: what the robot records over time

`pi/telemetry.py` keeps the robot's health in one SQLite file, so power, temperature, speed and the link
can be looked at over days and weeks with plain SQL, pandas or a spreadsheet. Python standard library only.

- Database: `~/hmc-base/data/robot.db` on the Pi (`data/robot.db` next to `telemetry.py`; `--db FILE` changes it).
- Recorder: `python telemetry.py --run` (foreground, Ctrl+C stops), or `Recorder(...).start()` inside a program.
- Run logs: `python tools/runs_to_db.py` imports `logs/<YYYYMMDD-HHMMSS>/log.jsonl`. `--run` also does it
  every 10 min while no run is active.

## What is recorded, and how often

Every 5 s. Every 1 s while a run is active, meaning `recorder.busy = True` or (with `--run`) the newest run
log was written in the last 5 s. Rows are kept in memory and written in one transaction every 10 s (SD card
wear), so a crash loses at most 10 s. A value that cannot be read is null: `--once` prints it as `null`, and
the database stores no row for it. A missing row therefore means "not known". Samples older than 60 days
are deleted once a day. Events, runs and charge marks are kept.

| host | metric | unit | from |
|---|---|---|---|
| pi | `pi_temp_c` | °C | `vcgencmd measure_temp` |
| pi | `pi_cpu_thermal_c` | °C | `/sys/class/thermal/thermal_zone0/temp` |
| pi | `pi_throttled` | bit field | `vcgencmd get_throttled` (bit 0 under-voltage now, 1 frequency capped, 2 throttled, 3 soft temperature limit, bits 16-19 the same since boot) |
| pi | `pi_undervolt`, `pi_undervolt_boot` | 0/1 | bits 0 and 16 of the above |
| pi | `pi_arm_mhz` | MHz | `vcgencmd measure_clock arm` |
| pi | `pi_load1` | load | `/proc/loadavg` |
| pi | `pi_cpu_pct` | % | `/proc/stat`, between two samples |
| pi | `pi_mem_used_mb` | MiB | `/proc/meminfo` (total minus available) |
| pi | `pi_disk_free_gb` | GiB | free space on the SD card |
| pi | `pi_wifi_link`, `pi_wifi_level_dbm` | quality, dBm | `/proc/net/wireless` |
| pi | `pi_uptime_s` | s | `/proc/uptime` |
| link | `link_cable_ms`, `link_wifi_ms` | ms | TCP connect to 10.0.0.2 and 192.168.31.137, port 8090 (5601 if 8090 errors), 0.3 s timeout. A refused connection still proves the path and counts. |
| jetson | `jetson_reachable` | 0/1 | the agent's `GET :8090/status` answered |
| jetson | `jetson_path` | 0 cable, 1 Wi-Fi | which address answered |
| jetson | `jetson_status_ms` | ms | how long `/status` took |
| jetson | `jetson_temp_<zone>_c` | °C | `"thermal"`: cpu, gpu, soc0, soc1, soc2, tj |
| jetson | `jetson_<rail>_mv`, `_ma`, `_mw` | mV, mA, mW | `"power"` (INA3221): `vdd_in`, `vdd_cpu_gpu_cv`, `vdd_soc` |
| jetson | `jetson_gpu_load_pct`, `jetson_cpu_pct`, `jetson_load1`, `jetson_mem_used_mb`, `jetson_mem_total_mb`, `jetson_disk_free_gb`, `jetson_uptime_s` | as named | `/status` |
| jetson | `jetson_prog_<name>_running` | 0/1 | `"programs"` |
| jetson | `jetson_temp_max_c` | °C | the older agent's `"system"` block, if that is what answers |
| bank | `pi_est_w`, `motor_est_w` | W | the energy model below (model numbers, not measured) |
| bank | `bank_est_pct`, `bank_est_wh_used` | %, Wh | the power bank estimate below, once a charge mark exists |
| motor | `motor_bus_mv`, `motor_bus_ma` | mV, mA | a future sensor, null today (see below) |

Metric names are all lower case (`motor_bus_mv`, not `mV`), so `metric = '...'` in SQL never misses on case.
Anything in `/status` of a shape not listed above is ignored, not an error.

Events (`events` table, `kind`):
- `undervolt` when an under-voltage bit (0 or 16) newly appears. `throttled` for any other change of `pi_throttled`.
  The detail reads like `0x0 -> 0x50005: under-voltage now, throttled now, under-voltage since boot, throttled since boot`.
- `jetson` for `reachable via cable`, `unreachable` and `reachable again via wifi`.
- `link` for the path the Pi reaches the Jetson by: `start: cable`, `cable -> wifi`, `wifi -> none`, `none -> cable`.
- `recorder` for started and stopped. `error` when a reader or the database failed, at most one per minute.

Runs (`runs`, one row per run folder):
- `run_id` is the folder name, which is the local start time. `started` is that time as Unix seconds.
- `seconds` runs from the first cycle record to the last record, and `ended = started + seconds`. This is
  approximate: a wait for Enter before the first cycle shifts `ended` by that long.
- `end_reason` is the end record's text before the first `: `, for example `wheels`, `done`, `signal` or `slow cycle`.
  `end_detail` is the whole text. Both are null when the log has no end record (a crash or a cut log).
- `drive` is 1 when measured motion was logged, and null when the log does not say.
- `cycles`, `distance_m` (the sum of odometry steps), `max_v` and `mean_v_moving` (|v| commanded, mean over
  cycles above 0.01 m/s) are also stored.
- `locked_pct` is the share of cycles with the operator locked. `stop_counts` is JSON `{reason: cycles}`.

`run_samples` holds at most 5 per second: `t` (s from the first cycle), `x, y, th` (odometry), `v_cmd, w_cmd`,
`v_meas, w_meas`, `locked`, `operator_m` (operator distance), `nearest_m`, `stop`, `phase` (null in old logs),
`mode`, `link`, `cycle_ms` and `wheels`. Lines that are not cycle records (says, remote status, wheel episodes)
are skipped. So is a truncated last line. A log without an end record waits until it has been untouched for
60 s, because it may still be running. Re-importing is safe: runs already in the database are skipped.

## How big

This is an estimate from one synthetic hour written on a PC, not measured on the robot. A full sample with
the Jetson answering is 47 rows. A row costs about 81 bytes with its indexes.

| state | per hour |
|---|---|
| idle, every 5 s | about 2.7 MB |
| run active, every 1 s | about 13.7 MB, plus about 2.3 MB of `run_samples` for an hour of run |
| Jetson off | about 40 % of the above (19 rows per sample) |

A day with the robot on for 8 hours, 2 of them in runs, comes to about 50 MB. On 24 hours a day idle it is
about 65 MB a day. The 60-day retention caps samples at about 4 GB in that worst case, against 30 GB free.

## How to run

On the Pi, from `~/hmc-base`:

    .venv/bin/python telemetry.py --once           # one sample as JSON: check that values are not all null
    .venv/bin/python telemetry.py --run            # record; leave it in a tmux or nohup
    .venv/bin/python tools/runs_to_db.py           # import finished runs now
    .venv/bin/python telemetry.py --export pi_temp_c,pi_cpu_pct,jetson_vdd_in_mw --since 2d --out temps.csv
    .venv/bin/python telemetry.py --export runs --out runs.csv

`--export` takes a table (`samples`, `events`, `runs`, `run_samples`, `charge_marks`, `bank_fits`) or a
comma list of metrics. Metrics give one row per sample time with a column per metric. `--since` and
`--until` take Unix seconds, a local `2026-11-15 20:00`, or an age like `30m`, `2h` or `7d`.

To copy the database off the robot while the recorder runs, take a consistent copy first. Copying only
`robot.db` can miss the last writes, which are still in `robot.db-wal`.

    python3 -c "import sqlite3; sqlite3.connect('data/robot.db').backup(sqlite3.connect('robot-copy.db'))"

## How to query

`ts` is Unix time in seconds. SQLite turns it into local time with `datetime(ts, 'unixepoch', 'localtime')`.

Jetson input power over one run:

    SELECT s.ts - r.started AS t_s, s.value AS vdd_in_mw
    FROM samples s JOIN runs r ON s.ts BETWEEN r.started AND r.ended
    WHERE r.run_id = '20261115-201500' AND s.metric = 'jetson_vdd_in_mw' ORDER BY s.ts;

Pi temperature by hour over the last week:

    SELECT strftime('%Y-%m-%d %H:00', ts, 'unixepoch', 'localtime') AS hour,
           round(avg(value), 1) AS mean_c, max(value) AS max_c
    FROM samples WHERE metric = 'pi_temp_c' AND ts > unixepoch('now', '-7 days')
    GROUP BY hour ORDER BY hour;

Distance and length of every run:

    SELECT run_id, round(distance_m, 2) AS m, round(seconds / 60, 1) AS minutes, max_v, locked_pct, end_reason
    FROM runs ORDER BY run_id;

Runs that ended because the wheels did not turn:

    SELECT run_id, end_detail FROM runs WHERE end_reason = 'wheels' ORDER BY run_id;

Link path changes and under-voltage, newest first:

    SELECT datetime(ts, 'unixepoch', 'localtime') AS at, kind, detail FROM events
    WHERE kind IN ('link', 'undervolt', 'jetson') ORDER BY ts DESC LIMIT 50;

Throttled now (bit 2) at any time today:

    SELECT count(*) FROM samples WHERE metric = 'pi_throttled' AND (CAST(value AS INTEGER) & 4) != 0
      AND ts > unixepoch('now', 'start of day');

pandas (on a copy of the database):

    import sqlite3, pandas as pd
    db = sqlite3.connect("robot-copy.db")
    df = pd.read_sql("SELECT ts, metric, value FROM samples WHERE metric IN ('pi_temp_c', 'jetson_vdd_in_mw', "
                     "'bank_est_pct') AND ts > unixepoch('now', '-1 day')", db)
    wide = df.pivot_table(index="ts", columns="metric", values="value")
    wide.index = pd.to_datetime(wide.index, unit="s")
    wide.plot(subplots=True)
    runs = pd.read_sql("SELECT * FROM runs", db)
    track = pd.read_sql("SELECT t, x, y, v_cmd, v_meas FROM run_samples WHERE run_id = ?", db, params=["20261115-201500"])

## What cannot be measured: the power bank's charge

The UGREEN power bank feeds the Pi (with the Xtion, the LiDAR and the base board's logic) and the wheel
motors over USB-C. The Jetson is on mains. The bank has no data link, so its percentage cannot be read.
What the robot does have that hints at power:
- the Pi's under-voltage bits (`pi_undervolt`, the `undervolt` events)
- the Jetson's input power `jetson_vdd_in_mw`, which is mains power and says nothing about the bank
- from the run logs, the motor-side symptom: `wheels` in `run_samples`, and runs with `end_reason = 'wheels'`
  (2026-11-14: the motors lost power while the display read 15 %)

### The estimate

`bank_est_pct` is an estimate from a model, not a measurement:

- Charge marks are the starting point. `python telemetry.py --charged` says the bank was charged to 100 %.
  `--reading 62` records the percentage on the bank's display. `mark_full(db)` and `add_reading(db, 62)`
  do the same from code, for the console's buttons. The newest mark is the baseline.
- The model estimates, every sample, the watts drawn from the bank:
  - `pi_est_w = 3.0 + 3.0 × CPU fraction + 2.5`, which is the Pi idle, its CPU, and the Xtion and LiDAR,
    assumed always on.
  - `motor_est_w = 12.0 × min(1, |v| / 0.5 + |w| / 3.0)`. It uses the newest command in the run log being
    written. The logs carry the command, not the PWM. The draw is 0 when no run is active, and 0 while the
    log says the wheels are not turning, because they have no power then.
  - Every coefficient is a guess to be fitted (`telemetry.py`, top of the file). The bank is 25,000 mAh at
    3.7 V, which is 92.5 Wh, computed from the label and not measured. The bank's output efficiency is a guessed 0.85.
- `percent = mark % - 100 × scale × (model Wh since the mark / 0.85) / 92.5`, clamped to 0..100. Each
  sample's watts count until the next sample, but never longer than 60 s. A longer gap counts as the Pi off.
- `estimate(db)` (or `--estimate`) gives `percent` and a `low`/`high` band of ±10 % of the energy used (a guess).
  It also gives `wh_used`, the mark used, `hours_on` and `driving_min` since the mark, and a `warning` below 30 %
  ("the wheels may stop below ~15-20 %").
- Readings improve it. `--fit` (`fit_coefficients(db)`) takes each pair of consecutive marks where the
  display dropped by at least 5 % and the recorder ran in between. A full mark means a charge came between,
  so that pair is not used. It fits one scale on the whole model by least squares, bounded to 0.5..2.0. The
  scale is stored and used from then on only when at least 2 pairs were usable. The result says how many it used.
- What it cannot see: power drawn while the recorder was not running, motors driven by a program that
  writes no run log (wheel_test, back_up), the bank's own ageing, and the display's 1 % steps.

### Hardware that would measure it (prices not verified)

- An inline USB-C power meter with a data port, between the bank and the robot's feed, for example the
  FNIRSI FNB58. It logs volts, amps and Wh, and reads over USB, so a reader could become a source.
- An INA226 or INA219 current and voltage sensor on the motor supply line, on the Pi's I2C bus (`/dev/i2c-1`,
  usually address 0x40). The place for it already exists: `motor_bus()` in `telemetry.py` returns
  `motor_bus_mv` and `motor_bus_ma` as null today. Make it return the sensor's numbers, or add another
  `(host, function)` to `SOURCES`, and the recorder stores them every sample with no other change. Python
  can read the chip without extra packages (`fcntl.ioctl(fd, 0x0703, 0x40)` on `/dev/i2c-1`, then `os.write`
  and `os.read` its registers). With measured motor power, the motor coefficients above could be fitted
  directly instead of guessed.
