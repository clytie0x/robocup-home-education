"""Import run_follow.py's run logs into the telemetry database (telemetry.import_runs). Safe to run again:
runs already in the database are skipped, and a run still being written waits for the next time.

  python tools/runs_to_db.py [--logs DIR] [--db FILE]
  (defaults: logs/ and data/robot.db next to telemetry.py: ~/hmc-base/logs and ~/hmc-base/data/robot.db on the Pi)
"""
import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import telemetry  # noqa: E402


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.partition("\n")[0])
    parser.add_argument("--logs", default=str(telemetry.LOGS))
    parser.add_argument("--db", default=str(telemetry.DB_PATH))
    args = parser.parse_args(argv)
    db = telemetry.open_db(args.db)
    try:
        imported = telemetry.import_runs(db, Path(args.logs))
        total = db.execute("SELECT count(*) FROM runs").fetchone()[0]
    finally:
        db.close()
    print("imported %d runs%s; %d in %s" % (len(imported), (": " + ", ".join(imported)) if imported else "", total, args.db))
    return 0


if __name__ == "__main__":
    sys.exit(main())
