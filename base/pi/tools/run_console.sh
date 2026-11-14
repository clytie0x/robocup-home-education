#!/bin/sh
# Keep the robot console running: start it, and start it again 2 s after it ends, with a
# timestamped line in logs/console/console.out each time (Python crash traces on).
#
#   tools/run_console.sh            # usually from the user crontab: @reboot sleep 20 && ~/hmc-base/tools/run_console.sh
#
# Only one copy runs (a lock file). To stop the console for good, stop this script first
# (pkill -f "[r]un_console.sh"), then the console. See base/docs/robot-console.md.
cd "$(dirname "$0")/.." || exit 1
mkdir -p logs/console
exec 9>/tmp/hmc-console.lock
flock -n 9 || exit 0
while true; do
    echo "=== console start $(date '+%F %T')" >> logs/console/console.out
    .venv/bin/python -X faulthandler -u console.py < /dev/null >> logs/console/console.out 2>&1
    echo "=== console ended with code $? at $(date '+%F %T')" >> logs/console/console.out
    sleep 2
done
