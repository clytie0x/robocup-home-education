#!/bin/sh
# Install the systemd user units on one machine. Run as the robot's user, no sudo:
#
#   sh base/systemd/install.sh pi
#   sh base/systemd/install.sh jetson
#   sh base/systemd/install.sh jetson --start-all    # optional, after Start all works by hand
#
# It copies the unit files to ~/.config/systemd/user, enables them and turns on lingering so
# they start at boot without a login. It removes nothing: the crontab lines that become
# redundant are printed, never edited. base/systemd/README.md explains the units.

set -eu

HERE=$(cd "$(dirname "$0")" && pwd)
MACHINE=${1:-}
START_ALL=${2:-}

case "$MACHINE" in
  pi)
    UNITS="hmc-console.service"
    OLD='run_console\.sh|console\.py'
    ;;
  jetson)
    UNITS="hmc-agent.service"
    OLD='agent\.py'
    [ "$START_ALL" = "--start-all" ] && UNITS="$UNITS hmc-start-all.service"
    ;;
  *)
    echo "usage: $0 pi|jetson [--start-all]" >&2
    exit 2
    ;;
esac

if ! command -v systemctl >/dev/null 2>&1; then
  echo "systemctl not found: run this on the robot, as its user, not on a laptop" >&2
  exit 1
fi

USER_NAME=$(id -un)
mkdir -p "$HOME/.config/systemd/user"
for u in $UNITS; do
  cp "$HERE/$u" "$HOME/.config/systemd/user/$u"
  echo "copied $u to ~/.config/systemd/user/"
done

systemctl --user daemon-reload
for u in $UNITS; do
  systemctl --user enable "$u"
done

# Without lingering the user manager ends at logout and nothing starts at boot.
if ! loginctl enable-linger "$USER_NAME"; then
  echo "could not turn on lingering; run: sudo loginctl enable-linger $USER_NAME" >&2
fi

echo
echo "Enabled: $UNITS"
echo "Not started yet. Stop the crontab copy if it runs, then start:"
for u in $UNITS; do
  echo "  systemctl --user start $u"
done
echo
echo "Redundant crontab line(s), not removed:"
crontab -l 2>/dev/null | grep -E "$OLD" || echo "  none found in this user's crontab"
echo
echo "State: systemctl --user status hmc-console.service  (or hmc-agent.service)"
