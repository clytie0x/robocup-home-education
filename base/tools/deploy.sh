#!/usr/bin/env bash
# Copy the base code to the Pi (and the Jetson code if base/jetson/ exists). Nothing is ever deleted.
set -euo pipefail
cd "$(dirname "$0")/../.."

PI=xohm@100.73.128.36
JETSON=robotpsu@100.108.202.48

# tar over ssh: no rsync on Windows. Extraction only adds or overwrites files.
push() { # <host> <remote dir> <local dir> [tar args...]
  local host=$1 dest=$2 src=$3; shift 3
  tar -C "$src" --exclude=__pycache__ -cf - "$@" . | ssh "$host" "mkdir -p '$dest' && tar -C '$dest' -xf -"
}

push "$PI" hmc-base/hmc hmc/src/hmc
push "$PI" hmc-base base/pi
scp -q base/params.json "$PI":hmc-base/params.json
# the Pi tests import server.py (test_link.py looks in ../jetson there)
ssh "$PI" "mkdir -p hmc-base/jetson" && scp -q base/jetson/server.py "$PI":hmc-base/jetson/server.py

if [ -d base/jetson ]; then
  push "$JETSON" hmc-follow base/jetson
else
  echo "base/jetson/ does not exist yet, Jetson copy skipped"
fi

# venv on the Pi with system packages; pip install only what fails to import.
ssh "$PI" 'bash -s' <<'REMOTE'
set -euo pipefail
cd ~/hmc-base
[ -d .venv ] || python3 -m venv --system-site-packages .venv
for pair in numpy:numpy cv2:opencv-python-headless serial:pyserial; do
  .venv/bin/python -c "import ${pair%%:*}" 2>/dev/null || .venv/bin/pip install "${pair##*:}"
done
echo "deploy ok"
REMOTE
