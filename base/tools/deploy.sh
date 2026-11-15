#!/usr/bin/env bash
# Deploy a git commit to the robot (never the working tree), with a backup first and a one-command rollback.
#
#   base/tools/deploy.sh                 deploy HEAD to the Pi and the Jetson
#   base/tools/deploy.sh --ref b22c5b2   deploy that commit
#   base/tools/deploy.sh --dry-run       show what would happen, change nothing
#   base/tools/deploy.sh --restart       afterwards restart the Pi console (only when no job runs) and the
#                                        Jetson agent and its programs (stop-all, new agent, start-all)
#   base/tools/deploy.sh --params        also overwrite the Pi's params.json when it differs from git
#   base/tools/deploy.sh --rollback      put back the files the newest deploy backup holds, on both hosts
#   base/tools/deploy.sh --voice         also voice/voice_assistant -> Jetson ~/ros2_ws/src/voice_assistant, then
#                                        colcon build --symlink-install --packages-select voice_assistant
#   base/tools/deploy.sh --arm [--arm-ref arm-system]
#                                        also the arm packages (branch arm-system, arm/so100_*, API.md, README.md)
#                                        -> Jetson ~/so100_ws/src, then colcon build of the three packages; the
#                                        running arm stack keeps the old code until it is restarted
#
# What goes where (as before 2026-11-15):
#   Pi      hmc/src/hmc -> ~/hmc-base/hmc, base/pi -> ~/hmc-base, base/params.json -> ~/hmc-base/params.json,
#           base/jetson/{server,slam_bridge,pointing}.py -> ~/hmc-base/jetson (the Pi tests and bag.py use them)
#   Jetson  base/jetson -> ~/hmc-follow
# Each host gets VERSION.json ({"commit", "describe", "branch", "deployed"}) in its folder; the console shows it.
# Before files are overwritten, the ones that exist are packed into <folder>/backups/deploy-<time>.tar.gz (with
# the old VERSION.json). Nothing is ever deleted: files that git no longer has are only listed.
# Host addresses: the lab Wi-Fi first, then Tailscale (it needs the internet).
set -euo pipefail
cd "$(dirname "$0")/../.."

PI_HOSTS="xohm@192.168.31.21 xohm@100.73.128.36"
JETSON_HOSTS="robotpsu@192.168.31.137 robotpsu@100.108.202.48"
REF=HEAD DRY=0 RESTART=0 PARAMS=0 ROLLBACK=0 VOICE=0 ARM=0 ARM_REF=arm-system
while [ $# -gt 0 ]; do
  case $1 in
    --ref) REF=$2; shift ;;
    --dry-run) DRY=1 ;;
    --restart) RESTART=1 ;;
    --params) PARAMS=1 ;;
    --rollback) ROLLBACK=1 ;;
    --voice) VOICE=1 ;;
    --arm) ARM=1 ;;
    --arm-ref) ARM_REF=$2; shift ;;
    *) echo "unknown option $1" >&2; exit 2 ;;
  esac
  shift
done

pick() { # first host of the list that answers ssh within 4 s
  for h in "$@"; do
    if ssh -o ConnectTimeout=4 -o BatchMode=yes "$h" true 2>/dev/null; then echo "$h"; return 0; fi
  done
  return 1
}
PI=$(pick $PI_HOSTS) || { echo "Pi not reachable at: $PI_HOSTS" >&2; exit 1; }
JETSON=$(pick $JETSON_HOSTS) || { echo "Jetson not reachable at: $JETSON_HOSTS" >&2; exit 1; }
echo "Pi $PI, Jetson $JETSON"

if [ $ROLLBACK = 1 ]; then
  for pair in "$PI:hmc-base" "$JETSON:hmc-follow"; do
    host=${pair%%:*} dir=${pair#*:}
    ssh "$host" "cd ~/$dir && f=\$(ls -t backups/deploy-*.tar.gz 2>/dev/null | head -1) && [ -n \"\$f\" ] \
      && { [ $DRY = 1 ] && echo \"$host would restore \$f\" || { tar -xzf \"\$f\" && echo \"$host restored \$f\"; }; } \
      || echo \"$host: no deploy backup\""
  done
  exit 0
fi

COMMIT=$(git rev-parse --verify "$REF^{commit}")
DESCRIBE=$(git describe --always --dirty=-uncommitted-elsewhere "$COMMIT" 2>/dev/null || echo "$COMMIT")
BRANCH=$(git rev-parse --abbrev-ref HEAD)
if [ "$REF" = HEAD ] && [ -n "$(git status --porcelain -- base hmc)" ]; then
  echo "note: base/ or hmc/ has uncommitted changes; they are NOT deployed (only commit $COMMIT is)" >&2
fi
STAMP=$(date +%Y%m%d-%H%M%S)
VERSION=$(printf '{"commit": "%s", "describe": "%s", "branch": "%s", "deployed": "%s"}' \
  "$COMMIT" "$(git describe --always "$COMMIT")" "$BRANCH" "$(date -Iseconds)")

# One host's file set: "<git tree path>=<subfolder of the host folder>" pairs, plus single files.
backup() { # <host> <folder>; the file list on stdin (paths relative to the folder)
  local host=$1 dir=$2
  ssh "$host" "cd ~/$dir && mkdir -p backups && files=\$(while read -r f; do [ -e \"\$f\" ] && echo \"\$f\"; done; \
    if [ -e VERSION.json ]; then echo VERSION.json; fi) && n=\$(printf '%s\n' \"\$files\" | grep -c . || true) && \
    printf '%s\n' \"\$files\" | tar -czf backups/deploy-$STAMP.tar.gz -T - && echo \"$host: backed up \$n files to $dir/backups/deploy-$STAMP.tar.gz\""
}
send() { # <host> <folder> <git tree path> <subfolder>
  # autocrlf off: git archive of a subtree does not see the root .gitattributes, and on Windows it would
  # otherwise write CRLF (seen 2026-11-15: every file on the robot differed from git by its CRs)
  git -c core.autocrlf=false -c core.eol=lf archive --format=tar "$COMMIT:$3" | ssh "$1" "mkdir -p ~/$2/$4 && tar -C ~/$2/$4 -xf -"
}
listing() { # <git tree path> <subfolder prefix>
  git ls-tree -r --name-only "$COMMIT:$1" | sed "s#^#$2#"
}
stale() { # <host> <folder>; tracked list on stdin: .py/.html/.js/.css/.sh files on the host that git does not have
  ssh "$1" "cd ~/$2 && sort > /tmp/deploy-tracked.$$ && find . -path ./backups -prune -o -path ./.venv -prune -o \
    -path ./logs -prune -o -path ./data -prune -o -path ./models -prune -o -name __pycache__ -prune -o -type f \
    \\( -name '*.py' -o -name '*.html' -o -name '*.js' -o -name '*.css' -o -name '*.sh' \\) -print | sed 's#^\./##' | sort \
    | comm -23 - /tmp/deploy-tracked.$$; rm -f /tmp/deploy-tracked.$$"
}

PI_LIST=$( { listing base/pi ""; listing hmc/src/hmc hmc/; echo params.json; \
  for f in server.py slam_bridge.py pointing.py; do echo "jetson/$f"; done; } )
JET_LIST=$(listing base/jetson "")

# params.json holds values measured on the robot: never overwritten silently.
REMOTE_PARAMS=$(ssh "$PI" "cat ~/hmc-base/params.json 2>/dev/null" | tr -d '\r' || true)
if [ -n "$REMOTE_PARAMS" ] && ! diff -q <(git show "$COMMIT:base/params.json" | tr -d '\r') <(echo "$REMOTE_PARAMS") >/dev/null; then
  if [ $PARAMS = 0 ]; then
    echo "Pi params.json differs from git (kept; --params overwrites it):" >&2
    diff <(git show "$COMMIT:base/params.json" | tr -d '\r') <(echo "$REMOTE_PARAMS") >&2 || true
    PI_LIST=$(echo "$PI_LIST" | grep -vx params.json)
    SKIP_PARAMS=1
  fi
fi

echo "deploying $COMMIT ($DESCRIBE): $(echo "$PI_LIST" | wc -l) files to the Pi, $(echo "$JET_LIST" | wc -l) to the Jetson"
if [ $DRY = 1 ]; then
  echo "Pi files git no longer has (left in place):"; echo "$PI_LIST" | stale "$PI" hmc-base | sed 's/^/  /'
  echo "Jetson files git no longer has (left in place):"; echo "$JET_LIST" | stale "$JETSON" hmc-follow | sed 's/^/  /'
  echo "dry run: nothing changed"; exit 0
fi

echo "$PI_LIST" | backup "$PI" hmc-base
echo "$JET_LIST" | backup "$JETSON" hmc-follow

send "$PI" hmc-base base/pi .
send "$PI" hmc-base hmc/src/hmc hmc
[ "${SKIP_PARAMS:-0}" = 1 ] || git show "$COMMIT:base/params.json" | ssh "$PI" "cat > ~/hmc-base/params.json"
for f in server.py slam_bridge.py pointing.py; do
  git show "$COMMIT:base/jetson/$f" | ssh "$PI" "mkdir -p ~/hmc-base/jetson && cat > ~/hmc-base/jetson/$f"
done
send "$JETSON" hmc-follow base/jetson .
echo "$VERSION" | ssh "$PI" "cat > ~/hmc-base/VERSION.json"
echo "$VERSION" | ssh "$JETSON" "cat > ~/hmc-follow/VERSION.json"

echo "Pi files git no longer has (left in place):"; echo "$PI_LIST" | stale "$PI" hmc-base | sed 's/^/  /'
echo "Jetson files git no longer has (left in place):"; echo "$JET_LIST" | stale "$JETSON" hmc-follow | sed 's/^/  /'

# venv on the Pi with system packages; pip install only what fails to import.
ssh "$PI" 'bash -s' <<'REMOTE'
set -euo pipefail
cd ~/hmc-base
[ -d .venv ] || python3 -m venv --system-site-packages .venv
for pair in numpy:numpy cv2:opencv-python-headless serial:pyserial pytest:pytest; do  # pytest: run the suite on the robot
  .venv/bin/python -c "import ${pair%%:*}" 2>/dev/null || .venv/bin/pip install "${pair##*:}"
done
REMOTE

if [ $VOICE = 1 ]; then  # the voice package, built where it always was (symlink install)
  echo "$(listing voice/voice_assistant "")" | backup "$JETSON" ros2_ws/src/voice_assistant
  send "$JETSON" ros2_ws/src/voice_assistant voice/voice_assistant .
  echo "$VERSION" | ssh "$JETSON" "cat > ~/ros2_ws/src/voice_assistant/VERSION.json"
  ssh "$JETSON" 'source /opt/ros/jazzy/setup.bash && cd ~/ros2_ws && colcon build --symlink-install --packages-select voice_assistant 2>&1 | tail -3'
fi
if [ $ARM = 1 ]; then  # the arm packages from their own branch; calibration and top-level files stay as they are
  ARM_COMMIT=$(git rev-parse --verify "$ARM_REF^{commit}")
  ARM_LIST=$( { for d in so100_driver so100_interfaces so100_moveit_config; do git ls-tree -r --name-only "$ARM_COMMIT:arm/$d" | sed "s#^#$d/#"; done; echo API.md; echo README.md; } )
  echo "$ARM_LIST" | backup "$JETSON" so100_ws/src
  for d in so100_driver so100_interfaces so100_moveit_config; do
    git -c core.autocrlf=false -c core.eol=lf archive --format=tar "$ARM_COMMIT:arm/$d" | ssh "$JETSON" "mkdir -p ~/so100_ws/src/$d && tar -C ~/so100_ws/src/$d -xf -"
  done
  for f in API.md README.md; do git show "$ARM_COMMIT:arm/$f" | ssh "$JETSON" "cat > ~/so100_ws/src/$f"; done
  printf '{"commit": "%s", "describe": "%s", "branch": "%s", "deployed": "%s"}' "$ARM_COMMIT"     "$(git describe --always "$ARM_COMMIT")" "$ARM_REF" "$(date -Iseconds)" | ssh "$JETSON" "cat > ~/so100_ws/src/VERSION.json"
  ssh "$JETSON" 'source /opt/ros/jazzy/setup.bash && cd ~/so100_ws && colcon build --symlink-install --packages-select so100_interfaces so100_driver so100_moveit_config 2>&1 | tail -4'
fi

if [ $RESTART = 1 ]; then
  # Pi: the run_console.sh keeper (or base/systemd's hmc-console unit) starts the console again 2 s after it ends;
  # only when no job runs. Jetson: through systemd when hmc-agent.service is installed, else a nohup copy.
  ssh "$PI" 'busy=$(curl -s -m 4 http://127.0.0.1:8081/api/state | python3 -c "import sys,json; print(json.load(sys.stdin)[\"job\"][\"running\"])" 2>/dev/null || echo unknown)
    if [ "$busy" = False ]; then pid=$(pgrep -f "^.venv/bin/python -X faulthandler -u console.py"); [ -n "$pid" ] && kill $pid && echo "Pi console restarted (keeper)"; else echo "Pi console NOT restarted: job running = $busy"; fi'
  # Jetson: stop the programs, start the new agent, start the programs again.
  ssh "$JETSON" 'cd ~/hmc-follow && curl -s -m 30 -X POST http://127.0.0.1:8090/stop-all >/dev/null || true
    if systemctl --user is-enabled hmc-agent.service >/dev/null 2>&1; then systemctl --user restart hmc-agent.service
    else pid=$(pgrep -f "^python3 agent.py"); [ -n "$pid" ] && kill $pid && sleep 2
      mkdir -p logs && setsid nohup python3 agent.py >> logs/agent.out 2>&1 < /dev/null &
    fi
    for i in $(seq 1 20); do curl -s -m 2 http://127.0.0.1:8090/status >/dev/null && break; sleep 0.5; done
    curl -s -m 120 -X POST http://127.0.0.1:8090/start-all | head -c 300; echo'
fi
echo "deploy ok: $COMMIT"
