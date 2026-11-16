"""Runs on the Pi started and stopped over ssh, so the PC app needs no person and no PIN.

The PC's OpenSSH client logs in as xohm@192.168.31.21 (the lab Wi-Fi; Tailscale 100.73.128.36 when that does
not answer) with the key already set up, and sends a bash script on stdin (`ssh host bash -s`), so nothing is
quoted twice. A tunnel (`ssh -N -L 127.0.0.1:18080:127.0.0.1:8080`) brings run_follow's own page (/status,
/scan, /control) to this PC: it listens on the Pi's 127.0.0.1 only.

Start: when the robot console answers on the Pi (it holds the camera and the LiDAR for its live view), the
run is started through it, with the PIN read from ~/.config/hmc-console-pin on the Pi itself (it never comes
here); otherwise run_follow is started directly, detached (setsid, stdin /dev/null), a Drive under
tools/safe_run.py, its process group id in logs/virtual/run.pid. Stop: the console's STOP, run_follow's own
end ({"end": true} to :8080/control, it sends S#), then SIGTERM and at last SIGKILL to that process group
from the pid file. Nothing is ever killed by a name pattern.
"""
import json
import shlex
import socket
import subprocess
import threading

USER = "xohm"
HOSTS = ("192.168.31.21", "100.73.128.36")  # lab Wi-Fi, then Tailscale
TUNNEL_PORT = 18080  # this PC's end of the tunnel to run_follow's page (the Pi's 127.0.0.1:8080)
RUN_PORT = 8080
CONSOLE = "http://127.0.0.1:8081"  # on the Pi
SPEEDS = (0.08, 0.12)
SSH_OPTIONS = ["-o", "BatchMode=yes", "-o", "ConnectTimeout=5", "-o", "ServerAliveInterval=5",
               "-o", "ServerAliveCountMax=2"]


def run_args(settings, pc, port):
    """run_follow's arguments for a virtual operator run (checked here: they go into a shell script)."""
    drive, speed, minutes = settings["drive"], settings["speed"], settings["minutes"]
    if not isinstance(drive, bool) or speed not in SPEEDS or not isinstance(minutes, (int, float)) \
            or isinstance(minutes, bool) or not 1 <= minutes <= 30:
        raise ValueError("drive true/false, speed 0.08 or 0.12, minutes 1 to 30")
    socket.inet_aton(pc)  # one IPv4 address, or OSError
    if pc.count(".") != 3 or not isinstance(port, int):
        raise ValueError("the PC's address must be IPv4")
    return ["--seconds", str(int(round(minutes * 60))), "--max-speed", "%.2f" % speed, "--remote",
            "--dashboard-host", "127.0.0.1", "--virtual-operator", str(port), "--virtual-allow", pc]


def start_script(settings, pc, port):
    args = run_args(settings, pc, port)
    body = json.dumps({"drive": settings["drive"], "speed": settings["speed"], "minutes": settings["minutes"],
                       "virtual": True, "virtual_host": pc})
    run = ".venv/bin/python run_follow.py " + ("--drive " if settings["drive"] else "") + " ".join(map(shlex.quote, args))
    if settings["drive"]:
        run = ".venv/bin/python tools/safe_run.py " + run  # S# however the run ends
    return f"""set -u
cd ~/hmc-base || {{ echo "ERROR no ~/hmc-base"; exit 3; }}
mkdir -p logs/virtual
if [ -f logs/virtual/run.pid ] && kill -0 -- "-$(cat logs/virtual/run.pid)" 2>/dev/null; then
  echo "ERROR a virtual operator run is already running (pid $(cat logs/virtual/run.pid))"; exit 4
fi
if curl -s -m 2 -o /dev/null {CONSOLE}/api/state; then
  # the console holds the camera and the LiDAR: start through it; its PIN stays on this Pi
  PIN=$(cat ~/.config/hmc-console-pin 2>/dev/null || true)
  REPLY=$(curl -s -m 30 -X POST -H 'Content-Type: application/json' -H 'X-HMC-Console: 1' \\
    -H "X-Console-Pin: $PIN" -d {shlex.quote(body)} {CONSOLE}/api/task/start)
  echo "CONSOLE $REPLY"
  exit 0
fi
LOG="logs/virtual/$(date +%Y%m%d-%H%M%S).log"
setsid {run} > "$LOG" 2>&1 < /dev/null &
echo $! > logs/virtual/run.pid
echo "STARTED $! $LOG"
"""


def stop_script():
    return f"""cd ~/hmc-base || exit 3
curl -s -m 3 -X POST -H 'Content-Type: application/json' -d '{{}}' {CONSOLE}/api/stop > /dev/null 2>&1
curl -s -m 2 -X POST -H 'Content-Type: application/json' -H 'X-HMC-Console: 1' -d '{{"end": true}}' \\
  http://127.0.0.1:{RUN_PORT}/control > /dev/null 2>&1
if [ -f logs/virtual/run.pid ]; then
  PID=$(cat logs/virtual/run.pid)
  for i in 1 2 3 4 5 6; do kill -0 -- "-$PID" 2>/dev/null || break; sleep 0.5; done
  if kill -0 -- "-$PID" 2>/dev/null; then kill -TERM -- "-$PID" 2>/dev/null; sleep 3; fi
  if kill -0 -- "-$PID" 2>/dev/null; then kill -KILL -- "-$PID" 2>/dev/null; fi
  rm -f logs/virtual/run.pid
fi
echo STOPPED
"""


def last_record_script():
    """The last line of the newest run_follow log: its end record ("end": "done: back at the start", ...)."""
    return """cd ~/hmc-base || exit 3
D=$(ls -td logs/[0-9]*/ 2>/dev/null | head -n 1)
[ -n "$D" ] && tail -n 1 "$D/log.jsonl"
"""


class Ssh:
    """ssh to the Pi: scripts on stdin, and the tunnel to run_follow's page. run, popen: for tests."""

    def __init__(self, user=USER, hosts=HOSTS, run=subprocess.run, popen=subprocess.Popen):
        self.user, self.hosts = user, list(hosts)
        self.host = self.hosts[0]  # the one that answered last
        self.run, self.popen = run, popen
        self.tunnel = None
        self.lock = threading.Lock()
        self.error = None

    def call(self, script, timeout=40.0):
        """(exit code, output) from the first host that takes the login; ssh's own failure is code 255."""
        order = [self.host] + [h for h in self.hosts if h != self.host]
        last = (255, "")
        for host in order:
            try:
                # bytes: text mode on Windows sends \r\n and bash reads "set -u\r"
                done = self.run(["ssh"] + SSH_OPTIONS + ["%s@%s" % (self.user, host), "bash", "-s"],
                                input=script.encode("utf-8"), capture_output=True, timeout=timeout)
            except (OSError, subprocess.TimeoutExpired) as error:
                last = (255, str(error))
                continue
            out, err = (done.stdout or b"").decode("utf-8", "replace"), (done.stderr or b"").decode("utf-8", "replace")
            if done.returncode != 255:  # 255: ssh itself failed (no route, no login): try the next host
                self.host = host
                return done.returncode, out + err
            last = (255, err)
        self.error = last[1].strip() or "ssh failed"
        return last

    def ensure_tunnel(self):
        """The tunnel up (started again when it died). True when a tunnel process runs."""
        with self.lock:
            if self.tunnel is not None and self.tunnel.poll() is None:
                return True
            try:
                self.tunnel = self.popen(["ssh", "-N"] + SSH_OPTIONS + ["-o", "ExitOnForwardFailure=yes", "-L",
                                         "127.0.0.1:%d:127.0.0.1:%d" % (TUNNEL_PORT, RUN_PORT),
                                         "%s@%s" % (self.user, self.host)],
                                         stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            except OSError as error:
                self.error = "ssh: %s" % error
                self.tunnel = None
                return False
            return True

    def close(self):
        with self.lock:
            if self.tunnel is not None and self.tunnel.poll() is None:
                self.tunnel.terminate()
            self.tunnel = None

    def start(self, settings, pc, port):
        """None, or why the run did not start."""
        try:
            script = start_script(settings, pc, port)
        except (ValueError, OSError) as error:
            return str(error)
        code, out = self.call(script)
        line = next((l for l in reversed(out.splitlines()) if l.startswith(("STARTED", "CONSOLE", "ERROR"))), "")
        if line.startswith("STARTED"):
            return None
        if line.startswith("CONSOLE"):
            try:
                reply = json.loads(line[len("CONSOLE"):].strip() or "null")
            except ValueError:
                return "the console answered: %s" % line[8:200]
            return None if isinstance(reply, dict) and reply.get("error") is None else (
                (reply or {}).get("error") if isinstance(reply, dict) else "the console answered nothing")
        return line[6:] if line.startswith("ERROR") else (out.strip()[-300:] or "ssh exit %d" % code)

    def stop(self):
        code, out = self.call(stop_script(), timeout=30.0)
        return None if "STOPPED" in out else (out.strip()[-300:] or "ssh exit %d" % code)

    def last_end(self):
        """The newest run's end text from its log on the Pi, or None."""
        code, out = self.call(last_record_script(), timeout=15.0)
        try:
            return json.loads(out.strip().splitlines()[-1]).get("end")
        except (ValueError, IndexError, AttributeError):
            return None


def local_address(host):
    """This PC's address on the way to host (no datagram is sent): what the Pi sees as the sender."""
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.connect((host, 9))
        return s.getsockname()[0]
