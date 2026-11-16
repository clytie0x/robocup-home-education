"""Runs on the Pi started and stopped over ssh, so the PC app needs no person and no PIN.

The PC's OpenSSH client logs in as xohm@192.168.31.21 (the lab Wi-Fi; Tailscale 100.73.128.36 when that does
not answer) with the key already set up, and sends a bash script on stdin (`ssh host bash -s`), so nothing is
quoted twice. A tunnel (`ssh -N -L 127.0.0.1:18080:127.0.0.1:8080`) brings run_follow's own page (/status,
/scan, /control) to this PC: it listens on the Pi's 127.0.0.1 only.

Start and stop go only through the robot console, with ~/hmc-base/robotctl on the Pi (it reads the PIN from
~/.config/hmc-console-pin there, so the PIN never comes here): `robotctl task dry|drive SPEED MINUTES --virtual
<this PC>` and `robotctl stop`. Without a console the app says so; it never starts run_follow itself.
"""
import importlib.util
import json
import shlex
import socket
import subprocess
import threading
from pathlib import Path

USER = "xohm"
HOSTS = ("192.168.31.21", "100.73.128.36")  # lab Wi-Fi, then Tailscale
TUNNEL_PORT = 18080  # this PC's end of the tunnel to run_follow's page (the Pi's 127.0.0.1:8080)
_spec = importlib.util.spec_from_file_location("ports", Path(__file__).resolve().parents[2] / "jetson" / "ports.py")
ports = importlib.util.module_from_spec(_spec)  # base/jetson/ports.py, the one port table
_spec.loader.exec_module(ports)
RUN_PORT = ports.DASHBOARD
SPEEDS = (0.08, 0.12)
SSH_OPTIONS = ["-o", "BatchMode=yes", "-o", "ConnectTimeout=5", "-o", "ServerAliveInterval=5",
               "-o", "ServerAliveCountMax=2"]


def task_args(settings, pc):
    """robotctl's arguments for a virtual operator run (checked here: they go into a shell script)."""
    drive, speed, minutes = settings["drive"], settings["speed"], settings["minutes"]
    if not isinstance(drive, bool) or speed not in SPEEDS or not isinstance(minutes, (int, float)) \
            or isinstance(minutes, bool) or not 1 <= minutes <= 30:
        raise ValueError("drive true/false, speed 0.08 or 0.12, minutes 1 to 30")
    socket.inet_aton(pc)  # one IPv4 address, or OSError
    if pc.count(".") != 3:
        raise ValueError("the PC's address must be IPv4")
    return ["task", "drive" if drive else "dry", "%.2f" % speed, "%g" % minutes, "--virtual", pc]


def robotctl_script(args):
    """robotctl's answer as one line: CONSOLE <its JSON> when the console took it, else ERROR <why>."""
    words = " ".join(map(shlex.quote, args))
    return f"""set -u
cd ~/hmc-base || {{ echo "ERROR no ~/hmc-base"; exit 3; }}
[ -x ./robotctl ] || {{ echo "ERROR no ~/hmc-base/robotctl on the Pi: deploy first"; exit 3; }}
if REPLY=$(./robotctl --who virtual-operator {words} 2>&1); then
  echo "CONSOLE $REPLY"
else
  echo "ERROR $(printf '%s\\n' "$REPLY" | tail -n 1)"
fi
"""


def start_script(settings, pc, port):
    """port: the console picks its own (VIRTUAL_PORT); kept for the app's call."""
    return robotctl_script(task_args(settings, pc))


def stop_script():
    return robotctl_script(["stop"])


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
        return self._robotctl(script)

    def _robotctl(self, script, timeout=40.0):
        """None when the console took robotctl's command, else why not."""
        code, out = self.call(script, timeout=timeout)
        line = next((l for l in reversed(out.splitlines()) if l.startswith(("CONSOLE", "ERROR"))), "")
        if line.startswith("CONSOLE"):
            try:
                reply = json.loads(line[len("CONSOLE"):].strip() or "null")
            except ValueError:
                return "the console answered: %s" % line[8:200]
            return None if isinstance(reply, dict) and reply.get("error") is None else (
                (reply or {}).get("error") if isinstance(reply, dict) else "the console answered nothing")
        return line[6:] if line.startswith("ERROR") else (out.strip()[-300:] or "ssh exit %d" % code)

    def stop(self):
        return self._robotctl(stop_script(), timeout=30.0)

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
