"""robotctl: the robot console's API for programs and AI sessions, on the Pi (the console takes a program only from
127.0.0.1, so from elsewhere: ssh xohm@<pi> hmc-base/robotctl arm center).

  robotctl [--who NAME] state | stop | hold | release | log [N]
  robotctl check NAME [args...]                     wheel, floor, base_check, approach, back_up, lap_check,
                                                    tether_run, walk_check (the console's allow-list)
  robotctl task dry|drive SPEED MINUTES [--virtual HOST]
  robotctl run follow|wait|resume|return|stop
  robotctl arm home|rest|center|stop|state | arm torque on|off
  robotctl jetson start|stop|restart NAME | jetson start-all|stop-all

Sends X-HMC-Client (NAME, default $USER@<first field of $SSH_CLIENT, or "local">) and the PIN from
~/.config/hmc-console-pin when that file exists (never printed). Prints the console's JSON answer; exit 0 when it
took the command, 1 when it refused (the error on stderr), 2 when it does not answer, 3 for arguments that do not fit.
Standard library only: deploy.sh runs it with the Pi's python3 before the venv is checked.
"""
import json
import os
import sys
import urllib.error
import urllib.request
from pathlib import Path

CONSOLE = "http://127.0.0.1:8081"
PIN_FILE = Path.home() / ".config" / "hmc-console-pin"
RUN = ("follow", "wait", "resume", "return")
JETSON = ("start", "stop", "restart")


def default_who(environ=os.environ):
    ssh = (environ.get("SSH_CLIENT") or "").split()
    return "%s@%s" % (environ.get("USER") or "unknown", ssh[0] if ssh else "local")


def request(argv, who="local"):
    """(method, path, body or None) for robotctl's arguments; ValueError when they do not fit."""
    cmd, rest = (argv[0], list(argv[1:])) if argv else ("", [])
    if cmd in ("state", "stop", "hold", "release") and not rest:
        return {"state": ("GET", "/api/state", None), "stop": ("POST", "/api/stop", {}),
                "hold": ("POST", "/api/hold", {"who": who}), "release": ("POST", "/api/release", {})}[cmd]
    if cmd == "log" and len(rest) <= 1 and all(r.isdigit() for r in rest):
        return "GET", "/api/commands?n=%s" % (rest[0] if rest else "20"), None
    if cmd == "check" and rest:
        return "POST", "/api/check/start", {"check": rest[0], "args": rest[1:]}
    if cmd == "task" and len(rest) in (3, 5) and rest[0] in ("dry", "drive") and (len(rest) == 3 or rest[3] == "--virtual"):
        body = {"drive": rest[0] == "drive", "speed": float(rest[1]), "minutes": float(rest[2])}
        if len(rest) == 5:
            body.update(virtual=True, virtual_host=rest[4])
        return "POST", "/api/task/start", body
    if cmd == "run" and len(rest) == 1 and rest[0] in RUN + ("stop",):
        return ("POST", "/api/stop", {}) if rest[0] == "stop" else ("POST", "/api/run/command", {"command": rest[0]})
    if cmd == "arm" and len(rest) == 1 and rest[0] in ("home", "rest", "center", "stop", "state"):
        return {"stop": ("POST", "/api/arm/stop", {}), "state": ("GET", "/api/arm", None)}.get(
            rest[0], ("POST", "/api/arm", {"cmd": rest[0]}))
    if cmd == "arm" and len(rest) == 2 and rest[0] == "torque" and rest[1] in ("on", "off"):
        return "POST", "/api/arm", {"cmd": "torque", "on": rest[1] == "on"}
    if cmd == "jetson" and len(rest) == 2 and rest[0] in JETSON:
        return "POST", "/api/jetson/" + rest[0], {"name": rest[1]}
    if cmd == "jetson" and len(rest) == 1 and rest[0] in ("start-all", "stop-all"):
        return "POST", "/api/jetson/" + rest[0], {}
    raise ValueError("arguments do not fit")


def headers(who, pin_file=PIN_FILE):
    out = {"Content-Type": "application/json", "X-HMC-Client": who}
    try:
        pin = Path(pin_file).read_text().strip()
    except OSError:
        pin = ""
    if pin:
        out["X-Console-Pin"] = pin
    return out


def send(method, path, body, heads, base=CONSOLE, timeout=120.0):
    """(HTTP code, JSON answer). OSError when the console does not answer."""
    data = None if body is None else json.dumps(body).encode()
    req = urllib.request.Request(base + path, data, heads, method=method)
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return r.status, json.loads(r.read() or b"null")
    except urllib.error.HTTPError as e:
        try:
            return e.code, json.loads(e.read() or b"null")
        except ValueError:
            return e.code, {"error": "HTTP %d" % e.code}


def main(argv=None, send=send, out=sys.stdout, err=sys.stderr):
    argv = list(sys.argv[1:] if argv is None else argv)
    who = default_who()
    if argv[:1] == ["--who"] and len(argv) > 1:
        who, argv = argv[1], argv[2:]
    try:
        method, path, body = request(argv, who)
    except ValueError:  # also a speed or minutes that is not a number
        print(__doc__.split("\n\n")[1], file=err)
        return 3
    try:
        code, answer = send(method, path, body, headers(who))
    except (OSError, ValueError) as error:
        print("the console does not answer on %s (%s)" % (CONSOLE, error), file=err)
        return 2
    print(json.dumps(answer, ensure_ascii=False), file=out)
    error = answer.get("error") if isinstance(answer, dict) else None
    if 200 <= code < 300 and not error:
        return 0
    print(error or "HTTP %d" % code, file=err)
    return 1


if __name__ == "__main__":
    sys.exit(main())
