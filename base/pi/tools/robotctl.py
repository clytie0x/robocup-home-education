"""robotctl: the robot console's API for programs and AI sessions, on the Pi (the console takes a program only from
127.0.0.1, so from elsewhere: ssh xohm@<pi> hmc-base/robotctl arm center).

  robotctl [--who NAME] state | stop | hold | release | log [N]
  robotctl check NAME [args...]                     wheel, floor, base_check, approach, back_up, lap_check,
                                                    tether_run, walk_check (the console's allow-list)
  robotctl task dry|drive SPEED MINUTES [--virtual HOST]
  robotctl run follow|wait|resume|return|stop|bag|bag_again|bag_near
  robotctl arm home|rest|center|stop|state | arm torque on|off
  robotctl arm pick [X Y [TOP YAW]] | arm drop X Y   robot frame m, rad; a pick without X Y: the run's reached bag
  robotctl arm bagpick [--putback]                  the arm team's camera pick of the bag in front (up to 300 s)
  robotctl fetch                                    run bag_near, wait for the bag (FETCH_S), then arm pick
  robotctl jetson start|stop|restart NAME | jetson start-all|stop-all

Sends X-HMC-Client (NAME, default $USER@<first field of $SSH_CLIENT, or "local">) and the PIN from
~/.config/hmc-console-pin when that file exists (never printed). Prints the console's JSON answer; exit 0 when it
took the command, 1 when it refused (the error on stderr), 2 when it does not answer, 3 for arguments that do not fit.
Standard library only: deploy.sh runs it with the Pi's python3 before the venv is checked.
"""
import json
import os
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path

CONSOLE = "http://127.0.0.1:8081"  # ports.CONSOLE, written out: deploy.sh runs this file alone (python3 - < robotctl.py)
PIN_FILE = Path.home() / ".config" / "hmc-console-pin"
RUN = ("follow", "wait", "resume", "return", "bag", "bag_again", "bag_near")
FETCH_S = 120.0  # s fetch waits for the bag task's result
POLL_S = 0.5  # s between fetch's status reads: also the run's page heartbeat (dashboard.PAGE_ALIVE 2 s)
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
    if cmd == "arm" and (rest[:1] == ["pick"] and len(rest) in (1, 3, 5) or rest[:1] == ["drop"] and len(rest) == 3):
        body = {"cmd": "pick_bag" if rest[0] == "pick" else "drop_bag"}
        body.update(zip(("x", "y", "top", "yaw"), map(float, rest[1:])))
        return "POST", "/api/arm", body
    if cmd == "arm" and rest in (["bagpick"], ["bagpick", "--putback"]):
        return "POST", "/api/arm", {"cmd": "bagpick", "putback": len(rest) == 2}
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


def send(method, path, body, heads, base=CONSOLE, timeout=350.0):  # a bagpick: the console answers within 335 s
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


def fetch(send, heads, out, err, clock=time.monotonic, sleep=time.sleep):
    """bag_near, then the run's status until its new bag task reached the bag or failed, then arm pick with the result.
    Ctrl+C: STOP. The exit code as main's."""
    def say(text):
        print("fetch: %s" % text, file=out, flush=True)

    def bag_task(answer):
        record = answer.get("record") if isinstance(answer, dict) else None
        return (record or {}).get("bag_task") or {}

    before = bag_task(send("GET", "/api/run", None, heads)[1]).get("id")
    code, answer = send("POST", "/api/run/command", {"command": "bag_near"}, heads)
    if code != 200 or answer.get("error"):
        print(answer.get("error") or "HTTP %d" % code, file=err)
        return 1
    say("bag_near sent")
    lead = dict(heads, **{"X-HMC-Session": answer.get("session") or ""})  # its status reads keep the run's page alive
    end, state, result = clock() + FETCH_S, None, None
    try:
        while result is None:
            if clock() > end:
                print("no bag result in %.0f s (a refused bag_near is only in the run's log)" % FETCH_S, file=err)
                return 1
            sleep(POLL_S)
            task = bag_task(send("GET", "/api/run?heartbeat=1", None, lead)[1])
            if task.get("id") is None or task.get("id") == before:
                continue  # the run has not started it yet
            if task.get("state") != state:
                state = task.get("state")
                say("bag task %s" % state)
            result = task.get("result")
    except KeyboardInterrupt:
        say("Ctrl+C: STOP")
        send("POST", "/api/stop", {}, heads)
        return 130
    if not result.get("ok"):
        print("bag task failed: %s %s" % (result.get("code"), result.get("message")), file=err)
        return 1
    pick = {"cmd": "pick_bag"}
    pick.update((k, result[k]) for k in ("x", "y", "top", "yaw") if result.get(k) is not None)
    say("reached the bag (%s): arm pick %s" % (result.get("code"), json.dumps(pick)))
    code, answer = send("POST", "/api/arm", pick, heads)
    print(json.dumps(answer, ensure_ascii=False), file=out)
    error = answer.get("error") if isinstance(answer, dict) else None
    if 200 <= code < 300 and not error:
        return 0
    print(error or "HTTP %d" % code, file=err)
    return 1


def main(argv=None, send=send, out=sys.stdout, err=sys.stderr):
    argv = list(sys.argv[1:] if argv is None else argv)
    who = default_who()
    if argv[:1] == ["--who"] and len(argv) > 1:
        who, argv = argv[1], argv[2:]
    if argv == ["fetch"]:
        try:
            return fetch(send, headers(who), out, err)
        except (OSError, ValueError) as error:
            print("the console does not answer on %s (%s)" % (CONSOLE, error), file=err)
            return 2
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
