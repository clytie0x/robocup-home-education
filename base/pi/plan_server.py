"""Plans paths for run_follow.py in its own process, so a plan never holds up the control cycle.

  python plan_server.py [--port 5603]          on the Pi (a free core) or on the Jetson

One JSON object per line each way.
  request: {"id", "start", "goal", "shapes": [["Circle", x, y, r] or ["Box", x0, y0, x1, y1]],
            "near_side", "heading", "robot_radius", "close_plan"}
  answer:  {"id", "path": [[x, y], ...] or null, "seconds"}
Only the newest request waiting in the socket is planned; older ones are dropped unanswered.
The planner is hmc.navigation.plan_path with the real base's settings (the close plan on).
"""
import argparse
import json
import socket
import time

from hmc import navigation, world
from hmc.world import Box, Circle

PORT = 5603
KINDS = {"Circle": Circle, "Box": Box}


def answer(request):
    """The planner's answer to one request (a dict), as a dict."""
    world.ROBOT_RADIUS = request["robot_radius"]
    navigation.CLOSE_PLAN = request.get("close_plan", True)
    shapes = [KINDS[kind](*values) for kind, *values in request["shapes"]]
    t = time.perf_counter()
    path = navigation.plan_path(tuple(request["start"]), tuple(request["goal"]), shapes,
                                request["near_side"], request["heading"])
    return {"id": request["id"], "path": None if path is None else [list(p) for p in path],
            "seconds": round(time.perf_counter() - t, 4)}


def newest(lines):
    """The last complete request among the received lines (the rest are already out of date)."""
    return json.loads(lines[-1]) if lines else None


def serve(conn):
    buffer = b""
    while True:
        data = conn.recv(1 << 20)
        if not data:
            return
        buffer += data
        conn.setblocking(False)
        try:
            while True:  # take everything already waiting, so only the newest is planned
                more = conn.recv(1 << 20)
                if not more:
                    return
                buffer += more
        except BlockingIOError:
            pass
        finally:
            conn.setblocking(True)
        *lines, buffer = buffer.split(b"\n")
        request = newest([line for line in lines if line.strip()])
        if request is not None:
            conn.sendall((json.dumps(answer(request)) + "\n").encode())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=PORT)
    ap.add_argument("--host", default="0.0.0.0")
    args = ap.parse_args()
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((args.host, args.port))
    server.listen(1)
    print("plan server listening on %s:%d" % (args.host, args.port), flush=True)
    while True:
        conn, peer = server.accept()
        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        print("client", peer, flush=True)
        try:
            serve(conn)
        except (OSError, ValueError, KeyError) as error:
            print("client dropped:", error, flush=True)
        finally:
            conn.close()


if __name__ == "__main__":
    main()
