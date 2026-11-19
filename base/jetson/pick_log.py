"""Per-pick phase timings written next to a pick (bag_arm and floor_pick).

Same record as the arm node's pick_log: a phase starts when it is marked and ends
when the next one starts, or at finish. The file is the per-pick JSON.
"""
import json
import os
import time


class PhaseLog:
    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self.rows = []
        self._open = None

    def mark(self, name):
        now = self.clock()
        if self._open is not None:
            phase, started = self._open
            self.rows.append({"phase": phase, "s": round(now - started, 4)})
        self._open = None if name is None else (name, now)

    def finish(self, **extra):
        self.mark(None)
        phases = list(self.rows)
        body = {"phases": phases, "total_s": round(sum(row["s"] for row in phases), 4)}
        body.update(extra)
        return body


def write_pick(directory, body, stamp, task):
    os.makedirs(directory, exist_ok=True)
    safe = "".join(c if c.isalnum() or c in "-_" else "_" for c in str(task)) or "pick"
    path = os.path.join(directory, "%s-%s.json" % (stamp, safe))
    with open(path, "w") as f:
        json.dump(body, f, indent=2)
        f.write("\n")
    return path
