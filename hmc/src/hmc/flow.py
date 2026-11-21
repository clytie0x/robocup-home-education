"""The one Help Me Carry executive (RoboCup@Home 2025.1, section 5.1).

Pure Python, standard library only. The Pi console starts and resets it from
POST /api/task/start. It owns the whole run: follow the operator to the car,
ask "Are we at the car?" and store the car pose, take a point (or the words
left/right), pick, ask for the bag if the pick misses, then always try to
return to the start. The 5:00 clock is this object's. run_follow is a step
it drives (follow, wait, bag, return), not a second mission.

Every reason code has English, Thai, and a suggested fix. Events are one
JSON object per line (state enter/exit, outcome, reason, timestamps).
"""

import json
import re
import threading
from datetime import datetime, timedelta, timezone
from pathlib import Path

from hmc import hri

ICT = timezone(timedelta(hours=7))
LIMIT_S = 300.0  # the rulebook 5:00
RETURN_BUDGET = 180.0  # extra seconds to try to get home after a fault or the clock
ENROL_S = 30.0
ENROL_TRIES = 3
LOST_SEARCH_S = 15.0  # drive toward the last spot, then turn: then ask them back and stay
CONFIRM_S = 20.0
CONFIRM_TRIES = 3
REASK_S = 15.0
POINT_S = 180.0
BAG_TRIES = 3
PICK_S = 240.0  # the arm's pick, as the Jetson flow is launched
HANDOVER_S = 30.0
DROP_S = 60.0
LINK_GRACE_S = 4.0
LINK_ASKS = 2  # spoken refusals while the base link is unknown or down; then the reason sticks
PAUSE_ASK_S = 20.0  # a wait on the way back never resumes by itself

# The spine the console shows as step i of N. LOST_SEARCH shares FOLLOWING;
# HANDOVER shares PICKING. FAILED and ABORTED keep the step they left.
STEPS = (
    "PRECHECK",
    "READY",
    "ENROLLING",
    "FOLLOWING",
    "CONFIRM_CAR",
    "AT_CAR",
    "POINTING",
    "PICKING",
    "RETURNING",
    "DROPPING",
    "DONE",
)
TERMINAL = ("DONE", "FAILED", "ABORTED")
RETRY_BAG = ("NOT_POINTED", "NO_BAG", "BLOCKED", "BAG_TIMEOUT", "LOST_BAG", "TIMEOUT")
REMEASURE = ("MISSED", "NOT_FOUND", "PICK_MISSED")

# code -> operator text. outcome is SUCCESS, FAILED, or ABORTED when the code ends a run.
REASONS = {
    "SUCCESS": {
        "en": "Back at the start.",
        "th": "กลับถึงจุดเริ่มแล้ว",
        "fix": "No fix. The run finished.",
        "level": "info",
        "outcome": "SUCCESS",
    },
    "ODOM_STALE": {
        "en": "Odometry was interrupted. The run has stopped.",
        "th": "ค่าตำแหน่งล้อขาดช่วง การวิ่งหยุดแล้ว",
        "fix": "Check the wheel-encoder cable, then start the run again.",
        "level": "error",
        "outcome": "FAILED",
    },
    "BASE_LINK_DOWN": {
        "en": "The base link is not up, so following and the bag search stay off.",
        "th": "ลิงก์ไปฐานไม่พร้อม จึงยังไม่ตามและยังไม่หาถุง",
        "fix": "Start the Pi base program, and the base bridge on the Jetson tab: without it the base holds still.",
        "level": "error",
        "outcome": "FAILED",
    },
    "NO_LINK": {
        "en": "The base is not connected, so the bag search cannot start.",
        "th": "ฐานไม่เชื่อมต่อ จึงหาถุงไม่ได้",
        "fix": "Start the run from the console so the Pi is listening, then try again.",
        "level": "error",
        "outcome": "FAILED",
    },
    "TIME_LIMIT": {
        "en": "The 5 minute limit is up. The robot is trying to return.",
        "th": "ครบ 5 นาทีแล้ว หุ่นกำลังพยายามกลับจุดเริ่ม",
        "fix": "No fault. Start a new run if you want another try.",
        "level": "warn",
        "outcome": "ABORTED",
    },
    "OPERATOR_LOST": {
        "en": "The operator is lost. Please come back in front of the robot.",
        "th": "หาคนนำไม่เจอ กรุณากลับมายืนข้างหน้าหุ่น",
        "fix": "Stand in front of the camera, in the same light as at the start.",
        "level": "warn",
        "outcome": "FAILED",
    },
    "PICK_FAILED": {
        "en": "The bag was not picked up. The robot is going back.",
        "th": "หยิบถุงไม่สำเร็จ หุ่นกำลังกลับจุดเริ่ม",
        "fix": "Point again, or place the bag in the gripper when asked.",
        "level": "error",
        "outcome": "FAILED",
    },
    "OPERATOR_STOP": {
        "en": "STOP was pressed. Motion has stopped.",
        "th": "กด STOP แล้ว การเคลื่อนที่หยุด",
        "fix": "Start a new run when the area is clear. STOP does not shut the Jetson programs.",
        "level": "warn",
        "outcome": "ABORTED",
    },
    "WHEEL_FAULT_02": {
        "en": "The right wheel reported fault 02. Check the motor power.",
        "th": "ล้อขวาแจ้งรหัส 02 ตรวจไฟมอเตอร์",
        "fix": "Check that the 21 V motor battery is plugged in and charged.",
        "level": "error",
        "outcome": "FAILED",
    },
    "WHEEL_FAULT_03": {
        "en": "A wheel reported fault 03. Check the motor power.",
        "th": "ล้อแจ้งรหัส 03 ตรวจไฟมอเตอร์",
        "fix": "Check that the 21 V motor battery is plugged in and charged.",
        "level": "error",
        "outcome": "FAILED",
    },
    "WHEELS_NOT_TURNING": {
        "en": "The wheels are not turning.",
        "th": "ล้อไม่หมุน",
        "fix": "Check that the 21 V motor battery is plugged in and the wheels are free.",
        "level": "error",
        "outcome": "FAILED",
    },
    "REID_DOWN": {
        "en": "Person re-identification is down, so the operator cannot be learned.",
        "th": "ระบบจำคนไม่ทำงาน จึงจำผู้ควบคุมไม่ได้",
        "fix": "Restart the Jetson detector program and check detector.log for TensorRT errors.",
        "level": "error",
        "outcome": "FAILED",
    },
    "NO_MOTOR": {
        "en": "The motor battery has no power.",
        "th": "แบตมอเตอร์ไม่มีไฟ",
        "fix": "Plug in and charge the 21 V motor battery, then run the wheel check.",
        "level": "error",
        "outcome": "FAILED",
    },
    "NO_CAMERA": {
        "en": "A camera is not open.",
        "th": "กล้องไม่เปิด",
        "fix": "Check the lower camera on the Pi and the bag camera on the Jetson.",
        "level": "error",
        "outcome": "FAILED",
    },
    "AGENT_DOWN": {
        "en": "The Jetson agent is not answering.",
        "th": "เอเจนต์บน Jetson ไม่ตอบ",
        "fix": "On the Jetson, start agent.py and check that port 8090 is open.",
        "level": "error",
        "outcome": "FAILED",
    },
    "HELD": {
        "en": "The console is held by a deploy or a repair.",
        "th": "คอนโซลถูกถือไว้ระหว่างดีพลอยหรือซ่อม",
        "fix": "Release the console hold, then start the run.",
        "level": "warn",
        "outcome": "FAILED",
    },
    "ENROL_TIMEOUT": {
        "en": "The operator was not learned in time.",
        "th": "จำผู้ควบคุมไม่ทันเวลา",
        "fix": "Stand about 1 m in front, alone, in the light, then say follow me again.",
        "level": "warn",
        "outcome": "FAILED",
    },
    "NOBODY_IN_VIEW": {
        "en": "Nobody is in the camera view.",
        "th": "ไม่เห็นคนในภาพ",
        "fix": "Step in front of the lower camera.",
        "level": "warn",
        "outcome": "FAILED",
    },
    "DARK_FRAME": {
        "en": "The camera frame is too dark to learn the operator.",
        "th": "ภาพมืดเกินไป จะจำผู้ควบคุมไม่ได้",
        "fix": "Turn on a light or face a brighter spot, then try again.",
        "level": "warn",
        "outcome": "FAILED",
    },
    "REID_FAIL": {
        "en": "The person in view does not look like the operator.",
        "th": "คนในภาพไม่ตรงกับผู้ควบคุม",
        "fix": "The operator should stand alone in front. Restart if the detector is failing.",
        "level": "warn",
        "outcome": "FAILED",
    },
    "AMBIGUOUS": {
        "en": "More than one person is close. The operator is not clear.",
        "th": "มีคนใกล้กันหลายคน แยกผู้ควบคุมไม่ได้",
        "fix": "Ask the others to step back, then say follow me again.",
        "level": "warn",
        "outcome": "FAILED",
    },
    "NOT_POINTED": {
        "en": "No pointing was seen.",
        "th": "ไม่เห็นการชี้",
        "fix": "Step back about two steps and point at the bag with a straight arm.",
        "level": "warn",
        "outcome": "FAILED",
    },
    "NO_BAG": {
        "en": "No bag was seen where you pointed.",
        "th": "ไม่เห็นถุงตรงที่ชี้",
        "fix": "Point at a bag on the floor in front of the camera, or say left or right.",
        "level": "warn",
        "outcome": "FAILED",
    },
    "BLOCKED": {
        "en": "Something is blocking the way to the bag.",
        "th": "มีสิ่งกีดขวางทางไปหาถุง",
        "fix": "Clear the floor between the robot and the bag, then point again.",
        "level": "warn",
        "outcome": "FAILED",
    },
    "BAG_TIMEOUT": {
        "en": "Looking for the bag took too long.",
        "th": "หาถุงนานเกินไป",
        "fix": "Point again, or say left or right.",
        "level": "warn",
        "outcome": "FAILED",
    },
    "LOST_BAG": {
        "en": "The bag was lost on the way to it.",
        "th": "ถุงหายจากภาพระหว่างเข้าไปหา",
        "fix": "Point at the bag again.",
        "level": "warn",
        "outcome": "FAILED",
    },
    "BAG_REFUSED": {
        "en": "The base refused the bag search.",
        "th": "ฐานปฏิเสธการหาถุง",
        "fix": "Wait until the robot is holding still at the car, then say ready.",
        "level": "error",
        "outcome": "FAILED",
    },
    "PICK_MISSED": {
        "en": "The gripper missed the bag. Looking once more.",
        "th": "มือจับพลาดถุง กำลังมองอีกครั้ง",
        "fix": "Leave the bag still. If it misses again, place it in the gripper.",
        "level": "warn",
        "outcome": "FAILED",
    },
    "PICK_UNREACHABLE": {
        "en": "The bag is out of the arm's reach.",
        "th": "ถุงอยู่ไกลเกินที่แขนจะถึง",
        "fix": "Move the bag closer, or place it in the gripper when asked.",
        "level": "error",
        "outcome": "FAILED",
    },
    "PICK_TIMEOUT": {
        "en": "The pick took too long.",
        "th": "การหยิบใช้เวลานานเกินไป",
        "fix": "Place the bag in the gripper when asked, or start the pick again.",
        "level": "error",
        "outcome": "FAILED",
    },
    "NOT_HOLDING": {
        "en": "The robot is not holding a bag, so it cannot put one down.",
        "th": "หุ่นไม่ได้ถือถุง จึงวางถุงไม่ได้",
        "fix": "Do not drop unless a pick just succeeded. Start a new run to try again.",
        "level": "error",
        "outcome": "FAILED",
    },
    "DROP_FAILED": {
        "en": "The bag was not put down.",
        "th": "วางถุงไม่สำเร็จ",
        "fix": "Take the bag from the gripper. Check the arm log if it stays closed.",
        "level": "error",
        "outcome": "FAILED",
    },
    "DROP_TIMEOUT": {
        "en": "Putting the bag down took too long.",
        "th": "การวางถุงใช้เวลานานเกินไป",
        "fix": "Take the bag from the gripper.",
        "level": "error",
        "outcome": "FAILED",
    },
    "BASE_RUN_ENDED": {
        "en": "The base run ended before the task was done.",
        "th": "การวิ่งของฐานจบก่อนงานเสร็จ",
        "fix": "Read the reason below. Start a new run from the console.",
        "level": "error",
        "outcome": "FAILED",
    },
    "SLOW_CYCLE": {
        "en": "A control cycle was too slow, so the base stopped.",
        "th": "รอบควบคุมช้าเกินไป ฐานจึงหยุด",
        "fix": "Close extra camera viewers and start the run again.",
        "level": "error",
        "outcome": "FAILED",
    },
    "CAR_UNCONFIRMED": {
        "en": "Still waiting for a yes or a no: are we at the car?",
        "th": "ยังรอคำตอบว่าถึงรถหรือยัง",
        "fix": "Say yes to store this place as the car, or no to keep following.",
        "level": "warn",
        "outcome": "FAILED",
    },
}

# Speech. A negation in the same clause cancels the word ("do not stop").
_NEG = re.compile(r"\b(don'?t|do not|never|not now)\b")
_CLAUSE = re.compile(r"[,.!?;]+")
_WORDS = (
    ("side", re.compile(r"\b(left|right)\b")),
    ("arrive", re.compile(r"\b(stop|halt|freeze|stay)\b|\bthis is (the|my|your) car\b|\bwe'?re here\b"
                          r"|\bwe are here\b|\bhere we are\b")),
    ("yes", re.compile(r"\b(yes|yeah|yep|correct|we are)\b")),
    ("no", re.compile(r"\b(no|nope|not yet)\b")),
    ("follow", re.compile(r"\bfollow\b|\bcome with me\b")),
    ("back", re.compile(r"\b(go|take it|bring it) back\b|\bgo to the start\b|\bgo home\b|\breturn\b")),
    ("resume", re.compile(r"\b(continue|resume|keep going|go on)\b")),
    ("done", re.compile(r"\b(done|finished|ready|go ahead|got it|okay|ok)\b")),
    ("wait", re.compile(r"\bwait\b")),
)
_SHORT = 3
_PI_END = (
    ("ODOM_STALE", ("odometry", "odom")),
    ("WHEEL_FAULT_02", ("fault 02", "fault 2")),
    ("WHEEL_FAULT_03", ("fault 03", "fault 3")),
    ("SLOW_CYCLE", ("slow cycle",)),
    ("OPERATOR_STOP", ("dashboard", "stop pressed")),
    ("WHEELS_NOT_TURNING", ("not turning", "wheels")),
    ("TIME_LIMIT", ("time", "seconds")),
)


def classify(text):
    """follow, arrive, yes, no, side, back, resume, done, wait, or None.

    left/right, yes/no and wait count only in a short sentence, so a long
    remark does not steer the task. "here" alone is not an arrival.
    """
    low = (text or "").lower().strip()
    if not low:
        return None, ""
    short = len(re.findall(r"[a-z']+", low)) <= _SHORT
    for kind, rx in _WORDS:
        if kind in ("side", "yes", "no", "wait", "done") and not short and kind != "done":
            # "done"/"ready" may be short only; a long "ready" sentence is not a command
            if kind != "done":
                continue
        if kind == "done" and not short and not re.search(r"\b(done|finished|ready|got it)\b", low):
            continue
        for match in rx.finditer(low):
            if _NEG.search(_CLAUSE.split(low[:match.start()])[-1]):
                continue
            extra = match.group(1) if kind == "side" and match.lastindex else ""
            return kind, extra
    return None, ""


def code_from_pi_end(text):
    """Map a Pi end line ('odometry stale...', 'fault 03...') onto a reason code."""
    low = (text or "").lower()
    for code, keys in _PI_END:
        if any(key in low for key in keys):
            return code
    return "BASE_RUN_ENDED"


def _precheck_code(report):
    """The first failed precheck, or None when the report does not show a fault.

    A missing key or None means unknown, not a failure: a dry start has no
    Jetson snapshot yet and must still be allowed to reach READY.
    """
    if not isinstance(report, dict):
        return None, ""
    if report.get("held") is True:
        return "HELD", "console hold"
    fault = report.get("wheel_fault")
    if fault in ("02", "2", "WHEEL_FAULT_02"):
        return "WHEEL_FAULT_02", "precheck"
    if fault in ("03", "3", "WHEEL_FAULT_03"):
        return "WHEEL_FAULT_03", "precheck"
    checks = (
        ("link", "NO_LINK"),
        ("odom_fresh", "ODOM_STALE"),
        ("motor_ok", "NO_MOTOR"),
        ("camera", "NO_CAMERA"),
        ("reid", "REID_DOWN"),
        ("agent", "AGENT_DOWN"),
    )
    for key, code in checks:
        if report.get(key) is False:
            return code, "precheck %s" % key
    return None, ""


class HmcFlow:
    """Mission executive. Methods return ("say"|"cmd"|"state"|"outcome"|"log", value).

    now is seconds on the task clock (monotonic is fine). wall() is used only
    for the log timestamp, in Asia/Bangkok (+07:00).
    """

    def __init__(self, use_arm=True, limit_s=LIMIT_S, wall=None):
        self.use_arm = use_arm
        self.limit_s = float(limit_s)
        self.wall = wall or (lambda: __import__("time").time())
        self._lock = threading.Lock()
        self.log_path = None
        self.run_id = None
        self._cleared(0.0)
        self.state = "IDLE"
        self.running = False

    def _cleared(self, now):
        self.pace = None  # m/s the operator is asked to walk at (hri.PACE_SHARE of the top speed), or None
        self.t0 = None
        self.last_now = now
        self.outcome = None
        self.reason = None
        self.pending = None
        self.pending_detail = ""
        self.anchor = None
        self.left_start = False
        self.carrying = False
        self.had_bag = False
        self.car_pose = None
        self.pose = None
        self.side = None
        self.bag_at = None
        self.bag_top = None
        self.bag_yaw = None
        self.bag_tries = 0
        self.remeasured = False
        self.enrol_tries = 0
        self.confirm_asks = 0
        self.link_ok = None
        self.link_lost_at = None
        self.link_tries = 0
        self.link_stuck = False
        self.side_fallback_used = False
        self.return_deadline = None
        self.paused = False
        self.paused_at = None
        self.pause_asked = False
        self.come_back_said = False
        self.entered_at = now
        self.deadline = None
        self.last_say = None
        self.last_prompt = now
        self._saw_end = None
        self._wheel_sent = None
        self.events = []

    # --- log and snapshot -------------------------------------------------

    def open_log(self, folder, run_id):
        """Start events.jsonl in the run folder. Truncates a file already there."""
        folder = Path(folder)
        folder.mkdir(parents=True, exist_ok=True)
        self.run_id = run_id
        self.log_path = folder / "events.jsonl"
        self.log_path.write_text("", encoding="utf-8")

    def _stamp(self):
        try:
            moment = float(self.wall())
        except (TypeError, ValueError):
            moment = 0.0
        return datetime.fromtimestamp(moment, ICT).isoformat(timespec="milliseconds")

    def _write(self, record):
        self.events.append(record)
        if len(self.events) > 200:
            self.events = self.events[-200:]
        path = self.log_path
        if path is None:
            return
        try:
            with open(path, "a", encoding="utf-8") as handle:
                handle.write(json.dumps(record, ensure_ascii=False) + "\n")
        except OSError:
            self.log_path = None

    def _log(self, event, state, code="", detail="", say="", visible=False, level="info", outcome=None):
        record = {
            "ts": self._stamp(),
            "run_id": self.run_id,
            "src": "hmc.flow",
            "state": state,
            "event": event,
            "code": code or "",
            "detail": detail or "",
            "visible": bool(visible),
            "say": say or "",
            "level": level,
            "outcome": outcome,
        }
        self._write(record)
        return record

    def _set_reason(self, code, detail=""):
        spec = REASONS[code]
        self.reason = {
            "code": code,
            "en": spec["en"],
            "th": spec["th"],
            "fix": spec["fix"],
            "detail": detail or "",
            "level": spec["level"],
        }
        self._log("reason", self.state, code=code, detail=detail, say=spec["en"],
                  visible=True, level=spec["level"])

    def snapshot(self, now=None):
        """The GET /api/state mission object. See base/docs/api-state.md."""
        with self._lock:
            now = self.last_now if now is None else now
            elapsed = 0.0 if self.t0 is None else max(0.0, now - self.t0)
            remaining = max(0.0, self.limit_s - elapsed)
            name = self._step_name()
            index = 0 if name == "IDLE" else STEPS.index(name) + 1
            return {
                "active": bool(self.running),
                "run_id": self.run_id,
                "state": self.state,
                "outcome": self.outcome,
                "step_index": index,
                "step_total": len(STEPS),
                "elapsed_s": round(elapsed, 1),
                "remaining_s": round(remaining, 1),
                "limit_s": self.limit_s,
                "reason": None if self.reason is None else dict(self.reason),
                "carrying": self.carrying,
                "car_pose": None if self.car_pose is None else dict(self.car_pose),
                "say": self.last_say,
                "side": self.side,
                "log": None if self.log_path is None else str(self.log_path),
            }

    def _step_name(self):
        if self.state in ("FAILED", "ABORTED") and self.anchor:
            return self.anchor
        if self.state == "LOST_SEARCH":
            return "FOLLOWING"
        if self.state == "HANDOVER":
            return "PICKING"
        if self.state in STEPS:
            return self.state
        return "IDLE"

    # --- public inputs ----------------------------------------------------

    def start(self, now, report=None, run_id=None, limit_s=None, top_speed=None):
        """Reset and start a run. report fails a precheck only when a key is False. top_speed (m/s) sets the pace
        the operator is asked to walk at once the robot follows."""
        with self._lock:
            keep_log, keep_id = self.log_path, self.run_id
            self._cleared(now)
            self.log_path, self.run_id = keep_log, run_id or keep_id
            if limit_s is not None:
                self.limit_s = float(limit_s)
            if top_speed:
                self.pace = hri.PACE_SHARE * float(top_speed)
            self.t0 = now
            self.running = True
            self.last_now = now
            if isinstance(report, dict) and "link" in report:
                self.link_ok = bool(report["link"])
            outs = self._enter("PRECHECK", now, say="Checking the base before we start.")
            code, detail = _precheck_code(report)
            if code:
                return outs + self._finish(now, code, detail)
            return outs + self._enter("READY", now, say="Say follow me when you are ready.")

    def stop(self, now):
        """Operator STOP: stop motion, record OPERATOR_STOP, do not drive home and do not stop Jetson programs."""
        with self._lock:
            self.last_now = now
            if self.state in TERMINAL or self.state == "IDLE" or not self.running:
                if self.reason is None or self.reason.get("code") != "OPERATOR_STOP":
                    self._set_reason("OPERATOR_STOP", "STOP while idle")
                self.running = False
                self.outcome = "ABORTED"
                if self.state not in TERMINAL:
                    self.anchor = self._step_name()
                    return self._enter("ABORTED", now, say=REASONS["OPERATOR_STOP"]["en"],
                                       cmds=[{"action": "stop", "params": {}}])
                return [("say", REASONS["OPERATOR_STOP"]["en"]), ("cmd", {"action": "stop", "params": {}})]
            return self._finish(now, "OPERATOR_STOP", "STOP", say=REASONS["OPERATOR_STOP"]["en"],
                                cmds=[{"action": "stop", "params": {}}])

    def on_heard(self, text, now):
        with self._lock:
            self.last_now = now
            if not self.running:
                return [("log", "(heard while no run is active)")]
            self.last_prompt = now
            kind, extra = classify(text)
            if kind is None:
                return [("log", "(unhandled speech)")]
            return self._heard(kind, extra, now)

    def on_event(self, ev, now):
        """A string 'name|CODE|detail' or a dict {name, code, detail, x, y, ...}."""
        with self._lock:
            self.last_now = now
            name, code, detail, extra = _parse_event(ev)
            if not self.running and name not in ("pose",):
                self._log("ignored", self.state, code=code, detail=name)
                return [("log", "(event %s ignored: run is not active)" % name)]
            return self._event(name, code, detail, extra, now)

    def on_status(self, status, now):
        with self._lock:
            self.last_now = now
            if not isinstance(status, dict):
                return []
            return self._status(status, now)

    def on_tick(self, now):
        with self._lock:
            self.last_now = now
            if not self.running:
                return []
            return self._tick(now)

    def on_button(self, name, now):
        """A console task button: follow, wait, resume, return, bag, bag_again, bag_near."""
        text = {
            "follow": "follow me",
            "wait": "wait",
            "resume": "continue",
            "return": "go back",
            "bag": "ready",
            "bag_again": "ready",
            "bag_near": "ready",
        }.get(name)
        if text is None:
            return [("log", "(unknown button)")]
        return self.on_heard(text, now)

    # --- transitions ------------------------------------------------------

    def _enter(self, state, now, say=None, cmds=None):
        prev = self.state
        if prev and prev != state:
            self._log("exit", prev)
        if state in ("FAILED", "ABORTED") and self.anchor is None:
            self.anchor = self._step_name() if prev not in ("FAILED", "ABORTED", "IDLE") else "PRECHECK"
        self.state = state
        self.entered_at = now
        self.deadline = self._deadline(state, now)
        if state == "FOLLOWING":
            self.left_start = True
            self.paused = False
        if state in ("AT_CAR", "CONFIRM_CAR", "POINTING", "READY", "HANDOVER"):
            self.last_prompt = now
        if state == "RETURNING":
            self.paused = False
            self.pause_asked = False
            if self.return_deadline is None:
                self.return_deadline = now + RETURN_BUDGET
        self._log("enter", state, say=say or "", code=(self.reason or {}).get("code", ""),
                  visible=bool(say), level=(self.reason or {}).get("level", "info"))
        outs = [("state", state)]
        if say:
            self.last_say = say
            outs.append(("say", say))
        for cmd in cmds or []:
            outs.append(("cmd", cmd))
        if state == "FOLLOWING" and not any(c.get("action") == "follow_me" for c in (cmds or [])):
            pass
        if state in TERMINAL:
            self.running = False
            self.outcome = (REASONS.get((self.reason or {}).get("code", ""), {}).get("outcome")
                            or ("SUCCESS" if state == "DONE" else "FAILED"))
            self._log("outcome", state, code=(self.reason or {}).get("code", ""),
                      detail=(self.reason or {}).get("detail", ""), say=say or "",
                      visible=True, level=(self.reason or {}).get("level", "info"), outcome=self.outcome)
            outs.append(("outcome", self.outcome))
        return outs

    def _deadline(self, state, now):
        limit = {
            "ENROLLING": ENROL_S,
            "LOST_SEARCH": LOST_SEARCH_S,
            "CONFIRM_CAR": CONFIRM_S,
            "POINTING": POINT_S,
            "PICKING": PICK_S,
            "HANDOVER": HANDOVER_S,
            "DROPPING": DROP_S,
        }.get(state)
        return None if limit is None else now + limit

    def _say_cmd(self, say, cmds):
        self.last_say = say
        return [("say", say)] + [("cmd", c) for c in cmds]

    def _finish(self, now, code, detail="", say=None, cmds=None):
        """End the run. A fault after we have left the start returns home first, except OPERATOR_STOP."""
        if code != "OPERATOR_STOP" and self._can_return():
            self._set_reason(code, detail)
            self.pending = code
            self.pending_detail = detail
            self.return_deadline = now + RETURN_BUDGET
            line = REASONS[code]["en"]
            return self._say_cmd(line, []) + self._enter(
                "RETURNING", now, say="I will go back to the start now.",
                cmds=[{"action": "go_back", "params": {}}])
        self._set_reason(code, detail)
        state = {"SUCCESS": "DONE", "ABORTED": "ABORTED"}.get(REASONS[code]["outcome"], "FAILED")
        line = say or REASONS[code]["en"]
        extra = list(cmds or [])
        if state == "DONE" and self.carrying and self.use_arm and self.state != "DROPPING":
            return self._enter("DROPPING", now, say=line, cmds=[{"action": "drop_bag", "params": {"x": 0.31, "y": 0.0}}])
        return self._enter(state, now, say=line, cmds=extra)

    def _can_return(self):
        return bool(self.running and self.left_start and self.state not in ("RETURNING", "DROPPING", *TERMINAL))

    def _heard(self, kind, extra, now):
        state = self.state
        if state == "READY" and kind == "follow":
            blocked = self._block_without_link("follow")
            if blocked is not None:
                return blocked
            return self._enter("ENROLLING", now, say="I will follow you. Please stand in front of me.",
                               cmds=[{"action": "follow_me", "params": {}}])
        if state == "ENROLLING" and kind == "follow":
            blocked = self._block_without_link("follow")
            if blocked is not None:
                return blocked
            self.enrol_tries = 0
            return self._say_cmd("I will look for you again.", [{"action": "follow_me", "params": {}}])
        if state == "FOLLOWING" and kind in ("arrive", "wait"):
            return self._confirm(now)
        if state == "LOST_SEARCH" and kind in ("arrive", "wait"):
            return self._confirm(now)
        if state == "FOLLOWING" and kind == "back":
            return self._enter("RETURNING", now, say="I will go back to the start now.",
                               cmds=[{"action": "go_back", "params": {}}])
        if state == "CONFIRM_CAR":
            if kind in ("yes", "arrive", "done"):
                return self._store_car(now)
            if kind in ("no", "follow"):
                return self._enter("FOLLOWING", now, say="I will keep following you.",
                                   cmds=[{"action": "resume", "params": {}}])
        if state in ("AT_CAR", "POINTING") and kind == "side":
            return self._point(now, extra or self.side)
        if state == "AT_CAR" and kind in ("done", "yes"):
            return self._point(now, self.side)
        if state == "AT_CAR" and kind == "follow":
            return self._enter("FOLLOWING", now, say="I will keep following you.",
                               cmds=[{"action": "resume", "params": {}}])
        if state == "POINTING" and kind == "follow":
            return self._enter("FOLLOWING", now, say="I will keep following you.",
                               cmds=[{"action": "resume", "params": {}}])
        if state == "HANDOVER" and kind in ("done", "yes"):
            self.carrying = True
            self.had_bag = True
            return self._enter("RETURNING", now, say="Thank you. I will go back to the start now.",
                               cmds=[{"action": "go_back", "params": {}}])
        if state == "RETURNING" and kind in ("wait", "arrive"):
            self.paused, self.paused_at, self.pause_asked = True, now, False
            return self._say_cmd("Okay, I will wait.", [{"action": "wait", "params": {}}])
        if state == "RETURNING" and kind in ("resume", "back", "done") and self.paused:
            self.paused = False
            return self._say_cmd("Going back to the start.", [{"action": "resume", "params": {}}])
        if state == "FOLLOWING" and kind == "follow":
            return [("say", "I am following you.")]
        return [("log", "(speech %s ignored in %s)" % (kind, state))]

    def _confirm(self, now):
        self.confirm_asks = 1
        self.last_prompt = now
        return self._enter("CONFIRM_CAR", now, say="Are we at the car?",
                           cmds=[{"action": "wait", "params": {}}])

    def _store_car(self, now):
        self.car_pose = dict(self.pose) if self.pose else {"x": None, "y": None, "heading": None}
        self._log("car_pose", "CONFIRM_CAR", detail=json.dumps(self.car_pose), visible=True)
        return self._enter("AT_CAR", now,
                           say="Please point at the bag. You can also say left or right.")

    def _block_without_link(self, detail):
        """None when the base link is known up. Otherwise no follow_me and no find_bag.

        An unknown link counts as down. Two refusals are spoken; after that the reason sticks
        and further tries are logged only, until base_link_ok.
        """
        if self.link_ok is True:
            self.link_tries = 0
            self.link_stuck = False
            return None
        if self.link_stuck:
            return [("log", "(BASE_LINK_DOWN)")]
        self.link_tries += 1
        self._set_reason("BASE_LINK_DOWN", detail)
        if self.link_tries >= LINK_ASKS:
            self.link_stuck = True
        self.last_say = REASONS["BASE_LINK_DOWN"]["en"]
        return [("say", self.last_say)]

    def _point(self, now, side):
        blocked = self._block_without_link("find_bag")
        if blocked is not None:
            return blocked
        self.side = side or self.side
        self.bag_tries = 0
        self.side_fallback_used = False
        params = {}
        if self.side:
            params["side"] = self.side
        line = "Please point at the bag."
        if self.side:
            line = "I will look to the %s." % self.side
        return self._enter("POINTING", now, say=line, cmds=[{"action": "find_bag", "params": params}])

    def _event(self, name, code, detail, extra, now):
        if name == "pose":
            self._remember_pose(extra or {})
            return []
        if name in ("locked", "see_person"):
            return self._locked(now)
        if name == "lost_person" and self.state == "FOLLOWING":
            self.come_back_said = False
            return self._enter("LOST_SEARCH", now, say="I lost you. I will look around.")
        if name in ("nobody_in_view", "dark_frame", "reid_fail", "ambiguous") and self.state == "ENROLLING":
            return self._enrol_fail(now, {
                "nobody_in_view": "NOBODY_IN_VIEW",
                "dark_frame": "DARK_FRAME",
                "reid_fail": "REID_FAIL",
                "ambiguous": "AMBIGUOUS",
            }[name], detail)
        if name in ("reid_down", "detector_down"):
            return self._finish(now, "REID_DOWN", detail)
        if name == "base_link_ok":
            self.link_ok = True
            self.link_lost_at = None
            self.link_tries = 0
            self.link_stuck = False
            if self.reason and self.reason.get("code") == "BASE_LINK_DOWN":
                self.reason = None
            return []
        if name == "base_link_lost":
            self.link_ok = False
            if self.link_lost_at is None:
                self.link_lost_at = now
            return [("log", "(base link lost)")]
        if name in ("odom_stale", "odometry"):
            return self._finish(now, "ODOM_STALE", detail)
        if name in ("wheel_fault", "wheels"):
            return self._wheels(now, code or detail)
        if name in ("base_run_ended", "pi_end"):
            return self._pi_end(now, code, detail)
        if name == "slow_cycle":
            return self._finish(now, "SLOW_CYCLE", detail)
        if name == "at_start" and self.state == "RETURNING":
            return self._arrived_home(now)
        if self.state == "POINTING" and name in ("bag_reached", "bag_failed", "bag_chosen"):
            return self._bag(name, code, detail, extra, now)
        if self.state == "PICKING" and name in ("grab_success", "grab_failed"):
            return self._grab(name, code, detail, now)
        if self.state == "HANDOVER" and name == "grab_success":
            self.carrying = True
            self.had_bag = True
            return self._enter("RETURNING", now, say="Thank you. I will go back to the start now.",
                               cmds=[{"action": "go_back", "params": {}}])
        if self.state == "DROPPING" and name == "drop_done":
            self.carrying = False
            return self._finish(now, "SUCCESS", "drop_done", say="Here is your bag.")
        if self.state == "DROPPING" and name == "drop_failed":
            mapped = "NOT_HOLDING" if code in ("NOT_HOLDING", "NOTHOLDING") else "DROP_FAILED"
            return self._finish(now, mapped, detail or code)
        if name == "arm_empty":
            self.carrying = False
            return []
        if name == "arm_holding":
            self.carrying = True
            self.had_bag = True
            return []
        self._log("ignored", self.state, code=code, detail=name)
        return [("log", "(event %s ignored in %s)" % (name, self.state))]

    def _locked(self, now):
        if self.state == "ENROLLING":
            out = self._enter("FOLLOWING", now, say="I will follow you.")
            return out + ([("say", hri.walk_slowly_text(self.pace))] if self.pace else [])
        if self.state == "LOST_SEARCH":
            self.come_back_said = False
            return self._enter("FOLLOWING", now, say="I see you again. I will follow you.")
        return []

    def _enrol_fail(self, now, code, detail):
        self.enrol_tries += 1
        if self.enrol_tries >= ENROL_TRIES:
            code = "ENROL_TIMEOUT"
        self._set_reason(code, detail)
        self.last_say = REASONS[code]["en"]
        self.deadline = now + ENROL_S
        return [("say", self.last_say)]

    def _remember_pose(self, pose):
        if isinstance(pose, (list, tuple)) and len(pose) >= 2:
            pose = {"x": pose[0], "y": pose[1], "heading": pose[2] if len(pose) > 2 else 0}
        if not isinstance(pose, dict) or pose.get("x") is None:
            return
        try:
            self.pose = {"x": float(pose["x"]), "y": float(pose["y"]),
                         "heading": float(pose.get("heading") or 0)}
        except (TypeError, ValueError):
            return

    def _wheels(self, now, text):
        low = (text or "").lower()
        if "02" in low:
            code = "WHEEL_FAULT_02"
        elif "03" in low:
            code = "WHEEL_FAULT_03"
        else:
            code = "WHEELS_NOT_TURNING"
        if self._wheel_sent == code and self.pending == code:
            return []
        self._wheel_sent = code
        return self._finish(now, code, text)

    def _pi_end(self, now, code, detail):
        if self._saw_end:
            return []
        self._saw_end = detail or code or "ended"
        if code in REASONS and code not in ("SUCCESS",):
            mapped = code
        else:
            mapped = code_from_pi_end(detail or code)
        if self.state in TERMINAL or not self.running or self.state == "DROPPING":  # the drop needs no base
            self._log("ignored", self.state, code=mapped, detail=self._saw_end)
            return [("log", "(base run ended in %s: expected)" % self.state)]
        return self._finish(now, mapped, self._saw_end)

    def _bag(self, name, code, detail, extra, now):
        if name == "bag_chosen":
            if detail:
                self.last_say = detail
                return [("say", detail)]
            return []
        if name == "bag_reached":
            return self._reached(detail, extra, now)
        mapped = _bag_code(code)
        if mapped == "NOT_POINTED" and self.side and not self.side_fallback_used:
            self.side_fallback_used = True
            self._set_reason("NOT_POINTED", detail or "pointing timed out")
            line = "No pointing was seen in 20 s. I will use the %s you said." % self.side
            self.last_say = line
            self.deadline = now + POINT_S
            return [("say", line), ("cmd", {"action": "find_bag", "params": {"side": self.side, "again": True}})]
        if mapped == "NO_LINK":
            self.link_ok = False
            return self._block_without_link(detail or "find_bag")
        if mapped in ("REFUSED", "CANCELLED", "BAG_REFUSED"):
            self._set_reason("BAG_REFUSED", detail or code)
            return self._enter("AT_CAR", now, say=REASONS["BAG_REFUSED"]["en"],
                               cmds=[{"action": "wait", "params": {}}])
        if mapped in RETRY_BAG and self.bag_tries < BAG_TRIES:
            self.bag_tries += 1
            if mapped == "TIMEOUT":
                mapped = "BAG_TIMEOUT"
            self._set_reason(mapped, detail)
            params = {"again": True}
            if self.side:
                params["side"] = self.side
            line = REASONS[mapped]["en"]
            if mapped == "NOT_POINTED":
                line = "Please step back two steps and point at the bag."
            self.deadline = now + POINT_S
            self.last_say = line
            return [("say", line), ("cmd", {"action": "find_bag", "params": params})]
        self._set_reason(mapped if mapped in REASONS else "BAG_TIMEOUT", detail)
        return self._handover(now)

    def _reached(self, detail, extra, now):
        nums = _numbers(detail)
        if extra:
            nums.setdefault("x", extra.get("x"))
            nums.setdefault("y", extra.get("y"))
        if nums.get("x") is None or nums.get("y") is None:
            self._set_reason("NO_BAG", detail or "bad bag pose")
            return self._enter("AT_CAR", now, say=REASONS["NO_BAG"]["en"])
        self.bag_at = (nums["x"], nums["y"])
        self.bag_top = nums.get("top", self.bag_top)
        self.bag_yaw = nums.get("yaw", self.bag_yaw)
        self.bag_tries = 0
        if not self.use_arm:
            self.carrying = True
            self.had_bag = True
            return self._say_cmd("I would pick up the bag now.", []) + self._enter(
                "RETURNING", now, say="I will go back to the start now.",
                cmds=[{"action": "go_back", "params": {}}])
        params = {"x": self.bag_at[0], "y": self.bag_at[1]}
        if self.bag_top is not None:
            params["top"] = self.bag_top
        if self.bag_yaw is not None:
            params["yaw"] = self.bag_yaw
        return self._enter("PICKING", now, say="I will pick up the bag.",
                           cmds=[{"action": "pick_bag", "params": params}])

    def _grab(self, name, code, detail, now):
        if name == "grab_success" or code == "HOLDING":
            self.carrying = True
            self.had_bag = True
            return self._enter("RETURNING", now, say="I will go back to the start now.",
                               cmds=[{"action": "go_back", "params": {}}])
        if code in REMEASURE or code == "PICK_MISSED":
            if not self.remeasured:
                self.remeasured = True
                self._set_reason("PICK_MISSED", detail)
                return self._enter("POINTING", now, say=REASONS["PICK_MISSED"]["en"],
                                   cmds=[{"action": "find_bag", "params": {"again": True}}])
            return self._handover(now, "PICK_FAILED", detail)
        if code in ("UNREACHABLE", "PICK_UNREACHABLE"):
            self._set_reason("PICK_UNREACHABLE", detail)
            return self._handover(now, "PICK_UNREACHABLE", detail)
        if code in ("TIMEOUT", "PICK_TIMEOUT"):
            self._set_reason("PICK_TIMEOUT", detail)
            return self._handover(now, "PICK_TIMEOUT", detail)
        self._set_reason("PICK_FAILED", detail or code)
        return self._handover(now, "PICK_FAILED", detail or code)

    def _handover(self, now, code="PICK_FAILED", detail=""):
        if code:
            self._set_reason(code, detail)
        return self._enter("HANDOVER", now, say="Please hand me the bag.")

    def _arrived_home(self, now):
        if self.pending:
            say = "I am back at the start."
            return self._finish(now, self.pending, self.pending_detail, say=say)
        if self.use_arm and self.had_bag and not self.carrying:
            return self._finish(now, "NOT_HOLDING", "empty at the start")
        if self.use_arm and self.carrying:
            return self._enter("DROPPING", now, say="I am back at the start.",
                               cmds=[{"action": "drop_bag", "params": {"x": 0.31, "y": 0.0}}])
        if self.carrying or self.had_bag:
            return self._finish(now, "SUCCESS", "home", say="Here is your bag.")
        return self._finish(now, "SUCCESS", "home without a bag", say="I am back at the start.")

    def _status(self, status, now):
        believed = status.get("pose")
        if believed is None:
            believed = status.get("believed")
        if believed is not None:
            self._remember_pose(believed)
        if not self.running:
            return []
        outs = []
        if "link" in status:
            if status.get("link"):
                self.link_ok = True
                self.link_lost_at = None
                self.link_tries = 0
                self.link_stuck = False
                if self.reason and self.reason.get("code") == "BASE_LINK_DOWN":
                    self.reason = None
            elif self.link_lost_at is None:
                self.link_ok = False
                self.link_lost_at = now
        searching = status.get("mode") == "search"  # hmc.follower Mode.SEARCH: the brain lost the operator, looks
        if status.get("locked") and not searching:
            outs += self._locked(now)
        if searching and self.state == "FOLLOWING":
            outs += self._event("lost_person", "", "", {}, now)
        if status.get("at_start") and self.state == "RETURNING":
            outs += self._arrived_home(now)
        wheels = status.get("wheels")
        if isinstance(wheels, str) and wheels:
            outs += self._wheels(now, wheels)
        end = status.get("end")
        if end and self.running:
            outs += self._pi_end(now, "", str(end))
        return outs

    def _tick(self, now):
        if self.t0 is not None and now - self.t0 >= self.limit_s and self.pending != "TIME_LIMIT":
            if self.state in ("RETURNING", "DROPPING"):
                self.pending = self.pending or "TIME_LIMIT"
                self._set_reason("TIME_LIMIT", "clock")
            else:
                return self._finish(now, "TIME_LIMIT", "clock")
        if self.pending and self.return_deadline is not None and now >= self.return_deadline \
                and self.state == "RETURNING":
            return self._finish(now, self.pending, self.pending_detail or "return budget")
        if self.link_lost_at is not None and now - self.link_lost_at >= LINK_GRACE_S \
                and self.state not in TERMINAL:
            self.link_lost_at = None
            if self.link_ok is False:
                return self._finish(now, "BASE_LINK_DOWN", "no link for %.0f s" % LINK_GRACE_S)
        state = self.state
        if state == "ENROLLING" and self.deadline is not None and now >= self.deadline:
            return self._enrol_fail(now, "NOBODY_IN_VIEW", "no lock")
        if state == "LOST_SEARCH" and not self.come_back_said and self.deadline is not None \
                and now >= self.deadline:
            self.come_back_said = True
            self._set_reason("OPERATOR_LOST", "search budget")  # the brain asks them back once (hri.COME_BACK_TEXT)
            return []
        if state == "CONFIRM_CAR" and now - self.last_prompt >= CONFIRM_S:
            self.confirm_asks += 1
            self.last_prompt = now
            if self.confirm_asks > CONFIRM_TRIES:
                self._set_reason("CAR_UNCONFIRMED", "no answer")
            self.last_say = "Are we at the car?"
            return [("say", "Are we at the car?")]
        if state == "AT_CAR" and now - self.last_prompt >= REASK_S:
            self.last_prompt = now
            self.last_say = "Please point at the bag. You can also say left or right."
            return [("say", self.last_say)]
        if state == "RETURNING" and self.paused and not self.pause_asked \
                and now - self.paused_at >= PAUSE_ASK_S:
            self.pause_asked = True
            self.last_say = "Say continue when I can go on."
            return [("say", self.last_say)]
        if state == "POINTING" and self.deadline is not None and now >= self.deadline:
            return self._bag("bag_failed", "TIMEOUT", "pointing deadline", {}, now)
        if state == "PICKING" and self.deadline is not None and now >= self.deadline:
            return self._grab("grab_failed", "TIMEOUT", "pick deadline", now)
        if state == "HANDOVER" and self.deadline is not None and now >= self.deadline:
            return self._finish(now, "PICK_FAILED", "handover timeout")
        if state == "DROPPING" and self.deadline is not None and now >= self.deadline:
            return self._finish(now, "DROP_TIMEOUT", "drop deadline")
        return []


def _bag_code(code):
    code = (code or "").upper()
    if code == "TIMEOUT":
        return "BAG_TIMEOUT"
    return code


def _numbers(text):
    """x,y from '0.31,0.01' or '0.31,0.01,top=0.2,yaw=0.4'."""
    out = {}
    if not text:
        return out
    parts = [p.strip() for p in str(text).split(",")]
    try:
        out["x"] = float(parts[0])
        out["y"] = float(parts[1])
    except (ValueError, IndexError):
        pass
    for part in parts[2:]:
        key, _, value = part.partition("=")
        try:
            out[key.strip()] = float(value)
        except ValueError:
            continue
    return out


def _parse_event(ev):
    if isinstance(ev, dict):
        return (str(ev.get("name") or ev.get("event") or "").lower(),
                str(ev.get("code") or "").upper(),
                str(ev.get("detail") or ""),
                ev)
    parts = str(ev).strip().split("|", 2)
    name = parts[0].strip().lower()
    code = parts[1].strip().upper() if len(parts) > 1 else ""
    detail = parts[2].strip() if len(parts) > 2 else ""
    return name, code, detail, {}
