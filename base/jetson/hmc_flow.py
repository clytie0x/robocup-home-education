"""Help Me Carry task flow (decision D1 of docs/superpowers/specs/2026-11-14-hmc-integration-design.md).

Launched instead of voice's flow_node for this task, on the same topics (ROS 2 Jazzy, domain 42):

    stt_node --/speech/heard--> hmc_flow --/speech/text--> rule_node | llm_node --/robot/intent--> hmc_flow
    base_bridge, arm_action_bridge --/robot/event--> hmc_flow --/robot/command--> base_bridge, arm_action_bridge
    hmc_flow --/tts/say--> tts_node        hmc_flow --/flow/state--> (anyone)

The flow matches the task's own words in /speech/heard first ("follow me", "stop", "this is the car",
"go back", "done", ...). Only sentences it does not handle in the current state go on to the brain
on /speech/text, so one sentence never triggers twice. Intents from the brain are used as well.

Start: the flow enters ready once tts_node subscribes to /tts/say (so the first line is heard), or after
START_WAIT s without it (warning logged; lines are still published), and once one /base/status was read
(at most STATUS_WAIT s: link and person as the Pi has them, so a restarted flow does not take the person
for lost), or at the first /speech/heard or /robot/intent, which is then handled in ready. So the base
obeys even with no voice. A /robot/event before that (base_bridge sends base_link_ok on connect) is only
noted (link, person) and logged.

Words: "here" counts only in "we are here", "we're here", "here we are" ("this is the car" too); "wait"
and "okay"/"ok" only in a sentence of at most SHORT (3) words ("wait here", "please wait"); a negation
("don't", "never", ...) cancels only a word after it in the same clause, so "I don't know, stop" is stop.

States (spec "HmcFlow states"; the robot finds and approaches the pointed bag, the arm picks it):
  ready      entry: say how to start. "follow me" -> following
  following  sent follow_me (Pi follow). Lost: say "Please come back." once, when /base/status "search"
             (base_bridge) ended without a lock, or after come_back_s (20 s) lost (each ask risks -150).
             "stop" / "wait" / "this is the car" / "we are here" (or "go back") -> wait (Pi wait),
             then pointing
  pointing   say "Please point at the bag." and send find_bag (Pi BAG: base/pi/bag.py finds the pointing,
             chooses the bag, turns and creeps up to it). bag_chosen -> say its text (which bag, which side).
             bag_reached x,y -> picking (no arm: say what it would do, then returning).
             bag_failed with NOT_POINTED, NO_BAG, BLOCKED, TIMEOUT or LOST_BAG -> say a short reason and
             "Please point at the bag again." and send find_bag again (MAX_BAG_TRIES in a row, then at_car);
             NOT_POINTED first says "Please step back two steps and point at the bag." (the camera sees
             the shoulder only from about 2.2 m);
             any other bag_failed (REFUSED, NO_LINK, CANCELLED) or no answer in bag_timeout -> at_car.
             "stop" -> wait (cancels the Pi's bag task) -> at_car. "follow me" -> following (Pi resume).
  at_car     standing at the car, no bag task: say "Please point at the bag, then say ready." (again after
             reask_s of silence). "ready" / "done" / "okay" / "take it back" -> pointing.
             "follow me" -> following (Pi resume)
  picking    sent pick_bag {"x", "y"} (robot frame, from bag_reached) to the arm, timeout grab_timeout (90 s).
             grab_success (or grab_failed HOLDING) -> returning. grab_failed MISSED or NOT_FOUND, the first
             time: find_bag {"again": true} (the Pi measures and approaches the same bag once more) ->
             pointing, whose bag_reached picks again. Other grab_failed, a second miss, or timeout -> error.
             "stop" -> stop (the arm holds) -> at_car
  returning  say "I will go back to the start now.", go_back: the Pi does the breadcrumbs (the drive to the
             bag counts as path), queue and start heading. "stop"/"wait" -> wait (paused),
             "go back"/"continue" -> resume; after RESUME_AFTER_S (20 s) it only asks (never moves by itself), "Going back to
             the start.". at_start -> dropping_bag (no arm: say "Here is your bag." -> done)
  dropping_bag  sent drop_bag {"x": 0.31, "y": 0.0}. drop_done -> say "Here is your bag." -> done.
             drop_failed / timeout -> error
  done       "follow me" starts a new run
  error      on base_link_lost from following / pointing / at_car / picking / returning with no base_link_ok
             in LINK_GRACE_S (4 s: base_bridge restores the Pi's intent itself; while waiting the flow says
             and sends nothing). After at_start the Pi's run ends and its link with it, so a lost link in
             dropping_bag or done is only logged. Or on an arm failure: Pi stop ("end", or error_pi_cmd), say why.
             "follow me" starts again; "go back" tries the return again.

Commands on /robot/command (JSON {"action","params","stamp","source"}):
  follow_me, wait, resume, go_back, end (Pi stop), stop, find_bag {"again"} -> base_bridge
  pick_bag {"x", "y"}, drop_bag {"x", "y"}, stop                          -> bag_arm (arm_action_bridge)
Events read: see_person, lost_person, base_link_ok, base_link_lost, at_start, bag_chosen|SIDE|text,
  bag_reached|CODE|x,y, bag_failed|CODE|message (base_bridge); grab_success, grab_failed|CODE|message,
  drop_done, drop_failed|CODE|message (the arm). /base/status (base_bridge): "link", "locked", "search".

Run: source /opt/ros/jazzy/setup.bash && ROS_DOMAIN_ID=42 python3 hmc_flow.py [--ros-args -p use_arm:=true]
"""

import json
import re
import time

READY, FOLLOWING, POINTING, AT_CAR, PICKING, RETURNING, DROPPING, DONE, ERROR = (
    "ready", "following", "pointing", "at_car", "picking", "returning", "dropping_bag", "done", "error")
NEEDS_BASE = (FOLLOWING, POINTING, AT_CAR, PICKING, RETURNING)  # a lost Pi link here is an error; not after at_start
DROP_AT = {"x": 0.31, "y": 0.0}  # m, robot frame: where the arm puts the bag down at the start
MAX_BAG_TRIES = 3  # bag_failed answered with "point again" this many times in a row, then at_car
RETRY_CODES = ("NOT_POINTED", "NO_BAG", "BLOCKED", "TIMEOUT", "LOST_BAG")  # the Pi looked: point again
REMEASURE_CODES = ("MISSED", "NOT_FOUND")  # a grab that missed: the Pi measures the bag once more
BAG_REASONS = {"NOT_POINTED": "I did not see you point.", "NO_BAG": "I see no bag there.",
               "BLOCKED": "Something is in my way.", "TIMEOUT": "That took me too long.",
               "LOST_BAG": "I lost sight of the bag."}
# s to wait for tts_node on /tts/say before starting anyway: it may start late, crash or be absent,
# and the base must still be commandable
START_WAIT = 3.0
STATUS_WAIT = 1.0  # s to wait at start for one /base/status (link and person as the Pi has them)
# s a lost Pi link may last before the flow gives up (error): base_bridge reconnects and restores the
# Pi's intent itself, so a short drop needs nothing from the flow
LINK_GRACE_S = 4.0
RESUME_AFTER_S = 20.0  # s into a "stop"/"wait" pause on the way back, the robot asks to be told to go on
COME_BACK_TEXT = "Please come back."
STEP_BACK_TEXT = "Please step back two steps and point at the bag."  # the camera sees the shoulder from about 2.2 m

# Other names teammates' nodes may send for the same event (copied from voice's flow_node).
EVENT_ALIASES = {"drop_success": "drop_done", "grab_done": "grab_success", "pick_success": "grab_success",
                 "pick_failed": "grab_failed", "person_seen": "see_person", "person_lost": "lost_person"}

NEG = re.compile(r"\b(don'?t|do not|never|not now)\b")
CLAUSE = re.compile(r"[,.!?;]+")  # a negation counts only before its word in the same clause
WORDS = [  # checked in this order; the first match wins
    # "here" alone is no arrival: "come here", "over here", "this one here" cancelled the bag search
    ("stop", re.compile(r"\b(stop|halt|freeze|stay)\b|\bthis is (the|my|your) car\b|\bwe'?re here\b"
                        r"|\bwe are here\b|\bhere we are\b")),
    ("follow", re.compile(r"\bfollow\b|\bcome with me\b")),
    ("back", re.compile(r"\b(go|take it|take the bag|bring it|bring the bag) back\b|\bgo to the start\b"
                        r"|\bgo home\b|\breturn\b")),
    ("resume", re.compile(r"\b(continue|resume|keep going|go on)\b")),
    ("done", re.compile(r"\b(done|finished|ready|go ahead|got it)\b")),
]
# Words that count only in a short command ("wait", "wait here", "please wait", "okay"), not inside a
# longer sentence ("you have to wait for the bag", "okay I can see the bag over there")
SHORT_WORDS = {"stop": re.compile(r"\bwait\b"), "done": re.compile(r"\b(okay|ok)\b")}
# On the way back "wait" anywhere pauses (review 2026-11-15: "robot please wait a moment" drove on); there a
# stop only pauses, while in following it would mean "this is the car"
WAIT_ANYWHERE = re.compile(r"\bwait\b")
SHORT = 3  # words at most in a short command
# Brain intents with the same meaning
INTENT_KINDS = {"follow_me": "follow", "stop": "stop", "wait": "stop", "go_back": "back",
                "resume": "resume", "carry_bag": "done"}


def _negated(low, start):
    """A negation before the word at start, in its clause: "don't stop" is, "I don't know, stop" is not."""
    return bool(NEG.search(CLAUSE.split(low[:start])[-1]))


def classify(text):
    """The task word in a sentence: follow | stop | back | resume | done, or None."""
    low = text.lower().strip()
    short = len(re.findall(r"[a-z']+", low)) <= SHORT
    for kind, rx in WORDS:
        rxs = [rx, SHORT_WORDS[kind]] if short and kind in SHORT_WORDS else [rx]
        if any(not _negated(low, m.start()) for r in rxs for m in r.finditer(low)):
            return kind
    return None


def should_start(listeners, waited_s, wait_s=START_WAIT, status_seen=True, status_wait_s=STATUS_WAIT):
    """The node starts the flow once someone listens on /tts/say (or after wait_s without a listener) and a
    /base/status was read (or after status_wait_s without one)."""
    return (listeners > 0 or waited_s >= wait_s) and (status_seen or waited_s >= status_wait_s)


class HmcFlow:
    """Pure state machine (no ROS), unit-testable like voice's FlowLogic. Every method returns a list of
    ("say", text) | ("cmd", {"action", "params"}) | ("state", name) | ("text", sentence for the brain) | ("log", text)."""

    def __init__(self, use_arm=True, grab_timeout=90.0, drop_timeout=20.0, come_back_s=20.0, reask_s=15.0,
                 error_pi_cmd="end", start_text="Say follow me when you are ready.",
                 point_text="Please point at the bag.", bag_timeout=180.0):
        self.use_arm = use_arm
        self.grab_timeout, self.drop_timeout = grab_timeout, drop_timeout  # grab: the arm's pick (slow on purpose)
        # come_back_s: lost this long, ask the operator back even if the Pi's search has not ended (each ask
        # risks -150 in the rules, so not after a short gap)
        self.come_back_s, self.reask_s = come_back_s, reask_s
        self.error_pi_cmd = error_pi_cmd  # "end" = Pi stop as in the spec; "wait" keeps the Pi's run
        self.start_text, self.point_text = start_text, point_text
        self.ready_text = "Please point at the bag, then say ready."
        self.bag_timeout = bag_timeout  # s without a bag result: longer than the Pi's own limits (20 s + 120 s)
        self.bag_tries = 0  # bag_failed in a row answered with find_bag again
        self.remeasured = False  # this bag was measured again after a missed grab
        self.bag_at = None  # (x, y) robot frame from the last bag_reached
        self.state = None
        self.deadline = None  # arm timeout of the current state
        self.carrying = False
        self.link_ok = None  # None until base_bridge says anything
        self.person_visible = False
        self.lost_since = None  # following: when the operator was lost (None = seen)
        self.come_back_said = False
        self.pi_searched = False  # following: /base/status said the Pi searches since the person was last seen
        self.status_seen = False  # a /base/status was read
        self.link_lost_at = None  # when base_link_lost came, until base_link_ok
        self.last_prompt = 0.0  # at_car: when the ready line was said or someone spoke
        self.paused = False  # returning: halted by "stop"
        self.paused_at = None
        self.pause_asked = False  # returning: "Say continue ..." said for this pause

    # ---------------- helpers ----------------
    @staticmethod
    def _cmd(action, **params):
        return ("cmd", {"action": action, "params": params})

    def _goto(self, state, now, msg="", again=False):
        self.state = state
        self.deadline = None
        out = [("state", state)]
        if state == READY:
            out.append(("say", self.start_text))
        elif state == FOLLOWING:
            self.lost_since = None if self.person_visible else now
            self.come_back_said = self.pi_searched = False
            out.append(("say", "I will follow you."))
        elif state == AT_CAR:
            self.last_prompt = now
            out.append(("say", (msg + " " if msg else "") + self.ready_text))
        elif state == POINTING:
            self.deadline = now + self.bag_timeout
            out.append(("say", msg or self.point_text))
            out.append(self._cmd("find_bag", again=True) if again else self._cmd("find_bag"))
        elif state == PICKING:
            self.deadline = now + self.grab_timeout
            out.append(self._cmd("pick_bag", x=self.bag_at[0], y=self.bag_at[1]))
        elif state == RETURNING:
            self.paused = False
            out += [("say", "I will go back to the start now."), self._cmd("go_back")]
        elif state == DROPPING:
            self.deadline = now + self.drop_timeout
            out.append(self._cmd("drop_bag", **DROP_AT))
        elif state == DONE:
            out.append(("say", msg or "Here is your bag."))
        elif state == ERROR:
            out += [self._cmd(self.error_pi_cmd),
                    ("say", (msg or "Sorry, something went wrong.") + " Say follow me to start again.")]
        return out

    def _start_following(self, now):
        if self.link_ok is False:
            return [("say", "I cannot reach my wheels. Please check the base.")]
        return [self._cmd("follow_me")] + self._goto(FOLLOWING, now)

    def _after_pick(self, now):
        self.carrying = True
        return self._goto(RETURNING, now)

    def _point(self, now, msg=""):
        """pointing afresh: from following or at_car (not a retry)."""
        self.bag_tries, self.remeasured = 0, False
        return self._goto(POINTING, now, msg)

    def _at_start(self, now):
        if self.use_arm:
            return self._goto(DROPPING, now)
        return self._goto(DONE, now)

    # ---------------- inputs ----------------
    def start(self, now):
        return self._goto(READY, now)

    def _start_first(self, now):
        """Input before start(): enter ready now so it is handled, not lost. Its outputs come first."""
        return self.start(now) if self.state is None else []

    def on_heard(self, text, now):
        """Raw speech. Handled here if it is a task word for this state, else forwarded to the brain."""
        first = self._start_first(now)
        self.last_prompt = max(self.last_prompt, now)  # someone is talking: don't nag
        kind = classify(text)
        if kind is None and self.state == RETURNING:
            low = text.lower()
            if any(not _negated(low, m.start()) for m in WAIT_ANYWHERE.finditer(low)):
                kind = "stop"  # on the way back a long "wait" pauses (WAIT_ANYWHERE)
        out = self._handle(kind, now) if kind else None
        return first + (out if out is not None else [("text", text)])

    def on_intent(self, intent, now):
        first = self._start_first(now)
        kind = INTENT_KINDS.get(intent.get("action", "none"))
        out = self._handle(kind, now) if kind else None
        return first + (out if out is not None
                        else [("log", "(intent %s ignored in state %s)" % (intent.get("action"), self.state))])

    def _handle(self, kind, now):
        """Outputs for a task word, or None when the word means nothing in this state."""
        s = self.state
        if s in (READY, DONE, ERROR):
            if kind == "follow":
                return self._start_following(now)
            if kind == "stop":
                return [self._cmd("stop")]
            if kind == "back" and s == ERROR:
                return self._goto(RETURNING, now)
            return None
        if s == FOLLOWING:
            if kind in ("stop", "back"):
                return [self._cmd("wait")] + self._point(now)
            if kind == "follow":
                return [("say", "I'm following you right now.")]
            return None
        if s == POINTING:
            if kind == "stop":  # wait cancels the Pi's bag task and holds it
                return [self._cmd("wait")] + self._goto(AT_CAR, now, "Okay, cancelled.")
            if kind in ("follow", "resume"):
                return [self._cmd("resume")] + self._goto(FOLLOWING, now)
            return [("say", self.point_text)]  # "take it back", "done": the bag first
        if s == AT_CAR:
            if kind in ("done", "back"):
                return self._point(now)
            if kind in ("follow", "resume"):
                return [self._cmd("resume")] + self._goto(FOLLOWING, now)
            if kind == "stop":
                return [self._cmd("wait")]
            return None
        if s == PICKING:
            if kind == "stop":
                return [self._cmd("stop"), ("say", "Okay, cancelled.")] + self._goto(AT_CAR, now)
            return [("say", "Please wait a moment.")]
        if s == RETURNING:
            if kind == "stop":
                self.paused, self.paused_at, self.pause_asked = True, now, False
                return [self._cmd("wait"), ("say", "Okay, I will wait.")]
            if kind in ("back", "resume", "done") and self.paused:
                self.paused = False
                return [self._cmd("resume"), ("say", "Going back to the start.")]
            if kind in ("back", "resume"):
                return [("say", "I'm going back to the start right now.")]
            return None
        if s == DROPPING:
            if kind == "stop":
                return [self._cmd("stop")] + self._goto(DONE, now, "Okay, I stopped. Please take your bag.")
            return [("say", "Please wait a moment.")]
        return None

    def on_event(self, ev, now):
        if self.state is None:  # base_bridge connects before tts_node: note it (link_ok, ...), don't start
            self._event(ev, now)
            return [("log", "(event '%s' before start: noted)" % ev.strip())]
        return self._event(ev, now)

    def _event(self, ev, now):
        # "name" or "name|ERROR_CODE|English message" (from arm_action_bridge / SO-100 API)
        parts = ev.strip().split("|", 2)
        name = EVENT_ALIASES.get(parts[0].strip().lower(), parts[0].strip().lower())
        code = parts[1].strip().upper() if len(parts) > 1 else ""
        arm_msg = parts[2].strip() if len(parts) > 2 else ""
        # bag_arm appends the grasp reading ("gripper_pos=R gripper_load=L", maybe in brackets) for the log: never said
        arm_msg = re.sub(r"\s*\[?gripper_pos=\S+ gripper_load=[^\]\s]+\]?", "", arm_msg).strip()
        if name == "grab_failed" and code == "HOLDING":  # the arm already holds something: the bag
            name = "grab_success"
        s = self.state

        if name == "see_person":
            self.person_visible = True
            self.lost_since = None
            self.pi_searched = False
            return []
        if name == "lost_person":
            self.person_visible = False
            if s == FOLLOWING and self.lost_since is None:
                self.lost_since = now
                self.come_back_said = False
            return []
        if name == "base_link_ok":
            self.link_ok = True
            self.link_lost_at = None
            return []
        if name == "base_link_lost":
            self.link_ok = False
            if self.link_lost_at is None:
                self.link_lost_at = now
            if s in NEEDS_BASE:  # on_tick: an error only if it is not back in LINK_GRACE_S
                return [("log", "(base link lost in state %s: waiting %.0f s for it)" % (s, LINK_GRACE_S))]
            # after at_start the Pi's run ends and its link goes with it: not an error
            return [("log", "(base link lost in state %s: not needed now)" % s)]
        if name == "arm_holding":
            self.carrying = True
            return []
        if name == "arm_empty":
            self.carrying = False
            return []
        if s == POINTING and name in ("bag_chosen", "bag_reached", "bag_failed"):
            return self._bag_event(name, code, arm_msg, now)
        if name == "grab_success" and s == PICKING:
            return self._after_pick(now)
        if name == "grab_failed" and s == PICKING:
            if code in REMEASURE_CODES and not self.remeasured:
                self.remeasured = True
                return self._goto(POINTING, now, "I missed the bag. Let me look at it again.", again=True)
            return self._goto(ERROR, now, "Sorry. " + arm_msg if arm_msg else "Sorry, I could not pick up the bag.")
        if name == "at_start" and s == RETURNING:
            return self._at_start(now)
        if name == "drop_done" and s == DROPPING:
            self.carrying = False
            return self._goto(DONE, now)
        if name == "drop_failed" and s == DROPPING:
            return self._goto(ERROR, now, "Sorry. " + arm_msg if arm_msg else "Sorry, I could not put the bag down.")
        return [("log", "(event '%s' ignored in state %s)" % (name, s))]

    def _bag_event(self, name, code, text, now):
        """bag_chosen, bag_reached or bag_failed from base_bridge while pointing."""
        if name == "bag_chosen":
            return [("say", text)] if text else []
        if name == "bag_reached":
            try:
                x, y = (float(v) for v in text.split(","))
            except ValueError:
                return [self._cmd("wait")] + self._goto(
                    AT_CAR, now, "Sorry, I could not find the bag.")
            self.bag_at, self.bag_tries = (x, y), 0
            if self.use_arm:
                return self._goto(PICKING, now)
            return [("say", "I would pick up the bag now.")] + self._after_pick(now)
        first = [("say", STEP_BACK_TEXT)] if code == "NOT_POINTED" else []  # too close for the camera to see it
        if code in RETRY_CODES and self.bag_tries < MAX_BAG_TRIES:
            self.bag_tries += 1
            return first + self._goto(POINTING, now, "%s Please point at the bag again." % BAG_REASONS[code])
        return first + self._goto(AT_CAR, now, BAG_REASONS.get(code, "Sorry, I could not look for the bag."))

    def on_status(self, status, now):
        """One /base/status from base_bridge (also before start). The first one sets link_ok and person_visible:
        the bridge sends base_link_ok and see_person only on a change, so a restarted flow would never hear
        them. While following, a Pi search that ended without a lock asks the operator back."""
        out = []
        if not self.status_seen:
            self.status_seen = True
            self.link_ok = bool(status.get("link"))
            self.person_visible = self.link_ok and bool(status.get("locked"))
            if self.person_visible:
                self.lost_since = None
            out.append(("log", "(first /base/status: link %s, person %s)" % (self.link_ok, self.person_visible)))
        if self.state != FOLLOWING or self.link_lost_at is not None:
            return out
        if status.get("search"):
            self.pi_searched = True
        elif self.pi_searched and not status.get("locked") and self.lost_since is not None \
                and not self.come_back_said:
            self.pi_searched, self.come_back_said = False, True
            out.append(("say", COME_BACK_TEXT))
        return out

    def on_tick(self, now):
        s = self.state
        if self.link_lost_at is not None and s in NEEDS_BASE:
            if now - self.link_lost_at >= LINK_GRACE_S:
                return self._goto(ERROR, now, "I lost the connection to my wheels.")
            return []  # the bridge restores the Pi's intent itself: say nothing, send nothing
        if s == FOLLOWING and self.lost_since is not None and not self.come_back_said \
                and now - self.lost_since >= self.come_back_s:
            self.come_back_said = True
            return [("say", COME_BACK_TEXT)]
        if s == RETURNING and self.paused and not self.pause_asked and now - self.paused_at >= RESUME_AFTER_S:
            self.pause_asked = True  # never moves by itself: the stop may be for the cable or a foot (review)
            return [("say", "Say continue when I can go on.")]
        if s == AT_CAR and now - self.last_prompt > self.reask_s:
            self.last_prompt = now
            return [("say", self.ready_text)]
        if s == POINTING and self.deadline is not None and now > self.deadline:  # no answer: cancel the Pi's task
            return [self._cmd("wait")] + self._goto(AT_CAR, now, "Sorry, that took me too long.")
        if s == PICKING and self.deadline is not None and now > self.deadline:
            return self._goto(ERROR, now, "Sorry, I could not pick up the bag.")
        if s == DROPPING and self.deadline is not None and now > self.deadline:
            return self._goto(ERROR, now, "Sorry, I could not put the bag down.")
        return []


def make_node():
    """The rclpy node, built here so the rest of the file imports without ROS."""
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String

    class HmcFlowNode(Node):
        def __init__(self):
            super().__init__("hmc_flow")
            # false by default: the arm is the arm side's (bag_arm.py takes pick_bag and drop_bag); turn on
            # when it runs. Off, bag_reached goes straight to the return.
            self.declare_parameter("use_arm", False)
            self.declare_parameter("grab_timeout", 90.0)  # pick_bag: the arm moves slowly (correctness over speed)
            self.declare_parameter("drop_timeout", 20.0)
            self.declare_parameter("come_back_s", 20.0)
            self.declare_parameter("reask_s", 15.0)
            # "wait" by default: a short Wi-Fi drop on the way back must not end the Pi's run (500 points)
            self.declare_parameter("error_pi_cmd", "wait")
            self.declare_parameter("bag_timeout", 180.0)
            p = lambda n: self.get_parameter(n).value  # noqa: E731
            self.logic = HmcFlow(bool(p("use_arm")), float(p("grab_timeout")), float(p("drop_timeout")),
                                 float(p("come_back_s")), float(p("reask_s")), str(p("error_pi_cmd")),
                                 bag_timeout=float(p("bag_timeout")))
            self.pub_text = self.create_publisher(String, "/speech/text", 10)
            self.pub_say = self.create_publisher(String, "/tts/say", 10)
            self.pub_cmd = self.create_publisher(String, "/robot/command", 10)
            self.pub_state = self.create_publisher(String, "/flow/state", 10)
            self.create_subscription(String, "/speech/heard", self.on_heard, 10)
            self.create_subscription(String, "/robot/intent", self.on_intent, 10)
            self.create_subscription(String, "/robot/event", self.on_event, 10)
            self.create_subscription(String, "/base/status", self.on_status, 10)
            self.create_timer(0.2, self.on_tick)
            self.t0 = time.time()

        def emit(self, outputs):
            for kind, val in outputs:
                if kind == "say":
                    self.get_logger().info('say: "%s"' % val)
                    self.pub_say.publish(String(data=val))
                elif kind == "cmd":
                    self.get_logger().info("-> robot: %s %s" % (val["action"], val.get("params") or ""))
                    self.pub_cmd.publish(String(data=json.dumps(dict(val, stamp=time.time(), source="hmc_flow"))))
                elif kind == "state":
                    self.get_logger().info("== state: %s ==" % val)
                    self.pub_state.publish(String(data=val))
                elif kind == "text":
                    self.pub_text.publish(String(data=val))
                else:
                    self.get_logger().info(val)

        def on_heard(self, msg):
            self.emit(self.logic.on_heard(msg.data, time.time()))

        def on_intent(self, msg):
            try:
                intent = json.loads(msg.data)
            except json.JSONDecodeError:
                self.get_logger().warn("bad intent: %s" % msg.data)
                return
            self.emit(self.logic.on_intent(intent, time.time()))

        def on_event(self, msg):
            if msg.data.strip().lower() not in ("see_person", "lost_person"):
                self.get_logger().info("event: %s" % msg.data)
            self.emit(self.logic.on_event(msg.data, time.time()))

        def on_status(self, msg):
            try:
                status = json.loads(msg.data)
            except json.JSONDecodeError:
                return
            if isinstance(status, dict):
                self.emit(self.logic.on_status(status, time.time()))

        def on_tick(self):
            now = time.time()
            if self.logic.state is None:  # wait for tts_node (first line heard) and one /base/status
                listeners = self.pub_say.get_subscription_count()
                if not should_start(listeners, now - self.t0, status_seen=self.logic.status_seen):
                    return
                if listeners == 0:
                    self.get_logger().warn("no tts_node on /tts/say after %.0f s: starting without voice"
                                           % START_WAIT)
                if not self.logic.status_seen:
                    self.get_logger().warn("no /base/status after %.0f s (base_bridge down?): starting without it"
                                           % STATUS_WAIT)
                self.emit(self.logic.start(now))
                return
            self.emit(self.logic.on_tick(now))

    rclpy.init()
    return HmcFlowNode()


def main():
    import rclpy

    node = make_node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
