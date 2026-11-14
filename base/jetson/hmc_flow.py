"""Help Me Carry task flow (decision D1 of docs/superpowers/specs/2026-11-14-hmc-integration-design.md).

Launched instead of voice's flow_node for this task, on the same topics (ROS 2 Jazzy, domain 42):

    stt_node --/speech/heard--> hmc_flow --/speech/text--> rule_node | llm_node --/robot/intent--> hmc_flow
    base_bridge, arm_action_bridge --/robot/event--> hmc_flow --/robot/command--> base_bridge, arm_action_bridge
    hmc_flow --/tts/say--> tts_node        hmc_flow --/flow/state--> (anyone)

The flow matches the task's own words in /speech/heard first ("follow me", "stop", "this is the car",
"go back", "done", ...). Only sentences it does not handle in the current state go on to the brain
on /speech/text, so one sentence never triggers twice. Intents from the brain are used as well.

States (spec "HmcFlow states"; slice 1 = the operator hands the bag over):
  ready      entry: say how to start. "follow me" -> following
  following  sent follow_me (Pi follow). lost_person (or no lock) for 5 s: say "Please come back." once.
             "stop" / "wait" / "this is the car" / "here" / "we are here" (or "go back") -> at_car
  at_car     sent wait (Pi wait). Say "Please put the bag in my gripper." (again after reask_s of silence).
             "done" / "okay" / "go back" -> handover. "follow me" -> following (Pi resume)
  handover   sent carry_bag {"handover": true}: the arm closes its gripper on the bag in it.
             grab_success -> say "I will go back to the start now.", go_back (Pi return) -> returning
             grab_failed / timeout -> error. "stop" -> at_car
             (use_arm false: "done" goes straight to returning, no arm involved)
  returning  the Pi does the breadcrumbs, queue and start heading. "stop"/"wait" -> wait (paused),
             "go back"/"continue" -> resume. at_start -> dropping_bag (no arm: say "Here is your bag." -> done)
  dropping_bag  sent drop_bag. drop_done -> say "Here is your bag." -> done. drop_failed / timeout -> error
  done       "follow me" starts a new run
  error      on base_link_lost from following / at_car / handover / returning (after at_start the Pi's run
             ends and its link with it, so a lost link in dropping_bag or done is only logged),
             or on an arm failure: Pi stop ("end", or error_pi_cmd), say why.
             "follow me" starts again; "go back" tries the return again.

Commands on /robot/command (JSON {"action","params","stamp","source"}):
  follow_me, wait, resume, go_back, end (Pi stop), stop  -> base_bridge
  carry_bag {"handover": true}, drop_bag                 -> arm_action_bridge

Run: source /opt/ros/jazzy/setup.bash && ROS_DOMAIN_ID=42 python3 hmc_flow.py [--ros-args -p use_arm:=true]
"""

import json
import re
import time

READY, FOLLOWING, AT_CAR, HANDOVER, RETURNING, DROPPING, DONE, ERROR = (
    "ready", "following", "at_car", "handover", "returning", "dropping_bag", "done", "error")
NEEDS_BASE = (FOLLOWING, AT_CAR, HANDOVER, RETURNING)  # a lost Pi link here is an error; not after at_start

# Other names teammates' nodes may send for the same event (copied from voice's flow_node).
EVENT_ALIASES = {"drop_success": "drop_done", "grab_done": "grab_success", "pick_success": "grab_success",
                 "pick_failed": "grab_failed", "person_seen": "see_person", "person_lost": "lost_person"}

NEG = re.compile(r"\b(don'?t|do not|never|not now)\b")
WORDS = [  # checked in this order; the first match wins
    ("stop", re.compile(r"\b(stop|halt|freeze|wait|stay)\b|\bthis is (the|my|your) car\b|\bwe'?re here\b"
                        r"|\bwe are here\b|\bhere\b")),
    ("follow", re.compile(r"\bfollow\b|\bcome with me\b")),
    ("back", re.compile(r"\b(go|take it|take the bag|bring it|bring the bag) back\b|\bgo to the start\b"
                        r"|\bgo home\b|\breturn\b")),
    ("resume", re.compile(r"\b(continue|resume|keep going|go on)\b")),
    ("done", re.compile(r"\b(done|okay|ok|finished|ready|go ahead|got it)\b")),
]
# Brain intents with the same meaning
INTENT_KINDS = {"follow_me": "follow", "stop": "stop", "wait": "stop", "go_back": "back",
                "resume": "resume", "carry_bag": "done"}


def classify(text):
    """The task word in a sentence: follow | stop | back | resume | done, or None."""
    low = text.lower().strip()
    if NEG.search(low):
        return None
    for kind, rx in WORDS:
        if rx.search(low):
            return kind
    return None


class HmcFlow:
    """Pure state machine (no ROS), unit-testable like voice's FlowLogic. Every method returns a list of
    ("say", text) | ("cmd", {"action", "params"}) | ("state", name) | ("text", sentence for the brain) | ("log", text)."""

    def __init__(self, use_arm=True, grab_timeout=30.0, drop_timeout=20.0, come_back_s=5.0, reask_s=15.0,
                 error_pi_cmd="end", start_text="Say follow me when you are ready.",
                 handover_text="Please put the bag in my gripper."):
        self.use_arm = use_arm
        self.grab_timeout, self.drop_timeout = grab_timeout, drop_timeout
        self.come_back_s, self.reask_s = come_back_s, reask_s
        self.error_pi_cmd = error_pi_cmd  # "end" = Pi stop as in the spec; "wait" keeps the Pi's run
        self.start_text, self.handover_text = start_text, handover_text
        self.state = None
        self.deadline = None  # arm timeout of the current state
        self.carrying = False
        self.link_ok = None  # None until base_bridge says anything
        self.person_visible = False
        self.lost_since = None  # following: when the operator was lost (None = seen)
        self.come_back_said = False
        self.last_prompt = 0.0  # at_car: when the handover line was said or someone spoke
        self.paused = False  # returning: halted by "stop"

    # ---------------- helpers ----------------
    @staticmethod
    def _cmd(action, **params):
        return ("cmd", {"action": action, "params": params})

    def _goto(self, state, now, msg=""):
        self.state = state
        self.deadline = None
        out = [("state", state)]
        if state == READY:
            out.append(("say", self.start_text))
        elif state == FOLLOWING:
            self.lost_since = None if self.person_visible else now
            self.come_back_said = False
            out.append(("say", "I will follow you."))
        elif state == AT_CAR:
            self.last_prompt = now
            out.append(("say", self.handover_text))
        elif state == HANDOVER:
            self.deadline = now + self.grab_timeout
            out.append(self._cmd("carry_bag", handover=True))
        elif state == RETURNING:
            self.paused = False
            out += [("say", "I will go back to the start now."), self._cmd("go_back")]
        elif state == DROPPING:
            self.deadline = now + self.drop_timeout
            out.append(self._cmd("drop_bag"))
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

    def _after_handover(self, now):
        self.carrying = True
        return self._goto(RETURNING, now)

    def _at_start(self, now):
        if self.use_arm:
            return self._goto(DROPPING, now)
        return self._goto(DONE, now)

    # ---------------- inputs ----------------
    def start(self, now):
        return self._goto(READY, now)

    def on_heard(self, text, now):
        """Raw speech. Handled here if it is a task word for this state, else forwarded to the brain."""
        self.last_prompt = max(self.last_prompt, now)  # someone is talking: don't nag
        kind = classify(text)
        out = self._handle(kind, now) if kind else None
        return out if out is not None else [("text", text)]

    def on_intent(self, intent, now):
        kind = INTENT_KINDS.get(intent.get("action", "none"))
        out = self._handle(kind, now) if kind else None
        return out if out is not None else [("log", "(intent %s ignored in state %s)" % (intent.get("action"), self.state))]

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
                return [self._cmd("wait")] + self._goto(AT_CAR, now)
            if kind == "follow":
                return [("say", "I'm following you right now.")]
            return None
        if s == AT_CAR:
            if kind in ("done", "back"):
                return self._goto(HANDOVER, now) if self.use_arm else self._after_handover(now)
            if kind in ("follow", "resume"):
                out = [self._cmd("resume")] + self._goto(FOLLOWING, now)
                return out
            if kind == "stop":
                return [self._cmd("wait")]
            return None
        if s == HANDOVER:
            if kind == "stop":
                return [self._cmd("stop"), ("say", "Okay, cancelled.")] + self._goto(AT_CAR, now)
            return [("say", "Please wait a moment.")]
        if s == RETURNING:
            if kind == "stop":
                self.paused = True
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
        # "name" or "name|ERROR_CODE|English message" (from arm_action_bridge / SO-100 API)
        parts = ev.strip().split("|", 2)
        name = EVENT_ALIASES.get(parts[0].strip().lower(), parts[0].strip().lower())
        code = parts[1].strip().upper() if len(parts) > 1 else ""
        arm_msg = parts[2].strip() if len(parts) > 2 else ""
        if name == "grab_failed" and code == "HOLDING":  # the arm already holds something: the bag
            name = "grab_success"
        s = self.state

        if name == "see_person":
            self.person_visible = True
            self.lost_since = None
            return []
        if name == "lost_person":
            self.person_visible = False
            if s == FOLLOWING and self.lost_since is None:
                self.lost_since = now
                self.come_back_said = False
            return []
        if name == "base_link_ok":
            self.link_ok = True
            return []
        if name == "base_link_lost":
            self.link_ok = False
            if s in NEEDS_BASE:
                return self._goto(ERROR, now, "I lost the connection to my wheels.")
            # after at_start the Pi's run ends and its link goes with it: not an error
            return [("log", "(base link lost in state %s: not needed now)" % s)]
        if name == "arm_holding":
            self.carrying = True
            return []
        if name == "arm_empty":
            self.carrying = False
            return []
        if name == "grab_success" and s == HANDOVER:
            return self._after_handover(now)
        if name == "grab_failed" and s == HANDOVER:
            return self._goto(ERROR, now, "Sorry. " + arm_msg if arm_msg else "Sorry, I could not hold the bag.")
        if name == "at_start" and s == RETURNING:
            return self._at_start(now)
        if name == "drop_done" and s == DROPPING:
            self.carrying = False
            return self._goto(DONE, now)
        if name == "drop_failed" and s == DROPPING:
            return self._goto(ERROR, now, "Sorry. " + arm_msg if arm_msg else "Sorry, I could not put the bag down.")
        return [("log", "(event '%s' ignored in state %s)" % (name, s))]

    def on_tick(self, now):
        s = self.state
        if s == FOLLOWING and self.lost_since is not None and not self.come_back_said \
                and now - self.lost_since >= self.come_back_s:
            self.come_back_said = True
            return [("say", "Please come back.")]
        if s == AT_CAR and now - self.last_prompt > self.reask_s:
            self.last_prompt = now
            return [("say", self.handover_text)]
        if s == HANDOVER and self.deadline is not None and now > self.deadline:
            return self._goto(ERROR, now, "Sorry, I could not hold the bag.")
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
            # false by default: arm_action_bridge ignores {"handover": true} and would run a full pick at
            # its fixed test point, where the operator's hand is. Turn on once the arm side handles handover.
            self.declare_parameter("use_arm", False)
            self.declare_parameter("grab_timeout", 30.0)
            self.declare_parameter("drop_timeout", 20.0)
            self.declare_parameter("come_back_s", 5.0)
            self.declare_parameter("reask_s", 15.0)
            # "wait" by default: a short Wi-Fi drop on the way back must not end the Pi's run (500 points)
            self.declare_parameter("error_pi_cmd", "wait")
            p = lambda n: self.get_parameter(n).value  # noqa: E731
            self.logic = HmcFlow(bool(p("use_arm")), float(p("grab_timeout")), float(p("drop_timeout")),
                                 float(p("come_back_s")), float(p("reask_s")), str(p("error_pi_cmd")))
            self.pub_text = self.create_publisher(String, "/speech/text", 10)
            self.pub_say = self.create_publisher(String, "/tts/say", 10)
            self.pub_cmd = self.create_publisher(String, "/robot/command", 10)
            self.pub_state = self.create_publisher(String, "/flow/state", 10)
            self.create_subscription(String, "/speech/heard", self.on_heard, 10)
            self.create_subscription(String, "/robot/intent", self.on_intent, 10)
            self.create_subscription(String, "/robot/event", self.on_event, 10)
            self.create_timer(0.2, self.on_tick)
            self.started = False

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

        def on_tick(self):
            now = time.time()
            if not self.started:  # wait until tts_node listens, so the first line is heard
                if self.pub_say.get_subscription_count() == 0:
                    return
                self.started = True
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
