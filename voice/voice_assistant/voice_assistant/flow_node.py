"""Flow node: the robot's task state machine (team state chart, corrected), no wake word.

    stt_node --/speech/heard--> flow_node --/speech/text--> rule_node | llm_node
                                    ^                              |
                                    +-------- /robot/intent -------+   (what the user wants)
    teammates' nodes --/robot/event--> flow_node --/robot/command--> command_bridge_node / robot
                                    flow_node --/tts/say, /tts/play--> tts_node

States
  LISTENING        entry: ask "How can I help you?"; no command for `reask_s` -> ask again
  PICKING_BAG      sent carry_bag, waiting for event grab_success
  ANNOUNCE_READY   entry: say "I am ready to follow you"; when the speech ends and a person is seen -> FOLLOWING
  FOLLOWING        sent follow_me; stop -> LISTENING, drop the bag -> DROPPING_BAG
  DROPPING_BAG     sent drop_bag, waiting for event drop_done
  ERROR            entry: say what went wrong; when the speech ends -> LISTENING

Events on /robot/event (std_msgs/String), sent by the robot side:
  grab_success, grab_failed, drop_done, drop_failed, see_person, lost_person, bag_lost
  optionally with the arm's reason: "grab_failed|UNREACHABLE|I can't reach it." -> the robot says the message
  arm_holding / arm_empty (from /arm/state via arm_action_bridge) keep "carrying" in sync with the real gripper

Fixes vs. the original drawing
  1. following + "not see the bag" -> ERROR ("I think I dropped the bag"), not DROPPING_BAG
  2. picking bag + "drop the bag" / "stop" -> cancel -> LISTENING (not ERROR)
  3. ERROR always speaks first, then returns to LISTENING
  4. "stop" works in every state
  5. timeouts when the robot never answers (grab 30 s / drop 20 s / person 5 s)
"""
import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

LISTENING, PICKING_BAG, ANNOUNCE_READY, FOLLOWING, DROPPING_BAG, ERROR = (
    'listening', 'picking_bag', 'announce_ready', 'following', 'dropping_bag', 'error')
MOTION = {'move_forward', 'move_backward', 'turn', 'go_to', 'look_at'}
# Other names teammates' nodes may send for the same event (unknown events are only logged, never a crash)
EVENT_ALIASES = {'drop_success': 'drop_done', 'grab_done': 'grab_success', 'pick_success': 'grab_success',
                 'pick_failed': 'grab_failed', 'person_seen': 'see_person', 'person_lost': 'lost_person'}


class FlowLogic:
    """Pure state machine (no ROS), unit-testable. Every method returns a list of outputs:
    ("say", text) | ("play", wav_path) | ("cmd", {"action":..., "params":{...}}) | ("state", name) | ("log", text)"""

    def __init__(self, ask_text="How can I help you?", ready_text="I am ready to follow you.",
                 reask_s=10.0, grab_timeout=30.0, drop_timeout=20.0, person_timeout=5.0,
                 require_person=True, beatbox_file=""):
        self.ask_text, self.ready_text = ask_text, ready_text
        self.reask_s, self.grab_timeout, self.drop_timeout = reask_s, grab_timeout, drop_timeout
        self.person_timeout, self.require_person = person_timeout, require_person
        self.beatbox_file = beatbox_file
        self.state = None
        self.carrying = False
        self.person_visible = False
        self.deadline = None          # timeout for the current state
        self.last_activity = 0.0      # for the LISTENING re-ask timer
        # speech tracking: "speech done" = tts went speaking True -> False after we asked it to talk
        self.speaking = False
        self.await_speech = False
        self.speech_started = False
        self.speech_sent_at = 0.0
        self.error_text = ""

    # ---------------- helpers ----------------
    def _say(self, text, now):
        self.await_speech, self.speech_started, self.speech_sent_at = True, False, now
        return [("say", text)]

    def _goto(self, state, now, msg=""):
        self.state = state
        self.deadline = None
        out = [("state", state)]
        if state == LISTENING:
            self.last_activity = now
            out += self._say(self.ask_text, now)
        elif state == PICKING_BAG:
            self.deadline = now + self.grab_timeout
            out += [("cmd", {"action": "carry_bag", "params": {}})]
            out += self._say(msg or "Okay, picking up the bag.", now)
        elif state == ANNOUNCE_READY:
            out += self._say(self.ready_text, now)
        elif state == FOLLOWING:
            out += [("cmd", {"action": "follow_me", "params": {}})]
        elif state == DROPPING_BAG:
            self.deadline = now + self.drop_timeout
            out += [("cmd", {"action": "drop_bag", "params": {}})]
            out += self._say(msg or "Okay, dropping the bag.", now)
        elif state == ERROR:
            self.error_text = msg or "Sorry, something went wrong."
            # wheels stop; the arm is left alone (it already failed / is busy with its own task, e.g. going to rest)
            out += [("cmd", {"action": "stop", "params": {"arm": False}})]
            out += self._say(self.error_text, now)
        return out

    def _speech_done(self, now):
        """True once after the speech we asked for has finished (or TTS never started within 4 s)."""
        if not self.await_speech:
            return False
        if self.speech_started and not self.speaking:
            self.await_speech = False
            return True
        if not self.speech_started and now - self.speech_sent_at > 4.0:   # TTS dead / muted: don't hang
            self.await_speech = False
            return True
        return False

    def _stop(self, now, reply="Stopping."):
        out = [("cmd", {"action": "stop", "params": {}})]
        if self.state == LISTENING:
            self.last_activity = now
            return out + self._say(reply, now)
        return out + self._goto(LISTENING, now)

    # ---------------- events ----------------
    def start(self, now):
        return self._goto(LISTENING, now)

    def on_heard(self, text, now):
        self.last_activity = now            # someone is talking: don't nag
        return []

    def on_speaking(self, speaking, now):
        self.speaking = speaking
        if speaking and self.await_speech:
            self.speech_started = True
        if not speaking:
            self.last_activity = now        # re-ask timer counts from the end of the robot's speech
        return []

    def on_event(self, ev, now):
        # "name" or "name|ERROR_CODE|English message" (from arm_action_bridge / SO-100 API)
        parts = ev.strip().split('|', 2)
        ev = EVENT_ALIASES.get(parts[0].strip().lower(), parts[0].strip().lower())
        code = parts[1].strip().upper() if len(parts) > 1 else ''
        arm_msg = parts[2].strip() if len(parts) > 2 else ''
        if ev == 'grab_failed' and code == 'HOLDING':      # arm already holds something: treat as picked up
            ev = 'grab_success'
        if ev == 'drop_failed' and code == 'NOT_HOLDING':  # nothing in the gripper any more
            self.carrying = False
        if ev == 'see_person':
            self.person_visible = True
            return []
        if ev == 'lost_person':
            self.person_visible = False
            if self.state == FOLLOWING:
                return self._goto(ERROR, now, "I lost you. Please come back in front of me.")
            return []
        if ev == 'grab_success' and self.state == PICKING_BAG:
            self.carrying = True
            return self._goto(ANNOUNCE_READY, now)
        if ev == 'grab_failed' and self.state == PICKING_BAG:
            return self._goto(ERROR, now, f"Sorry. {arm_msg}" if arm_msg else "Sorry, I could not pick up the bag.")
        if ev == 'drop_done' and self.state == DROPPING_BAG:
            self.carrying = False
            return [("say", "Here is your bag.")] + self._goto(LISTENING, now)
        if ev == 'drop_failed' and self.state == DROPPING_BAG:
            return self._goto(ERROR, now, f"Sorry. {arm_msg}" if arm_msg else "Sorry, I could not drop the bag.")
        if ev == 'arm_holding':                   # the arm reports something in its gripper
            self.carrying = True
            return []
        if ev == 'arm_empty':                     # the arm reports an empty gripper
            was = self.carrying
            self.carrying = False
            if self.state == FOLLOWING and was:   # fell out while walking
                return self._goto(ERROR, now, "I think I dropped the bag.")
            return []
        if ev == 'bag_lost':
            was = self.carrying
            self.carrying = False
            if self.state == FOLLOWING and was:
                return self._goto(ERROR, now, "I think I dropped the bag.")
            return []
        return [("log", f"(event '{ev}' ignored in state {self.state})")]

    def on_tick(self, now):
        done = self._speech_done(now)
        s = self.state
        if s == LISTENING:
            if not self.speaking and not self.await_speech and now - self.last_activity > self.reask_s:
                self.last_activity = now
                return self._say(self.ask_text, now)            # "no command [10 s] / ask again"
        elif s == ANNOUNCE_READY:
            if done:
                if self.person_visible or not self.require_person:
                    return self._goto(FOLLOWING, now)
                self.deadline = now + self.person_timeout       # speech done but nobody seen yet: wait
            elif not self.await_speech and self.deadline is not None:
                if self.person_visible:
                    return self._goto(FOLLOWING, now)
                if now > self.deadline:
                    return self._goto(ERROR, now, "I can't see you. Please stand in front of me.")
        elif s == ERROR:
            if done:
                return self._goto(LISTENING, now)
        elif s == PICKING_BAG and self.deadline and now > self.deadline:
            return self._goto(ERROR, now, "Sorry, I could not pick up the bag.")
        elif s == DROPPING_BAG and self.deadline and now > self.deadline:
            return self._goto(ERROR, now, "Sorry, I could not drop the bag.")
        return []

    def on_intent(self, intent, now):
        a = intent.get("action", "none")
        params = intent.get("params") or {}
        reply = intent.get("reply") or ""
        s = self.state
        self.last_activity = now

        if a == "stop":                                     # fix 4: stop works everywhere
            return self._stop(now)

        if s == LISTENING:
            if a == "carry_bag":
                if self.carrying:
                    return self._say("I'm already carrying your bag.", now)
                return self._goto(PICKING_BAG, now)
            if a == "drop_bag":
                if not self.carrying:
                    return self._goto(ERROR, now, "I don't have a bag.")   # "dont have a bag"
                return self._goto(DROPPING_BAG, now)
            if a == "follow_me":
                return self._goto(ANNOUNCE_READY, now)
            if a == "beatbox":
                out = self._say(reply or "Okay, check this out!", now)
                return out + ([("play", self.beatbox_file)] if self.beatbox_file else [])
            if a in MOTION:
                return [("cmd", {"action": a, "params": params})] + (self._say(reply, now) if reply else [])
            return self._say(reply, now) if reply else []       # chat / didn't understand

        if s == PICKING_BAG and a in ("drop_bag", "cancel"):   # fix 2: cancel, don't fail
            return [("cmd", {"action": "stop", "params": {}}), ("say", "Okay, cancelled.")] + \
                self._goto(LISTENING, now)
        if s == FOLLOWING and a == "drop_bag":
            return self._goto(DROPPING_BAG, now) if self.carrying else self._say("I don't have a bag.", now)
        if s in (PICKING_BAG, DROPPING_BAG, ANNOUNCE_READY):
            return self._say("Please wait a moment.", now)
        if s == FOLLOWING:
            return self._say("I'm following you right now.", now)
        return []


class FlowNode(Node):
    def __init__(self):
        super().__init__('flow_node')
        self.declare_parameter('ask_text', 'How can I help you?')
        self.declare_parameter('ready_text', 'I am ready to follow you.')
        self.declare_parameter('reask_s', 10.0)          # ask again after this much silence in LISTENING
        self.declare_parameter('grab_timeout', 30.0)     # no grab_success within this -> error (arm pick: 12-18 s)
        self.declare_parameter('drop_timeout', 20.0)     # no drop_done within this -> error (arm place: 8-12 s)
        self.declare_parameter('person_timeout', 5.0)    # no see_person after "ready to follow" -> error
        self.declare_parameter('require_person', True)   # False = follow without waiting for see_person
        self.declare_parameter('beatbox_file', '~/robot_voice/sounds/cat_beatbox.wav')
        p = lambda n: self.get_parameter(n).value
        self.logic = FlowLogic(p('ask_text'), p('ready_text'), float(p('reask_s')), float(p('grab_timeout')),
                               float(p('drop_timeout')), float(p('person_timeout')), bool(p('require_person')),
                               p('beatbox_file'))

        self.pub_text = self.create_publisher(String, '/speech/text', 10)
        self.pub_say = self.create_publisher(String, '/tts/say', 10)
        self.pub_play = self.create_publisher(String, '/tts/play', 10)
        self.pub_cmd = self.create_publisher(String, '/robot/command', 10)
        self.pub_state = self.create_publisher(String, '/flow/state', 10)
        self.create_subscription(String, '/speech/heard', self._on_heard, 10)
        self.create_subscription(String, '/robot/intent', self._on_intent, 10)
        self.create_subscription(String, '/robot/event', self._on_event, 10)
        self.create_subscription(Bool, '/tts/speaking', self._on_speaking, 10)
        self.create_timer(0.2, self._on_tick)
        self.started = False
        self.get_logger().info('flow ready (no wake word). Robot events: /robot/event '
                               '[grab_success, grab_failed, drop_done, drop_failed, see_person, lost_person, bag_lost]')

    def _emit(self, outputs):
        for kind, val in outputs:
            if kind == 'say':
                self.get_logger().info(f'say: "{val}"')
                self.pub_say.publish(String(data=val))
            elif kind == 'play':
                self.pub_play.publish(String(data=val))
            elif kind == 'cmd':
                cmd = dict(val, stamp=time.time(), source='flow')
                self.get_logger().info(f'-> robot: {val["action"]} {val.get("params") or ""}')
                self.pub_cmd.publish(String(data=json.dumps(cmd)))
            elif kind == 'state':
                self.get_logger().info(f'== state: {val} ==')
                self.pub_state.publish(String(data=val))
            else:
                self.get_logger().info(val)

    def _on_heard(self, msg: String):
        self._emit(self.logic.on_heard(msg.data, time.time()))
        self.pub_text.publish(msg)                       # hand the sentence to the brain (rules / LLM)

    def _on_intent(self, msg: String):
        try:
            intent = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(f'bad intent: {msg.data}')
            return
        self._emit(self.logic.on_intent(intent, time.time()))

    def _on_event(self, msg: String):
        if msg.data.strip().lower() != 'see_person':     # see_person may arrive many times per second
            self.get_logger().info(f'event: {msg.data}')
        self._emit(self.logic.on_event(msg.data, time.time()))

    def _on_speaking(self, msg: Bool):
        self._emit(self.logic.on_speaking(msg.data, time.time()))

    def _on_tick(self):
        now = time.time()
        if not self.started:                            # wait until tts_node listens, so the first question is heard
            if self.pub_say.get_subscription_count() == 0:
                return
            self.started = True
            self._emit(self.logic.start(now))
            return
        self._emit(self.logic.on_tick(now))


def main():
    rclpy.init()
    node = FlowNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
