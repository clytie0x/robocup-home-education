"""LLM node: /speech/text -> llama.cpp server -> /tts/say + /robot/command

The LLM is forced (JSON schema / grammar) to answer as:
    {"reply": "<what to say>", "action": "<one of allowed_actions>", "params": {...}}
so the robot side always receives machine-readable commands.

Topics
  sub  /speech/text     std_msgs/String  user utterance (from STT, or `ros2 topic pub` for testing)
  sub  /robot/state     std_msgs/String  optional free-text robot status injected into the prompt
  pub  /llm/response    std_msgs/String  full JSON from the LLM (debug / logging)
  pub  /robot/intent    std_msgs/String  JSON {"action", "params", "reply", "utterance"} -> flow_node
"""
import json
import queue
import re
import threading
import time

import requests
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

DEFAULT_SYSTEM = (
    "You are the voice of a small mobile robot. Speak English only. "
    "Keep replies short (max 2 sentences), friendly and natural to say out loud: "
    "no lists, no markdown, no emojis. "
    "If the user asks the robot to do something physical, choose the matching action and fill params; "
    "otherwise use action \"none\". Never invent actions that are not in the allowed list. "
    "Units: distance in meters, angle in degrees (positive = left/counter-clockwise, negative = right), "
    "speed in m/s.\n"
    "Actions:\n"
    "- move_forward / move_backward: params.distance (default 0.5), optional params.speed\n"
    "- turn: params.angle in degrees (left positive, right negative; 'a little' = 20, 'around' = 180)\n"
    "- stop: stop moving now\n"
    "- carry_bag: the user wants the robot to carry / take / hold / pick up a bag for them\n"
    "- drop_bag: drop / release / put down / let go of the bag the robot is carrying\n"
    "- beatbox: the user asks the robot to beatbox, drop a beat or play music\n"
    "- go_to: params.target = place name; follow_me; look_at: params.target\n"
    "- none: chatting, questions, or anything the robot cannot do\n"
    "If the sentence is unclear, incomplete or not a command (e.g. 'After that.', 'Okay then.'), "
    "use action \"none\" and ask the user to repeat. Never guess a movement."
)

# An action is only accepted if the sentence contains one of its command words; otherwise the LLM guessed
# (whisper noise like "After that." -> move_forward). 'stop' and 'none' are always allowed.
ACTION_WORDS = {
    'move_forward': r"\b(forward|ahead|straight|closer)\b|\bcome\s+(here|to\s+me)\b|\bmove\b.*\b(meters?|metres?|bit|steps?|little)\b",
    'move_backward': r"\b(back|backwards?|reverse)\b",
    'turn': r"\b(turn|rotate|spin|left|right|around)\b",
    'carry_bag': r"\b(bags?|carry|pick|hold|grab|take)\b",
    'drop_bag': r"\b(drop|release|let\s+go|put|down)\b",
    'follow_me': r"\bfollow\b|\bcome\s+with\b",
    'go_to': r"\b(go|navigate|take\s+me|bring\s+me)\b",
    'look_at': r"\b(look|face|watch)\b",
    'beatbox': r"\b(beat|box|music|song|rap)",
}
ACTION_RX = {a: re.compile(rx, re.I) for a, rx in ACTION_WORDS.items()}
UNCLEAR_REPLY = "Sorry, I didn't catch that. Please say it again."

# Example turns shown to the LLM before the real conversation (few-shot).
FEW_SHOT = [
    ("Can you help me carry this bag?", {"reply": "Sure, I will carry the bag for you.",
                                         "action": "carry_bag", "params": {}}),
    ("Drop the bag", {"reply": "Okay, dropping the bag.", "action": "drop_bag", "params": {}}),
    ("Move forward one meter", {"reply": "Moving forward one meter.", "action": "move_forward",
                                "params": {"distance": 1.0}}),
    ("Turn right a little", {"reply": "Turning right a little.", "action": "turn",
                             "params": {"angle": -20}}),
    ("Hi, how are you?", {"reply": "I'm great, thanks! How can I help you?", "action": "none", "params": {}}),
    ("After that.", {"reply": "Sorry, I didn't catch that. Please say it again.", "action": "none", "params": {}}),
]

# Fast path: safety-critical / must-not-miss commands are matched by keyword and executed
# immediately, without waiting for (or trusting) the LLM.
NEGATION = re.compile(r"\b(don'?t|do not|never|not|no)\b", re.I)
from voice_assistant.rule_node import STOP_ALIKE, FUZZY_REPLY, fuzzy_command   # shared with the rule brain

FAST_RULES = [
    ("stop", re.compile(r"^\W*(?:(?:hey\s+)?robot\W+|please\W+)*(stop|halt|freeze)\b", re.I),
     "Stopping."),
    ("beatbox", re.compile(r"\b(beat|bit|beet|beats)[\s-]?box(ing)?\b|\bbeatbox|\b(drop|play)\s+(a|the|some)\s+beats?\b|\bplay\s+(some\s+)?music\b", re.I), "Okay, check this out!"),
    ("drop_bag", re.compile(r"\b(drop|release|unload|let\s+go)\b|\bput\b.*\bdown\b", re.I),
     "Okay, dropping the bag."),
    ("carry_bag", re.compile(r"\bcarry\b|\b(pick\s+up|hold|take|grab)\b.*\bbags?\b", re.I),
     "Sure, I will carry the bag for you."),
]


class LlmNode(Node):
    def __init__(self):
        super().__init__('llm_node')
        self.declare_parameter('llm_url', 'http://127.0.0.1:8081/v1/chat/completions')
        self.declare_parameter('allowed_actions',
                               ['none', 'move_forward', 'move_backward', 'turn', 'stop',
                                'go_to', 'follow_me', 'look_at', 'drop_bag', 'carry_bag', 'beatbox'])
        self.declare_parameter('system_prompt', DEFAULT_SYSTEM)
        self.declare_parameter('robot_name', 'Predictably Slightly Unpredictable')
        self.declare_parameter('beatbox_file', '~/robot_voice/sounds/cat_beatbox.wav')
        self.declare_parameter('history_turns', 4)       # small history = small KV cache = low RAM
        self.declare_parameter('max_tokens', 128)
        self.declare_parameter('temperature', 0.3)
        self.declare_parameter('fast_commands', True)    # keyword fast path for stop / drop_bag
        self.declare_parameter('check_action_words', True)   # reject actions whose command words are missing

        p = lambda n: self.get_parameter(n).value
        self.url = p('llm_url')
        self.actions = list(p('allowed_actions'))
        self.robot_name = p('robot_name')
        self.beatbox_file = p('beatbox_file')
        self.system_prompt = (f"Your name is {self.robot_name}. If asked who you are, say your name. "
                              + p('system_prompt'))
        self.history_turns = int(p('history_turns'))
        self.max_tokens = int(p('max_tokens'))
        self.temperature = float(p('temperature'))
        self.fast_commands = bool(p('fast_commands'))
        self.check_words = bool(p('check_action_words'))
        self.few_shot = []
        for user, ans in FEW_SHOT:
            if ans["action"] in self.actions:
                self.few_shot += [{"role": "user", "content": user},
                                  {"role": "assistant", "content": json.dumps(ans)}]

        self.schema = {
            "type": "object",
            "properties": {
                "reply": {"type": "string"},
                "action": {"type": "string", "enum": self.actions},
                "params": {
                    "type": "object",
                    "properties": {
                        "distance": {"type": "number"},
                        "angle": {"type": "number"},
                        "speed": {"type": "number"},
                        "target": {"type": "string"},
                    },
                },
            },
            "required": ["reply", "action", "params"],
        }

        self.history: list = []
        self.robot_state = ''
        self.jobs: queue.Queue = queue.Queue(maxsize=1)

        # Only reports WHAT the user wants; flow_node decides what the robot says and does.
        self.pub_intent = self.create_publisher(String, '/robot/intent', 10)
        self.pub_raw = self.create_publisher(String, '/llm/response', 10)
        self.create_subscription(String, '/speech/text', self._on_text, 10)
        self.create_subscription(String, '/robot/state', self._on_state, 10)

        threading.Thread(target=self._worker, daemon=True).start()
        self.get_logger().info(f'LLM ready ({self.url}), actions: {self.actions}')

    def _intent(self, action, params, reply, text, source):
        self.pub_intent.publish(String(data=json.dumps(
            {"action": action, "params": params, "reply": reply, "utterance": text, "source": source})))

    def _on_state(self, msg: String):
        self.robot_state = msg.data

    def _fast_match(self, text: str):
        if not self.fast_commands or NEGATION.search(text):
            return None
        parts = [q.strip() for q in re.split(r"[.!?,]+", text.lower()) if q.strip()]
        if "stop" in self.actions and parts and all(
                STOP_ALIKE.match(" ".join(re.sub(r"[^a-z' -]", " ", q).split())) for q in parts):
            return "stop", "Stopping."
        for action, rx, reply in FAST_RULES:
            if action in self.actions and rx.search(text):
                return action, reply
        a, _ = fuzzy_command(text, self.actions)       # "Dopting the bag", "Hollow me" ...
        if a:
            return a, FUZZY_REPLY[a]
        return None

    def _on_text(self, msg: String):
        text = msg.data.strip()
        if not text:
            return
        hit = self._fast_match(text)
        if hit:
            action, reply = hit
            try:                      # a fast command cancels a sentence still waiting for the LLM
                self.jobs.get_nowait()
            except queue.Empty:
                pass
            self.get_logger().info(f'[fast] action={action} for "{text}"')
            self._intent(action, {}, reply, text, 'fast')
            self._remember(text, {"reply": reply, "action": action, "params": {}})
            return
        try:
            self.jobs.put_nowait(text)
        except queue.Full:   # still thinking about the previous sentence -> keep only the newest
            try:
                self.jobs.get_nowait()
            except queue.Empty:
                pass
            self.jobs.put_nowait(text)

    def _worker(self):
        while rclpy.ok():
            try:
                text = self.jobs.get(timeout=0.5)
            except queue.Empty:
                continue
            self._handle(text)

    def _handle(self, text: str):
        sys_msg = self.system_prompt + f"\nAllowed actions: {', '.join(self.actions)}."
        if self.robot_state:
            sys_msg += f"\nCurrent robot state: {self.robot_state}"
        messages = [{"role": "system", "content": sys_msg}] + self.few_shot + self.history + \
                   [{"role": "user", "content": text}]

        body = {
            "messages": messages,
            "max_tokens": self.max_tokens,
            "temperature": self.temperature,
            "response_format": {"type": "json_schema",
                                "json_schema": {"name": "robot_reply", "schema": self.schema}},
        }
        t0 = time.time()
        try:
            r = requests.post(self.url, json=body, timeout=60)
            r.raise_for_status()
            content = r.json()['choices'][0]['message']['content']
            out = json.loads(content)
        except Exception as e:
            self.get_logger().error(f'LLM request/parse failed: {e}')
            self._intent('none', {}, "Sorry, I didn't catch that.", text, 'llm')
            return

        reply = str(out.get('reply', '')).strip()
        action = out.get('action', 'none')
        if action not in self.actions:
            action = 'none'
        params = out.get('params') or {}
        rx = ACTION_RX.get(action)
        if self.check_words and rx is not None and not rx.search(text):
            self.get_logger().info(f'(gated) LLM chose {action} but "{text}" has no {action} words -> none')
            action, params, reply = 'none', {}, UNCLEAR_REPLY
            out = {"reply": reply, "action": action, "params": params}
        self.get_logger().info(f'({time.time() - t0:.2f}s) action={action} params={params} reply="{reply}"')

        self.pub_raw.publish(String(data=json.dumps(out)))
        self._intent(action, params, reply, text, 'llm')
        self._remember(text, out)

    def _remember(self, text: str, out: dict):
        self.history += [{"role": "user", "content": text},
                         {"role": "assistant", "content": json.dumps(out)}]
        self.history = self.history[-2 * self.history_turns:]


def main():
    rclpy.init()
    node = LlmNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
