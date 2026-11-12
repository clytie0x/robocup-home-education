"""Dialog node: wake word + "how can I help?" conversation layer between STT and the LLM.

    stt_node --/speech/heard--> dialog_node --/speech/text--> llm_node
                                     └------/tts/say-------> tts_node (greeting, capabilities, goodbye)

States
  sleeping : only reacts to the wake word ("hey robot") and to "stop" (always allowed, for safety)
  awake    : every sentence goes to the LLM; no wake word needed.
             Goes back to sleeping after `awake_timeout` s of silence (counted from the end of the
             robot's own speech) or when the user says "that's all", "goodbye", "no thanks", ...

"Hey robot" / "Hello"       -> first time: spoken greeting + things it can do; later: a short chime
"What's your name?"         -> introduces itself
"Robot, drop the bag"       -> command forwarded straight away (no greeting), then awake
"What can you do?" (awake)  -> capabilities
Publishes /dialog/state ("sleeping" / "awake").
"""
import difflib
import re
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

STOP_RX = re.compile(r"^\W*(?:(?:hey\s+)?\w+\W+)?(stop|halt|freeze)\b", re.I)
FAREWELL_RX = re.compile(
    r"^\W*(no|nope|nothing|no thanks?|no thank you|that'?s (all|it)|that is all|goodbye|good bye|bye"
    r"|never ?mind|thank you|thanks|i'?m (fine|good|ok|okay)|go to sleep)\W*(robot)?\W*$", re.I)
CAPS_RX = re.compile(
    r"what (can|do|else can) you do|what are you able to do|your (skills|abilities|functions|commands)"
    r"|how can you help|what can i (ask|say)", re.I)
NAME_RX = re.compile(r"what'?s your name|what is your name|who are you|introduce yourself", re.I)
FUZZY_IGNORE = {"root", "roots", "robe", "robes", "rob", "robin", "about"}
FILLER = {"hi", "hello", "hey", "there", "please", "yo", "ok", "okay", "are", "you", "buddy", "hi there"}


class DialogLogic:
    """Pure state machine (no ROS) so it can be unit-tested. Every method returns a list of
    (kind, text) outputs: kind in {"say", "play", "forward", "state", "log"}."""

    def __init__(self, wake_words, greeting, capabilities, farewell, sleep_message,
                 awake_timeout=15.0, require_wake_word=True, wake_fuzzy=0.78,
                 fuzzy_words=("robot",), robot_name="Predictably Slightly Unpredictable",
                 start_only_words=("hello",), wake_chime="", sleep_chime="", intro_once=True):
        self.wake_list = [w.strip().lower() for w in wake_words if w.strip()]
        self.fuzzy_list = [w.strip().lower() for w in fuzzy_words if w.strip()]
        self.start_only = {w.strip().lower() for w in start_only_words if w.strip()}
        self.wake_fuzzy = wake_fuzzy
        self.name = robot_name
        fmt = lambda t: t.replace("{name}", robot_name)
        greeting, capabilities, farewell, sleep_message = map(fmt, (greeting, capabilities, farewell, sleep_message))
        self.intro = f"I'm {robot_name}. {fmt(capabilities)}"
        words = "|".join(re.escape(w) for w in self.wake_list)
        self.wake_rx = re.compile(rf"\b(?:(?:hey|hi|hello|ok|okay)\s+)?(?:{words})\b\W*", re.I)
        self.greeting, self.capabilities = greeting, capabilities
        self.farewell, self.sleep_message = farewell, sleep_message
        self.awake_timeout = awake_timeout
        self.require_wake_word = require_wake_word
        self.awake = not require_wake_word
        self.awake_until = 0.0
        self.speaking = False
        self.wake_chime, self.sleep_chime = wake_chime, sleep_chime
        self.intro_once = intro_once      # full spoken intro only on the first wake; later wakes just chime
        self.intro_done = False

    # -- helpers --
    def _wake(self, now):
        out = [] if self.awake else [("state", "awake")]
        self.awake = True
        self.awake_until = now + self.awake_timeout
        return out

    def _sleep(self):
        self.awake = False
        return [("state", "sleeping")]

    def _refresh(self, now):
        self.awake_until = now + self.awake_timeout

    def _find_wake(self, text):
        """(start, end) of the wake word in text, or None. Exact match first, then fuzzy:
        a word (or two words joined, e.g. 'ro bot') that sounds close to a wake word."""
        for m in self.wake_rx.finditer(text):
            core = re.findall(r"[a-z]+", m.group().lower())[-1]
            # words like "hello" only wake the robot at the start of a sentence ("say hello to him" does not)
            if core in self.start_only and re.search(r"[a-z]", text[:m.start()], re.I):
                continue
            return m.start(), m.end()
        if not self.wake_fuzzy:
            return None
        toks = list(re.finditer(r"[A-Za-z']+", text))
        for i, t in enumerate(toks):
            cands = [(t.group().lower(), t.end())]
            if i + 1 < len(toks):
                cands.append((t.group().lower() + toks[i + 1].group().lower(), toks[i + 1].end()))
            for word, end in cands:
                if len(word) < 4 or word in FUZZY_IGNORE:
                    continue
                if any(difflib.SequenceMatcher(None, word, w).ratio() >= self.wake_fuzzy for w in self.fuzzy_list):
                    return t.start(), end
        return None

    # -- events --
    def on_speaking(self, speaking, now):
        self.speaking = speaking
        if not speaking and self.awake:
            self._refresh(now)          # start the silence timer after the robot stops talking
        return []

    def on_tick(self, now):
        if self.require_wake_word and self.awake and not self.speaking and now > self.awake_until:
            out = self._sleep()
            if self.sleep_message:
                out.insert(0, ("say", self.sleep_message))
            elif self.sleep_chime:
                out.insert(0, ("play", self.sleep_chime))
            return out
        return []

    def on_heard(self, text, now):
        text = text.strip()
        if not text:
            return []
        if not self.require_wake_word:
            return [("forward", text)]

        hit = self._find_wake(text)
        if hit:
            start, end = hit
            rest = text[end:].strip(" ,.!?")
            if not rest:                               # "Drop the bag, robot" (wake word at the end)
                rest = text[:start].strip(" ,.!?")
            out = self._wake(now)
            words = [w for w in re.findall(r"[a-z']+", rest.lower()) if w not in FILLER]
            if words and not FAREWELL_RX.match(rest):
                if NAME_RX.search(rest):
                    return out + [("say", self.intro)]
                if CAPS_RX.search(rest):
                    return out + [("say", self.capabilities)]
                return out + [("forward", rest)]
            if not out:                               # already awake: just acknowledge, no long intro again
                return [("play", self.wake_chime)] if self.wake_chime else [("say", "Yes? I'm listening.")]
            if self.intro_once and self.intro_done and self.wake_chime:
                return out + [("play", self.wake_chime)]   # quick "ding": I'm listening
            self.intro_done = True
            return out + [("say", f"{self.greeting} {self.capabilities}")]

        if STOP_RX.match(text):                       # always obey stop, even when sleeping
            if self.awake:
                self._refresh(now)
            return [("forward", text)]

        if not self.awake:
            return [("log", f'(sleeping, ignored) "{text}"')]

        if FAREWELL_RX.match(text):
            return [("say", self.farewell)] + self._sleep()
        self._refresh(now)
        if NAME_RX.search(text):
            return [("say", self.intro)]
        if CAPS_RX.search(text):
            return [("say", self.capabilities)]
        return [("forward", text)]


class DialogNode(Node):
    def __init__(self):
        super().__init__('dialog_node')
        self.declare_parameter('require_wake_word', True)
        self.declare_parameter('robot_name', 'Predictably Slightly Unpredictable')
        self.declare_parameter('wake_words', ['robot', 'robert', 'rowboat', 'hello'])
        self.declare_parameter('fuzzy_words', ['robot'])  # only these also match similar-sounding words
        self.declare_parameter('start_only_words', ['hello'])  # these wake the robot only at the start of a sentence
        self.declare_parameter('wake_fuzzy', 0.78)    # 0 = exact only; lower = easier to wake (more false wakes)
        self.declare_parameter('awake_timeout', 15.0)
        self.declare_parameter('greeting', "Hi! I'm {name}. How can I help you?")
        self.declare_parameter('capabilities',
                               'I can carry your bag, drop the bag, follow you, move forward or backward, '
                               'turn left or right, do a beatbox, and stop.')
        self.declare_parameter('farewell', 'Okay. Just say hello if you need me.')
        self.declare_parameter('sleep_message', '')   # said when the awake window times out ('' = silent)
        # Short sounds instead of long speech. '' = off (then it speaks instead).
        self.declare_parameter('wake_chime', '~/robot_voice/sounds/wake_chime.wav')
        self.declare_parameter('sleep_chime', '~/robot_voice/sounds/sleep_chime.wav')
        self.declare_parameter('intro_once', True)    # full spoken intro only on the first wake after start

        p = lambda n: self.get_parameter(n).value
        self.logic = DialogLogic(list(p('wake_words')), p('greeting'), p('capabilities'),
                                 p('farewell'), p('sleep_message'),
                                 float(p('awake_timeout')), bool(p('require_wake_word')),
                                 float(p('wake_fuzzy')), list(p('fuzzy_words')), p('robot_name'),
                                 list(p('start_only_words')), p('wake_chime'), p('sleep_chime'),
                                 bool(p('intro_once')))

        self.pub_text = self.create_publisher(String, '/speech/text', 10)
        self.pub_say = self.create_publisher(String, '/tts/say', 10)
        self.pub_play = self.create_publisher(String, '/tts/play', 10)
        self.pub_state = self.create_publisher(String, '/dialog/state', 10)
        self.create_subscription(String, '/speech/heard', self._on_heard, 10)
        self.create_subscription(Bool, '/tts/speaking', self._on_speaking, 10)
        self.create_timer(0.5, self._on_tick)

        state = 'awake' if self.logic.awake else 'sleeping'
        self.pub_state.publish(String(data=state))
        self.get_logger().info(f'dialog ready ({state}), say "Hey robot" to wake me '
                               f'(wake words: {list(p("wake_words"))}, fuzzy {p("wake_fuzzy")})')

    def _emit(self, outputs):
        for kind, text in outputs:
            if kind == 'say':
                self.get_logger().info(f'say: "{text}"')
                self.pub_say.publish(String(data=text))
            elif kind == 'play':
                self.get_logger().info(f'play: {text}')
                self.pub_play.publish(String(data=text))
            elif kind == 'forward':
                self.get_logger().info(f'-> LLM: "{text}"')
                self.pub_text.publish(String(data=text))
            elif kind == 'state':
                self.get_logger().info(f'state: {text}')
                self.pub_state.publish(String(data=text))
            else:
                self.get_logger().info(text)

    def _on_heard(self, msg: String):
        self._emit(self.logic.on_heard(msg.data, time.time()))

    def _on_speaking(self, msg: Bool):
        self._emit(self.logic.on_speaking(msg.data, time.time()))

    def _on_tick(self):
        self._emit(self.logic.on_tick(time.time()))


def main():
    rclpy.init()
    node = DialogNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
