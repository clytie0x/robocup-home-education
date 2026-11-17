"""Rule-based brain (no LLM): /speech/text -> keyword/regex rules -> /robot/intent (-> flow_node)

Drop-in replacement for llm_node (same topics), answers in a few milliseconds and needs no
llama-server (saves ~1.4 GB RAM). The patterns and RuleParser live in rules.py (pure, no ROS, unit-tested
there); this file is only the node.

  task_words  false = the generic voice launch (llm-style commands only)
              true  = Help Me Carry: also "this is the car", "go back to the start", "continue" and
                      "ready" / "done" / "okay" at the car (launch/hmc_voice.launch.py sets it)
"""
import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from voice_assistant.rules import RuleParser


class RuleNode(Node):
    def __init__(self):
        super().__init__("rule_node")
        self.declare_parameter("allowed_actions",
                               ["none", "move_forward", "move_backward", "turn", "stop",
                                "go_to", "follow_me", "go_back", "resume", "look_at", "drop_bag",
                                "carry_bag", "beatbox"])
        self.declare_parameter("capabilities",
                               "I can carry your bag, drop the bag, follow you, move forward or backward, "
                               "turn left or right, do a beatbox, and stop.")
        self.declare_parameter("robot_name", "Predictably Slightly Unpredictable")
        self.declare_parameter("beatbox_file", "~/robot_voice/sounds/cat_beatbox.wav")
        self.declare_parameter("fuzzy_commands", True)   # sound-alike fallback for carry/drop/follow/stop
        self.declare_parameter("fuzzy_min", 0.75)        # 0..1, higher = stricter
        self.declare_parameter("task_words", False)      # Help Me Carry words (see rules.TASK_WORDS)
        # speak_chat: also say the chat replies (greetings, thanks, "what can you do") straight to /tts/say,
        # so the robot answers general talk even when the task flow (hmc_flow) only reacts to task words.
        # Task commands still go only to /robot/intent, so the robot never speaks a command twice.
        self.declare_parameter("speak_chat", False)
        p = lambda n: self.get_parameter(n).value
        self.beatbox_file = p("beatbox_file")
        self.parser = RuleParser(p("capabilities"), list(p("allowed_actions")), p("robot_name"))
        self.parser.fuzzy = bool(p("fuzzy_commands"))
        self.parser.fuzzy_min = float(p("fuzzy_min"))
        self.parser.task_words = bool(p("task_words"))
        self.speak_chat = bool(p("speak_chat"))

        # Only reports WHAT the user wants; flow_node decides what the robot says and does.
        self.pub_intent = self.create_publisher(String, "/robot/intent", 10)
        self.pub_raw = self.create_publisher(String, "/llm/response", 10)
        self.pub_say = self.create_publisher(String, "/tts/say", 10)   # used only when speak_chat is on
        self.create_subscription(String, "/speech/text", self._on_text, 10)
        self.get_logger().info("rule brain ready (no LLM) -> /robot/intent"
                               + (", Help Me Carry words" if self.parser.task_words else ""))

    def _on_text(self, msg: String):
        text = msg.data.strip()
        if not text:
            return
        t0 = time.time()
        self.parser.last_fuzzy = None
        action, params, reply = self.parser.parse(text)
        if action not in self.parser.allowed:
            action, params = "none", {}
        fz = f" [sounds like {action}, {self.parser.last_fuzzy[1]:.2f}]" if self.parser.last_fuzzy else ""
        self.get_logger().info(f'({(time.time() - t0) * 1000:.1f} ms) action={action} params={params} '
                               f'reply="{reply}"{fz}')
        out = {"action": action, "params": params, "reply": reply}
        self.pub_raw.publish(String(data=json.dumps(out)))
        self.pub_intent.publish(String(data=json.dumps(dict(out, utterance=text, source="rules"))))
        # Greetings / thanks / capabilities are action 'none' with a friendly reply; hmc_flow ignores them,
        # so say them here. The "didn't catch that" fallback is skipped, or the robot would answer every noise.
        if self.speak_chat and action == "none" and reply and not reply.lower().startswith("sorry, i didn't catch"):
            self.pub_say.publish(String(data=reply))


def main():
    rclpy.init()
    node = RuleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
