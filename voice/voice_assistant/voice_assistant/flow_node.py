"""Flow node: the robot's task state machine (team state chart, corrected), no wake word.

    stt_node --/speech/heard--> flow_node --/speech/text--> rule_node | llm_node
                                    ^                              |
                                    +-------- /robot/intent -------+   (what the user wants)
    teammates' nodes --/robot/event--> flow_node --/robot/command--> command_bridge_node / robot
                                    flow_node --/tts/say, /tts/play--> tts_node

The states, events and the rules are documented in flow_logic.py (pure, no ROS, unit-tested there);
this file is only the node and the topic wiring.
"""
import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

from voice_assistant.flow_logic import FlowLogic


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
        self.declare_parameter('ask_follow', True)       # after picking up: ask "Should I follow you?" first
        self.declare_parameter('ask_follow_text', 'I have your bag. Should I follow you?')
        p = lambda n: self.get_parameter(n).value
        self.logic = FlowLogic(p('ask_text'), p('ready_text'), float(p('reask_s')), float(p('grab_timeout')),
                               float(p('drop_timeout')), float(p('person_timeout')), bool(p('require_person')),
                               p('beatbox_file'), bool(p('ask_follow')), str(p('ask_follow_text')))

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
        # Start (= first "How can I help you?") only once tts_node says it can talk (/tts/ready, latched) plus a
        # short settle time; a message published the instant tts_node appears can be lost during discovery.
        self.declare_parameter('start_delay_s', 1.5)
        self.start_delay = float(self.get_parameter('start_delay_s').value)
        self.tts_ready_at = None
        self.sub_seen_at = None
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Bool, '/tts/ready', self._on_tts_ready, latched)
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

    def _on_tts_ready(self, msg: Bool):
        if msg.data and self.tts_ready_at is None:
            self.tts_ready_at = time.time()

    def _on_speaking(self, msg: Bool):
        self._emit(self.logic.on_speaking(msg.data, time.time()))

    def _on_tick(self):
        now = time.time()
        if not self.started:                            # wait until tts_node can talk, so the first question is heard
            if self.pub_say.get_subscription_count() == 0:
                return
            if self.sub_seen_at is None:
                self.sub_seen_at = now
            ready = self.tts_ready_at is not None and now - self.tts_ready_at >= self.start_delay
            old_tts = self.tts_ready_at is None and now - self.sub_seen_at >= 5.0   # tts_node without /tts/ready
            if not (ready or old_tts):
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
