"""FlowLogic (voice's own state machine, used by the generic voice launch) without ROS.

The class is pure and now lives in flow_logic.py, so it is importable here; flow_node.py keeps only the
rclpy node and the topic wiring.
"""
from voice_assistant.flow_logic import (ANNOUNCE_READY, ASK_FOLLOW, ERROR, FOLLOWING, LISTENING,
                                        PICKING_BAG, FlowLogic, ready_to_start)


def actions(out):
    return [value["action"] for kind, value in out if kind == "cmd"]


def states(out):
    return [value for kind, value in out if kind == "state"]


def test_start_asks_how_it_can_help():
    flow = FlowLogic()
    out = flow.start(0.0)
    assert states(out) == [LISTENING]
    assert ("say", "How can I help you?") in out


def test_carry_bag_picks_the_bag_up_then_asks_to_follow():
    flow = FlowLogic(ask_follow=True)
    flow.start(0.0)
    out = flow.on_intent({"action": "carry_bag"}, 1.0)
    assert actions(out) == ["carry_bag"] and states(out) == [PICKING_BAG]
    out = flow.on_event("grab_success", 2.0)
    assert states(out) == [ASK_FOLLOW]


def test_grab_failed_with_holding_counts_as_picked_up():
    flow = FlowLogic(ask_follow=False)
    flow.start(0.0)
    flow.on_intent({"action": "carry_bag"}, 1.0)
    out = flow.on_event("grab_failed|HOLDING|already holding", 2.0)
    assert flow.carrying and states(out) == [ANNOUNCE_READY]


def test_stop_works_from_any_state():
    flow = FlowLogic(ask_follow=False)
    flow.start(0.0)
    flow.on_intent({"action": "carry_bag"}, 1.0)
    flow.on_event("grab_success", 2.0)                    # announce_ready
    out = flow.on_intent({"action": "stop"}, 3.0)
    assert actions(out) == ["stop"] and states(out) == [LISTENING]


def test_follow_me_makes_it_ready_and_following():
    flow = FlowLogic()
    flow.start(0.0)
    out = flow.on_intent({"action": "follow_me"}, 1.0)
    assert states(out) == [ANNOUNCE_READY] and ("say", "I am ready to follow you.") in out
    flow.on_event("see_person", 2.0)
    flow.on_speaking(True, 2.0)
    flow.on_speaking(False, 3.0)
    out = flow.on_tick(3.5)                               # speech done, the operator is in view
    assert states(out) == [FOLLOWING] and actions(out) == ["follow_me"]


def test_losing_the_person_while_following_is_an_error():
    flow = FlowLogic(require_person=False)
    flow.start(0.0)
    flow.on_intent({"action": "follow_me"}, 1.0)
    flow.on_speaking(True, 2.0)
    flow.on_speaking(False, 3.0)
    flow.on_tick(3.5)
    out = flow.on_event("lost_person", 4.0)
    assert states(out) == [ERROR] and actions(out) == ["stop"]


def test_it_asks_again_when_nobody_says_anything():
    flow = FlowLogic(reask_s=10.0)
    flow.start(0.0)
    assert flow.on_tick(5.0) == []
    out = flow.on_tick(11.0)
    assert ("say", "How can I help you?") in out


def test_blank_heard_does_not_postpone_the_reask():
    # an empty transcript is not "someone is talking"; it must not reset the silence timer
    flow = FlowLogic(reask_s=10.0)
    flow.start(0.0)
    flow.on_tick(5.0)                                 # the first question's speech window has closed
    flow.on_heard("", 9.0)
    flow.on_heard("   ", 9.5)
    flow.on_heard(None, 9.5)
    out = flow.on_tick(11.0)
    assert ("say", "How can I help you?") in out


def test_grab_and_drop_time_out_when_the_arm_never_answers():
    flow = FlowLogic(grab_timeout=30.0, drop_timeout=20.0)
    flow.start(0.0)
    flow.on_intent({"action": "carry_bag"}, 1.0)      # deadline 31
    assert flow.on_tick(31.0) == []
    out = flow.on_tick(31.1)
    assert states(out) == [ERROR] and any(k == "say" and "pick up" in v for k, v in out)

    flow = FlowLogic(ask_follow=False, require_person=False, drop_timeout=20.0)
    flow.start(0.0)
    flow.on_intent({"action": "carry_bag"}, 1.0)
    flow.on_event("grab_success", 2.0)                # announce_ready
    flow.on_speaking(True, 2.1)
    flow.on_speaking(False, 3.0)
    assert states(flow.on_tick(3.1)) == [FOLLOWING]
    flow.on_intent({"action": "drop_bag"}, 4.0)       # deadline 24
    assert flow.on_tick(24.0) == []
    out = flow.on_tick(24.1)
    assert states(out) == [ERROR] and any(k == "say" and "drop" in v for k, v in out)


def test_a_zero_deadline_still_times_out():
    # deadline 0.0 is a real timestamp here; it must not be treated as "no deadline"
    flow = FlowLogic(grab_timeout=0.0)
    flow.start(0.0)
    flow.on_intent({"action": "carry_bag"}, 0.0)
    out = flow.on_tick(0.2)
    assert states(out) == [ERROR]


def test_a_non_finite_timeout_uses_the_documented_wait():
    flow = FlowLogic(grab_timeout=float("nan"))
    flow.start(0.0)
    flow.on_intent({"action": "carry_bag"}, 1.0)
    assert flow.on_tick(30.0) == []                    # fallback is 30 s, measured from entering picking
    out = flow.on_tick(31.1)
    assert states(out) == [ERROR]

    flow = FlowLogic(grab_timeout=float("inf"))
    flow.start(0.0)
    flow.on_intent({"action": "carry_bag"}, 1.0)
    assert states(flow.on_tick(31.1)) == [ERROR]


def test_person_timeout_errors_unless_someone_is_seen():
    flow = FlowLogic(person_timeout=5.0, require_person=True)
    flow.start(0.0)
    flow.on_intent({"action": "follow_me"}, 1.0)
    flow.on_speaking(True, 1.1)
    flow.on_speaking(False, 2.0)
    assert flow.on_tick(2.1) == []                     # speech done, deadline = 7.1
    flow.on_event("see_person", 4.0)
    assert states(flow.on_tick(4.1)) == [FOLLOWING]

    flow = FlowLogic(person_timeout=5.0, require_person=True)
    flow.start(0.0)
    flow.on_intent({"action": "follow_me"}, 1.0)
    flow.on_speaking(True, 1.1)
    flow.on_speaking(False, 2.0)
    flow.on_tick(2.1)
    out = flow.on_tick(7.2)
    assert states(out) == [ERROR] and any(k == "say" and "can't see you" in v for k, v in out)


def test_speech_that_starts_and_never_ends_does_not_stick():
    flow = FlowLogic(require_person=False, speech_limit_s=5.0)
    flow.start(0.0)
    flow.on_intent({"action": "follow_me"}, 1.0)      # speech_sent_at = 1
    flow.on_speaking(True, 1.2)                        # TTS started, then never says it finished
    assert flow.on_tick(4.0) == []
    assert states(flow.on_tick(6.2)) == [FOLLOWING]


def test_malformed_intent_and_event_are_ignored():
    flow = FlowLogic()
    flow.start(0.0)
    for bad in (None, [], "stop", 5, {"action": "move_forward", "params": "fast"}):
        out = flow.on_intent(bad, 1.0)
        assert actions(out) == [] and states(out) == []
        assert flow.state == LISTENING
    assert flow.on_event(None, 2.0) == [("log", "(ignored empty event)")]
    assert flow.on_event("", 2.0) == [("log", "(ignored empty event)")]
    assert flow.on_event("   ", 2.0) == [("log", "(ignored empty event)")]
    # a real stop still wins when the params are garbage
    out = flow.on_intent({"action": " stop ", "params": ["nope"]}, 3.0)
    assert actions(out) == ["stop"]


def test_a_bad_start_delay_still_lets_the_flow_begin():
    assert not ready_to_start(10.0, 10.0, 10.0, float("nan"))
    assert ready_to_start(11.5, 10.0, 10.0, float("nan"))     # non-finite delay falls back to 1.5 s
    assert not ready_to_start(12.0, None, 10.0, 1.5)
    assert ready_to_start(15.0, None, 10.0, 1.5)              # tts_node without /tts/ready: 5 s
    assert not ready_to_start(float("nan"), 0.0, 0.0, 1.5)
