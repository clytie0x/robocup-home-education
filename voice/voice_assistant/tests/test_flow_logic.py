"""FlowLogic (voice's own state machine, used by the generic voice launch) without ROS.

The class is pure and now lives in flow_logic.py, so it is importable here; flow_node.py keeps only the
rclpy node and the topic wiring.
"""
from voice_assistant.flow_logic import (ANNOUNCE_READY, ASK_FOLLOW, ERROR, FOLLOWING, LISTENING,
                                        PICKING_BAG, FlowLogic)


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
