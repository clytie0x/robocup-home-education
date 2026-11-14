import importlib.util
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import hmc_flow  # noqa: E402
from hmc_flow import (  # noqa: E402
    AT_CAR,
    DONE,
    DROPPING,
    ERROR,
    FOLLOWING,
    HANDOVER,
    READY,
    RETURNING,
    HmcFlow,
)


def says(out):
    return [v for k, v in out if k == "say"]


def actions(out):
    return [v["action"] for k, v in out if k == "cmd"]


def cmds(out):
    return [v for k, v in out if k == "cmd"]


def forwarded(out):
    return [v for k, v in out if k == "text"]


class FlowCase(unittest.TestCase):
    def make(self, **kw):
        self.flow = HmcFlow(**kw)
        self.flow.start(0.0)
        return self.flow

    def go(self, state, now=1.0):
        """Drive a fresh flow to `state` through the normal path."""
        f = self.flow
        if state == READY:
            return
        f.on_heard("follow me", now)
        if state == FOLLOWING:
            return
        f.on_heard("this is the car", now)
        if state == AT_CAR:
            return
        f.on_heard("done", now)
        if state == HANDOVER:
            return
        f.on_event("grab_success", now)
        if state == RETURNING:
            return
        f.on_event("at_start", now)
        if state == DROPPING:
            return
        f.on_event("drop_done", now)


class ClassifyTests(unittest.TestCase):
    def test_words(self):
        c = hmc_flow.classify
        for text, kind in [
            ("Follow me please", "follow"),
            ("come with me", "follow"),
            ("Stop!", "stop"),
            ("wait", "stop"),
            ("This is the car.", "stop"),
            ("here", "stop"),
            ("We are here", "stop"),
            ("go back", "back"),
            ("take it back", "back"),
            ("Go to the start", "back"),
            ("done", "done"),
            ("okay", "done"),
            ("ok", "done"),
            ("continue", "resume"),
            ("what is the weather", None),
            ("where are you", None),
            ("don't stop", None),
            ("don't follow me", None),
        ]:
            self.assertEqual(c(text), kind, text)


class StartTests(FlowCase):
    def test_start_is_ready_and_speaks(self):
        f = HmcFlow()
        out = f.start(0.0)
        self.assertEqual(f.state, READY)
        self.assertIn(("state", READY), out)
        self.assertEqual(len(says(out)), 1)


class ReadyTests(FlowCase):
    def test_follow_me_words_start_following(self):
        f = self.make()
        out = f.on_heard("follow me", 1.0)
        self.assertEqual(f.state, FOLLOWING)
        self.assertEqual(actions(out), ["follow_me"])
        self.assertIn("I will follow you.", says(out))
        self.assertEqual(forwarded(out), [])

    def test_follow_me_intent_starts_following(self):
        f = self.make()
        out = f.on_intent({"action": "follow_me", "params": {}, "reply": "Okay"}, 1.0)
        self.assertEqual(f.state, FOLLOWING)
        self.assertEqual(actions(out), ["follow_me"])

    def test_unmatched_text_goes_to_the_brain(self):
        f = self.make()
        out = f.on_heard("hello robot", 1.0)
        self.assertEqual(forwarded(out), ["hello robot"])
        self.assertEqual(f.state, READY)

    def test_done_word_in_ready_is_not_ours_and_is_forwarded(self):
        f = self.make()
        out = f.on_heard("okay", 1.0)
        self.assertEqual(forwarded(out), ["okay"])
        self.assertEqual(f.state, READY)

    def test_stop_in_ready_stops_and_stays(self):
        f = self.make()
        out = f.on_heard("stop", 1.0)
        self.assertEqual(actions(out), ["stop"])
        self.assertEqual(f.state, READY)

    def test_follow_me_refused_while_link_is_down(self):
        f = self.make()
        f.on_event("base_link_lost", 0.5)
        self.assertEqual(f.state, READY)
        out = f.on_heard("follow me", 1.0)
        self.assertEqual(f.state, READY)
        self.assertEqual(actions(out), [])
        self.assertEqual(len(says(out)), 1)
        f.on_event("base_link_ok", 2.0)
        f.on_heard("follow me", 3.0)
        self.assertEqual(f.state, FOLLOWING)


class FollowingTests(FlowCase):
    def test_arrival_words_send_wait_and_ask_for_the_handover(self):
        for words in ("stop", "wait", "this is the car", "here", "we are here"):
            self.make()
            self.go(FOLLOWING)
            out = self.flow.on_heard(words, 2.0)
            self.assertEqual(self.flow.state, AT_CAR, words)
            self.assertEqual(actions(out), ["wait"], words)
            self.assertIn("Please put the bag in my gripper.", says(out))

    def test_stop_intent_goes_to_at_car(self):
        f = self.make()
        self.go(FOLLOWING)
        out = f.on_intent({"action": "stop"}, 2.0)
        self.assertEqual(f.state, AT_CAR)
        self.assertEqual(actions(out), ["wait"])

    def test_go_back_while_following_stops_at_the_car_first(self):
        f = self.make()
        self.go(FOLLOWING)
        out = f.on_heard("go back", 2.0)
        self.assertEqual(f.state, AT_CAR)
        self.assertEqual(actions(out), ["wait"])

    def test_lost_person_for_5_s_asks_to_come_back_once(self):
        f = self.make()
        self.go(FOLLOWING)
        f.on_event("see_person", 1.5)
        f.on_event("lost_person", 2.0)
        self.assertEqual(says(f.on_tick(6.9)), [])
        self.assertEqual(says(f.on_tick(7.0)), ["Please come back."])
        self.assertEqual(says(f.on_tick(9.0)), [])
        self.assertEqual(f.state, FOLLOWING)

    def test_seen_again_cancels_the_come_back(self):
        f = self.make()
        self.go(FOLLOWING)
        f.on_event("see_person", 1.5)
        f.on_event("lost_person", 2.0)
        f.on_event("see_person", 4.0)
        self.assertEqual(says(f.on_tick(8.0)), [])

    def test_never_locked_after_follow_also_asks(self):
        f = self.make()
        self.go(FOLLOWING, now=1.0)
        self.assertEqual(says(f.on_tick(6.0)), ["Please come back."])

    def test_lost_person_is_not_an_error(self):
        f = self.make()
        self.go(FOLLOWING)
        out = f.on_event("lost_person", 2.0)
        self.assertEqual(f.state, FOLLOWING)
        self.assertEqual(actions(out), [])

    def test_link_lost_while_following_is_an_error(self):
        f = self.make()
        self.go(FOLLOWING)
        out = f.on_event("base_link_lost", 2.0)
        self.assertEqual(f.state, ERROR)
        self.assertEqual(actions(out), ["end"])
        self.assertEqual(len(says(out)), 1)


class AtCarTests(FlowCase):
    def test_done_closes_the_gripper_as_a_handover(self):
        f = self.make()
        self.go(AT_CAR)
        out = f.on_heard("done", 3.0)
        self.assertEqual(f.state, HANDOVER)
        self.assertEqual(cmds(out), [{"action": "carry_bag", "params": {"handover": True}}])

    def test_okay_and_go_back_also_mean_done(self):
        for words in ("okay", "go back", "take it back"):
            self.make()
            self.go(AT_CAR)
            self.flow.on_heard(words, 3.0)
            self.assertEqual(self.flow.state, HANDOVER, words)

    def test_carry_bag_intent_means_done(self):
        f = self.make()
        self.go(AT_CAR)
        f.on_intent({"action": "carry_bag"}, 3.0)
        self.assertEqual(f.state, HANDOVER)

    def test_without_the_arm_done_starts_the_return_directly(self):
        f = self.make(use_arm=False)
        self.go(AT_CAR)
        out = f.on_heard("done", 3.0)
        self.assertEqual(f.state, RETURNING)
        self.assertEqual(actions(out), ["go_back"])
        self.assertIn("I will go back to the start now.", says(out))

    def test_follow_me_at_the_car_resumes_following(self):
        f = self.make()
        self.go(AT_CAR)
        out = f.on_heard("follow me", 3.0)
        self.assertEqual(f.state, FOLLOWING)
        self.assertEqual(actions(out), ["resume"])

    def test_asks_again_after_silence(self):
        f = self.make(reask_s=15.0)
        self.go(AT_CAR, now=1.0)
        self.assertEqual(says(f.on_tick(15.9)), [])
        self.assertEqual(says(f.on_tick(16.1)), ["Please put the bag in my gripper."])

    def test_unrelated_speech_is_forwarded(self):
        f = self.make()
        self.go(AT_CAR)
        out = f.on_heard("what is your name", 3.0)
        self.assertEqual(forwarded(out), ["what is your name"])
        self.assertEqual(f.state, AT_CAR)


class HandoverTests(FlowCase):
    def test_grab_success_starts_the_return(self):
        f = self.make()
        self.go(HANDOVER)
        out = f.on_event("grab_success", 4.0)
        self.assertEqual(f.state, RETURNING)
        self.assertEqual(actions(out), ["go_back"])
        self.assertIn("I will go back to the start now.", says(out))
        self.assertTrue(f.carrying)

    def test_grab_failed_holding_counts_as_success(self):
        f = self.make()
        self.go(HANDOVER)
        f.on_event("grab_failed|HOLDING|I already hold something.", 4.0)
        self.assertEqual(f.state, RETURNING)

    def test_grab_failed_is_an_error_with_the_arm_message(self):
        f = self.make()
        self.go(HANDOVER)
        out = f.on_event("grab_failed|MISSED|I missed it.", 4.0)
        self.assertEqual(f.state, ERROR)
        self.assertEqual(actions(out), ["end"])
        self.assertTrue(any("I missed it." in s for s in says(out)))

    def test_grab_timeout_is_an_error(self):
        f = self.make(grab_timeout=30.0)
        self.go(HANDOVER, now=1.0)
        self.assertEqual(f.on_tick(30.9), [])
        f.on_tick(31.1)
        self.assertEqual(f.state, ERROR)

    def test_stop_cancels_back_to_at_car(self):
        f = self.make()
        self.go(HANDOVER)
        out = f.on_heard("stop", 4.0)
        self.assertEqual(f.state, AT_CAR)
        self.assertEqual(actions(out)[0], "stop")

    def test_other_words_get_please_wait(self):
        f = self.make()
        self.go(HANDOVER)
        out = f.on_heard("go back", 4.0)
        self.assertEqual(f.state, HANDOVER)
        self.assertEqual(says(out), ["Please wait a moment."])


class ReturningTests(FlowCase):
    def test_at_start_drops_the_bag(self):
        f = self.make()
        self.go(RETURNING)
        out = f.on_event("at_start", 5.0)
        self.assertEqual(f.state, DROPPING)
        self.assertEqual(actions(out), ["drop_bag"])

    def test_at_start_without_the_arm_is_done(self):
        f = self.make(use_arm=False)
        self.go(AT_CAR)
        f.on_heard("done", 3.0)
        out = f.on_event("at_start", 5.0)
        self.assertEqual(f.state, DONE)
        self.assertIn("Here is your bag.", says(out))

    def test_stop_waits_and_go_back_resumes(self):
        f = self.make()
        self.go(RETURNING)
        out = f.on_heard("stop", 5.0)
        self.assertEqual(actions(out), ["wait"])
        self.assertEqual(f.state, RETURNING)
        self.assertTrue(f.paused)
        out = f.on_heard("go back", 6.0)
        self.assertEqual(actions(out), ["resume"])
        self.assertFalse(f.paused)

    def test_continue_resumes_after_a_pause(self):
        f = self.make()
        self.go(RETURNING)
        f.on_heard("wait", 5.0)
        out = f.on_heard("continue", 6.0)
        self.assertEqual(actions(out), ["resume"])

    def test_link_lost_while_returning_is_an_error(self):
        f = self.make()
        self.go(RETURNING)
        f.on_event("base_link_lost", 5.0)
        self.assertEqual(f.state, ERROR)

    def test_at_start_outside_returning_is_ignored(self):
        f = self.make()
        self.go(FOLLOWING)
        out = f.on_event("at_start", 2.0)
        self.assertEqual(f.state, FOLLOWING)
        self.assertEqual(actions(out), [])


class DroppingTests(FlowCase):
    def test_drop_done_hands_over_and_is_done(self):
        f = self.make()
        self.go(DROPPING)
        out = f.on_event("drop_done", 6.0)
        self.assertEqual(f.state, DONE)
        self.assertIn("Here is your bag.", says(out))
        self.assertFalse(f.carrying)

    def test_drop_failed_is_an_error(self):
        f = self.make()
        self.go(DROPPING)
        f.on_event("drop_failed|COLLISION|Something is in the way.", 6.0)
        self.assertEqual(f.state, ERROR)

    def test_drop_timeout_is_an_error(self):
        f = self.make(drop_timeout=20.0)
        self.go(DROPPING, now=1.0)
        f.on_tick(21.1)
        self.assertEqual(f.state, ERROR)

    def test_stop_while_dropping_ends_the_run(self):
        f = self.make()
        self.go(DROPPING)
        out = f.on_heard("stop", 6.0)
        self.assertEqual(actions(out), ["stop"])
        self.assertEqual(f.state, DONE)


class ErrorTests(FlowCase):
    def test_error_cmd_can_be_set_to_wait(self):
        f = self.make(error_pi_cmd="wait")
        self.go(FOLLOWING)
        out = f.on_event("base_link_lost", 2.0)
        self.assertEqual(actions(out), ["wait"])

    def test_follow_me_after_an_error_starts_again(self):
        f = self.make()
        self.go(FOLLOWING)
        f.on_event("base_link_lost", 2.0)
        f.on_event("base_link_ok", 3.0)
        out = f.on_heard("follow me", 4.0)
        self.assertEqual(f.state, FOLLOWING)
        self.assertEqual(actions(out), ["follow_me"])

    def test_go_back_after_an_error_tries_the_return(self):
        f = self.make()
        self.go(RETURNING)
        f.on_event("base_link_lost", 5.0)
        f.on_event("base_link_ok", 6.0)
        out = f.on_heard("go back", 7.0)
        self.assertEqual(f.state, RETURNING)
        self.assertEqual(actions(out), ["go_back"])

    def test_link_lost_while_the_base_is_needed_is_an_error(self):
        for state in (FOLLOWING, AT_CAR, HANDOVER, RETURNING):
            self.make()
            self.go(state)
            self.flow.on_event("base_link_lost", 9.0)
            self.assertEqual(self.flow.state, ERROR, state)

    def test_link_lost_after_at_start_is_only_logged(self):
        for use_arm, state in ((True, DROPPING), (True, DONE), (False, DONE)):
            self.make(use_arm=use_arm)
            self.go(AT_CAR)
            self.flow.on_heard("done", 3.0)
            if use_arm:
                self.flow.on_event("grab_success", 4.0)
            self.flow.on_event("at_start", 5.0)
            if state == DONE and use_arm:
                self.flow.on_event("drop_done", 6.0)
            self.assertEqual(self.flow.state, state)
            out = self.flow.on_event("base_link_lost", 6.5)
            self.assertEqual(self.flow.state, state, (use_arm, state))
            self.assertEqual([k for k, _ in out], ["log"], (use_arm, state))

    def test_the_pi_run_ending_does_not_spoil_the_drop(self):
        """returning -> at_start -> link lost (the Pi's run ends) -> drop_done -> done, with the arm."""
        f = self.make(use_arm=True)
        self.go(RETURNING)
        out = f.on_event("at_start", 5.0)
        self.assertEqual(actions(out), ["drop_bag"])
        out = f.on_event("base_link_lost", 5.8)
        self.assertEqual(f.state, DROPPING)
        self.assertEqual(actions(out), [])
        self.assertEqual(f.on_tick(10.0), [])  # no come-back, no timeout yet
        out = f.on_event("drop_done", 14.0)
        self.assertEqual(f.state, DONE)
        self.assertIn("Here is your bag.", says(out))

    def test_unknown_events_are_only_logged(self):
        f = self.make()
        out = f.on_event("something_odd", 1.0)
        self.assertEqual([k for k, _ in out], ["log"])
        self.assertEqual(f.state, READY)

    def test_done_can_start_again(self):
        f = self.make()
        self.go(DONE)
        self.assertEqual(f.state, DONE)
        f.on_heard("follow me", 9.0)
        self.assertEqual(f.state, FOLLOWING)


class ScriptedRunTests(unittest.TestCase):
    """The whole task with a fake publisher, as the node would emit it."""

    def test_full_run(self):
        published = []  # (topic, data)

        def emit(out):
            for kind, val in out:
                topic = {"say": "/tts/say", "cmd": "/robot/command", "state": "/flow/state",
                         "text": "/speech/text"}.get(kind)
                if topic:
                    published.append((topic, val))

        f = HmcFlow()
        emit(f.start(0.0))
        emit(f.on_event("base_link_ok", 0.5))
        emit(f.on_intent({"action": "follow_me", "params": {}, "utterance": "follow me"}, 1.0))
        emit(f.on_event("see_person", 1.5))
        emit(f.on_event("lost_person", 10.0))
        emit(f.on_tick(15.5))
        emit(f.on_event("see_person", 16.0))
        emit(f.on_heard("this is the car", 30.0))
        emit(f.on_heard("done", 40.0))
        emit(f.on_event("grab_success", 43.0))
        for t in range(44, 120, 5):
            emit(f.on_tick(float(t)))
        emit(f.on_event("at_start", 120.0))
        emit(f.on_event("drop_done", 130.0))

        states = [v for t, v in published if t == "/flow/state"]
        self.assertEqual(states, [READY, FOLLOWING, AT_CAR, HANDOVER, RETURNING, DROPPING, DONE])
        acts = [v["action"] for t, v in published if t == "/robot/command"]
        self.assertEqual(acts, ["follow_me", "wait", "carry_bag", "go_back", "drop_bag"])
        spoken = [v for t, v in published if t == "/tts/say"]
        for line in ("I will follow you.", "Please come back.", "Please put the bag in my gripper.",
                     "I will go back to the start now.", "Here is your bag."):
            self.assertIn(line, spoken)
        self.assertEqual(f.state, DONE)


@unittest.skipUnless(importlib.util.find_spec("rclpy"), "rclpy is not installed")
class RosTests(unittest.TestCase):
    def test_the_node_can_be_built(self):
        import rclpy

        node = hmc_flow.make_node()
        try:
            self.assertEqual(node.get_name(), "hmc_flow")
        finally:
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    unittest.main()
