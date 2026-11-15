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
    PICKING,
    POINTING,
    READY,
    RETURNING,
    HmcFlow,
)

REACHED = "bag_reached|OK|0.310,0.015"


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
        if state == POINTING:
            return
        if state == AT_CAR:
            f.on_heard("stop", now)
            return
        f.on_event(REACHED, now)
        if state in (PICKING, RETURNING) and not f.use_arm:
            return
        if state == PICKING:
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
            ("We are here", "stop"),
            ("we're here", "stop"),
            ("Here we are.", "stop"),
            ("here", None),
            ("come here", None),
            ("over here", None),
            ("this one here", None),
            ("wait here", "stop"),
            ("please wait", "stop"),
            ("wait for me", "stop"),
            ("wait, I want to show you something", None),
            ("you have to wait for the bag", None),
            ("I don't know, stop", "stop"),
            ("I don't know. Stop.", "stop"),
            ("okay, stop", "stop"),
            ("okay I can see the bag over there", None),
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


class EarlyInputTests(unittest.TestCase):
    """Input before start() (no tts_node listening yet): start the flow, then handle it. Nothing is lost."""

    def test_follow_me_heard_before_start_starts_and_follows(self):
        f = HmcFlow()
        out = f.on_heard("follow me", 1.0)
        self.assertEqual(f.state, FOLLOWING)
        self.assertEqual([v for k, v in out if k == "state"], [READY, FOLLOWING])
        self.assertLess(out.index(("state", READY)), out.index(("cmd", {"action": "follow_me", "params": {}})))
        self.assertEqual(actions(out), ["follow_me"])
        self.assertEqual(forwarded(out), [])

    def test_other_speech_before_start_starts_and_goes_to_the_brain(self):
        f = HmcFlow()
        out = f.on_heard("hello robot", 1.0)
        self.assertEqual(f.state, READY)
        self.assertEqual(out[0], ("state", READY))
        self.assertEqual(forwarded(out), ["hello robot"])

    def test_intent_before_start_starts_and_follows(self):
        f = HmcFlow()
        out = f.on_intent({"action": "follow_me", "params": {}}, 1.0)
        self.assertEqual(f.state, FOLLOWING)
        self.assertEqual(out[0], ("state", READY))
        self.assertEqual(actions(out), ["follow_me"])

    def test_event_before_start_is_noted_but_does_not_start(self):
        """base_bridge sends base_link_ok on connect, usually before tts_node: don't speak to nobody."""
        f = HmcFlow()
        out = f.on_event("base_link_ok", 1.0)
        self.assertIsNone(f.state)
        self.assertEqual([k for k, _ in out], ["log"])
        self.assertTrue(f.link_ok)
        out = f.on_event("base_link_lost", 1.5)
        self.assertIsNone(f.state)
        self.assertEqual([k for k, _ in out], ["log"])
        self.assertFalse(f.link_ok)

    def test_start_only_once(self):
        f = HmcFlow()
        f.on_heard("hello robot", 1.0)
        out = f.on_heard("follow me", 2.0)
        self.assertNotIn(("state", READY), out)
        self.assertEqual(f.state, FOLLOWING)

    def test_tick_before_start_does_nothing(self):
        f = HmcFlow()
        self.assertEqual(f.on_tick(100.0), [])
        self.assertIsNone(f.state)


class ShouldStartTests(unittest.TestCase):
    def test_starts_when_tts_listens(self):
        self.assertTrue(hmc_flow.should_start(1, 0.0))

    def test_waits_for_tts_up_to_start_wait(self):
        self.assertFalse(hmc_flow.should_start(0, hmc_flow.START_WAIT - 0.1))
        self.assertTrue(hmc_flow.should_start(0, hmc_flow.START_WAIT))

    def test_start_wait_is_about_3_s(self):
        self.assertTrue(1.0 <= hmc_flow.START_WAIT <= 5.0)


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
    def test_arrival_words_send_wait_then_ask_for_the_pointing(self):
        for words in ("stop", "wait", "this is the car", "here we are", "we are here", "we're here"):
            self.make()
            self.go(FOLLOWING)
            out = self.flow.on_heard(words, 2.0)
            self.assertEqual(self.flow.state, POINTING, words)
            self.assertEqual(cmds(out), [{"action": "wait", "params": {}}, {"action": "find_bag", "params": {}}], words)
            self.assertEqual(says(out), ["Please point at the bag."], words)

    def test_stop_intent_goes_to_pointing(self):
        f = self.make()
        self.go(FOLLOWING)
        out = f.on_intent({"action": "stop"}, 2.0)
        self.assertEqual(f.state, POINTING)
        self.assertEqual(actions(out), ["wait", "find_bag"])

    def test_go_back_while_following_stops_at_the_car_first(self):
        f = self.make()
        self.go(FOLLOWING)
        out = f.on_heard("go back", 2.0)
        self.assertEqual(f.state, POINTING)
        self.assertEqual(actions(out), ["wait", "find_bag"])

    def test_lost_person_for_20_s_asks_to_come_back_once(self):
        """Asking the operator back costs -150 (rules): not after a short gap the Pi's search covers."""
        f = self.make()
        self.go(FOLLOWING)
        f.on_event("see_person", 1.5)
        f.on_event("lost_person", 2.0)
        self.assertEqual(says(f.on_tick(7.0)), [])  # the old 5 s
        self.assertEqual(says(f.on_tick(21.9)), [])
        self.assertEqual(says(f.on_tick(22.0)), ["Please come back."])
        self.assertEqual(says(f.on_tick(30.0)), [])
        self.assertEqual(f.state, FOLLOWING)

    def test_lost_person_for_6_s_while_the_pi_searches_says_nothing(self):
        f = self.make()
        self.go(FOLLOWING)
        f.on_event("see_person", 1.5)
        f.on_event("lost_person", 2.0)
        for t in (2.5, 4.0, 5.0, 6.0, 7.0, 8.0):
            searching = t >= 4.0  # the bridge says search 2 s after the lock went
            self.assertEqual(says(f.on_status({"link": True, "phase": "FOLLOW", "locked": False,
                                               "search": searching}, t)), [], t)
            self.assertEqual(says(f.on_tick(t)), [], t)

    def test_asks_when_the_pi_search_ended_without_a_lock(self):
        f = self.make()
        self.go(FOLLOWING)
        f.on_event("see_person", 1.5)
        f.on_event("lost_person", 2.0)
        f.on_status({"link": True, "phase": "FOLLOW", "locked": False, "search": True}, 4.0)
        out = f.on_status({"link": True, "phase": "FOLLOW", "locked": False, "search": False}, 9.0)
        self.assertEqual(says(out), ["Please come back."])
        self.assertEqual(says(f.on_tick(22.0)), [])  # once
        self.assertEqual(actions(out), [])

    def test_a_search_that_ends_with_a_lock_asks_nothing(self):
        f = self.make()
        self.go(FOLLOWING)
        f.on_event("see_person", 1.5)
        f.on_event("lost_person", 2.0)
        f.on_status({"link": True, "phase": "FOLLOW", "locked": False, "search": True}, 4.0)
        self.assertEqual(says(f.on_status({"link": True, "phase": "FOLLOW", "locked": True, "search": False}, 6.0)), [])
        f.on_event("see_person", 6.0)
        self.assertEqual(says(f.on_tick(30.0)), [])

    def test_seen_again_cancels_the_come_back(self):
        f = self.make()
        self.go(FOLLOWING)
        f.on_event("see_person", 1.5)
        f.on_event("lost_person", 2.0)
        f.on_event("see_person", 4.0)
        self.assertEqual(says(f.on_tick(30.0)), [])

    def test_never_locked_after_follow_also_asks(self):
        f = self.make()
        self.go(FOLLOWING, now=1.0)
        self.assertEqual(says(f.on_tick(20.9)), [])
        self.assertEqual(says(f.on_tick(21.0)), ["Please come back."])

    def test_lost_person_is_not_an_error(self):
        f = self.make()
        self.go(FOLLOWING)
        out = f.on_event("lost_person", 2.0)
        self.assertEqual(f.state, FOLLOWING)
        self.assertEqual(actions(out), [])

    def test_link_lost_while_following_is_an_error_after_the_grace(self):
        f = self.make()
        self.go(FOLLOWING)
        out = f.on_event("base_link_lost", 2.0)
        self.assertEqual((f.state, actions(out), says(out)), (FOLLOWING, [], []))
        out = f.on_tick(6.0)
        self.assertEqual(f.state, ERROR)
        self.assertEqual(actions(out), ["end"])
        self.assertEqual(len(says(out)), 1)


class PointingTests(FlowCase):
    def test_bag_chosen_is_said(self):
        f = self.make()
        self.go(POINTING)
        out = f.on_event("bag_chosen|RIGHT|I will take the right bag, on your right.", 3.0)
        self.assertEqual(says(out), ["I will take the right bag, on your right."])
        self.assertEqual(f.state, POINTING)
        self.assertEqual(actions(out), [])

    def test_bag_reached_picks_it_where_the_pi_measured_it(self):
        f = self.make()
        self.go(POINTING)
        out = f.on_event(REACHED, 20.0)
        self.assertEqual(f.state, PICKING)
        self.assertEqual(cmds(out), [{"action": "pick_bag", "params": {"x": 0.31, "y": 0.015}}])
        self.assertEqual(f.deadline, 20.0 + 90.0)  # the arm moves slowly

    def test_without_the_arm_bag_reached_says_so_and_goes_back(self):
        f = self.make(use_arm=False)
        self.go(POINTING)
        out = f.on_event("bag_reached|ESTIMATED|0.300,0.010", 20.0)
        self.assertEqual(f.state, RETURNING)
        self.assertEqual(actions(out), ["go_back"])
        self.assertIn("I would pick up the bag now.", says(out))
        self.assertIn("I will go back to the start now.", says(out))

    def test_bag_failed_says_why_and_asks_to_point_again(self):
        for code, why in (("NO_BAG", "I see no bag there."),
                          ("BLOCKED", "Something is in my way."), ("TIMEOUT", "That took me too long."),
                          ("LOST_BAG", "I lost sight of the bag.")):
            f = self.make()
            self.go(POINTING)
            out = f.on_event("bag_failed|%s|details for the log" % code, 10.0)
            self.assertEqual(f.state, POINTING, code)
            self.assertEqual(says(out), [why + " Please point at the bag again."], code)
            self.assertEqual(cmds(out), [{"action": "find_bag", "params": {}}], code)

    def test_not_pointed_asks_to_step_back_before_the_retry(self):
        """The camera sees the shoulder only from about 2.2 m."""
        f = self.make()
        self.go(POINTING)
        out = f.on_event("bag_failed|NOT_POINTED|no steady pointing in 20 s", 10.0)
        self.assertEqual(f.state, POINTING)
        self.assertEqual(says(out), ["Please step back two steps and point at the bag.",
                                     "I did not see you point. Please point at the bag again."])
        self.assertEqual(cmds(out), [{"action": "find_bag", "params": {}}])
        for i in range(3):
            out = f.on_event("bag_failed|NOT_POINTED|again", 20.0 + i)
        self.assertEqual(f.state, AT_CAR)  # the fourth in a row
        self.assertEqual(says(out), ["Please step back two steps and point at the bag.",
                                     "I did not see you point. Please point at the bag, then say ready."])

    def test_here_words_do_not_cancel_the_bag_search(self):
        f = self.make()
        self.go(POINTING)
        for words in ("come here", "over here", "this one here", "okay I am pointing at it now"):
            out = f.on_heard(words, 5.0)
            self.assertEqual(f.state, POINTING, words)
            self.assertEqual(actions(out), [], words)

    def test_after_three_failures_in_a_row_it_waits_for_ready(self):
        f = self.make()
        self.go(POINTING)
        for i in range(3):
            f.on_event("bag_failed|NO_BAG|none", 10.0 + i)
            self.assertEqual(f.state, POINTING)
        out = f.on_event("bag_failed|NO_BAG|none", 14.0)
        self.assertEqual(f.state, AT_CAR)
        self.assertEqual(actions(out), [])
        self.assertEqual(says(out), ["I see no bag there. Please point at the bag, then say ready."])
        out = f.on_heard("ready", 20.0)
        self.assertEqual(f.state, POINTING)
        self.assertEqual(actions(out), ["find_bag"])
        f.on_event("bag_failed|NO_BAG|none", 30.0)
        self.assertEqual(f.state, POINTING)  # the count starts again

    def test_a_refused_or_cancelled_bag_is_not_retried_by_itself(self):
        for code in ("REFUSED", "NO_LINK", "CANCELLED", "SOMETHING_NEW"):
            f = self.make()
            self.go(POINTING)
            out = f.on_event("bag_failed|%s|why" % code, 10.0)
            self.assertEqual(f.state, AT_CAR, code)
            self.assertEqual(actions(out), [], code)

    def test_stop_cancels_the_bag_task(self):
        f = self.make()
        self.go(POINTING)
        out = f.on_heard("stop", 5.0)
        self.assertEqual(f.state, AT_CAR)
        self.assertEqual(actions(out), ["wait"])  # the Pi's wait cancels its bag task
        self.assertEqual(says(out), ["Okay, cancelled. Please point at the bag, then say ready."])
        self.assertEqual([k for k, _ in f.on_event("bag_failed|CANCELLED|wait came", 5.2) if k != "log"], [])

    def test_follow_me_while_pointing_resumes_following(self):
        f = self.make()
        self.go(POINTING)
        out = f.on_heard("follow me", 5.0)
        self.assertEqual(f.state, FOLLOWING)
        self.assertEqual(actions(out), ["resume"])

    def test_other_task_words_ask_for_the_pointing(self):
        f = self.make()
        self.go(POINTING)
        for words in ("take it back", "done"):
            out = f.on_heard(words, 5.0)
            self.assertEqual(says(out), ["Please point at the bag."], words)
            self.assertEqual(actions(out), [], words)
            self.assertEqual(f.state, POINTING)

    def test_no_answer_in_time_cancels_and_waits(self):
        f = self.make(bag_timeout=180.0)
        self.go(POINTING, now=1.0)
        self.assertEqual(f.on_tick(180.9), [])
        out = f.on_tick(181.1)
        self.assertEqual(f.state, AT_CAR)
        self.assertEqual(actions(out), ["wait"])

    def test_a_bad_reached_line_is_not_a_pick(self):
        f = self.make()
        self.go(POINTING)
        out = f.on_event("bag_reached|OK|nowhere", 5.0)
        self.assertEqual(f.state, AT_CAR)
        self.assertNotIn("pick_bag", actions(out))

    def test_bag_events_outside_pointing_are_only_logged(self):
        f = self.make()
        self.go(FOLLOWING)
        for ev in ("bag_chosen|LEFT|I will take the left bag, on your left.", REACHED, "bag_failed|NO_BAG|x"):
            out = f.on_event(ev, 3.0)
            self.assertEqual([k for k, _ in out], ["log"], ev)
        self.assertEqual(f.state, FOLLOWING)


class AtCarTests(FlowCase):
    def test_ready_okay_or_take_it_back_look_for_the_bag(self):
        for words in ("ready", "okay", "done", "take it back"):
            self.make()
            self.go(AT_CAR)
            out = self.flow.on_heard(words, 6.0)
            self.assertEqual(self.flow.state, POINTING, words)
            self.assertEqual(actions(out), ["find_bag"], words)

    def test_carry_bag_intent_looks_for_the_bag(self):
        f = self.make()
        self.go(AT_CAR)
        f.on_intent({"action": "carry_bag"}, 6.0)
        self.assertEqual(f.state, POINTING)

    def test_follow_me_at_the_car_resumes_following(self):
        f = self.make()
        self.go(AT_CAR)
        out = f.on_heard("follow me", 6.0)
        self.assertEqual(f.state, FOLLOWING)
        self.assertEqual(actions(out), ["resume"])

    def test_asks_again_after_silence(self):
        f = self.make(reask_s=15.0)
        self.go(AT_CAR, now=1.0)
        self.assertEqual(says(f.on_tick(15.9)), [])
        self.assertEqual(says(f.on_tick(16.1)), ["Please point at the bag, then say ready."])

    def test_unrelated_speech_is_forwarded(self):
        f = self.make()
        self.go(AT_CAR)
        out = f.on_heard("what is your name", 6.0)
        self.assertEqual(forwarded(out), ["what is your name"])
        self.assertEqual(f.state, AT_CAR)


class PickingTests(FlowCase):
    def test_grab_success_starts_the_return(self):
        f = self.make()
        self.go(PICKING)
        out = f.on_event("grab_success", 40.0)
        self.assertEqual(f.state, RETURNING)
        self.assertEqual(actions(out), ["go_back"])
        self.assertIn("I will go back to the start now.", says(out))
        self.assertTrue(f.carrying)

    def test_grab_failed_holding_counts_as_success(self):
        f = self.make()
        self.go(PICKING)
        f.on_event("grab_failed|HOLDING|I already hold something.", 40.0)
        self.assertEqual(f.state, RETURNING)

    def test_a_missed_grab_has_the_bag_measured_again_once(self):
        for code in ("MISSED", "NOT_FOUND"):
            f = self.make()
            self.go(PICKING)
            out = f.on_event("grab_failed|%s|I missed it." % code, 40.0)
            self.assertEqual(f.state, POINTING, code)
            self.assertEqual(cmds(out), [{"action": "find_bag", "params": {"again": True}}], code)
            out = f.on_event("bag_reached|OK|0.305,0.012", 50.0)
            self.assertEqual(cmds(out), [{"action": "pick_bag", "params": {"x": 0.305, "y": 0.012}}])
            out = f.on_event("grab_failed|%s|I missed it again." % code, 90.0)
            self.assertEqual(f.state, ERROR, code)
            self.assertTrue(any("I missed it again." in t for t in says(out)))

    def test_other_grab_failures_are_an_error_with_the_arm_message(self):
        f = self.make()
        self.go(PICKING)
        out = f.on_event("grab_failed|COLLISION|Something is in the way.", 40.0)
        self.assertEqual(f.state, ERROR)
        self.assertEqual(actions(out), ["end"])
        self.assertTrue(any("Something is in the way." in t for t in says(out)))

    def test_the_grasp_reading_is_never_said(self):
        # bag_arm appends gripper_pos and gripper_load for the log (lane C); the flow says only the words
        f = self.make()
        self.go(PICKING)
        out = f.on_event("grab_failed|COLLISION|Something is in the way. [gripper_pos=0.044 gripper_load=12]", 40.0)
        self.assertTrue(any(t.startswith("Sorry. Something is in the way.") for t in says(out)), says(out))
        self.assertFalse(any("gripper" in t for t in says(out)), says(out))

    def test_grab_timeout_is_an_error(self):
        f = self.make(grab_timeout=90.0)
        self.go(PICKING, now=1.0)
        self.assertEqual(f.on_tick(90.9), [])
        f.on_tick(91.1)
        self.assertEqual(f.state, ERROR)

    def test_stop_cancels_back_to_at_car(self):
        f = self.make()
        self.go(PICKING)
        out = f.on_heard("stop", 40.0)
        self.assertEqual(f.state, AT_CAR)
        self.assertEqual(actions(out)[0], "stop")

    def test_other_words_get_please_wait(self):
        f = self.make()
        self.go(PICKING)
        out = f.on_heard("go back", 40.0)
        self.assertEqual(f.state, PICKING)
        self.assertEqual(says(out), ["Please wait a moment."])


class ReturningTests(FlowCase):
    def test_at_start_drops_the_bag(self):
        f = self.make()
        self.go(RETURNING)
        out = f.on_event("at_start", 5.0)
        self.assertEqual(f.state, DROPPING)
        self.assertEqual(cmds(out), [{"action": "drop_bag", "params": {"x": 0.31, "y": 0.0}}])

    def test_at_start_without_the_arm_is_done(self):
        f = self.make(use_arm=False)
        self.go(RETURNING)
        self.assertEqual(f.state, RETURNING)
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

    def test_a_pause_on_the_way_back_never_moves_by_itself(self):
        # review 2026-11-15: a stop on the way back may be for the cable or a foot; after 20 s the robot only
        # asks, it drives on when told
        for words in ("stop", "wait"):
            f = self.make()
            self.go(RETURNING)
            f.on_heard(words, 5.0)
            self.assertTrue(f.paused, words)
            self.assertEqual(f.on_tick(24.9), [], words)
            out = f.on_tick(25.0)
            self.assertEqual(cmds(out), [], words)
            self.assertEqual(says(out), ["Say continue when I can go on."], words)
            self.assertTrue(f.paused, words)
            self.assertEqual(f.on_tick(50.0), [], words)  # asked once
            self.assertEqual(cmds(f.on_heard("continue", 51.0)), [{"action": "resume", "params": {}}], words)

    def test_wait_in_a_longer_sentence_pauses_the_way_back(self):
        # review 2026-11-15: "robot please wait a moment" drove on; in following a long "wait" stays no stop
        # (there stop means "this is the car")
        for words in ("robot please wait a moment", "wait a second please", "you need to wait",
                      "whoa wait wait wait wait", "please wait for me"):
            f = self.make()
            self.go(RETURNING)
            out = f.on_heard(words, 5.0)
            self.assertTrue(f.paused, words)
            self.assertEqual(cmds(out), [{"action": "wait", "params": {}}], words)
        f = self.make()
        self.go(RETURNING)
        f.on_heard("don't wait, keep going", 5.0)
        self.assertFalse(f.paused)
        self.assertIsNone(hmc_flow.classify("wait, I want to show you something"))

    def test_a_pause_resumed_by_voice_is_not_resumed_again(self):
        f = self.make()
        self.go(RETURNING)
        f.on_heard("stop", 5.0)
        f.on_heard("go back", 10.0)
        self.assertEqual(f.on_tick(30.0), [])

    def test_link_lost_while_returning_is_an_error(self):
        f = self.make()
        self.go(RETURNING)
        f.on_event("base_link_lost", 5.0)
        f.on_tick(9.0)
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
        f.on_event("base_link_lost", 2.0)
        out = f.on_tick(6.0)
        self.assertEqual(actions(out), ["wait"])

    def test_follow_me_after_an_error_starts_again(self):
        f = self.make()
        self.go(FOLLOWING)
        f.on_event("base_link_lost", 2.0)
        f.on_tick(6.0)
        f.on_event("base_link_ok", 7.0)
        out = f.on_heard("follow me", 8.0)
        self.assertEqual(f.state, FOLLOWING)
        self.assertEqual(actions(out), ["follow_me"])

    def test_go_back_after_an_error_tries_the_return(self):
        f = self.make()
        self.go(RETURNING)
        f.on_event("base_link_lost", 5.0)
        f.on_tick(9.0)
        f.on_event("base_link_ok", 10.0)
        out = f.on_heard("go back", 11.0)
        self.assertEqual(f.state, RETURNING)
        self.assertEqual(actions(out), ["go_back"])

    def test_link_lost_while_the_base_is_needed_is_an_error(self):
        for state in (FOLLOWING, POINTING, AT_CAR, PICKING, RETURNING):
            self.make()
            self.go(state)
            self.flow.on_event("base_link_lost", 9.0)
            self.flow.on_tick(14.0)  # lost for 5 s
            self.assertEqual(self.flow.state, ERROR, state)

    def test_link_lost_after_at_start_is_only_logged(self):
        for use_arm, state in ((True, DROPPING), (True, DONE), (False, DONE)):
            self.make(use_arm=use_arm)
            self.go(RETURNING)
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


class LinkGraceTests(FlowCase):
    """A short link drop: the bridge restores the Pi's intent itself, so the flow waits LINK_GRACE_S quietly."""

    def test_grace_is_4_s(self):
        self.assertEqual(hmc_flow.LINK_GRACE_S, 4.0)

    def test_lost_then_ok_after_2_s_keeps_the_state(self):
        for state in (FOLLOWING, POINTING, AT_CAR, PICKING, RETURNING):
            self.make()
            self.go(state)
            out = self.flow.on_event("base_link_lost", 9.0)
            self.assertEqual((says(out), actions(out)), ([], []), state)
            for t in (9.5, 10.0, 11.0):
                self.assertEqual([k for k, _ in self.flow.on_tick(t) if k != "log"], [], state)
            out = self.flow.on_event("base_link_ok", 11.0)
            self.assertEqual((says(out), actions(out)), ([], []), state)
            self.flow.on_tick(14.0)
            self.flow.on_tick(20.0)
            self.assertEqual(self.flow.state, state, state)

    def test_lost_for_5_s_is_an_error(self):
        for state in (FOLLOWING, POINTING, AT_CAR, PICKING, RETURNING):
            self.make()
            self.go(state)
            self.flow.on_event("base_link_lost", 9.0)
            self.assertEqual(self.flow.on_tick(12.9), [], state)
            self.assertEqual(self.flow.state, state, state)
            out = self.flow.on_tick(14.0)
            self.assertEqual(self.flow.state, ERROR, state)
            self.assertEqual(actions(out), ["end"], state)
            self.assertTrue(any("connection to my wheels" in t for t in says(out)), state)

    def test_nothing_is_said_while_waiting_for_the_link(self):
        f = self.make()
        self.go(FOLLOWING)
        f.on_event("lost_person", 2.0)
        f.on_event("base_link_lost", 21.0)
        self.assertEqual(f.on_tick(22.5), [])  # the come-back was due at 22.0

    def test_a_second_drop_starts_its_own_grace(self):
        f = self.make()
        self.go(FOLLOWING)
        f.on_event("base_link_lost", 2.0)
        f.on_event("base_link_ok", 4.0)
        f.on_event("base_link_lost", 5.0)
        f.on_tick(8.5)
        self.assertEqual(f.state, FOLLOWING)
        f.on_tick(9.0)
        self.assertEqual(f.state, ERROR)


class StartStatusTests(unittest.TestCase):
    """A restarted flow reads one /base/status first, so it knows the link and the person as the Pi does."""

    def test_should_start_waits_up_to_1_s_for_a_status(self):
        self.assertFalse(hmc_flow.should_start(1, 0.5, status_seen=False))
        self.assertTrue(hmc_flow.should_start(1, 0.5, status_seen=True))
        self.assertTrue(hmc_flow.should_start(1, hmc_flow.STATUS_WAIT, status_seen=False))
        self.assertEqual(hmc_flow.STATUS_WAIT, 1.0)

    def test_a_status_before_start_sets_the_person_and_the_link(self):
        f = HmcFlow()
        out = f.on_status({"link": True, "phase": "FOLLOW", "locked": True, "search": False}, 0.5)
        self.assertEqual([k for k, _ in out if k != "log"], [])
        self.assertIsNone(f.state)
        self.assertTrue(f.link_ok)
        self.assertTrue(f.person_visible)
        f.start(1.0)
        f.on_heard("follow me", 2.0)
        self.assertIsNone(f.lost_since)
        self.assertEqual(says(f.on_tick(40.0)), [])  # the person is not taken for lost

    def test_a_lost_link_status_before_start_refuses_follow(self):
        f = HmcFlow()
        f.on_status({"link": False}, 0.5)
        self.assertFalse(f.link_ok)
        f.start(1.0)
        out = f.on_heard("follow me", 2.0)
        self.assertEqual((f.state, actions(out)), (READY, []))

    def test_only_the_first_status_sets_the_person(self):
        """After it the bridge's debounced see_person and lost_person decide."""
        f = HmcFlow()
        f.on_status({"link": True, "phase": "FOLLOW", "locked": True}, 0.5)
        f.on_status({"link": True, "phase": "FOLLOW", "locked": False}, 0.7)
        self.assertTrue(f.person_visible)


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
        emit(f.on_tick(30.0))  # 20 s lost
        emit(f.on_event("see_person", 30.2))
        emit(f.on_heard("this is the car", 31.0))
        emit(f.on_event("bag_failed|NOT_POINTED|no steady pointing in 20 s", 50.5))
        emit(f.on_event("bag_chosen|LEFT|I will take the left bag, on your left.", 55.0))
        for t in range(56, 85, 5):
            emit(f.on_tick(float(t)))
        emit(f.on_event("bag_reached|OK|0.312,0.016", 85.0))
        emit(f.on_event("grab_success", 140.0))
        for t in range(141, 220, 5):
            emit(f.on_tick(float(t)))
        emit(f.on_event("at_start", 220.0))
        emit(f.on_event("drop_done", 230.0))

        states = [v for t, v in published if t == "/flow/state"]
        self.assertEqual(states, [READY, FOLLOWING, POINTING, POINTING, PICKING, RETURNING, DROPPING, DONE])
        commands = [v for t, v in published if t == "/robot/command"]
        self.assertEqual([v["action"] for v in commands],
                         ["follow_me", "wait", "find_bag", "find_bag", "pick_bag", "go_back", "drop_bag"])
        self.assertEqual(commands[4]["params"], {"x": 0.312, "y": 0.016})
        spoken = [v for t, v in published if t == "/tts/say"]
        for line in ("I will follow you.", "Please come back.", "Please point at the bag.",
                     "I did not see you point. Please point at the bag again.",
                     "I will take the left bag, on your left.",
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
