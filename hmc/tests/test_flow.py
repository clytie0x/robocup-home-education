"""The mission executive: happy path, every reason code, and each failure path."""

import json

import pytest

from hmc.flow import LIMIT_S, REASONS, STEPS, HmcFlow, classify

WALL = 1_736_000_000.0  # a fixed instant so log timestamps stay in +07:00
OK = {
    "link": True,
    "odom_fresh": True,
    "motor_ok": True,
    "camera": True,
    "reid": True,
    "agent": True,
    "held": False,
}


def actions(out):
    return [item["action"] for kind, item in out if kind == "cmd"]


def says(out):
    return [text for kind, text in out if kind == "say"]


def fresh(tmp_path, name, **kw):
    flow = HmcFlow(wall=lambda: WALL, **kw)
    flow.open_log(tmp_path / name, "20261119-120000")
    return flow


def begin(tmp_path, name="run", report=None, **kw):
    flow = fresh(tmp_path, name, **kw)
    flow.start(0.0, report=OK if report is None else report)
    return flow


def follow(flow, now=1.0):
    flow.on_heard("follow me", now)
    flow.on_event("locked", now + 0.1)
    return now + 0.1


def to_car(flow, now=3.0):
    flow.on_event({"name": "pose", "x": 4.0, "y": 1.5, "heading": 0.4}, now)
    flow.on_heard("this is the car", now + 0.1)
    flow.on_heard("yes", now + 0.2)
    return now + 0.2


def to_point(flow, now=4.0):
    flow.on_heard("left", now)
    return now


def to_pick(flow, now=5.0):
    flow.on_event("bag_reached|OK|0.31,0.02,top=0.20,yaw=0.10", now)
    return now


def home_with_bag(flow, now=6.0):
    flow.on_event("grab_success", now)
    flow.on_event("at_start", now + 0.1)
    flow.on_event("drop_done", now + 0.2)
    return now + 0.2


def test_classify_task_words():
    assert classify("follow me")[0] == "follow"
    assert classify("this is the car")[0] == "arrive"
    assert classify("we are here")[0] == "arrive"
    assert classify("here")[0] is None
    assert classify("yes")[0] == "yes"
    assert classify("not yet")[0] == "no"
    assert classify("left") == ("side", "left")
    assert classify("the right one") == ("side", "right")
    assert classify("do not stop")[0] is None
    assert classify("please wait")[0] == "wait"
    assert classify("go back")[0] == "back"


def test_happy_path_stores_the_car_and_returns(tmp_path):
    flow = begin(tmp_path, "happy")
    assert flow.state == "READY"
    assert flow.snapshot(0.0)["step_index"] == STEPS.index("READY") + 1
    assert flow.snapshot(0.0)["step_total"] == len(STEPS)
    follow(flow)
    assert flow.state == "FOLLOWING"
    assert flow.left_start
    to_car(flow)
    assert flow.state == "AT_CAR"
    assert flow.car_pose == {"x": 4.0, "y": 1.5, "heading": 0.4}
    to_point(flow)
    assert flow.state == "POINTING"
    assert flow.side == "left"
    to_pick(flow)
    assert flow.state == "PICKING"
    assert flow.bag_at == (0.31, 0.02)
    home_with_bag(flow)
    assert flow.state == "DONE"
    assert flow.outcome == "SUCCESS"
    assert flow.reason["code"] == "SUCCESS"
    assert flow.reason["th"] and flow.reason["fix"]
    snap = flow.snapshot(8.0)
    assert snap["active"] is False
    assert snap["outcome"] == "SUCCESS"
    assert snap["elapsed_s"] == 8.0
    assert snap["remaining_s"] == LIMIT_S - 8.0
    assert snap["step_index"] == len(STEPS)
    text = (tmp_path / "happy" / "events.jsonl").read_text(encoding="utf-8")
    rows = [json.loads(line) for line in text.splitlines()]
    events = [row["event"] for row in rows]
    assert "enter" in events and "exit" in events and "outcome" in events
    for row in rows:
        assert row["ts"].endswith("+07:00")
        assert row["run_id"] == "20261119-120000"
        assert row["src"] == "hmc.flow"
        assert "state" in row and "code" in row and "say" in row
    outcome = rows[-1]
    assert outcome["event"] == "outcome"
    assert outcome["code"] == "SUCCESS"
    assert outcome["outcome"] == "SUCCESS"
    spoken = " ".join(row["say"] for row in rows)
    assert "Are we at the car?" in spoken
    assert "I will look to the left." in spoken
    assert "Here is your bag." in spoken


def test_confirm_no_keeps_following_and_right_is_a_backup(tmp_path):
    flow = begin(tmp_path, "confirm")
    follow(flow)
    flow.on_heard("we are here", 3.0)
    assert flow.state == "CONFIRM_CAR"
    out = flow.on_heard("not yet", 3.2)
    assert flow.state == "FOLLOWING"
    assert "resume" in actions(out)
    flow.on_heard("this is the car", 4.0)
    flow.on_heard("yes", 4.2)
    out = flow.on_heard("right", 4.4)
    assert flow.side == "right"
    cmd = [item for kind, item in out if kind == "cmd"][0]
    assert cmd == {"action": "find_bag", "params": {"side": "right"}}


def test_stop_halts_motion_and_does_not_drive_home(tmp_path):
    flow = begin(tmp_path, "stop")
    follow(flow)
    out = flow.stop(10.0)
    assert flow.state == "ABORTED"
    assert flow.outcome == "ABORTED"
    assert flow.reason["code"] == "OPERATOR_STOP"
    assert "stop" in actions(out)
    assert "go_back" not in actions(out)
    assert flow.reason["en"]
    assert flow.reason["th"]
    # a later base-end must not replace the operator stop
    flow.on_event("base_run_ended|ODOM_STALE|stale", 11.0)
    assert flow.reason["code"] == "OPERATOR_STOP"
    assert flow.outcome == "ABORTED"


def test_time_limit_tries_to_return_then_aborts(tmp_path):
    flow = begin(tmp_path, "time")
    follow(flow, 1.0)
    out = flow.on_tick(LIMIT_S)
    assert flow.state == "RETURNING"
    assert flow.reason["code"] == "TIME_LIMIT"
    assert "go_back" in actions(out)
    flow.on_event("at_start", LIMIT_S + 10)
    assert flow.state == "ABORTED"
    assert flow.outcome == "ABORTED"
    assert flow.reason["code"] == "TIME_LIMIT"


def test_time_limit_before_moving_does_not_drive(tmp_path):
    flow = begin(tmp_path, "time-ready")
    out = flow.on_tick(LIMIT_S)
    assert flow.state == "ABORTED"
    assert "go_back" not in actions(out)
    assert flow.reason["code"] == "TIME_LIMIT"


def test_pick_failure_asks_for_the_bag_then_returns(tmp_path):
    flow = begin(tmp_path, "pick")
    follow(flow)
    to_car(flow)
    to_point(flow)
    to_pick(flow)
    out = flow.on_event("grab_failed|MISSED|missed the lip", 6.0)
    assert flow.state == "POINTING"
    assert flow.reason["code"] == "PICK_MISSED"
    assert "find_bag" in actions(out)
    flow.on_event("bag_reached|OK|0.31,0.02", 7.0)
    out = flow.on_event("grab_failed|MISSED|missed again", 8.0)
    assert flow.state == "HANDOVER"
    assert says(out) == ["Please hand me the bag."]
    out = flow.on_tick(8.0 + 30.0)
    assert flow.state == "RETURNING"
    assert flow.reason["code"] == "PICK_FAILED"
    assert "go_back" in actions(out)
    flow.on_event("at_start", 40.0)
    assert flow.state == "FAILED"
    assert flow.outcome == "FAILED"
    assert flow.reason["code"] == "PICK_FAILED"


def test_handover_done_counts_as_carrying(tmp_path):
    flow = begin(tmp_path, "hand")
    follow(flow)
    to_car(flow)
    now = to_point(flow)
    for _ in range(4):  # three retries, then the handover
        now += 1.0
        flow.on_event("bag_failed|NO_BAG|none", now)
    assert flow.state == "HANDOVER"
    out = flow.on_heard("ready", now + 1)
    assert flow.carrying
    assert flow.state == "RETURNING"
    assert "go_back" in actions(out)


def test_link_loss_waits_then_returns(tmp_path):
    flow = begin(tmp_path, "link")
    follow(flow)
    assert flow.on_event("base_link_lost", 10.0)
    assert flow.state == "FOLLOWING"
    assert flow.on_tick(13.9) == [] or flow.state == "FOLLOWING"
    assert flow.state == "FOLLOWING"
    out = flow.on_tick(14.0)
    assert flow.state == "RETURNING"
    assert flow.reason["code"] == "BASE_LINK_DOWN"
    assert "go_back" in actions(out)


def test_link_back_inside_the_grace_stays(tmp_path):
    flow = begin(tmp_path, "link-ok")
    follow(flow)
    flow.on_event("base_link_lost", 10.0)
    flow.on_event("base_link_ok", 12.0)
    flow.on_tick(20.0)
    assert flow.state == "FOLLOWING"
    assert flow.reason is None


def test_lost_operator_asks_them_back_and_stays(tmp_path):
    flow = begin(tmp_path, "lost")
    follow(flow, 2.0)
    flow.on_event("lost_person", 5.0)
    assert flow.state == "LOST_SEARCH"
    assert flow.on_tick(5.0 + 14.0) == []
    out = flow.on_tick(5.0 + 15.0)
    assert flow.state == "LOST_SEARCH"
    assert flow.running
    assert flow.reason["code"] == "OPERATOR_LOST"
    assert says(out) == ["Please come back."]
    flow.on_event("locked", 30.0)
    assert flow.state == "FOLLOWING"


def test_pause_on_the_way_back_does_not_resume_itself(tmp_path):
    flow = begin(tmp_path, "pause")
    follow(flow)
    to_car(flow)
    to_point(flow)
    to_pick(flow)
    flow.on_event("grab_success", 6.0)
    assert flow.state == "RETURNING"
    out = flow.on_heard("please wait", 7.0)
    assert "wait" in actions(out)
    assert flow.on_tick(7.0 + 19.0) == []
    out = flow.on_tick(7.0 + 20.0)
    assert says(out) == ["Say continue when I can go on."]
    assert actions(out) == []
    out = flow.on_heard("continue", 40.0)
    assert "resume" in actions(out)
    assert flow.state == "RETURNING"


def test_not_pointed_uses_the_spoken_side_before_the_step_back(tmp_path):
    flow = begin(tmp_path, "bags")
    follow(flow)
    to_car(flow)
    to_point(flow)
    out = flow.on_event("bag_failed|NOT_POINTED|no steady pointing in 20 s", 6.0)
    assert flow.state == "POINTING"
    assert flow.reason["code"] == "NOT_POINTED"
    assert says(out) == ["No pointing was seen in 20 s. I will use the left you said."]
    assert out[-1] == ("cmd", {"action": "find_bag", "params": {"side": "left", "again": True}})
    out = flow.on_event("bag_failed|NOT_POINTED|still nothing", 6.5)
    assert "step back" in says(out)[0]
    out = flow.on_event("bag_failed|REFUSED|not now", 7.0)
    assert flow.state == "AT_CAR"
    assert flow.reason["code"] == "BAG_REFUSED"
    assert "go_back" not in actions(out)


def test_not_pointed_without_a_side_still_asks_to_step_back(tmp_path):
    flow = begin(tmp_path, "noside")
    follow(flow)
    to_car(flow)
    flow.on_heard("ready", 4.0)
    out = flow.on_event("bag_failed|NOT_POINTED|too close", 6.0)
    assert flow.reason["code"] == "NOT_POINTED"
    assert "step back" in says(out)[0]
    assert "find_bag" in actions(out)


def test_no_link_on_the_bag_search_does_not_resend_find_bag(tmp_path):
    flow = begin(tmp_path, "nolink")
    follow(flow)
    to_car(flow)
    to_point(flow)
    out = flow.on_event("bag_failed|NO_LINK|the link to the base is down", 6.0)
    assert flow.reason["code"] == "BASE_LINK_DOWN"
    assert flow.reason["fix"] == "Start the Pi base program."
    assert flow.state == "POINTING"
    assert "find_bag" not in actions(out)
    assert "go_back" not in actions(out)
    out = flow.on_event("bag_failed|NO_LINK|still down", 7.0)
    assert says(out)
    assert "find_bag" not in actions(out)
    out = flow.on_event("bag_failed|NO_LINK|still down", 8.0)
    assert says(out) == []
    assert actions(out) == []
    assert flow.link_stuck
    flow.on_event("base_link_ok", 9.0)
    assert flow.reason is None
    out = flow.on_heard("left", 10.0)
    assert "find_bag" in actions(out)
    assert flow.state == "POINTING"


def test_ready_does_not_search_while_the_link_is_down(tmp_path):
    flow = begin(tmp_path, "ready-link")
    follow(flow)
    to_car(flow)
    flow.on_event("base_link_lost", 4.0)
    assert flow.state == "AT_CAR"
    for t in (5.0, 6.0, 7.0):
        out = flow.on_heard("ready", t)
        assert "find_bag" not in actions(out)
        assert "follow_me" not in actions(out)
        assert flow.state == "AT_CAR"
    assert flow.reason["code"] == "BASE_LINK_DOWN"
    assert flow.link_stuck
    flow.on_event("base_link_ok", 8.0)
    out = flow.on_heard("ready", 9.0)
    assert actions(out) == ["find_bag"]
    assert flow.state == "POINTING"


def test_an_unknown_link_does_not_start_following(tmp_path):
    report = {key: value for key, value in OK.items() if key != "link"}
    flow = begin(tmp_path, "unknown", report=report)
    assert flow.state == "READY"
    assert flow.link_ok is None
    out = flow.on_heard("follow me", 1.0)
    assert flow.state == "READY"
    assert actions(out) == []
    assert flow.reason["code"] == "BASE_LINK_DOWN"
    assert "ฐาน" in flow.reason["th"]
    assert flow.reason["fix"] == "Start the Pi base program."
    flow.on_heard("follow me", 2.0)
    out = flow.on_heard("follow me", 3.0)
    assert says(out) == []
    assert flow.link_stuck
    flow.on_event("base_link_ok", 4.0)
    follow(flow, 5.0)
    assert flow.state == "FOLLOWING"
    assert flow.reason is None


def test_drop_without_a_bag_is_not_holding(tmp_path):
    flow = begin(tmp_path, "empty")
    follow(flow)
    to_car(flow)
    to_point(flow)
    to_pick(flow)
    flow.on_event("grab_success", 6.0)
    flow.on_event("arm_empty", 6.5)
    flow.on_event("at_start", 7.0)
    assert flow.state == "FAILED"
    assert flow.reason["code"] == "NOT_HOLDING"


def test_a_pi_end_after_done_does_not_change_the_outcome(tmp_path):
    flow = begin(tmp_path, "done")
    follow(flow)
    to_car(flow)
    to_point(flow)
    to_pick(flow)
    home_with_bag(flow)
    flow.on_event("base_link_lost", 20.0)
    flow.on_tick(30.0)
    flow.on_event("base_run_ended|ODOM_STALE|stale", 31.0)
    assert flow.state == "DONE"
    assert flow.outcome == "SUCCESS"
    assert flow.reason["code"] == "SUCCESS"


def test_the_base_run_ending_at_the_start_does_not_fail_the_drop(tmp_path):
    # the real run_follow reports at_start and ends (DONE) in the same cycle: the arm drops after the base stopped
    flow = begin(tmp_path, "end_at_start")
    follow(flow)
    to_car(flow)
    to_point(flow)
    to_pick(flow)
    flow.on_event("grab_success", 6.0)
    out = flow.on_status({"link": True, "at_start": True, "end": "done: back at the start"}, 6.1)
    assert flow.state == "DROPPING"
    assert actions(out) == ["drop_bag"]
    flow.on_event("base_run_ended|BASE_RUN_ENDED|the run ended", 6.5)
    assert flow.state == "DROPPING"
    flow.on_event("drop_done", 7.0)
    assert (flow.state, flow.outcome) == ("DONE", "SUCCESS")


def test_unknown_precheck_fields_still_start(tmp_path):
    flow = fresh(tmp_path, "dry")
    flow.start(0.0, report=None)
    assert flow.state == "READY"
    assert flow.running


def _reach(tmp_path, code):
    """Drive a fresh executive until `code` is the sticky reason. Returns the flow."""
    if code == "SUCCESS":
        flow = begin(tmp_path, code)
        follow(flow)
        to_car(flow)
        to_point(flow)
        to_pick(flow)
        home_with_bag(flow)
        return flow
    if code in ("NO_LINK", "ODOM_STALE", "NO_MOTOR", "NO_CAMERA", "REID_DOWN", "AGENT_DOWN", "HELD",
                "WHEEL_FAULT_02", "WHEEL_FAULT_03"):
        # Precheck faults end before the robot moves. Wheel codes use the same door.
        report = dict(OK)
        if code == "HELD":
            report["held"] = True
        elif code == "WHEEL_FAULT_02":
            report["wheel_fault"] = "02"
        elif code == "WHEEL_FAULT_03":
            report["wheel_fault"] = "03"
        else:
            key = {
                "NO_LINK": "link",
                "ODOM_STALE": "odom_fresh",
                "NO_MOTOR": "motor_ok",
                "NO_CAMERA": "camera",
                "REID_DOWN": "reid",
                "AGENT_DOWN": "agent",
            }[code]
            report[key] = False
        return begin(tmp_path, code, report=report)
    flow = begin(tmp_path, code)
    if code in ("NOBODY_IN_VIEW", "DARK_FRAME", "REID_FAIL", "AMBIGUOUS", "ENROL_TIMEOUT"):
        flow.on_heard("follow me", 1.0)
        event = {
            "NOBODY_IN_VIEW": "nobody_in_view",
            "DARK_FRAME": "dark_frame",
            "REID_FAIL": "reid_fail",
            "AMBIGUOUS": "ambiguous",
        }.get(code, "nobody_in_view")
        times = 3 if code == "ENROL_TIMEOUT" else 1
        for i in range(times):
            flow.on_event(event, 2.0 + i)
        return flow
    follow(flow)
    if code == "OPERATOR_LOST":
        flow.on_event("lost_person", 5.0)
        flow.on_tick(20.0)
        return flow
    if code == "CAR_UNCONFIRMED":
        flow.on_heard("this is the car", 5.0)
        flow.on_tick(25.0)
        flow.on_tick(45.0)
        flow.on_tick(65.0)
        return flow
    if code == "OPERATOR_STOP":
        flow.stop(6.0)
        return flow
    if code == "TIME_LIMIT":
        flow.on_tick(LIMIT_S)
        return flow
    if code == "BASE_LINK_DOWN":
        flow.on_event("base_link_lost", 10.0)
        flow.on_tick(14.0)
        return flow
    if code == "WHEELS_NOT_TURNING":
        flow.on_event("wheel_fault|not turning|stalled", 10.0)
        return flow
    if code == "BASE_RUN_ENDED":
        flow.on_event("base_run_ended|BASE_RUN_ENDED|the pi run ended", 10.0)
        return flow
    if code == "SLOW_CYCLE":
        flow.on_event("slow_cycle", 10.0)
        return flow
    if code == "ODOM_STALE":
        flow.on_event("odom_stale|ODOM_STALE|0.32 s", 10.0)
        return flow
    # Bag and pick codes need the car.
    to_car(flow)
    to_point(flow)
    bag = {
        "NOT_POINTED": "NOT_POINTED",
        "NO_BAG": "NO_BAG",
        "BLOCKED": "BLOCKED",
        "BAG_TIMEOUT": "TIMEOUT",
        "LOST_BAG": "LOST_BAG",
        "BAG_REFUSED": "REFUSED",
    }
    if code in bag:
        flow.on_event("bag_failed|%s|detail" % bag[code], 8.0)
        return flow
    to_pick(flow)
    if code == "PICK_MISSED":
        flow.on_event("grab_failed|MISSED|miss", 8.0)
        return flow
    if code == "PICK_UNREACHABLE":
        flow.on_event("grab_failed|UNREACHABLE|too far", 8.0)
        return flow
    if code == "PICK_TIMEOUT":
        flow.on_tick(5.0 + 240.0)
        return flow
    if code == "PICK_FAILED":
        flow.on_event("grab_failed|MISSED|miss", 8.0)
        flow.on_event("bag_reached|OK|0.31,0.02", 9.0)
        flow.on_event("grab_failed|MISSED|miss", 10.0)
        flow.on_tick(40.0)
        return flow
    if code == "NOT_HOLDING":
        flow.on_event("grab_success", 8.0)
        flow.on_event("arm_empty", 8.5)
        flow.on_event("at_start", 9.0)
        return flow
    if code == "DROP_FAILED":
        flow.on_event("grab_success", 8.0)
        flow.on_event("at_start", 9.0)
        flow.on_event("drop_failed|FAILED|gripper", 10.0)
        return flow
    if code == "DROP_TIMEOUT":
        flow.on_event("grab_success", 8.0)
        flow.on_event("at_start", 9.0)
        flow.on_tick(9.0 + 60.0)
        return flow
    raise AssertionError(code)


@pytest.mark.parametrize("code", sorted(REASONS))
def test_every_reason_code_has_both_languages_a_fix_and_a_path(tmp_path, code):
    spec = REASONS[code]
    assert spec["en"] and spec["th"] and spec["fix"]
    assert spec["level"] in ("info", "warn", "error")
    assert spec["outcome"] in ("SUCCESS", "FAILED", "ABORTED")
    flow = _reach(tmp_path, code)
    assert flow.reason["code"] == code
    assert flow.reason["en"] == spec["en"]
    assert flow.reason["th"] == spec["th"]
    assert flow.reason["fix"] == spec["fix"]
    rows = [json.loads(line) for line in (tmp_path / code / "events.jsonl").read_text(encoding="utf-8").splitlines()]
    assert any(row["event"] == "reason" and row["code"] == code and row["visible"] for row in rows)


def test_mid_run_odometry_and_wheel_faults_try_to_return(tmp_path):
    flow = begin(tmp_path, "odom")
    follow(flow)
    out = flow.on_event("odom_stale|ODOM_STALE|bad frame", 10.0)
    assert flow.state == "RETURNING"
    assert flow.reason["code"] == "ODOM_STALE"
    assert "go_back" in actions(out)
    flow.on_event("at_start", 20.0)
    assert flow.state == "FAILED"
    assert flow.reason["code"] == "ODOM_STALE"

    flow = begin(tmp_path, "wheel02")
    follow(flow)
    out = flow.on_event("wheel_fault|02|right wheel", 10.0)
    assert flow.reason["code"] == "WHEEL_FAULT_02"
    assert "go_back" in actions(out)
    flow = begin(tmp_path, "wheel03")
    follow(flow)
    flow.on_status({"wheels": "fault 03"}, 10.0)
    assert flow.reason["code"] == "WHEEL_FAULT_03"
    assert flow.state == "RETURNING"


def test_unreachable_pick_and_a_slow_cycle_are_named(tmp_path):
    flow = begin(tmp_path, "far")
    follow(flow)
    to_car(flow)
    to_point(flow)
    to_pick(flow)
    flow.on_event("grab_failed|UNREACHABLE|out of reach", 8.0)
    assert flow.state == "HANDOVER"
    assert flow.reason["code"] == "PICK_UNREACHABLE"
    assert says(flow.on_tick(0.0)) == [] or flow.last_say == "Please hand me the bag."

    flow = begin(tmp_path, "slow")
    follow(flow)
    out = flow.on_event("base_run_ended||slow cycle 0.51 s", 9.0)
    assert flow.reason["code"] == "SLOW_CYCLE"
    assert "go_back" in actions(out)


def test_step_index_follows_the_rulebook_order(tmp_path):
    flow = begin(tmp_path, "steps")
    seen = [flow.snapshot(0.0)["step_index"]]
    follow(flow)
    seen.append(flow.snapshot(2.0)["step_index"])
    to_car(flow)
    seen.append(flow.snapshot(4.0)["step_index"])
    to_point(flow)
    seen.append(flow.snapshot(5.0)["step_index"])
    to_pick(flow)
    seen.append(flow.snapshot(6.0)["step_index"])
    flow.on_event("grab_success", 7.0)
    seen.append(flow.snapshot(7.0)["step_index"])
    flow.on_event("at_start", 8.0)
    seen.append(flow.snapshot(8.0)["step_index"])
    flow.on_event("drop_done", 9.0)
    seen.append(flow.snapshot(9.0)["step_index"])
    assert seen == sorted(seen)
    assert len(set(seen)) == len(seen)
    assert seen[-1] == len(STEPS)


def test_return_budget_ends_a_return_that_never_arrives(tmp_path):
    flow = begin(tmp_path, "budget")
    follow(flow)
    flow.on_tick(LIMIT_S)
    assert flow.state == "RETURNING"
    flow.on_tick(LIMIT_S + 179.0)
    assert flow.state == "RETURNING"
    flow.on_tick(LIMIT_S + 180.0)
    assert flow.state == "ABORTED"
    assert flow.reason["code"] == "TIME_LIMIT"
