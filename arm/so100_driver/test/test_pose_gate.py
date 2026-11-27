"""Pose reconciliation, the confidence gate, and the pick-trial JSON writer."""
import json
import os

import pytest

from so100_driver.bag_camera import (
    CONF_MIN,
    choose_grasp_xy,
    reconcile_edges,
    rim_span,
)
from so100_driver.pick_trial import (
    PhaseClock,
    build_trial,
    camera_params_hash,
    ict_timestamp,
    new_trial_id,
    read_commit,
    write_trial,
)


def test_agreeing_edges_are_averaged():
    got = reconcile_edges((0.42, 0.01), (0.40, -0.01))
    assert got["method"] == "mean"
    assert got["centre"] == pytest.approx((0.41, 0.0))
    assert got["spread"] == pytest.approx(0.02828, abs=1e-4)
    assert got["confidence"] > 0.8


def test_four_cm_is_still_an_agreement():
    got = reconcile_edges((0.44, 0.0), (0.40, 0.0))
    assert got["method"] == "mean"
    assert got["centre"] == pytest.approx((0.42, 0.0))
    assert got["confidence"] == pytest.approx(0.78)


def test_eight_cm_disagreement_falls_under_the_gate():
    got = reconcile_edges((0.48, 0.0), (0.40, 0.0))
    assert got["method"] == "foot"
    assert got["centre"] == pytest.approx((0.40, 0.0))
    assert got["confidence"] == pytest.approx(0.375)
    assert got["confidence"] < CONF_MIN


def test_five_cm_disagreement_trusts_the_foot_and_stays_above_the_gate():
    # Audit 19 Nov: the rim estimate and the foot estimate were about 5 cm apart.
    # The foot meets the known floor, so it wins, and the pick may still use it.
    got = reconcile_edges((0.46, 0.0), (0.41, 0.0))
    assert got["method"] == "foot"
    assert got["centre"] == pytest.approx((0.41, 0.0))
    assert got["spread"] == pytest.approx(0.05)
    assert got["confidence"] >= CONF_MIN


def test_a_wide_disagreement_is_not_confident():
    got = reconcile_edges((0.30, 0.0), (0.50, 0.0))
    assert got["method"] == "foot"
    assert got["confidence"] < CONF_MIN


def test_one_edge_has_a_documented_confidence():
    foot = reconcile_edges(None, (0.43, 0.02))
    rim = reconcile_edges((0.43, 0.02), None)
    assert foot["method"] == "foot" and foot["confidence"] == pytest.approx(0.80)
    assert rim["method"] == "rim" and rim["confidence"] < CONF_MIN     # alone it never replaces the goal
    assert reconcile_edges(None, None)["confidence"] == 0.0


def test_confident_camera_wins_over_the_requested_xy():
    # The console sent a fixed 0.36 m. The camera saw the bag at 0.41-0.46.
    xy, source = choose_grasp_xy((0.43, 0.01), 0.70, (0.36, 0.0))
    assert source == "camera"
    assert xy == pytest.approx((0.43, 0.01))


def test_low_confidence_falls_back_to_the_request_and_the_api_keeps_xy():
    xy, source = choose_grasp_xy((0.55, -0.04), 0.2, (0.36, 0.02))
    assert source == "request"
    assert xy == pytest.approx((0.36, 0.02))
    xy, source = choose_grasp_xy(None, 0.0, (0.36, -0.01))
    assert source == "request"
    assert xy == pytest.approx((0.36, -0.01))


def test_rim_span_is_the_top_edge_not_the_wide_foot():
    import numpy as np
    tops = np.array([
        [100, 200], [180, 198], [260, 202], [340, 200],  # the rim
        [40, 420], [400, 430],                           # the foot, much wider
    ], float)
    assert rim_span(tops, 200, 180, 440) == pytest.approx((100.0, 340.0))


def test_ict_timestamp_is_seven_hours_ahead_of_utc():
    assert ict_timestamp(0) == "1970-01-01T07:00:00+07:00"
    assert new_trial_id(0, 42, 3) == "19700101T070000-42-3"


def test_camera_hash_ignores_key_order():
    a = camera_params_hash({"fx": 805.7, "z": 0.89, "pitch_deg": 61.37})
    b = camera_params_hash({"pitch_deg": 61.37, "z": 0.89, "fx": 805.7})
    assert a == b
    assert len(a) == 16
    assert a != camera_params_hash({"fx": 100.0})


def test_read_commit_prefers_the_env_then_the_file_then_git(tmp_path):
    path = tmp_path / "VERSION.json"
    path.write_text(json.dumps({"commit": "abc1234"}), encoding="utf-8")
    assert read_commit(path, {"HMC_GIT_COMMIT": "  from-env  "}, run=lambda _a: "should-not-run") == "from-env"
    assert read_commit(path, {}, run=lambda _a: "should-not-run") == "abc1234"
    assert read_commit(tmp_path / "missing.json", {}, run=lambda _a: "deadbeef\n") == "deadbeef"
    assert read_commit(tmp_path / "missing.json", {}, run=lambda _a: "not a sha") == "unknown"


def test_write_trial_is_one_json_file_per_trial(tmp_path):
    record = build_trial(
        trial_id="20261119T143052-9-1",
        ict="2026-11-19T14:30:52+07:00",
        commit="5e74c38",
        camera_hash="abcdef1234567890",
        requested={"x": 0.36, "y": 0.0, "yaw": None},
        measured={"centre": [0.43, 0.01], "method": "foot", "spread": 0.05},
        phases={"look": 1.2, "grasp": 8.4, "lift": 6.1, "raise": 12.0},
        confidences={"pose": 0.66},
        success=False,
        failure="FAILED: bag fallen",
        frames=["20261119T143052-9-1_look.jpg"],
    )
    path = write_trial(record, tmp_path)
    assert os.path.basename(path) == "20261119T143052-9-1.json"
    loaded = json.loads(open(path, encoding="utf-8").read())
    assert loaded["trial_id"] == record["trial_id"]
    assert loaded["ict"].endswith("+07:00")
    assert loaded["commit"] == "5e74c38"
    assert loaded["camera_params_hash"] == "abcdef1234567890"
    assert loaded["requested"]["x"] == 0.36
    assert loaded["measured"]["centre"] == [0.43, 0.01]
    assert loaded["phases_s"]["grasp"] == 8.4
    assert loaded["confidences"]["pose"] == 0.66
    assert loaded["success"] is False
    assert loaded["failure"] == "FAILED: bag fallen"
    assert loaded["frames"] == ["20261119T143052-9-1_look.jpg"]
    other = dict(record, trial_id="20261119T143053-9-2", success=True, failure=None)
    write_trial(other, tmp_path)
    assert sorted(os.listdir(tmp_path)) == [
        "20261119T143052-9-1.json",
        "20261119T143053-9-2.json",
    ]


def test_phase_clock_adds_time_spent_in_a_phase_that_runs_twice():
    ticks = iter([0.0, 0.0, 1.5, 4.0, 4.25])
    clock = PhaseClock(now=lambda: next(ticks))
    clock.phase("look")
    clock.phase("grasp")
    clock.phase("look")
    clock.close()
    assert clock.phases["look"] == pytest.approx(1.75)
    assert clock.phases["grasp"] == pytest.approx(2.5)
    assert "tail" not in clock.phases
