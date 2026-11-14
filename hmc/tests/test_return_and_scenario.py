import csv

from hmc.run import COLUMNS, main, run_scenario
from hmc.scoresheet import START_RADIUS
from hmc.task import State
from hmc.world import PLACED, TIME_LIMIT, distance


def test_robot_returns_to_start_with_the_bag():
    result = run_scenario()
    world = result.world
    assert distance(world.robot.pos, world.start) <= START_RADIUS
    bag = world.bags["blue"]
    assert bag.state == PLACED
    assert distance(bag.pos, world.start) <= START_RADIUS
    assert result.sheet.counts["re_reach_start"] == 1


def test_end_to_end_score_and_time():
    result = run_scenario()
    assert result.task.state == State.DONE
    assert result.sheet.total() == 1115  # every main and bonus row
    assert result.world.time < TIME_LIMIT
    assert result.sheet.counts.get("drop_bag", 0) == 0


def test_out_of_scope_rows_stay_unawarded():
    result = run_scenario()
    for key in ("dem_natural", "dem_unnatural", "dem_contact",
                "alt_start_signal", "not_attending", "outstanding"):  # fmt: skip
        assert result.sheet.status(key) == "not-run"
    # The robot never loses the operator in the script, so it never asks them back.
    assert result.sheet.status("dem_come_back") == "not-earned"


def test_scenario_is_deterministic():
    assert run_scenario().rows == run_scenario().rows


def test_cli_writes_outputs(tmp_path, capsys):
    code = main(["--out", str(tmp_path)])
    assert code == 0
    assert "Awarded 1115 of 1115" in capsys.readouterr().out
    assert (tmp_path / "score.txt").exists()
    with open(tmp_path / "timeline.csv", newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        rows = list(reader)
    assert reader.fieldnames == COLUMNS
    assert rows[-1]["state"] == "done"
