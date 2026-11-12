import pytest

from hmc.scoresheet import DEM_CAP, MAX_TOTAL, ROW_BY_KEY, ROWS, Scoresheet

# Every row of the rulebook score sheet and its value.
EXPECTED = {
    "follow_to_car": 300,
    "perceive_bag": 15,
    "pick_bag": 100,
    "avoid_crowd": 50,
    "avoid_small_object": 50,
    "avoid_hard_to_see": 50,
    "avoid_barriers": 50,
    "re_reach_start": 200,
    "queue": 300,
    "drop_bag": -50,
    "dem_natural": -50,
    "dem_unnatural": -100,
    "dem_come_back": -150,
    "dem_contact": -200,
    "not_attending": -500,
    "alt_start_signal": -100,
    "outstanding": 111,
}


def test_rows_match_rulebook():
    assert {row.key: row.points for row in ROWS} == EXPECTED


@pytest.mark.parametrize("key, points", EXPECTED.items())
def test_each_row_alone(key, points):
    sheet = Scoresheet()
    sheet.award(key)
    assert sheet.total() == points


def test_max_total_is_1115():
    assert MAX_TOTAL == 1115


def test_all_main_and_bonus_rows_give_1115():
    sheet = Scoresheet()
    for row in ROWS:
        if row.group in ("main", "bonus"):
            sheet.award(row.key)
    assert sheet.total() == 1115


def test_special_rows_are_outside_the_1115():
    sheet = Scoresheet()
    for row in ROWS:
        if row.group in ("main", "bonus"):
            sheet.award(row.key)
    sheet.award("outstanding")
    assert sheet.total() == 1115 + 111


def test_drop_penalty_can_repeat():
    sheet = Scoresheet()
    sheet.award("drop_bag", 2)
    assert sheet.total() == -100


def test_deus_ex_machina_is_capped_at_250():
    sheet = Scoresheet()
    for key in ("dem_natural", "dem_unnatural", "dem_come_back", "dem_contact"):
        sheet.award(key)  # -500 in total before the cap
    assert sheet.total() == DEM_CAP == -250


def test_unknown_row_is_rejected():
    with pytest.raises(KeyError):
        Scoresheet().award("nonsense")


def test_status_labels():
    sheet = Scoresheet()
    sheet.award("pick_bag")
    assert sheet.status("pick_bag") == "awarded"
    assert sheet.status("follow_to_car") == "not-earned"
    assert sheet.status("dem_natural") == "not-run"
    assert ROW_BY_KEY["queue"].points == 300
