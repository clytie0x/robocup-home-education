"""evaluate prints success rate, median time and collision count."""

from rl_grasp.evaluate import format_report


def test_report_fields():
    text = format_report("scripted", episodes=4, successes=3, times=[1.5, 2.5, 3.5], collisions=1)
    lines = dict(line.split(": ", 1) for line in text.splitlines())
    assert lines["policy"] == "scripted"
    assert lines["episodes"] == "4"
    assert lines["success_rate"] == "0.750000"
    assert lines["median_time_s"] == "2.500"
    assert lines["collision_count"] == "1"
    assert lines["successes"] == "3"


def test_no_success_prints_nan_for_the_median():
    text = format_report("ppo", episodes=2, successes=0, times=[], collisions=2)
    assert "median_time_s: nan" in text
    assert "success_rate: 0.000000" in text
