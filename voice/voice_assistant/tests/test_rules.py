"""RuleParser (the rules brain, no LLM) on the sentences the Help Me Carry task uses.

rules.py is pure (no rclpy), so this runs on a PC without ROS: the node file rule_node.py holds only the
ROS part. Expected actions are the words base/jetson/hmc_flow.py acts on (INTENT_KINDS there: follow_me,
stop, go_back, resume, carry_bag = its "done").
"""
import pytest

from voice_assistant.rules import RuleParser

ALLOWED = ['none', 'move_forward', 'move_backward', 'turn', 'stop', 'go_to', 'follow_me', 'look_at',
           'drop_bag', 'carry_bag', 'beatbox', 'go_back', 'resume']


def parser(task_words=True):
    p = RuleParser('I can carry your bag and stop.', ALLOWED, 'Test Robot')
    p.task_words = task_words
    return p


@pytest.mark.parametrize("sentence, action", [
    ("follow me", "follow_me"),
    ("this is the car", "stop"),                  # arrived: hmc_flow waits there and asks for the bag
    ("that's the car", "stop"),
    ("we have arrived at the car", "stop"),
    ("we are here", "stop"),
    ("stop", "stop"),
    ("wait for me", "stop"),                      # hmc_flow's FOLLOWING answer to "wait"
    ("take the bag back", "go_back"),             # hmc_flow classifies this as its "back" word
    ("go back to the start", "go_back"),
    ("done", "carry_bag"),                        # at_car: the flow's next step is the bag
    ("ready", "carry_bag"),
    ("I don't know, stop", "stop"),               # a negation in another clause must not eat the stop
    ("nice weather today", "none"),
])
def test_task_sentence(sentence, action):
    assert parser().parse(sentence)[0] == action


def test_negation_still_cancels_a_command():
    assert parser().parse("don't stop")[0] == "none"
    assert parser().parse("don't drop the bag")[0] == "none"


def test_dont_move_freezes_the_robot():
    # the robot stopping where it is, so "don't move" is a stop, as it was before the task words
    assert parser().parse("don't move")[0] == "stop"


def test_task_words_off_keeps_the_generic_launch_the_same():
    # voice_assistant.launch.py (no task_words) still drives the old flow_node: "take the bag back" there
    # means pick it up, and "done" is not a command
    p = parser(task_words=False)
    assert p.parse("take the bag back")[0] == "carry_bag"
    assert p.parse("done")[0] == "none"
    assert p.parse("go back to the start")[0] == "move_backward"
    assert p.parse("this is the car")[0] == "none"


def test_task_words_do_not_break_the_generic_commands():
    p = parser()
    assert p.parse("carry the bag")[0] == "carry_bag"
    assert p.parse("drop the bag")[0] == "drop_bag"
    assert p.parse("put the bag down at the car")[0] == "drop_bag"   # not "at the car" -> stop
    assert p.parse("follow me please")[0] == "follow_me"
    assert p.parse("go to the kitchen")[0] == "go_to"
    action, params, _ = p.parse("go backward one meter")
    assert (action, params["distance"]) == ("move_backward", 1.0)
    action, params, _ = p.parse("turn left 90 degrees")
    assert (action, params["angle"]) == ("turn", 90.0)


def test_fuzzy_match_still_fixes_a_misheard_follow_me():
    # whisper writes "Hollow me"; nothing matched above, so the sound-alike fallback takes it
    assert parser().parse("Hollow me")[0] == "follow_me"
