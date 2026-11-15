"""stt_node's pure parts: the one-word filter, the volume gate and the barge-in detector.

They live in speech_rules.py (numpy only) so they can be tested on a PC without rclpy, sounddevice or
webrtcvad. The parameter defaults below are read out of stt_node.py with ast, because that file imports
rclpy and cannot be imported here.
"""
import ast
from pathlib import Path

import numpy as np
import pytest

from voice_assistant.speech_rules import BargeInDetector, LoudnessGate, noise_transcript

STT_NODE = Path(__file__).resolve().parents[1] / 'voice_assistant' / 'stt_node.py'
PARAMS = Path(__file__).resolve().parents[1] / 'config' / 'params.yaml'

# the task flow says these as one word (base/jetson/hmc_flow.py: ready, done/okay, follow, stop/wait,
# here, back, continue/resume)
TASK_WORDS = ["ready", "done", "okay", "ok", "back", "follow", "continue", "resume", "stop", "wait",
              "here", "yes", "no"]


def stt_param_default(name):
    """Default of stt_node's parameter `name`, read from the source (the node imports rclpy)."""
    for node in ast.walk(ast.parse(STT_NODE.read_text(encoding='utf-8'))):
        if (isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
                and node.func.attr == 'declare_parameter' and node.args
                and isinstance(node.args[0], ast.Constant) and node.args[0].value == name):
            return ast.literal_eval(node.args[1])
    raise AssertionError(f'declare_parameter({name!r}) not found in {STT_NODE.name}')


def params_yaml_line(key):
    """The text of a `key: ...` line in config/params.yaml (there is no yaml module in this venv)."""
    for line in PARAMS.read_text(encoding='utf-8').splitlines():
        stripped = line.strip()
        if stripped.startswith(key + ':'):
            return stripped
    raise AssertionError(f'{key} not found in {PARAMS.name}')


def params_single_word_allow():
    line = params_yaml_line('single_word_allow')
    body = line.split('[', 1)[1].rsplit(']', 1)[0]
    return {w.strip(' "\'') for w in body.split(',') if w.strip(' "\'')}


@pytest.mark.parametrize("word", TASK_WORDS)
def test_task_word_passes_the_one_word_filter(word):
    allow = params_single_word_allow()
    assert not noise_transcript(word, allow)
    assert not noise_transcript(word.capitalize() + '.', allow)     # whisper writes "Ready."


@pytest.mark.parametrize("word", TASK_WORDS)
def test_task_word_is_in_the_stt_node_default(word):
    # the code default must match params.yaml, or a robot without params.yaml drops the word again
    assert word in {w.lower() for w in stt_param_default('single_word_allow')}


@pytest.mark.parametrize("text", ["Go.", "you", "", "[BLANK_AUDIO]", "(", "a"])
def test_noise_transcript_is_dropped(text):
    assert noise_transcript(text, params_single_word_allow())


def test_a_sentence_is_never_dropped_by_the_filter():
    # longer sentences go on to the flow, which decides what they mean
    assert not noise_transcript("nice weather today", params_single_word_allow())


def test_empty_allow_list_keeps_every_transcript():
    assert not noise_transcript("Go.", set())


def test_initial_prompt_uses_the_task_words():
    prompts = [stt_param_default('initial_prompt'), params_yaml_line('initial_prompt')]
    for prompt in prompts:
        low = prompt.lower()
        for phrase in ("follow me", "this is the car", "stop", "wait", "go back", "done", "ready",
                       "bag", "left", "right"):
            assert phrase in low, f'{phrase!r} missing from {prompt!r}'
        assert "beatbox" not in low and "turn left" not in low


# ---------- volume gate ----------

def test_gate_passes_speech_clearly_above_the_room():
    gate = LoudnessGate(min_snr=2.0, min_rms=0.0, floor_min=30.0)
    for _ in range(50):
        gate.update_noise(40.0)
    ok, level, noise, snr = gate.check([100.0] * 10)
    assert ok and noise >= 39.0 and snr > 2.0


def test_gate_drops_a_clip_at_the_room_level():
    gate = LoudnessGate(min_snr=2.0, min_rms=0.0, floor_min=30.0)
    for _ in range(50):
        gate.update_noise(40.0)
    ok, _, _, snr = gate.check([50.0] * 10)
    assert not ok and snr < 2.0


def test_gate_uses_the_floor_in_a_silent_room():
    # nothing heard yet: the floor keeps the gate from dividing by (almost) zero
    gate = LoudnessGate(min_snr=2.0, min_rms=0.0, floor_min=30.0)
    ok, level, noise, _ = gate.check([10.0] * 10)
    assert not ok and noise == 30.0 and level == 10.0


def test_gate_can_be_switched_off_with_min_snr_zero():
    gate = LoudnessGate(min_snr=0.0, min_rms=0.0, floor_min=30.0)
    assert gate.check([5.0] * 10)[0]


def test_gate_absolute_minimum_rms():
    gate = LoudnessGate(min_snr=2.0, min_rms=60.0, floor_min=1.0)
    assert not gate.check([50.0] * 10)[0]
    assert gate.check([70.0] * 10)[0]


# ---------- barge-in detector ----------

def frames(n, value):
    return [np.full(480, value, dtype=np.int16) for _ in range(n)]


def test_barge_threshold_needs_echo_history():
    det = BargeInDetector(ratio=2.0)
    assert det.threshold() is None               # the robot has not talked yet
    for frame in frames(10, 100):
        det.feed(frame, 100.0)
    assert det.threshold() == pytest.approx(200.0)   # p90 x ratio


def test_barge_detector_returns_the_loud_segment():
    det = BargeInDetector(ratio=2.0)
    for frame in frames(12, 100):
        assert det.feed(frame, 100.0) is None
    for frame in frames(6, 300):
        assert det.feed(frame, 300.0) is None    # the segment is still open
    segment = None
    for frame in frames(12, 100):
        out = det.feed(frame, 100.0)
        if out is not None:
            segment = out
    assert segment is not None and len(segment) > 0
    assert int(np.max(np.abs(segment))) == 300


def test_barge_detector_ignores_a_short_bump():
    det = BargeInDetector(ratio=2.0)
    for frame in frames(12, 100):
        det.feed(frame, 100.0)
    for frame in frames(3, 300):                  # enough to start a segment, not to keep it (min_loud 5)
        det.feed(frame, 300.0)
    assert all(det.feed(frame, 100.0) is None for frame in frames(12, 100))


def test_barge_detector_needs_more_margin_while_the_robot_says_stop():
    det = BargeInDetector(ratio=2.0)
    for frame in frames(10, 100):
        det.feed(frame, 100.0)
    plain = det.threshold()
    det.extra = 1.5                              # stt_node raises this when its own line contains "stop"
    assert det.threshold() == pytest.approx(plain * 1.5)
