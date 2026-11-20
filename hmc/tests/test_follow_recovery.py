"""Losing the operator, and a look that is only good enough together with the track and the clothes."""
import math

import numpy as np

from hmc import hri
from hmc.follower import ASK_WAIT, LOST_AFTER, OPERATOR_LEARN, OPERATOR_MATCH, Follower, Mode
from hmc.world import DT

from test_follower import LOOK, STRANGER, cycle, learn, make, seen, unseen


def _tilt(look, cosine):
    """A unit vector whose cosine with `look` is `cosine`."""
    base = np.asarray(look, dtype=float)
    ortho = np.zeros_like(base)
    ortho[0], ortho[1] = -base[1], base[0]
    ortho /= np.linalg.norm(ortho)
    return tuple(cosine * base + math.sqrt(1.0 - cosine * cosine) * ortho)


def test_on_loss_the_robot_turns_toward_the_last_heading_and_asks_them_to_wait():
    follower, body, speech = make()
    time = learn(follower, operator=(2.0, 0.0))
    lost = time + LOST_AFTER + 1.0
    cycle(follower, unseen(lost))  # the gap itself starts the search; this step does not drive
    assert follower.mode == Mode.SEARCH
    follower.last_operator_pos = (0.0, 2.0)  # they were last seen off to the left
    before = body.heading
    cycle(follower, unseen(lost + DT))
    assert body.speed == 0.0
    assert body.heading > before + 0.02
    for i in range(1, 12):
        cycle(follower, unseen(lost + DT * (i + 1)))
    assert speech.has_said(hri.WAIT_TEXT)
    assert not speech.has_said(hri.COME_BACK_TEXT)
    assert min(t for t, text in speech.messages if text == hri.WAIT_TEXT) <= follower.search_since + ASK_WAIT + DT


def test_a_weak_look_is_accepted_with_the_clothes_and_a_continuous_track():
    follower, _, _ = make()
    colour = (1.0, 0.0)
    for i in range(OPERATOR_LEARN):
        sample = seen(i * DT)
        sample.operator_colour = colour
        follower.watch(sample)
    assert follower.mode == Mode.FOLLOW
    weak = seen(2.1, (4.1, 0.0), _tilt(LOOK, 0.45))
    weak.operator_colour = colour
    follower.watch(weak)
    assert follower.operator_now == (4.1, 0.0)
    assert follower.fused_score(_tilt(LOOK, 0.45), colour, continuous=True) >= OPERATOR_MATCH


def test_a_stranger_on_the_same_track_is_still_rejected():
    follower, _, _ = make()
    time = learn(follower)
    before = follower.last_operator_pos
    follower.watch(seen(time, (4.1, 0.0), STRANGER))
    assert follower.last_operator_pos == before
    assert follower.fused_score(STRANGER, continuous=True) < OPERATOR_MATCH


def test_a_jump_does_not_rewrite_the_operator():
    follower, _, _ = make()
    time = learn(follower)
    before = follower.last_operator_pos
    follower.watch(seen(time, (10.0, 10.0), LOOK))
    assert follower.last_operator_pos == before


def test_continual_update_keeps_twenty_looks_and_moves_the_vector():
    follower, _, _ = make()
    learn(follower)
    before = list(follower.operator_look.total)
    moved = seen(3.0, (4.1, 0.0), _tilt(LOOK, 0.9))
    follower.watch(moved)
    assert follower.operator_look.count == OPERATOR_LEARN
    assert follower.operator_look.total != before
