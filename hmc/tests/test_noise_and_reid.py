import random

from hmc.appearance import LookMemory, give_looks, random_look, similarity
from hmc.bystanders import add_bystanders
from hmc.noise import Noise
from hmc.perception import MotionTracker, see_looks, see_operator, see_people
from hmc.run import run_scenario
from hmc.task import State
from hmc.world import build_world, distance

# Looks (re-ID)


def test_same_look_is_similar_and_different_looks_are_not():
    rng = random.Random(1)
    a, b = random_look(rng), random_look(rng)
    assert similarity(a, a) > 0.999
    assert abs(similarity(a, b)) < 0.65


def test_look_memory_averages_noisy_views():
    rng = random.Random(2)
    look = random_look(rng)
    memory = LookMemory()
    for _ in range(20):
        memory.add(tuple(v + rng.gauss(0.0, 0.1) for v in look))
    assert memory.similarity(look) > 0.95


def test_a_swapped_tracker_id_keeps_the_real_look():
    world = build_world()
    give_looks(world, seed=0)
    world.noise = Noise(miss_rate=0.0, swap_rate=0.0, look_sigma=0.0)
    world.noise.ids = {1: 2, 2: 1, 3: 3}  # the tracker swapped persons 1 and 2
    world.robot.x, world.robot.y = 6.0, 1.0
    looks = see_looks(world)
    person1 = world.people[0]
    assert similarity(looks[2], person1.look) > 0.999  # reported as id 2, looks like 1


# Noise


def test_noise_is_repeatable_with_the_same_seed():
    a = run_scenario(noise="realistic", seed=3)
    b = run_scenario(noise="realistic", seed=3)
    assert a.rows == b.rows


def test_noise_moves_and_misses_detections():
    world = build_world()
    world.noise = Noise(seed=0)
    truth = world.operator.pos
    seen = []
    for step in range(200):
        world.time = step * 0.1
        seen.append(see_operator(world))
    found = [p for p in seen if p is not None]
    assert 0 < len(found) < len(seen)  # some frames missed the operator
    assert any(distance(p, truth) > 0.01 for p in found)  # and the positions are noisy
    assert all(distance(p, truth) < 0.5 for p in found)


def test_tracker_sees_a_noisy_still_person_as_still():
    rng = random.Random(4)
    tracker = MotionTracker(window=2.0)
    for i in range(25):
        tracker.update(i * 0.1, (5.0 + rng.gauss(0, 0.08), 2.0 + rng.gauss(0, 0.08)))
    assert tracker.is_stationary()
    assert not tracker.has_moved()


def test_tracker_sees_a_noisy_walking_person_walk():
    rng = random.Random(5)
    tracker = MotionTracker(window=2.0)
    for i in range(25):
        tracker.update(i * 0.1, (i * 0.08 + rng.gauss(0, 0.08), rng.gauss(0, 0.08)))
    assert not tracker.is_stationary()
    walk = tracker.direction()
    assert walk is not None and walk[0] > 0.9


# Passers-by


def test_passers_by_are_added_and_seen_like_anyone_else():
    world = build_world()
    add_bystanders(world, 5, seed=0)
    assert len(world.bystanders) == 5
    world.robot.x, world.robot.y = 8.0, 3.0
    assert any(pid >= 100 for pid in see_people(world))


def test_passers_by_step_away_from_the_robot():
    world = build_world()
    add_bystanders(world, 1, seed=0)
    walker = world.bystanders[0]
    world.robot.x, world.robot.y = walker.x + 0.8, walker.y
    before = distance(walker.pos, world.robot.pos)
    for _ in range(5):
        walker.step(0.1, world)
    assert distance(walker.pos, world.robot.pos) > before


def test_a_run_with_passers_by_and_noise_still_finishes():
    result = run_scenario(bystanders=10, seed=0, noise="realistic")
    assert result.task.state == State.DONE
    assert result.sheet.counts.get("re_reach_start") == 1


def test_robustness_check_reports_no_problem_for_a_clean_run():
    from hmc.robustness import check

    assert check("default", bystanders=0, seed=0) == []
