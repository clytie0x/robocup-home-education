"""Appearance for re-identification (re-ID): telling people apart by how they look.

A real robot gets an appearance vector (an "embedding") for every person a detector
finds, from a small re-ID network looking at clothes and shape. Two pictures of the same
person give similar vectors; different people give different ones. Tracker ids can get
swapped when people pass each other, but how a person looks stays with that person.

In the simulation every person gets a random unit vector as their look. Perception
reports it with some noise (see noise.py). The robot compares looks with cosine
similarity: 1.0 is identical, around 0 is unrelated.
"""

import math
import random

LOOK_SIZE = 16  # numbers per look; a real re-ID embedding has a few hundred


def random_look(rng: random.Random) -> tuple[float, ...]:
    values = [rng.gauss(0.0, 1.0) for _ in range(LOOK_SIZE)]
    length = math.sqrt(sum(v * v for v in values))
    return tuple(v / length for v in values)


def similarity(a, b) -> float:
    """Cosine similarity of two looks."""
    dot = sum(x * y for x, y in zip(a, b))
    length = math.sqrt(sum(x * x for x in a)) * math.sqrt(sum(y * y for y in b))
    return dot / length if length > 0 else 0.0


class LookMemory:
    """The average look of one person, built from many noisy observations."""

    def __init__(self) -> None:
        self.total: list[float] = []
        self.count = 0

    def add(self, look) -> None:
        if not self.count:
            self.total = [0.0] * len(look)  # sized by the first look: any re-ID size works
        self.total = [t + v for t, v in zip(self.total, look)]
        self.count += 1

    def similarity(self, look) -> float:
        return similarity(self.total, look) if self.count else 0.0


def give_looks(world, seed: int = 0) -> None:
    """Give every person in the world a look. The same seed gives the same looks."""
    rng = random.Random(seed * 7919 + 1)
    for person in [world.operator] + list(world.people) + list(world.bystanders):
        person.look = random_look(rng)
