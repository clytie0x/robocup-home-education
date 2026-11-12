"""Sensor noise: the mistakes a real camera detector (YOLO), tracker and lidar make.

Without noise, perception is perfect. With noise, every time step (every frame):
- positions are off by a little (Gaussian),
- people, the operator and bags are sometimes not detected, more often far away,
- the pointing direction is off by a few degrees,
- the tracker sometimes swaps the ids of two people who pass close to each other,
  and keeps the swapped ids (this is what breaks "remember who was in the crowd"),
- for a frame, another person close to the operator is sometimes taken for the operator.

Walls still block the view, and obstacles are seen without noise (the lidar sees static
things well). All random numbers come from one seeded generator, so a run is repeatable.
The levels are our guess for a small YOLO model with a simple tracker, not measurements.
"""

import random
from dataclasses import dataclass, field


@dataclass
class Noise:
    position_sigma: float = 0.08  # metres, people and the operator
    bag_sigma: float = 0.05  # metres
    miss_rate: float = 0.10  # chance to miss a person or a bag in a frame
    far_from: float = 6.0  # metres; beyond this, misses get more likely
    miss_per_metre: float = 0.05  # extra miss chance per metre beyond far_from
    pointing_sigma_deg: float = 5.0
    look_sigma: float = 0.1  # noise on each number of a person's look (re-ID)
    swap_radius: float = 0.8  # metres; two people this close may get their ids swapped
    swap_rate: float = 0.02  # chance per frame for such a pair
    confuse_radius: float = 1.0  # metres; someone this close to the operator...
    confuse_rate: float = (
        0.05  # ...is taken for the operator with this chance per frame
    )
    seed: int = 0
    rng: random.Random = field(init=False, repr=False)
    ids: dict[int, int] = field(init=False, repr=False)  # true id -> reported id
    frame_time: float | None = field(init=False, default=None, repr=False)
    frame: dict = field(init=False, default_factory=dict, repr=False)

    def __post_init__(self) -> None:
        self.rng = random.Random(self.seed)
        self.ids = {}

    def missed(self, range_m: float) -> bool:
        chance = self.miss_rate + self.miss_per_metre * max(
            0.0, range_m - self.far_from
        )
        return self.rng.random() < chance

    def jitter(self, p, sigma: float):
        return (p[0] + self.rng.gauss(0.0, sigma), p[1] + self.rng.gauss(0.0, sigma))


PRESETS = {
    "off": None,
    "realistic": dict(),
    "hard": dict(
        position_sigma=0.15,
        miss_rate=0.25,
        pointing_sigma_deg=10.0,
        swap_rate=0.05,
        look_sigma=0.2,
        confuse_rate=0.15,
    ),  # fmt: skip
}


def make_noise(level: str, seed: int = 0) -> Noise | None:
    settings = PRESETS[level]
    return None if settings is None else Noise(seed=seed, **settings)
