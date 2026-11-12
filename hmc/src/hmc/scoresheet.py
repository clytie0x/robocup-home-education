"""The score sheet of section 5.1 and a referee that fills it from world facts."""

from dataclasses import dataclass

from hmc.world import World, distance

MAIN, BONUS, PENALTY, DEM, SPECIAL = "main", "bonus", "penalty", "deus_ex", "special"

DEM_CAP = -250  # "Penalties can be applied multiple times up to a total of 250 points"
CAR_RADIUS = 3.0  # metres, robot counts as "at the car" inside this
START_RADIUS = 1.0  # metres, robot counts as "at the start" inside this


@dataclass(frozen=True)
class Row:
    key: str
    label: str
    points: int
    group: str


ROWS = [
    Row("follow_to_car", "Following the person to the car", 300, MAIN),
    Row("perceive_bag", "Perceiving the correct bag", 15, MAIN),
    Row("pick_bag", "Picking up the correct bag", 100, MAIN),
    Row("avoid_crowd", "Avoiding the crowd of people", 50, MAIN),
    Row("avoid_small_object", "Avoiding the small object on the ground", 50, MAIN),
    Row("avoid_hard_to_see", "Avoiding the hard-to-see object", 50, MAIN),
    Row("avoid_barriers", "Avoiding the retractable barriers", 50, MAIN),
    Row("re_reach_start", "Re-reaching the starting location", 200, BONUS),
    Row("queue", "Joining and staying in the queue", 300, BONUS),
    Row("drop_bag", "Dropping the bag", -50, PENALTY),
    Row("dem_natural", "Rediscovering operator, natural interaction", -50, DEM),
    Row("dem_unnatural", "Rediscovering operator, unnatural interaction", -100, DEM),
    Row("dem_come_back", "Rediscovering operator, asking them to come back", -150, DEM),
    Row("dem_contact", "Rediscovering operator, direct contact", -200, DEM),
    Row("not_attending", "Not attending", -500, SPECIAL),
    Row("alt_start_signal", "Using alternative start signal", -100, SPECIAL),
    Row("outstanding", "Outstanding performance", 111, SPECIAL),
]
ROW_BY_KEY = {row.key: row for row in ROWS}

# The best possible score without special penalties and bonuses.
MAX_TOTAL = sum(r.points for r in ROWS if r.group in (MAIN, BONUS))

# Rows this simulation never runs (slice 1 scope).
NOT_RUN = {
    "avoid_crowd", "avoid_small_object", "avoid_hard_to_see", "avoid_barriers",
    "queue", "dem_natural", "dem_unnatural", "dem_come_back", "dem_contact",
    "not_attending", "alt_start_signal", "outstanding",
}  # fmt: skip


class Scoresheet:
    def __init__(self) -> None:
        self.counts: dict[str, int] = {}  # row key -> how many times awarded

    def award(self, key: str, times: int = 1) -> None:
        if key not in ROW_BY_KEY:
            raise KeyError(key)
        self.counts[key] = self.counts.get(key, 0) + times

    def points(self, key: str) -> int:
        return ROW_BY_KEY[key].points * self.counts.get(key, 0)

    def total(self) -> int:
        dem = sum(self.points(r.key) for r in ROWS if r.group == DEM)
        others = sum(self.points(r.key) for r in ROWS if r.group != DEM)
        return others + max(dem, DEM_CAP)

    def status(self, key: str) -> str:
        if self.counts.get(key, 0) > 0:
            return "awarded"
        return "not-run" if key in NOT_RUN else "not-earned"

    def table(self) -> str:
        lines = [f"{'Row':<52}{'Max':>6}{'Awarded':>9}  Status"]
        for row in ROWS:
            lines.append(
                f"{row.label:<52}{row.points:>6}{self.points(row.key):>9}  {self.status(row.key)}"
            )
        lines.append(f"TOTAL (max {MAX_TOTAL} without special rows): {self.total()}")
        return "\n".join(lines)


class Referee:
    """Looks at the true world state and awards rows once the facts are true."""

    def __init__(self, sheet: Scoresheet) -> None:
        self.sheet = sheet
        self._drops_seen = 0

    def _once(self, key: str, condition: bool) -> None:
        if condition and self.sheet.counts.get(key, 0) == 0:
            self.sheet.award(key)

    def update(self, world: World, perceived_bag: str | None) -> None:
        target = world.operator.target_bag
        bag = world.bags[target]
        robot = world.robot.pos

        self._once("follow_to_car", distance(robot, world.car) <= CAR_RADIUS)
        self._once("perceive_bag", perceived_bag == target)
        self._once("pick_bag", bag.state == "carried")

        # Re-reaching the start only counts after the robot was at the car,
        # and here we also require the target bag to come with it.
        back = distance(robot, world.start) <= START_RADIUS
        bag_home = distance(bag.pos, world.start) <= START_RADIUS
        self._once(
            "re_reach_start",
            self.sheet.counts.get("follow_to_car", 0) > 0 and back and bag_home,
        )

        new_drops = world.drop_count - self._drops_seen
        if new_drops > 0:
            self.sheet.award("drop_bag", new_drops)
            self._drops_seen = world.drop_count
