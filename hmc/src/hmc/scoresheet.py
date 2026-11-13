"""The score sheet of section 5.1 and a referee that fills it from world facts."""

from dataclasses import dataclass

from hmc.world import (
    CAR_RADIUS, PERSON_RADIUS, ROBOT_RADIUS, World, body_clearance, distance,
)  # fmt: skip


def _person_clearance(person):
    return lambda p: distance(p, person.pos) - PERSON_RADIUS

MAIN, BONUS, PENALTY, DEM, SPECIAL = "main", "bonus", "penalty", "deus_ex", "special"

DEM_CAP = -250  # "Penalties can be applied multiple times up to a total of 250 points"
START_RADIUS = 1.0  # metres, robot counts as "at the start" inside this
FACED = 1.5  # metres; an obstacle this close to the robot counts as "faced"
QUEUE_JOIN = 2.0  # metres behind the last person to count as "joined"
QUEUE_STAY = 3.0  # metres; further behind than this and the robot has left the queue
QUEUE_END_NEAR_START = 2.0  # metres; the queue part of the way back ends here


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
    "dem_natural", "dem_unnatural", "dem_contact",
    "not_attending", "alt_start_signal", "outstanding",
}  # fmt: skip

# Which avoid row belongs to which obstacle kind. The crowd is made of people.
AVOID_ROW = {
    "small_object": "avoid_small_object",
    "hard_to_see": "avoid_hard_to_see",
    "barriers": "avoid_barriers",
    "crowd": "avoid_crowd",
}


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
        self.faced: set[str] = set()  # obstacle kinds the robot came close to
        self.touched: set[str] = set()  # obstacle kinds the robot collided with
        self.queue_joined = False
        self.queue_left = False  # fell back, overtook or touched someone
        self._come_backs_seen = 0
        self.pending_come_back = 0  # applied only if the robot scores again afterwards

    def _once(self, key: str, condition: bool) -> None:
        if condition and self.sheet.counts.get(key, 0) == 0:
            self.sheet.award(key)

    def update(self, world: World, perceived_bag: str | None) -> None:
        target = world.operator.target_bag
        bag = world.bags[target]
        robot = world.robot.pos

        picked_before = self.sheet.counts.get("pick_bag", 0) > 0
        scored_before = self._scored()
        # The operator came back because the robot asked (Deus ex Machina).
        self.pending_come_back += world.operator.came_back - self._come_backs_seen
        self._come_backs_seen = world.operator.came_back
        if not picked_before:
            self._watch_obstacles(world)
        self._once("follow_to_car", distance(robot, world.car) <= CAR_RADIUS)
        self._watch_queue(world)
        self._once("perceive_bag", perceived_bag == target)
        self._once("pick_bag", bag.state == "carried")
        if not picked_before and self.sheet.counts.get("pick_bag", 0) > 0:
            # The obstacles are on the way out. The way out ends when the bag is picked
            # (not when the robot is near the car: going around people, it may get near
            # the car before it passes the last obstacle).
            for kind in sorted(self.faced - self.touched):
                if kind in AVOID_ROW:  # walls and tables have no row of their own
                    self.sheet.award(AVOID_ROW[kind])

        # Re-reaching the start only counts after the robot was at the car,
        # and here we also require the target bag to come with it.
        back = distance(robot, world.start) <= START_RADIUS
        bag_home = distance(bag.pos, world.start) <= START_RADIUS
        self._once(
            "re_reach_start",
            self.sheet.counts.get("follow_to_car", 0) > 0 and back and bag_home,
        )

        # The rulebook: penalties for human assistance "are only applied if the robot
        # continues to score after" it. The -250 cap is in Scoresheet.total().
        if self.pending_come_back and self._scored() > scored_before:
            self.sheet.award("dem_come_back", self.pending_come_back)
            self.pending_come_back = 0

        new_drops = world.drop_count - self._drops_seen
        if new_drops > 0:
            self.sheet.award("drop_bag", new_drops)
            self._drops_seen = world.drop_count

    def _scored(self) -> int:
        """How many main and bonus rows have been awarded so far."""
        return sum(
            self.sheet.counts.get(r.key, 0) for r in ROWS if r.group in (MAIN, BONUS)
        )

    def _watch_obstacles(self, world: World) -> None:
        robot = world.robot.pos
        clearances = [(o.kind, o.shape.clearance(robot), o.shape.clearance) for o in world.obstacles]
        clearances += [
            ("crowd", distance(robot, p.pos) - PERSON_RADIUS, _person_clearance(p))
            for p in world.people
        ]
        for kind, clearance, of in clearances:
            if clearance <= FACED:
                self.faced.add(kind)
            # Judge touching with the real outline, only when the circle says it is close.
            if clearance < ROBOT_RADIUS and body_clearance(world.robot, of) < 0:
                self.touched.add(kind)

    def _watch_queue(self, world: World) -> None:
        """Joining and staying in the queue on the way back to the start."""
        if self.queue_left or self.sheet.counts.get("queue", 0) > 0:
            return
        walking = [p for p in world.people if p.walking]
        if not walking or self.sheet.counts.get("follow_to_car", 0) == 0:
            return
        robot = world.robot.pos
        tail = walking[-1]  # people start walking in order, so the last one is the end
        behind = distance(robot, world.start) > distance(tail.pos, world.start)
        gap = distance(robot, tail.pos)
        if not self.queue_joined:
            # Joining counts only on the way back, so with the bag.
            carrying = world.carried_bag() is not None
            self.queue_joined = carrying and behind and gap <= QUEUE_JOIN
            return
        # The queue part ends when the robot or the last person has reached the start
        # area, or when the last person has walked to the end of their route.
        near = min(distance(robot, world.start), distance(tail.pos, world.start))
        if near <= QUEUE_END_NEAR_START or not tail.route:
            self.sheet.award("queue")
            return
        touched = any(
            distance(robot, p.pos) - PERSON_RADIUS < ROBOT_RADIUS
            and body_clearance(world.robot, _person_clearance(p)) < 0
            for p in walking
        )
        if not behind or gap > QUEUE_STAY or touched:
            self.queue_left = True
