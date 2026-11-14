"""Abstract manipulation. There is no real grasp, a bag is just a state flag."""

from hmc.world import CARRIED, DROPPED, PLACED, World, distance

PICK_RANGE = 0.8  # metres between robot and bag


def pick(world: World, bag_name: str) -> bool:
    """Pick up a bag if we are close and hands are free."""
    bag = world.bags[bag_name]
    if world.carried_bag() is not None:
        return False
    if distance(world.robot.pos, bag.pos) > PICK_RANGE:
        return False
    bag.state = CARRIED
    return True


def place(world: World) -> bool:
    """Put the carried bag on the floor, on purpose. No penalty."""
    bag = world.carried_bag()
    if bag is None:
        return False
    bag.state = PLACED
    return True


def slip(world: World) -> bool:
    """Simulate an accidental drop. Used by tests to trigger the penalty."""
    bag = world.carried_bag()
    if bag is None:
        return False
    bag.state = DROPPED
    world.drop_count += 1
    return True
