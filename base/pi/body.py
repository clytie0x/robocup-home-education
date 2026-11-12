"""The robot's body for the brain: an hmc Robot with this base's limits."""
from hmc import world


class Body:
    def __init__(self, p):
        self.p = p
        # These three are module-wide in hmc.world, like hmc.run.use_base does it.
        world.MAX_ACCEL = p["accel"]
        world.MAX_TURN_ACCEL = p["turn_accel"]
        world.ROBOT_RADIUS = p["robot_radius"]
        self.robot = world.Robot(max_speed=p["max_speed"], max_turn_rate=p["max_turn_rate"])

    def sync(self, pose):
        """Take the measured pose (x, y, heading). Speed stays: the brain ramps from it."""
        self.robot.x, self.robot.y, self.robot.heading = pose

    def command(self):
        """The wheel command (v, w) after the brain's step."""
        return self.robot.speed, self.robot.turn_speed

    def limit(self, max_speed):
        """Cap the speed for this cycle (never above the params cap)."""
        if max_speed is None or max_speed != max_speed or max_speed < 0:
            raise ValueError("cap must be a number >= 0")
        self.robot.max_speed = min(self.p["max_speed"], max_speed)
