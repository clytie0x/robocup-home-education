"""The robot's body for the brain: an hmc Robot with this base's limits."""
import math

from hmc import navigation, world

HEADING_DEADBAND_DEG = 4.0  # standing still, no turn for less: it turned back and forth 12 times in one run


class _Robot(world.Robot):
    """hmc's Robot that keeps the heading error its last turn went for (cable.Cable: does the long way round
    still get there?). Every turn of the brain, on the spot or while driving, goes through _turn."""

    turn_error = None

    def _turn(self, error, dt):
        self.turn_error = error
        return super()._turn(error, dt)


class Body:
    def __init__(self, p):
        self.p = p
        # These three are module-wide in hmc.world, like hmc.run.use_base does it.
        world.MAX_ACCEL = p["accel"]
        world.MAX_TURN_ACCEL = p["turn_accel"]
        world.ROBOT_RADIUS = p["robot_radius"]
        world.MAX_DECEL = p["decel"]  # measured: the brain brakes like the base, not 4 times harder
        world.STOP_DELAY = p["stop_delay"]
        world.HEADING_DEADBAND = math.radians(HEADING_DEADBAND_DEG)
        navigation.CLOSE_PLAN = True  # plan through gaps the 0.40 m body fits, checked with its outline
        # No way to the goal: drive to the reachable spot nearest it and wait, not stand still far away
        # (2026-11-15: 2.5 m from the operator over open floor). plan_server.py sets it as well.
        navigation.TOWARD_UNREACHABLE = True
        navigation.REPLAN_ON_NEW_SHAPES = False  # the Pi cannot plan every cycle (0.12 s a plan)
        self.robot = _Robot(max_speed=p["max_speed"], max_turn_rate=p["max_turn_rate"])

    def sync(self, pose):
        """Take the measured pose (x, y, heading). Speed stays: the brain ramps from it."""
        self.robot.x, self.robot.y, self.robot.heading = pose
        self.robot.turn_error = None  # this cycle's brain step sets it again when it turns

    def command(self):
        """The wheel command (v, w) after the brain's step."""
        return self.robot.speed, self.robot.turn_speed

    def limit(self, max_speed):
        """Cap the speed for this cycle (never above the params cap)."""
        if max_speed is None or max_speed != max_speed or max_speed < 0:
            raise ValueError("cap must be a number >= 0")
        self.robot.max_speed = min(self.p["max_speed"], max_speed)
