"""The measured base between the brain and the wheels.

The simulation's Robot drives at once and has no dead zone. The real base, measured on
2026-11-13 and 2026-11-15 (base/params.json and four online latency estimates), is
slower: a command arrives about 0.40 s later, a turn on the spot below 0.10 rad/s and a
forward speed below 0.01 m/s give no motion, the top speed is 0.12 m/s and it slows at
0.14 m/s^2 (measured 0.135 to 0.157).

Run the scenario with  python -m hmc.run --plant real-base  to drive this plant.
"""

import math
from collections import deque

from hmc.world import Robot, wrap_angle


class Plant:
    """A Robot for the task code, with the measured base's delay and limits in front of it.

    The task still calls drive_toward, turn_toward and brake. Each call is one command
    (v, w) for that cycle. The command waits in a FIFO for `delay` seconds and only then
    reaches the wheels. A shadow Robot picks the command from the plant's own pose, so
    the acceleration and turn limits in world.py still decide what the brain asks for.
    """

    def __init__(self, robot, delay=0.40, turn_deadzone=0.10, drive_deadzone=0.01,
                 max_speed=0.12, decel=0.14):
        self._shadow = Robot(max_speed=min(robot.max_speed, max_speed),
                             max_turn_rate=robot.max_turn_rate)
        self.x, self.y, self.heading = robot.x, robot.y, robot.heading
        self.max_speed = min(robot.max_speed, max_speed)  # the brain must know the real top speed
        self.max_turn_rate = robot.max_turn_rate
        self.speed = 0.0  # m/s at the wheels now
        self.turn_speed = 0.0  # rad/s at the wheels now
        self.delay = delay
        self.turn_deadzone = turn_deadzone
        self.drive_deadzone = drive_deadzone
        self.decel = decel
        self.time = 0.0  # the plant's clock: one dt per command cycle
        self._sent = deque()  # (time, v, w) commands still on their way to the wheels
        self.requests = []  # (v, w) the brain asked for, before the dead zones and the cap

    @property
    def pos(self):
        return (self.x, self.y)

    def drive_toward(self, target, dt, stop_distance=0.0, stop_in=None):
        shadow = self._shadow_now()
        shadow.drive_toward(target, dt, stop_distance, stop_in)
        self.command(shadow.speed, shadow.turn_speed, dt)

    def turn_toward(self, target, dt):
        shadow = self._shadow_now()
        error = shadow.turn_toward(target, dt)
        self.command(0.0, shadow.turn_speed, dt)
        return error

    def brake(self, dt):
        self.command(0.0, 0.0, dt)

    def command(self, v, w, dt):
        """Send one cycle's command, then move with the one that has waited out the delay."""
        now = self.time
        self.requests.append((v, w))
        self._sent.append((now, v, w))
        self.time += dt
        arrived = None
        while self._sent and self._sent[0][0] <= now - self.delay + 1e-9:
            arrived = self._sent.popleft()
        if arrived is not None:
            self._drive(arrived[1], arrived[2], dt)

    def _shadow_now(self):
        """The command generator, at the plant's actual pose and speed."""
        shadow = self._shadow
        shadow.x, shadow.y, shadow.heading = self.x, self.y, self.heading
        shadow.speed, shadow.turn_speed = self.speed, self.turn_speed
        return shadow

    def _drive(self, v, w, dt):
        v = max(-self.max_speed, min(self.max_speed, v))
        if abs(v) < self.drive_deadzone:
            v = 0.0
        if abs(w) < self.turn_deadzone:
            w = 0.0
        self.speed = self._slower(self.speed, v, dt)
        self.turn_speed = w
        if abs(w) < 1e-12:
            self.x += self.speed * dt * math.cos(self.heading)
            self.y += self.speed * dt * math.sin(self.heading)
        else:
            end = self.heading + w * dt
            r = self.speed / w
            self.x += r * (math.sin(end) - math.sin(self.heading))
            self.y -= r * (math.cos(end) - math.cos(self.heading))
        self.heading = wrap_angle(self.heading + w * dt)

    def _slower(self, speed, wanted, dt):
        """Slowing is limited by the measured decel; speeding up is the command itself."""
        if abs(wanted) >= abs(speed):
            return wanted
        left = abs(speed) - min(self.decel * dt, abs(speed))
        if wanted != 0.0 and (wanted > 0.0) != (speed > 0.0):
            return math.copysign(left, speed) if left > 0.0 else 0.0
        return math.copysign(max(left, abs(wanted)), speed)
