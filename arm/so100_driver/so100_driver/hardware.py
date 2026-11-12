"""Real servos (STS3215 via feetech.Bus) or a simulated arm with the same interface."""
from . import feetech as ft

IDS = (1, 2, 3, 4, 5, 6)
GRIPPER_ID = 6


class RealArm:
    # LeRobot SO-100 defaults: P=16 (factory 32 shakes), I=0, D=32, fastest internal profile.
    P, I, D = 16, 0, 32
    ACC = 254
    # Factory dead zone is 1 count. An old trial left 8 (0.7 deg) in RAM: the servo then ignores
    # the small 50 Hz steps of a slow move and jumps in 0.7 deg hops -> visible shaking when slow.
    DEAD_ZONE = 1
    # 2026-11-07 A/B trial on the real arm (slow moves, operator watching): dead zone 3 on
    # base/shoulder/elbow looked smoothest; D=64 and P=24/32 were worse. Base P=12 removed the
    # back-and-forth in the ~0.5 deg gear backlash; P=8 was too soft (tripped stall check).
    # Defaults are set in driver_node.

    def __init__(self, port, gripper_torque_limit=500, p=None, d=None, dead_zone=None):
        # per-servo (ID1..6) gains; defaults above, overridable from ROS parameters
        self.p = list(p or [self.P] * 6)
        self.d = list(d or [self.D] * 6)
        self.dead_zone = list(dead_zone or [self.DEAD_ZONE] * 6)
        self.acc = [self.ACC] * 6
        self.bus = ft.Bus(port)
        self.gripper_torque_limit = gripper_torque_limit
        missing = [i for i in IDS if not self.bus.ping(i)]
        if missing:
            raise RuntimeError(f'servos not answering: {missing}. Check 12V power and USB cable.')

    def configure(self):
        """RAM-only settings, re-applied at every start. EEPROM stays untouched (Lock=1)."""
        for i in IDS:
            if self.bus.read1(i, ft.LOCK) != 1:
                self.bus.write1(i, ft.LOCK, 1)
            self.bus.write1(i, ft.I_GAIN, self.I)
            self.bus.write2(i, ft.GOAL_POSITION + 4, 0)          # goal speed 0 = no speed cap
            limit = self.gripper_torque_limit if i == GRIPPER_ID else 1000
            self.bus.write2(i, ft.TORQUE_LIMIT, limit)
        self.apply_gains()

    def apply_gains(self):
        """Write P/D/dead zone to RAM (safe with torque on; Lock=1 so nothing reaches EEPROM)."""
        for k, i in enumerate(IDS):
            for addr, val in ((ft.P_GAIN, self.p[k]), (ft.D_GAIN, self.d[k]), (ft.ACCELERATION, self.acc[k]),
                              (ft.CW_DEAD_ZONE, self.dead_zone[k]), (ft.CCW_DEAD_ZONE, self.dead_zone[k])):
                if not 0 <= val <= 254:
                    raise ValueError(f'ID{i}: gain {val} out of range')
                self.bus.write1(i, addr, val)

    def read_positions(self):
        return self.bus.sync_read_positions(IDS)

    def write_goals(self, goals, speeds=None):
        # Always rewrite the speed register too: a stale slow speed must never linger (0 = unlimited).
        self.bus.sync_write_goals_speed(goals, speeds or {})

    def torque(self):
        return {i: self.bus.read1(i, ft.TORQUE_ENABLE) for i in IDS}

    def set_torque(self, enabled):
        for i in IDS:
            self.bus.write1(i, ft.TORQUE_ENABLE, 1 if enabled else 0)

    def health(self):
        """load %, voltage, temperature, servo error bits (addr 65) and torque state per servo."""
        out = {}
        data = self.bus.sync_read(IDS, ft.PRESENT_LOAD, 6)   # load(2) voltage temp async-flag status
        torque = self.bus.sync_read(IDS, ft.TORQUE_ENABLE, 1)
        for i, d in data.items():
            load = (d[0] | (d[1] << 8))
            load = -(load & 0x3FF) if load & 0x400 else load & 0x3FF
            out[i] = {'load': load / 10.0, 'voltage': d[2] / 10.0, 'temp': d[3], 'status': d[5],
                      'torque': torque[i][0]}
        return out

    def close(self):
        self.bus.close()


class SimArm:
    """Perfect, slightly lagging follower used to test MoveIt without hardware."""

    def __init__(self, start_raw):
        self.pos = {i: float(start_raw[i]) for i in IDS}
        self.goal = dict(self.pos)
        self.enabled = False

    def configure(self):
        pass

    def read_positions(self):
        if self.enabled:
            for i in IDS:
                self.pos[i] += 0.5 * (self.goal[i] - self.pos[i])
        return {i: int(round(p)) for i, p in self.pos.items()}

    def write_goals(self, goals, speeds=None):
        self.goal.update({i: float(g) for i, g in goals.items()})

    def torque(self):
        return {i: int(self.enabled) for i in IDS}

    def set_torque(self, enabled):
        self.enabled = enabled
        if enabled:
            self.goal = dict(self.pos)

    def health(self):
        return {i: {'load': 0.0, 'voltage': 12.0, 'temp': 30, 'status': 0, 'torque': int(self.enabled)}
                for i in IDS}

    def close(self):
        pass
