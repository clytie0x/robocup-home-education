"""ROS-free motion core: turns trajectories / direct targets into smooth 50 Hz servo goals.

Every tick the reference is clamped to the calibrated range and a maximum joint speed, then
sent as a look-ahead goal (where the reference will be in 0.1 s) together with a matching
speed, so the servo glides instead of sprinting to each step and waiting (which buzzed).
"""
import math

from .calib import ARM_JOINTS, COUNTS_PER_RAD, IDS, JOINTS


def trapezoid_pos(t, a, b, vmax, acc):
    """Position at time t of a trapezoidal move a->b (rad, rad/s, rad/s^2). Returns (pos, duration)."""
    dist = abs(b - a)
    if dist < 1e-9:
        return b, 0.0
    ta = vmax / acc
    if acc * ta * ta > dist:
        ta = math.sqrt(dist / acc)
        vmax = acc * ta
    tc = (dist - acc * ta * ta) / vmax
    total = 2 * ta + tc
    if t <= 0:
        x = 0.0
    elif t < ta:
        x = 0.5 * acc * t * t
    elif t < ta + tc:
        x = 0.5 * acc * ta * ta + vmax * (t - ta)
    elif t < total:
        r = total - t
        x = dist - 0.5 * acc * r * r
    else:
        x = dist
    return a + math.copysign(x, b - a), total


class Trajectory:
    def __init__(self, names, points, start, min_speed=0.0, retime_speed=0.17, retime_acc=1.5):
        """points: [(t_from_start, [rad per name])], sorted by time.

        min_speed (rad/s): a joint whose AVERAGE speed over the move would be slower than this is
        not made to creep (servos judder when crawling at a fraction of a degree per second, seen
        with RViz 0.1/0.1 scaling: the joints that move only a little shook). Instead it makes its
        whole move with a smooth trapezoid at retime_speed and simply arrives early.
        """
        if not points:
            raise ValueError('empty trajectory')
        self.names, self.points, self.start = list(names), points, start
        self.duration = points[-1][0]
        self.retimed = {}
        if min_speed > 0 and self.duration > 0:
            for k, n in enumerate(self.names):
                a, b = points[0][1][k], points[-1][1][k]
                if 1e-4 < abs(b - a) < min_speed * self.duration and self._one_way(k, a, b):
                    self.retimed[n] = (a, b, max(retime_speed, abs(b - a) / self.duration), retime_acc)

    def _one_way(self, k, a, b, tol=0.01):
        """True if joint k only moves from a toward b. A joint that swings out and back (e.g. to
        go around an obstacle) must keep MoveIt's path, so it is never retimed."""
        lo, hi = min(a, b) - tol, max(a, b) + tol
        xs = [pt[1][k] for pt in self.points]
        if any(not lo <= x <= hi for x in xs):
            return False
        sign = 1.0 if b >= a else -1.0
        return all(sign * (x1 - x0) >= -tol for x0, x1 in zip(xs, xs[1:]))

    def sample(self, now):
        t = now - self.start
        out = self._sample_points(t)
        for n, (a, b, v, acc) in self.retimed.items():
            out[n] = trapezoid_pos(t, a, b, v, acc)[0]
        return out

    def _sample_points(self, t):
        pts = self.points
        if t <= pts[0][0]:
            return dict(zip(self.names, pts[0][1]))
        if t >= self.duration:
            return dict(zip(self.names, pts[-1][1]))
        for (t0, p0), (t1, p1) in zip(pts, pts[1:]):
            if t0 <= t <= t1:
                a = 0.0 if t1 == t0 else (t - t0) / (t1 - t0)
                return {n: x0 + a * (x1 - x0) for n, x0, x1 in zip(self.names, p0, p1)}
        return dict(zip(self.names, pts[-1][1]))


class Motion:
    def __init__(self, calib, urdf_limits, max_speed=1.5, gripper_speed=3.0,
                 follow_error=0.6, follow_error_time=1.0, sag_gain=1.5, sag_max_deg=10.0, lag_time=0.0):
        self.calib, self.urdf_limits = calib, urdf_limits
        self.max_speed, self.gripper_speed = max_speed, gripper_speed
        self.follow_error, self.follow_error_time, self.lag_time = follow_error, follow_error_time, lag_time
        self.goal = {}            # raw goal last sent, per joint name
        self.target = {}          # raw target we are heading to (direct mode / hold)
        self.traj = None
        self.fault = None
        self._lag_since = None
        # Gravity sag compensation: with the soft P=16 gain the loaded shoulder/elbow settle a few
        # degrees low. While the reference is standing still we slowly add the remaining error to
        # the command (integral action), capped, and keep that offset during the next move.
        self.sag_gain, self.sag_max = sag_gain, math.radians(sag_max_deg) * COUNTS_PER_RAD
        self.offset = {n: 0.0 for n in JOINTS}
        self.vel_env = {}
        self.max_lag = 0.0        # worst follow error (rad); the driver resets it every second

    def limits(self, name):
        return self.calib.rad_limits(name, self.urdf_limits.get(name))

    def clamp_rad(self, name, rad):
        lo, hi = self.limits(name)
        return min(max(rad, lo), hi)

    def hold(self, present_raw):
        """Stop everything and hold the measured pose (used at torque-on and after faults)."""
        self.traj = None
        self.goal = {n: float(present_raw[n]) for n in JOINTS}
        self.target = dict(self.goal)
        self.offset = {n: 0.0 for n in JOINTS}
        self._lag_since = None

    def freeze(self):
        """Stop moving: keep the current reference (and sag compensation) where it is."""
        self.traj = None
        self.target = dict(self.goal)

    def set_trajectory(self, traj):
        for n in traj.names:
            if n not in ARM_JOINTS and n != 'gripper':
                raise ValueError(f'unknown joint {n}')
        self.traj = traj

    def set_target(self, name, rad):
        self.target[name] = self.calib.to_raw(name, self.clamp_rad(name, rad))

    def step(self, now, present_raw, dt):
        """Returns {joint name: raw goal} to send this tick."""
        desired = dict(self.target)
        if self.traj is not None:
            for n, rad in self.traj.sample(now).items():
                desired[n] = self.calib.to_raw(n, self.clamp_rad(n, rad))
                self.target[n] = desired[n]
        out = {}
        self.speeds = {}
        for n in JOINTS:
            prev = self.goal.get(n, float(present_raw[n]))
            want = desired.get(n, prev)
            lo, hi = self.calib.raw_limits(n)
            want = min(max(want, lo), hi)
            speed = self.gripper_speed if n == 'gripper' else self.max_speed
            step = speed * COUNTS_PER_RAD * dt
            new = prev + max(-step, min(step, want - prev))
            self.goal[n] = new
            # recent reference speed (peak-hold, ~0.3 s decay) for the speed-aware collision check
            v = abs(new - prev) / COUNTS_PER_RAD / dt if dt > 0 else 0.0
            self.vel_env[n] = max(v, self.vel_env.get(n, 0.0) * math.exp(-dt / 0.3))
            if n in ARM_JOINTS:
                moving = abs(new - prev) > 0.05 or (self.traj is not None and now < self.traj.start + self.traj.duration)
                if not moving:
                    off = self.offset[n] + self.sag_gain * (new - present_raw[n]) * dt
                    self.offset[n] = max(-self.sag_max, min(self.sag_max, off))
                vel = (new - prev) / dt if dt > 0 else 0.0          # signed reference speed, counts/s
                ref = new + self.offset[n]
                lag = abs(ref - present_raw[n])
                # Look-ahead goal (where the reference will be in LOOKAHEAD s) + matching speed: the
                # servo never arrives before the next update, so it never stops-and-goes.
                goal = ref + vel * self.LOOKAHEAD
                final = self.final_raw(n) + self.offset[n]
                if vel > 0:
                    goal = min(goal, max(final, ref))      # never lead past the end of the move
                elif vel < 0:
                    goal = max(goal, min(final, ref))
                out[n] = min(max(goal, lo), hi)
                self.speeds[n] = self.servo_speed(abs(vel), lag)
            else:
                out[n] = new
                self.speeds[n] = 0
        self._check_following(now, present_raw)
        return out

    LOOKAHEAD = 0.1   # s

    def final_raw(self, n):
        """Where the current move of joint n ends (trajectory end point, or the direct target)."""
        t = self.traj
        if t is not None and n in t.names:
            return self.calib.to_raw(n, self.clamp_rad(n, t.points[-1][1][t.names.index(n)]))
        return self.target.get(n, self.goal.get(n, 0.0))

    @staticmethod
    def servo_speed(ref_counts_per_s, lag_counts):
        """Speed limit sent with each goal (STS units = counts/s, 0 = unlimited).

        Without it the servo sprints to every 20 ms goal step and waits -> 50 Hz micro-steps,
        felt as a fine buzz/tremor at slow speeds. With it (and the look-ahead goal) the servo
        glides at the reference speed, plus a little extra to close any lag in ~0.3 s.
        """
        if ref_counts_per_s < 1.0 and lag_counts < 3:
            return 0                       # holding still: let the servo correct freely
        v = max(1.05 * ref_counts_per_s + lag_counts / 0.3, 20.0)
        return int(min(v, 3000))

    def allowed_lag(self, n):
        """Servos trail a moving reference by roughly lag_time * speed; a collision shows as more."""
        return self.follow_error + self.lag_time * self.vel_env.get(n, 0.0)

    def _check_following(self, now, present_raw):
        lag = {n: abs(present_raw[n] - self.goal[n]) / COUNTS_PER_RAD for n in ARM_JOINTS}
        self.max_lag = max(self.max_lag, max(lag.values()))
        over = [n for n in ARM_JOINTS if lag[n] > self.allowed_lag(n)]
        if not over:
            self._lag_since = None
            return
        if self._lag_since is None:
            self._lag_since = now
        elif now - self._lag_since > self.follow_error_time:
            bad = max(over, key=lag.get)
            self.fault = (f'{bad} cannot reach its goal (off by '
                          f'{math.degrees(abs(present_raw[bad] - self.goal[bad]) / COUNTS_PER_RAD):.0f} deg): '
                          'blocked/collision, overloaded or power too weak. Holding current pose.')
            self.hold(present_raw)

    def joint_error(self, present_raw, names):
        return {n: abs(present_raw[n] - self.target.get(n, present_raw[n])) / COUNTS_PER_RAD for n in names}


def ids_to_names(raw_by_id):
    return {n: raw_by_id[IDS[n]] for n in JOINTS}


def names_to_ids(raw_by_name):
    return {IDS[n]: v for n, v in raw_by_name.items()}
