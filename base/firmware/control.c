#include "control.h"

#include <math.h>

static float clamp(float x, float low, float high)
{
    return x < low ? low : (x > high ? high : x);
}

wheel_pair body_to_wheels(float v, float w, float track, float max_wheel)
{
    wheel_pair out = {v - w * track / 2.0f, v + w * track / 2.0f};
    float biggest = fmaxf(fabsf(out.left), fabsf(out.right));
    if (biggest > max_wheel) {
        float k = max_wheel / biggest;
        out.left *= k;
        out.right *= k;
    }
    return out;
}

/* The speed this wheel heads for during this tick. A wheel that must change direction
 * first stops at zero, so the turn-around uses the brake rate down and the accel rate up. */
static float step_target(float current, float target)
{
    return current * target < 0.0f ? 0.0f : target;
}

/* The largest change allowed this tick: brake rate when slowing, accel rate otherwise. */
static float allowed_change(float current, float target, float accel, float brake, float dt)
{
    return (fabsf(target) < fabsf(current) ? brake : accel) * dt;
}

/* Fraction (0 to 1] of the wanted change this wheel may make in one tick. */
static float step_fraction(float change, float allowed)
{
    return fabsf(change) > allowed ? allowed / fabsf(change) : 1.0f;
}

wheel_pair ramp_wheels(wheel_pair current, wheel_pair target, float accel, float brake, float dt)
{
    float tl = step_target(current.left, target.left);
    float tr = step_target(current.right, target.right);
    float dl = tl - current.left;
    float dr = tr - current.right;
    /* Both wheels take the same fraction of their change, so during the ramp the base
     * moves along the line from the old wheel speeds to the new ones and its curvature
     * does not wander. When one wheel speeds up and the other slows, the slower rate
     * limits both. */
    float k = fminf(step_fraction(dl, allowed_change(current.left, tl, accel, brake, dt)),
                    step_fraction(dr, allowed_change(current.right, tr, accel, brake, dt)));
    wheel_pair out = {current.left + dl * k, current.right + dr * k};
    return out;
}

float wheel_pi(pi_state *s, const pi_gains *g, float target, float measured, float dt)
{
    if (target == 0.0f) {
        /* Ramp finished at zero: no PWM, and forget the integral, so a parked base never
         * creeps on a leftover correction. */
        s->integral = 0.0f;
        return 0.0f;
    }
    float error = target - measured;
    float before = g->kff * target + g->kp * error + g->ki * s->integral;
    /* Anti-wind-up: while the PWM is already at its limit, an error that pushes further
     * the same way must not grow the integral, or the wheel would overshoot once the
     * load drops. An error that pulls back from the limit may still integrate. */
    int pushing_past_limit = (before >= g->pwm_max && error > 0.0f) ||
                             (before <= -g->pwm_max && error < 0.0f);
    if (!pushing_past_limit) {
        s->integral = clamp(s->integral + error * dt, -g->integral_max, g->integral_max);
    }
    float pwm = g->kff * target + g->kp * error + g->ki * s->integral;
    return clamp(pwm, -g->pwm_max, g->pwm_max);
}

void speed_init(speed_window *w)
{
    w->newest = 0;
    w->filled = 0;
}

float speed_update(speed_window *w, int32_t counts, float meters_per_count, float dt)
{
    const int size = SPEED_WINDOW_TICKS + 1;
    w->newest = (w->newest + 1) % size;
    w->counts[w->newest] = counts;
    if (w->filled < size) {
        w->filled++;
    }
    int intervals = w->filled - 1;
    if (intervals == 0) {
        return 0.0f;
    }
    int oldest = (w->newest - intervals + size) % size;
    /* Unsigned subtraction: right even when the 32-bit running count wraps. */
    int32_t change = (int32_t)((uint32_t)counts - (uint32_t)w->counts[oldest]);
    return (float)change * meters_per_count / ((float)intervals * dt);
}
