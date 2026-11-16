/* Wheel speed control: body speed to wheel speeds, ramps, and the per-wheel PI loop.
 * No hardware calls; the laptop tests cover all of it. */
#ifndef CONTROL_H
#define CONTROL_H

#include <stdint.h>

#include "config.h"

typedef struct {
    float left;
    float right;
} wheel_pair; /* m/s for each wheel, forward positive */

/* Body speed (m/s, rad/s left positive) to wheel speeds. If one wheel would pass
 * max_wheel, both are scaled down by the same factor, so the path keeps the asked
 * curvature (a pure turn stays a pure turn). */
wheel_pair body_to_wheels(float v, float w, float track, float max_wheel);

/* One tick of the ramp: move current toward target by at most accel * dt when a wheel
 * speeds up and brake * dt when it slows down. */
wheel_pair ramp_wheels(wheel_pair current, wheel_pair target, float accel, float brake, float dt);

typedef struct {
    float kff;          /* PWM % per m/s, feed-forward */
    float kp;           /* PWM % per m/s of error */
    float ki;           /* PWM % per m of integrated error */
    float integral_max; /* m */
    float pwm_max;      /* % */
} pi_gains;

typedef struct {
    float integral; /* m of speed error integrated over time */
} pi_state;

/* One tick of the speed loop for one wheel. Returns PWM in percent. */
float wheel_pi(pi_state *s, const pi_gains *g, float target, float measured, float dt);

/* Wheel speed from encoder counts over the last SPEED_WINDOW_TICKS ticks. */
typedef struct {
    int32_t counts[SPEED_WINDOW_TICKS + 1];
    int newest; /* index of the newest sample */
    int filled; /* samples stored so far, up to SPEED_WINDOW_TICKS + 1 */
} speed_window;

void speed_init(speed_window *w);
/* Store this tick's running count and return the speed in m/s. */
float speed_update(speed_window *w, int32_t counts, float meters_per_count, float dt);

#endif
