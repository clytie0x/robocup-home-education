#include <stdint.h>

#include "../control.h"
#include "check.h"

static void body_to_wheels_splits_and_keeps_curvature(void)
{
    wheel_pair w = body_to_wheels(0.10f, 0.0f, 0.31f, 0.15f);
    CHECK_NEAR(w.left, 0.10f, 1e-6);
    CHECK_NEAR(w.right, 0.10f, 1e-6);

    w = body_to_wheels(0.0f, 0.5f, 0.31f, 0.15f); /* left turn on the spot */
    CHECK_NEAR(w.left, -0.0775f, 1e-6);
    CHECK_NEAR(w.right, 0.0775f, 1e-6);

    /* 0.12 m/s and 0.49 rad/s ask the right wheel for 0.196 m/s: both shrink together. */
    w = body_to_wheels(0.12f, 0.49f, 0.31f, 0.15f);
    CHECK_NEAR(w.right, 0.15f, 1e-6);
    float asked = 0.49f / 0.12f; /* curvature, 1/m */
    float got = (w.right - w.left) / 0.31f / ((w.left + w.right) / 2.0f);
    CHECK_NEAR(got, asked, 1e-4);

    w = body_to_wheels(-0.10f, 0.0f, 0.31f, 0.15f); /* B packets drive backward */
    CHECK_NEAR(w.left, -0.10f, 1e-6);
}

static void ramp_limits_acceleration_and_braking(void)
{
    const float dt = 0.005f;
    wheel_pair zero = {0.0f, 0.0f}, cruise = {0.10f, 0.10f};
    wheel_pair r = ramp_wheels(zero, cruise, 0.25f, 0.5f, dt);
    CHECK_NEAR(r.left, 0.00125f, 1e-7); /* 0.25 m/s^2 for 5 ms */

    int ticks = 1;
    while (r.left < 0.10f && ticks < 1000) {
        r = ramp_wheels(r, cruise, 0.25f, 0.5f, dt);
        ticks++;
    }
    CHECK(ticks >= 79 && ticks <= 81); /* 0.10 / 0.25 = 0.4 s = 80 ticks */
    CHECK(r.left <= 0.10f + 1e-6f);

    r = ramp_wheels(cruise, zero, 0.25f, 0.5f, dt);
    CHECK_NEAR(r.left, 0.0975f, 1e-6); /* braking is twice as fast */
    ticks = 1;
    while (r.left > 0.0f && ticks < 1000) {
        r = ramp_wheels(r, zero, 0.25f, 0.5f, dt);
        ticks++;
    }
    CHECK(ticks >= 39 && ticks <= 41);
    CHECK(r.left == 0.0f && r.right == 0.0f); /* lands exactly on zero, so the PI can park */
}

static void ramp_keeps_the_wheel_ratio(void)
{
    wheel_pair zero = {0.0f, 0.0f}, arc = {0.05f, 0.10f};
    wheel_pair r = ramp_wheels(zero, arc, 0.25f, 0.5f, 0.005f);
    CHECK_NEAR(r.right, 0.00125f, 1e-7);
    CHECK_NEAR(r.left / r.right, 0.5f, 1e-5);
}

static void ramp_stops_before_reversing(void)
{
    wheel_pair fwd = {0.01f, 0.01f}, back = {-0.01f, -0.01f};
    wheel_pair r = ramp_wheels(fwd, back, 0.25f, 0.5f, 0.005f);
    CHECK_NEAR(r.left, 0.0075f, 1e-7); /* toward zero at the brake rate */
    for (int i = 0; i < 3; i++) {
        r = ramp_wheels(r, back, 0.25f, 0.5f, 0.005f);
    }
    CHECK(r.left == 0.0f);
    r = ramp_wheels(r, back, 0.25f, 0.5f, 0.005f);
    CHECK_NEAR(r.left, -0.00125f, 1e-7); /* then away from zero at the accel rate */
}

static const pi_gains TEST_GAINS = {500.0f, 300.0f, 1500.0f, 0.02f, 100.0f};

static void pi_parks_at_zero(void)
{
    pi_state s = {0.01f};
    CHECK(wheel_pi(&s, &TEST_GAINS, 0.0f, 0.03f, 0.005f) == 0.0f);
    CHECK(s.integral == 0.0f);
}

static void pi_feed_forward_alone_when_on_target(void)
{
    pi_state s = {0.0f};
    CHECK_NEAR(wheel_pi(&s, &TEST_GAINS, 0.10f, 0.10f, 0.005f), 50.0f, 1e-4);
}

static void pi_does_not_wind_up_at_full_pwm(void)
{
    pi_gains g = TEST_GAINS;
    g.pwm_max = 60.0f;
    pi_state s = {0.0f};
    /* Wheel held still: the PWM sits at its limit for a whole second. */
    for (int i = 0; i < 200; i++) {
        CHECK_NEAR(wheel_pi(&s, &g, 0.10f, 0.0f, 0.005f), 60.0f, 1e-4);
    }
    CHECK(s.integral == 0.0f);
    /* Load gone, wheel at speed: no stored push left to overshoot with. */
    CHECK_NEAR(wheel_pi(&s, &g, 0.10f, 0.10f, 0.005f), 50.0f, 1e-4);

    /* At the limit, an error the other way may still unwind the integral. */
    s.integral = 0.02f;
    wheel_pi(&s, &g, 0.10f, 0.12f, 0.005f);
    CHECK(s.integral < 0.02f);
}

static void pi_integral_is_clamped(void)
{
    pi_state s = {0.0f};
    for (int i = 0; i < 2000; i++) {
        wheel_pi(&s, &TEST_GAINS, 0.05f, 0.04f, 0.005f); /* 25 + 3 + I stays under 100 */
    }
    CHECK_NEAR(s.integral, 0.02f, 1e-6);
}

/* A motor 20 % weaker than the feed-forward assumes: the integral makes up the rest. */
static void pi_removes_steady_error(void)
{
    pi_state s = {0.0f};
    float speed = 0.0f;
    for (int i = 0; i < 600; i++) {
        float pwm = wheel_pi(&s, &TEST_GAINS, 0.10f, speed, 0.005f);
        speed += (pwm / 600.0f - speed) * 0.2f; /* first-order motor, 600 % per m/s */
    }
    CHECK_NEAR(speed, 0.10f, 0.001f);
}

static void speed_from_counts(void)
{
    speed_window w;
    speed_init(&w);
    CHECK(speed_update(&w, 0, 0.001f, 0.005f) == 0.0f); /* one sample: no speed yet */
    CHECK_NEAR(speed_update(&w, 10, 0.001f, 0.005f), 2.0f, 1e-5);
    CHECK_NEAR(speed_update(&w, 20, 0.001f, 0.005f), 2.0f, 1e-5);
    for (int i = 3; i <= 10; i++) {
        speed_update(&w, i * 10, 0.001f, 0.005f);
    }
    /* Window full (4 intervals): one slow tick moves the speed by a quarter. */
    CHECK_NEAR(speed_update(&w, 100 + 2, 0.001f, 0.005f), (30.0f + 2.0f) * 0.001f / 0.02f, 1e-5);

    /* The 32-bit running count wraps: the speed must not jump. */
    speed_init(&w);
    int32_t start = INT32_MAX - 5;
    speed_update(&w, start, 0.001f, 0.005f);
    int32_t wrapped = (int32_t)((uint32_t)start + 10u);
    CHECK_NEAR(speed_update(&w, wrapped, 0.001f, 0.005f), 2.0f, 1e-5);
}

void test_control(void)
{
    body_to_wheels_splits_and_keeps_curvature();
    ramp_limits_acceleration_and_braking();
    ramp_keeps_the_wheel_ratio();
    ramp_stops_before_reversing();
    pi_parks_at_zero();
    pi_feed_forward_alone_when_on_target();
    pi_does_not_wind_up_at_full_pwm();
    pi_integral_is_clamped();
    pi_removes_steady_error();
    speed_from_counts();
}
