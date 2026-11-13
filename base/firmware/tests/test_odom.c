#include "../odom.h"
#include "check.h"

#define MPC 0.0001f   /* m per count, test value */
#define TRACK 0.31f

static void straight_line(void)
{
    odom_pose p;
    odom_reset(&p);
    for (int i = 0; i < 1000; i++) {
        odom_update(&p, 10, 10, MPC, MPC, TRACK, (uint32_t)(i * 5));
    }
    CHECK_NEAR(p.x, 1.0f, 1e-4);
    CHECK_NEAR(p.y, 0.0f, 1e-6);
    CHECK_NEAR(p.th, 0.0f, 1e-6);
    CHECK(p.t_ms == 4995u);
}

static void turn_on_the_spot_is_not_wrapped(void)
{
    odom_pose p;
    odom_reset(&p);
    /* Two full turns: each wheel rolls 2 * 2 * pi * 0.155 = 1.948 m, at 5 counts per tick. */
    int ticks = (int)(4.0f * 3.14159265f * TRACK / 2.0f / (5.0f * MPC) + 0.5f);
    for (int i = 0; i < ticks; i++) {
        odom_update(&p, -5, 5, MPC, MPC, TRACK, 0u);
    }
    CHECK_NEAR(p.th, 4.0f * 3.14159265f, 5e-3);
    CHECK_NEAR(p.x, 0.0f, 1e-6);
    CHECK_NEAR(p.y, 0.0f, 1e-6);
}

/* Constant wheel speeds make a circle: after a quarter of it the base sits at (R, R). */
static void quarter_circle(void)
{
    odom_pose p;
    odom_reset(&p);
    const int dl = 6, dr = 10;
    float radius = TRACK / 2.0f * (float)(dr + dl) / (float)(dr - dl); /* 0.62 m */
    float dth = (float)(dr - dl) * MPC / TRACK;
    int ticks = (int)(3.14159265f / 2.0f / dth + 0.5f);
    for (int i = 0; i < ticks; i++) {
        odom_update(&p, dl, dr, MPC, MPC, TRACK, 0u);
    }
    CHECK_NEAR(p.th, 3.14159265f / 2.0f, dth);
    CHECK_NEAR(p.x, radius, 2e-3);
    CHECK_NEAR(p.y, radius, 2e-3);
}

/* One big tick of turning: the step must point along the heading halfway through the
 * turn, not the heading at its start. */
static void step_uses_the_midpoint_heading(void)
{
    odom_pose p;
    odom_reset(&p);
    odom_update(&p, 0, 620, MPC, MPC, TRACK, 0u); /* right wheel 0.062 m: dth = 0.2 rad */
    CHECK_NEAR(p.th, 0.2f, 1e-5);
    CHECK_NEAR(atan2f(p.y, p.x), 0.1f, 1e-5);
}

/* Equal counts on wheels of different size: the bigger right wheel goes further, so the
 * base turns left. */
static void each_wheel_has_its_own_scale(void)
{
    odom_pose p;
    odom_reset(&p);
    odom_update(&p, 100, 100, MPC, 1.1f * MPC, TRACK, 0u);
    CHECK_NEAR(p.th, 0.1f * 100.0f * MPC / TRACK, 1e-6);
    CHECK_NEAR(p.x, 1.05f * 100.0f * MPC, 1e-6);
}

static void reset_returns_to_zero(void)
{
    odom_pose p;
    odom_reset(&p);
    odom_update(&p, 100, 300, MPC, MPC, TRACK, 7u);
    odom_reset(&p);
    CHECK(p.x == 0.0f && p.y == 0.0f && p.th == 0.0f);
}

void test_odom(void)
{
    straight_line();
    turn_on_the_spot_is_not_wrapped();
    quarter_circle();
    step_uses_the_midpoint_heading();
    each_wheel_has_its_own_scale();
    reset_returns_to_zero();
}
