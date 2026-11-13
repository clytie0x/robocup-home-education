#include "../config.h"
#include "../params.h"
#include "check.h"

static void defaults_are_config_h(void)
{
    params p;
    params_defaults(&p);
    float v = 0.0f;
    CHECK(params_get(&p, "kff", &v) == PARAM_OK && v == KFF_PCT_PER_MPS);
    CHECK(params_get(&p, "kp", &v) == PARAM_OK && v == KP_PCT_PER_MPS);
    CHECK(params_get(&p, "ki", &v) == PARAM_OK && v == KI_PCT_PER_M);
    CHECK(params_get(&p, "mpc_left", &v) == PARAM_OK);
    CHECK_NEAR(v, 12.36f, 0.01f);
    CHECK(params_get(&p, "mpc_right", &v) == PARAM_OK);
    CHECK_NEAR(v, 12.36f, 0.01f);
    CHECK(params_get(&p, "track", &v) == PARAM_OK && v == TRACK_M);
    CHECK(params_get(&p, "accel", &v) == PARAM_OK && v == ACCEL_MPS2);
    CHECK(params_get(&p, "brake", &v) == PARAM_OK && v == BRAKE_MPS2);
    CHECK(params_get(&p, "vmax", &v) == PARAM_OK && v == MAX_WHEEL_SPEED_MPS);
    CHECK(params_get(&p, "brake_at_zero", &v) == PARAM_OK && v == 1.0f);
}

static void set_inside_the_bounds(void)
{
    params p;
    params_defaults(&p);
    CHECK(params_set(&p, "mpc_left", 12.5f) == PARAM_OK);
    CHECK(p.um_per_count[0] == 12.5f && p.um_per_count[1] != 12.5f); /* one wheel only */
    CHECK(params_set(&p, "track", 0.20f) == PARAM_OK && p.track == 0.20f); /* bounds included */
    CHECK(params_set(&p, "vmax", 0.0f) == PARAM_OK && p.vmax == 0.0f);
    CHECK(params_set(&p, "brake_at_zero", 0.0f) == PARAM_OK && p.brake_at_zero == 0.0f);
}

static void refuse_and_keep(void)
{
    params p, before;
    params_defaults(&p);
    before = p;
    CHECK(params_set(&p, "track", 0.19f) == PARAM_OUT_OF_RANGE);
    CHECK(params_set(&p, "track", 0.41f) == PARAM_OUT_OF_RANGE);
    CHECK(params_set(&p, "kp", -1.0f) == PARAM_OUT_OF_RANGE);
    CHECK(params_set(&p, "brake", 0.0f) == PARAM_OUT_OF_RANGE); /* never a base that cannot stop */
    CHECK(params_set(&p, "vmax", 0.61f) == PARAM_OUT_OF_RANGE);
    CHECK(params_set(&p, "brake_at_zero", 0.5f) == PARAM_OUT_OF_RANGE);
    CHECK(params_set(&p, "timeout", 1.0f) == PARAM_UNKNOWN); /* safety stays compile-time */
    CHECK(params_set(&p, "", 1.0f) == PARAM_UNKNOWN);
    float v;
    CHECK(params_get(&p, "kd", &v) == PARAM_UNKNOWN);
    CHECK(memcmp(&p, &before, sizeof p) == 0);
}

void test_params(void)
{
    defaults_are_config_h();
    set_inside_the_bounds();
    refuse_and_keep();
}
