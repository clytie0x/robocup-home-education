#include "params.h"

#include <stddef.h>
#include <string.h>

#include "config.h"

typedef struct {
    const char *name;
    size_t offset; /* of the float inside params */
    float low, high;
} param_info;

/* The bounds keep every value where the loop still makes sense: a typo cannot make the
 * base run away or divide by zero. Wide enough for any honest calibration. */
static const param_info TABLE[] = {
    {"kff", offsetof(params, kff), 0.0f, 3000.0f},
    {"kp", offsetof(params, kp), 0.0f, 3000.0f},
    {"ki", offsetof(params, ki), 0.0f, 20000.0f},
    {"mpc_left", offsetof(params, um_per_count[0]), 5.0f, 30.0f},
    {"mpc_right", offsetof(params, um_per_count[1]), 5.0f, 30.0f},
    {"track", offsetof(params, track), 0.20f, 0.40f},
    {"accel", offsetof(params, accel), 0.05f, 2.0f},
    {"brake", offsetof(params, brake), 0.10f, 3.0f},
    {"vmax", offsetof(params, vmax), 0.0f, 0.60f},
    {"brake_at_zero", offsetof(params, brake_at_zero), 0.0f, 1.0f},
};

void params_defaults(params *p)
{
    p->kff = KFF_PCT_PER_MPS;
    p->kp = KP_PCT_PER_MPS;
    p->ki = KI_PCT_PER_M;
    p->um_per_count[0] = METERS_PER_COUNT * 1e6f;
    p->um_per_count[1] = METERS_PER_COUNT * 1e6f;
    p->track = TRACK_M;
    p->accel = ACCEL_MPS2;
    p->brake = BRAKE_MPS2;
    p->vmax = MAX_WHEEL_SPEED_MPS;
    p->brake_at_zero = BRAKE_AT_ZERO;
}

static const param_info *find(const char *name)
{
    for (size_t i = 0; i < sizeof TABLE / sizeof TABLE[0]; i++) {
        if (strcmp(TABLE[i].name, name) == 0) {
            return &TABLE[i];
        }
    }
    return NULL;
}

param_result params_set(params *p, const char *name, float value)
{
    const param_info *info = find(name);
    if (!info) {
        return PARAM_UNKNOWN;
    }
    /* brake_at_zero is a switch: 0 or 1, nothing in between. */
    int is_switch = info->offset == offsetof(params, brake_at_zero);
    if (!(value >= info->low && value <= info->high) || (is_switch && value != 0.0f && value != 1.0f)) {
        return PARAM_OUT_OF_RANGE;
    }
    *(float *)((char *)p + info->offset) = value;
    return PARAM_OK;
}

param_result params_get(const params *p, const char *name, float *value)
{
    const param_info *info = find(name);
    if (!info) {
        return PARAM_UNKNOWN;
    }
    *value = *(const float *)((const char *)p + info->offset);
    return PARAM_OK;
}
