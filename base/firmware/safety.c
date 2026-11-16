#include "safety.h"

#include <math.h>

#include "config.h"

void safety_init(safety_state *s)
{
    s->driven_since_boot = 0;
    s->last_drive_ms = 0;
    s->faults = 0;
    for (int i = 0; i < 2; i++) {
        s->suspect[i] = 0;
        s->suspect_since_ms[i] = 0;
    }
}

void safety_drive_received(safety_state *s, uint32_t now_ms)
{
    s->driven_since_boot = 1;
    s->last_drive_ms = now_ms;
}

int safety_command_expired(const safety_state *s, uint32_t now_ms)
{
    /* Subtract, never compare times directly: right across the 49-day wrap. */
    return !s->driven_since_boot || (uint32_t)(now_ms - s->last_drive_ms) >= COMMAND_TIMEOUT_MS;
}

/* PWM pushes hard, but the wheel stands still or rolls the other way. */
static int wheel_looks_wrong(float pwm_pct, float speed_mps)
{
    if (fabsf(pwm_pct) <= FAULT_PWM_PCT) {
        return 0;
    }
    return pwm_pct > 0.0f ? speed_mps < FAULT_MIN_SPEED_MPS : speed_mps > -FAULT_MIN_SPEED_MPS;
}

void safety_check_wheel(safety_state *s, int wheel, float pwm_pct, float speed_mps, uint32_t now_ms)
{
    if (!wheel_looks_wrong(pwm_pct, speed_mps)) {
        s->suspect[wheel] = 0;
        return;
    }
    if (!s->suspect[wheel]) {
        s->suspect[wheel] = 1;
        s->suspect_since_ms[wheel] = now_ms;
    }
    if ((uint32_t)(now_ms - s->suspect_since_ms[wheel]) >= FAULT_MS) {
        s->faults |= wheel == WHEEL_LEFT ? FLAG_LEFT_ENCODER_FAULT : FLAG_RIGHT_ENCODER_FAULT;
    }
}

void safety_clear_faults(safety_state *s)
{
    s->faults = 0;
    s->suspect[WHEEL_LEFT] = 0;
    s->suspect[WHEEL_RIGHT] = 0;
}

unsigned safety_flags(const safety_state *s, uint32_t now_ms)
{
    return s->faults | (safety_command_expired(s, now_ms) ? FLAG_COMMAND_TIMEOUT : 0u);
}
