#include "../config.h"
#include "../safety.h"
#include "check.h"

static void timeout(void)
{
    safety_state s;
    safety_init(&s);
    CHECK(safety_command_expired(&s, 5000u)); /* nothing since power-up: motors held */

    safety_drive_received(&s, 1000u);
    CHECK(!safety_command_expired(&s, 1000u + COMMAND_TIMEOUT_MS - 1u));
    CHECK(safety_command_expired(&s, 1000u + COMMAND_TIMEOUT_MS));

    /* Across the 49-day wrap of the millisecond clock. */
    safety_drive_received(&s, 0xFFFFFF00u);
    CHECK(!safety_command_expired(&s, 0xFFFFFF00u + 0x100u)); /* 256 ms later, clock at 0 */
    CHECK(safety_command_expired(&s, 0xFFFFFF00u + COMMAND_TIMEOUT_MS));
}

/* Hold one wheel in a state from t = 0 to t = until_ms, checking every 5 ms. */
static void hold(safety_state *s, float pwm, float speed, uint32_t from_ms, uint32_t until_ms)
{
    for (uint32_t t = from_ms; t <= until_ms; t += 5u) {
        safety_check_wheel(s, WHEEL_LEFT, pwm, speed, t);
    }
}

static void encoder_fault(void)
{
    safety_state s;
    safety_init(&s);
    hold(&s, 50.0f, 0.0f, 0u, FAULT_MS - 5u); /* no counts, not long enough yet */
    CHECK(s.faults == 0u);
    hold(&s, 50.0f, 0.0f, FAULT_MS, FAULT_MS);
    CHECK(s.faults == FLAG_LEFT_ENCODER_FAULT);

    safety_clear_faults(&s);
    CHECK(s.faults == 0u);

    hold(&s, -50.0f, -0.0f, 1000u, 1000u + FAULT_MS); /* backward PWM, still wheel */
    CHECK(s.faults == FLAG_LEFT_ENCODER_FAULT);

    safety_init(&s);
    hold(&s, 50.0f, -0.05f, 0u, FAULT_MS); /* wheel turning against the PWM */
    CHECK(s.faults == FLAG_LEFT_ENCODER_FAULT);

    safety_init(&s);
    hold(&s, FAULT_PWM_PCT, 0.0f, 0u, 2000u); /* gentle PWM may not move a heavy base */
    hold(&s, 50.0f, 0.10f, 2005u, 4000u);     /* hard PWM, wheel turning: fine */
    CHECK(s.faults == 0u);

    safety_init(&s); /* a brief stall that recovers is not a fault */
    hold(&s, 50.0f, 0.0f, 0u, 200u);
    hold(&s, 50.0f, 0.05f, 205u, 205u);
    hold(&s, 50.0f, 0.0f, 210u, 410u);
    CHECK(s.faults == 0u);
}

static void flags(void)
{
    safety_state s;
    safety_init(&s);
    CHECK(safety_flags(&s, 0u) == FLAG_COMMAND_TIMEOUT);
    safety_drive_received(&s, 10u);
    for (uint32_t t = 10u; t <= 10u + FAULT_MS; t += 5u) {
        safety_check_wheel(&s, WHEEL_RIGHT, 80.0f, 0.0f, t);
    }
    CHECK(safety_flags(&s, 20u) == FLAG_RIGHT_ENCODER_FAULT);
    CHECK(safety_flags(&s, 10u + COMMAND_TIMEOUT_MS) == (FLAG_RIGHT_ENCODER_FAULT | FLAG_COMMAND_TIMEOUT));
}

void test_safety(void)
{
    timeout();
    encoder_fault();
    flags();
}
