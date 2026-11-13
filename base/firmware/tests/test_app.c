/* The whole firmware on the laptop: board_stub.c stands in for the STM32 and a simple
 * motor model turns PWM into encoder counts. */
#include "../app.h"
#include "../board.h"
#include "../config.h"
#include "board_stub.h"
#include "check.h"

/* Model motor: speed follows PWM with a short lag; 100 % gives 0.167 m/s. */
#define PLANT_MPS_PER_PCT (1.0f / 600.0f)
/* The motor is 22 % stronger than the feed-forward thinks. With the placeholder gains the
 * integral makes that up with a time constant of about 0.6 s, so speed checks wait 3 s. */
#define SETTLE_MS 3000

static float wheel_speed[2];
static float wheel_position[2]; /* m, forward positive */
static int left_encoder_dead;

static void plant_reset(void)
{
    for (int i = 0; i < 2; i++) {
        wheel_speed[i] = 0.0f;
        wheel_position[i] = 0.0f;
    }
    left_encoder_dead = 0;
}

static void plant_step(void)
{
    const int motor_sign[2] = {LEFT_MOTOR_SIGN, RIGHT_MOTOR_SIGN};
    const int encoder_sign[2] = {LEFT_ENCODER_SIGN, RIGHT_ENCODER_SIGN};
    for (int i = 0; i < 2; i++) {
        float wanted = motor_sign[i] * stub_pwm[i] * PLANT_MPS_PER_PCT;
        wheel_speed[i] += (wanted - wheel_speed[i]) * 0.2f;
        wheel_position[i] += wheel_speed[i] * CONTROL_DT;
        if (!(i == 0 && left_encoder_dead)) {
            stub_counts[i] = encoder_sign[i] * (int32_t)floorf(wheel_position[i] / METERS_PER_COUNT);
        }
    }
}

/* Run the firmware for ms milliseconds; resend packet every 40 ms if it is not NULL, as
 * wheels.py does. */
static void run(app *a, int ms, const char *packet)
{
    for (int t = 0; t < ms; t += 5) {
        if (packet && stub_ms % 40u == 0u) {
            stub_receive(packet);
        }
        stub_ms += 5u;
        plant_step();
        app_poll_uart(a);
        app_tick(a);
    }
}

/* Mean wheel speeds over the next 0.5 s of driving. One-tick speeds jitter by a count or
 * two of the encoder, so a check on a single tick would test noise. */
static void mean_speeds(app *a, const char *packet, float mean[2])
{
    float before[2] = {wheel_position[0], wheel_position[1]};
    run(a, 500, packet);
    for (int i = 0; i < 2; i++) {
        mean[i] = (wheel_position[i] - before[i]) / 0.5f;
    }
}

static void start(app *a)
{
    stub_reset();
    plant_reset();
    stub_ms = 1000u;
    board_init();
    app_init(a);
}

/* Send a request and return the reply text. */
static const char *ask(app *a, const char *request)
{
    stub_clear_tx();
    stub_receive(request);
    app_poll_uart(a);
    return stub_tx;
}

static void motors_stay_still_after_power_up(void)
{
    app a;
    start(&a);
    run(&a, 1000, NULL);
    CHECK(stub_pwm[0] == 0.0f && stub_pwm[1] == 0.0f);
    CHECK_STR(ask(&a, "E#"), "Eleft:+0000000,right:+0000000,pwm:+000,+000,flags:04#\r\n");
}

static void drives_the_asked_speed(void)
{
    app a;
    start(&a);
    float s[2];
    run(&a, SETTLE_MS, "F010|+000#");
    mean_speeds(&a, "F010|+000#", s);
    CHECK_NEAR(s[0], 0.10f, 0.002f);
    CHECK_NEAR(s[1], 0.10f, 0.002f);
    CHECK(a.pose.x > 0.08f);
    CHECK_NEAR(a.pose.th, 0.0f, 0.01f);
}

static void turns_right_on_a_negative_rate(void)
{
    app a;
    start(&a);
    float s[2];
    run(&a, SETTLE_MS, "F000|-034#"); /* test_wheels.py: 0.40 rad/s right after the 0.862 scale */
    mean_speeds(&a, "F000|-034#", s);
    CHECK_NEAR(s[0], 0.34f * TRACK_M / 2.0f, 0.002f);
    CHECK_NEAR(s[1], -0.34f * TRACK_M / 2.0f, 0.002f);
    CHECK(a.pose.th < -0.3f);
}

static void silence_ramps_to_a_stop(void)
{
    app a;
    start(&a);
    run(&a, SETTLE_MS, "F010|+000#");
    uint32_t last_packet = stub_ms - stub_ms % 40u; /* when run() last queued it */

    /* O# and a malformed drive message during the silence must not keep it driving. */
    run(&a, (int)(last_packet + 200u - stub_ms), NULL);
    ask(&a, "O#");
    stub_receive("F01x|+000#");
    stub_receive("F010|+00");
    stub_receive_error();
    stub_receive("0#");
    run(&a, (int)(last_packet + COMMAND_TIMEOUT_MS - 10u - stub_ms), NULL);
    CHECK(stub_pwm[0] > 30.0f); /* still cruising just before the timeout */

    run(&a, 60, NULL);
    CHECK(stub_pwm[0] > 0.0f);        /* past the timeout, but ramping, not cut */
    CHECK(a.ramped.left < 0.10f);

    run(&a, 250, NULL); /* 0.10 m/s at 0.5 m/s^2 takes 0.2 s */
    CHECK(stub_pwm[0] == 0.0f && stub_pwm[1] == 0.0f);
    CHECK(ask(&a, "E#")[stub_tx_length - 4] == '4'); /* flags:04# and CRLF, timeout bit set */
}

static void stop_message_ramps_down(void)
{
    app a;
    start(&a);
    run(&a, SETTLE_MS, "F010|+000#");
    stub_receive("S#S#S#");
    run(&a, 100, NULL);
    CHECK(a.ramped.left > 0.0f && a.ramped.left < 0.06f); /* 0.10 - 0.5 * 0.1 = 0.05 */
    run(&a, 150, NULL);
    CHECK(stub_pwm[0] == 0.0f && stub_pwm[1] == 0.0f);
}

static void dead_encoder_stops_both_motors_until_s(void)
{
    app a;
    start(&a);
    left_encoder_dead = 1;
    run(&a, 1500, "F010|+000#");
    CHECK(stub_pwm[0] == 0.0f && stub_pwm[1] == 0.0f);
    CHECK(a.safety.faults == FLAG_LEFT_ENCODER_FAULT);
}

static void fault_report_and_clear(void)
{
    app a;
    start(&a);
    left_encoder_dead = 1;
    run(&a, 1500, "F010|+000#");
    const char *e = ask(&a, "E#");
    CHECK(strstr(e, "pwm:+000,+000,flags:05#") != NULL); /* fault, and timeout: F is ignored */

    run(&a, 500, "F010|+000#"); /* drive commands are ignored while the fault holds */
    CHECK(stub_pwm[0] == 0.0f && stub_pwm[1] == 0.0f);

    left_encoder_dead = 0;
    stub_receive("S#");
    run(&a, 100, NULL);
    CHECK(a.safety.faults == 0u);
    float s[2];
    run(&a, SETTLE_MS, "F010|+000#");
    mean_speeds(&a, "F010|+000#", s);
    CHECK_NEAR(s[0], 0.10f, 0.002f);
}

static void replies_and_reset(void)
{
    app a;
    start(&a);
    run(&a, 1500, "F010|+000#");
    const char *o = ask(&a, "O#");
    CHECK(strncmp(o, "Ox:+000.", 8) == 0 && strlen(o) == 34);

    const char *p = ask(&a, "P#");
    char want_t[16];
    snprintf(want_t, sizeof want_t, "t:%010lu#", (unsigned long)stub_ms);
    CHECK(strncmp(p, "Px:+0000", 8) == 0 && strstr(p, want_t) != NULL);

    ask(&a, "R#");
    CHECK_STR(ask(&a, "O#"), "Ox:+000.00,y:+000.00,th:+000.00#\r\n");
}

static void max_speed_message_lowers_the_cap(void)
{
    app a;
    start(&a);
    float s[2];
    stub_receive("M005#");
    run(&a, SETTLE_MS, "F010|+000#");
    mean_speeds(&a, "F010|+000#", s);
    CHECK_NEAR(s[0], 0.05f, 0.002f);

    stub_receive("M900#"); /* never above the compiled cap */
    run(&a, SETTLE_MS, "F999|+000#");
    mean_speeds(&a, "F999|+000#", s);
    CHECK_NEAR(s[0], MAX_WHEEL_SPEED_MPS, 0.002f);
}

void test_app(void)
{
    motors_stay_still_after_power_up();
    drives_the_asked_speed();
    turns_right_on_a_negative_rate();
    silence_ramps_to_a_stop();
    stop_message_ramps_down();
    dead_encoder_stops_both_motors_until_s();
    fault_report_and_clear();
    replies_and_reset();
    max_speed_message_lowers_the_cap();
}
