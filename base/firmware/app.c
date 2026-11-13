#include "app.h"

#include <math.h>

#include "board.h"
#include "config.h"

static const pi_gains GAINS = {KFF_PCT_PER_MPS, KP_PCT_PER_MPS, KI_PCT_PER_M, INTEGRAL_MAX_M,
                               PWM_MAX_PCT};

/* Running counts with the wiring signs applied, so forward is positive on both wheels. */
static void read_counts(int32_t counts[2])
{
    int32_t left, right;
    board_read_counts(&left, &right);
    counts[WHEEL_LEFT] = LEFT_ENCODER_SIGN * left;
    counts[WHEEL_RIGHT] = RIGHT_ENCODER_SIGN * right;
}

void app_init(app *a)
{
    parser_init(&a->rx);
    safety_init(&a->safety);
    odom_reset(&a->pose);
    a->pose.t_ms = board_millis();
    a->cmd_v = 0.0f;
    a->cmd_w = 0.0f;
    a->max_speed = MAX_WHEEL_SPEED_MPS;
    a->ramped.left = 0.0f;
    a->ramped.right = 0.0f;
    read_counts(a->counts);
    for (int i = 0; i < 2; i++) {
        speed_init(&a->speed[i]);
        speed_update(&a->speed[i], a->counts[i], METERS_PER_COUNT, CONTROL_DT);
        a->pi[i].integral = 0.0f;
        a->measured[i] = 0.0f;
        a->pwm[i] = 0.0f;
    }
}

static void reply(const char *text, int length)
{
    board_uart_write(text, length);
}

static void handle(app *a, const message *m)
{
    char out[PROTOCOL_MAX_REPLY];
    uint32_t now = board_millis();
    switch (m->kind) {
    case MSG_DRIVE:
        if (a->safety.faults) {
            return; /* a fault holds the motors until S#, whatever the Pi asks */
        }
        a->cmd_v = m->v_mps;
        a->cmd_w = m->w_radps;
        safety_drive_received(&a->safety, now);
        return;
    case MSG_STOP:
        a->cmd_v = 0.0f;
        a->cmd_w = 0.0f;
        safety_clear_faults(&a->safety);
        return;
    case MSG_ODOM:
        reply(out, format_odom(out, a->pose.x, a->pose.y, a->pose.th));
        return;
    case MSG_POSE:
        reply(out, format_pose(out, a->pose.x, a->pose.y, a->pose.th, a->pose.t_ms));
        return;
    case MSG_RESET:
        odom_reset(&a->pose);
        return;
    case MSG_MAX_SPEED:
        a->max_speed = fminf(m->max_speed_mps, MAX_WHEEL_SPEED_MPS);
        return;
    case MSG_ENCODERS:
        reply(out, format_encoders(out, a->counts[WHEEL_LEFT], a->counts[WHEEL_RIGHT],
                                   a->pwm[WHEEL_LEFT], a->pwm[WHEEL_RIGHT],
                                   safety_flags(&a->safety, now)));
        return;
    }
}

void app_poll_uart(app *a)
{
    message m;
    int byte;
    while ((byte = board_uart_read()) != BOARD_UART_EMPTY) {
        if (byte == BOARD_UART_ERROR) {
            parser_line_error(&a->rx);
        } else if (parser_feed(&a->rx, (uint8_t)byte, &m)) {
            handle(a, &m);
        }
    }
}

/* Encoder counts to odometry and wheel speeds. */
static void measure(app *a, uint32_t now)
{
    int32_t counts[2];
    read_counts(counts);
    int32_t d_left = (int32_t)((uint32_t)counts[WHEEL_LEFT] - (uint32_t)a->counts[WHEEL_LEFT]);
    int32_t d_right = (int32_t)((uint32_t)counts[WHEEL_RIGHT] - (uint32_t)a->counts[WHEEL_RIGHT]);
    odom_update(&a->pose, d_left, d_right, METERS_PER_COUNT, TRACK_M, now);
    for (int i = 0; i < 2; i++) {
        a->counts[i] = counts[i];
        a->measured[i] = speed_update(&a->speed[i], counts[i], METERS_PER_COUNT, CONTROL_DT);
    }
}

/* Cut both motors at once. With an encoder broken the speed loop would drive blind, so
 * a ramp cannot be trusted here. */
static void hold_motors(app *a)
{
    a->ramped.left = 0.0f;
    a->ramped.right = 0.0f;
    for (int i = 0; i < 2; i++) {
        a->pi[i].integral = 0.0f;
        a->pwm[i] = 0.0f;
    }
}

void app_tick(app *a)
{
    uint32_t now = board_millis();
    measure(a, now);

    /* The PWM sent last tick against the speed it produced. */
    safety_check_wheel(&a->safety, WHEEL_LEFT, a->pwm[WHEEL_LEFT], a->measured[WHEEL_LEFT], now);
    safety_check_wheel(&a->safety, WHEEL_RIGHT, a->pwm[WHEEL_RIGHT], a->measured[WHEEL_RIGHT], now);

    if (a->safety.faults) {
        hold_motors(a);
    } else {
        /* A silent Pi gets the same ramped stop as S#. */
        int expired = safety_command_expired(&a->safety, now);
        wheel_pair target = body_to_wheels(expired ? 0.0f : a->cmd_v, expired ? 0.0f : a->cmd_w,
                                           TRACK_M, a->max_speed);
        a->ramped = ramp_wheels(a->ramped, target, ACCEL_MPS2, BRAKE_MPS2, CONTROL_DT);
        a->pwm[WHEEL_LEFT] = wheel_pi(&a->pi[WHEEL_LEFT], &GAINS, a->ramped.left,
                                      a->measured[WHEEL_LEFT], CONTROL_DT);
        a->pwm[WHEEL_RIGHT] = wheel_pi(&a->pi[WHEEL_RIGHT], &GAINS, a->ramped.right,
                                       a->measured[WHEEL_RIGHT], CONTROL_DT);
    }
    board_set_pwm(LEFT_MOTOR_SIGN * a->pwm[WHEEL_LEFT], RIGHT_MOTOR_SIGN * a->pwm[WHEEL_RIGHT]);
}
