#include "app.h"

#include <math.h>
#include <string.h>

#include "board.h"
#include "config.h"

#ifndef BUILD_ID
#define BUILD_ID "unknown" /* the Makefile passes the git hash of the sources */
#endif

/* One string in flash, so tools/update.py can find the same text in the image it writes
 * and compare it with the V# reply after the update. */
static const char VERSION_REPLY[] = "Vversion:" FIRMWARE_VERSION ",build:" BUILD_ID "#";

/* M<vvv># not received yet: no limit beyond p.vmax. */
#define NO_SPEED_LIMIT 9.99f

/* Running counts with the wiring signs applied, so forward is positive on both wheels. */
static void read_counts(int32_t counts[2])
{
    int32_t left, right;
    board_read_counts(&left, &right);
    counts[WHEEL_LEFT] = LEFT_ENCODER_SIGN * left;
    counts[WHEEL_RIGHT] = RIGHT_ENCODER_SIGN * right;
}

static float meters_per_count(const app *a, int wheel)
{
    return a->p.um_per_count[wheel] * 1e-6f;
}

void app_init(app *a)
{
    parser_init(&a->rx);
    safety_init(&a->safety);
    params_defaults(&a->p);
    odom_reset(&a->pose);
    a->pose.t_ms = board_millis();
    a->cmd_v = 0.0f;
    a->cmd_w = 0.0f;
    a->max_speed = NO_SPEED_LIMIT;
    a->ramped.left = 0.0f;
    a->ramped.right = 0.0f;
    read_counts(a->counts);
    for (int i = 0; i < 2; i++) {
        speed_init(&a->speed[i]);
        speed_update(&a->speed[i], a->counts[i], meters_per_count(a, i), CONTROL_DT);
        a->pi[i].integral = 0.0f;
        a->measured[i] = 0.0f;
        a->pwm[i] = 0.0f;
    }
}

static void reply(const char *text, int length)
{
    board_uart_write(text, length);
    board_uart_write("\r\n", 2); /* the old firmware ended every reply with println */
}

static void reply_text(const char *text)
{
    reply(text, (int)strlen(text));
}

/* U# restarts the chip, so only when nothing asks the motors to move: no drive command
 * in force (stopped with S#, or silent for the timeout), the ramp at zero, no PWM... */
static int motors_idle(const app *a, uint32_t now)
{
    int no_command = safety_command_expired(&a->safety, now) || (a->cmd_v == 0.0f && a->cmd_w == 0.0f);
    return no_command && a->ramped.left == 0.0f && a->ramped.right == 0.0f && a->pwm[0] == 0.0f &&
           a->pwm[1] == 0.0f;
}

/* ...and both wheels still. Uforce# skips this part only. */
static int wheels_still(const app *a)
{
    return fabsf(a->measured[0]) < FAULT_MIN_SPEED_MPS && fabsf(a->measured[1]) < FAULT_MIN_SPEED_MPS;
}

static void reply_param(const app *a, const char *name, param_result result)
{
    char out[PROTOCOL_MAX_REPLY];
    float value;
    if (result == PARAM_OK && params_get(&a->p, name, &value) == PARAM_OK) {
        reply(out, format_param(out, name, value)); /* always the value now in force */
    } else {
        reply_text(result == PARAM_OUT_OF_RANGE ? "K!range#" : "K!unknown#");
    }
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
        a->max_speed = m->max_speed_mps; /* never above p.vmax: app_tick takes the smaller */
        return;
    case MSG_ENCODERS:
        reply(out, format_encoders(out, a->counts[WHEEL_LEFT], a->counts[WHEEL_RIGHT],
                                   a->pwm[WHEEL_LEFT], a->pwm[WHEEL_RIGHT],
                                   safety_flags(&a->safety, now)));
        return;
    case MSG_DIAG: {
        const float target[2] = {a->ramped.left, a->ramped.right};
        reply(out, format_diag(out, target, a->measured, a->pwm));
        return;
    }
    case MSG_VERSION:
        reply_text(VERSION_REPLY);
        return;
    case MSG_PARAM_SET:
        reply_param(a, m->name, params_set(&a->p, m->name, m->value));
        return;
    case MSG_PARAM_GET:
        reply_param(a, m->name, PARAM_OK);
        return;
    case MSG_PARAM_BAD:
        reply_text("K!syntax#");
        return;
    case MSG_BOOTLOADER:
    case MSG_BOOTLOADER_FORCE:
        if (!motors_idle(a, now) || (m->kind == MSG_BOOTLOADER && !wheels_still(a))) {
            reply_text("U!busy#");
            return;
        }
        reply_text("Uok#");
        board_enter_bootloader(); /* on the board this does not return */
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
    odom_update(&a->pose, d_left, d_right, meters_per_count(a, WHEEL_LEFT),
                meters_per_count(a, WHEEL_RIGHT), a->p.track, now);
    for (int i = 0; i < 2; i++) {
        a->counts[i] = counts[i];
        a->measured[i] = speed_update(&a->speed[i], counts[i], meters_per_count(a, i), CONTROL_DT);
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
                                           a->p.track, fminf(a->max_speed, a->p.vmax));
        a->ramped = ramp_wheels(a->ramped, target, a->p.accel, a->p.brake, CONTROL_DT);
        const pi_gains gains = {a->p.kff, a->p.kp, a->p.ki, INTEGRAL_MAX_M, PWM_MAX_PCT};
        a->pwm[WHEEL_LEFT] = wheel_pi(&a->pi[WHEEL_LEFT], &gains, a->ramped.left,
                                      a->measured[WHEEL_LEFT], CONTROL_DT);
        a->pwm[WHEEL_RIGHT] = wheel_pi(&a->pi[WHEEL_RIGHT], &gains, a->ramped.right,
                                       a->measured[WHEEL_RIGHT], CONTROL_DT);
    }
    board_set_pwm(LEFT_MOTOR_SIGN * a->pwm[WHEEL_LEFT], RIGHT_MOTOR_SIGN * a->pwm[WHEEL_RIGHT],
                  a->p.brake_at_zero != 0.0f);
}
