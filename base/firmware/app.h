/* The firmware's state and its two jobs: answer messages and run one control tick.
 * Talks to the hardware only through board.h, so the laptop tests run it whole with
 * tests/board_stub.c. */
#ifndef APP_H
#define APP_H

#include <stdint.h>

#include "control.h"
#include "odom.h"
#include "protocol.h"
#include "safety.h"

typedef struct {
    parser rx;
    safety_state safety;
    odom_pose pose;
    speed_window speed[2];
    pi_state pi[2];
    float cmd_v;        /* latest valid drive command, m/s */
    float cmd_w;        /* rad/s, left positive */
    float max_speed;    /* per wheel, m/s: MAX_WHEEL_SPEED_MPS or lower after M<vvv># */
    wheel_pair ramped;  /* wheel speed targets after the ramp, m/s */
    int32_t counts[2];  /* running counts, forward positive */
    float measured[2];  /* wheel speeds, m/s */
    float pwm[2];       /* PWM sent last tick, %, forward positive */
} app;

/* Call after board_init(). Motors stay at zero until the first valid F or B. */
void app_init(app *a);

/* Read every waiting byte and act on complete messages. */
void app_poll_uart(app *a);

/* One control tick: odometry, safety, ramp, speed loop, PWM out. */
void app_tick(app *a);

#endif
