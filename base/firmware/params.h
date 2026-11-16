/* Tuning values the Pi can change at run time with K<name>=<value>#, so tuning and
 * calibration never need a reflash. They live in RAM: every start begins from the
 * defaults in config.h, and the Pi sends its values again after a reset.
 *
 * Safety values (command timeout, encoder fault, watchdog) are not here on purpose: they
 * stay compile-time.
 *
 * No hardware calls here, so the laptop tests cover all of it. */
#ifndef PARAMS_H
#define PARAMS_H

#define PARAM_NAME_MAX 16 /* longest name plus its NUL */

typedef struct {
    float kff;              /* PWM % per m/s */
    float kp;               /* PWM % per m/s of speed error */
    float ki;               /* PWM % per m of integrated error */
    float um_per_count[2];  /* micrometres per encoder count, left and right wheel */
    float track;            /* m, wheel to wheel */
    float accel;            /* m/s^2, speeding up */
    float brake;            /* m/s^2, slowing down, also for S# and the timeout */
    float vmax;             /* m/s, top speed of each wheel; M<vvv># can only lower it */
    float brake_at_zero;    /* 1: zero PWM brakes the motor (L298N), 0: it coasts */
} params;

typedef enum { PARAM_OK, PARAM_UNKNOWN, PARAM_OUT_OF_RANGE } param_result;

void params_defaults(params *p);

/* Set one value by name. Out of range or unknown leaves everything unchanged. */
param_result params_set(params *p, const char *name, float value);

/* Read one value by name into *value. */
param_result params_get(const params *p, const char *name, float *value);

#endif
