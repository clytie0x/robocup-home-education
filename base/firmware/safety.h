/* Safety rules: command timeout and encoder faults.
 * No hardware calls; the laptop tests cover all of it. */
#ifndef SAFETY_H
#define SAFETY_H

#include <stdint.h>

/* Bits of the E# flags field. */
#define FLAG_LEFT_ENCODER_FAULT 0x01u
#define FLAG_RIGHT_ENCODER_FAULT 0x02u
#define FLAG_COMMAND_TIMEOUT 0x04u /* no valid F or B lately (also true after power-up) */
#define FAULT_FLAGS (FLAG_LEFT_ENCODER_FAULT | FLAG_RIGHT_ENCODER_FAULT)

enum { WHEEL_LEFT = 0, WHEEL_RIGHT = 1 };

typedef struct {
    int driven_since_boot;   /* a valid F or B has arrived since power-up */
    uint32_t last_drive_ms;  /* when the latest valid F or B arrived */
    int suspect[2];          /* this wheel looked wrong on the last check */
    uint32_t suspect_since_ms[2];
    unsigned faults;         /* FAULT_FLAGS bits, held until S# */
} safety_state;

void safety_init(safety_state *s);

/* A valid F or B arrived. Nothing else (O#, E#, malformed bytes) keeps the base driving. */
void safety_drive_received(safety_state *s, uint32_t now_ms);

/* 1 when the base must not follow its last drive command: none since power-up, or none
 * for COMMAND_TIMEOUT_MS. */
int safety_command_expired(const safety_state *s, uint32_t now_ms);

/* Check one wheel: PWM above FAULT_PWM_PCT while the wheel stands still or turns against
 * it, for FAULT_MS, is an encoder fault (or a blocked wheel). */
void safety_check_wheel(safety_state *s, int wheel, float pwm_pct, float speed_mps, uint32_t now_ms);

/* S#: the operator saw the stop; clear the faults. */
void safety_clear_faults(safety_state *s);

/* FAULT_FLAGS and FLAG_COMMAND_TIMEOUT bits for E#. */
unsigned safety_flags(const safety_state *s, uint32_t now_ms);

#endif
