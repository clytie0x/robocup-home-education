/* Serial protocol of the wheeled base: read the Pi's messages, write the replies.
 *
 * Kept from the old firmware, so base/pi/wheels.py keeps working:
 *   F<vvv>|<s><www>#  forward vvv/100 m/s, turn s www/100 rad/s (left positive)
 *   B<vvv>|<s><www>#  the same, backward
 *   S#                stop (ramped)
 *   O#                reply Ox:+000.00,y:+000.00,th:+000.00#  (m, m, rad)
 *   R#                reset odometry
 * New (base/docs/firmware-design.md, section 2):
 *   M<vvv>#           lower the top speed to vvv/100 m/s
 *   E#                reply Eleft:+0000000,right:+0000000,pwm:+000,+000,flags:00#
 *   P#                reply Px:+0000000,y:+0000000,th:+0000000,t:0000000000#
 *                     (mm, mm, milliradians, ms since power-up when the pose was computed)
 * New for updates without the BOOT0 jumper and for tuning without a reflash:
 *   U#                reply Uok# and restart into the ST ROM bootloader, or U!busy#
 *   Uforce#           the same without the wheel-speed check (an encoder that counts
 *                     on its own must not lock out updates); still needs zero PWM
 *   V#                reply Vversion:1,build:<git hash>#
 *   D#                reply Dtarget:+0000,+0000,speed:+0000,+0000,pwm:+000,+000# (mm/s, %)
 *   K<name>=<value>#  set a tuning value (params.h); reply K<name>=<value in force>#
 *   K<name>?#         read one; refused with K!unknown#, K!range# or K!syntax#
 *
 * No hardware calls here, so the laptop tests cover all of it. */
#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>

#include "params.h"

/* The longest valid message is a K set, "Kbrake_at_zero=1" or a long value (under 32
 * bytes before '#'); the old ones are at most "F000|+000" (9). */
#define PROTOCOL_MAX_MESSAGE 32
#define PROTOCOL_MAX_REPLY 64

typedef enum {
    MSG_DRIVE,     /* F or B: v_mps and w_radps are set */
    MSG_STOP,      /* S# */
    MSG_ODOM,      /* O#: reply with the centimetre pose */
    MSG_POSE,      /* P#: reply with the fine pose and its timestamp */
    MSG_RESET,     /* R#: odometry back to zero */
    MSG_MAX_SPEED, /* M<vvv>#: max_speed_mps is set */
    MSG_ENCODERS,  /* E#: raw counts, PWM, fault flags */
    MSG_BOOTLOADER, /* U#: restart into the ROM bootloader for an update */
    MSG_BOOTLOADER_FORCE, /* Uforce#: the same, without the wheel-speed check */
    MSG_VERSION,   /* V# */
    MSG_DIAG,      /* D#: wheel targets, measured speeds, PWM */
    MSG_PARAM_SET, /* K<name>=<value>#: name and value are set */
    MSG_PARAM_GET, /* K<name>?#: name is set */
    MSG_PARAM_BAD  /* a K message that does not parse: answered with K!syntax# */
} msg_kind;

typedef struct {
    msg_kind kind;
    float v_mps;         /* MSG_DRIVE: forward speed, negative for B */
    float w_radps;       /* MSG_DRIVE: turn rate, left positive */
    float max_speed_mps; /* MSG_MAX_SPEED */
    char name[PARAM_NAME_MAX]; /* MSG_PARAM_SET, MSG_PARAM_GET */
    float value;         /* MSG_PARAM_SET */
} message;

typedef struct {
    char text[PROTOCOL_MAX_MESSAGE];
    int length;
    int broken; /* too long or hit a line error: drop it when its '#' comes */
} parser;

void parser_init(parser *p);

/* Feed one received byte. Returns 1 and fills *out when this byte ('#') ends a valid
 * message; returns 0 otherwise, including for malformed messages, which are dropped. */
int parser_feed(parser *p, uint8_t byte, message *out);

/* The UART flagged a parity or framing error: the message in progress cannot be trusted. */
void parser_line_error(parser *p);

/* Parse the text of one message without its '#'. Returns 1 if valid. */
int parse_message(const char *text, int length, message *out);

/* Replies. Each writes a NUL-terminated string into out (PROTOCOL_MAX_REPLY bytes) and
 * returns its length. */
int format_odom(char *out, float x_m, float y_m, float th_rad);
int format_pose(char *out, float x_m, float y_m, float th_rad, uint32_t t_ms);
int format_encoders(char *out, int32_t left, int32_t right, float pwm_left_pct,
                    float pwm_right_pct, unsigned flags);
/* K<name>=<value>#, the value with 6 decimals: Kkp=300.000000# */
int format_param(char *out, const char *name, float value);
/* m/s in, mm/s out */
int format_diag(char *out, const float target_mps[2], const float speed_mps[2],
                const float pwm_pct[2]);

#endif
