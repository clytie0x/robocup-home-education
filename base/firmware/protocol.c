#include "protocol.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

/* Every message starts with one of these capital letters, and none of them can appear
 * inside a message. So when one arrives, a new message begins: if a byte of the previous
 * message (even its '#') was lost, the next S# still parses instead of being glued onto
 * the broken one. */
static const char START_LETTERS[] = "FBSORMEP";

void parser_init(parser *p)
{
    p->length = 0;
    p->broken = 0;
}

void parser_line_error(parser *p)
{
    p->broken = 1;
}

static int is_start_letter(uint8_t byte)
{
    return byte != 0 && strchr(START_LETTERS, byte) != NULL;
}

int parser_feed(parser *p, uint8_t byte, message *out)
{
    if (byte == '\r' || byte == '\n') {
        return 0; /* line endings from a terminal or another driver carry no meaning */
    }
    if (is_start_letter(byte)) {
        parser_init(p);
    }
    if (byte == '#') {
        int ok = !p->broken && parse_message(p->text, p->length, out);
        parser_init(p);
        return ok;
    }
    if (p->length < PROTOCOL_MAX_MESSAGE) {
        p->text[p->length++] = (char)byte;
    } else {
        p->broken = 1; /* longer than any valid message */
    }
    return 0;
}

/* Three ASCII digits to 0..999, or -1 if any is not a digit. */
static int three_digits(const char *s)
{
    int n = 0;
    for (int i = 0; i < 3; i++) {
        if (s[i] < '0' || s[i] > '9') {
            return -1;
        }
        n = n * 10 + (s[i] - '0');
    }
    return n;
}

/* "F027|+000" or "B027|-034": exactly this shape, nothing else. */
static int parse_drive(const char *text, int length, message *out)
{
    if (length != 9 || text[4] != '|' || (text[5] != '+' && text[5] != '-')) {
        return 0;
    }
    int v = three_digits(text + 1);
    int w = three_digits(text + 6);
    if (v < 0 || w < 0) {
        return 0;
    }
    out->kind = MSG_DRIVE;
    out->v_mps = (text[0] == 'B' ? -v : v) / 100.0f;
    out->w_radps = (text[5] == '-' ? -w : w) / 100.0f;
    return 1;
}

static int parse_max_speed(const char *text, int length, message *out)
{
    int v = length == 4 ? three_digits(text + 1) : -1;
    if (v < 0) {
        return 0;
    }
    out->kind = MSG_MAX_SPEED;
    out->max_speed_mps = v / 100.0f;
    return 1;
}

/* One-letter messages: S, O, R, E, P. */
static int parse_single(char letter, message *out)
{
    switch (letter) {
    case 'S': out->kind = MSG_STOP; return 1;
    case 'O': out->kind = MSG_ODOM; return 1;
    case 'R': out->kind = MSG_RESET; return 1;
    case 'E': out->kind = MSG_ENCODERS; return 1;
    case 'P': out->kind = MSG_POSE; return 1;
    default: return 0;
    }
}

int parse_message(const char *text, int length, message *out)
{
    if (length < 1) {
        return 0;
    }
    if (text[0] == 'F' || text[0] == 'B') {
        return parse_drive(text, length, out);
    }
    if (text[0] == 'M') {
        return parse_max_speed(text, length, out);
    }
    return length == 1 && parse_single(text[0], out);
}

/* value * scale rounded to a whole number, held inside +-limit so the reply keeps its
 * fixed width. */
static long scaled(float value, float scale, long limit)
{
    float s = value * scale;
    if (s > (float)limit) {
        return limit;
    }
    if (s < -(float)limit) {
        return -limit;
    }
    if (s != s) {
        return 0; /* NaN cannot come from integer counts; keep the reply parseable anyway */
    }
    return lroundf(s);
}

/* "+000.00": sign, 3 digits, 2 decimals, as the old firmware writes. Rounds -0.004 to
 * "+000.00", not "-000.00". */
static void centi_text(char text[8], float value)
{
    long n = scaled(value, 100.0f, 99999);
    long a = n < 0 ? -n : n;
    snprintf(text, 8, "%c%03ld.%02ld", n < 0 ? '-' : '+', a / 100, a % 100);
}

int format_odom(char *out, float x_m, float y_m, float th_rad)
{
    char x[8], y[8], th[8];
    centi_text(x, x_m);
    centi_text(y, y_m);
    centi_text(th, th_rad);
    return snprintf(out, PROTOCOL_MAX_REPLY, "Ox:%s,y:%s,th:%s#", x, y, th);
}

int format_pose(char *out, float x_m, float y_m, float th_rad, uint32_t t_ms)
{
    return snprintf(out, PROTOCOL_MAX_REPLY, "Px:%+08ld,y:%+08ld,th:%+08ld,t:%010lu#",
                    scaled(x_m, 1000.0f, 9999999), scaled(y_m, 1000.0f, 9999999),
                    scaled(th_rad, 1000.0f, 9999999), (unsigned long)t_ms);
}

int format_encoders(char *out, int32_t left, int32_t right, float pwm_left_pct,
                    float pwm_right_pct, unsigned flags)
{
    /* Only the last 7 digits of the counts fit; calibration uses differences, which
     * stay right as long as a run turns a wheel less than 10 million counts. */
    return snprintf(out, PROTOCOL_MAX_REPLY, "Eleft:%+08ld,right:%+08ld,pwm:%+04ld,%+04ld,flags:%02X#",
                    (long)(left % 10000000), (long)(right % 10000000),
                    scaled(pwm_left_pct, 1.0f, 100), scaled(pwm_right_pct, 1.0f, 100),
                    flags & 0xFFu);
}
