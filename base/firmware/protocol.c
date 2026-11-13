#include "protocol.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

/* Every message starts with one of these capital letters, and none of them can appear
 * inside a message. So when one arrives, a new message begins: if a byte of the previous
 * message (even its '#') was lost, the next S# still parses instead of being glued onto
 * the broken one. */
static const char START_LETTERS[] = "FBSORMEPUVDK";

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

/* One-letter messages: S, O, R, E, P, U, V, D. */
static int parse_single(char letter, message *out)
{
    switch (letter) {
    case 'S': out->kind = MSG_STOP; return 1;
    case 'O': out->kind = MSG_ODOM; return 1;
    case 'R': out->kind = MSG_RESET; return 1;
    case 'E': out->kind = MSG_ENCODERS; return 1;
    case 'P': out->kind = MSG_POSE; return 1;
    case 'U': out->kind = MSG_BOOTLOADER; return 1;
    case 'V': out->kind = MSG_VERSION; return 1;
    case 'D': out->kind = MSG_DIAG; return 1;
    default: return 0;
    }
}

/* "12.5", "-0.25", "300": optional sign, digits, optional decimals, at most 9 digits in
 * all (so they fit a uint32_t). No exponent: 'E' would start a new message. */
static int parse_value(const char *s, int length, float *out)
{
    static const float POW10[10] = {1e0f, 1e1f, 1e2f, 1e3f, 1e4f, 1e5f, 1e6f, 1e7f, 1e8f, 1e9f};
    int i = 0, digits = 0, decimals = -1;
    int negative = 0;
    uint32_t n = 0;
    if (i < length && (s[i] == '+' || s[i] == '-')) {
        negative = s[i] == '-';
        i++;
    }
    for (; i < length; i++) {
        if (s[i] == '.' && decimals < 0) {
            decimals = 0;
        } else if (s[i] >= '0' && s[i] <= '9' && digits < 9) {
            n = n * 10u + (uint32_t)(s[i] - '0');
            digits++;
            if (decimals >= 0) {
                decimals++;
            }
        } else {
            return 0;
        }
    }
    if (digits == 0) {
        return 0;
    }
    float v = (float)n / POW10[decimals > 0 ? decimals : 0];
    *out = negative ? -v : v;
    return 1;
}

/* "Kkp=300" or "Kkp?". Names are lower case, digits and '_' (no capital: it would start
 * a new message). A broken one becomes MSG_PARAM_BAD, so the Pi gets an answer. */
static int parse_param(const char *text, int length, message *out)
{
    int i = 1;
    while (i < length && ((text[i] >= 'a' && text[i] <= 'z') || (text[i] >= '0' && text[i] <= '9') ||
                          text[i] == '_')) {
        i++;
    }
    int name_length = i - 1;
    out->kind = MSG_PARAM_BAD;
    if (name_length < 1 || name_length >= PARAM_NAME_MAX || i >= length) {
        return 1;
    }
    memcpy(out->name, text + 1, (size_t)name_length);
    out->name[name_length] = '\0';
    if (text[i] == '?' && i == length - 1) {
        out->kind = MSG_PARAM_GET;
    } else if (text[i] == '=' && parse_value(text + i + 1, length - i - 1, &out->value)) {
        out->kind = MSG_PARAM_SET;
    }
    return 1;
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
    if (text[0] == 'K') {
        return parse_param(text, length, out);
    }
    if (length == 6 && memcmp(text, "Uforce", 6) == 0) {
        out->kind = MSG_BOOTLOADER_FORCE;
        return 1;
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
#define CENTI_TEXT 16 /* 8 are used; the compiler cannot see that n stays in +-99999 */
static void centi_text(char text[CENTI_TEXT], float value)
{
    long n = scaled(value, 100.0f, 99999);
    long a = n < 0 ? -n : n;
    snprintf(text, CENTI_TEXT, "%c%03ld.%02ld", n < 0 ? '-' : '+', a / 100, a % 100);
}

int format_odom(char *out, float x_m, float y_m, float th_rad)
{
    char x[CENTI_TEXT], y[CENTI_TEXT], th[CENTI_TEXT];
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

int format_param(char *out, const char *name, float value)
{
    float a = fabsf(value);
    long whole = (long)a;
    long micro = lroundf((a - (float)whole) * 1e6f);
    if (micro >= 1000000L) {
        whole++;
        micro -= 1000000L;
    }
    return snprintf(out, PROTOCOL_MAX_REPLY, "K%s=%c%ld.%06ld#", name, value < 0.0f ? '-' : '+',
                    whole, micro);
}

int format_diag(char *out, const float target_mps[2], const float speed_mps[2],
                const float pwm_pct[2])
{
    return snprintf(out, PROTOCOL_MAX_REPLY, "Dtarget:%+05ld,%+05ld,speed:%+05ld,%+05ld,pwm:%+04ld,%+04ld#",
                    scaled(target_mps[0], 1000.0f, 9999), scaled(target_mps[1], 1000.0f, 9999),
                    scaled(speed_mps[0], 1000.0f, 9999), scaled(speed_mps[1], 1000.0f, 9999),
                    scaled(pwm_pct[0], 1.0f, 100), scaled(pwm_pct[1], 1.0f, 100));
}
