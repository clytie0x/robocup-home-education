/* The packets here are the ones base/pi/tests/test_wheels.py expects from wheels.py,
 * so the Pi and the firmware agree on the wire format. */
#include "../protocol.h"
#include "check.h"

/* Feed a whole string; return how many valid messages it held, the last one in *last. */
static int feed(parser *p, const char *bytes, message *last)
{
    int count = 0;
    message m = {0};
    for (const char *c = bytes; *c; c++) {
        if (parser_feed(p, (uint8_t)*c, &m)) {
            *last = m;
            count++;
        }
    }
    return count;
}

static int feed_fresh(const char *bytes, message *last)
{
    parser p;
    parser_init(&p);
    return feed(&p, bytes, last);
}

static void check_drive(const char *packet, float v, float w)
{
    message m = {0};
    CHECK(feed_fresh(packet, &m) == 1);
    CHECK(m.kind == MSG_DRIVE);
    CHECK_NEAR(m.v_mps, v, 1e-6);
    CHECK_NEAR(m.w_radps, w, 1e-6);
}

static void drive_packets_from_the_pi(void)
{
    check_drive("F027|+000#", 0.27f, 0.0f);  /* test_packet_scales_and_formats */
    check_drive("F000|-034#", 0.0f, -0.34f);
    check_drive("F015|+000#", 0.15f, 0.0f);  /* test_packet_is_capped_and_never_backward */
    check_drive("F010|+000#", 0.10f, 0.0f);  /* test_silence_becomes_a_stop */
    check_drive("F008|+000#", 0.08f, 0.0f);  /* hardware assessment, silence test */
    check_drive("F014|+026#", 0.14f, 0.26f); /* hardware assessment, protocol table */
    check_drive("B010|+000#", -0.10f, 0.0f);
    check_drive("B012|-049#", -0.12f, -0.49f);
    check_drive("F999|+999#", 9.99f, 9.99f);
}

static void single_letter_messages(void)
{
    message m = {0};
    parser p;
    parser_init(&p);
    CHECK(feed(&p, "S#S#S#", &m) == 3); /* wheels.STOP_REPEATS stops in a row */
    CHECK(m.kind == MSG_STOP);
    CHECK(feed_fresh("O#", &m) == 1 && m.kind == MSG_ODOM);
    CHECK(feed_fresh("R#", &m) == 1 && m.kind == MSG_RESET);
    CHECK(feed_fresh("E#", &m) == 1 && m.kind == MSG_ENCODERS);
    CHECK(feed_fresh("P#", &m) == 1 && m.kind == MSG_POSE);
    CHECK(feed_fresh("M010#", &m) == 1 && m.kind == MSG_MAX_SPEED);
    CHECK_NEAR(m.max_speed_mps, 0.10f, 1e-6);
}

static void malformed_messages_are_dropped(void)
{
    const char *bad[] = {
        "F010|+00#", "F0100|+000#", "F01|+000#", "F010+000#", "F010|*000#", "F010|+0000#",
        "f010|+000#", "F0a0|+000#", "F010|+0 0#", "F-10|+000#", "X#", "#", "S1#", "Sx#",
        "O #", "M12#", "M0123#", "Ma10#", "E0#", "|+000#", "010|+000#",
    };
    for (unsigned i = 0; i < sizeof bad / sizeof bad[0]; i++) {
        message m = {0};
        int n = feed_fresh(bad[i], &m);
        if (n != 0) {
            printf("  accepted malformed \"%s\"\n", bad[i]);
        }
        CHECK(n == 0);
    }
}

static void stream_recovers(void)
{
    message m = {0};
    parser p;
    parser_init(&p);

    /* Line endings between messages are ignored. */
    CHECK(feed(&p, "F010|+000#\r\nS#", &m) == 2 && m.kind == MSG_STOP);

    /* The drive message lost its tail: the stop that follows still counts. */
    CHECK(feed(&p, "F010|+0S#", &m) == 1 && m.kind == MSG_STOP);

    /* A parity error inside a message drops that message only. */
    CHECK(feed(&p, "F01", &m) == 0);
    parser_line_error(&p);
    CHECK(feed(&p, "0|+000#", &m) == 0);
    CHECK(feed(&p, "S#", &m) == 1 && m.kind == MSG_STOP);

    /* An over-long run of junk is dropped, and the parser keeps working. */
    CHECK(feed(&p, "F01234567890123456789012345|+000#", &m) == 0);
    CHECK(feed(&p, "F012|+005#", &m) == 1 && m.kind == MSG_DRIVE);
    CHECK_NEAR(m.v_mps, 0.12f, 1e-6);
}

static void odometry_reply_keeps_the_old_format(void)
{
    char out[PROTOCOL_MAX_REPLY];
    /* The exact line FakePort in test_wheels.py returns, which parse_odom accepts. */
    CHECK(format_odom(out, 0.90f, 0.0f, 0.86f) == 32);
    CHECK_STR(out, "Ox:+000.90,y:+000.00,th:+000.86#");
    format_odom(out, -1.234f, 0.006f, -3.1416f);
    CHECK_STR(out, "Ox:-001.23,y:+000.01,th:-003.14#");
    format_odom(out, -0.004f, 0.0f, 12.566f); /* tiny negative rounds to +000.00; th not wrapped */
    CHECK_STR(out, "Ox:+000.00,y:+000.00,th:+012.57#");
    format_odom(out, 1234.5f, -1234.5f, 0.0f); /* held at the field's width */
    CHECK_STR(out, "Ox:+999.99,y:-999.99,th:+000.00#");
}

static void fine_pose_reply(void)
{
    char out[PROTOCOL_MAX_REPLY];
    format_pose(out, 1.2346f, -0.0004f, 3.14159f, 123456u);
    CHECK_STR(out, "Px:+0001235,y:+0000000,th:+0003142,t:0000123456#");
    format_pose(out, -0.902f, 0.25f, -0.862f, 4294967295u);
    CHECK_STR(out, "Px:-0000902,y:+0000250,th:-0000862,t:4294967295#");
}

static void encoder_reply(void)
{
    char out[PROTOCOL_MAX_REPLY];
    format_encoders(out, 1320, -45, 37.4f, -100.0f, 0x05u);
    CHECK_STR(out, "Eleft:+0001320,right:-0000045,pwm:+037,-100,flags:05#");
    format_encoders(out, 12345678, 0, 0.0f, 0.0f, 0u); /* only the last 7 digits fit */
    CHECK_STR(out, "Eleft:+2345678,right:+0000000,pwm:+000,+000,flags:00#");
}

void test_protocol(void)
{
    drive_packets_from_the_pi();
    single_letter_messages();
    malformed_messages_are_dropped();
    stream_recovers();
    odometry_reply_keeps_the_old_format();
    fine_pose_reply();
    encoder_reply();
}
