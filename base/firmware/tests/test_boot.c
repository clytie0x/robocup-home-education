#include <stdint.h>
#include <string.h>

#include "../boot.h"
#include "check.h"

static void crc_matches_zlib(void)
{
    CHECK(boot_crc32((const uint8_t *)"123456789", 9) == 0xCBF43926u);
    CHECK(boot_crc32((const uint8_t *)"", 0) == 0u);
}

static void bootloader_request_wins(void)
{
    unsigned crashes = 1;
    CHECK(boot_decide(RESET_SOFTWARE | RESET_PIN, BOOT_MAGIC, &crashes, 1) == BOOT_ROM);
    CHECK(crashes == 0u); /* U# is not a crash */
}

static void normal_starts_run_the_application(void)
{
    unsigned crashes = 2;
    CHECK(boot_decide(RESET_POWER | RESET_PIN, 0, &crashes, 1) == BOOT_APP);
    CHECK(crashes == 0u);
    crashes = 2;
    CHECK(boot_decide(RESET_PIN, 0, &crashes, 1) == BOOT_APP); /* RESET button */
    CHECK(crashes == 0u);
    crashes = 0;
    CHECK(boot_decide(0, 0, &crashes, 1) == BOOT_APP); /* Go from the ROM bootloader */
    CHECK(crashes == 0u);
}

static void repeated_crashes_go_to_the_bootloader(void)
{
    unsigned crashes = 0;
    /* On the F1 a watchdog or software reset also sets the pin flag. */
    CHECK(boot_decide(RESET_WATCHDOG | RESET_PIN, 0, &crashes, 1) == BOOT_APP);
    CHECK(boot_decide(RESET_SOFTWARE | RESET_PIN, 0, &crashes, 1) == BOOT_APP);
    CHECK(crashes == 2u);
    CHECK(boot_decide(RESET_WATCHDOG | RESET_PIN, 0, &crashes, 1) == BOOT_ROM);
    CHECK(crashes == 0u); /* after Go the application gets three new tries */
    CHECK(boot_decide(0, 0, &crashes, 1) == BOOT_APP);
}

static void broken_application_goes_to_the_bootloader(void)
{
    unsigned crashes = 0;
    CHECK(boot_decide(RESET_POWER, 0, &crashes, 0) == BOOT_ROM);
}

/* A small fake application laid out like the real one. */
#define FAKE_LENGTH 0x200u
static uint8_t image[FAKE_LENGTH + 4u];

static void put32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16);
    p[3] = (uint8_t)(v >> 24);
}

static void make_image(void)
{
    for (uint32_t i = 0; i < sizeof image; i++) {
        image[i] = (uint8_t)(i * 7u);
    }
    put32(image, RAM_END);
    put32(image + 4, APP_BASE + 0x151u);
    put32(image + APP_HEADER_OFFSET, APP_MAGIC);
    put32(image + APP_HEADER_OFFSET + 4, FAKE_LENGTH);
    put32(image + FAKE_LENGTH, boot_crc32(image, FAKE_LENGTH));
}

static int valid(void)
{
    return boot_app_valid(image, APP_BASE, APP_SPACE);
}

static void application_check(void)
{
    make_image();
    CHECK(valid());

    image[0x180] ^= 1u; /* one flipped bit anywhere in the code */
    CHECK(!valid());

    make_image();
    memset(image + 0x180, 0xFF, sizeof image - 0x180); /* write stopped half way */
    CHECK(!valid());

    memset(image, 0xFF, sizeof image); /* erased */
    CHECK(!valid());

    make_image();
    put32(image + APP_HEADER_OFFSET + 4, APP_SPACE); /* length past the flash */
    CHECK(!valid());

    make_image();
    put32(image, 0x08001000u); /* stack pointer not in RAM */
    put32(image + FAKE_LENGTH, boot_crc32(image, FAKE_LENGTH));
    CHECK(!valid());

    make_image();
    put32(image + 4, APP_BASE + 0x150u); /* reset vector without the Thumb bit */
    put32(image + FAKE_LENGTH, boot_crc32(image, FAKE_LENGTH));
    CHECK(!valid());

    make_image();
    put32(image + 4, STAGE1_BASE + 0x101u); /* reset vector outside the image */
    put32(image + FAKE_LENGTH, boot_crc32(image, FAKE_LENGTH));
    CHECK(!valid());
}

/* Feed a waveform given as (level, ms) segments, repeated `times` times, then `tail_ms` of
 * tail_level. Returns the first result that is not BREAK_WAIT, or BREAK_WAIT, and the ms
 * it took in *ms. */
typedef struct {
    int level;
    unsigned ms;
} segment;

static int run_line(const segment *s, int n, int times, int tail_level, unsigned tail_ms, unsigned *ms)
{
    break_detector d;
    int started = 0, r = BREAK_WAIT;
    *ms = 0;
    for (int t = 0; t < times; t++) {
        for (int i = 0; i < n; i++) {
            for (unsigned k = 0; k < s[i].ms; k++) {
                if (!started) {
                    break_init(&d, s[i].level);
                    started = 1;
                    continue;
                }
                ++*ms;
                if ((r = break_feed(&d, s[i].level)) != BREAK_WAIT) {
                    return r;
                }
            }
        }
    }
    for (unsigned k = 0; k < tail_ms; k++) {
        ++*ms;
        if ((r = break_feed(&d, tail_level)) != BREAK_WAIT) {
            return r;
        }
    }
    return r;
}

static void normal_lines_start_at_once(void)
{
    unsigned ms;
    const segment idle[] = {{1, 1}};
    CHECK(run_line(idle, 1, 1, 1, 1000, &ms) == BREAK_NONE && ms <= 50u); /* idle CH340 */
    const segment low[] = {{0, 1}};
    CHECK(run_line(low, 1, 1, 0, 1000, &ms) == BREAK_NONE && ms <= 280u); /* unpowered CH340 */
    /* The CH340 powers up while stage1 looks: one rising edge, then idle. */
    const segment power_up[] = {{0, 150}};
    CHECK(run_line(power_up, 1, 1, 1, 1000, &ms) == BREAK_NONE && ms <= 210u);
    /* 115200 baud traffic (wheels.py): at 1 ms samples, lows last a sample or two. */
    const segment traffic[] = {{1, 3}, {0, 1}, {1, 2}, {0, 2}};
    CHECK(run_line(traffic, 4, 50, 1, 0, &ms) == BREAK_NONE && ms <= 10u);
    /* A long break the Pi forgot to release, then quiet: one low phase only. */
    const segment held[] = {{0, 2000}};
    CHECK(run_line(held, 1, 1, 1, 1000, &ms) == BREAK_NONE);
}

static void pattern_from_the_pi_enters_the_bootloader(void)
{
    unsigned ms;
    /* Zero bytes at 50 baud, 8N1 and 8E1, from different starting points. */
    const segment n81[] = {{0, 180}, {1, 20}};
    const segment e81[] = {{0, 200}, {1, 20}};
    const segment mid[] = {{1, 7}, {0, 180}, {1, 20}};
    CHECK(run_line(n81, 2, 10, 1, 400, &ms) == BREAK_ROM);
    CHECK(run_line(e81, 2, 10, 1, 400, &ms) == BREAK_ROM);
    CHECK(run_line(mid, 3, 10, 1, 400, &ms) == BREAK_ROM);
    /* The ROM starts at most 300 ms after the line goes quiet (the last stop bit counts). */
    CHECK(run_line(n81, 2, 10, 1, 400, &ms) == BREAK_ROM);
    CHECK(ms > 10u * 200u - 20u && ms <= 10u * 200u - 1u + 300u);
    /* A pyserial break with sleep jitter. */
    const segment jitter[] = {{0, 215}, {1, 12}, {0, 190}, {1, 45}};
    CHECK(run_line(jitter, 4, 5, 1, 400, &ms) == BREAK_ROM);
}

static void incomplete_or_endless_patterns_start_the_application(void)
{
    unsigned ms;
    const segment n81[] = {{0, 180}, {1, 20}};
    /* Too few phases: the quiet line after them is just idle. */
    CHECK(run_line(n81, 2, 3, 1, 400, &ms) == BREAK_NONE);
    /* Seen, but the line never goes quiet: give up and run the application. */
    CHECK(run_line(n81, 2, 400, 1, 0, &ms) == BREAK_NONE && ms >= BREAK_IDLE_TIMEOUT_MS);
    /* Phases out of their windows. */
    const segment slow[] = {{0, 300}, {1, 20}};
    const segment short_low[] = {{0, 100}, {1, 20}};
    const segment long_high[] = {{0, 180}, {1, 60}};
    CHECK(run_line(slow, 2, 10, 1, 400, &ms) == BREAK_NONE);
    CHECK(run_line(short_low, 2, 10, 1, 400, &ms) == BREAK_NONE);
    CHECK(run_line(long_high, 2, 10, 1, 400, &ms) == BREAK_NONE);
}

void test_boot(void)
{
    normal_lines_start_at_once();
    pattern_from_the_pi_enters_the_bootloader();
    incomplete_or_endless_patterns_start_the_application();
    crc_matches_zlib();
    bootloader_request_wins();
    normal_starts_run_the_application();
    repeated_crashes_go_to_the_bootloader();
    broken_application_goes_to_the_bootloader();
    application_check();
}
