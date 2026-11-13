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

void test_boot(void)
{
    crc_matches_zlib();
    bootloader_request_wins();
    normal_starts_run_the_application();
    repeated_crashes_go_to_the_bootloader();
    broken_application_goes_to_the_bootloader();
    application_check();
}
