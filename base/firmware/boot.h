/* Boot decisions of stage1, the small loader at the start of flash that is written once
 * (with the BOOT0 jumper) and never again. Every later update rewrites only the
 * application above it, through the ST ROM bootloader, without the jumper:
 *
 *   0x08000000  stage1 (4 KiB): pick the application or the ROM bootloader
 *   0x08001000  application: vector table, header at +0x100, code, CRC-32 after the end
 *
 * No hardware calls here, so the laptop tests cover all of it. */
#ifndef BOOT_H
#define BOOT_H

#include <stdint.h>

#define STAGE1_BASE 0x08000000u
#define APP_BASE 0x08001000u
#define APP_SPACE (0x08010000u - APP_BASE) /* to the end of the 64 KiB the C8 guarantees */
#define APP_HEADER_OFFSET 0x100u           /* after the 59-entry vector table */
#define APP_MAGIC 0x31505041u              /* "APP1" */
#define RAM_START 0x20000000u
#define RAM_END 0x20005000u                /* 20 KiB */
#define ROM_BOOTLOADER 0x1FFFF000u         /* ST system memory: MSP at +0, reset at +4 */

/* Backup register 1 holds this after U#: the next start goes to the ROM bootloader. */
#define BOOT_MAGIC 0xB007u
/* This many watchdog or fault resets in a row, each before the application ran 2 s,
 * and stage1 sends the next start to the ROM bootloader so the Pi can reflash. */
#define BOOT_MAX_CRASHES 3u

/* Why the chip started (from RCC_CSR). None of them set: the ROM bootloader's Go
 * command jumped here without a reset. */
#define RESET_POWER 0x1u    /* power-on or brown-out */
#define RESET_WATCHDOG 0x2u /* the independent watchdog fired */
#define RESET_SOFTWARE 0x4u /* SYSRESETREQ: U#, or the fault handler */
#define RESET_PIN 0x8u      /* the RESET button (on the F1 also set with the two above) */

enum { BOOT_APP = 0, BOOT_ROM = 1 };

/* Where to go: BOOT_APP or BOOT_ROM. *crashes is backup register 2, updated in place;
 * stage1 writes it back. */
int boot_decide(unsigned reset_flags, unsigned magic, unsigned *crashes, int app_ok);

/* The usual CRC-32 (zlib, poly 0xEDB88320); "123456789" gives 0xCBF43926. */
uint32_t boot_crc32(const uint8_t *data, uint32_t length);

/* 1 if image (placed at address base, at most space bytes) is a whole application: header
 * magic, a length that fits, a stack pointer in RAM, a reset vector inside the image,
 * and the CRC-32 stored right after the image matches. */
int boot_app_valid(const uint8_t *image, uint32_t base, uint32_t space);

#endif
