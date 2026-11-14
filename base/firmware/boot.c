#include "boot.h"

int boot_decide(unsigned reset_flags, unsigned magic, unsigned *crashes, int app_ok)
{
    if (reset_flags & RESET_POWER) {
        *crashes = 0; /* a fresh power-up forgets earlier crashes */
    } else if (reset_flags & (RESET_WATCHDOG | RESET_SOFTWARE)) {
        if (magic != BOOT_MAGIC) {
            *crashes += 1; /* the application hung or faulted */
        }
    } else if (reset_flags & RESET_PIN) {
        *crashes = 0; /* someone pressed RESET */
    }
    /* No flag at all: the ROM bootloader's Go command. Keep the count. */

    if (magic == BOOT_MAGIC || *crashes >= BOOT_MAX_CRASHES || !app_ok) {
        *crashes = 0; /* after the next flash or Go, the application gets fresh tries */
        return BOOT_ROM;
    }
    return BOOT_APP;
}

void break_init(break_detector *d, int rx_level)
{
    d->level = rx_level;
    d->ms = 1;
    d->first = 1;
    d->full_phases = 0;
    d->seen = 0;
    d->idle_ms = 0;
    d->waited_ms = 0;
}

int break_feed(break_detector *d, int rx_level)
{
    if (d->seen) {
        d->waited_ms++;
        d->idle_ms = rx_level ? d->idle_ms + 1u : 0u;
        if (d->idle_ms >= BREAK_IDLE_MS) {
            return BREAK_ROM;
        }
        return d->waited_ms >= BREAK_IDLE_TIMEOUT_MS ? BREAK_NONE : BREAK_WAIT;
    }
    if (rx_level == d->level) {
        d->ms++;
        return d->ms > (rx_level ? BREAK_HIGH_MAX_MS : BREAK_LOW_MAX_MS) ? BREAK_NONE : BREAK_WAIT;
    }
    /* A phase ended. The first one started before stage1 looked, so only its upper
     * limit (checked above) applies. */
    if (d->first) {
        d->first = 0;
    } else {
        if (d->ms < (d->level ? BREAK_HIGH_MIN_MS : BREAK_LOW_MIN_MS)) {
            return BREAK_NONE;
        }
        if (++d->full_phases >= BREAK_FULL_PHASES) {
            d->seen = 1;
        }
    }
    d->level = rx_level;
    d->ms = 1;
    return BREAK_WAIT;
}

uint32_t boot_crc32(const uint8_t *data, uint32_t length)
{
    /* Four bits at a time from a 16-entry table: fast enough on the 8 MHz reset clock
     * (about 30 ms for 15 KiB) and no RAM, since stage1 has none set up. */
    static const uint32_t table[16] = {
        0x00000000u, 0x1DB71064u, 0x3B6E20C8u, 0x26D930ACu, 0x76DC4190u, 0x6B6B51F4u,
        0x4DB26158u, 0x5005713Cu, 0xEDB88320u, 0xF00F9344u, 0xD6D6A3E8u, 0xCB61B38Cu,
        0x9B64C2B0u, 0x86D3D2D4u, 0xA00AE278u, 0xBDBDF21Cu,
    };
    uint32_t crc = 0xFFFFFFFFu;
    for (uint32_t i = 0; i < length; i++) {
        crc ^= data[i];
        crc = (crc >> 4) ^ table[crc & 0xFu];
        crc = (crc >> 4) ^ table[crc & 0xFu];
    }
    return ~crc;
}

static uint32_t read32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

int boot_app_valid(const uint8_t *image, uint32_t base, uint32_t space)
{
    /* Erased flash reads 0xFF: magic and length fail first, before any CRC. */
    if (read32(image + APP_HEADER_OFFSET) != APP_MAGIC) {
        return 0;
    }
    uint32_t length = read32(image + APP_HEADER_OFFSET + 4u);
    if (length < APP_HEADER_OFFSET + 8u || length > space - 4u) {
        return 0;
    }
    uint32_t sp = read32(image);
    uint32_t reset = read32(image + 4u);
    if (sp <= RAM_START || sp > RAM_END || (sp & 3u) != 0) {
        return 0;
    }
    if (!(reset & 1u) || reset < base + APP_HEADER_OFFSET || reset >= base + length) {
        return 0; /* must be Thumb code inside the image */
    }
    return boot_crc32(image, length) == read32(image + length);
}
