/* The application on the STM32F103C8: from APP_BASE (boot.h) to the end of the 64 KiB the
   C8 guarantees (the chip answers for 128 KiB), 20 KiB RAM. stage1 owns the first 4 KiB.
   The last 4 bytes are kept for the CRC-32 tools/image.py appends after the image. */
MEMORY
{
    FLASH (rx)  : ORIGIN = 0x08001000, LENGTH = 60K - 4
    RAM   (rwx) : ORIGIN = 0x20000000, LENGTH = 20K
}

ENTRY(Reset_Handler)
_estack = ORIGIN(RAM) + LENGTH(RAM);

SECTIONS
{
    /* The header sits at a fixed place after the vector table: stage1 and image.py look
       for it at APP_BASE + 0x100. */
    .isr_vector : { KEEP(*(.isr_vector)) . = 0x100; KEEP(*(.app_header)) } > FLASH
    .text : { *(.text*) *(.rodata*) . = ALIGN(4); } > FLASH
    .ARM.exidx : { *(.ARM.exidx*) } > FLASH

    _sidata = LOADADDR(.data);
    .data : { _sdata = .; *(.data*) . = ALIGN(4); _edata = .; } > RAM AT > FLASH
    .bss (NOLOAD) : { _sbss = .; *(.bss*) *(COMMON) . = ALIGN(4); _ebss = .; } > RAM

    /* newlib's _sbrk grows the heap from here; snprintf may ask for a little. */
    end = .;
    /* Keep at least 2 KiB between the variables and the top of RAM for the stack. */
    ASSERT(end + 2K <= _estack, "RAM full: no room left for the stack")
}
