/* STM32F103C8: 64 KiB flash (the chip answers for 128 KiB, but only 64 are guaranteed),
   20 KiB RAM. */
MEMORY
{
    FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 64K
    RAM   (rwx) : ORIGIN = 0x20000000, LENGTH = 20K
}

ENTRY(Reset_Handler)
_estack = ORIGIN(RAM) + LENGTH(RAM);

SECTIONS
{
    .isr_vector : { KEEP(*(.isr_vector)) } > FLASH
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
