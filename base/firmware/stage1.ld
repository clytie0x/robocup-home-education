/* stage1: the first 4 KiB of flash (boot.h). It has no variables in RAM: nothing copies
   .data or clears .bss, so the link fails if either is used. */
MEMORY
{
    FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 4K
    RAM   (rwx) : ORIGIN = 0x20000000, LENGTH = 20K
}

ENTRY(stage1_start)
_estack = ORIGIN(RAM) + LENGTH(RAM);

SECTIONS
{
    .isr_vector : { KEEP(*(.isr_vector)) } > FLASH
    .text : { *(.text*) *(.rodata*) . = ALIGN(4); } > FLASH
    .data : { *(.data*) *(.bss*) *(COMMON) } > RAM AT > FLASH
    ASSERT(SIZEOF(.data) == 0, "stage1 must not use variables in RAM")
}
