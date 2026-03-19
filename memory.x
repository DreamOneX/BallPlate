/*
  STM32F407ZGTx
  Flash : 1024K
  RAM   : 192K (112K SRAM1 + 16K SRAM2 + 64K CCM)

  Keep it minimal for now; we can refine origins if we decide to place stacks/heap.
*/

MEMORY
{
  FLASH : ORIGIN = 0x08000000, LENGTH = 1024K
  RAM   : ORIGIN = 0x20000000, LENGTH = 192K
}

/* Required by cortex-m-rt */

ENTRY(Reset);

EXTERN(__RESET_VECTOR);

SECTIONS
{
  .vector_table ORIGIN(FLASH) :
  {
    KEEP(*(.vector_table .vector_table.*));
  } > FLASH

  .text :
  {
    *(.text .text.*);
    *(.rodata .rodata.*);
    . = ALIGN(4);
  } > FLASH

  .data : AT (ADDR(.text) + SIZEOF(.text))
  {
    . = ALIGN(4);
    __sdata = .;
    *(.data .data.*);
    . = ALIGN(4);
    __edata = .;
  } > RAM

  .bss :
  {
    . = ALIGN(4);
    __sbss = .;
    *(.bss .bss.*);
    *(COMMON);
    . = ALIGN(4);
    __ebss = .;
  } > RAM

  /DISCARD/ : { *(.eh_frame .eh_frame.*) }
}
