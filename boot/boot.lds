SECTIONS
{
  . = 0x10020;

  .text : {
    KEEP(*(.boot_head))
    KEEP(*(.start))
    *(.text*)
  }
}
