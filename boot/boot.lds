SECTIONS
{
  . = 0x10000;

  .text : {
    KEEP(*(.boot_head))
    KEEP(*(.reset))
    *(.text*)
  }
}
