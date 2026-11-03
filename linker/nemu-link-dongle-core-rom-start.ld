
/* The nRF52833 vector table and 48-entry IRQ vector table end at 0x1100. */
. = 0x00000100;
__nemu_link_dongle_core_start = .;
KEEP(*(.nemu_link_dongle_core.text))
. = ALIGN(4);
__nemu_link_dongle_core_end = .;
