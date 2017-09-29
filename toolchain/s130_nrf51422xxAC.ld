/* Linker script to configure memory regions. */

SEARCH_DIR(.)
GROUP(-lgcc -lc -lnosys)

MEMORY
{
    FLASH (rx)      : ORIGIN = 0x0001b000, LENGTH = 0x1F000
    RAM (rwx)       : ORIGIN = 0x200022D8, LENGTH = 0x5A28
    /* Reserve space for the bootloader */
    /* CORE_BL_RAM (rw): ORIGIN = 0x20007D00, LENGTH = 0x2FC */
}

INCLUDE "nrf5x_common.ld"
