/* Linker script to configure memory regions. */

SEARCH_DIR(.)
GROUP(-lgcc -lc -lnosys)

MEMORY
{
    FLASH (rx)      : ORIGIN = 0x00018000, LENGTH = 0x22000
    RAM (rwx)       : ORIGIN = 0x20002000, LENGTH = 0x5D00
    /* Reserve space for the bootloader */
    /* CORE_BL_RAM (rw): ORIGIN = 0x20007D00, LENGTH = 0x02FC */
}

INCLUDE "nrf5x_common.ld"
