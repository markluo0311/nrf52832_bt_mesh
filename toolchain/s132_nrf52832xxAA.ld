/* Linker script to configure memory regions. */

SEARCH_DIR(.)
GROUP(-lgcc -lc -lnosys)

MEMORY
{
    FLASH (rx)      : ORIGIN = 0x0001F000, LENGTH = 0x59000
    RAM (rwx)       : ORIGIN = 0x200022D8, LENGTH = 0xCD28
    /* Reserve space for the bootloader */
    /* CORE_BL_RAM (rw): ORIGIN = 0x2000F000, LENGTH = 0x1000 */
}

INCLUDE "nrf5x_common.ld"
