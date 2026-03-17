/* Linker script for the boot stub.
 * Code is placed at the SPI flash reset address. */

MEMORY {
    flash (rx) : ORIGIN = 0x30000000, LENGTH = 0x200
}

SECTIONS {
    .text : {
        *(.text)
    } > flash
}
