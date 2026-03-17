MEMORY {
    ram (rwx) : ORIGIN = 0x20000000, LENGTH = 0x100000
}

SECTIONS {
    .text : { *(.text) } > ram
}
