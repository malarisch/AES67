MEMORY {
    ram (rwx) : ORIGIN = 0x20000000, LENGTH = 0x1000
}

SECTIONS {
    .text : { *(.text) } > ram
}
