/*
 * SPI Flash -> HyperRAM boot stub for LiteX VexRiscv.
 *
 * This tiny stub is placed at the CPU reset address (start of SPI flash).
 * It sets the HyperRAM controller latency to match the chip's power-on
 * default (6 CK for IS66WVH8M8ALL), copies the BIOS image from SPI flash
 * into HyperRAM, flushes caches, and jumps there.
 *
 * Flash layout:
 *   0x30000000  boot_stub  (this file, <=512 bytes)
 *   0x30000200  bios.bin   (LiteX BIOS, up to 63.5 KB)
 *
 * BIOS linked at 0x207F0000 (top of 8 MB HyperRAM).
 */

.section .text
.global _start

.equ FLASH_BASE,    0x30000000
.equ STUB_SIZE,     0x200
.equ BIOS_SRC,      FLASH_BASE + STUB_SIZE
.equ BIOS_MAX_SIZE, 0x10000
.equ BIOS_DST,      0x20000000 + 0x800000 - BIOS_MAX_SIZE

.equ HYPERRAM_CFG,  0xF0001800
.equ HR_LATENCY,    6           /* IS66WVH8M8ALL power-on default */

_start:
    li      sp, 0x10001000

    /* Set HyperRAM controller latency to match chip power-on default */
    li      t0, HYPERRAM_CFG
    li      t1, (HR_LATENCY << 8)
    sw      t1, 0(t0)

    /* Copy BIOS from SPI flash to HyperRAM */
    li      a0, BIOS_DST
    li      a1, BIOS_SRC
    li      a2, BIOS_MAX_SIZE
copy_loop:
    lw      t0, 0(a1)
    sw      t0, 0(a0)
    addi    a0, a0, 4
    addi    a1, a1, 4
    addi    a2, a2, -4
    bnez    a2, copy_loop

    /* Flush D-cache then I-cache */
    .word   0x500F
    .word   0x100F
    nop
    nop
    nop
    nop
    nop

    /* Jump to BIOS in HyperRAM */
    li      t0, BIOS_DST
    jr      t0
