/*
 * SPI Flash -> HyperRAM boot stub for LiteX VexRiscv.
 *
 * This tiny stub is placed at the CPU reset address (start of SPI flash).
 *
 * It resets the HyperRAM chip via the CSR (pulse stretcher in the SoC
 * extends this to 32 sys_clk cycles = 400 ns), waits for the chip to
 * recover, sets the HyperRAM controller latency to match the chip's
 * power-on default (6 CK), copies the BIOS image from SPI flash into
 * HyperRAM, flushes caches, and jumps there.
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

.equ UART_RXTX,    0xF0005800
.equ UART_TXFULL,  0xF0005804

_start:
    li      sp, 0x10001000

    /* --- Debug: print "B" (Boot stub entered) --- */
    li      s0, UART_RXTX
    li      a0, 'B'
    jal     ra, uart_putc

    /*
     * Reset the HyperRAM chip via CSR.
     */
    li      t0, HYPERRAM_CFG
    li      t1, (HR_LATENCY << 8) | 1  /* rst=1 (bit 0) + latency=6 (bits 15:8) */
    sw      t1, 0(t0)

    /* --- Debug: print "R" (RST# triggered) --- */
    li      a0, 'R'
    jal     ra, uart_putc

    /*
     * Wait for HyperRAM chip recovery.
     */
    li      t2, 10000
.rst_wait:
    addi    t2, t2, -1
    bnez    t2, .rst_wait

    /* Set HyperRAM controller latency to match chip power-on default (6 CK). */
    li      t0, HYPERRAM_CFG
    li      t1, (HR_LATENCY << 8)
    sw      t1, 0(t0)

    /* --- Debug: print "L" (Latency set) --- */
    li      a0, 'L'
    jal     ra, uart_putc

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

    /* --- Debug: print "C" (Copy done) --- */
    li      s0, UART_RXTX
    li      a0, 'C'
    jal     ra, uart_putc

    /* Flush D-cache then I-cache */
    .word   0x500F
    .word   0x100F
    nop
    nop
    nop
    nop
    nop

    /* --- Debug: print "J\n" (Jumping to BIOS) --- */
    li      s0, UART_RXTX
    li      a0, 'J'
    jal     ra, uart_putc
    li      a0, '\n'
    jal     ra, uart_putc

    /* Jump to BIOS in HyperRAM */
    li      t0, BIOS_DST
    jr      t0

/*
 * uart_putc: send byte in a0 via LiteX UART.
 * Clobbers: t3, t4.  Expects s0 = UART_RXTX base address.
 */
uart_putc:
    li      t3, UART_TXFULL
.uart_wait:
    lw      t4, 0(t3)
    bnez    t4, .uart_wait       /* spin while TX FIFO full */
    sw      a0, 0(s0)            /* write byte to RXTX */
    ret
