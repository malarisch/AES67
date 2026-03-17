/*
 * Test BSS-zeroing like Zephyr does, then print to UART.
 * Zeros 64 KB of RAM (like z_bss_zero), then prints result.
 *
 * Build:
 *   riscv64-unknown-elf-as -march=rv32im -mabi=ilp32 -o uart_test_bss.o uart_test_bss.S
 *   riscv64-unknown-elf-ld -m elf32lriscv -T uart_test_big.ld -o uart_test_bss.elf uart_test_bss.o
 *   riscv64-unknown-elf-objcopy -O binary uart_test_bss.elf uart_test_bss.bin
 */

.section .text
.global _start

.equ UART_RXTX,   0xF0005000
.equ UART_TXFULL, 0xF0005004

/* Simulate Zephyr: set mtvec, jump to init far away */
_start:
    auipc   t0, 0
    addi    t0, t0, 20         /* point to trap_handler below */
    csrw    mtvec, t0
    j       far_init

/* Simple trap handler — print 'E' for exception and loop */
trap_handler:
    li      a0, 'E'
    jal     uart_putc
    li      a0, '!'
    jal     uart_putc
    li      a0, '\n'
    jal     uart_putc
1:  j       1b

uart_putc:
    li      t0, UART_TXFULL
1:  lw      t1, 0(t0)
    bnez    t1, 1b
    li      t0, UART_RXTX
    sw      a0, 0(t0)
    ret

/* Pad to push far_init to a high offset (like Zephyr's __initialize) */
.balign 4
.rept 10000
    nop
.endr

far_init:
    /* Set up stack in upper HyperRAM area */
    li      sp, 0x200B0000

    /* Print 'I' — reached far_init */
    li      a0, 'I'
    jal     uart_putc

    /* Zero 64 KB of RAM at 0x20078000 (simulating BSS zero) */
    li      t0, 0x20078000
    li      t1, 0x20088000      /* +64 KB */
zero_loop:
    sw      zero, 0(t0)
    addi    t0, t0, 4
    blt     t0, t1, zero_loop

    /* Print 'B' — BSS zeroed OK */
    li      a0, 'B'
    jal     uart_putc

    /* Print '\n' */
    li      a0, '\n'
    jal     uart_putc

    /* Done, loop */
1:  j       1b
