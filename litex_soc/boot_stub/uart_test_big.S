/*
 * UART test with size padding — prints "OK<size>\n" then loops.
 * Padded to ~128 KB to test if larger images break boot.
 *
 * Build:
 *   riscv64-unknown-elf-as -march=rv32im -mabi=ilp32 -o uart_test_big.o uart_test_big.S
 *   riscv64-unknown-elf-ld -m elf32lriscv -T uart_test.ld -o uart_test_big.elf uart_test_big.o
 *   riscv64-unknown-elf-objcopy -O binary uart_test_big.elf uart_test_big.bin
 */

.section .text
.global _start

.equ UART_RXTX,   0xF0005000
.equ UART_TXFULL, 0xF0005004

_start:
    li      a0, 'O'
    jal     uart_putc
    li      a0, 'K'
    jal     uart_putc
    li      a0, '1'
    jal     uart_putc
    li      a0, '2'
    jal     uart_putc
    li      a0, '8'
    jal     uart_putc
    li      a0, 'K'
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

/* Pad to 128 KB with NOPs */
.balign 4
.rept 32700
    nop
.endr
