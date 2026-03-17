/*
 * Minimal UART test — linked at 0x20000000.
 * Prints "OK\n" by polling UART TXFULL, then loops forever.
 *
 * Load via serialboot or flash, should print immediately after Liftoff.
 * If nothing prints, the problem is in the boot handoff, not Zephyr.
 *
 * Build:
 *   riscv64-unknown-elf-as -march=rv32im -mabi=ilp32 -o uart_test.o uart_test.S
 *   riscv64-unknown-elf-ld -T uart_test.ld -o uart_test.elf uart_test.o
 *   riscv64-unknown-elf-objcopy -O binary uart_test.elf uart_test.bin
 */

.section .text
.global _start

.equ UART_RXTX,   0xF0005000
.equ UART_TXFULL, 0xF0005004

_start:
    /* Print 'O' */
    li      a0, 'O'
    jal     uart_putc

    /* Print 'K' */
    li      a0, 'K'
    jal     uart_putc

    /* Print '\n' */
    li      a0, '\n'
    jal     uart_putc

    /* Loop forever */
1:  j       1b

/* uart_putc: send byte in a0 to UART */
uart_putc:
    li      t0, UART_TXFULL
1:  lw      t1, 0(t0)
    bnez    t1, 1b          /* spin while TX full */
    li      t0, UART_RXTX
    sw      a0, 0(t0)
    ret
