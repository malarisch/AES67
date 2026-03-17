CROSS = riscv64-unknown-elf-

all: boot_stub.bin

boot_stub.elf: boot_stub.S boot_stub.ld
	$(CROSS)gcc -march=rv32i -mabi=ilp32 -nostdlib -T boot_stub.ld -o $@ boot_stub.S

boot_stub.bin: boot_stub.elf
	$(CROSS)objcopy -O binary $< $@
	@SIZE=$$(wc -c < $@); \
	if [ $$SIZE -gt 512 ]; then \
		echo "ERROR: boot_stub.bin is $$SIZE bytes (max 512)"; exit 1; \
	fi
	@echo "boot_stub.bin: $$(wc -c < $@) bytes"

clean:
	rm -f boot_stub.elf boot_stub.bin

.PHONY: all clean
