CROSS = riscv64-unknown-elf-

# Per-target boot stubs.  TARGET_xxx defines select the RAM layout and
# whether to run the HyperRAM reset/latency sequence.  See boot_stub.S.
VARIANTS = cyclone10 cyc1000

all: $(addsuffix .bin,$(addprefix boot_stub_,$(VARIANTS)))

# Map variant -> CPP define
DEFINE_cyclone10 = -DTARGET_CYCLONE10
DEFINE_cyc1000   = -DTARGET_CYC1000

boot_stub_%.elf: boot_stub.S boot_stub.ld
	$(CROSS)gcc -march=rv32i -mabi=ilp32 -nostdlib \
	    $(DEFINE_$*) \
	    -T boot_stub.ld -o $@ boot_stub.S

boot_stub_%.bin: boot_stub_%.elf
	$(CROSS)objcopy -O binary $< $@
	@SIZE=$$(wc -c < $@); \
	if [ $$SIZE -gt 512 ]; then \
		echo "ERROR: $@ is $$SIZE bytes (max 512)"; exit 1; \
	fi
	@echo "$@: $$(wc -c < $@) bytes"

# Backwards-compatible default: boot_stub.bin == cyclone10 variant.
boot_stub.bin: boot_stub_cyclone10.bin
	cp $< $@

clean:
	rm -f boot_stub.elf boot_stub.bin \
	      $(addsuffix .elf,$(addprefix boot_stub_,$(VARIANTS))) \
	      $(addsuffix .bin,$(addprefix boot_stub_,$(VARIANTS)))

.PHONY: all clean
