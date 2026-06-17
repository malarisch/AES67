CROSS = riscv64-unknown-elf-

# Per-target boot stubs.  TARGET_xxx defines select the RAM layout and
# whether to run the HyperRAM reset/latency sequence.  See boot_stub.S.
VARIANTS = cyclone10 cyc1000

all: $(addsuffix .bin,$(addprefix boot_stub_,$(VARIANTS)))

# Map variant -> CPP define
DEFINE_cyclone10 = -DTARGET_CYCLONE10
DEFINE_cyc1000   = -DTARGET_CYC1000

# LiteX-generated CSR map per target.  The stub's UART/HyperRAM addresses must
# track this map (it shifts when peripherals are added/removed), so we extract
# the live addresses from csr.csv and pass them as -D overrides.  If the csv is
# absent the stub falls back to the hardcoded defaults in boot_stub.S.
BUILD_DIR = ../build

boot_stub_%.elf: boot_stub.S boot_stub.ld
	@CSV=$(BUILD_DIR)/$*/csr.csv; DEFS=""; \
	if [ -f "$$CSV" ]; then \
	    U=$$(awk -F, '$$1=="csr_register"&&$$2=="uart_rxtx"{print $$3}' "$$CSV"); \
	    H=$$(awk -F, '$$1=="csr_register"&&$$2=="hyperram_config"{print $$3}' "$$CSV"); \
	    [ -n "$$U" ] && DEFS="$$DEFS -DUART_RXTX_ADDR=$$U"; \
	    [ -n "$$H" ] && DEFS="$$DEFS -DHYPERRAM_CONFIG_ADDR=$$H"; \
	    echo "boot_stub $*: CSR map from $$CSV:$$DEFS"; \
	else \
	    echo "boot_stub $*: WARNING $$CSV not found, using boot_stub.S fallback addresses"; \
	fi; \
	$(CROSS)gcc -march=rv32i -mabi=ilp32 -nostdlib \
	    $(DEFINE_$*) $$DEFS \
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
