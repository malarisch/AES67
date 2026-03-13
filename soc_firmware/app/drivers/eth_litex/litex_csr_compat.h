/*
 * Compatibility shim to import LiteX-generated CSR definitions into Zephyr.
 *
 * The generated csr.h includes <system.h> and <hw/common.h> which are
 * LiteX BIOS headers (not available in Zephyr). We provide minimal stubs
 * so the #define constants (addresses, field offsets) and the inline
 * accessor functions can be used directly.
 *
 * Regenerate csr.h by running: python litex_soc/generate.py
 */

#ifndef LITEX_CSR_COMPAT_H_
#define LITEX_CSR_COMPAT_H_

#include <stdint.h>

/* --- Stub <system.h>: nothing needed, just suppress the include --- */

/* --- Stub <hw/common.h>: provide csr_read_simple / csr_write_simple --- */
#ifndef CSR_ACCESSORS_DEFINED
#define CSR_ACCESSORS_DEFINED

static inline void csr_write_simple(uint32_t v, unsigned long a)
{
	*(volatile uint32_t *)a = v;
}

static inline uint32_t csr_read_simple(unsigned long a)
{
	return *(volatile uint32_t *)a;
}

#endif /* CSR_ACCESSORS_DEFINED */

/* Now include the actual generated header.
 * The include path must be added in CMakeLists.txt:
 *   target_include_directories(app PRIVATE ${LITEX_GENERATED_DIR})
 */
#include <generated/csr.h>
#include <generated/mem.h>
#include <generated/soc.h>

#endif /* LITEX_CSR_COMPAT_H_ */
