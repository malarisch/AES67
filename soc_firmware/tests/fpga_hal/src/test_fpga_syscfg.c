/*
 * Unit tests for the backend-neutral system_cfg reader
 * (drivers/fpga_hal/fpga_hal_syscfg.c).
 *
 * These CSRs carry the gateware's build-time configuration, and the
 * firmware dispatches on them at boot: hardware vs. software PTP, and
 * the stream/channel counts the NMOS node reports as its resources. A
 * mis-parsed bitfield here silently advertises hardware that isn't
 * there.
 *
 * The mock's counts are set to non-default values in prj.conf, so the
 * expected values below cannot be satisfied by accident.
 */

#include <zephyr/ztest.h>
#include <errno.h>
#include <string.h>

#include "fpga_hal.h"
#include "eth_litex/eth_litex.h"

/* Mirrors the packing in fpga_hal_mock.c / the gateware's syscfg record. */
#define PACK(reg, field, v)                                                   \
	(((uint32_t)(v) &                                                     \
	  ((1UL << CSR_AES67_CSR_SYSTEM_CFG_##reg##_##field##_SIZE) - 1))     \
	 << CSR_AES67_CSR_SYSTEM_CFG_##reg##_##field##_OFFSET)

ZTEST(fpga_syscfg, test_read_reflects_kconfig)
{
	struct fpga_hal_system_cfg cfg;

	memset(&cfg, 0xFF, sizeof(cfg));
	zassert_equal(fpga_hal_read_system_cfg(&cfg), 0);

	zassert_false(cfg.ptp_in_software,
		      "the mock emulates hardware-PTP gateware");
	zassert_false(cfg.static_ptp_config);
	zassert_true(cfg.metering);

	zassert_equal(cfg.tx_max_streams, CONFIG_FPGA_HAL_MOCK_TX_STREAMS);
	zassert_equal(cfg.tx_channels, CONFIG_FPGA_HAL_MOCK_TX_CHANNELS);
	zassert_equal(cfg.rx_max_streams, CONFIG_FPGA_HAL_MOCK_RX_STREAMS);
	zassert_equal(cfg.rx_channels, CONFIG_FPGA_HAL_MOCK_RX_CHANNELS);

	zassert_equal(cfg.rx_buffer_depth, 256);
	zassert_equal(cfg.tx_buffer_depth, 64);
}

ZTEST(fpga_syscfg, test_load_populates_cache)
{
	zassert_equal(fpga_hal_syscfg_load(), 0);
	zassert_true(fpga_hal_syscfg_valid());

	const struct fpga_hal_system_cfg *c = fpga_hal_syscfg();

	zassert_not_null(c);
	zassert_equal(c->tx_max_streams, CONFIG_FPGA_HAL_MOCK_TX_STREAMS);
	zassert_equal(c->rx_channels, CONFIG_FPGA_HAL_MOCK_RX_CHANNELS);
	zassert_equal(fpga_hal_ptp_in_software(), c->ptp_in_software);
}

ZTEST(fpga_syscfg, test_field_decoding_is_independent)
{
	/* Drive the CSRs directly with values that would alias if an
	 * offset or width were wrong (every field distinct, high bits set
	 * in the widest one). */
	struct fpga_hal_system_cfg cfg;
	uint32_t saved_flags, saved_rx, saved_tx;

	zassert_equal(fpga_hal_csr_read(CSR_AES67_CSR_SYSTEM_CFG_FLAGS_ADDR,
					&saved_flags), 0);
	zassert_equal(fpga_hal_csr_read(CSR_AES67_CSR_SYSTEM_CFG_RX_ADDR,
					&saved_rx), 0);
	zassert_equal(fpga_hal_csr_read(CSR_AES67_CSR_SYSTEM_CFG_TX_ADDR,
					&saved_tx), 0);

	zassert_equal(fpga_hal_csr_write(
		CSR_AES67_CSR_SYSTEM_CFG_FLAGS_ADDR,
		PACK(FLAGS, PTP_IN_SOFTWARE, 1) |
		PACK(FLAGS, STATIC_PTP_CONFIG, 1) |
		PACK(FLAGS, METERING, 0)), 0);
	zassert_equal(fpga_hal_csr_write(
		CSR_AES67_CSR_SYSTEM_CFG_RX_ADDR,
		PACK(RX, MAX_STREAMS, 5) | PACK(RX, CHANNELS, 0xA5) |
		PACK(RX, BUFFER_DEPTH, 4096)), 0);
	zassert_equal(fpga_hal_csr_write(
		CSR_AES67_CSR_SYSTEM_CFG_TX_ADDR,
		PACK(TX, MAX_STREAMS, 0xFF) | PACK(TX, CHANNELS, 1) |
		PACK(TX, BUFFER_DEPTH, 0)), 0);

	zassert_equal(fpga_hal_read_system_cfg(&cfg), 0);

	zassert_true(cfg.ptp_in_software);
	zassert_true(cfg.static_ptp_config);
	zassert_false(cfg.metering);
	zassert_equal(cfg.rx_max_streams, 5);
	zassert_equal(cfg.rx_channels, 0xA5);
	zassert_equal(cfg.rx_buffer_depth, 4096);
	zassert_equal(cfg.tx_max_streams, 0xFF);
	zassert_equal(cfg.tx_channels, 1);
	zassert_equal(cfg.tx_buffer_depth, 0);

	/* Restore, and make sure the cache follows a re-load (the value is
	 * per bitstream, so the FPGA-recovery path re-reads it). */
	zassert_equal(fpga_hal_csr_write(CSR_AES67_CSR_SYSTEM_CFG_FLAGS_ADDR,
					 saved_flags), 0);
	zassert_equal(fpga_hal_csr_write(CSR_AES67_CSR_SYSTEM_CFG_RX_ADDR,
					 saved_rx), 0);
	zassert_equal(fpga_hal_csr_write(CSR_AES67_CSR_SYSTEM_CFG_TX_ADDR,
					 saved_tx), 0);
	zassert_equal(fpga_hal_syscfg_load(), 0);
	zassert_false(fpga_hal_ptp_in_software());
	zassert_equal(fpga_hal_syscfg()->rx_max_streams,
		      CONFIG_FPGA_HAL_MOCK_RX_STREAMS);
}

ZTEST_SUITE(fpga_syscfg, NULL, NULL, NULL, NULL, NULL);
