/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * Static FPGA build configuration (system_cfg CSRs) — backend-neutral.
 *
 * The aes67_bridge exposes the syscfg generic the gateware was built with
 * (FPGA/packages/system_cfg_pkg.vhd) through three read-only CSRs. This
 * module reads them via fpga_hal_csr_read() — identical addresses on the
 * LiteX and SPI backends — and caches the parsed result for the runtime
 * service dispatch (hardware vs. software PTP, metering, stream limits).
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <string.h>

#include "fpga_hal.h"

#if defined(CONFIG_FPGA_HAL_LITEX) || defined(CONFIG_FPGA_HAL_SPI) || \
	defined(CONFIG_FPGA_HAL_MOCK)
/* Macro layer only: CSR_AES67_CSR_SYSTEM_CFG_* addresses + field offsets
 * from the generated aes67_bridge headers. */
#include "../eth_litex/eth_litex.h"
#endif

LOG_MODULE_REGISTER(fpga_syscfg, LOG_LEVEL_INF);

static struct fpga_hal_system_cfg syscfg_cache;
static bool syscfg_loaded;

#ifdef CSR_AES67_CSR_SYSTEM_CFG_FLAGS_ADDR

#define SYSCFG_FIELD(reg, field, val) \
	(((val) >> CSR_AES67_CSR_SYSTEM_CFG_##reg##_##field##_OFFSET) & \
	 ((1UL << CSR_AES67_CSR_SYSTEM_CFG_##reg##_##field##_SIZE) - 1))

int fpga_hal_read_system_cfg(struct fpga_hal_system_cfg *cfg)
{
	uint32_t flags, rx, tx;
	int ret;

	ret = fpga_hal_csr_read(CSR_AES67_CSR_SYSTEM_CFG_FLAGS_ADDR, &flags);
	if (ret == 0) {
		ret = fpga_hal_csr_read(CSR_AES67_CSR_SYSTEM_CFG_RX_ADDR, &rx);
	}
	if (ret == 0) {
		ret = fpga_hal_csr_read(CSR_AES67_CSR_SYSTEM_CFG_TX_ADDR, &tx);
	}
	if (ret < 0) {
		return ret;
	}

	cfg->ptp_in_software   = SYSCFG_FIELD(FLAGS, PTP_IN_SOFTWARE, flags) != 0;
	cfg->static_ptp_config = SYSCFG_FIELD(FLAGS, STATIC_PTP_CONFIG, flags) != 0;
	cfg->metering          = SYSCFG_FIELD(FLAGS, METERING, flags) != 0;

	cfg->rx_max_streams  = SYSCFG_FIELD(RX, MAX_STREAMS, rx);
	cfg->rx_channels     = SYSCFG_FIELD(RX, CHANNELS, rx);
	cfg->rx_buffer_depth = SYSCFG_FIELD(RX, BUFFER_DEPTH, rx);

	cfg->tx_max_streams  = SYSCFG_FIELD(TX, MAX_STREAMS, tx);
	cfg->tx_channels     = SYSCFG_FIELD(TX, CHANNELS, tx);
	cfg->tx_buffer_depth = SYSCFG_FIELD(TX, BUFFER_DEPTH, tx);

	return 0;
}

#else /* !CSR_AES67_CSR_SYSTEM_CFG_FLAGS_ADDR (headers without system_cfg) */

int fpga_hal_read_system_cfg(struct fpga_hal_system_cfg *cfg)
{
	ARG_UNUSED(cfg);
	return -ENOTSUP;
}

#endif /* CSR_AES67_CSR_SYSTEM_CFG_FLAGS_ADDR */

int fpga_hal_syscfg_load(void)
{
	struct fpga_hal_system_cfg cfg;
	int ret = fpga_hal_read_system_cfg(&cfg);

	if (ret < 0) {
		LOG_ERR("system_cfg read failed (err %d)", ret);
		return ret;
	}

	syscfg_cache = cfg;
	syscfg_loaded = true;

	LOG_INF("FPGA build config: PTP=%s%s metering=%s "
		"RX %u streams/%u ch/depth %u, TX %u streams/%u ch/depth %u",
		cfg.ptp_in_software ? "software" : "hardware",
		cfg.static_ptp_config ? " (static servo cfg)" : "",
		cfg.metering ? "on" : "off",
		cfg.rx_max_streams, cfg.rx_channels, cfg.rx_buffer_depth,
		cfg.tx_max_streams, cfg.tx_channels, cfg.tx_buffer_depth);
	return 0;
}

const struct fpga_hal_system_cfg *fpga_hal_syscfg(void)
{
	return &syscfg_cache;
}

bool fpga_hal_syscfg_valid(void)
{
	return syscfg_loaded;
}

bool fpga_hal_ptp_in_software(void)
{
	return syscfg_cache.ptp_in_software;
}
