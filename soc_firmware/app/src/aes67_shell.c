/*
 * "aes67" shell command tree — presentation layer over aes67_config
 * (device identity), aes67_conn (stream tables, discovery registry)
 * and sap_sdp (announcement transport).
 */

#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/net/socket.h>
#include <string.h>
#include <stdlib.h>

#include "aes67_config.h"
#include "aes67_conn.h"
#include "sap_sdp.h"

static int cmd_aes67_status(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	struct aes67_device_config *cfg = aes67_config_get();
	char node_id[AES67_NODE_ID_MAX];
	char addr_str[INET_ADDRSTRLEN];

	aes67_config_build_node_id(node_id, sizeof(node_id));

	shell_print(sh, "=== Device Identity ===");
	shell_print(sh, "Vendor:       %s", cfg->vendor);
	shell_print(sh, "Product:      %s", cfg->product);
	shell_print(sh, "Serial:       %s", cfg->serial);
	shell_print(sh, "Node ID:      %s", node_id);
	shell_print(sh, "Device Name:  %s", cfg->device_name);
	shell_print(sh, "SAP announce: %s (every %us)",
		    cfg->sap_announce_enabled ? "ON" : "OFF",
		    cfg->sap_announce_interval_s);

	/* TX streams */
	shell_print(sh, "\n=== TX Streams ===");
	int tx_count = 0;
	struct aes67_tx_stream tx;

	for (uint8_t i = 0; i < AES67_MAX_TX_STREAMS; i++) {
		if (!aes67_conn_copy_tx_stream(i, &tx)) {
			continue;
		}
		zsock_inet_ntop(AF_INET, &tx.dst_ip,
				addr_str, sizeof(addr_str));
		shell_fprintf(sh, SHELL_NORMAL,
			      "  [%u] \"%s\" dst=%s ch=%u spp=%u ids=[",
			      tx.stream_id, tx.name, addr_str,
			      tx.channel_count, tx.samples_per_packet);
		for (int j = 0; j < tx.channel_count; j++) {
			shell_fprintf(sh, SHELL_NORMAL, "%s%u",
				      j ? "," : "", tx.ch_ids[j]);
		}
		shell_print(sh, "]");
		tx_count++;
	}
	if (tx_count == 0) {
		shell_print(sh, "  (none)");
	}

	/* RX streams */
	shell_print(sh, "\n=== RX Streams ===");
	const struct aes67_rx_stream *rxs = aes67_conn_get_rx_streams();
	int rx_count = 0;

	for (int i = 0; i < AES67_MAX_RX_STREAMS; i++) {
		if (!rxs[i].active) {
			continue;
		}
		zsock_inet_ntop(AF_INET, &rxs[i].dst_ip,
				addr_str, sizeof(addr_str));
		shell_print(sh, "  [%u] dst=%s:%u ch=%u spc=%u delay=%u",
			    rxs[i].stream_id, addr_str, rxs[i].dst_port,
			    rxs[i].channel_count, rxs[i].samples_per_channel,
			    rxs[i].output_delay);
		rx_count++;
	}
	if (rx_count == 0) {
		shell_print(sh, "  (none)");
	}

	/* Discovered streams */
	shell_print(sh, "\n=== Discovered Streams ===");
	int count = 0;
	const struct aes67_foreign_stream *foreign =
		aes67_conn_get_foreign_streams(NULL);

	for (int i = 0; i < AES67_MAX_FOREIGN_STREAMS; i++) {
		if (!foreign[i].valid) {
			continue;
		}
		zsock_inet_ntop(AF_INET, &foreign[i].mcast_addr,
				addr_str, sizeof(addr_str));
		shell_print(sh, "  [%d] %s @ %s:%u  %uch %ubit %uHz",
			    count, foreign[i].name,
			    addr_str, foreign[i].port,
			    foreign[i].channels,
			    foreign[i].bit_depth,
			    foreign[i].sample_rate);
		count++;
	}
	if (count == 0) {
		shell_print(sh, "  (none)");
	}

	return 0;
}

static int cmd_aes67_announce(const struct shell *sh, size_t argc, char **argv)
{
	if (argc < 2) {
		shell_print(sh, "SAP announce: %s",
			    aes67_config_get()->sap_announce_enabled ?
			    "ON" : "OFF");
		return 0;
	}

	if (strcmp(argv[1], "on") == 0) {
		sap_sdp_set_announce(true);
		shell_print(sh, "SAP announce enabled");
	} else if (strcmp(argv[1], "off") == 0) {
		sap_sdp_set_announce(false);
		shell_print(sh, "SAP announce disabled");
	} else {
		shell_error(sh, "Usage: aes67 announce [on|off]");
		return -EINVAL;
	}

	return 0;
}

static int cmd_aes67_txstream(const struct shell *sh, size_t argc, char **argv)
{
	/* aes67 txstream <stream_id> <dst_ip> <ch_count> <samples_per_pkt> <ch0> [ch1] ... */
	if (argc < 6) {
		shell_error(sh, "Usage: aes67 txstream <id 0-7> <dst_ip> "
			    "<ch_count 1-8> <samples_per_pkt> <ch0> [ch1..ch7]");
		return -EINVAL;
	}

	unsigned long id = strtoul(argv[1], NULL, 10);

	if (id >= AES67_MAX_TX_STREAMS) {
		shell_error(sh, "stream_id must be 0..%d",
			    AES67_MAX_TX_STREAMS - 1);
		return -EINVAL;
	}

	struct in_addr dst;

	if (zsock_inet_pton(AF_INET, argv[2], &dst) != 1) {
		shell_error(sh, "Invalid IP: %s", argv[2]);
		return -EINVAL;
	}

	unsigned long ch_count = strtoul(argv[3], NULL, 10);

	if (ch_count < 1 || ch_count > AES67_MAX_CH_PER_STREAM) {
		shell_error(sh, "ch_count must be 1..%d",
			    AES67_MAX_CH_PER_STREAM);
		return -EINVAL;
	}

	unsigned long spp = strtoul(argv[4], NULL, 10);

	if (spp < 1 || spp > 255) {
		shell_error(sh, "samples_per_pkt must be 1..255");
		return -EINVAL;
	}

	/* Remaining args are channel IDs */
	int num_ch_args = argc - 5;

	if (num_ch_args < (int)ch_count) {
		shell_error(sh, "Expected %lu channel IDs, got %d",
			    ch_count, num_ch_args);
		return -EINVAL;
	}

	uint8_t ch_ids[AES67_MAX_CH_PER_STREAM] = {0};

	for (int i = 0; i < (int)ch_count; i++) {
		unsigned long cid = strtoul(argv[5 + i], NULL, 10);

		if (cid > 255) {
			shell_error(sh, "Channel ID must be 0..255");
			return -EINVAL;
		}
		ch_ids[i] = (uint8_t)cid;
	}

	int ret = aes67_conn_configure_tx_stream((uint8_t)id, &dst,
						 (uint8_t)ch_count,
						 (uint8_t)spp,
						 ch_ids, (uint8_t)ch_count,
						 0, NULL); /* SSRC + name: auto */
	if (ret < 0) {
		shell_error(sh, "Failed to configure stream: %d", ret);
		return ret;
	}

	char addr_str[INET_ADDRSTRLEN];

	zsock_inet_ntop(AF_INET, &dst, addr_str, sizeof(addr_str));
	shell_print(sh, "TX stream %lu: dst=%s ch=%lu spp=%lu",
		    id, addr_str, ch_count, spp);

	return 0;
}

/* ---- Device identity commands ---- */

/* Common helper: get or set a bounded string field of the device config. */
static int identity_get_set(const struct shell *sh, size_t argc, char **argv,
			    const char *label, char *field, size_t field_size)
{
	if (argc < 2) {
		shell_print(sh, "%s: %s", label, field);
		return 0;
	}

	aes67_config_lock();
	strncpy(field, argv[1], field_size - 1);
	field[field_size - 1] = '\0';
	aes67_config_unlock();

	aes67_config_persist();
	shell_print(sh, "%s set to: %s", label, field);
	return 0;
}

static int cmd_aes67_vendor(const struct shell *sh, size_t argc, char **argv)
{
	struct aes67_device_config *cfg = aes67_config_get();

	return identity_get_set(sh, argc, argv, "Vendor",
				cfg->vendor, AES67_VENDOR_MAX);
}

static int cmd_aes67_product(const struct shell *sh, size_t argc, char **argv)
{
	struct aes67_device_config *cfg = aes67_config_get();

	return identity_get_set(sh, argc, argv, "Product",
				cfg->product, AES67_PRODUCT_MAX);
}

static int cmd_aes67_serial(const struct shell *sh, size_t argc, char **argv)
{
	struct aes67_device_config *cfg = aes67_config_get();

	return identity_get_set(sh, argc, argv, "Serial",
				cfg->serial, AES67_SERIAL_MAX);
}

static int cmd_aes67_name(const struct shell *sh, size_t argc, char **argv)
{
	struct aes67_device_config *cfg = aes67_config_get();

	return identity_get_set(sh, argc, argv, "Device name",
				cfg->device_name, AES67_DEVICE_NAME_MAX);
}

static int cmd_aes67_nodeid(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	char node_id[AES67_NODE_ID_MAX];
	char hostname[AES67_NODE_ID_MAX];

	aes67_config_build_node_id(node_id, sizeof(node_id));
	aes67_config_build_hostname(hostname, sizeof(hostname));

	shell_print(sh, "Node ID:  %s", node_id);
	shell_print(sh, "Hostname: %s", hostname);
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(aes67_cmds,
	SHELL_CMD(status, NULL, "Show AES67 stream config and discovered streams",
		  cmd_aes67_status),
	SHELL_CMD(vendor, NULL, "Get/set vendor: aes67 vendor [name]",
		  cmd_aes67_vendor),
	SHELL_CMD(product, NULL, "Get/set product: aes67 product [name]",
		  cmd_aes67_product),
	SHELL_CMD(serial, NULL, "Get/set serial: aes67 serial [number]",
		  cmd_aes67_serial),
	SHELL_CMD(name, NULL, "Get/set device name: aes67 name [friendly_name]",
		  cmd_aes67_name),
	SHELL_CMD(nodeid, NULL, "Show RAVENNA node ID and hostname",
		  cmd_aes67_nodeid),
	SHELL_CMD(announce, NULL, "Enable/disable SAP: aes67 announce [on|off]",
		  cmd_aes67_announce),
	SHELL_CMD(txstream, NULL,
		  "Configure TX stream: aes67 txstream <id> <ip> <ch_count> <spp> <ch0> [ch1..7]",
		  cmd_aes67_txstream),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(aes67, &aes67_cmds, "AES67 stream configuration", NULL);
