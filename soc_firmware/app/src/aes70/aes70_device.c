/*
 * AES70 device model: the object tree of this AES67 node.
 *
 * Object numbers (fixed, so controllers can bookmark them):
 *
 *   1      OcaDeviceManager          4     OcaSubscriptionManager
 *   3      OcaFirmwareManager        7     OcaMediaClockManager
 *   100    Root block
 *   1000   "Inputs"   block         1100+10n  "In n+1" block
 *                                     +1 OcaGain "Gain" (preamp dB)
 *                                     +2 OcaMute "Mute"
 *                                     +3 OcaBooleanActuator "Phantom"
 *   2000   "Outputs"  block         2100+10n  "Out n+1" block
 *                                     +1 OcaGain "Level" (clip level dB)
 *                                     +2 OcaMute "Mute"
 *   3000   "Streams"  block
 *   3100   "TX" block               3110+10s  "TX s+1" block
 *                                     +1 OcaBooleanActuator "Active"
 *                                     +2 OcaStringActuator  "Name"
 *                                     +3 OcaStringActuator  "Destination" (multicast IP)
 *                                     +4 OcaStringActuator  "Channels" ("0,1,...")
 *                                     +5 OcaUint16Actuator  "SamplesPerPacket"
 *   3200   "RX" block               3210+10s  "RX s+1" block
 *                                     +1 Active, +2 Name, +3 "Address" ("ip:port"),
 *                                     +4 "ChannelMap", +5 "OutputDelay",
 *                                     +6 "SamplesPerChannel"
 *   4000   "Clock" block
 *   4001   OcaTimeSource "PTP"      4002  OcaMediaClock3 "MediaClock"
 *   4010   OcaStringSensor "Role"   4011  OcaInt32Sensor "OffsetNs"
 *   4012   OcaInt32Sensor "PathDelayNs"  4013 OcaBooleanSensor "Locked"
 *   4014   OcaStringSensor "Grandmaster"
 *   4020   OcaUint8Actuator "Domain"  4021 "Priority1"  4022 "Priority2"
 *   5000   "Network" block
 *   5001   OcaStringSensor "IPAddress"  5002 "MACAddress"  5003 "Hostname"
 *   5010   OcaBooleanActuator "SAPAnnounce"
 *   6000   "Auracast" block (Auracast card only)
 *   6001   OcaBooleanActuator "Enabled"       6002 OcaStringActuator "Name"
 *   6003   OcaStringActuator "Code"           6004 OcaUint8Actuator "Preset"
 *   6005   OcaUint8Actuator "Channels"        6006 OcaUint16Actuator "Context"
 *   6007   OcaBooleanActuator "ImmediateRendering"
 *   6008   OcaInt32Actuator "TxPowerDbm"      6009 OcaUint16Actuator "PresentationDelayUs"
 *   6010   OcaUint16Actuator "FifoTarget"     6011 OcaStringActuator "BroadcastID" ("random" | hex)
 *   6012   OcaUint8Actuator "HQPreset" (255 = none)  6013 "HQChannels"  6014 OcaUint16Actuator "HQContext"
 *   6020   OcaStringSensor "State"  6021 "Error"  6022 OcaBooleanSensor "Streaming"
 *   6023   OcaStringSensor "ActiveBroadcastID"  6024 OcaInt32Sensor "FifoFill"  6025 "DriftCppm"
 *
 * The Inputs/Outputs blocks exist only when a card with the respective
 * converters was detected at start.
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/net_ip.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/hostname.h>
#include <zephyr/sys/util.h>
#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

#include "aes70.h"
#include "aes70_device.h"
#include "aes70_model.h"
#include "../aes67_config.h"
#include "../aes67_conn.h"
#include "../ptp_ctrl.h"
#include "../card_manager.h"
#include "../card_settings.h"
#include "../sap_sdp.h"
#ifdef CONFIG_MDNS_SD
#include "../mdns_sd.h"
#endif
#ifdef CONFIG_MI_CARD
#include "../../drivers/mi_card/mi_card.h"
#endif
#ifdef CONFIG_LO_CARD
#include "../../drivers/lo_card/lo_card.h"
#endif
#ifdef CONFIG_IO_CARD
#include "../../drivers/io_card/io_card.h"
#endif
#ifdef CONFIG_AURACAST_CARD
#include "../../drivers/auracast_card/auracast_card.h"
#endif

LOG_MODULE_REGISTER(aes70_dev, LOG_LEVEL_INF);

/* ================================================================
 * Class definitions
 * ================================================================ */

static const uint16_t cid_devmgr[]   = {1, 3, 1};
static const uint16_t cid_fwmgr[]    = {1, 3, 3};
static const uint16_t cid_mclkmgr[]  = {1, 3, 7};
static const uint16_t cid_gain[]     = {1, 1, 1, 5};
static const uint16_t cid_mute[]     = {1, 1, 1, 2};
static const uint16_t cid_bool_act[] = {1, 1, 1, 1, 1};
static const uint16_t cid_u8_act[]   = {1, 1, 1, 1, 6};
static const uint16_t cid_u16_act[]  = {1, 1, 1, 1, 7};
static const uint16_t cid_i32_act[]  = {1, 1, 1, 1, 4};
static const uint16_t cid_str_act[]  = {1, 1, 1, 1, 12};
static const uint16_t cid_bool_sen[] = {1, 1, 2, 1, 1};
static const uint16_t cid_i32_sen[]  = {1, 1, 2, 1, 4};
static const uint16_t cid_str_sen[]  = {1, 1, 2, 1, 12};
static const uint16_t cid_timesrc[]  = {1, 2, 16};
static const uint16_t cid_mclk3[]    = {1, 2, 15};

static int devmgr_custom(struct oca_object *, struct oca_session *, uint16_t,
			 uint16_t, uint8_t, struct ocp1_reader *,
			 struct ocp1_writer *, uint8_t *);
static int fwmgr_custom(struct oca_object *, struct oca_session *, uint16_t,
			uint16_t, uint8_t, struct ocp1_reader *,
			struct ocp1_writer *, uint8_t *);
static int mclkmgr_custom(struct oca_object *, struct oca_session *, uint16_t,
			  uint16_t, uint8_t, struct ocp1_reader *,
			  struct ocp1_writer *, uint8_t *);
static int sensor_custom(struct oca_object *, struct oca_session *, uint16_t,
			 uint16_t, uint8_t, struct ocp1_reader *,
			 struct ocp1_writer *, uint8_t *);
static int string_custom(struct oca_object *, struct oca_session *, uint16_t,
			 uint16_t, uint8_t, struct ocp1_reader *,
			 struct ocp1_writer *, uint8_t *);
static int timesrc_custom(struct oca_object *, struct oca_session *, uint16_t,
			  uint16_t, uint8_t, struct ocp1_reader *,
			  struct ocp1_writer *, uint8_t *);
static int mclk3_custom(struct oca_object *, struct oca_session *, uint16_t,
			uint16_t, uint8_t, struct ocp1_reader *,
			struct ocp1_writer *, uint8_t *);

/* OcaDeviceManager: only DeviceName is tracked for PropertyChanged; the
 * methods themselves are all custom. */
static const struct oca_prop props_devmgr[] = {
	{ .level = 3, .index = 4, .vt = OCA_VT_STR, .flags = OCA_PF_NOTIFY },
};
static const struct oca_kind kind_devmgr = {
	.name = "OcaDeviceManager", .class_id = cid_devmgr, .class_id_len = 3,
	.class_version = 3, .base = OCA_BASE_MANAGER,
	.props = props_devmgr, .nprops = ARRAY_SIZE(props_devmgr),
	.custom = devmgr_custom,
};
static const struct oca_kind kind_fwmgr = {
	.name = "OcaFirmwareManager", .class_id = cid_fwmgr, .class_id_len = 3,
	.class_version = 3, .base = OCA_BASE_MANAGER, .custom = fwmgr_custom,
};
static const struct oca_kind kind_mclkmgr = {
	.name = "OcaMediaClockManager", .class_id = cid_mclkmgr,
	.class_id_len = 3, .class_version = 3, .base = OCA_BASE_MANAGER,
	.custom = mclkmgr_custom,
};

static const struct oca_prop props_gain[] = {
	{ 4, 1, OCA_VT_F32, 1, 2, OCA_PF_RANGE | OCA_PF_NOTIFY },
};
static const struct oca_kind kind_gain = {
	.name = "OcaGain", .class_id = cid_gain, .class_id_len = 4,
	.class_version = 3, .base = OCA_BASE_WORKER,
	.props = props_gain, .nprops = 1,
};

static const struct oca_prop props_mute[] = {
	{ 4, 1, OCA_VT_U8, 1, 2, OCA_PF_NOTIFY },
};
static const struct oca_kind kind_mute = {
	.name = "OcaMute", .class_id = cid_mute, .class_id_len = 4,
	.class_version = 3, .base = OCA_BASE_WORKER,
	.props = props_mute, .nprops = 1,
};

static const struct oca_prop props_bool_act[] = {
	{ 5, 1, OCA_VT_BOOL, 1, 2, OCA_PF_NOTIFY },
};
static const struct oca_kind kind_bool_act = {
	.name = "OcaBooleanActuator", .class_id = cid_bool_act,
	.class_id_len = 5, .class_version = 3, .base = OCA_BASE_WORKER,
	.props = props_bool_act, .nprops = 1,
};

static const struct oca_prop props_u8_act[] = {
	{ 5, 1, OCA_VT_U8, 1, 2, OCA_PF_RANGE | OCA_PF_NOTIFY },
};
static const struct oca_kind kind_u8_act = {
	.name = "OcaUint8Actuator", .class_id = cid_u8_act, .class_id_len = 5,
	.class_version = 3, .base = OCA_BASE_WORKER,
	.props = props_u8_act, .nprops = 1,
};

static const struct oca_prop props_u16_act[] = {
	{ 5, 1, OCA_VT_U16, 1, 2, OCA_PF_RANGE | OCA_PF_NOTIFY },
};
static const struct oca_kind kind_u16_act = {
	.name = "OcaUint16Actuator", .class_id = cid_u16_act, .class_id_len = 5,
	.class_version = 3, .base = OCA_BASE_WORKER,
	.props = props_u16_act, .nprops = 1,
};

static const struct oca_prop props_i32_act[] = {
	{ 5, 1, OCA_VT_I32, 1, 2, OCA_PF_RANGE | OCA_PF_NOTIFY },
};
static const struct oca_kind kind_i32_act = {
	.name = "OcaInt32Actuator", .class_id = cid_i32_act, .class_id_len = 5,
	.class_version = 3, .base = OCA_BASE_WORKER,
	.props = props_i32_act, .nprops = 1,
};

static const struct oca_prop props_str_act[] = {
	{ 5, 1, OCA_VT_STR, 1, 2, OCA_PF_NOTIFY },
};
static const struct oca_kind kind_str_act = {
	.name = "OcaStringActuator", .class_id = cid_str_act, .class_id_len = 5,
	.class_version = 3, .base = OCA_BASE_WORKER,
	.props = props_str_act, .nprops = 1, .custom = string_custom,
};

static const struct oca_prop props_bool_sen[] = {
	{ 5, 1, OCA_VT_BOOL, 1, 0, OCA_PF_NOTIFY },
};
static const struct oca_kind kind_bool_sen = {
	.name = "OcaBooleanSensor", .class_id = cid_bool_sen, .class_id_len = 5,
	.class_version = 3, .base = OCA_BASE_WORKER,
	.props = props_bool_sen, .nprops = 1, .custom = sensor_custom,
};

static const struct oca_prop props_i32_sen[] = {
	{ 5, 1, OCA_VT_I32, 1, 0, OCA_PF_RANGE | OCA_PF_NOTIFY },
};
static const struct oca_kind kind_i32_sen = {
	.name = "OcaInt32Sensor", .class_id = cid_i32_sen, .class_id_len = 5,
	.class_version = 3, .base = OCA_BASE_WORKER,
	.props = props_i32_sen, .nprops = 1, .custom = sensor_custom,
};

static const struct oca_prop props_str_sen[] = {
	{ 5, 1, OCA_VT_STR, 1, 0, OCA_PF_NOTIFY },
};
static const struct oca_kind kind_str_sen = {
	.name = "OcaStringSensor", .class_id = cid_str_sen, .class_id_len = 5,
	.class_version = 3, .base = OCA_BASE_WORKER,
	.props = props_str_sen, .nprops = 1, .custom = string_custom,
};

/* OcaTimeSource: prop table order = getter prop index. */
enum { TS_AVAIL = 0, TS_SYNC, TS_MECH, TS_SDP, TS_REFID, TS_PARAMS };
static const struct oca_prop props_timesrc[] = {
	{ 3, 1, OCA_VT_U8,  1,  0, OCA_PF_NOTIFY },   /* Availability */
	{ 3, 6, OCA_VT_U8,  10, 0, OCA_PF_NOTIFY },   /* SyncStatus */
	{ 3, 2, OCA_VT_U8,  2,  0, 0 },               /* TimeDeliveryMechanism */
	{ 3, 3, OCA_VT_STR, 4,  0, 0 },               /* ReferenceSDPDescription */
	{ 3, 5, OCA_VT_STR, 8,  0, 0 },               /* ReferenceID */
	{ 3, 7, OCA_VT_STR, 12, 0, 0 },               /* TimeDeliveryParameters */
};
static const struct oca_kind kind_timesrc = {
	.name = "OcaTimeSource", .class_id = cid_timesrc, .class_id_len = 3,
	.class_version = 3, .base = OCA_BASE_AGENT,
	.props = props_timesrc, .nprops = ARRAY_SIZE(props_timesrc),
	.custom = timesrc_custom,
};

static const struct oca_prop props_mclk3[] = {
	{ 3, 1, OCA_VT_U8, 1, 0, OCA_PF_NOTIFY },     /* Availability */
};
static const struct oca_kind kind_mclk3 = {
	.name = "OcaMediaClock3", .class_id = cid_mclk3, .class_id_len = 3,
	.class_version = 3, .base = OCA_BASE_AGENT,
	.props = props_mclk3, .nprops = 1, .custom = mclk3_custom,
};

/* ================================================================
 * Shared helpers
 * ================================================================ */

/* ONo layout */
#define ONO_INPUTS       1000
#define ONO_IN_BASE      1100
#define ONO_OUTPUTS      2000
#define ONO_OUT_BASE     2100
#define ONO_STREAMS      3000
#define ONO_TX           3100
#define ONO_TX_BASE      3110
#define ONO_RX           3200
#define ONO_RX_BASE      3210
#define ONO_CLOCK        4000
#define ONO_TIMESRC      4001
#define ONO_MCLK         4002
#define ONO_NETWORK      5000
#define ONO_AURACAST     6000

static struct in_addr dev_ip;
static struct net_if *dev_iface;

/* RAM-only OcaDeviceManager strings */
static char dev_role[OCA_STR_MAX];
static char dev_inventory[OCA_STR_MAX];
static char dev_message[OCA_STR_MAX];

static int sensor_custom(struct oca_object *obj, struct oca_session *s,
			 uint16_t level, uint16_t index, uint8_t in_count,
			 struct ocp1_reader *in, struct ocp1_writer *out,
			 uint8_t *out_count)
{
	ARG_UNUSED(obj); ARG_UNUSED(s); ARG_UNUSED(in_count); ARG_UNUSED(in);
	if (level == 3 && index == 1) {   /* OcaSensor.GetReadingState */
		ocp1_w_u8(out, 1);            /* Valid */
		*out_count = 1;
		return OCA_STATUS_OK;
	}
	return OCA_METHOD_UNHANDLED;
}

/* OcaStringActuator.GetMaxLen (5.3) / OcaStringSensor.GetMaxLen (5.2)
 * + ReadingState for the sensor. */
static int string_custom(struct oca_object *obj, struct oca_session *s,
			 uint16_t level, uint16_t index, uint8_t in_count,
			 struct ocp1_reader *in, struct ocp1_writer *out,
			 uint8_t *out_count)
{
	bool sensor = obj->kind == &kind_str_sen;

	if (sensor && level == 3 && index == 1) {
		return sensor_custom(obj, s, level, index, in_count, in, out,
				     out_count);
	}
	if (level == 5 && index == (sensor ? 2 : 3)) {
		ocp1_w_u16(out, OCA_STR_MAX - 1);
		*out_count = 1;
		return OCA_STATUS_OK;
	}
	if (sensor && level == 5 && index == 3) {   /* SetMaxLen */
		return OCA_STATUS_NOT_IMPLEMENTED;
	}
	return OCA_METHOD_UNHANDLED;
}

static void set_str(struct oca_value *v, const char *s)
{
	v->vt = OCA_VT_STR;
	strncpy(v->str, s ? s : "", sizeof(v->str) - 1);
	v->str[sizeof(v->str) - 1] = '\0';
}

static void copy_str(char *dst, size_t cap, const char *src)
{
	strncpy(dst, src, cap - 1);
	dst[cap - 1] = '\0';
}

/* ================================================================
 * Analog card channels
 * ================================================================ */

static card_type_t card_type(void)
{
	const struct card_info *ci = card_manager_get_info(CARD_SLOT_MAIN);

	return (ci != NULL && ci->present) ? ci->type : CARD_TYPE_NONE;
}

static int in_get(const struct oca_object *obj, uint8_t prop,
		  struct oca_value *v)
{
	uint8_t ch = (uint8_t)obj->arg;
	int r;

	ARG_UNUSED(prop);
	ARG_UNUSED(ch);
	switch (card_type()) {
#ifdef CONFIG_MI_CARD
	case CARD_TYPE_MI:
		if (obj->kind == &kind_gain) {
			r = mi_card_get_gain(ch);
			v->min = MI_GAIN_MIN;
			v->max = MI_GAIN_MAX;
		} else if (obj->kind == &kind_mute) {
			r = mi_card_get_mute(ch);
		} else {
			r = mi_card_get_phantom(ch);
		}
		break;
#endif
#ifdef CONFIG_IO_CARD
	case CARD_TYPE_IO:
		if (obj->kind == &kind_gain) {
			r = io_card_get_in_gain(ch);
			v->min = IO_IN_GAIN_MIN;
			v->max = IO_IN_GAIN_MAX;
		} else if (obj->kind == &kind_mute) {
			r = io_card_get_in_mute(ch);
		} else {
			r = io_card_get_in_phantom(ch);
		}
		break;
#endif
	default:
		return -ENODEV;
	}
	if (obj->kind == &kind_gain) {
		/* Gains are legitimately negative; anything below the
		 * range is a driver errno. */
		if ((float)r < v->min - 0.5f) {
			return -ENODEV;
		}
		v->f32 = (float)r;
	} else if (r < 0) {
		return r;
	} else if (obj->kind == &kind_mute) {
		v->u8 = r ? 1 : 2;   /* OcaMuteState: Muted=1, Unmuted=2 */
	} else {
		v->b = r > 0;
	}
	return 0;
}

static int in_set(const struct oca_object *obj, uint8_t prop,
		  const struct oca_value *v)
{
	uint8_t ch = (uint8_t)obj->arg;
	int r;

	ARG_UNUSED(prop);
	ARG_UNUSED(ch);
	if (obj->kind == &kind_mute && v->u8 != 1 && v->u8 != 2) {
		return -EINVAL;
	}
	if (obj->kind == &kind_gain && !(v->f32 >= -200.0f && v->f32 <= 200.0f)) {
		return -ERANGE;
	}
	switch (card_type()) {
#ifdef CONFIG_MI_CARD
	case CARD_TYPE_MI:
		if (obj->kind == &kind_gain) {
			int g = (int)lroundf(v->f32);

			if (g < MI_GAIN_MIN || g > MI_GAIN_MAX) {
				return -ERANGE;
			}
			r = mi_card_set_gain(ch, (int8_t)g);
		} else if (obj->kind == &kind_mute) {
			r = mi_card_set_mute(ch, v->u8 == 1);
		} else {
			r = mi_card_set_phantom(ch, v->b);
		}
		break;
#endif
#ifdef CONFIG_IO_CARD
	case CARD_TYPE_IO:
		if (obj->kind == &kind_gain) {
			int g = (int)lroundf(v->f32);

			if (g < IO_IN_GAIN_MIN || g > IO_IN_GAIN_MAX) {
				return -ERANGE;
			}
			r = io_card_set_in_gain(ch, (int8_t)g);
		} else if (obj->kind == &kind_mute) {
			r = io_card_set_in_mute(ch, v->u8 == 1);
		} else {
			r = io_card_set_in_phantom(ch, v->b);
		}
		break;
#endif
	default:
		return -ENODEV;
	}
	if (r < 0) {
		return r;
	}
	card_settings_schedule_save();
	return 0;
}

static int out_get(const struct oca_object *obj, uint8_t prop,
		   struct oca_value *v)
{
	uint8_t ch = (uint8_t)obj->arg;
	int r;

	ARG_UNUSED(prop);
	ARG_UNUSED(ch);
	switch (card_type()) {
#ifdef CONFIG_LO_CARD
	case CARD_TYPE_LO:
		if (obj->kind == &kind_gain) {
			r = lo_card_get_clip(ch);
			v->min = LO_CLIP_MIN;
			v->max = LO_CLIP_MAX;
		} else {
			r = lo_card_get_mute(ch);
		}
		break;
#endif
#ifdef CONFIG_IO_CARD
	case CARD_TYPE_IO:
		if (obj->kind == &kind_gain) {
			r = io_card_get_out_clip(ch);
			v->min = IO_OUT_CLIP_MIN;
			v->max = IO_OUT_CLIP_MAX;
		} else {
			r = io_card_get_out_mute(ch);
		}
		break;
#endif
	default:
		return -ENODEV;
	}
	if (obj->kind == &kind_gain) {
		if ((float)r < v->min - 0.5f) {
			return -ENODEV;
		}
		v->f32 = (float)r;
	} else if (r < 0) {
		return r;
	} else {
		v->u8 = r ? 1 : 2;
	}
	return 0;
}

static int out_set(const struct oca_object *obj, uint8_t prop,
		   const struct oca_value *v)
{
	uint8_t ch = (uint8_t)obj->arg;
	int r;

	ARG_UNUSED(prop);
	ARG_UNUSED(ch);
	if (obj->kind == &kind_mute && v->u8 != 1 && v->u8 != 2) {
		return -EINVAL;
	}
	if (obj->kind == &kind_gain && !(v->f32 >= -200.0f && v->f32 <= 200.0f)) {
		return -ERANGE;
	}
	switch (card_type()) {
#ifdef CONFIG_LO_CARD
	case CARD_TYPE_LO:
		if (obj->kind == &kind_gain) {
			int g = (int)lroundf(v->f32);

			if (g < LO_CLIP_MIN || g > LO_CLIP_MAX) {
				return -ERANGE;
			}
			r = lo_card_set_clip(ch, (int8_t)g);
		} else {
			r = lo_card_set_mute(ch, v->u8 == 1);
		}
		break;
#endif
#ifdef CONFIG_IO_CARD
	case CARD_TYPE_IO:
		if (obj->kind == &kind_gain) {
			int g = (int)lroundf(v->f32);

			if (g < IO_OUT_CLIP_MIN || g > IO_OUT_CLIP_MAX) {
				return -ERANGE;
			}
			r = io_card_set_out_clip(ch, (int8_t)g);
		} else {
			r = io_card_set_out_mute(ch, v->u8 == 1);
		}
		break;
#endif
	default:
		return -ENODEV;
	}
	if (r < 0) {
		return r;
	}
	card_settings_schedule_save();
	return 0;
}

/* ================================================================
 * Streams
 * ================================================================ */

enum { TXF_ACTIVE = 1, TXF_NAME, TXF_DEST, TXF_CHANNELS, TXF_SPP };
enum { RXF_ACTIVE = 1, RXF_NAME, RXF_ADDR, RXF_CHMAP, RXF_DELAY, RXF_SPC };

/* Pending edits for inactive slots: a controller sets the fields one by
 * one and finally flips Active. Active slots are read from / written
 * through the live tables. */
static struct aes67_tx_stream tx_shadow[AES67_MAX_TX_STREAMS];
static struct aes67_rx_stream rx_shadow[AES67_MAX_RX_STREAMS];

static void fmt_list(char *out, size_t cap, const uint8_t *ids, uint8_t n)
{
	size_t pos = 0;

	out[0] = '\0';
	for (uint8_t i = 0; i < n && i < AES67_MAX_CH_PER_STREAM; i++) {
		int w = snprintf(out + pos, cap - pos, "%s%u", i ? "," : "", ids[i]);

		if (w < 0 || (size_t)w >= cap - pos) {
			break;
		}
		pos += (size_t)w;
	}
}

/* "0,1,2" -> ids; returns count or -EINVAL. */
static int parse_list(const char *s, uint8_t *ids, uint8_t max)
{
	int n = 0;

	while (*s != '\0') {
		char *end;
		long v = strtol(s, &end, 10);

		if (end == s || v < 0 || v > 255 || n >= max) {
			return -EINVAL;
		}
		ids[n++] = (uint8_t)v;
		s = end;
		while (*s == ' ') {
			s++;
		}
		if (*s == ',') {
			s++;
		} else if (*s != '\0') {
			return -EINVAL;
		}
	}
	return n;
}

static void tx_current(uint8_t slot, struct aes67_tx_stream *out)
{
	const struct aes67_tx_stream *live = &aes67_conn_get_tx_streams()[slot];

	if (live->active) {
		*out = *live;
	} else {
		*out = tx_shadow[slot];
	}
}

static int tx_get(const struct oca_object *obj, uint8_t prop,
		  struct oca_value *v)
{
	uint8_t slot = (uint8_t)(obj->arg >> 8);
	uint8_t field = (uint8_t)(obj->arg & 0xFF);
	struct aes67_tx_stream st;

	ARG_UNUSED(prop);
	tx_current(slot, &st);
	switch (field) {
	case TXF_ACTIVE:
		v->b = aes67_conn_get_tx_streams()[slot].active;
		break;
	case TXF_NAME:
		set_str(v, st.name);
		break;
	case TXF_DEST:
		net_addr_ntop(AF_INET, &st.dst_ip, v->str, sizeof(v->str));
		break;
	case TXF_CHANNELS:
		fmt_list(v->str, sizeof(v->str), st.ch_ids, st.channel_count);
		break;
	case TXF_SPP:
		v->u16 = st.samples_per_packet;
		v->min = 1;
		v->max = 255;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int tx_apply(uint8_t slot, const struct aes67_tx_stream *st, bool active)
{
	struct in_addr zero = { .s_addr = 0 };

	tx_shadow[slot] = *st;
	tx_shadow[slot].active = false;
	if (!active) {
		/* Writing the zero config deactivates the slot in hardware. */
		return aes67_conn_configure_tx_stream(slot, &zero, 0, 0, NULL, 0,
						      0, NULL);
	}
	if (st->dst_ip.s_addr == 0 || st->channel_count == 0) {
		return -EINVAL;
	}
	return aes67_conn_configure_tx_stream(slot, &st->dst_ip,
					      st->channel_count,
					      st->samples_per_packet,
					      st->ch_ids, st->channel_count,
					      st->ssrc,
					      st->name[0] ? st->name : NULL);
}

static int tx_set(const struct oca_object *obj, uint8_t prop,
		  const struct oca_value *v)
{
	uint8_t slot = (uint8_t)(obj->arg >> 8);
	uint8_t field = (uint8_t)(obj->arg & 0xFF);
	struct aes67_tx_stream st;
	bool active;
	int n;

	ARG_UNUSED(prop);
	tx_current(slot, &st);
	active = aes67_conn_get_tx_streams()[slot].active;
	switch (field) {
	case TXF_ACTIVE:
		return tx_apply(slot, &st, v->b);
	case TXF_NAME:
		copy_str(st.name, sizeof(st.name), v->str);
		break;
	case TXF_DEST:
		if (net_addr_pton(AF_INET, v->str, &st.dst_ip) < 0) {
			return -EINVAL;
		}
		break;
	case TXF_CHANNELS:
		n = parse_list(v->str, st.ch_ids, AES67_MAX_CH_PER_STREAM);
		if (n < 0) {
			return n;
		}
		st.channel_count = (uint8_t)n;
		break;
	case TXF_SPP:
		if (v->u16 < 1 || v->u16 > 255) {
			return -ERANGE;
		}
		st.samples_per_packet = (uint8_t)v->u16;
		break;
	default:
		return -EINVAL;
	}
	/* A live stream is reconfigured on the spot; an idle one just
	 * remembers the edit until Active is set. */
	return tx_apply(slot, &st, active);
}

static void rx_current(uint8_t slot, struct aes67_rx_stream *out)
{
	const struct aes67_rx_stream *live = &aes67_conn_get_rx_streams()[slot];

	if (live->active) {
		*out = *live;
	} else {
		*out = rx_shadow[slot];
	}
}

static int rx_get(const struct oca_object *obj, uint8_t prop,
		  struct oca_value *v)
{
	uint8_t slot = (uint8_t)(obj->arg >> 8);
	uint8_t field = (uint8_t)(obj->arg & 0xFF);
	struct aes67_rx_stream st;
	char ip[NET_IPV4_ADDR_LEN];

	ARG_UNUSED(prop);
	rx_current(slot, &st);
	switch (field) {
	case RXF_ACTIVE:
		v->b = aes67_conn_get_rx_streams()[slot].active;
		break;
	case RXF_NAME:
		set_str(v, st.name);
		break;
	case RXF_ADDR:
		net_addr_ntop(AF_INET, &st.dst_ip, ip, sizeof(ip));
		snprintf(v->str, sizeof(v->str), "%s:%u", ip, st.dst_port);
		break;
	case RXF_CHMAP:
		fmt_list(v->str, sizeof(v->str), st.ch_map, st.channel_count);
		break;
	case RXF_DELAY:
		v->u16 = st.output_delay;
		v->min = 0;
		v->max = 255;
		break;
	case RXF_SPC:
		v->u16 = st.samples_per_channel;
		v->min = 1;
		v->max = 255;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int rx_apply(uint8_t slot, const struct aes67_rx_stream *st, bool active)
{
	struct in_addr zero = { .s_addr = 0 };

	rx_shadow[slot] = *st;
	rx_shadow[slot].active = false;
	if (!active) {
		return aes67_conn_configure_rx_stream(slot, &zero, 0, NULL, 0, 0,
						      0, NULL, NULL, NULL);
	}
	if (st->dst_ip.s_addr == 0 || st->channel_count == 0) {
		return -EINVAL;
	}
	return aes67_conn_configure_rx_stream(slot, &st->dst_ip, st->dst_port,
					      st->ch_map, st->channel_count,
					      st->output_delay,
					      st->samples_per_channel,
					      st->name[0] ? st->name : NULL,
					      st->sender_name[0] ? st->sender_name : NULL,
					      st->sender_ip.s_addr ? &st->sender_ip : NULL);
}

static int rx_set(const struct oca_object *obj, uint8_t prop,
		  const struct oca_value *v)
{
	uint8_t slot = (uint8_t)(obj->arg >> 8);
	uint8_t field = (uint8_t)(obj->arg & 0xFF);
	struct aes67_rx_stream st;
	bool active;
	int n;

	ARG_UNUSED(prop);
	rx_current(slot, &st);
	active = aes67_conn_get_rx_streams()[slot].active;
	switch (field) {
	case RXF_ACTIVE:
		return rx_apply(slot, &st, v->b);
	case RXF_NAME:
		copy_str(st.name, sizeof(st.name), v->str);
		break;
	case RXF_ADDR: {
		char ip[OCA_STR_MAX];
		const char *colon = strchr(v->str, ':');
		size_t n_ip = colon ? (size_t)(colon - v->str) : strlen(v->str);
		long port = AES67_DEFAULT_PORT;

		if (n_ip >= sizeof(ip)) {
			return -EINVAL;
		}
		memcpy(ip, v->str, n_ip);
		ip[n_ip] = '\0';
		if (net_addr_pton(AF_INET, ip, &st.dst_ip) < 0) {
			return -EINVAL;
		}
		if (colon != NULL) {
			char *end;

			port = strtol(colon + 1, &end, 10);
			if (*end != '\0' || port < 1 || port > 65535) {
				return -EINVAL;
			}
		}
		st.dst_port = (uint16_t)port;
		break;
	}
	case RXF_CHMAP:
		n = parse_list(v->str, st.ch_map, AES67_MAX_CH_PER_STREAM);
		if (n < 0) {
			return n;
		}
		st.channel_count = (uint8_t)n;
		break;
	case RXF_DELAY:
		if (v->u16 > 255) {
			return -ERANGE;
		}
		st.output_delay = (uint8_t)v->u16;
		break;
	case RXF_SPC:
		if (v->u16 < 1 || v->u16 > 255) {
			return -ERANGE;
		}
		st.samples_per_channel = (uint8_t)v->u16;
		break;
	default:
		return -EINVAL;
	}
	return rx_apply(slot, &st, active);
}

static void stream_observer(uint8_t stream_id)
{
	ARG_UNUSED(stream_id);
	aes70_core_set_dirty();
}

/* ================================================================
 * Clock (PTP) block
 * ================================================================ */

enum {
	CLK_ROLE = 10, CLK_OFFSET, CLK_PATH_DELAY, CLK_LOCKED, CLK_GM,
	CLK_DOMAIN = 20, CLK_PRIO1, CLK_PRIO2,
};

static void fmt_clock_id(char *out, size_t cap, const uint8_t id[8])
{
	snprintf(out, cap, "%02x%02x%02x.%02x%02x.%02x%02x%02x",
		 id[0], id[1], id[2], id[3], id[4], id[5], id[6], id[7]);
}

static int clock_get(const struct oca_object *obj, uint8_t prop,
		     struct oca_value *v)
{
	struct ptp_ctrl_status st;
	const struct aes67_device_config *cfg = aes67_config_get();

	ARG_UNUSED(prop);
	switch (obj->arg) {
	case CLK_ROLE:
		ptp_ctrl_get_status(&st);
		set_str(v, st.role == PTP_CTRL_ROLE_LEADER ? "leader" :
			   st.role == PTP_CTRL_ROLE_FOLLOWER ? "follower" :
			   "listening");
		break;
	case CLK_OFFSET:
		ptp_ctrl_get_status(&st);
		v->i32 = st.offset_ns;
		v->min = -2.0e9f;
		v->max = 2.0e9f;
		break;
	case CLK_PATH_DELAY:
		ptp_ctrl_get_status(&st);
		v->i32 = st.path_delay_ns;
		v->min = -2.0e9f;
		v->max = 2.0e9f;
		break;
	case CLK_LOCKED:
		v->b = ptp_ctrl_wallclock_locked();
		break;
	case CLK_GM:
		ptp_ctrl_get_status(&st);
		if (st.gm_valid) {
			fmt_clock_id(v->str, sizeof(v->str), st.gm_id);
		} else if (st.role == PTP_CTRL_ROLE_LEADER) {
			fmt_clock_id(v->str, sizeof(v->str), st.clock_id);
		} else {
			v->str[0] = '\0';
		}
		break;
	case CLK_DOMAIN:
		v->u8 = cfg->ptp_domain;
		v->min = 0;
		v->max = 127;
		break;
	case CLK_PRIO1:
		v->u8 = cfg->ptp_priority1;
		v->min = 0;
		v->max = 255;
		break;
	case CLK_PRIO2:
		v->u8 = cfg->ptp_priority2;
		v->min = 0;
		v->max = 255;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int clock_set(const struct oca_object *obj, uint8_t prop,
		     const struct oca_value *v)
{
	struct aes67_device_config *cfg = aes67_config_get();

	ARG_UNUSED(prop);
	aes67_config_lock();
	switch (obj->arg) {
	case CLK_DOMAIN:
		if (v->u8 > 127) {
			aes67_config_unlock();
			return -ERANGE;
		}
		cfg->ptp_domain = v->u8;
		break;
	case CLK_PRIO1:
		cfg->ptp_priority1 = v->u8;
		break;
	case CLK_PRIO2:
		cfg->ptp_priority2 = v->u8;
		break;
	default:
		aes67_config_unlock();
		return -EINVAL;
	}
	aes67_config_unlock();
	ptp_ctrl_apply_config();
	(void)aes67_config_persist();
	return 0;
}

/* OcaTimeSource */
static int ts_get(const struct oca_object *obj, uint8_t prop,
		  struct oca_value *v)
{
	struct ptp_ctrl_status st;

	ARG_UNUSED(obj);
	ptp_ctrl_get_status(&st);
	switch (prop) {
	case TS_AVAIL:
		v->u8 = (st.role == PTP_CTRL_ROLE_LEADER || st.gm_valid) ? 1 : 0;
		break;
	case TS_SYNC:
		/* Undefined=0, Unsynchronized=1, Synchronizing=2, Synchronized=3 */
		if (st.role == PTP_CTRL_ROLE_LEADER || st.locked) {
			v->u8 = 3;
		} else if (st.gm_valid) {
			v->u8 = 2;
		} else {
			v->u8 = 1;
		}
		break;
	case TS_MECH:
		v->u8 = 6;   /* OcaTimeDeliveryMechanism.IEEE1588_2008 */
		break;
	case TS_SDP:
		v->str[0] = '\0';
		break;
	case TS_REFID:
		if (st.gm_valid) {
			fmt_clock_id(v->str, sizeof(v->str), st.gm_id);
		} else {
			fmt_clock_id(v->str, sizeof(v->str), st.clock_id);
		}
		break;
	case TS_PARAMS: {
		const struct aes67_device_config *cfg = aes67_config_get();

		snprintf(v->str, sizeof(v->str), "{\"Domain\":%u}",
			 cfg->ptp_domain);
		break;
	}
	default:
		return -EINVAL;
	}
	return 0;
}

static int timesrc_custom(struct oca_object *obj, struct oca_session *s,
			  uint16_t level, uint16_t index, uint8_t in_count,
			  struct ocp1_reader *in, struct ocp1_writer *out,
			  uint8_t *out_count)
{
	ARG_UNUSED(obj); ARG_UNUSED(s); ARG_UNUSED(in_count); ARG_UNUSED(in);
	ARG_UNUSED(out); ARG_UNUSED(out_count);
	if (level != 3) {
		return OCA_METHOD_UNHANDLED;
	}
	switch (index) {
	case 3: case 5: case 6: case 7: case 9: case 11: case 13:
		/* setters, deprecated ReferenceType, Reset */
		return OCA_STATUS_NOT_IMPLEMENTED;
	default:
		return OCA_METHOD_UNHANDLED;
	}
}

/* OcaMediaClock3 */
static int mc_get(const struct oca_object *obj, uint8_t prop,
		  struct oca_value *v)
{
	ARG_UNUSED(obj); ARG_UNUSED(prop);
	v->u8 = ptp_ctrl_wallclock_locked() ? 1 : 0;
	return 0;
}

static void write_clock_rate(struct ocp1_writer *out, float nominal)
{
	ocp1_w_f32(out, nominal);   /* NominalRate (Hz) */
	ocp1_w_f32(out, NAN);       /* PullRange: unspecified */
	ocp1_w_f32(out, NAN);       /* Accuracy */
	ocp1_w_f32(out, NAN);       /* JitterMax */
}

static int mclk3_custom(struct oca_object *obj, struct oca_session *s,
			uint16_t level, uint16_t index, uint8_t in_count,
			struct ocp1_reader *in, struct ocp1_writer *out,
			uint8_t *out_count)
{
	ARG_UNUSED(obj); ARG_UNUSED(s); ARG_UNUSED(in_count); ARG_UNUSED(in);
	if (level != 3) {
		return OCA_METHOD_UNHANDLED;
	}
	switch (index) {
	case 3: /* GetCurrentRate -> rate, TimeSourceONo */
		write_clock_rate(out, (float)aes67_config_get()->default_sample_rate);
		ocp1_w_u32(out, ONO_TIMESRC);
		*out_count = 2;
		return OCA_STATUS_OK;
	case 5: /* GetOffset -> OcaTime {Negative, Seconds, Nanoseconds} */
		ocp1_w_bool(out, false);
		ocp1_w_u64(out, 0);
		ocp1_w_u32(out, 0);
		*out_count = 1;
		return OCA_STATUS_OK;
	case 7: /* GetSupportedRates -> map<ONo, list<rate>> */
		ocp1_w_u16(out, 1);
		ocp1_w_u32(out, ONO_TIMESRC);
		ocp1_w_u16(out, 2);
		write_clock_rate(out, 48000.0f);
		write_clock_rate(out, 96000.0f);
		*out_count = 1;
		return OCA_STATUS_OK;
	case 2: case 4: case 6:
		return OCA_STATUS_NOT_IMPLEMENTED;
	default:
		return OCA_METHOD_UNHANDLED;
	}
}

/* ================================================================
 * Network block
 * ================================================================ */

enum { NET_IP = 1, NET_MAC, NET_HOSTNAME, NET_SAP = 10 };

static int net_get(const struct oca_object *obj, uint8_t prop,
		   struct oca_value *v)
{
	ARG_UNUSED(prop);
	switch (obj->arg) {
	case NET_IP:
		if (dev_ip.s_addr == 0) {
			v->str[0] = '\0';
		} else {
			net_addr_ntop(AF_INET, &dev_ip, v->str, sizeof(v->str));
		}
		break;
	case NET_MAC: {
		const struct net_linkaddr *la =
			dev_iface ? net_if_get_link_addr(dev_iface) : NULL;

		if (la != NULL && la->len == 6) {
			snprintf(v->str, sizeof(v->str),
				 "%02x:%02x:%02x:%02x:%02x:%02x",
				 la->addr[0], la->addr[1], la->addr[2],
				 la->addr[3], la->addr[4], la->addr[5]);
		} else {
			v->str[0] = '\0';
		}
		break;
	}
	case NET_HOSTNAME:
#ifdef CONFIG_NET_HOSTNAME_ENABLE
		set_str(v, net_hostname_get());
#else
		v->str[0] = '\0';
#endif
		break;
	case NET_SAP:
		v->b = aes67_config_get()->sap_announce_enabled;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int net_set(const struct oca_object *obj, uint8_t prop,
		   const struct oca_value *v)
{
	ARG_UNUSED(prop);
	if (obj->arg != NET_SAP) {
		return -ENOTSUP;
	}
	aes67_config_lock();
	aes67_config_get()->sap_announce_enabled = v->b;
	aes67_config_unlock();
	sap_sdp_set_announce(v->b);
	(void)aes67_config_persist();
	return 0;
}

/* Tree construction helpers (defined below). */
static int add(uint32_t ono, const struct oca_kind *kind, const char *role,
	       uint32_t owner, uint16_t arg, oca_get_fn get, oca_set_fn set);
static int add_block(uint32_t ono, const char *role, uint32_t owner);

/* ================================================================
 * Auracast card (ESP32-S31 BLE Audio transmitter)
 * ================================================================ */

#ifdef CONFIG_AURACAST_CARD

enum {
	AU_ENABLED = 1, AU_NAME, AU_CODE, AU_PRESET, AU_CHANNELS, AU_CONTEXT,
	AU_IMMEDIATE, AU_TX_POWER, AU_PD_US, AU_FIFO_TARGET, AU_BROADCAST_ID,
	AU_HQ_PRESET, AU_HQ_CHANNELS, AU_HQ_CONTEXT,
	AU_STATE = 20, AU_ERROR, AU_STREAMING, AU_ACTIVE_ID, AU_FIFO_FILL,
	AU_DRIFT,
};

static int aura_get(const struct oca_object *obj, uint8_t prop,
		    struct oca_value *v)
{
	struct auracast_config cfg;
	struct auracast_status st;
	int ret;

	ARG_UNUSED(prop);
	if (obj->arg >= AU_STATE) {
		ret = auracast_card_get_status(&st);
	} else {
		ret = auracast_card_get_config(&cfg);
	}
	if (ret < 0) {
		return ret;
	}
	switch (obj->arg) {
	case AU_ENABLED:   v->b = cfg.enabled; break;
	case AU_NAME:      set_str(v, cfg.name); break;
	case AU_CODE:      set_str(v, cfg.code); break;
	case AU_PRESET:
		v->u8 = cfg.preset;
		v->min = 0;
		v->max = AURA_PRESET_COUNT - 1;
		break;
	case AU_CHANNELS:
		v->u8 = cfg.channels;
		v->min = 0;
		v->max = AURA_CH_COUNT - 1;
		break;
	case AU_CONTEXT:
		v->u16 = cfg.context;
		v->min = 0;
		v->max = AURA_CONTEXT_MASK;
		break;
	case AU_IMMEDIATE: v->b = (cfg.flags & AURA_FLAG_IMMEDIATE) != 0; break;
	case AU_TX_POWER:
		v->i32 = cfg.tx_power_dbm;
		v->min = AURA_TX_POWER_MIN;
		v->max = AURA_TX_POWER_MAX;
		break;
	case AU_PD_US:
		v->u16 = cfg.pd_us;
		v->min = AURA_PD_US_MIN;
		v->max = AURA_PD_US_MAX;
		break;
	case AU_FIFO_TARGET:
		v->u16 = cfg.fifo_target;
		v->min = AURA_FIFO_TARGET_MIN;
		v->max = AURA_FIFO_TARGET_MAX;
		break;
	case AU_BROADCAST_ID:
		if (cfg.broadcast_id == AURA_BROADCAST_ID_RANDOM) {
			set_str(v, "random");
		} else {
			snprintf(v->str, sizeof(v->str), "0x%06X",
				 cfg.broadcast_id & 0xFFFFFF);
		}
		break;
	case AU_HQ_PRESET:
		v->u8 = cfg.hq_preset;
		v->min = 0;
		v->max = 255;
		break;
	case AU_HQ_CHANNELS:
		v->u8 = cfg.hq_channels;
		v->min = 0;
		v->max = AURA_CH_COUNT - 1;
		break;
	case AU_HQ_CONTEXT:
		v->u16 = cfg.hq_context;
		v->min = 0;
		v->max = AURA_CONTEXT_MASK;
		break;
	case AU_STATE:
		set_str(v, st.present ? aura_proto_state_name(st.state) : "offline");
		break;
	case AU_ERROR:
		if (st.state == AURA_STATE_CONFIG_ERROR || st.state == AURA_STATE_FAULT) {
			snprintf(v->str, sizeof(v->str), "%s (%s)",
				 aura_proto_error_name(st.error),
				 aura_proto_reg_name(st.error_reg));
		} else {
			v->str[0] = '\0';
		}
		break;
	case AU_STREAMING:
		v->b = st.present && st.state == AURA_STATE_STREAMING;
		break;
	case AU_ACTIVE_ID:
		if (st.stats_valid) {
			snprintf(v->str, sizeof(v->str), "0x%06X",
				 st.stats.active_id & 0xFFFFFF);
		} else {
			v->str[0] = '\0';
		}
		break;
	case AU_FIFO_FILL:
		v->i32 = st.stats_valid ? st.stats.fifo_fill : 0;
		v->min = 0;
		v->max = 65535;
		break;
	case AU_DRIFT:
		v->i32 = st.stats_valid ? st.stats.drift_cppm : 0;
		v->min = -2.0e9f;
		v->max = 2.0e9f;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int aura_set(const struct oca_object *obj, uint8_t prop,
		    const struct oca_value *v)
{
	struct auracast_config cfg;
	int ret = auracast_card_get_config(&cfg);

	ARG_UNUSED(prop);
	if (ret < 0) {
		return ret;
	}
	switch (obj->arg) {
	case AU_ENABLED:
		cfg.enabled = v->b;
		break;
	case AU_NAME:
		if (strlen(v->str) > AURA_NAME_SIZE) {
			return -ERANGE;
		}
		copy_str(cfg.name, sizeof(cfg.name), v->str);
		break;
	case AU_CODE:
		if (strlen(v->str) > AURA_CODE_SIZE) {
			return -ERANGE;
		}
		copy_str(cfg.code, sizeof(cfg.code), v->str);
		break;
	case AU_PRESET:      cfg.preset = v->u8; break;
	case AU_CHANNELS:    cfg.channels = v->u8; break;
	case AU_CONTEXT:     cfg.context = v->u16; break;
	case AU_IMMEDIATE:
		cfg.flags = v->b ? (cfg.flags | AURA_FLAG_IMMEDIATE)
				 : (cfg.flags & ~AURA_FLAG_IMMEDIATE);
		break;
	case AU_TX_POWER:
		if (v->i32 < AURA_TX_POWER_MIN || v->i32 > AURA_TX_POWER_MAX) {
			return -ERANGE;
		}
		cfg.tx_power_dbm = (int8_t)v->i32;
		break;
	case AU_PD_US:       cfg.pd_us = v->u16; break;
	case AU_FIFO_TARGET: cfg.fifo_target = v->u16; break;
	case AU_BROADCAST_ID:
		if (strcasecmp(v->str, "random") == 0 || v->str[0] == '\0') {
			cfg.broadcast_id = AURA_BROADCAST_ID_RANDOM;
		} else {
			char *end;
			unsigned long id = strtoul(v->str, &end, 0);

			if (end == v->str || *end != '\0' || id > 0xFFFFFF) {
				return -EINVAL;
			}
			cfg.broadcast_id = (uint32_t)id;
		}
		break;
	case AU_HQ_PRESET:   cfg.hq_preset = v->u8; break;
	case AU_HQ_CHANNELS: cfg.hq_channels = v->u8; break;
	case AU_HQ_CONTEXT:  cfg.hq_context = v->u16; break;
	default:
		return -ENOTSUP;
	}
	/* Local protocol-limit check first so an S31 rejection and a bad
	 * parameter both come back as ParameterError/OutOfRange, not as a
	 * device error. */
	if (aura_proto_config_validate(&cfg, NULL) < 0) {
		return -ERANGE;
	}
	ret = auracast_card_set_config(&cfg);
	if (ret < 0) {
		return ret;
	}
	card_settings_schedule_save();
	return 0;
}

static void add_auracast(void)
{
	static const struct {
		uint16_t off;
		const struct oca_kind *kind;
		const char *role;
		bool settable;
	} items[] = {
		{ AU_ENABLED,      &kind_bool_act, "Enabled",              true },
		{ AU_NAME,         &kind_str_act,  "Name",                 true },
		{ AU_CODE,         &kind_str_act,  "Code",                 true },
		{ AU_PRESET,       &kind_u8_act,   "Preset",               true },
		{ AU_CHANNELS,     &kind_u8_act,   "Channels",             true },
		{ AU_CONTEXT,      &kind_u16_act,  "Context",              true },
		{ AU_IMMEDIATE,    &kind_bool_act, "ImmediateRendering",   true },
		{ AU_TX_POWER,     &kind_i32_act,  "TxPowerDbm",           true },
		{ AU_PD_US,        &kind_u16_act,  "PresentationDelayUs",  true },
		{ AU_FIFO_TARGET,  &kind_u16_act,  "FifoTarget",           true },
		{ AU_BROADCAST_ID, &kind_str_act,  "BroadcastID",          true },
		{ AU_HQ_PRESET,    &kind_u8_act,   "HQPreset",             true },
		{ AU_HQ_CHANNELS,  &kind_u8_act,   "HQChannels",           true },
		{ AU_HQ_CONTEXT,   &kind_u16_act,  "HQContext",            true },
		{ AU_STATE,        &kind_str_sen,  "State",                false },
		{ AU_ERROR,        &kind_str_sen,  "Error",                false },
		{ AU_STREAMING,    &kind_bool_sen, "Streaming",            false },
		{ AU_ACTIVE_ID,    &kind_str_sen,  "ActiveBroadcastID",    false },
		{ AU_FIFO_FILL,    &kind_i32_sen,  "FifoFill",             false },
		{ AU_DRIFT,        &kind_i32_sen,  "DriftCppm",            false },
	};

	add_block(ONO_AURACAST, "Auracast", OCA_ONO_ROOT_BLOCK);
	for (size_t i = 0; i < ARRAY_SIZE(items); i++) {
		add(ONO_AURACAST + items[i].off, items[i].kind, items[i].role,
		    ONO_AURACAST, items[i].off, aura_get,
		    items[i].settable ? aura_set : NULL);
	}
}
#endif /* CONFIG_AURACAST_CARD */

/* ================================================================
 * Managers
 * ================================================================ */

static int devmgr_get(const struct oca_object *obj, uint8_t prop,
		      struct oca_value *v)
{
	ARG_UNUSED(obj); ARG_UNUSED(prop);
	aes67_config_lock();
	set_str(v, aes67_config_get()->device_name);
	aes67_config_unlock();
	return 0;
}

static int set_device_name(const char *name)
{
	struct aes67_device_config *cfg = aes67_config_get();

	if (name[0] == '\0' || strlen(name) >= AES67_DEVICE_NAME_MAX) {
		return OCA_STATUS_PARAMETER_OUT_OF_RANGE;
	}
	aes67_config_lock();
	copy_str(cfg->device_name, AES67_DEVICE_NAME_MAX, name);
	aes67_config_unlock();
	(void)aes67_config_persist();
#ifdef CONFIG_MDNS_SD
	mdns_sd_update_device_name();
#endif
	return OCA_STATUS_OK;
}

static int devmgr_custom(struct oca_object *obj, struct oca_session *s,
			 uint16_t level, uint16_t index, uint8_t in_count,
			 struct ocp1_reader *in, struct ocp1_writer *out,
			 uint8_t *out_count)
{
	const struct aes67_device_config *cfg = aes67_config_get();
	char tmp[OCA_STR_MAX];
	int ret;

	ARG_UNUSED(in_count);
	if (level != 3) {
		return OCA_STATUS_BAD_METHOD;
	}

	/* Setters first: they need the write-lock check. */
	switch (index) {
	case 5:  /* SetDeviceName */
	case 8:  /* SetDeviceRole */
	case 10: /* SetUserInventoryCode */
	case 18: /* SetMessage */
		ret = aes70_core_check_write(obj, s);
		if (ret != OCA_STATUS_OK) {
			return ret;
		}
		ocp1_r_str(in, tmp, sizeof(tmp));
		if (in->err) {
			return OCA_STATUS_BAD_FORMAT;
		}
		if (index == 5) {
			ret = set_device_name(tmp);
			if (ret == OCA_STATUS_OK) {
				aes70_core_refresh_object(obj->ono, k_uptime_get());
			}
			return ret;
		}
		copy_str(index == 8 ? dev_role :
			 index == 10 ? dev_inventory : dev_message,
			 OCA_STR_MAX, tmp);
		return OCA_STATUS_OK;
	default:
		break;
	}

	*out_count = 1;
	switch (index) {
	case 1: /* GetOcaVersion */
		ocp1_w_u16(out, AES70_OCA_VERSION);
		return OCA_STATUS_OK;
	case 2: { /* GetModelGUID: reserved(1) mfr(3) model(4) */
		static const uint8_t guid[8] = {0, 0, 0, 0, 'A', 'E', '6', '7'};

		ocp1_w_raw(out, guid, sizeof(guid));
		return OCA_STATUS_OK;
	}
	case 3: /* GetSerialNumber */
		aes67_config_lock();
		ocp1_w_str(out, cfg->serial);
		aes67_config_unlock();
		return OCA_STATUS_OK;
	case 4: /* GetDeviceName */
		aes67_config_lock();
		ocp1_w_str(out, cfg->device_name);
		aes67_config_unlock();
		return OCA_STATUS_OK;
	case 6: /* GetModelDescription {Manufacturer, Name, Version} */
		aes67_config_lock();
		ocp1_w_str(out, cfg->vendor);
		ocp1_w_str(out, cfg->product);
		aes67_config_unlock();
		ocp1_w_str(out, AES70_FW_VERSION_STR);
		return OCA_STATUS_OK;
	case 7: /* GetDeviceRole */
		ocp1_w_str(out, dev_role);
		return OCA_STATUS_OK;
	case 9: /* GetUserInventoryCode */
		ocp1_w_str(out, dev_inventory);
		return OCA_STATUS_OK;
	case 11: /* GetEnabled */
		ocp1_w_bool(out, true);
		return OCA_STATUS_OK;
	case 13: /* GetState (deprecated OcaDeviceState bitset): operational */
		ocp1_w_u16(out, 0);
		return OCA_STATUS_OK;
	case 15: /* GetResetCause */
		ocp1_w_u8(out, 0);   /* PowerOn */
		return OCA_STATUS_OK;
	case 16: /* ClearResetCause */
		*out_count = 0;
		return OCA_STATUS_OK;
	case 17: /* GetMessage */
		ocp1_w_str(out, dev_message);
		return OCA_STATUS_OK;
	case 19: { /* GetManagers -> list<OcaManagerDescriptor> */
		size_t off = out->len;
		uint16_t n = 0;

		ocp1_w_u16(out, 0);
		for (int i = 0; ; i++) {
			const struct oca_object *o = aes70_core_object_at(i);

			if (o == NULL) {
				break;
			}
			if (o->kind->base != OCA_BASE_MANAGER) {
				continue;
			}
			ocp1_w_u32(out, o->ono);
			ocp1_w_str(out, o->kind->name);
			ocp1_w_class_id(out, o->kind->class_id,
					o->kind->class_id_len);
			ocp1_w_u16(out, o->kind->class_version);
			n++;
		}
		ocp1_w_patch_u16(out, off, n);
		return OCA_STATUS_OK;
	}
	case 20: /* GetDeviceRevisionID */
		ocp1_w_str(out, AES70_FW_VERSION_STR);
		return OCA_STATUS_OK;
	case 21: { /* GetManufacturer {Name, OrganizationID(3), Website,
		    *                  BusinessContact, TechnicalContact} */
		static const uint8_t oui[3] = {0, 0, 0};

		aes67_config_lock();
		ocp1_w_str(out, cfg->vendor);
		aes67_config_unlock();
		ocp1_w_raw(out, oui, 3);
		ocp1_w_str(out, "");
		ocp1_w_str(out, "");
		ocp1_w_str(out, "");
		return OCA_STATUS_OK;
	}
	case 22: /* GetProduct {Name, ModelID, RevisionLevel, BrandName,
		  *             UUID, Description} */
		aes67_config_lock();
		ocp1_w_str(out, cfg->product);
		ocp1_w_str(out, "AES67-FPGA");
		ocp1_w_str(out, AES70_FW_VERSION_STR);
		ocp1_w_str(out, cfg->vendor);
		aes67_config_unlock();
		ocp1_w_str(out, "");
		ocp1_w_str(out, "AES67 audio-over-IP node with FPGA data plane");
		return OCA_STATUS_OK;
	case 23: /* GetOperationalState {Generic, Details} */
		ocp1_w_u8(out, 0);        /* NormalOperation */
		ocp1_w_u16(out, 0);       /* empty blob */
		return OCA_STATUS_OK;
	case 24: /* GetLoggingEnabled */
		ocp1_w_bool(out, false);
		return OCA_STATUS_OK;
	case 26: /* GetMostRecentPatchDatasetONo */
		ocp1_w_u32(out, 0);
		return OCA_STATUS_OK;
	case 12: /* SetEnabled */
	case 14: /* SetResetKey (device reset mechanism not implemented) */
	case 25: /* SetLoggingEnabled */
	case 27: /* ApplyPatch */
		*out_count = 0;
		return OCA_STATUS_NOT_IMPLEMENTED;
	default:
		*out_count = 0;
		return OCA_STATUS_BAD_METHOD;
	}
}

static int fwmgr_custom(struct oca_object *obj, struct oca_session *s,
			uint16_t level, uint16_t index, uint8_t in_count,
			struct ocp1_reader *in, struct ocp1_writer *out,
			uint8_t *out_count)
{
	ARG_UNUSED(obj); ARG_UNUSED(s); ARG_UNUSED(in_count); ARG_UNUSED(in);
	if (level != 3) {
		return OCA_STATUS_BAD_METHOD;
	}
	if (index == 1) { /* GetComponentVersions -> list<OcaVersion> */
		ocp1_w_u16(out, 1);
		ocp1_w_u32(out, AES70_FW_VERSION_MAJOR);
		ocp1_w_u32(out, AES70_FW_VERSION_MINOR);
		ocp1_w_u32(out, AES70_FW_VERSION_BUILD);
		ocp1_w_u16(out, 1);   /* OcaComponent: 1 = application (0 = boot loader) */
		*out_count = 1;
		return OCA_STATUS_OK;
	}
	if (index >= 2 && index <= 8) {
		return OCA_STATUS_NOT_IMPLEMENTED;
	}
	return OCA_STATUS_BAD_METHOD;
}

static int mclkmgr_custom(struct oca_object *obj, struct oca_session *s,
			  uint16_t level, uint16_t index, uint8_t in_count,
			  struct ocp1_reader *in, struct ocp1_writer *out,
			  uint8_t *out_count)
{
	ARG_UNUSED(obj); ARG_UNUSED(s); ARG_UNUSED(in_count); ARG_UNUSED(in);
	if (level != 3) {
		return OCA_STATUS_BAD_METHOD;
	}
	switch (index) {
	case 1: /* GetClocks (deprecated OcaMediaClock): none */
	case 2: /* GetMediaClockTypesSupported */
		ocp1_w_u16(out, 0);
		*out_count = 1;
		return OCA_STATUS_OK;
	case 3: /* GetClock3s */
		ocp1_w_u16(out, 1);
		ocp1_w_u32(out, ONO_MCLK);
		*out_count = 1;
		return OCA_STATUS_OK;
	default:
		return OCA_STATUS_BAD_METHOD;
	}
}

/* ================================================================
 * Tree construction
 * ================================================================ */

static int add(uint32_t ono, const struct oca_kind *kind, const char *role,
	       uint32_t owner, uint16_t arg, oca_get_fn get, oca_set_fn set)
{
	struct oca_object o = {
		.ono = ono, .kind = kind, .role = role, .owner = owner,
		.arg = arg, .lockable = (set != NULL), .get = get, .set = set,
	};
	int ret = aes70_core_add_object(&o);

	if (ret < 0) {
		LOG_ERR("object %u (%s) not added: %d", ono, role, ret);
	}
	return ret;
}

static int add_block(uint32_t ono, const char *role, uint32_t owner)
{
	return add(ono, &oca_kind_block, role, owner, 0, NULL, NULL);
}

/* Role strings must outlive the model: static tables per channel/slot. */
static char in_roles[CARD_SETTINGS_MAX_IN][8];
static char out_roles[CARD_SETTINGS_MAX_OUT][8];
static char tx_roles[AES67_MAX_TX_STREAMS][8];
static char rx_roles[AES67_MAX_RX_STREAMS][8];

static void add_streams(void)
{
	add_block(ONO_STREAMS, "Streams", OCA_ONO_ROOT_BLOCK);
	add_block(ONO_TX, "TX", ONO_STREAMS);
	for (int s = 0; s < AES67_MAX_TX_STREAMS; s++) {
		uint32_t b = ONO_TX_BASE + 10 * s;

		snprintf(tx_roles[s], sizeof(tx_roles[s]), "TX %d", s + 1);
		add_block(b, tx_roles[s], ONO_TX);
		add(b + 1, &kind_bool_act, "Active", b, (s << 8) | TXF_ACTIVE,
		    tx_get, tx_set);
		add(b + 2, &kind_str_act, "Name", b, (s << 8) | TXF_NAME,
		    tx_get, tx_set);
		add(b + 3, &kind_str_act, "Destination", b, (s << 8) | TXF_DEST,
		    tx_get, tx_set);
		add(b + 4, &kind_str_act, "Channels", b, (s << 8) | TXF_CHANNELS,
		    tx_get, tx_set);
		add(b + 5, &kind_u16_act, "SamplesPerPacket", b,
		    (s << 8) | TXF_SPP, tx_get, tx_set);
	}
	add_block(ONO_RX, "RX", ONO_STREAMS);
	for (int s = 0; s < AES67_MAX_RX_STREAMS; s++) {
		uint32_t b = ONO_RX_BASE + 10 * s;

		snprintf(rx_roles[s], sizeof(rx_roles[s]), "RX %d", s + 1);
		add_block(b, rx_roles[s], ONO_RX);
		add(b + 1, &kind_bool_act, "Active", b, (s << 8) | RXF_ACTIVE,
		    rx_get, rx_set);
		add(b + 2, &kind_str_act, "Name", b, (s << 8) | RXF_NAME,
		    rx_get, rx_set);
		add(b + 3, &kind_str_act, "Address", b, (s << 8) | RXF_ADDR,
		    rx_get, rx_set);
		add(b + 4, &kind_str_act, "ChannelMap", b, (s << 8) | RXF_CHMAP,
		    rx_get, rx_set);
		add(b + 5, &kind_u16_act, "OutputDelay", b, (s << 8) | RXF_DELAY,
		    rx_get, rx_set);
		add(b + 6, &kind_u16_act, "SamplesPerChannel", b,
		    (s << 8) | RXF_SPC, rx_get, rx_set);
	}

	/* Seed the shadows with the restored configuration so an idle
	 * slot shows its last settings (and sensible defaults). */
	const struct aes67_device_config *cfg = aes67_config_get();

	memcpy(tx_shadow, aes67_conn_get_tx_streams(), sizeof(tx_shadow));
	memcpy(rx_shadow, aes67_conn_get_rx_streams(), sizeof(rx_shadow));
	for (int s = 0; s < AES67_MAX_TX_STREAMS; s++) {
		tx_shadow[s].active = false;
		if (tx_shadow[s].samples_per_packet == 0) {
			tx_shadow[s].samples_per_packet =
				(uint8_t)cfg->default_samples_per_pkt;
		}
	}
	for (int s = 0; s < AES67_MAX_RX_STREAMS; s++) {
		rx_shadow[s].active = false;
		if (rx_shadow[s].dst_port == 0) {
			rx_shadow[s].dst_port = cfg->default_port;
		}
		if (rx_shadow[s].samples_per_channel == 0) {
			rx_shadow[s].samples_per_channel =
				(uint8_t)cfg->default_samples_per_pkt;
		}
	}
	aes67_conn_register_tx_observer(stream_observer);
	aes67_conn_register_rx_observer(stream_observer);
}

static void add_card(void)
{
	int n_in = 0, n_out = 0;

	switch (card_type()) {
	case CARD_TYPE_MI: n_in = 8; break;
	case CARD_TYPE_LO: n_out = 8; break;
	case CARD_TYPE_IO: n_in = 16; n_out = 8; break;
	default: break;
	}
	if (n_in > CARD_SETTINGS_MAX_IN) {
		n_in = CARD_SETTINGS_MAX_IN;
	}
	if (n_out > CARD_SETTINGS_MAX_OUT) {
		n_out = CARD_SETTINGS_MAX_OUT;
	}

	if (n_in > 0) {
		add_block(ONO_INPUTS, "Inputs", OCA_ONO_ROOT_BLOCK);
		for (int ch = 0; ch < n_in; ch++) {
			uint32_t b = ONO_IN_BASE + 10 * ch;

			snprintf(in_roles[ch], sizeof(in_roles[ch]), "In %d",
				 ch + 1);
			add_block(b, in_roles[ch], ONO_INPUTS);
			add(b + 1, &kind_gain, "Gain", b, ch, in_get, in_set);
			add(b + 2, &kind_mute, "Mute", b, ch, in_get, in_set);
			add(b + 3, &kind_bool_act, "Phantom", b, ch, in_get,
			    in_set);
		}
	}
#ifdef CONFIG_AURACAST_CARD
	if (card_type() == CARD_TYPE_AURACAST) {
		add_auracast();
	}
#endif
	if (n_out > 0) {
		add_block(ONO_OUTPUTS, "Outputs", OCA_ONO_ROOT_BLOCK);
		for (int ch = 0; ch < n_out; ch++) {
			uint32_t b = ONO_OUT_BASE + 10 * ch;

			snprintf(out_roles[ch], sizeof(out_roles[ch]), "Out %d",
				 ch + 1);
			add_block(b, out_roles[ch], ONO_OUTPUTS);
			add(b + 1, &kind_gain, "Level", b, ch, out_get, out_set);
			add(b + 2, &kind_mute, "Mute", b, ch, out_get, out_set);
		}
	}
}

int aes70_device_build(struct net_if *iface)
{
	dev_iface = iface;
	aes70_core_init();

	add(OCA_ONO_DEVICE_MANAGER, &kind_devmgr, "DeviceManager", 0, 0,
	    devmgr_get, NULL);
	/* lockable so a controller can claim the device name */
	aes70_core_find(OCA_ONO_DEVICE_MANAGER)->lockable = true;
	add(OCA_ONO_FIRMWARE_MANAGER, &kind_fwmgr, "FirmwareManager", 0, 0,
	    NULL, NULL);
	add(OCA_ONO_SUBSCRIPTION_MANAGER, &oca_kind_subscription_manager,
	    "SubscriptionManager", 0, 0, NULL, NULL);
	add(OCA_ONO_MEDIA_CLOCK_MANAGER, &kind_mclkmgr, "MediaClockManager",
	    0, 0, NULL, NULL);
	add_block(OCA_ONO_ROOT_BLOCK, "Root", 0);

	add_card();
	add_streams();

	add_block(ONO_CLOCK, "Clock", OCA_ONO_ROOT_BLOCK);
	add(ONO_TIMESRC, &kind_timesrc, "PTP", ONO_CLOCK, 0, ts_get, NULL);
	add(ONO_MCLK, &kind_mclk3, "MediaClock", ONO_CLOCK, 0, mc_get, NULL);
	add(4010, &kind_str_sen, "Role", ONO_CLOCK, CLK_ROLE, clock_get, NULL);
	add(4011, &kind_i32_sen, "OffsetNs", ONO_CLOCK, CLK_OFFSET, clock_get,
	    NULL);
	add(4012, &kind_i32_sen, "PathDelayNs", ONO_CLOCK, CLK_PATH_DELAY,
	    clock_get, NULL);
	add(4013, &kind_bool_sen, "Locked", ONO_CLOCK, CLK_LOCKED, clock_get,
	    NULL);
	add(4014, &kind_str_sen, "Grandmaster", ONO_CLOCK, CLK_GM, clock_get,
	    NULL);
	add(4020, &kind_u8_act, "Domain", ONO_CLOCK, CLK_DOMAIN, clock_get,
	    clock_set);
	add(4021, &kind_u8_act, "Priority1", ONO_CLOCK, CLK_PRIO1, clock_get,
	    clock_set);
	add(4022, &kind_u8_act, "Priority2", ONO_CLOCK, CLK_PRIO2, clock_get,
	    clock_set);

	add_block(ONO_NETWORK, "Network", OCA_ONO_ROOT_BLOCK);
	add(5001, &kind_str_sen, "IPAddress", ONO_NETWORK, NET_IP, net_get, NULL);
	add(5002, &kind_str_sen, "MACAddress", ONO_NETWORK, NET_MAC, net_get,
	    NULL);
	add(5003, &kind_str_sen, "Hostname", ONO_NETWORK, NET_HOSTNAME, net_get,
	    NULL);
	add(5010, &kind_bool_act, "SAPAnnounce", ONO_NETWORK, NET_SAP, net_get,
	    net_set);

	LOG_INF("AES70 model: %d objects", aes70_core_object_count());
	return 0;
}

void aes70_device_set_ip(const struct in_addr *addr)
{
	dev_ip = *addr;
	aes70_core_set_dirty();
}
