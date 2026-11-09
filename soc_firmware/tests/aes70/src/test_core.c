/*
 * AES70 session core driven through the byte-level session API with a
 * synthetic object model: root block -> "Inputs" block -> gain / mute /
 * a string sensor, plus the subscription manager.
 */

#include <zephyr/ztest.h>
#include <string.h>

#include "aes70_model.h"

/* ---- synthetic device ---- */

static const uint16_t cid_gain[] = {1, 1, 1, 5};
static const struct oca_prop props_gain[] = {
	{ 4, 1, OCA_VT_F32, 1, 2, OCA_PF_RANGE | OCA_PF_NOTIFY },
};
static const struct oca_kind kind_gain = {
	.name = "OcaGain", .class_id = cid_gain, .class_id_len = 4,
	.class_version = 3, .base = OCA_BASE_WORKER,
	.props = props_gain, .nprops = 1,
};

static const uint16_t cid_strsen[] = {1, 1, 2, 1, 12};
static const struct oca_prop props_strsen[] = {
	{ 5, 1, OCA_VT_STR, 1, 0, OCA_PF_NOTIFY },
};
static const struct oca_kind kind_strsen = {
	.name = "OcaStringSensor", .class_id = cid_strsen, .class_id_len = 5,
	.class_version = 3, .base = OCA_BASE_WORKER,
	.props = props_strsen, .nprops = 1,
};

static float gain_value;
static char sensor_value[OCA_STR_MAX];
static int set_err;   /* forced errno for the setter */

static int gain_get(const struct oca_object *obj, uint8_t prop,
		    struct oca_value *v)
{
	ARG_UNUSED(obj); ARG_UNUSED(prop);
	v->f32 = gain_value;
	v->min = -10.0f;
	v->max = 10.0f;
	return 0;
}

static int gain_set(const struct oca_object *obj, uint8_t prop,
		    const struct oca_value *v)
{
	ARG_UNUSED(obj); ARG_UNUSED(prop);
	if (set_err) {
		return set_err;
	}
	gain_value = v->f32;
	return 0;
}

static int sensor_get(const struct oca_object *obj, uint8_t prop,
		      struct oca_value *v)
{
	ARG_UNUSED(obj); ARG_UNUSED(prop);
	strcpy(v->str, sensor_value);
	return 0;
}

#define ONO_INPUTS 1000
#define ONO_GAIN   1001
#define ONO_SENSOR 1002

static void build_model(void)
{
	struct oca_object o;

	aes70_core_init();
	gain_value = 0.0f;
	set_err = 0;
	strcpy(sensor_value, "idle");

	o = (struct oca_object){ .ono = OCA_ONO_SUBSCRIPTION_MANAGER,
		.kind = &oca_kind_subscription_manager, .role = "SubMgr" };
	zassert_equal(aes70_core_add_object(&o), 0);
	o = (struct oca_object){ .ono = OCA_ONO_ROOT_BLOCK,
		.kind = &oca_kind_block, .role = "Root" };
	zassert_equal(aes70_core_add_object(&o), 0);
	o = (struct oca_object){ .ono = ONO_INPUTS, .kind = &oca_kind_block,
		.role = "Inputs", .owner = OCA_ONO_ROOT_BLOCK };
	zassert_equal(aes70_core_add_object(&o), 0);
	o = (struct oca_object){ .ono = ONO_GAIN, .kind = &kind_gain,
		.role = "Gain", .owner = ONO_INPUTS, .lockable = true,
		.get = gain_get, .set = gain_set };
	zassert_equal(aes70_core_add_object(&o), 0);
	o = (struct oca_object){ .ono = ONO_SENSOR, .kind = &kind_strsen,
		.role = "State", .owner = ONO_INPUTS, .get = sensor_get };
	zassert_equal(aes70_core_add_object(&o), 0);
	/* duplicate ONo is refused */
	zassert_equal(aes70_core_add_object(&o), -EEXIST);
}

/* ---- capture transport ---- */

struct capture {
	uint8_t buf[2048];
	size_t len;
};

static struct capture cap[2];
static struct oca_session *sess[2];

static int cap_send(struct oca_session *s, const uint8_t *buf, size_t len)
{
	struct capture *c = s->transport;

	zassert_true(c->len + len <= sizeof(c->buf), "capture overflow");
	memcpy(c->buf + c->len, buf, len);
	c->len += len;
	return 0;
}

static void open_sessions(void)
{
	for (int i = 0; i < 2; i++) {
		cap[i].len = 0;
		sess[i] = aes70_core_session_open(cap_send, &cap[i], 1000);
		zassert_not_null(sess[i]);
	}
}

/* Build one Ocp1CommandPdu (response required) with a single command. */
static size_t build_cmd(uint8_t *out, size_t cap_len, uint32_t handle,
			uint32_t ono, uint16_t level, uint16_t index,
			uint8_t pcount, const uint8_t *params, size_t plen)
{
	struct ocp1_writer w;
	size_t off, cmd_off;

	ocp1_w_init(&w, out, cap_len);
	off = ocp1_begin_pdu(&w, OCP1_PDU_CMD_RRQ, 1);
	cmd_off = w.len;
	ocp1_w_u32(&w, 0);
	ocp1_w_u32(&w, handle);
	ocp1_w_u32(&w, ono);
	ocp1_w_u16(&w, level);
	ocp1_w_u16(&w, index);
	ocp1_w_u8(&w, pcount);
	ocp1_w_raw(&w, params, plen);
	ocp1_w_patch_u32(&w, cmd_off, (uint32_t)(w.len - cmd_off));
	ocp1_end_pdu(&w, off);
	zassert_false(w.overflow);
	return w.len;
}

struct response {
	uint32_t handle;
	uint8_t status;
	uint8_t count;
	struct ocp1_reader params;
};

/* Send a command on session i and parse the (single) response. */
static void call(int i, uint32_t ono, uint16_t level, uint16_t index,
		 uint8_t pcount, const uint8_t *params, size_t plen,
		 struct response *rsp)
{
	uint8_t pdu[256];
	struct ocp1_header h;
	struct ocp1_reader r;
	uint32_t size;
	size_t n = build_cmd(pdu, sizeof(pdu), 0x5151, ono, level, index,
			     pcount, params, plen);

	cap[i].len = 0;
	zassert_equal(aes70_core_session_input(sess[i], pdu, n, 2000), 0);
	zassert_true(cap[i].len >= OCP1_MIN_PDU_LEN, "no response");
	zassert_equal(ocp1_parse_header(cap[i].buf, cap[i].len, &h), 0);
	zassert_equal(h.type, OCP1_PDU_RSP);
	zassert_equal(h.count, 1);
	zassert_equal(1 + h.pdu_size, cap[i].len);

	ocp1_r_init(&r, cap[i].buf + OCP1_MIN_PDU_LEN,
		    cap[i].len - OCP1_MIN_PDU_LEN);
	size = ocp1_r_u32(&r);
	zassert_equal(size, cap[i].len - OCP1_MIN_PDU_LEN);
	rsp->handle = ocp1_r_u32(&r);
	rsp->status = ocp1_r_u8(&r);
	rsp->count = ocp1_r_u8(&r);
	ocp1_r_init(&rsp->params, r.buf + r.pos, r.len - r.pos);
	zassert_equal(rsp->handle, 0x5151);
}

static void before_each(void *fixture)
{
	ARG_UNUSED(fixture);
	build_model();
	open_sessions();
}

ZTEST_SUITE(aes70_core, NULL, NULL, before_each, NULL, NULL);

/* ---- tests ---- */

ZTEST(aes70_core, test_class_identification_and_role)
{
	struct response rsp;
	char role[16];

	call(0, ONO_GAIN, 1, 1, 0, NULL, 0, &rsp);
	zassert_equal(rsp.status, OCA_STATUS_OK);
	zassert_equal(rsp.count, 1);
	zassert_equal(ocp1_r_u16(&rsp.params), 4);     /* field count */
	zassert_equal(ocp1_r_u16(&rsp.params), 1);
	zassert_equal(ocp1_r_u16(&rsp.params), 1);
	zassert_equal(ocp1_r_u16(&rsp.params), 1);
	zassert_equal(ocp1_r_u16(&rsp.params), 5);
	zassert_equal(ocp1_r_u16(&rsp.params), 3);     /* class version */
	zassert_equal(ocp1_r_remaining(&rsp.params), 0);

	call(0, ONO_GAIN, 1, 5, 0, NULL, 0, &rsp);
	zassert_equal(rsp.status, OCA_STATUS_OK);
	ocp1_r_str(&rsp.params, role, sizeof(role));
	zassert_str_equal(role, "Gain");
}

ZTEST(aes70_core, test_error_statuses)
{
	struct response rsp;

	call(0, 4242, 1, 1, 0, NULL, 0, &rsp);
	zassert_equal(rsp.status, OCA_STATUS_BAD_ONO);
	zassert_equal(rsp.count, 0);

	call(0, ONO_GAIN, 4, 9, 0, NULL, 0, &rsp);
	zassert_equal(rsp.status, OCA_STATUS_BAD_METHOD);

	/* OcaWorker.SetEnabled is present but not implemented */
	call(0, ONO_GAIN, 2, 2, 1, (const uint8_t[]){1}, 1, &rsp);
	zassert_equal(rsp.status, OCA_STATUS_NOT_IMPLEMENTED);

	/* sensor has no setter */
	call(0, ONO_SENSOR, 5, 2, 1, (const uint8_t[]){0, 0}, 2, &rsp);
	zassert_equal(rsp.status, OCA_STATUS_BAD_METHOD);

	/* lock methods on a non-lockable object */
	call(0, ONO_SENSOR, 1, 3, 0, NULL, 0, &rsp);
	zassert_equal(rsp.status, OCA_STATUS_NOT_IMPLEMENTED);

	/* truncated parameter */
	call(0, ONO_GAIN, 4, 2, 1, (const uint8_t[]){0, 0}, 2, &rsp);
	zassert_equal(rsp.status, OCA_STATUS_BAD_FORMAT);

	/* setter errno mapping */
	set_err = -ERANGE;
	call(0, ONO_GAIN, 4, 2, 1, (const uint8_t[]){0x41, 0x20, 0, 0}, 4, &rsp);
	zassert_equal(rsp.status, OCA_STATUS_PARAMETER_OUT_OF_RANGE);
	set_err = 0;
}

ZTEST(aes70_core, test_gain_get_set)
{
	struct response rsp;

	gain_value = -3.0f;
	call(0, ONO_GAIN, 4, 1, 0, NULL, 0, &rsp);
	zassert_equal(rsp.status, OCA_STATUS_OK);
	zassert_equal(rsp.count, 3);
	zassert_equal(ocp1_r_f32(&rsp.params), -3.0f);
	zassert_equal(ocp1_r_f32(&rsp.params), -10.0f);
	zassert_equal(ocp1_r_f32(&rsp.params), 10.0f);

	/* SetGain(2.5f = 0x40200000) */
	call(0, ONO_GAIN, 4, 2, 1, (const uint8_t[]){0x40, 0x20, 0, 0}, 4, &rsp);
	zassert_equal(rsp.status, OCA_STATUS_OK);
	zassert_equal(rsp.count, 0);
	zassert_equal(gain_value, 2.5f);
}

ZTEST(aes70_core, test_block_enumeration_and_path)
{
	struct response rsp;
	char str[16];

	/* Root: GetActionObjects -> one member, the Inputs block */
	call(0, OCA_ONO_ROOT_BLOCK, 3, 5, 0, NULL, 0, &rsp);
	zassert_equal(rsp.status, OCA_STATUS_OK);
	zassert_equal(ocp1_r_u16(&rsp.params), 1);
	zassert_equal(ocp1_r_u32(&rsp.params), ONO_INPUTS);
	zassert_equal(ocp1_r_u16(&rsp.params), 3);   /* class id 1.1.3 */

	/* Root: GetActionObjectsRecursive -> Inputs, Gain, State */
	call(0, OCA_ONO_ROOT_BLOCK, 3, 6, 0, NULL, 0, &rsp);
	zassert_equal(rsp.status, OCA_STATUS_OK);
	zassert_equal(ocp1_r_u16(&rsp.params), 3);
	zassert_equal(ocp1_r_u32(&rsp.params), ONO_INPUTS);
	ocp1_r_skip(&rsp.params, 2 + 6 + 2);          /* class ident */
	zassert_equal(ocp1_r_u32(&rsp.params), OCA_ONO_ROOT_BLOCK);
	zassert_equal(ocp1_r_u32(&rsp.params), ONO_GAIN);
	ocp1_r_skip(&rsp.params, 2 + 8 + 2);
	zassert_equal(ocp1_r_u32(&rsp.params), ONO_INPUTS);

	/* Gain: GetPath -> ["Inputs","Gain"], [1000,1001] */
	call(0, ONO_GAIN, 2, 13, 0, NULL, 0, &rsp);
	zassert_equal(rsp.status, OCA_STATUS_OK);
	zassert_equal(rsp.count, 2);
	zassert_equal(ocp1_r_u16(&rsp.params), 2);
	ocp1_r_str(&rsp.params, str, sizeof(str));
	zassert_str_equal(str, "Inputs");
	ocp1_r_str(&rsp.params, str, sizeof(str));
	zassert_str_equal(str, "Gain");
	zassert_equal(ocp1_r_u16(&rsp.params), 2);
	zassert_equal(ocp1_r_u32(&rsp.params), ONO_INPUTS);
	zassert_equal(ocp1_r_u32(&rsp.params), ONO_GAIN);

	/* FindActionObjectsByRoleRecursive("Ga", Substring, any class,
	 * flags ONo|Role) */
	static const uint8_t find[] = {
		0, 2, 'G', 'a',   /* SearchName */
		1,                /* Substring */
		0, 0,             /* empty class id */
		0, 9,             /* ONo | Role */
	};
	call(0, OCA_ONO_ROOT_BLOCK, 3, 18, 4, find, sizeof(find), &rsp);
	zassert_equal(rsp.status, OCA_STATUS_OK);
	zassert_equal(ocp1_r_u16(&rsp.params), 1);
	zassert_equal(ocp1_r_u32(&rsp.params), ONO_GAIN);
	ocp1_r_str(&rsp.params, str, sizeof(str));
	zassert_str_equal(str, "Gain");
	zassert_equal(ocp1_r_remaining(&rsp.params), 0);
}

ZTEST(aes70_core, test_locks)
{
	struct response rsp;

	/* session 0 takes a write lock */
	call(0, ONO_GAIN, 1, 6, 0, NULL, 0, &rsp);
	zassert_equal(rsp.status, OCA_STATUS_OK);
	call(1, ONO_GAIN, 1, 7, 0, NULL, 0, &rsp);
	zassert_equal(ocp1_r_u8(&rsp.params), 1);   /* LockNoWrite */

	/* session 1 may read but not write */
	call(1, ONO_GAIN, 4, 1, 0, NULL, 0, &rsp);
	zassert_equal(rsp.status, OCA_STATUS_OK);
	call(1, ONO_GAIN, 4, 2, 1, (const uint8_t[]){0, 0, 0, 0}, 4, &rsp);
	zassert_equal(rsp.status, OCA_STATUS_LOCKED);
	call(1, ONO_GAIN, 1, 4, 0, NULL, 0, &rsp);
	zassert_equal(rsp.status, OCA_STATUS_LOCKED);

	/* full lock blocks reads too, root methods stay available */
	call(0, ONO_GAIN, 1, 3, 0, NULL, 0, &rsp);
	zassert_equal(rsp.status, OCA_STATUS_OK);
	call(1, ONO_GAIN, 4, 1, 0, NULL, 0, &rsp);
	zassert_equal(rsp.status, OCA_STATUS_LOCKED);
	call(1, ONO_GAIN, 1, 5, 0, NULL, 0, &rsp);
	zassert_equal(rsp.status, OCA_STATUS_OK);

	/* closing the owner's session releases the lock */
	aes70_core_session_close(sess[0]);
	call(1, ONO_GAIN, 4, 2, 1, (const uint8_t[]){0, 0, 0, 0}, 4, &rsp);
	zassert_equal(rsp.status, OCA_STATUS_OK);
	sess[0] = aes70_core_session_open(cap_send, &cap[0], 3000);
}

/* Parse the single EV2 notification captured on session i. */
static void expect_ntf2(int i, uint32_t emitter, uint16_t p_level,
			uint16_t p_index, float value)
{
	struct ocp1_header h;
	struct ocp1_reader r;

	zassert_true(cap[i].len > 0, "no notification on session %d", i);
	zassert_equal(ocp1_parse_header(cap[i].buf, cap[i].len, &h), 0);
	zassert_equal(h.type, OCP1_PDU_NTF2);
	zassert_equal(h.count, 1);
	ocp1_r_init(&r, cap[i].buf + OCP1_MIN_PDU_LEN,
		    cap[i].len - OCP1_MIN_PDU_LEN);
	zassert_equal(ocp1_r_u32(&r), cap[i].len - OCP1_MIN_PDU_LEN);
	zassert_equal(ocp1_r_u32(&r), emitter);
	zassert_equal(ocp1_r_u16(&r), 1);           /* PropertyChanged 1.1 */
	zassert_equal(ocp1_r_u16(&r), 1);
	zassert_equal(ocp1_r_u8(&r), 0);            /* Event */
	zassert_equal(ocp1_r_u16(&r), p_level);
	zassert_equal(ocp1_r_u16(&r), p_index);
	zassert_equal(ocp1_r_f32(&r), value);
	zassert_equal(ocp1_r_u8(&r), 1);            /* CurrentChanged */
	zassert_equal(ocp1_r_remaining(&r), 0);
}

ZTEST(aes70_core, test_ev2_property_changed)
{
	struct response rsp;
	/* AddSubscription2(Event{1001, 1.1}, Normal, empty destination) */
	static const uint8_t sub[] = {
		0, 0, 0x03, 0xE9,  0, 1,  0, 1,   /* OcaEvent */
		1,                                /* Normal */
		0, 0,                             /* OcaNetworkAddress: null */
	};

	call(0, OCA_ONO_SUBSCRIPTION_MANAGER, 3, 8, 3, sub, sizeof(sub), &rsp);
	zassert_equal(rsp.status, OCA_STATUS_OK);

	/* nothing changed: a tick stays silent */
	cap[0].len = 0;
	aes70_core_tick(3000);
	zassert_equal(cap[0].len, 0);

	/* a change from "elsewhere" (web UI, card) is picked up by the scan */
	gain_value = 4.0f;
	aes70_core_set_dirty();
	aes70_core_tick(3100);
	expect_ntf2(0, ONO_GAIN, 4, 1, 4.0f);

	/* a Set from another controller notifies the subscriber at once */
	cap[0].len = 0;
	call(1, ONO_GAIN, 4, 2, 1, (const uint8_t[]){0x40, 0x20, 0, 0}, 4, &rsp);
	zassert_equal(rsp.status, OCA_STATUS_OK);
	expect_ntf2(0, ONO_GAIN, 4, 1, 2.5f);

	/* Lightweight delivery is refused on TCP */
	static const uint8_t sub_lw[] = {
		0, 0, 0x03, 0xEA,  0, 1,  0, 1,  2,  0, 0,
	};
	call(0, OCA_ONO_SUBSCRIPTION_MANAGER, 3, 8, 3, sub_lw, sizeof(sub_lw),
	     &rsp);
	zassert_equal(rsp.status, OCA_STATUS_NOT_IMPLEMENTED);

	/* RemoveSubscription2 */
	call(0, OCA_ONO_SUBSCRIPTION_MANAGER, 3, 9, 3, sub, sizeof(sub), &rsp);
	zassert_equal(rsp.status, OCA_STATUS_OK);
	zassert_equal(aes70_core_subscription_count(), 0);
	cap[0].len = 0;
	gain_value = 1.0f;
	aes70_core_set_dirty();
	aes70_core_tick(3200);
	zassert_equal(cap[0].len, 0);
}

ZTEST(aes70_core, test_ev1_notification_format)
{
	struct response rsp;
	struct ocp1_header h;
	struct ocp1_reader r;
	size_t n;
	const uint8_t *ctx;
	/* AddSubscription(Event{1002,1.1}, Subscriber{0x1234, 5.6},
	 * ctx "AB", Reliable, null dest) */
	static const uint8_t sub[] = {
		0, 0, 0x03, 0xEA,  0, 1,  0, 1,
		0, 0, 0x12, 0x34,  0, 5,  0, 6,
		0, 2, 'A', 'B',
		1,
		0, 0,
	};

	call(1, OCA_ONO_SUBSCRIPTION_MANAGER, 3, 1, 5, sub, sizeof(sub), &rsp);
	zassert_equal(rsp.status, OCA_STATUS_OK);

	cap[1].len = 0;
	strcpy(sensor_value, "running");
	aes70_core_set_dirty();
	aes70_core_tick(4000);

	zassert_equal(ocp1_parse_header(cap[1].buf, cap[1].len, &h), 0);
	zassert_equal(h.type, OCP1_PDU_NTF1);
	ocp1_r_init(&r, cap[1].buf + OCP1_MIN_PDU_LEN,
		    cap[1].len - OCP1_MIN_PDU_LEN);
	zassert_equal(ocp1_r_u32(&r), cap[1].len - OCP1_MIN_PDU_LEN);
	zassert_equal(ocp1_r_u32(&r), 0x1234);      /* subscriber ONo */
	zassert_equal(ocp1_r_u16(&r), 5);           /* subscriber method */
	zassert_equal(ocp1_r_u16(&r), 6);
	zassert_equal(ocp1_r_u8(&r), 2);            /* parameter count */
	ctx = ocp1_r_blob(&r, &n);
	zassert_equal(n, 2);
	zassert_mem_equal(ctx, "AB", 2);
	zassert_equal(ocp1_r_u32(&r), ONO_SENSOR);  /* event */
	zassert_equal(ocp1_r_u16(&r), 1);
	zassert_equal(ocp1_r_u16(&r), 1);
	zassert_equal(ocp1_r_u16(&r), 5);           /* property 5.1 */
	zassert_equal(ocp1_r_u16(&r), 1);
	{
		char s[16];

		ocp1_r_str(&r, s, sizeof(s));
		zassert_str_equal(s, "running");
	}
	zassert_equal(ocp1_r_u8(&r), 1);
	zassert_equal(ocp1_r_remaining(&r), 0);

	/* closing the session drops the subscription */
	aes70_core_session_close(sess[1]);
	zassert_equal(aes70_core_subscription_count(), 0);
	sess[1] = aes70_core_session_open(cap_send, &cap[1], 5000);
}

ZTEST(aes70_core, test_keepalive_supervision)
{
	uint8_t ka[OCP1_MIN_PDU_LEN + 2];
	struct ocp1_writer w;
	struct ocp1_header h;
	size_t off;

	ocp1_w_init(&w, ka, sizeof(ka));
	off = ocp1_begin_pdu(&w, OCP1_PDU_KEEPALIVE, 1);
	ocp1_w_u16(&w, 1);   /* 1 s */
	ocp1_end_pdu(&w, off);

	cap[0].len = 0;
	zassert_equal(aes70_core_session_input(sess[0], ka, w.len, 10000), 0);
	zassert_equal(sess[0]->heartbeat_ms, 1000);
	/* answered immediately with a keep-alive */
	zassert_equal(ocp1_parse_header(cap[0].buf, cap[0].len, &h), 0);
	zassert_equal(h.type, OCP1_PDU_KEEPALIVE);

	/* device keeps sending one per heartbeat interval */
	cap[0].len = 0;
	aes70_core_tick(10500);
	zassert_equal(cap[0].len, 0);
	aes70_core_tick(11100);
	zassert_true(cap[0].len > 0);

	/* three missed heartbeats -> connection considered lost */
	zassert_false(sess[0]->close_pending);
	aes70_core_tick(13500);
	zassert_true(sess[0]->close_pending);
	aes70_core_session_close(sess[0]);
	sess[0] = aes70_core_session_open(cap_send, &cap[0], 20000);
}

ZTEST(aes70_core, test_segmented_input_and_bad_sync)
{
	uint8_t pdu[64];
	struct ocp1_header h;
	size_t n = build_cmd(pdu, sizeof(pdu), 7, ONO_GAIN, 1, 5, 0, NULL, 0);

	/* byte-wise delivery: the response appears with the last byte */
	cap[0].len = 0;
	for (size_t i = 0; i < n; i++) {
		zassert_equal(aes70_core_session_input(sess[0], &pdu[i], 1,
						       30000), 0);
		if (i + 1 < n) {
			zassert_equal(cap[0].len, 0);
		}
	}
	zassert_equal(ocp1_parse_header(cap[0].buf, cap[0].len, &h), 0);
	zassert_equal(h.type, OCP1_PDU_RSP);

	/* two command PDUs in one read: answered in one Response PDU
	 * carrying both responses */
	uint8_t two[128];

	memcpy(two, pdu, n);
	memcpy(two + n, pdu, n);
	cap[0].len = 0;
	zassert_equal(aes70_core_session_input(sess[0], two, 2 * n, 30001), 0);
	zassert_equal(ocp1_parse_header(cap[0].buf, cap[0].len, &h), 0);
	zassert_equal(h.type, OCP1_PDU_RSP);
	zassert_equal(h.count, 2);
	zassert_equal(cap[0].len, 1 + h.pdu_size);

	/* a bad sync value is a protocol error: connection must close */
	pdu[0] = 0x00;
	zassert_true(aes70_core_session_input(sess[0], pdu, n, 30002) < 0);
	zassert_true(sess[0]->close_pending);
	aes70_core_session_close(sess[0]);
	sess[0] = aes70_core_session_open(cap_send, &cap[0], 40000);
}

ZTEST(aes70_core, test_no_response_command)
{
	uint8_t pdu[64];
	size_t n = build_cmd(pdu, sizeof(pdu), 9, ONO_GAIN, 4, 2, 1,
			     (const uint8_t[]){0x41, 0x20, 0, 0}, 4);

	pdu[7] = OCP1_PDU_CMD;   /* no response required */
	cap[0].len = 0;
	zassert_equal(aes70_core_session_input(sess[0], pdu, n, 50000), 0);
	zassert_equal(cap[0].len, 0);
	zassert_equal(gain_value, 10.0f);   /* 0x41200000 */
}

/* AES70 Explorer batches its whole discovery into one command PDU (~90
 * commands, 1.5 KB): larger than the per-session receive buffer, so the
 * commands must be consumed one by one as they arrive. */
ZTEST(aes70_core, test_large_batched_command_pdu)
{
	enum { N = 90 };
	static uint8_t pdu[OCP1_MIN_PDU_LEN + N * 17];
	struct ocp1_writer w;
	size_t off;
	size_t pos = 0;
	uint32_t responses = 0;

	zassert_true(sizeof(pdu) > AES70_RX_BUF_SIZE, "test PDU must exceed rx buffer");

	ocp1_w_init(&w, pdu, sizeof(pdu));
	off = ocp1_begin_pdu(&w, OCP1_PDU_CMD_RRQ, N);
	for (uint32_t i = 0; i < N; i++) {
		ocp1_w_u32(&w, 17);
		ocp1_w_u32(&w, 1000 + i);          /* handle */
		ocp1_w_u32(&w, (i & 1) ? ONO_GAIN : ONO_INPUTS);
		ocp1_w_u16(&w, 1);
		ocp1_w_u16(&w, 5);                  /* GetRole */
		ocp1_w_u8(&w, 0);
	}
	ocp1_end_pdu(&w, off);
	zassert_false(w.overflow);

	/* deliver in TCP-segment-sized chunks */
	cap[0].len = 0;
	while (pos < w.len) {
		size_t n = MIN((size_t)200, w.len - pos);

		zassert_equal(aes70_core_session_input(sess[0], pdu + pos, n,
						       60000), 0);
		pos += n;
	}

	/* walk every Response PDU in the capture */
	pos = 0;
	while (pos < cap[0].len) {
		struct ocp1_header h;
		struct ocp1_reader r;

		zassert_equal(ocp1_parse_header(cap[0].buf + pos,
						cap[0].len - pos, &h), 0);
		zassert_equal(h.type, OCP1_PDU_RSP);
		ocp1_r_init(&r, cap[0].buf + pos + OCP1_MIN_PDU_LEN,
			    h.pdu_size - OCP1_HEADER_LEN);
		for (uint16_t i = 0; i < h.count; i++) {
			char role[16];
			uint32_t size = ocp1_r_u32(&r);
			uint32_t handle = ocp1_r_u32(&r);
			uint8_t status = ocp1_r_u8(&r);
			uint8_t count = ocp1_r_u8(&r);

			zassert_equal(handle, 1000 + responses);
			zassert_equal(status, OCA_STATUS_OK);
			zassert_equal(count, 1);
			ocp1_r_str(&r, role, sizeof(role));
			zassert_str_equal(role, (responses & 1) ? "Gain" : "Inputs");
			zassert_equal(size, 10 + 2 + strlen(role));
			responses++;
		}
		zassert_false(r.err);
		pos += 1 + h.pdu_size;
	}
	zassert_equal(responses, N);
	zassert_equal(sess[0]->cmd_remaining, 0);
}
