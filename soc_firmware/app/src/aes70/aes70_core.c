/*
 * AES70 device-model core and OCP.1 session handling — see aes70_model.h.
 */

#include <string.h>
#include <errno.h>
#include <stdio.h>
#include <strings.h>

#include "aes70_model.h"
#include "../aes67_mem.h"

/* ================================================================
 * Storage
 * ================================================================ */

static struct oca_object objs[AES70_MAX_OBJECTS];
static int nobjs;

static struct oca_session sessions[AES70_MAX_SESSIONS] AES67_BIG_BSS;

/* Last value seen per notifiable property, per object (same index as
 * objs[]). Only consulted for objects somebody subscribed to. */
static struct oca_value snap[AES70_MAX_OBJECTS][OCA_NOTIFY_SLOTS] AES67_BIG_BSS;
static bool snap_valid[AES70_MAX_OBJECTS][OCA_NOTIFY_SLOTS];

static uint8_t tx_buf[AES70_TX_BUF_SIZE] AES67_BIG_BSS;
/* Scratch for one marshaled event payload. */
static uint8_t ev_buf[OCA_STR_MAX + 16];

static aes70_trace_fn trace_fn;

void aes70_core_set_trace(aes70_trace_fn fn)
{
	trace_fn = fn;
}

static volatile bool model_dirty;
static int64_t last_scan_ms;
#define SCAN_INTERVAL_MS 250

/* ================================================================
 * Class IDs
 * ================================================================ */

static const uint16_t cid_block[] = {1, 1, 3};
static const uint16_t cid_submgr[] = {1, 3, 4};

/* Properties of OcaBlock we expose generically (none notifiable). */
const struct oca_kind oca_kind_block = {
	.name = "OcaBlock",
	.class_id = cid_block,
	.class_id_len = 3,
	.class_version = 3,
	.base = OCA_BASE_BLOCK,
};

static int submgr_custom(struct oca_object *obj, struct oca_session *s,
			 uint16_t level, uint16_t index, uint8_t in_count,
			 struct ocp1_reader *in, struct ocp1_writer *out,
			 uint8_t *out_count);

const struct oca_kind oca_kind_subscription_manager = {
	.name = "OcaSubscriptionManager",
	.class_id = cid_submgr,
	.class_id_len = 3,
	.class_version = 4,
	.base = OCA_BASE_MANAGER,
	.custom = submgr_custom,
};

/* ================================================================
 * Model
 * ================================================================ */

void aes70_core_init(void)
{
	memset(objs, 0, sizeof(objs));
	nobjs = 0;
	memset(sessions, 0, sizeof(sessions));
	memset(snap_valid, 0, sizeof(snap_valid));
	model_dirty = false;
	last_scan_ms = 0;
}

int aes70_core_add_object(const struct oca_object *tmpl)
{
	if (nobjs >= AES70_MAX_OBJECTS) {
		return -ENOMEM;
	}
	if (aes70_core_find(tmpl->ono) != NULL) {
		return -EEXIST;
	}
	objs[nobjs] = *tmpl;
	objs[nobjs].lock_state = 0;
	objs[nobjs].lock_owner = NULL;
	nobjs++;
	return 0;
}

struct oca_object *aes70_core_find(uint32_t ono)
{
	for (int i = 0; i < nobjs; i++) {
		if (objs[i].ono == ono) {
			return &objs[i];
		}
	}
	return NULL;
}

struct oca_object *aes70_core_object_at(int idx)
{
	return (idx >= 0 && idx < nobjs) ? &objs[idx] : NULL;
}

int aes70_core_object_count(void)
{
	return nobjs;
}

static int obj_index(const struct oca_object *obj)
{
	return (int)(obj - objs);
}

/* Is `obj` inside block `block` (directly or nested)? */
static bool is_descendant(const struct oca_object *obj, uint32_t block,
			  bool direct)
{
	uint32_t owner = obj->owner;

	if (direct) {
		return owner == block;
	}
	for (int depth = 0; depth < 16 && owner != 0; depth++) {
		if (owner == block) {
			return true;
		}
		const struct oca_object *o = aes70_core_find(owner);

		if (o == NULL) {
			return false;
		}
		owner = o->owner;
	}
	return false;
}

/* ================================================================
 * Values
 * ================================================================ */

void aes70_core_write_value(struct ocp1_writer *w, const struct oca_value *v)
{
	switch (v->vt) {
	case OCA_VT_BOOL: ocp1_w_bool(w, v->b); break;
	case OCA_VT_U8:   ocp1_w_u8(w, v->u8); break;
	case OCA_VT_U16:  ocp1_w_u16(w, v->u16); break;
	case OCA_VT_I32:  ocp1_w_i32(w, v->i32); break;
	case OCA_VT_F32:  ocp1_w_f32(w, v->f32); break;
	case OCA_VT_STR:  ocp1_w_str(w, v->str); break;
	default: break;
	}
}

static void write_range(struct ocp1_writer *w, const struct oca_value *v)
{
	switch (v->vt) {
	case OCA_VT_U8:
		ocp1_w_u8(w, (uint8_t)v->min);
		ocp1_w_u8(w, (uint8_t)v->max);
		break;
	case OCA_VT_U16:
		ocp1_w_u16(w, (uint16_t)v->min);
		ocp1_w_u16(w, (uint16_t)v->max);
		break;
	case OCA_VT_I32:
		ocp1_w_i32(w, (int32_t)v->min);
		ocp1_w_i32(w, (int32_t)v->max);
		break;
	case OCA_VT_F32:
		ocp1_w_f32(w, v->min);
		ocp1_w_f32(w, v->max);
		break;
	default:
		break;
	}
}

static bool read_value(struct ocp1_reader *r, uint8_t vt, struct oca_value *v)
{
	memset(v, 0, sizeof(*v));
	v->vt = vt;
	switch (vt) {
	case OCA_VT_BOOL: v->b = ocp1_r_bool(r); break;
	case OCA_VT_U8:   v->u8 = ocp1_r_u8(r); break;
	case OCA_VT_U16:  v->u16 = ocp1_r_u16(r); break;
	case OCA_VT_I32:  v->i32 = ocp1_r_i32(r); break;
	case OCA_VT_F32:  v->f32 = ocp1_r_f32(r); break;
	case OCA_VT_STR:  ocp1_r_str(r, v->str, sizeof(v->str)); break;
	default: return false;
	}
	return !r->err;
}

static bool value_equal(const struct oca_value *a, const struct oca_value *b)
{
	if (a->vt != b->vt) {
		return false;
	}
	switch (a->vt) {
	case OCA_VT_BOOL: return a->b == b->b;
	case OCA_VT_U8:   return a->u8 == b->u8;
	case OCA_VT_U16:  return a->u16 == b->u16;
	case OCA_VT_I32:  return a->i32 == b->i32;
	case OCA_VT_F32:  return a->f32 == b->f32;
	case OCA_VT_STR:  return strcmp(a->str, b->str) == 0;
	default: return true;
	}
}

static int errno_to_status(int err)
{
	switch (err) {
	case 0:        return OCA_STATUS_OK;
	case -EINVAL:  return OCA_STATUS_PARAMETER_ERROR;
	case -ERANGE:  return OCA_STATUS_PARAMETER_OUT_OF_RANGE;
	case -ENOTSUP: return OCA_STATUS_NOT_IMPLEMENTED;
	case -EBUSY:   return OCA_STATUS_BUSY;
	case -ENOMEM:  return OCA_STATUS_OUT_OF_MEMORY;
	case -EACCES:  return OCA_STATUS_PERMISSION_DENIED;
	case -ENODEV:  return OCA_STATUS_DEVICE_ERROR;
	default:       return OCA_STATUS_PROCESSING_FAILED;
	}
}

/* ================================================================
 * Sessions
 * ================================================================ */

struct oca_session *aes70_core_session_open(oca_send_fn send, void *transport,
					    int64_t now_ms)
{
	for (int i = 0; i < AES70_MAX_SESSIONS; i++) {
		struct oca_session *s = &sessions[i];

		if (s->used) {
			continue;
		}
		memset(s, 0, sizeof(*s));
		s->used = true;
		s->send = send;
		s->transport = transport;
		s->last_rx_ms = now_ms;
		s->last_tx_ms = now_ms;
		return s;
	}
	return NULL;
}

void aes70_core_session_close(struct oca_session *s)
{
	if (s == NULL || !s->used) {
		return;
	}
	/* AES70-3 6.4.2: locks and subscriptions made on the connection
	 * are removed. */
	for (int i = 0; i < nobjs; i++) {
		if (objs[i].lock_owner == s) {
			objs[i].lock_owner = NULL;
			objs[i].lock_state = 0;
		}
	}
	s->used = false;
}

struct oca_session *aes70_core_session_at(int idx)
{
	return (idx >= 0 && idx < AES70_MAX_SESSIONS) ? &sessions[idx] : NULL;
}

int aes70_core_subscription_count(void)
{
	int n = 0;

	for (int i = 0; i < AES70_MAX_SESSIONS; i++) {
		if (!sessions[i].used) {
			continue;
		}
		for (int j = 0; j < AES70_MAX_SUBSCRIPTIONS; j++) {
			n += sessions[i].subs[j].used;
		}
	}
	return n;
}

static int session_send(struct oca_session *s, const uint8_t *buf, size_t len,
			int64_t now_ms)
{
	int ret = s->send(s, buf, len);

	if (ret >= 0) {
		s->last_tx_ms = now_ms;
	} else {
		s->close_pending = true;
	}
	return ret;
}

/* ================================================================
 * Locks
 * ================================================================ */

int aes70_core_check_write(struct oca_object *obj, struct oca_session *s)
{
	if (obj->lock_state != 0 && obj->lock_owner != s) {
		return OCA_STATUS_LOCKED;
	}
	return OCA_STATUS_OK;
}

static int check_read(struct oca_object *obj, struct oca_session *s)
{
	if (obj->lock_state == 2 && obj->lock_owner != s) {
		return OCA_STATUS_LOCKED;
	}
	return OCA_STATUS_OK;
}

/* ================================================================
 * Generic method handlers
 * ================================================================ */

static void write_object_identification(struct ocp1_writer *w,
					const struct oca_object *o)
{
	ocp1_w_u32(w, o->ono);
	ocp1_w_class_id(w, o->kind->class_id, o->kind->class_id_len);
	ocp1_w_u16(w, o->kind->class_version);
}

/* OcaRoot, level 1 */
static int root_method(struct oca_object *obj, struct oca_session *s,
		       uint16_t index, struct ocp1_writer *out,
		       uint8_t *out_count)
{
	switch (index) {
	case 1: /* GetClassIdentification */
		ocp1_w_class_id(out, obj->kind->class_id, obj->kind->class_id_len);
		ocp1_w_u16(out, obj->kind->class_version);
		*out_count = 1;
		return OCA_STATUS_OK;
	case 2: /* GetLockable */
		ocp1_w_bool(out, obj->lockable);
		*out_count = 1;
		return OCA_STATUS_OK;
	case 3: /* SetLockNoReadWrite */
	case 6: /* SetLockNoWrite */
		if (!obj->lockable) {
			return OCA_STATUS_NOT_IMPLEMENTED;
		}
		if (obj->lock_state != 0 && obj->lock_owner != s) {
			return OCA_STATUS_LOCKED;
		}
		obj->lock_state = (index == 3) ? 2 : 1;
		obj->lock_owner = s;
		return OCA_STATUS_OK;
	case 4: /* Unlock */
		if (!obj->lockable) {
			return OCA_STATUS_NOT_IMPLEMENTED;
		}
		if (obj->lock_state != 0 && obj->lock_owner != s) {
			return OCA_STATUS_LOCKED;
		}
		obj->lock_state = 0;
		obj->lock_owner = NULL;
		return OCA_STATUS_OK;
	case 5: /* GetRole */
		ocp1_w_str(out, obj->role);
		*out_count = 1;
		return OCA_STATUS_OK;
	case 7: /* GetLockState */
		ocp1_w_u8(out, obj->lock_state);
		*out_count = 1;
		return OCA_STATUS_OK;
	default:
		return OCA_STATUS_BAD_METHOD;
	}
}

/* GetPath: role path + ONo path from the root block's member down to
 * the object (the root block itself is implicit). */
static int write_path(const struct oca_object *obj, struct ocp1_writer *out,
		      uint8_t *out_count)
{
	const struct oca_object *chain[16];
	int n = 0;
	const struct oca_object *o = obj;

	while (o != NULL && o->ono != OCA_ONO_ROOT_BLOCK && n < 16) {
		chain[n++] = o;
		if (o->owner == 0) {
			break;
		}
		o = aes70_core_find(o->owner);
	}
	ocp1_w_u16(out, (uint16_t)n);
	for (int i = n - 1; i >= 0; i--) {
		ocp1_w_str(out, chain[i]->role);
	}
	ocp1_w_u16(out, (uint16_t)n);
	for (int i = n - 1; i >= 0; i--) {
		ocp1_w_u32(out, chain[i]->ono);
	}
	*out_count = 2;
	return OCA_STATUS_OK;
}

/* OcaWorker, level 2 */
static int worker_method(struct oca_object *obj, uint16_t index,
			 struct ocp1_writer *out, uint8_t *out_count)
{
	switch (index) {
	case 1: /* GetEnabled */
		ocp1_w_bool(out, true);
		*out_count = 1;
		return OCA_STATUS_OK;
	case 5: /* GetPorts */
		ocp1_w_u16(out, 0);
		*out_count = 1;
		return OCA_STATUS_OK;
	case 8: /* GetLabel */
		ocp1_w_str(out, obj->role);
		*out_count = 1;
		return OCA_STATUS_OK;
	case 10: /* GetOwner */
		ocp1_w_u32(out, obj->owner);
		*out_count = 1;
		return OCA_STATUS_OK;
	case 13: /* GetPath */
		return write_path(obj, out, out_count);
	case 2: case 3: case 4: case 6: case 7: case 9: case 11: case 12:
	case 14: case 15: case 16: case 17: case 18:
		return OCA_STATUS_NOT_IMPLEMENTED;
	default:
		return OCA_STATUS_BAD_METHOD;
	}
}

/* OcaAgent, level 2 */
static int agent_method(struct oca_object *obj, uint16_t index,
			struct ocp1_writer *out, uint8_t *out_count)
{
	switch (index) {
	case 1: /* GetLabel */
		ocp1_w_str(out, obj->role);
		*out_count = 1;
		return OCA_STATUS_OK;
	case 2: /* SetLabel */
		return OCA_STATUS_NOT_IMPLEMENTED;
	case 3: /* GetOwner */
		ocp1_w_u32(out, obj->owner);
		*out_count = 1;
		return OCA_STATUS_OK;
	case 4: /* GetPath */
		return write_path(obj, out, out_count);
	default:
		return OCA_STATUS_BAD_METHOD;
	}
}

static bool class_id_prefix(const struct oca_object *o, const uint16_t *pfx,
			    size_t n)
{
	if (n > o->kind->class_id_len) {
		return false;
	}
	for (size_t i = 0; i < n; i++) {
		if (o->kind->class_id[i] != pfx[i]) {
			return false;
		}
	}
	return true;
}

static bool role_matches(const char *role, const char *needle, uint8_t cmp)
{
	size_t rl = strlen(role), nl = strlen(needle);

	switch (cmp) {
	case 0: /* Exact */
		return strcmp(role, needle) == 0;
	case 1: /* Substring (starts with) */
		return nl <= rl && strncmp(role, needle, nl) == 0;
	case 2: /* Contains */
		return strstr(role, needle) != NULL;
	case 3: /* ExactCaseInsensitive */
		return strcasecmp(role, needle) == 0;
	case 4: /* SubstringCaseInsensitive */
		return nl <= rl && strncasecmp(role, needle, nl) == 0;
	case 5: { /* ContainsCaseInsensitive */
		if (nl == 0) {
			return true;
		}
		for (size_t i = 0; i + nl <= rl; i++) {
			if (strncasecmp(role + i, needle, nl) == 0) {
				return true;
			}
		}
		return false;
	}
	default:
		return false;
	}
}

/* FindActionObjectsByRole(Recursive): dynamic result record, only the
 * flagged fields are marshaled (ONo=1, ClassIdentification=2,
 * ContainerPath=4, Role=8, Label=16). */
static int block_find(const struct oca_object *blk, bool recursive,
		      struct ocp1_reader *in, struct ocp1_writer *out,
		      uint8_t *out_count)
{
	char needle[OCA_STR_MAX];
	uint16_t cid[8];
	uint16_t cid_n, flags;
	uint8_t cmp;
	size_t count_off;
	uint16_t found = 0;

	ocp1_r_str(in, needle, sizeof(needle));
	cmp = ocp1_r_u8(in);
	cid_n = ocp1_r_u16(in);
	if (cid_n > 8) {
		return OCA_STATUS_PARAMETER_ERROR;
	}
	for (int i = 0; i < cid_n; i++) {
		cid[i] = ocp1_r_u16(in);
	}
	flags = ocp1_r_u16(in);
	if (in->err) {
		return OCA_STATUS_BAD_FORMAT;
	}

	count_off = out->len;
	ocp1_w_u16(out, 0);
	for (int i = 0; i < nobjs; i++) {
		const struct oca_object *o = &objs[i];

		if (!is_descendant(o, blk->ono, !recursive) ||
		    !class_id_prefix(o, cid, cid_n) ||
		    !role_matches(o->role, needle, cmp)) {
			continue;
		}
		if (flags & 1) {
			ocp1_w_u32(out, o->ono);
		}
		if (flags & 2) {
			ocp1_w_class_id(out, o->kind->class_id,
					o->kind->class_id_len);
			ocp1_w_u16(out, o->kind->class_version);
		}
		if (flags & 4) {
			/* ContainerPath: ONos from the root block's member
			 * down to the owner. */
			uint32_t chain[16];
			int n = 0;
			const struct oca_object *c = aes70_core_find(o->owner);

			while (c != NULL && c->ono != OCA_ONO_ROOT_BLOCK &&
			       n < 16) {
				chain[n++] = c->ono;
				c = aes70_core_find(c->owner);
			}
			ocp1_w_u16(out, (uint16_t)n);
			for (int k = n - 1; k >= 0; k--) {
				ocp1_w_u32(out, chain[k]);
			}
		}
		if (flags & 8) {
			ocp1_w_str(out, o->role);
		}
		if (flags & 16) {
			ocp1_w_str(out, o->role);
		}
		found++;
	}
	ocp1_w_patch_u16(out, count_off, found);
	*out_count = 1;
	return OCA_STATUS_OK;
}

/* OcaBlock, level 3 */
static int block_method(struct oca_object *obj, uint16_t index,
			struct ocp1_reader *in, struct ocp1_writer *out,
			uint8_t *out_count)
{
	switch (index) {
	case 1: /* GetType (deprecated): 0 = no block type */
		ocp1_w_u32(out, 0);
		*out_count = 1;
		return OCA_STATUS_OK;
	case 5: { /* GetActionObjects */
		size_t off = out->len;
		uint16_t n = 0;

		ocp1_w_u16(out, 0);
		for (int i = 0; i < nobjs; i++) {
			if (objs[i].owner == obj->ono) {
				write_object_identification(out, &objs[i]);
				n++;
			}
		}
		ocp1_w_patch_u16(out, off, n);
		*out_count = 1;
		return OCA_STATUS_OK;
	}
	case 6: { /* GetActionObjectsRecursive -> OcaList<OcaBlockMember> */
		size_t off = out->len;
		uint16_t n = 0;

		ocp1_w_u16(out, 0);
		for (int i = 0; i < nobjs; i++) {
			if (is_descendant(&objs[i], obj->ono, false)) {
				write_object_identification(out, &objs[i]);
				ocp1_w_u32(out, objs[i].owner);
				n++;
			}
		}
		ocp1_w_patch_u16(out, off, n);
		*out_count = 1;
		return OCA_STATUS_OK;
	}
	case 9:  /* GetSignalPaths: empty map */
	case 10: /* GetSignalPathsRecursive */
	case 16: /* GetONoMap: empty map */
	case 29: /* GetDatasetObjects: empty list */
	case 30: /* GetDatasetObjectsRecursive */
		ocp1_w_u16(out, 0);
		*out_count = 1;
		return OCA_STATUS_OK;
	case 15: { /* GetGlobalType: null identifier */
		static const uint8_t org[3] = {0, 0, 0};

		ocp1_w_raw(out, org, 3);
		ocp1_w_u32(out, 0);
		*out_count = 1;
		return OCA_STATUS_OK;
	}
	case 17: /* FindActionObjectsByRole */
		return block_find(obj, false, in, out, out_count);
	case 18: /* FindActionObjectsByRoleRecursive */
		return block_find(obj, true, in, out, out_count);
	case 21: /* GetConfigurability: fixed block */
		ocp1_w_u16(out, 0);
		*out_count = 1;
		return OCA_STATUS_OK;
	case 22: /* GetMostRecentParamDatasetONo */
	case 33: /* GetBlockFactoryONo */
		ocp1_w_u32(out, 0);
		*out_count = 1;
		return OCA_STATUS_OK;
	default:
		if (index >= 2 && index <= 33) {
			return OCA_STATUS_NOT_IMPLEMENTED;
		}
		return OCA_STATUS_BAD_METHOD;
	}
}

static const struct oca_prop *find_prop(const struct oca_kind *k,
					uint16_t level, uint16_t method,
					bool *is_set)
{
	for (int i = 0; i < k->nprops; i++) {
		const struct oca_prop *p = &k->props[i];

		if (p->level != level) {
			continue;
		}
		if (p->get_method == method && method != 0) {
			*is_set = false;
			return p;
		}
		if (p->set_method == method && method != 0) {
			*is_set = true;
			return p;
		}
	}
	return NULL;
}

/* Property Get/Set from the kind's table. */
static int prop_method(struct oca_object *obj, struct oca_session *s,
		       uint16_t level, uint16_t index, struct ocp1_reader *in,
		       struct ocp1_writer *out, uint8_t *out_count,
		       int64_t now_ms)
{
	bool is_set;
	const struct oca_prop *p = find_prop(obj->kind, level, index, &is_set);
	struct oca_value v;
	int ret;

	if (p == NULL) {
		return OCA_STATUS_BAD_METHOD;
	}
	if (!is_set) {
		if (obj->get == NULL) {
			return OCA_STATUS_NOT_IMPLEMENTED;
		}
		memset(&v, 0, sizeof(v));
		v.vt = p->vt;
		ret = obj->get(obj, (uint8_t)(p - obj->kind->props), &v);
		if (ret < 0) {
			return errno_to_status(ret);
		}
		aes70_core_write_value(out, &v);
		if (p->flags & OCA_PF_RANGE) {
			write_range(out, &v);
			*out_count = 3;
		} else {
			*out_count = 1;
		}
		return OCA_STATUS_OK;
	}

	if (obj->set == NULL) {
		return OCA_STATUS_NOT_IMPLEMENTED;
	}
	ret = aes70_core_check_write(obj, s);
	if (ret != OCA_STATUS_OK) {
		return ret;
	}
	if (!read_value(in, p->vt, &v)) {
		return OCA_STATUS_BAD_FORMAT;
	}
	ret = obj->set(obj, (uint8_t)(p - obj->kind->props), &v);
	if (ret < 0) {
		return errno_to_status(ret);
	}
	aes70_core_refresh_object(obj->ono, now_ms);
	return OCA_STATUS_OK;
}

static int dispatch(struct oca_session *s, uint32_t ono, uint16_t level,
		    uint16_t index, uint8_t in_count, struct ocp1_reader *in,
		    struct ocp1_writer *out, uint8_t *out_count, int64_t now_ms)
{
	struct oca_object *obj = aes70_core_find(ono);
	int ret;

	*out_count = 0;
	if (obj == NULL) {
		return OCA_STATUS_BAD_ONO;
	}
	if (level == 1) {
		return root_method(obj, s, index, out, out_count);
	}
	ret = check_read(obj, s);
	if (ret != OCA_STATUS_OK) {
		return ret;
	}
	if (level == 2) {
		switch (obj->kind->base) {
		case OCA_BASE_WORKER:
		case OCA_BASE_BLOCK:
			return worker_method(obj, index, out, out_count);
		case OCA_BASE_AGENT:
			return agent_method(obj, index, out, out_count);
		default:
			return OCA_STATUS_BAD_METHOD;
		}
	}
	if (level == 3 && obj->kind->base == OCA_BASE_BLOCK) {
		return block_method(obj, index, in, out, out_count);
	}
	if (obj->kind->custom != NULL) {
		ret = obj->kind->custom(obj, s, level, index, in_count, in, out,
					out_count);
		if (ret != OCA_METHOD_UNHANDLED) {
			return ret;
		}
	}
	return prop_method(obj, s, level, index, in, out, out_count, now_ms);
}

/* ================================================================
 * Subscriptions
 * ================================================================ */

struct sub_key {
	uint32_t emitter;
	uint16_t ev_level, ev_index;
	bool prop_specific;
	uint16_t p_level, p_index;
	bool ev1;
	uint32_t sub_ono;
	uint16_t sub_level, sub_index;
};

static bool sub_matches_key(const struct oca_subscription *x,
			    const struct sub_key *k)
{
	if (!x->used || x->emitter != k->emitter || x->ev_level != k->ev_level ||
	    x->ev_index != k->ev_index || x->prop_specific != k->prop_specific ||
	    x->ev1 != k->ev1) {
		return false;
	}
	if (k->prop_specific &&
	    (x->p_level != k->p_level || x->p_index != k->p_index)) {
		return false;
	}
	if (k->ev1 && (x->sub_ono != k->sub_ono || x->sub_level != k->sub_level ||
		       x->sub_index != k->sub_index)) {
		return false;
	}
	return true;
}

/* Take a fresh snapshot of an object's notifiable properties so a new
 * subscription does not fire on the first scan. */
static void snapshot_object(struct oca_object *obj)
{
	int idx = obj_index(obj);
	int slot = 0;

	for (int i = 0; i < obj->kind->nprops && slot < OCA_NOTIFY_SLOTS; i++) {
		const struct oca_prop *p = &obj->kind->props[i];

		if (!(p->flags & OCA_PF_NOTIFY)) {
			continue;
		}
		memset(&snap[idx][slot], 0, sizeof(snap[idx][slot]));
		snap[idx][slot].vt = p->vt;
		snap_valid[idx][slot] = obj->get != NULL &&
			obj->get(obj, (uint8_t)i, &snap[idx][slot]) == 0;
		slot++;
	}
}

static int add_subscription(struct oca_session *s, const struct sub_key *k,
			    const uint8_t *ctx, size_t ctx_len, uint8_t mode)
{
	struct oca_object *obj = aes70_core_find(k->emitter);

	if (obj == NULL) {
		return OCA_STATUS_BAD_ONO;
	}
	/* Only OcaRoot.PropertyChanged (1.1) is emitted by this device. */
	if (k->ev_level != 1 || k->ev_index != 1) {
		return OCA_STATUS_PARAMETER_ERROR;
	}
	/* Lightweight (UDP) delivery is not offered on the TCP transport. */
	if (mode != 1) {
		return OCA_STATUS_NOT_IMPLEMENTED;
	}
	if (ctx_len > OCA_EV1_CTX_MAX) {
		return OCA_STATUS_PARAMETER_OUT_OF_RANGE;
	}
	for (int i = 0; i < AES70_MAX_SUBSCRIPTIONS; i++) {
		if (sub_matches_key(&s->subs[i], k)) {
			return OCA_STATUS_OK;   /* idempotent */
		}
	}
	for (int i = 0; i < AES70_MAX_SUBSCRIPTIONS; i++) {
		struct oca_subscription *x = &s->subs[i];

		if (x->used) {
			continue;
		}
		memset(x, 0, sizeof(*x));
		x->used = true;
		x->emitter = k->emitter;
		x->ev_level = k->ev_level;
		x->ev_index = k->ev_index;
		x->prop_specific = k->prop_specific;
		x->p_level = k->p_level;
		x->p_index = k->p_index;
		x->ev1 = k->ev1;
		x->sub_ono = k->sub_ono;
		x->sub_level = k->sub_level;
		x->sub_index = k->sub_index;
		x->ctx_len = (uint8_t)ctx_len;
		if (ctx_len) {
			memcpy(x->ctx, ctx, ctx_len);
		}
		snapshot_object(obj);
		return OCA_STATUS_OK;
	}
	return OCA_STATUS_OUT_OF_MEMORY;
}

static int remove_subscription(struct oca_session *s, const struct sub_key *k)
{
	for (int i = 0; i < AES70_MAX_SUBSCRIPTIONS; i++) {
		if (sub_matches_key(&s->subs[i], k)) {
			s->subs[i].used = false;
			return OCA_STATUS_OK;
		}
	}
	return OCA_STATUS_INVALID_REQUEST;
}

static void read_event(struct ocp1_reader *in, struct sub_key *k)
{
	k->emitter = ocp1_r_u32(in);
	k->ev_level = ocp1_r_u16(in);
	k->ev_index = ocp1_r_u16(in);
}

static void read_subscriber(struct ocp1_reader *in, struct sub_key *k)
{
	k->sub_ono = ocp1_r_u32(in);
	k->sub_level = ocp1_r_u16(in);
	k->sub_index = ocp1_r_u16(in);
}

static void skip_destination(struct ocp1_reader *in)
{
	size_t n;

	(void)ocp1_r_blob(in, &n);
}

static int submgr_custom(struct oca_object *obj, struct oca_session *s,
			 uint16_t level, uint16_t index, uint8_t in_count,
			 struct ocp1_reader *in, struct ocp1_writer *out,
			 uint8_t *out_count)
{
	struct sub_key k;
	const uint8_t *ctx = NULL;
	size_t ctx_len = 0;
	uint8_t mode;
	int ret;

	(void)obj;
	(void)in_count;
	memset(&k, 0, sizeof(k));
	if (level != 3) {
		return OCA_STATUS_BAD_METHOD;
	}

	switch (index) {
	case 1: /* AddSubscription (EV1) */
		read_event(in, &k);
		read_subscriber(in, &k);
		ctx = ocp1_r_blob(in, &ctx_len);
		mode = ocp1_r_u8(in);
		skip_destination(in);
		k.ev1 = true;
		if (in->err) {
			return OCA_STATUS_BAD_FORMAT;
		}
		return add_subscription(s, &k, ctx, ctx_len, mode);
	case 2: /* RemoveSubscription (EV1) */
		read_event(in, &k);
		read_subscriber(in, &k);
		k.ev1 = true;
		if (in->err) {
			return OCA_STATUS_BAD_FORMAT;
		}
		return remove_subscription(s, &k);
	case 5: /* AddPropertyChangeSubscription (EV1) */
		k.emitter = ocp1_r_u32(in);
		k.p_level = ocp1_r_u16(in);
		k.p_index = ocp1_r_u16(in);
		read_subscriber(in, &k);
		ctx = ocp1_r_blob(in, &ctx_len);
		mode = ocp1_r_u8(in);
		skip_destination(in);
		k.ev1 = true;
		k.prop_specific = true;
		k.ev_level = 1;
		k.ev_index = 1;
		if (in->err) {
			return OCA_STATUS_BAD_FORMAT;
		}
		return add_subscription(s, &k, ctx, ctx_len, mode);
	case 6: /* RemovePropertyChangeSubscription (EV1) */
		k.emitter = ocp1_r_u32(in);
		k.p_level = ocp1_r_u16(in);
		k.p_index = ocp1_r_u16(in);
		read_subscriber(in, &k);
		k.ev1 = true;
		k.prop_specific = true;
		k.ev_level = 1;
		k.ev_index = 1;
		if (in->err) {
			return OCA_STATUS_BAD_FORMAT;
		}
		return remove_subscription(s, &k);
	case 7: /* GetMaximumSubscriberContextLength */
		ocp1_w_u16(out, OCA_EV1_CTX_MAX);
		*out_count = 1;
		return OCA_STATUS_OK;
	case 8: /* AddSubscription2 */
		read_event(in, &k);
		mode = ocp1_r_u8(in);
		skip_destination(in);
		if (in->err) {
			return OCA_STATUS_BAD_FORMAT;
		}
		return add_subscription(s, &k, NULL, 0, mode);
	case 9: /* RemoveSubscription2 */
		read_event(in, &k);
		if (in->err) {
			return OCA_STATUS_BAD_FORMAT;
		}
		return remove_subscription(s, &k);
	case 10: /* AddPropertyChangeSubscription2 */
		k.emitter = ocp1_r_u32(in);
		k.p_level = ocp1_r_u16(in);
		k.p_index = ocp1_r_u16(in);
		mode = ocp1_r_u8(in);
		skip_destination(in);
		k.prop_specific = true;
		k.ev_level = 1;
		k.ev_index = 1;
		if (in->err) {
			return OCA_STATUS_BAD_FORMAT;
		}
		return add_subscription(s, &k, NULL, 0, mode);
	case 11: /* RemovePropertyChangeSubscription2 */
		k.emitter = ocp1_r_u32(in);
		k.p_level = ocp1_r_u16(in);
		k.p_index = ocp1_r_u16(in);
		k.prop_specific = true;
		k.ev_level = 1;
		k.ev_index = 1;
		if (in->err) {
			return OCA_STATUS_BAD_FORMAT;
		}
		return remove_subscription(s, &k);
	case 12:   /* AddSubscription2List */
	case 13: { /* RemoveSubscription2List */
		uint16_t n = ocp1_r_u16(in);
		size_t items = in->pos;

		/* The mode/destination follow the list; peek past it. */
		ocp1_r_skip(in, (size_t)n * 8);
		mode = ocp1_r_u8(in);
		skip_destination(in);
		if (in->err) {
			return OCA_STATUS_BAD_FORMAT;
		}
		in->pos = items;
		if (index == 12) {
			ocp1_w_u16(out, n);
		}
		ret = OCA_STATUS_OK;
		for (uint16_t i = 0; i < n; i++) {
			int st;

			memset(&k, 0, sizeof(k));
			read_event(in, &k);
			st = (index == 12) ? add_subscription(s, &k, NULL, 0, mode)
					   : remove_subscription(s, &k);
			if (index == 12) {
				ocp1_w_u8(out, (uint8_t)st);
			}
			if (st != OCA_STATUS_OK) {
				ret = OCA_STATUS_PARTIALLY_SUCCEEDED;
			}
		}
		*out_count = (index == 12) ? 1 : 0;
		return ret;
	}
	case 14:   /* AddPropertyChangeSubscription2List */
	case 15: { /* RemovePropertyChangeSubscription2List */
		uint16_t ne = ocp1_r_u16(in);
		size_t emitters = in->pos;
		uint16_t np;
		size_t props;

		ocp1_r_skip(in, (size_t)ne * 4);
		np = ocp1_r_u16(in);
		props = in->pos;
		ocp1_r_skip(in, (size_t)np * 4);
		mode = ocp1_r_u8(in);
		skip_destination(in);
		if (in->err || ne != np) {
			return OCA_STATUS_BAD_FORMAT;
		}
		if (index == 14) {
			ocp1_w_u16(out, ne);
		}
		ret = OCA_STATUS_OK;
		for (uint16_t i = 0; i < ne; i++) {
			int st;

			memset(&k, 0, sizeof(k));
			in->pos = emitters + (size_t)i * 4;
			k.emitter = ocp1_r_u32(in);
			in->pos = props + (size_t)i * 4;
			k.p_level = ocp1_r_u16(in);
			k.p_index = ocp1_r_u16(in);
			k.prop_specific = true;
			k.ev_level = 1;
			k.ev_index = 1;
			st = (index == 14) ? add_subscription(s, &k, NULL, 0, mode)
					   : remove_subscription(s, &k);
			if (index == 14) {
				ocp1_w_u8(out, (uint8_t)st);
			}
			if (st != OCA_STATUS_OK) {
				ret = OCA_STATUS_PARTIALLY_SUCCEEDED;
			}
		}
		*out_count = (index == 14) ? 1 : 0;
		return ret;
	}
	case 3: /* DisableNotifications */
	case 4: /* ReEnableNotifications */
		return OCA_STATUS_NOT_IMPLEMENTED;
	default:
		return OCA_STATUS_BAD_METHOD;
	}
}

/* ================================================================
 * Notifications
 * ================================================================ */

static void send_notification(struct oca_session *s,
			      const struct oca_subscription *x,
			      uint32_t emitter, uint16_t ev_level,
			      uint16_t ev_index, const uint8_t *data,
			      size_t len, int64_t now_ms)
{
	uint8_t buf[OCP1_MIN_PDU_LEN + 32 + OCA_EV1_CTX_MAX + sizeof(ev_buf)];
	struct ocp1_writer w;
	size_t size_off, nsize_off;

	ocp1_w_init(&w, buf, sizeof(buf));
	if (!x->ev1) {
		size_off = ocp1_begin_pdu(&w, OCP1_PDU_NTF2, 1);
		nsize_off = w.len;
		ocp1_w_u32(&w, 0);              /* NotificationSize */
		ocp1_w_u32(&w, emitter);
		ocp1_w_u16(&w, ev_level);
		ocp1_w_u16(&w, ev_index);
		ocp1_w_u8(&w, 0);               /* Event (not Exception) */
		ocp1_w_raw(&w, data, len);
		ocp1_w_patch_u32(&w, nsize_off, (uint32_t)(w.len - nsize_off));
	} else {
		size_off = ocp1_begin_pdu(&w, OCP1_PDU_NTF1, 1);
		nsize_off = w.len;
		ocp1_w_u32(&w, 0);              /* NotificationSize */
		ocp1_w_u32(&w, x->sub_ono);     /* TargetONo */
		ocp1_w_u16(&w, x->sub_level);   /* MethodID */
		ocp1_w_u16(&w, x->sub_index);
		ocp1_w_u8(&w, 2);               /* ParameterCount */
		ocp1_w_blob(&w, x->ctx, x->ctx_len);
		ocp1_w_u32(&w, emitter);        /* Ocp1EventData1.Event */
		ocp1_w_u16(&w, ev_level);
		ocp1_w_u16(&w, ev_index);
		ocp1_w_raw(&w, data, len);
		ocp1_w_patch_u32(&w, nsize_off, (uint32_t)(w.len - nsize_off));
	}
	ocp1_end_pdu(&w, size_off);
	if (w.overflow) {
		return;
	}
	s->notifications++;
	session_send(s, buf, w.len, now_ms);
}

void aes70_core_emit_event(uint32_t emitter, uint16_t level, uint16_t index,
			   const uint8_t *data, size_t len, int64_t now_ms)
{
	for (int i = 0; i < AES70_MAX_SESSIONS; i++) {
		struct oca_session *s = &sessions[i];

		if (!s->used || s->close_pending) {
			continue;
		}
		for (int j = 0; j < AES70_MAX_SUBSCRIPTIONS; j++) {
			const struct oca_subscription *x = &s->subs[j];

			if (x->used && !x->prop_specific &&
			    x->emitter == emitter && x->ev_level == level &&
			    x->ev_index == index) {
				send_notification(s, x, emitter, level, index,
						  data, len, now_ms);
			}
		}
	}
}

/* PropertyChanged for one property: generic subscriptions on the
 * object plus the property-specific ones. */
static void emit_property_changed(const struct oca_object *obj,
				  const struct oca_prop *p,
				  const struct oca_value *v, int64_t now_ms)
{
	struct ocp1_writer w;

	ocp1_w_init(&w, ev_buf, sizeof(ev_buf));
	ocp1_w_u16(&w, p->level);
	ocp1_w_u16(&w, p->index);
	aes70_core_write_value(&w, v);
	ocp1_w_u8(&w, 1);   /* OcaPropertyChangeType.CurrentChanged */
	if (w.overflow) {
		return;
	}

	for (int i = 0; i < AES70_MAX_SESSIONS; i++) {
		struct oca_session *s = &sessions[i];

		if (!s->used || s->close_pending) {
			continue;
		}
		for (int j = 0; j < AES70_MAX_SUBSCRIPTIONS; j++) {
			const struct oca_subscription *x = &s->subs[j];

			if (!x->used || x->emitter != obj->ono ||
			    x->ev_level != 1 || x->ev_index != 1) {
				continue;
			}
			if (x->prop_specific &&
			    (x->p_level != p->level || x->p_index != p->index)) {
				continue;
			}
			send_notification(s, x, obj->ono, 1, 1, ev_buf, w.len,
					  now_ms);
		}
	}
}

static bool object_subscribed(uint32_t ono)
{
	for (int i = 0; i < AES70_MAX_SESSIONS; i++) {
		const struct oca_session *s = &sessions[i];

		if (!s->used) {
			continue;
		}
		for (int j = 0; j < AES70_MAX_SUBSCRIPTIONS; j++) {
			if (s->subs[j].used && s->subs[j].emitter == ono) {
				return true;
			}
		}
	}
	return false;
}

static void scan_object(struct oca_object *obj, int64_t now_ms)
{
	int idx = obj_index(obj);
	int slot = 0;

	if (obj->get == NULL) {
		return;
	}
	for (int i = 0; i < obj->kind->nprops && slot < OCA_NOTIFY_SLOTS; i++) {
		const struct oca_prop *p = &obj->kind->props[i];
		struct oca_value v;

		if (!(p->flags & OCA_PF_NOTIFY)) {
			continue;
		}
		memset(&v, 0, sizeof(v));
		v.vt = p->vt;
		if (obj->get(obj, (uint8_t)i, &v) == 0) {
			if (!snap_valid[idx][slot] ||
			    !value_equal(&snap[idx][slot], &v)) {
				snap[idx][slot] = v;
				snap_valid[idx][slot] = true;
				emit_property_changed(obj, p, &v, now_ms);
			}
		}
		slot++;
	}
}

void aes70_core_refresh_object(uint32_t ono, int64_t now_ms)
{
	struct oca_object *obj = aes70_core_find(ono);

	if (obj != NULL && object_subscribed(ono)) {
		scan_object(obj, now_ms);
	}
}

void aes70_core_set_dirty(void)
{
	model_dirty = true;
}

static void scan_all(int64_t now_ms)
{
	for (int i = 0; i < nobjs; i++) {
		if (object_subscribed(objs[i].ono)) {
			scan_object(&objs[i], now_ms);
		}
	}
}

/* ================================================================
 * Keep-alive
 * ================================================================ */

static void send_keepalive(struct oca_session *s, int64_t now_ms)
{
	uint8_t buf[OCP1_MIN_PDU_LEN + 4];
	struct ocp1_writer w;
	size_t off;

	ocp1_w_init(&w, buf, sizeof(buf));
	off = ocp1_begin_pdu(&w, OCP1_PDU_KEEPALIVE, 1);
	if (s->heartbeat_ms % 1000 == 0 && s->heartbeat_ms / 1000 <= 0xFFFF) {
		ocp1_w_u16(&w, (uint16_t)(s->heartbeat_ms / 1000));
	} else {
		ocp1_w_u32(&w, s->heartbeat_ms);
	}
	ocp1_end_pdu(&w, off);
	session_send(s, buf, w.len, now_ms);
}

static void handle_keepalive(struct oca_session *s, const uint8_t *payload,
			     size_t len, int64_t now_ms)
{
	struct ocp1_reader r;
	uint32_t hb;

	ocp1_r_init(&r, payload, len);
	if (len == 2) {
		hb = (uint32_t)ocp1_r_u16(&r) * 1000u;
	} else if (len == 4) {
		hb = ocp1_r_u32(&r);
	} else {
		return;
	}
	s->heartbeat_ms = hb;
	/* Answer right away so the controller's own timer is satisfied. */
	if (hb > 0) {
		send_keepalive(s, now_ms);
	}
}

void aes70_core_tick(int64_t now_ms)
{
	for (int i = 0; i < AES70_MAX_SESSIONS; i++) {
		struct oca_session *s = &sessions[i];

		if (!s->used || s->close_pending || s->heartbeat_ms == 0) {
			continue;
		}
		if (now_ms - s->last_rx_ms > (int64_t)s->heartbeat_ms * 3) {
			s->close_pending = true;
			continue;
		}
		if (now_ms - s->last_tx_ms >= (int64_t)s->heartbeat_ms) {
			send_keepalive(s, now_ms);
		}
	}

	if (model_dirty || now_ms - last_scan_ms >= SCAN_INTERVAL_MS) {
		model_dirty = false;
		last_scan_ms = now_ms;
		scan_all(now_ms);
	}
}

/* ================================================================
 * Commands
 *
 * Command PDUs are consumed incrementally: a controller may batch its
 * whole discovery into one PDU (AES70 Explorer sends ~90 commands,
 * 1.5 KB, in a single Ocp1CommandPdu), so the session only ever needs
 * room for the largest single command, not for the whole PDU. The
 * responses of the commands handled in one go are batched into one
 * Response PDU; a response that does not fit next to the earlier ones
 * starts a new PDU, and only a response too big for an empty buffer is
 * reported as BufferOverflow.
 * ================================================================ */

/* Ocp1Command: size(4) handle(4) ono(4) level(2) index(2) count(1) */
#define OCP1_CMD_FIXED_LEN 17
/* Ocp1Response: size(4) handle(4) status(1) count(1) */
#define OCP1_RSP_FIXED_LEN 10

struct rsp_batch {
	struct ocp1_writer w;
	size_t size_off;
	uint16_t count;
	bool respond;
};

static void batch_begin(struct rsp_batch *b, bool respond)
{
	ocp1_w_init(&b->w, tx_buf, sizeof(tx_buf));
	b->count = 0;
	b->respond = respond;
	b->size_off = ocp1_begin_pdu(&b->w, OCP1_PDU_RSP, 0);
}

static void batch_flush(struct oca_session *s, struct rsp_batch *b,
			int64_t now_ms)
{
	if (b->respond && b->count > 0) {
		ocp1_patch_count(&b->w, b->size_off, b->count);
		ocp1_end_pdu(&b->w, b->size_off);
		session_send(s, tx_buf, b->w.len, now_ms);
	}
	batch_begin(b, b->respond);
}

/* Run one complete command (cmd points at its CommandSize field) and
 * append its response to the batch. */
static void run_command(struct oca_session *s, struct rsp_batch *b,
			const uint8_t *cmd, uint32_t cmd_size, int64_t now_ms)
{
	struct ocp1_reader hdr, in;
	uint32_t handle, ono;
	uint16_t level, index;
	uint8_t in_count;

	ocp1_r_init(&hdr, cmd, OCP1_CMD_FIXED_LEN);
	(void)ocp1_r_u32(&hdr);
	handle = ocp1_r_u32(&hdr);
	ono = ocp1_r_u32(&hdr);
	level = ocp1_r_u16(&hdr);
	index = ocp1_r_u16(&hdr);
	in_count = ocp1_r_u8(&hdr);
	s->commands++;

	for (int attempt = 0; attempt < 2; attempt++) {
		struct ocp1_writer *w = &b->w;
		size_t rsp_off = w->len, params_off;
		uint8_t out_count = 0;
		int status;

		ocp1_r_init(&in, cmd + OCP1_CMD_FIXED_LEN,
			    cmd_size - OCP1_CMD_FIXED_LEN);
		ocp1_w_u32(w, 0);          /* ResponseSize */
		ocp1_w_u32(w, handle);
		ocp1_w_u8(w, 0);           /* StatusCode */
		ocp1_w_u8(w, 0);           /* ParameterCount */
		params_off = w->len;

		status = dispatch(s, ono, level, index, in_count, &in, w,
				  &out_count, now_ms);
		if (w->overflow) {
			if (attempt == 0 && b->count > 0) {
				/* no room next to the earlier responses:
				 * send those and retry in a fresh PDU */
				w->len = rsp_off;
				w->overflow = false;
				batch_flush(s, b, now_ms);
				continue;
			}
			status = OCA_STATUS_BUFFER_OVERFLOW;
		}
		if (status != OCA_STATUS_OK) {
			w->len = params_off;
			w->overflow = false;
			out_count = 0;
		}
		if (trace_fn != NULL) {
			trace_fn(ono, level, index, status, out_count);
		}
		if (!b->respond) {
			w->len = rsp_off;
			return;
		}
		w->buf[rsp_off + 8] = (uint8_t)status;
		w->buf[rsp_off + 9] = out_count;
		ocp1_w_patch_u32(w, rsp_off, (uint32_t)(w->len - rsp_off));
		b->count++;
		return;
	}
}

int aes70_core_session_input(struct oca_session *s, const uint8_t *data,
			     size_t len, int64_t now_ms)
{
	struct rsp_batch batch;
	bool batch_open = false;

	if (!s->used) {
		return -ENOTCONN;
	}
	if (len > sizeof(s->rx) - s->rx_len) {
		s->close_pending = true;
		return -EMSGSIZE;
	}
	memcpy(&s->rx[s->rx_len], data, len);
	s->rx_len += len;
	s->last_rx_ms = now_ms;

	for (;;) {
		size_t consumed = 0;

		if (s->cmd_remaining > 0) {
			/* Inside a command PDU: take the next command once
			 * it is complete. */
			uint32_t cmd_size;

			if (s->rx_len < 4) {
				break;
			}
			cmd_size = ((uint32_t)s->rx[0] << 24) |
				   ((uint32_t)s->rx[1] << 16) |
				   ((uint32_t)s->rx[2] << 8) | s->rx[3];
			if (cmd_size < OCP1_CMD_FIXED_LEN ||
			    cmd_size > s->cmd_remaining ||
			    cmd_size > sizeof(s->rx)) {
				s->close_pending = true;
				return cmd_size > sizeof(s->rx) ? -EMSGSIZE
								 : -EBADMSG;
			}
			if (s->rx_len < cmd_size) {
				break;
			}
			if (!batch_open) {
				batch_begin(&batch, s->cmd_respond);
				batch_open = true;
			} else if (batch.respond != s->cmd_respond) {
				batch_flush(s, &batch, now_ms);
				batch.respond = s->cmd_respond;
			}
			run_command(s, &batch, s->rx, cmd_size, now_ms);
			s->cmd_remaining -= cmd_size;
			consumed = cmd_size;
		} else {
			struct ocp1_header h;
			int ret = ocp1_parse_header(s->rx, s->rx_len, &h);

			if (ret == -EAGAIN) {
				break;
			}
			if (ret < 0) {
				s->close_pending = true;
				return ret;
			}
			size_t body = h.pdu_size - OCP1_HEADER_LEN;

			if (h.type == OCP1_PDU_CMD || h.type == OCP1_PDU_CMD_RRQ) {
				/* header only; the commands follow one by one */
				s->cmd_remaining = body;
				s->cmd_respond = (h.type == OCP1_PDU_CMD_RRQ);
				consumed = OCP1_MIN_PDU_LEN;
			} else {
				size_t total = 1 + h.pdu_size;

				if (total > sizeof(s->rx)) {
					s->close_pending = true;
					return -EMSGSIZE;
				}
				if (total > s->rx_len) {
					break;
				}
				if (h.type == OCP1_PDU_KEEPALIVE) {
					handle_keepalive(s, s->rx + OCP1_MIN_PDU_LEN,
							 body, now_ms);
				}
				/* responses / notifications from a controller:
				 * nothing to do */
				consumed = total;
			}
		}

		s->rx_len -= consumed;
		if (s->rx_len) {
			memmove(s->rx, s->rx + consumed, s->rx_len);
		}
		if (s->close_pending) {
			return -ECONNRESET;
		}
	}

	if (batch_open) {
		batch_flush(s, &batch, now_ms);
	}
	return s->close_pending ? -ECONNRESET : 0;
}
