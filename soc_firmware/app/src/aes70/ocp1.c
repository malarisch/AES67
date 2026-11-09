/*
 * OCP.1 wire-format helpers — see ocp1.h.
 */

#include <string.h>
#include <errno.h>

#include "ocp1.h"

/* ================================================================
 * Writer
 * ================================================================ */

void ocp1_w_init(struct ocp1_writer *w, uint8_t *buf, size_t cap)
{
	w->buf = buf;
	w->cap = cap;
	w->len = 0;
	w->overflow = false;
}

static bool w_fits(struct ocp1_writer *w, size_t n)
{
	if (w->overflow || w->len + n > w->cap) {
		w->overflow = true;
		return false;
	}
	return true;
}

void ocp1_w_u8(struct ocp1_writer *w, uint8_t v)
{
	if (w_fits(w, 1)) {
		w->buf[w->len++] = v;
	}
}

void ocp1_w_u16(struct ocp1_writer *w, uint16_t v)
{
	if (w_fits(w, 2)) {
		w->buf[w->len++] = (uint8_t)(v >> 8);
		w->buf[w->len++] = (uint8_t)v;
	}
}

void ocp1_w_u32(struct ocp1_writer *w, uint32_t v)
{
	if (w_fits(w, 4)) {
		w->buf[w->len++] = (uint8_t)(v >> 24);
		w->buf[w->len++] = (uint8_t)(v >> 16);
		w->buf[w->len++] = (uint8_t)(v >> 8);
		w->buf[w->len++] = (uint8_t)v;
	}
}

void ocp1_w_u64(struct ocp1_writer *w, uint64_t v)
{
	ocp1_w_u32(w, (uint32_t)(v >> 32));
	ocp1_w_u32(w, (uint32_t)v);
}

void ocp1_w_i32(struct ocp1_writer *w, int32_t v)
{
	ocp1_w_u32(w, (uint32_t)v);
}

void ocp1_w_f32(struct ocp1_writer *w, float v)
{
	uint32_t bits;

	memcpy(&bits, &v, sizeof(bits));
	ocp1_w_u32(w, bits);
}

void ocp1_w_bool(struct ocp1_writer *w, bool v)
{
	ocp1_w_u8(w, v ? 1 : 0);
}

void ocp1_w_str(struct ocp1_writer *w, const char *s)
{
	size_t n = s ? strlen(s) : 0;

	if (n > 0xFFFF) {
		n = 0xFFFF;
	}
	ocp1_w_u16(w, (uint16_t)n);
	ocp1_w_raw(w, (const uint8_t *)s, n);
}

void ocp1_w_blob(struct ocp1_writer *w, const uint8_t *p, size_t n)
{
	if (n > 0xFFFF) {
		w->overflow = true;
		return;
	}
	ocp1_w_u16(w, (uint16_t)n);
	ocp1_w_raw(w, p, n);
}

void ocp1_w_raw(struct ocp1_writer *w, const uint8_t *p, size_t n)
{
	if (n == 0) {
		return;
	}
	if (w_fits(w, n)) {
		memcpy(&w->buf[w->len], p, n);
		w->len += n;
	}
}

void ocp1_w_patch_u32(struct ocp1_writer *w, size_t off, uint32_t v)
{
	if (off + 4 > w->cap) {
		w->overflow = true;
		return;
	}
	w->buf[off]     = (uint8_t)(v >> 24);
	w->buf[off + 1] = (uint8_t)(v >> 16);
	w->buf[off + 2] = (uint8_t)(v >> 8);
	w->buf[off + 3] = (uint8_t)v;
}

void ocp1_w_patch_u16(struct ocp1_writer *w, size_t off, uint16_t v)
{
	if (off + 2 > w->cap) {
		w->overflow = true;
		return;
	}
	w->buf[off]     = (uint8_t)(v >> 8);
	w->buf[off + 1] = (uint8_t)v;
}

void ocp1_w_class_id(struct ocp1_writer *w, const uint16_t *id, size_t n)
{
	ocp1_w_u16(w, (uint16_t)n);
	for (size_t i = 0; i < n; i++) {
		ocp1_w_u16(w, id[i]);
	}
}

/* ================================================================
 * Reader
 * ================================================================ */

void ocp1_r_init(struct ocp1_reader *r, const uint8_t *buf, size_t len)
{
	r->buf = buf;
	r->len = len;
	r->pos = 0;
	r->err = false;
}

static bool r_has(struct ocp1_reader *r, size_t n)
{
	if (r->err || r->pos + n > r->len) {
		r->err = true;
		return false;
	}
	return true;
}

uint8_t ocp1_r_u8(struct ocp1_reader *r)
{
	if (!r_has(r, 1)) {
		return 0;
	}
	return r->buf[r->pos++];
}

uint16_t ocp1_r_u16(struct ocp1_reader *r)
{
	if (!r_has(r, 2)) {
		return 0;
	}
	uint16_t v = ((uint16_t)r->buf[r->pos] << 8) | r->buf[r->pos + 1];

	r->pos += 2;
	return v;
}

uint32_t ocp1_r_u32(struct ocp1_reader *r)
{
	if (!r_has(r, 4)) {
		return 0;
	}
	uint32_t v = ((uint32_t)r->buf[r->pos] << 24) |
		     ((uint32_t)r->buf[r->pos + 1] << 16) |
		     ((uint32_t)r->buf[r->pos + 2] << 8) |
		     r->buf[r->pos + 3];

	r->pos += 4;
	return v;
}

uint64_t ocp1_r_u64(struct ocp1_reader *r)
{
	uint64_t hi = ocp1_r_u32(r);
	uint64_t lo = ocp1_r_u32(r);

	return (hi << 32) | lo;
}

int32_t ocp1_r_i32(struct ocp1_reader *r)
{
	return (int32_t)ocp1_r_u32(r);
}

float ocp1_r_f32(struct ocp1_reader *r)
{
	uint32_t bits = ocp1_r_u32(r);
	float v;

	memcpy(&v, &bits, sizeof(v));
	return v;
}

bool ocp1_r_bool(struct ocp1_reader *r)
{
	return ocp1_r_u8(r) != 0;
}

size_t ocp1_r_str(struct ocp1_reader *r, char *out, size_t cap)
{
	uint16_t n = ocp1_r_u16(r);

	if (cap > 0) {
		out[0] = '\0';
	}
	if (!r_has(r, n)) {
		return 0;
	}
	if (cap > 0) {
		size_t c = n < cap - 1 ? n : cap - 1;

		memcpy(out, &r->buf[r->pos], c);
		out[c] = '\0';
	}
	r->pos += n;
	return n;
}

const uint8_t *ocp1_r_blob(struct ocp1_reader *r, size_t *n)
{
	uint16_t len = ocp1_r_u16(r);

	*n = 0;
	if (!r_has(r, len)) {
		return NULL;
	}
	const uint8_t *p = &r->buf[r->pos];

	r->pos += len;
	*n = len;
	return p;
}

void ocp1_r_skip(struct ocp1_reader *r, size_t n)
{
	if (r_has(r, n)) {
		r->pos += n;
	}
}

size_t ocp1_r_remaining(const struct ocp1_reader *r)
{
	return r->err ? 0 : r->len - r->pos;
}

/* ================================================================
 * PDU header
 * ================================================================ */

int ocp1_parse_header(const uint8_t *buf, size_t len, struct ocp1_header *h)
{
	struct ocp1_reader r;

	if (len < OCP1_MIN_PDU_LEN) {
		return -EAGAIN;
	}
	if (buf[0] != OCP1_SYNC_VAL) {
		return -EBADMSG;
	}
	ocp1_r_init(&r, buf + 1, len - 1);
	h->version  = ocp1_r_u16(&r);
	h->pdu_size = ocp1_r_u32(&r);
	h->type     = ocp1_r_u8(&r);
	h->count    = ocp1_r_u16(&r);
	/* AES70-3 says the version "shall be 1", but AES70.js (and with it
	 * AES70 Explorer) stamps its own edition number (2 = 2018, 3 = 2023,
	 * 4 = 2024) and ignores the field on receive. Accept any non-zero
	 * version; the wire format has not changed between them. We keep
	 * answering with 1. */
	if (h->version == 0 || h->pdu_size < OCP1_HEADER_LEN) {
		return -EBADMSG;
	}
	return 0;
}

size_t ocp1_begin_pdu(struct ocp1_writer *w, uint8_t type, uint16_t count)
{
	size_t size_off;

	ocp1_w_u8(w, OCP1_SYNC_VAL);
	ocp1_w_u16(w, OCP1_PROTOCOL_VERSION);
	size_off = w->len;
	ocp1_w_u32(w, 0);
	ocp1_w_u8(w, type);
	ocp1_w_u16(w, count);
	return size_off;
}

void ocp1_end_pdu(struct ocp1_writer *w, size_t size_off)
{
	/* PduSize excludes the sync byte, which sits at size_off - 3. */
	ocp1_w_patch_u32(w, size_off, (uint32_t)(w->len - (size_off - 2)));
}

void ocp1_patch_count(struct ocp1_writer *w, size_t size_off, uint16_t count)
{
	ocp1_w_patch_u16(w, size_off + 5, count);
}
