/*
 * OCP.1 (AES70-3) wire format: PDU constants and big-endian marshaling
 * helpers.
 *
 * Pure data code (no kernel, no sockets) so it can be unit-tested on
 * the host/QEMU target.  Everything on the wire is network byte order;
 * strings are OcaString = u16 count + UTF-8 bytes (count = code points,
 * which equals the byte length for the ASCII we emit), blobs are u16
 * length + bytes, lists are u16 count + items.
 */

#ifndef AES70_OCP1_H_
#define AES70_OCP1_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- PDU framing (AES70-3 clause 6.2) ---- */

#define OCP1_SYNC_VAL          0x3B
#define OCP1_PROTOCOL_VERSION  1
/* Header after the sync byte: version(2) size(4) type(1) count(2). */
#define OCP1_HEADER_LEN        9
/* Minimum bytes needed before a PDU can be sized: sync + header. */
#define OCP1_MIN_PDU_LEN       (1 + OCP1_HEADER_LEN)

enum ocp1_pdu_type {
	OCP1_PDU_CMD        = 0,  /* command, no response */
	OCP1_PDU_CMD_RRQ    = 1,  /* command, response required */
	OCP1_PDU_NTF1       = 2,  /* EV1 notification (deprecated) */
	OCP1_PDU_RSP        = 3,  /* response */
	OCP1_PDU_KEEPALIVE  = 4,
	OCP1_PDU_NTF2       = 5,  /* EV2 notification */
};

/* ---- OcaStatus (AES70-2A Operating Datatypes) ---- */

enum oca_status {
	OCA_STATUS_OK                     = 0,
	OCA_STATUS_PROTOCOL_VERSION_ERROR = 1,
	OCA_STATUS_DEVICE_ERROR           = 2,
	OCA_STATUS_LOCKED                 = 3,
	OCA_STATUS_BAD_FORMAT             = 4,
	OCA_STATUS_BAD_ONO                = 5,
	OCA_STATUS_PARAMETER_ERROR        = 6,
	OCA_STATUS_PARAMETER_OUT_OF_RANGE = 7,
	OCA_STATUS_NOT_IMPLEMENTED        = 8,
	OCA_STATUS_INVALID_REQUEST        = 9,
	OCA_STATUS_PROCESSING_FAILED      = 10,
	OCA_STATUS_BAD_METHOD             = 11,
	OCA_STATUS_PARTIALLY_SUCCEEDED    = 12,
	OCA_STATUS_TIMEOUT                = 13,
	OCA_STATUS_BUFFER_OVERFLOW        = 14,
	OCA_STATUS_PERMISSION_DENIED      = 15,
	OCA_STATUS_OUT_OF_MEMORY          = 16,
	OCA_STATUS_BUSY                   = 17,
};

/* ---- Writer ---- */

struct ocp1_writer {
	uint8_t *buf;
	size_t   cap;
	size_t   len;
	bool     overflow;   /* sticky: a put did not fit */
};

void ocp1_w_init(struct ocp1_writer *w, uint8_t *buf, size_t cap);
void ocp1_w_u8(struct ocp1_writer *w, uint8_t v);
void ocp1_w_u16(struct ocp1_writer *w, uint16_t v);
void ocp1_w_u32(struct ocp1_writer *w, uint32_t v);
void ocp1_w_u64(struct ocp1_writer *w, uint64_t v);
void ocp1_w_i32(struct ocp1_writer *w, int32_t v);
void ocp1_w_f32(struct ocp1_writer *w, float v);
void ocp1_w_bool(struct ocp1_writer *w, bool v);
/* OcaString: u16 count + bytes (NULL -> empty string). */
void ocp1_w_str(struct ocp1_writer *w, const char *s);
/* OcaBlob: u16 length + bytes. */
void ocp1_w_blob(struct ocp1_writer *w, const uint8_t *p, size_t n);
/* Raw bytes, no length prefix (OcaBlobFixedLen / nested PDUs). */
void ocp1_w_raw(struct ocp1_writer *w, const uint8_t *p, size_t n);
/* Patch a big-endian u32 at an earlier offset (size fields). */
void ocp1_w_patch_u32(struct ocp1_writer *w, size_t off, uint32_t v);
void ocp1_w_patch_u16(struct ocp1_writer *w, size_t off, uint16_t v);
/* OcaClassID: u16 field count + fields. */
void ocp1_w_class_id(struct ocp1_writer *w, const uint16_t *id, size_t n);

/* ---- Reader ---- */

struct ocp1_reader {
	const uint8_t *buf;
	size_t len;
	size_t pos;
	bool   err;          /* sticky: a get ran past the end */
};

void     ocp1_r_init(struct ocp1_reader *r, const uint8_t *buf, size_t len);
uint8_t  ocp1_r_u8(struct ocp1_reader *r);
uint16_t ocp1_r_u16(struct ocp1_reader *r);
uint32_t ocp1_r_u32(struct ocp1_reader *r);
uint64_t ocp1_r_u64(struct ocp1_reader *r);
int32_t  ocp1_r_i32(struct ocp1_reader *r);
float    ocp1_r_f32(struct ocp1_reader *r);
bool     ocp1_r_bool(struct ocp1_reader *r);
/* OcaString into a NUL-terminated buffer (truncated to cap-1, the
 * whole string is still consumed). Returns the wire length. */
size_t   ocp1_r_str(struct ocp1_reader *r, char *out, size_t cap);
/* OcaBlob: returns a pointer into the message and its length (NULL on
 * error). */
const uint8_t *ocp1_r_blob(struct ocp1_reader *r, size_t *n);
/* Skip n raw bytes. */
void     ocp1_r_skip(struct ocp1_reader *r, size_t n);
size_t   ocp1_r_remaining(const struct ocp1_reader *r);

/* ---- PDU header ---- */

struct ocp1_header {
	uint16_t version;
	uint32_t pdu_size;   /* excludes the sync byte */
	uint8_t  type;
	uint16_t count;
};

/* Parse sync + header from the start of buf. Returns 0, -EAGAIN when
 * fewer than OCP1_MIN_PDU_LEN bytes are present, -EBADMSG on a bad
 * sync value or protocol version. */
int ocp1_parse_header(const uint8_t *buf, size_t len, struct ocp1_header *h);

/* Begin a PDU: writes sync + header with a placeholder size; returns
 * the offset of the size field for ocp1_end_pdu(). */
size_t ocp1_begin_pdu(struct ocp1_writer *w, uint8_t type, uint16_t count);
/* Fix up the PduSize field (= bytes written after the sync byte). */
void   ocp1_end_pdu(struct ocp1_writer *w, size_t size_off);
void   ocp1_patch_count(struct ocp1_writer *w, size_t size_off, uint16_t count);

#ifdef __cplusplus
}
#endif

#endif /* AES70_OCP1_H_ */
