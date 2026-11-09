/*
 * AES70 (OCA) device model and OCP.1 session core — transport-agnostic.
 *
 * The model is a flat table of objects. Each object is an instance of a
 * "kind" (an AES70 class as we implement it: class ID, class version,
 * the base class family it hangs off, and the list of properties with
 * their getter/setter method indices). The core handles everything the
 * standard defines generically:
 *
 *  - OcaRoot methods (class identification, role, locks),
 *  - OcaWorker / OcaAgent methods (enabled, label, owner, ports, path),
 *  - OcaBlock enumeration (members, recursive, role search),
 *  - property Get/Set methods from the kind's property table, marshaled
 *    per value type (with the value/min/max triple where the class
 *    defines it),
 *  - OcaSubscriptionManager (EV2 and the deprecated EV1 flavour),
 *  - PropertyChanged notifications: the core snapshots the notifiable
 *    properties of every subscribed object and emits a notification
 *    whenever a re-read differs — so changes made through the web UI,
 *    the shell, NMOS or the card itself reach AES70 controllers too,
 *  - keep-alive supervision per session.
 *
 * What differs per object is reduced to two callbacks (get / set of a
 * property value) plus an optional per-kind "custom" method handler
 * for the few classes with non-property methods (device manager, media
 * clock, time source).
 *
 * Threading: everything in here runs on the caller's thread (the OCP.1
 * server thread). The device-side callbacks may take their own locks.
 */

#ifndef AES70_MODEL_H_
#define AES70_MODEL_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "ocp1.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---- Standard object numbers (AES70-1 table 3) ---- */

#define OCA_ONO_DEVICE_MANAGER        1
#define OCA_ONO_FIRMWARE_MANAGER      3
#define OCA_ONO_SUBSCRIPTION_MANAGER  4
#define OCA_ONO_MEDIA_CLOCK_MANAGER   7
#define OCA_ONO_ROOT_BLOCK            100

/* ---- Limits (Kconfig on the firmware, defaults for the unit tests) ---- */

#ifndef AES70_MAX_OBJECTS
#ifdef CONFIG_AES70_MAX_OBJECTS
#define AES70_MAX_OBJECTS CONFIG_AES70_MAX_OBJECTS
#else
#define AES70_MAX_OBJECTS 256
#endif
#endif

#ifndef AES70_MAX_SESSIONS
#ifdef CONFIG_AES70_MAX_SESSIONS
#define AES70_MAX_SESSIONS CONFIG_AES70_MAX_SESSIONS
#else
#define AES70_MAX_SESSIONS 4
#endif
#endif

#ifndef AES70_MAX_SUBSCRIPTIONS
#ifdef CONFIG_AES70_MAX_SUBSCRIPTIONS
#define AES70_MAX_SUBSCRIPTIONS CONFIG_AES70_MAX_SUBSCRIPTIONS
#else
#define AES70_MAX_SUBSCRIPTIONS 64
#endif
#endif

#ifndef AES70_RX_BUF_SIZE
#ifdef CONFIG_AES70_RX_BUF_SIZE
#define AES70_RX_BUF_SIZE CONFIG_AES70_RX_BUF_SIZE
#else
#define AES70_RX_BUF_SIZE 1024
#endif
#endif

#ifndef AES70_TX_BUF_SIZE
#ifdef CONFIG_AES70_TX_BUF_SIZE
#define AES70_TX_BUF_SIZE CONFIG_AES70_TX_BUF_SIZE
#else
#define AES70_TX_BUF_SIZE 8192
#endif
#endif

/* Longest OcaString value a property can hold (device/stream names are
 * 32, "a.b.c.d:port" is 21). */
#define OCA_STR_MAX   40
/* Deprecated EV1 subscriber context we keep per subscription. */
#define OCA_EV1_CTX_MAX 16
/* Notifiable properties snapshotted per object. */
#define OCA_NOTIFY_SLOTS 2

/* ---- Values ---- */

enum oca_vt {
	OCA_VT_NONE = 0,
	OCA_VT_BOOL,
	OCA_VT_U8,
	OCA_VT_U16,
	OCA_VT_I32,
	OCA_VT_F32,
	OCA_VT_STR,
};

struct oca_value {
	uint8_t vt;
	union {
		bool     b;
		uint8_t  u8;
		uint16_t u16;
		int32_t  i32;
		float    f32;
		char     str[OCA_STR_MAX];
	};
	/* Range for the numeric Get methods that return (value, min, max). */
	float min;
	float max;
};

/* ---- Property descriptor ---- */

#define OCA_PF_RANGE    0x01  /* Get returns value, min, max */
#define OCA_PF_NOTIFY   0x02  /* snapshotted for PropertyChanged */

struct oca_prop {
	uint8_t level;        /* property / method DefLevel */
	uint8_t index;        /* PropertyIndex */
	uint8_t vt;           /* enum oca_vt */
	uint8_t get_method;   /* MethodIndex at `level`, 0 = none */
	uint8_t set_method;   /* MethodIndex at `level`, 0 = read-only */
	uint8_t flags;        /* OCA_PF_* */
};

/* ---- Kinds (classes) ---- */

enum oca_base {
	OCA_BASE_ROOT = 0,  /* only OcaRoot methods below the class's own */
	OCA_BASE_WORKER,    /* OcaWorker level-2 methods */
	OCA_BASE_AGENT,     /* OcaAgent level-2 methods */
	OCA_BASE_MANAGER,   /* OcaManager (no level-2 methods) */
	OCA_BASE_BLOCK,     /* OcaWorker + OcaBlock level-3 methods */
};

struct oca_object;
struct oca_session;

/* Custom method handler. Return a status, or OCA_METHOD_UNHANDLED to
 * fall through to the generic property handling. `in` holds the
 * command parameters (after ParameterCount), `out` receives the output
 * parameters (the core prepends the count). `in_count` is the
 * ParameterCount byte. */
#define OCA_METHOD_UNHANDLED (-1)

typedef int (*oca_custom_fn)(struct oca_object *obj, struct oca_session *s,
			     uint16_t level, uint16_t index,
			     uint8_t in_count, struct ocp1_reader *in,
			     struct ocp1_writer *out, uint8_t *out_count);

struct oca_kind {
	const char            *name;
	const uint16_t        *class_id;
	uint8_t                class_id_len;
	uint16_t               class_version;
	uint8_t                base;          /* enum oca_base */
	const struct oca_prop *props;
	uint8_t                nprops;
	oca_custom_fn          custom;        /* may be NULL */
};

/* ---- Objects ---- */

typedef int (*oca_get_fn)(const struct oca_object *obj, uint8_t prop,
			  struct oca_value *v);
typedef int (*oca_set_fn)(const struct oca_object *obj, uint8_t prop,
			  const struct oca_value *v);

struct oca_object {
	uint32_t               ono;
	const struct oca_kind *kind;
	const char            *role;
	uint32_t               owner;      /* containing block ONo, 0 = root */
	uint16_t               arg;        /* device-side index (channel, slot) */
	bool                   lockable;
	oca_get_fn             get;        /* NULL = no properties */
	oca_set_fn             set;        /* NULL = read-only object */
	/* runtime */
	uint8_t                lock_state; /* OcaLockState */
	struct oca_session    *lock_owner;
};

/* ---- Sessions ---- */

struct oca_subscription {
	bool     used;
	uint32_t emitter;
	uint16_t ev_level;
	uint16_t ev_index;
	bool     prop_specific;
	uint16_t p_level;
	uint16_t p_index;
	/* Deprecated EV1 flavour: notifications are delivered as a method
	 * call on the subscriber object with the stored context. */
	bool     ev1;
	uint32_t sub_ono;
	uint16_t sub_level;
	uint16_t sub_index;
	uint8_t  ctx_len;
	uint8_t  ctx[OCA_EV1_CTX_MAX];
};

typedef int (*oca_send_fn)(struct oca_session *s, const uint8_t *buf, size_t len);

struct oca_session {
	bool     used;
	bool     close_pending;    /* transport must close (protocol error,
				    * heartbeat loss) */
	oca_send_fn send;
	void    *transport;        /* opaque for the transport layer */
	uint8_t  rx[AES70_RX_BUF_SIZE];   /* must hold one command, not a PDU */
	size_t   rx_len;
	/* command PDU being consumed incrementally */
	size_t   cmd_remaining;    /* bytes of commands still to come */
	bool     cmd_respond;      /* Ocp1CmdRrq: answer them */
	/* keep-alive supervision (AES70-3 clause 6.4) */
	uint32_t heartbeat_ms;     /* 0 = monitoring not started */
	int64_t  last_rx_ms;
	int64_t  last_tx_ms;
	struct oca_subscription subs[AES70_MAX_SUBSCRIPTIONS];
	/* statistics */
	uint32_t commands;
	uint32_t notifications;
};

/* ---- Core API ---- */

/* Reset the model and all sessions (call once before adding objects). */
void aes70_core_init(void);

/* Add an object. `tmpl` is copied; the role string must stay valid.
 * Returns 0 or -ENOMEM / -EEXIST (duplicate ONo). */
int aes70_core_add_object(const struct oca_object *tmpl);

struct oca_object *aes70_core_find(uint32_t ono);
/* Iterate: index 0..n-1, NULL past the end. */
struct oca_object *aes70_core_object_at(int idx);
int aes70_core_object_count(void);

/* Open a session for a connected transport. NULL when all slots are
 * taken. */
struct oca_session *aes70_core_session_open(oca_send_fn send, void *transport,
					    int64_t now_ms);
/* Close: releases locks and subscriptions. */
void aes70_core_session_close(struct oca_session *s);
struct oca_session *aes70_core_session_at(int idx);

/* Feed received bytes. Complete PDUs are processed and answered through
 * s->send. Returns 0, or a negative errno when the stream is corrupt
 * (bad sync value, oversized PDU) — the transport must then close the
 * connection (the session is marked close_pending as well). */
int aes70_core_session_input(struct oca_session *s, const uint8_t *data,
			     size_t len, int64_t now_ms);

/* Periodic work: keep-alive supervision + PropertyChanged detection.
 * Call every ~100-500 ms and after every command batch. */
void aes70_core_tick(int64_t now_ms);

/* Flag that device state changed outside AES70 (observer hooks); the
 * next tick re-reads all subscribed objects. Cheap, ISR-safe. */
void aes70_core_set_dirty(void);

/* Re-read one object now and notify subscribers of changes. */
void aes70_core_refresh_object(uint32_t ono, int64_t now_ms);

/* Emit an arbitrary event with pre-marshaled data to all subscribers of
 * (emitter, level, index). */
void aes70_core_emit_event(uint32_t emitter, uint16_t level, uint16_t index,
			   const uint8_t *data, size_t len, int64_t now_ms);

/* Helpers for custom handlers: lock check (returns OCA_STATUS_LOCKED
 * when another session holds a write lock). */
int aes70_core_check_write(struct oca_object *obj, struct oca_session *s);

/* Write a property value in its wire form (no min/max). */
void aes70_core_write_value(struct ocp1_writer *w, const struct oca_value *v);

/* Standard kinds provided by the core (blocks and the subscription
 * manager need no device-side code). */
extern const struct oca_kind oca_kind_block;
extern const struct oca_kind oca_kind_subscription_manager;

/* Optional command trace (shell debugging): called after every command
 * with the result. NULL disables. */
typedef void (*aes70_trace_fn)(uint32_t ono, uint16_t level, uint16_t index,
			       int status, uint8_t out_count);
void aes70_core_set_trace(aes70_trace_fn fn);

/* Total number of active subscriptions (shell). */
int aes70_core_subscription_count(void);

#ifdef __cplusplus
}
#endif

#endif /* AES70_MODEL_H_ */
