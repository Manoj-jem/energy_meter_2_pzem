/* ================================================================
 * pzem_reading.pb.h  –  Hand-written nanopb descriptor for 0.4.9.x
 *
 * nanopb 0.4.x replaced the old pb_field_t[] array with a new
 * compact descriptor system using PB_BIND() macro and pb_msgdesc_t.
 *
 * Key differences from 0.3.x:
 *   OLD: extern const pb_field_t MyMessage_fields[];
 *        pb_encode(&stream, MyMessage_fields, &msg);
 *
 *   NEW: extern const pb_msgdesc_t MyMessage_msg;
 *        pb_encode(&stream, MyMessage_msg, &msg);    // note: no & on descriptor
 *        OR use the helper macro:
 *        pb_encode(&stream, MyMessage_fields, &msg); // still works via macro
 *
 * The PB_BIND() macro generates both the field_info[] array and the
 * pb_msgdesc_t struct in one shot using offsetof() — no manual offsets.
 * ================================================================ */

#ifndef PZEM_READING_PB_H
#define PZEM_READING_PB_H

#include <pb.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Struct definitions ─────────────────────────────────────────
 * These MUST exactly match the field names used in PB_BIND() below.
 * ────────────────────────────────────────────────────────────── */

typedef struct {
    float    voltage;
    float    current;
    float    power;
    float    energy;
    float    frequency;
    float    power_factor;
    bool     alarm;
    bool     valid;
} PzemChannel;

typedef struct {
    char        device_id[32];   /* max 31 chars + null */
    uint32_t    timestamp;
    PzemChannel pzem1;
    PzemChannel pzem2;
    PzemChannel pzem3;
} PzemPayload;

/* ── Initialiser zeros ──────────────────────────────────────────── */
#define PzemChannel_init_zero  {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, false, false}
#define PzemPayload_init_zero  {"", 0, PzemChannel_init_zero, \
                                     PzemChannel_init_zero, \
                                     PzemChannel_init_zero}

/* ── Message descriptor declarations ────────────────────────────
 * Defined in pzem_reading.pb.c via PB_BIND().
 * pb_encode() takes a pointer to these.
 * ────────────────────────────────────────────────────────────── */
extern const pb_msgdesc_t PzemChannel_msg;
extern const pb_msgdesc_t PzemPayload_msg;

/* ── Convenience macros matching the old API style ──────────────
 * pb_encode(&stream, PzemPayload_fields, &payload) still works.
 * ────────────────────────────────────────────────────────────── */
#define PzemChannel_fields   &PzemChannel_msg
#define PzemPayload_fields   &PzemPayload_msg

/* ── Max encoded sizes ──────────────────────────────────────────
 * Per channel: 6 floats×5 + 2 bools×2 = 34 bytes
 * Payload: 3 channels×(2-byte tag + 34) + string~33 + uint32~5 ≈ 150 bytes
 * ────────────────────────────────────────────────────────────── */
#define PzemChannel_size    40
#define PzemPayload_size    200

#ifdef __cplusplus
}
#endif

#endif /* PZEM_READING_PB_H */
