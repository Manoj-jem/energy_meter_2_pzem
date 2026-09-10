/* ================================================================
 * pzem_reading.pb.c  –  nanopb 0.4.9.x descriptor via PB_BIND()
 *
 * PB_BIND(MessageName, StructName, width, fields_x_macro)
 *
 * The X-macro lists fields as:
 *   X(field_name, atype, htype, ltype, tag)
 *
 * atype : STATIC | POINTER | CALLBACK
 * htype : REQUIRED | SINGULAR | OPTIONAL | REPEATED
 * ltype : BOOL | FLOAT | UINT32 | STRING | MESSAGE | etc.
 *
 * PB_BIND generates:
 *   - static uint32_t PzemChannel_field_info[]
 *   - static const pb_msgdesc_t* PzemChannel_submsg_info[]
 *   - const pb_msgdesc_t PzemChannel_msg  ← what pb_encode() needs
 * ================================================================ */

#include "pzem_reading.pb.h"

/* ── PzemChannel ─────────────────────────────────────────────────
 *
 * Fields (matching .proto tag numbers):
 *   tag 1: voltage      float   SINGULAR STATIC FLOAT
 *   tag 2: current      float   SINGULAR STATIC FLOAT
 *   tag 3: power        float   SINGULAR STATIC FLOAT
 *   tag 4: energy       float   SINGULAR STATIC FLOAT
 *   tag 5: frequency    float   SINGULAR STATIC FLOAT
 *   tag 6: power_factor float   SINGULAR STATIC FLOAT
 *   tag 7: alarm        bool    SINGULAR STATIC BOOL
 *   tag 8: valid        bool    SINGULAR STATIC BOOL
 * ─────────────────────────────────────────────────────────────── */

#define PzemChannel_FIELDLIST(X, a) \
    X(a, STATIC, SINGULAR, FLOAT,  voltage,      1) \
    X(a, STATIC, SINGULAR, FLOAT,  current,      2) \
    X(a, STATIC, SINGULAR, FLOAT,  power,        3) \
    X(a, STATIC, SINGULAR, FLOAT,  energy,       4) \
    X(a, STATIC, SINGULAR, FLOAT,  frequency,    5) \
    X(a, STATIC, SINGULAR, FLOAT,  power_factor, 6) \
    X(a, STATIC, SINGULAR, BOOL,   alarm,        7) \
    X(a, STATIC, SINGULAR, BOOL,   valid,        8)

/* No submessages in PzemChannel */
#define PzemChannel_CALLBACK NULL
#define PzemChannel_DEFAULT NULL

/* PB_BIND(proto_name, struct_name, width, fieldlist_macro)
 * width AUTO = let nanopb pick the smallest encoding width.     */
PB_BIND(PzemChannel, PzemChannel, AUTO)


/* ── PzemPayload ─────────────────────────────────────────────────
 *
 * Fields:
 *   tag 1: device_id  string  SINGULAR STATIC STRING  (max 32 bytes)
 *   tag 2: timestamp  uint32  SINGULAR STATIC UINT32
 *   tag 3: pzem1      msg     SINGULAR STATIC MESSAGE
 *   tag 4: pzem2      msg     SINGULAR STATIC MESSAGE
 *   tag 5: pzem3      msg     SINGULAR STATIC MESSAGE
 *
 * For submessage fields, nanopb needs a MSGTYPE typedef to locate
 * the nested descriptor. We provide it via the _MSGTYPE defines.
 * ─────────────────────────────────────────────────────────────── */

/* Tell PB_BIND what type each submessage field resolves to */
#define PzemPayload_pzem1_MSGTYPE PzemChannel
#define PzemPayload_pzem2_MSGTYPE PzemChannel
#define PzemPayload_pzem3_MSGTYPE PzemChannel

#define PzemPayload_FIELDLIST(X, a) \
    X(a, STATIC, SINGULAR, STRING,  device_id, 1) \
    X(a, STATIC, SINGULAR, UINT32,  timestamp, 2) \
    X(a, STATIC, SINGULAR, MESSAGE, pzem1,     3) \
    X(a, STATIC, SINGULAR, MESSAGE, pzem2,     4) \
    X(a, STATIC, SINGULAR, MESSAGE, pzem3,     5)

#define PzemPayload_CALLBACK NULL
#define PzemPayload_DEFAULT  NULL

PB_BIND(PzemPayload, PzemPayload, AUTO )
