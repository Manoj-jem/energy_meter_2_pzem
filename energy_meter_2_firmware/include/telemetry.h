#pragma once

#include <Arduino.h>
#include "pzem.h"

enum RecordType : uint8_t {
    RECORD_TELEMETRY = 1,
    RECORD_EVENT = 2,
};

enum DeliveryMode : uint8_t {
    DELIVERY_LIVE = 1,
    DELIVERY_REPLAY = 2,
};

enum TimeQuality : uint8_t {
    TIME_UNKNOWN = 0,
    TIME_SYNCED = 1,
    TIME_ESTIMATED = 2,
};

// Codes are stored in flash and decoded by the backend: only ever APPEND.
enum EventCode : uint8_t {
    EVENT_NORMAL = 0,
    EVENT_BOOT,
    EVENT_MAINS_LOST,
    EVENT_MAINS_RESTORED,
    EVENT_BACKUP_LOW,
    EVENT_GSM_LOW_SIGNAL,
    EVENT_GSM_RECOVERED,
    EVENT_GSM_SIGNAL_UNKNOWN,
    EVENT_SIM_NOT_READY,
    EVENT_NETWORK_DETACHED,
    EVENT_MQTT_DISCONNECTED,
    EVENT_MQTT_RECONNECTED,
    EVENT_PZEM_PHASE_FAULT,
    EVENT_PZEM_PHASE_RECOVERED,
    EVENT_STORAGE_READY,
    EVENT_STORAGE_UNAVAILABLE,
    EVENT_QUEUE_NEAR_FULL,
    EVENT_QUEUE_OVERFLOW,
    EVENT_QUEUE_APPEND_FAILED,
    EVENT_QUEUE_REPLAY_STARTED,
    EVENT_QUEUE_REPLAY_COMPLETED,
    EVENT_WATCHDOG_RESET,
    // ── fw 2.x ──
    EVENT_POWER_OUTAGE,       // detail_ts = last alive, event_ts = power back, detail_a = duration_s
    EVENT_SELF_RESTART,       // previous boot ended by the firmware: diag.restart_cause says why
    EVENT_CRASH_RESET,        // panic / interrupt watchdog / other watchdog
    EVENT_NET_RECOVERY_STEP,  // detail_a = ladder step (1 radio cycle, 2 modem reset, 3 ESP restart)
    EVENT_NET_RECOVERED,      // detail_a = offline_s, detail_b = highest ladder step, text = last net state
    EVENT_SAFE_MODE,          // boot loop detected: running metering + MQTT + OTA only
    EVENT_LAST = EVENT_SAFE_MODE,
};

enum ModemState : uint8_t {
    MODEM_BOOTING = 0,
    MODEM_AT_READY,
    MODEM_REGISTERED,
    MODEM_MQTT_CONNECTED,
    MODEM_BACKOFF,
};

struct HealthSnapshot {
    uint8_t mains_lost;
    uint8_t backup_low;
    uint8_t storage_ready;
    uint8_t mqtt_connected;
    uint8_t modem_state;
    int8_t rssi_raw;
    uint8_t pzem_valid_mask;
    uint8_t pzem_alarm_mask;
    uint16_t queue_depth;
    uint32_t dropped_records;
};

// Who/when of the device itself, captured when the record is made (so a
// replayed record still shows the uptime it had at the time).
struct DiagFields {
    uint32_t uptime_s;
    uint32_t boot_count;
    uint32_t offline_s;          // 0 while connected
    uint32_t min_free_heap;
    uint16_t events_suppressed;  // by the rate limiter, this boot
    uint8_t  reset_reason;       // ResetReason (diag_core.h), this boot
    uint8_t  restart_cause;      // RestartCause of the previous boot, if it was a firmware restart
    uint8_t  fast_boots;
    uint8_t  ladder_step;
    uint8_t  safe_mode;
    uint8_t  reserved;
};

// Extra data for some events (zero otherwise).
struct EventDetail {
    int64_t  ts_ms;              // e.g. POWER_OUTAGE: last alive time
    uint32_t a;
    uint32_t b;
    float    v[3];               // e.g. POWER_OUTAGE: phase voltages at last alive
    char     text[28];           // e.g. NET_RECOVERED: "sim=1 creg=1 cereg=0 csq=9 err=32"
};

struct TelemetryRecord {
    uint8_t record_type;
    uint8_t event_code;
    uint8_t event_phase;
    uint8_t time_quality;
    uint8_t reserved;
    uint32_t boot_id;
    uint64_t seq;
    int64_t event_ts_ms;
    PzemReading readings[3];
    HealthSnapshot health;
    DiagFields diag;
    EventDetail detail;
};

const char* event_name(EventCode code);
const char* modem_state_name(ModemState state);
const char* time_quality_name(TimeQuality quality);
