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
};

const char* event_name(EventCode code);
const char* modem_state_name(ModemState state);
const char* time_quality_name(TimeQuality quality);
