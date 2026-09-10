// ================================================================
// payload.cpp 
// ================================================================

#include <Arduino.h>
#include <inttypes.h>
#include "config.h"
#include "calibration.h"
#include "payload.h"
#include "time_service.h"

const char* event_name(EventCode code) {
    static const char* const names[] = {
        "NORMAL", "BOOT", "MAINS_LOST", "MAINS_RESTORED", "BACKUP_LOW",
        "GSM_LOW_SIGNAL", "GSM_RECOVERED", "GSM_SIGNAL_UNKNOWN", "SIM_NOT_READY",
        "NETWORK_DETACHED", "MQTT_DISCONNECTED", "MQTT_RECONNECTED",
        "PZEM_PHASE_FAULT", "PZEM_PHASE_RECOVERED", "STORAGE_READY",
        "STORAGE_UNAVAILABLE", "QUEUE_NEAR_FULL", "QUEUE_OVERFLOW",
        "QUEUE_APPEND_FAILED", "QUEUE_REPLAY_STARTED", "QUEUE_REPLAY_COMPLETED",
        "WATCHDOG_RESET"
    };
    return code <= EVENT_WATCHDOG_RESET ? names[code] : "UNKNOWN";
}

const char* modem_state_name(ModemState state) {
    static const char* const names[] = {"booting", "at_ready", "registered", "mqtt_connected", "backoff"};
    return state <= MODEM_BACKOFF ? names[state] : "unknown";
}

const char* time_quality_name(TimeQuality quality) {
    static const char* const names[] = {"unknown", "synced", "estimated"};
    return quality <= TIME_ESTIMATED ? names[quality] : "unknown";
}

size_t payload_build(const TelemetryRecord& record,
                     DeliveryMode delivery,
                     char* outBuf,
                     size_t outBufLen) {
    char timestamp[32];
    char energyTotal[24];
    time_service_format_iso8601(record.event_ts_ms, timestamp, sizeof(timestamp));
    const bool allValid = record.readings[0].valid && record.readings[1].valid && record.readings[2].valid;
    const float energyKwh = (record.readings[0].energy + record.readings[1].energy + record.readings[2].energy) / 1000.0f;
    if (allValid) snprintf(energyTotal, sizeof(energyTotal), "%.4f", energyKwh);
    else strncpy(energyTotal, "null", sizeof(energyTotal));

    int written = snprintf(outBuf, outBufLen,
        "{"
          "\"schema_version\":2,"
          "\"device_id\":\"%s\","
          "\"ts\":\"%s\","
          "\"event_ts\":\"%s\","
          "\"time_quality\":\"%s\","
          "\"record_type\":\"%s\","
          "\"record_id\":\"%s:%" PRIu32 ":%" PRIu64 "\","
          "\"boot_id\":%" PRIu32 ","
          "\"seq\":%" PRIu64 ","
          "\"delivery\":\"%s\","
          "\"event\":\"%s\","
          "\"event_phase\":%u,"
          "\"phases\":["
            "{"
              "\"phase\":1,"
              "\"voltage_v\":%.2f,"
              "\"current_a\":%.2f,"
              "\"power_kw\":%.4f,"
              "\"power_factor\":%.2f,"
              "\"frequency_hz\":%.2f,"
              "\"energy_kwh\":%.4f,"
              "\"valid\":%s,"
              "\"alarm\":%s"
            "},"
            "{"
              "\"phase\":2,"
              "\"voltage_v\":%.2f,"
              "\"current_a\":%.2f,"
              "\"power_kw\":%.4f,"
              "\"power_factor\":%.2f,"
              "\"frequency_hz\":%.2f,"
              "\"energy_kwh\":%.4f,"
              "\"valid\":%s,"
              "\"alarm\":%s"
            "},"
            "{"
              "\"phase\":3,"
              "\"voltage_v\":%.2f,"
              "\"current_a\":%.2f,"
              "\"power_kw\":%.4f,"
              "\"power_factor\":%.2f,"
              "\"frequency_hz\":%.2f,"
              "\"energy_kwh\":%.4f,"
              "\"valid\":%s,"
              "\"alarm\":%s"
            "}"
          "],"
          "\"energy_kwh_total\":%s,"
          "\"health\":{"
            "\"mains_lost\":%s,\"backup_low\":%s,\"storage_ready\":%s,"
            "\"mqtt_connected\":%s,\"modem_state\":\"%s\",\"rssi_raw\":%d,"
            "\"pzem_valid_mask\":%u,\"pzem_alarm_mask\":%u,"
            "\"queue_depth\":%u,\"dropped_records\":%" PRIu32
          "},"
          "\"meta\":{"
            "\"fw\":\"%s\","
            "\"rssi\":%d,"
            "\"ct_primary_a\":%.1f,\"ct_secondary_ma\":%.1f,"
            "\"ct_nominal_multiplier\":%.3f,"
            "\"phase_gain\":[%.3f,%.3f,%.3f]"
          "}"
        "}",

        DEVICE_ID,
        timestamp,
        timestamp,
        time_quality_name((TimeQuality)record.time_quality),
        record.record_type == RECORD_EVENT ? "event" : "telemetry",
        DEVICE_ID, record.boot_id, record.seq,
        record.boot_id,
        record.seq,
        delivery == DELIVERY_REPLAY ? "replay" : "live",
        event_name((EventCode)record.event_code),
        record.event_phase,

        record.readings[0].voltage, record.readings[0].current, record.readings[0].power / 1000.0f,
        record.readings[0].power_factor, record.readings[0].frequency, record.readings[0].energy / 1000.0f,
        record.readings[0].valid ? "true" : "false", record.readings[0].alarm ? "true" : "false",

        record.readings[1].voltage, record.readings[1].current, record.readings[1].power / 1000.0f,
        record.readings[1].power_factor, record.readings[1].frequency, record.readings[1].energy / 1000.0f,
        record.readings[1].valid ? "true" : "false", record.readings[1].alarm ? "true" : "false",

        record.readings[2].voltage, record.readings[2].current, record.readings[2].power / 1000.0f,
        record.readings[2].power_factor, record.readings[2].frequency, record.readings[2].energy / 1000.0f,
        record.readings[2].valid ? "true" : "false", record.readings[2].alarm ? "true" : "false",

        energyTotal,
        record.health.mains_lost ? "true" : "false", record.health.backup_low ? "true" : "false",
        record.health.storage_ready ? "true" : "false", record.health.mqtt_connected ? "true" : "false",
        modem_state_name((ModemState)record.health.modem_state), record.health.rssi_raw,
        record.health.pzem_valid_mask, record.health.pzem_alarm_mask,
        record.health.queue_depth, record.health.dropped_records,

        FIRMWARE_VERSION,
        record.health.rssi_raw,
        CT_PRIMARY_A, CT_SECONDARY_MA, CT_NOMINAL_MULTIPLIER,
        calibration_phase_gain(1), calibration_phase_gain(2), calibration_phase_gain(3)
    );

    if (written < 0 || (size_t)written >= outBufLen) {
        DBGLN("[PAYLOAD] Buffer overflow — increase mqttPayload size");
        return 0;
    }

    DBGF("[PAYLOAD] %d bytes: %s\n", written, outBuf);
    return (size_t)written;
}
