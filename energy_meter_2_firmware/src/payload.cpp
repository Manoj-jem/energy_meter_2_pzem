// ================================================================
// payload.cpp 
// ================================================================

#include <Arduino.h>
#include <inttypes.h>
#include "config.h"
#include "calibration.h"
#include "payload.h"
#include "time_service.h"
#include "diag_core.h"
#include "at_mqtt.h"

namespace {
void nullableInt(char* out, size_t len, bool known, int value) {
    if (known) snprintf(out, len, "%d", value);
    else snprintf(out, len, "null");
}
}  // namespace

const char* event_name(EventCode code) {
    static const char* const names[] = {
        "NORMAL", "BOOT", "MAINS_LOST", "MAINS_RESTORED", "BACKUP_LOW",
        "GSM_LOW_SIGNAL", "GSM_RECOVERED", "GSM_SIGNAL_UNKNOWN", "SIM_NOT_READY",
        "NETWORK_DETACHED", "MQTT_DISCONNECTED", "MQTT_RECONNECTED",
        "PZEM_PHASE_FAULT", "PZEM_PHASE_RECOVERED", "STORAGE_READY",
        "STORAGE_UNAVAILABLE", "QUEUE_NEAR_FULL", "QUEUE_OVERFLOW",
        "QUEUE_APPEND_FAILED", "QUEUE_REPLAY_STARTED", "QUEUE_REPLAY_COMPLETED",
        "WATCHDOG_RESET",
        "POWER_OUTAGE", "SELF_RESTART", "CRASH_RESET", "NET_RECOVERY_STEP", "NET_RECOVERED",
        "SAFE_MODE"
    };
    static_assert(sizeof(names) / sizeof(names[0]) == EVENT_LAST + 1, "one name per EventCode");
    return code <= EVENT_LAST ? names[code] : "UNKNOWN";
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

    // fw 2.1.0: network strength, as it was when the record was made.
    const HealthSnapshot& h = record.health;
    const bool csqKnown = h.rssi_raw >= 0 && h.rssi_raw <= 31;
    const bool lte = h.rat == 1 && h.rsrp_dbm < 0;
    char rssiDbm[8], rsrp[8], rsrq[8], sinr[8];
    nullableInt(rssiDbm, sizeof(rssiDbm), csqKnown, -113 + 2 * h.rssi_raw);
    nullableInt(rsrp, sizeof(rsrp), lte, h.rsrp_dbm);
    nullableInt(rsrq, sizeof(rsrq), lte && h.rsrq_db != 0, h.rsrq_db);
    nullableInt(sinr, sizeof(sinr), lte && h.sinr_db != -128, h.sinr_db);
    char signal[200];
    snprintf(signal, sizeof(signal),
             "{\"quality\":\"%s\",\"csq\":%d,\"rssi_dbm\":%s,\"rat\":\"%s\",\"band\":%u,"
             "\"rsrp_dbm\":%s,\"rsrq_db\":%s,\"sinr_db\":%s}",
             signal_quality(h.rssi_raw, h.rat, h.rsrp_dbm), (int)h.rssi_raw, rssiDbm,
             cell_rat_name(h.rat), (unsigned)h.band, rsrp, rsrq, sinr);

    int written = snprintf(outBuf, outBufLen,
        "{"
          "\"schema_version\":%d,"
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
            "\"queue_depth\":%u,\"dropped_records\":%" PRIu32 ","
            "\"signal\":%s"
          "},"
          "\"diag\":{"
            "\"uptime_s\":%" PRIu32 ",\"boot_count\":%" PRIu32 ",\"reset_reason\":\"%s\","
            "\"restart_cause\":\"%s\",\"fast_boots\":%u,\"offline_s\":%" PRIu32 ","
            "\"ladder_step\":%u,\"safe_mode\":%s,\"min_free_heap\":%" PRIu32 ","
            "\"events_suppressed\":%u"
          "},"
          "\"meta\":{"
            "\"fw\":\"%s\","
            "\"rssi\":%d,"
            "\"ct_primary_a\":%.1f,\"ct_secondary_ma\":%.1f,"
            "\"ct_nominal_multiplier\":%.3f,"
            "\"phase_gain\":[%.3f,%.3f,%.3f]"
          "}"
        "}",

        PAYLOAD_SCHEMA_VERSION,
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
        signal,

        record.diag.uptime_s, record.diag.boot_count, reset_reason_name(record.diag.reset_reason),
        restart_cause_name(record.diag.restart_cause), (unsigned)record.diag.fast_boots, record.diag.offline_s,
        (unsigned)record.diag.ladder_step, record.diag.safe_mode ? "true" : "false", record.diag.min_free_heap,
        (unsigned)record.diag.events_suppressed,

        FIRMWARE_VERSION,
        record.health.rssi_raw,
        CT_PRIMARY_A, CT_SECONDARY_MA, CT_NOMINAL_MULTIPLIER,
        calibration_phase_gain(1), calibration_phase_gain(2), calibration_phase_gain(3)
    );

    if (written < 0 || (size_t)written >= outBufLen) {
        DBGLN("[PAYLOAD] Buffer overflow — increase MQTT_PAYLOAD_BUF_SIZE");
        return 0;
    }

    // Events carry their detail: replace the final '}' with ,"detail":{...}}
    if (record.record_type == RECORD_EVENT) {
        const EventDetail& d = record.detail;
        char detailTs[32];
        time_service_format_iso8601(d.ts_ms, detailTs, sizeof(detailTs));
        char text[sizeof(d.text) + 1];
        size_t n = 0;
        for (size_t i = 0; i < sizeof(d.text) && d.text[i]; ++i) {
            const char c = d.text[i];
            if (c != '"' && c != '\\' && c >= 0x20) text[n++] = c;   // keep the JSON valid
        }
        text[n] = '\0';
        // fw 2.1.0: every event names the SIM (as read at the last modem init).
        const SimInfo sim = at_sim_info();
        const int more = snprintf(outBuf + written - 1, outBufLen - (size_t)written + 1,
            ",\"detail\":{\"ts\":%s%s%s,\"a\":%" PRIu32 ",\"b\":%" PRIu32 ","
            "\"v\":[%.1f,%.1f,%.1f],\"text\":\"%s\"},"
            "\"sim\":{\"iccid\":\"%s\",\"imsi\":\"%s\",\"msisdn\":\"%s\",\"operator\":\"%s\"}}",
            d.ts_ms > 0 ? "\"" : "", d.ts_ms > 0 ? detailTs : "null", d.ts_ms > 0 ? "\"" : "",
            d.a, d.b, d.v[0], d.v[1], d.v[2], text,
            sim.iccid, sim.imsi, sim.msisdn, sim.op);
        if (more < 0 || (size_t)(written - 1 + more) >= outBufLen) {
            DBGLN("[PAYLOAD] Buffer overflow (event detail)");
            return 0;
        }
        written += more - 1;
    }

#ifdef DEBUG_PAYLOAD
    DBGF("[PAYLOAD] %d bytes: %s\n", written, outBuf);
#else
    DBGF("[PAYLOAD] %d bytes, %s\n", written, event_name((EventCode)record.event_code));
#endif
    return (size_t)written;
}
