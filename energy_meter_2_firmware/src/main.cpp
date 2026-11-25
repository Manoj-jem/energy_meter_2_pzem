// ================================================================
// main.cpp - 3-phase PZEM energy meter, firmware 2.x
//
// Tasks
//   meter    reads the 3 PZEMs every SEND_INTERVAL_MS, queues telemetry, writes
//            the alive journal, runs commands.
//   network  owns the modem: connect, publish, replay the flash queue, OTA,
//            and the self-recovery ladder when offline.
//   power    only with HAS_POWER_SENSE (this board has no power-sense inputs).
//
// What a reading carries beyond the measurements (schema 3): who/when of the
// meter itself - uptime, boot_count, reset_reason, restart_cause, offline_s -
// so the server can tell a power cut from a crash from a network outage.
// ================================================================

#include <Arduino.h>
#include <Preferences.h>
#include <esp_system.h>
#include <esp_task_wdt.h>
#include <freertos/queue.h>

#include "at_mqtt.h"
#include "calibration.h"
#include "commands.h"
#include "config.h"
#include "diag.h"
#include "netlog.h"
#include "ota.h"
#include "payload.h"
#include "pzem.h"
#include "storage.h"
#include "telemetry.h"
#include "time_service.h"

namespace {

constexpr uint8_t PHASE_COUNT = 3;
constexpr uint8_t PHASE_FAULT_COUNT = 3;
constexpr uint32_t POWER_FAIL_DEBOUNCE_MS = 100;
constexpr uint32_t POWER_RESTORE_DEBOUNCE_MS = 5000;
constexpr uint32_t CONNECT_BACKOFF_MAX_MS = 300000UL;
constexpr uint32_t STORAGE_REPORT_INTERVAL_MS = 3600000UL;
constexpr uint32_t ALIVE_NVS_INTERVAL_MS = 600000UL;   // fallback when the SPI NOR is dead
constexpr uint32_t RECOVERED_REPORT_MIN_MS = 300000UL; // first connect after boot: report only if this slow
constexpr uint8_t LIVE_QUEUE_DEPTH = 24;
constexpr uint8_t URGENT_QUEUE_DEPTH = 8;

// A queued record and, if it was also written ahead to flash, where.
struct QueueItem {
    TelemetryRecord record;
    StorageRef ref;
};

QueueHandle_t liveQueue = nullptr;
QueueHandle_t urgentQueue = nullptr;
portMUX_TYPE stateMux = portMUX_INITIALIZER_UNLOCKED;
TaskHandle_t meterHandle = nullptr;
TaskHandle_t networkHandle = nullptr;
TaskHandle_t powerHandle = nullptr;

PzemReading lastReadings[PHASE_COUNT]{};
uint8_t pzemFailureCount[PHASE_COUNT]{};
uint8_t pzemSuccessCount[PHASE_COUNT]{};
bool pzemFaulted[PHASE_COUNT]{};

uint32_t bootId = 0;
uint64_t nextSequence = 1;
bool storageReady = false;
bool mainsLost = false;
bool backupLow = false;
bool queueNearFullReported = false;
uint32_t lastDroppedNotified = 0;
volatile bool powerFailInterrupt = false;
ModemState modemState = MODEM_BOOTING;
int rssiRaw = -1;

// Network health, shared with the meter task (stateMux).
volatile uint32_t lastGoodMs = 0;      // last acknowledged publish
uint8_t ladderStep = 0;
uint32_t publishOk = 0;
uint32_t publishFailed = 0;
uint32_t reconnects = 0;
NetSnapshot lastNet{};
uint32_t lastNetAtS = 0;
uint32_t downTotalS = 0;    // MQTT down time between sessions since boot (excludes the first connect)   // uptime when lastNet was taken (0 = never)
RateLimiter limiter;

// Outage proof, set at boot, sent once the clock is known.
bool outagePending = false;
AliveEntry previousAlive{};

uint64_t allocateSequence() {
    portENTER_CRITICAL(&stateMux);
    const uint64_t value = nextSequence++;
    portEXIT_CRITICAL(&stateMux);
    return value;
}

void copyLastReadings(PzemReading output[PHASE_COUNT]) {
    portENTER_CRITICAL(&stateMux);
    memcpy(output, lastReadings, sizeof(lastReadings));
    portEXIT_CRITICAL(&stateMux);
}

void updateLastReadings(const PzemReading input[PHASE_COUNT]) {
    portENTER_CRITICAL(&stateMux);
    memcpy(lastReadings, input, sizeof(lastReadings));
    portEXIT_CRITICAL(&stateMux);
}

void setMainsState(bool lost) {
    portENTER_CRITICAL(&stateMux);
    mainsLost = lost;
    portEXIT_CRITICAL(&stateMux);
}

bool isMainsLost() {
    portENTER_CRITICAL(&stateMux);
    const bool value = mainsLost;
    portEXIT_CRITICAL(&stateMux);
    return value;
}

void setBackupLow(bool low) {
    portENTER_CRITICAL(&stateMux);
    backupLow = low;
    portEXIT_CRITICAL(&stateMux);
}

void setModemState(ModemState value) {
    portENTER_CRITICAL(&stateMux);
    modemState = value;
    portEXIT_CRITICAL(&stateMux);
}

void setRssiRaw(int value) {
    portENTER_CRITICAL(&stateMux);
    rssiRaw = value;
    portEXIT_CRITICAL(&stateMux);
}

void setLadderStep(uint8_t step) {
    portENTER_CRITICAL(&stateMux);
    ladderStep = step;
    portEXIT_CRITICAL(&stateMux);
}

uint32_t offlineSeconds() {
    if (at_mqtt_is_connected()) return 0;
    return (millis() - lastGoodMs) / 1000UL;
}

HealthSnapshot healthSnapshot(const PzemReading readings[PHASE_COUNT]) {
    HealthSnapshot health{};
    StorageStats stats = storage_stats();
    uint8_t validMask = 0;
    uint8_t alarmMask = 0;
    for (uint8_t index = 0; index < PHASE_COUNT; ++index) {
        if (readings[index].valid) validMask |= (1U << index);
        if (readings[index].alarm) alarmMask |= (1U << index);
    }
    portENTER_CRITICAL(&stateMux);
    health.mains_lost = mainsLost;
    health.backup_low = backupLow;
    health.modem_state = modemState;
    health.rssi_raw = rssiRaw;
    portEXIT_CRITICAL(&stateMux);
    const CellSignal cell = at_last_cell_signal();
    health.rsrp_dbm = cell.rsrp_dbm;
    health.rsrq_db = cell.rsrq_db;
    health.sinr_db = cell.sinr_db;
    health.band = cell.band;
    health.rat = cell.rat;
    health.storage_ready = stats.ready;
    health.mqtt_connected = at_mqtt_is_connected();
    health.pzem_valid_mask = validMask;
    health.pzem_alarm_mask = alarmMask;
    health.queue_depth = stats.depth;
    health.dropped_records = stats.dropped_records;
    return health;
}

DiagFields diagSnapshot() {
    const DiagBootInfo& boot = diag_info();
    DiagFields d{};
    d.uptime_s = diag_uptime_s();
    d.boot_count = boot.boot_count;
    d.offline_s = offlineSeconds();
    d.min_free_heap = ESP.getMinFreeHeap();
    d.reset_reason = boot.reset_reason;
    d.restart_cause = boot.prev_cause;
    d.fast_boots = boot.fast_boots;
    d.safe_mode = boot.safe_mode ? 1 : 0;
    portENTER_CRITICAL(&stateMux);
    d.events_suppressed = limiter.suppressed;
    d.ladder_step = ladderStep;
    portEXIT_CRITICAL(&stateMux);
    return d;
}

TelemetryRecord makeRecord(RecordType type, EventCode event, const PzemReading readings[PHASE_COUNT], uint8_t eventPhase = 0) {
    TelemetryRecord record{};
    record.record_type = type;
    record.event_code = event;
    record.event_phase = eventPhase;
    record.time_quality = time_service_quality();
    record.boot_id = bootId;
    record.seq = allocateSequence();
    record.event_ts_ms = time_service_now_ms();
    memcpy(record.readings, readings, sizeof(record.readings));
    record.health = healthSnapshot(readings);
    record.diag = diagSnapshot();
    return record;
}

// Events that are rare by construction (once per boot / per ladder step) and
// carry the answer to "what happened": never rate limited.
bool eventIsEssential(EventCode event) {
    switch (event) {
        case EVENT_BOOT: case EVENT_POWER_OUTAGE: case EVENT_SELF_RESTART: case EVENT_CRASH_RESET:
        case EVENT_WATCHDOG_RESET: case EVENT_SAFE_MODE: case EVENT_STORAGE_READY:
        case EVENT_NET_RECOVERY_STEP: case EVENT_NET_RECOVERED:
            return true;
        default:
            return false;
    }
}

void setDetailText(EventDetail& detail, const char* text) {
    strncpy(detail.text, text, sizeof(detail.text) - 1);
    detail.text[sizeof(detail.text) - 1] = '\0';
}

void emitRecord(TelemetryRecord& record, bool writeAhead) {
    const bool connected = at_mqtt_is_connected();
    QueueItem item{};
    item.record = record;
    bool stored = false;
    if ((writeAhead || !connected) && storageReady) {
        stored = storage_append(record, writeAhead, &item.ref);
        if (!stored) DBGF("[EVENT] Failed to store %s\n", event_name((EventCode)record.event_code));
    }
    // Live copy when connected; when offline and there is no flash, the RAM
    // queue is the only place it can wait for the connection.
    if (connected || !stored) {
        if (xQueueSendToFront(urgentQueue, &item, 0) != pdPASS && !stored && storageReady) {
            storage_append(record);
        }
    }
}

void emitEvent(EventCode event, bool writeAhead = false, uint8_t eventPhase = 0, const EventDetail* detail = nullptr) {
    if (!eventIsEssential(event)) {
        portENTER_CRITICAL(&stateMux);
        const bool allowed = rate_allow(limiter, (uint8_t)event, millis(), EVENT_RATE_MAX_PER_HOUR);
        portEXIT_CRITICAL(&stateMux);
        if (!allowed) return;
    }
    PzemReading readings[PHASE_COUNT]{};
    copyLastReadings(readings);
    TelemetryRecord record = makeRecord(RECORD_EVENT, event, readings, eventPhase);
    if (detail) record.detail = *detail;
    emitRecord(record, writeAhead);
}

void reportQueueHealth() {
    if (!storageReady) return;
    const StorageStats stats = storage_stats();
    if (stats.capacity && stats.depth * 100U >= stats.capacity * 80U) {
        if (!queueNearFullReported) {
            queueNearFullReported = true;
            emitEvent(EVENT_QUEUE_NEAR_FULL);
        }
    } else if (stats.capacity && stats.depth * 100U < stats.capacity * 70U) {
        queueNearFullReported = false;
    }
    if (stats.dropped_records > lastDroppedNotified) {
        lastDroppedNotified = stats.dropped_records;
        emitEvent(EVENT_QUEUE_OVERFLOW);
    }
}

void reportStorageUnavailable() {
    const StorageHealth h = storage_health();
    EventDetail detail{};
    detail.a = h.init_attempts;
    detail.b = h.chip_bytes / 1024;
    setDetailText(detail, h.fail_step);
    emitEvent(EVENT_STORAGE_UNAVAILABLE, false, 0, &detail);
}

// Network task only.
bool publishRecord(TelemetryRecord record, DeliveryMode delivery) {
    static char payload[MQTT_PAYLOAD_BUF_SIZE];
    // A record made earlier in this boot before the clock was set gets its
    // time back from its uptime ("estimated").
    if (record.event_ts_ms <= 0 && record.diag.boot_count == diag_info().boot_count &&
        time_service_quality() != TIME_UNKNOWN) {
        record.event_ts_ms = backdate_epoch_ms(time_service_now_ms(), diag_uptime_s(), record.diag.uptime_s);
        if (record.event_ts_ms > 0) record.time_quality = TIME_ESTIMATED;
    }
    if (payload_build(record, delivery, payload, sizeof(payload)) == 0) return false;
    return at_mqtt_publish(payload);
}

void updatePzemHealth(const PzemReading readings[PHASE_COUNT]) {
    for (uint8_t phase = 0; phase < PHASE_COUNT; ++phase) {
        if (readings[phase].valid) {
            pzemFailureCount[phase] = 0;
            if (pzemFaulted[phase] && ++pzemSuccessCount[phase] >= PHASE_FAULT_COUNT) {
                pzemFaulted[phase] = false;
                pzemSuccessCount[phase] = 0;
                emitEvent(EVENT_PZEM_PHASE_RECOVERED, false, phase + 1);
            }
        } else {
            pzemSuccessCount[phase] = 0;
            if (!pzemFaulted[phase] && ++pzemFailureCount[phase] >= PHASE_FAULT_COUNT) {
                pzemFaulted[phase] = true;
                pzemFailureCount[phase] = 0;
                emitEvent(EVENT_PZEM_PHASE_FAULT, false, phase + 1);
            }
        }
    }
}

// ── alive journal ────────────────────────────────────────────────

AliveEntry makeAlive(const PzemReading readings[PHASE_COUNT]) {
    AliveEntry entry{};
    entry.boot_count = diag_info().boot_count;
    entry.uptime_s = diag_uptime_s();
    entry.epoch_ms = time_service_now_ms();
    entry.time_quality = time_service_quality();
    for (uint8_t i = 0; i < PHASE_COUNT; ++i) entry.voltage[i] = readings[i].valid ? readings[i].voltage : 0.0f;
    return entry;
}

void aliveNvsWrite(const AliveEntry& entry) {
    Preferences prefs;
    if (prefs.begin("alive", false)) {
        prefs.putBytes("last", &entry, sizeof(entry));
        prefs.end();
    }
}

bool aliveNvsRead(AliveEntry& entry) {
    Preferences prefs;
    if (!prefs.begin("alive", true)) return false;
    const bool ok = prefs.getBytes("last", &entry, sizeof(entry)) == sizeof(entry);
    prefs.end();
    return ok;
}

// Network task, after a time sync: a power cut is proven from the previous
// boot's last alive entry and the time this boot started.
void maybeEmitOutage() {
    if (!outagePending || time_service_quality() == TIME_UNKNOWN) return;
    outagePending = false;
    const OutageEstimate est = outage_estimate(previousAlive.epoch_ms, time_service_now_ms(), (uint32_t)millis());
    EventDetail detail{};
    detail.ts_ms = est.known ? est.startMs : 0;
    detail.a = est.durationS;
    detail.b = previousAlive.uptime_s;
    for (uint8_t i = 0; i < PHASE_COUNT; ++i) detail.v[i] = previousAlive.voltage[i];
    char text[sizeof(detail.text)];
    snprintf(text, sizeof(text), "%s%s", reset_reason_name(diag_info().reset_reason),
             est.known ? "" : " start_unknown");
    setDetailText(detail, text);

    PzemReading readings[PHASE_COUNT]{};
    copyLastReadings(readings);
    TelemetryRecord record = makeRecord(RECORD_EVENT, EVENT_POWER_OUTAGE, readings);
    record.detail = detail;
    if (est.endMs > 0) {
        record.event_ts_ms = est.endMs;   // when power came back
        record.time_quality = TIME_ESTIMATED;
    }
    DBGF("[DIAG] Power outage: %lu s (last alive uptime %lu s)%s\n", (unsigned long)est.durationS,
         (unsigned long)previousAlive.uptime_s, est.known ? "" : ", start unknown");
    emitRecord(record, true);
}

// ── tasks ────────────────────────────────────────────────────────

void IRAM_ATTR powerFailIsr() { powerFailInterrupt = true; }

RuntimeStatus runtimeStatus() {
    RuntimeStatus s{};
    s.mqttConnected = at_mqtt_is_connected();
    s.offlineS = offlineSeconds();
    portENTER_CRITICAL(&stateMux);
    s.modemState = modemState;
    s.rssiRaw = rssiRaw;
    s.ladderStep = ladderStep;
    s.publishOk = publishOk;
    s.publishFailed = publishFailed;
    s.reconnects = reconnects;
    s.net = lastNet;
    s.netAtS = lastNetAtS;
    s.downTotalS = downTotalS;
    portEXIT_CRITICAL(&stateMux);
    PzemReading readings[PHASE_COUNT]{};
    copyLastReadings(readings);
    for (uint8_t i = 0; i < PHASE_COUNT; ++i) if (readings[i].valid) s.pzemValidMask |= (1U << i);
    if (meterHandle) s.stackFreeMeter = uxTaskGetStackHighWaterMark(meterHandle);
    if (networkHandle) s.stackFreeNetwork = uxTaskGetStackHighWaterMark(networkHandle);
    if (powerHandle) s.stackFreePower = uxTaskGetStackHighWaterMark(powerHandle);
    return s;
}

void meteringTask(void*) {
    esp_task_wdt_add(NULL);
    TickType_t nextWake = xTaskGetTickCount();
    uint32_t lastOfflineStoreMs = 0;
    bool offlineStoredOnce = false;
    uint32_t lastAliveMs = 0;
    bool aliveWritten = false;
    uint32_t lastStorageReportMs = millis();
    for (;;) {
        PzemReading readings[PHASE_COUNT]{};
        pzem_read_all(readings);
        updateLastReadings(readings);
        updatePzemHealth(readings);

        const uint32_t now = millis();
        TelemetryRecord record = makeRecord(RECORD_TELEMETRY, EVENT_NORMAL, readings);
        if (at_mqtt_is_connected()) {
            offlineStoredOnce = false;
            QueueItem item{};
            item.record = record;
            // Queue full = the link is slower than the meter: keep everything.
            if (xQueueSend(liveQueue, &item, 0) != pdPASS && storageReady) storage_append(record);
        } else if (storageReady && (!offlineStoredOnce || now - lastOfflineStoreMs >= OFFLINE_STORE_INTERVAL_MS)) {
            // Offline: thin to one record per OFFLINE_STORE_INTERVAL_MS (energy is cumulative).
            if (storage_append(record)) {
                offlineStoredOnce = true;
                lastOfflineStoreMs = now;
            }
        }

        // Alive journal: the first entry right away, so even a short boot leaves a trace.
        // Without the SPI NOR it goes to NVS, less often (NVS wear).
        const uint32_t aliveInterval = storageReady ? ALIVE_INTERVAL_MS : ALIVE_NVS_INTERVAL_MS;
        if (!aliveWritten || now - lastAliveMs >= aliveInterval) {
            const AliveEntry entry = makeAlive(readings);
            if (storageReady) storage_alive_write(entry);
            else aliveNvsWrite(entry);
            aliveWritten = true;
            lastAliveMs = now;
        }

        reportQueueHealth();
        if (!storageReady && now - lastStorageReportMs >= STORAGE_REPORT_INTERVAL_MS) {
            lastStorageReportMs = now;
            reportStorageUnavailable();
        }
        diag_tick();
        commands_process(runtimeStatus());
        if (commands_restart_due()) diag_restart(RC_CMD_REBOOT, offlineSeconds());

        esp_task_wdt_reset();
        vTaskDelayUntil(&nextWake, pdMS_TO_TICKS(SEND_INTERVAL_MS));
    }
}

#if HAS_POWER_SENSE
void powerTask(void*) {
    esp_task_wdt_add(NULL);
    bool rawMainsState = false;
    bool rawBackupState = false;
    uint32_t mainsChangedAt = millis();
    uint32_t backupChangedAt = millis();
    for (;;) {
        const bool mainsInput = digitalRead(PWR_FAIL_PIN) == HIGH;
        const bool backupInput = digitalRead(BACKUP_LOW_PIN) == HIGH;
        const uint32_t now = millis();

        if (mainsInput != rawMainsState || powerFailInterrupt) {
            rawMainsState = mainsInput;
            mainsChangedAt = now;
            powerFailInterrupt = false;
        }
        if (backupInput != rawBackupState) {
            rawBackupState = backupInput;
            backupChangedAt = now;
        }

        bool currentMainsLost;
        bool currentBackupLow;
        portENTER_CRITICAL(&stateMux);
        currentMainsLost = mainsLost;
        currentBackupLow = backupLow;
        portEXIT_CRITICAL(&stateMux);

        if (rawMainsState && !currentMainsLost && now - mainsChangedAt >= POWER_FAIL_DEBOUNCE_MS) {
            setMainsState(true);
            emitEvent(EVENT_MAINS_LOST, true);
        } else if (!rawMainsState && currentMainsLost && now - mainsChangedAt >= POWER_RESTORE_DEBOUNCE_MS) {
            setMainsState(false);
            emitEvent(EVENT_MAINS_RESTORED);
        }

        if (rawBackupState && !currentBackupLow && now - backupChangedAt >= POWER_FAIL_DEBOUNCE_MS) {
            setBackupLow(true);
            emitEvent(EVENT_BACKUP_LOW, true);
            storage_flush();
        } else if (!rawBackupState && currentBackupLow && now - backupChangedAt >= POWER_RESTORE_DEBOUNCE_MS) {
            setBackupLow(false);
        }
        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
#endif

uint32_t nextBackoff(uint32_t current) {
    if (current == 0) return 5000UL;
    const uint32_t doubled = current > CONNECT_BACKOFF_MAX_MS / 2 ? CONNECT_BACKOFF_MAX_MS : current * 2;
    return doubled + (esp_random() % 1000UL);
}

// OTA abort condition (ota.h): data first while the backlog is large.
bool otaBlocked() { return storage_stats().depth > OTA_MAX_BACKLOG; }

void networkTask(void*) {
    esp_task_wdt_add(NULL);
    bool modemInitialised = false;
    bool wasConnected = false;
    bool everDelivered = false;      // a publish has succeeded in this boot
    bool deliveryLost = false;       // offline since the last successful publish
    bool replayActive = false;
    uint32_t nextConnectAt = 0;
    uint32_t reconnectBackoff = 0;
    uint32_t lastRssiAt = 0;
    uint8_t lowSignalCount = 0;
    uint8_t recoveredSignalCount = 0;
    uint8_t unknownSignalCount = 0;
    bool lowSignal = false;
    bool signalUnknown = false;
    uint32_t otaSeenAt = 0;
    uint8_t publishFailStreak = 0;
    uint8_t step = 0;                // recovery ladder step taken in this outage
    uint8_t stepMax = 0;
    uint32_t sessionUpAt = 0;        // when the current MQTT session came up (connection log)
    uint32_t sessionDownAt = 0;      // when it was last lost
    uint32_t lastProbeAt = 0;        // last layer probe in this outage, 0 = none yet

    // A successful publish: the link works end to end.
    auto delivered = [&]() {
        const uint32_t now = millis();
        if (deliveryLost || step > 0 || (!everDelivered && now - lastGoodMs >= RECOVERED_REPORT_MIN_MS)) {
            EventDetail detail{};
            detail.a = (now - lastGoodMs) / 1000UL;
            detail.b = stepMax;
            portENTER_CRITICAL(&stateMux);
            const NetSnapshot snap = lastNet;
            portEXIT_CRITICAL(&stateMux);
            setDetailText(detail, snap.text);
            lastGoodMs = now;   // before emitting: the event itself reports offline_s 0
            emitEvent(EVENT_NET_RECOVERED, false, 0, &detail);
        }
        lastGoodMs = now;
        everDelivered = true;
        deliveryLost = false;
        publishFailStreak = 0;
        step = 0;
        stepMax = 0;
        setLadderStep(0);
        diag_note_connected();
        portENTER_CRITICAL(&stateMux);
        ++publishOk;
        portEXIT_CRITICAL(&stateMux);
    };
    auto publishFailedOnce = [&]() {
        portENTER_CRITICAL(&stateMux);
        ++publishFailed;
        portEXIT_CRITICAL(&stateMux);
        if (++publishFailStreak >= PUBLISH_FAILS_OFFLINE && at_mqtt_is_connected()) {
            DBGLN("[NET] Publishes keep failing on an open session: tearing it down");
            netlog_add("%u pub fails in a row: teardown", (unsigned)publishFailStreak);
            at_mqtt_disconnect();
        }
    };

    for (;;) {
        at_mqtt_poll();
        ota_trial_tick();
        const uint32_t now = millis();

        if (!at_mqtt_is_connected()) {
#if HAS_POWER_SENSE
            if (isMainsLost()) {
                // Hold-up time is reserved for the already-connected final
                // publish and flash checkpoint, never a fresh network attach.
                vTaskDelay(pdMS_TO_TICKS(100));
                continue;
            }
#endif
            if (wasConnected) {
                wasConnected = false;
                replayActive = false;
                sessionDownAt = now;
                const AtMqttStats st = at_mqtt_stats();
                netlog_add("mqtt down after %lus up: %s", (unsigned long)((now - sessionUpAt) / 1000UL),
                           st.last_drop[0] ? st.last_drop : "?");
                emitEvent(EVENT_MQTT_DISCONNECTED);
            }
            if (everDelivered) deliveryLost = true;

            // Self-recovery ladder (diag_core.h): 10 / 20 / 30 min offline.
            const uint32_t offlineMs = now - lastGoodMs;
            const LadderConfig ladder{NET_L1_MS, NET_L2_MS,
                                      diag_net_loss_streak() ? NET_L3_REPEAT_MS : NET_L3_MS};
            const LadderAction action = ladder_next(offlineMs, step, ladder);
            if (action != LADDER_NONE) {
                stepMax = step;
                setLadderStep(step);
                EventDetail detail{};
                detail.a = step;
                detail.b = offlineMs / 1000UL;
                portENTER_CRITICAL(&stateMux);
                setDetailText(detail, lastNet.text);
                portEXIT_CRITICAL(&stateMux);
                // Written ahead: if the step does not help, this is the record of it.
                netlog_add("ladder step %u after %lus offline", (unsigned)step, (unsigned long)(offlineMs / 1000UL));
                emitEvent(EVENT_NET_RECOVERY_STEP, true, 0, &detail);
                if (action == LADDER_RESTART) diag_restart(RC_NET_LOSS, offlineMs / 1000UL);
                const bool ok = action == LADDER_RADIO_CYCLE ? at_modem_radio_cycle() : at_modem_reset();
                if (!ok) DBGLN("[NET] Recovery step did not get an answer from the modem");
                modemInitialised = false;
                reconnectBackoff = 0;
                nextConnectAt = millis();
            }

            if ((int32_t)(millis() - nextConnectAt) >= 0) {
                setModemState(MODEM_BOOTING);
                const uint32_t attemptAt = millis();
                const bool fullInit = !modemInitialised;
                bool ready = modemInitialised || gsm_modem_init();
                modemInitialised = ready;
                if (ready) {
                    setModemState(MODEM_AT_READY);
                    ready = at_mqtt_connect();
                }
                if (ready) ready = at_mqtt_subscribe();
                if (ready) {
                    setModemState(MODEM_MQTT_CONNECTED);
                    wasConnected = true;
                    reconnectBackoff = 0;
                    publishFailStreak = 0;
                    nextConnectAt = millis();
                    portENTER_CRITICAL(&stateMux);
                    ++reconnects;
                    portEXIT_CRITICAL(&stateMux);
                    sessionUpAt = millis();
                    lastProbeAt = 0;
                    if (sessionDownAt) downTotalS += (sessionUpAt - sessionDownAt) / 1000UL;
                    netlog_add("mqtt up: attempt %lums%s, down %lus", (unsigned long)(sessionUpAt - attemptAt),
                               fullInit ? " (modem init)" : "",
                               sessionDownAt ? (unsigned long)((sessionUpAt - sessionDownAt) / 1000UL) : 0UL);
                    time_service_sync_from_modem();
                    emitEvent(EVENT_MQTT_RECONNECTED);
                    ota_on_connected();
                    maybeEmitOutage();
                } else {
                    NetSnapshot snap{};
                    at_net_snapshot(snap);
                    if (snap.at_ok && snap.csq >= 0) setRssiRaw(snap.csq);
                    portENTER_CRITICAL(&stateMux);
                    lastNet = snap;
                    lastNetAtS = diag_uptime_s();
                    portEXIT_CRITICAL(&stateMux);
                    // "fail <stage> e<CMQTTCONNECT err> <s> at<0|1> q<csq> r<creg>/<cereg>"
                    netlog_add("fail %s e%d %lus at%d q%d r%d/%d", snap.stage, (int)snap.last_err,
                               (unsigned long)((millis() - attemptAt) / 1000UL), snap.at_ok ? 1 : 0,
                               (int)snap.csq, (int)snap.creg, (int)snap.cereg);
                    if (snap.at_ok && (!lastProbeAt || millis() - lastProbeAt >= NET_PROBE_EVERY_MS)) {
                        char probe[48];
                        at_net_probe(probe, sizeof(probe));
                        netlog_add("probe %s", probe);
                        lastProbeAt = millis() ? millis() : 1;
                    }
                    setModemState(MODEM_BACKOFF);
                    EventDetail detail{};
                    detail.a = (millis() - lastGoodMs) / 1000UL;
                    detail.b = (uint32_t)snap.last_err;
                    setDetailText(detail, snap.text);
                    emitEvent(snap.sim_ready == 0 ? EVENT_SIM_NOT_READY : EVENT_NETWORK_DETACHED, false, 0, &detail);
                    reconnectBackoff = nextBackoff(reconnectBackoff);
                    nextConnectAt = millis() + reconnectBackoff;
                    modemInitialised = false;
                }
            }
            esp_task_wdt_reset();
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        if (now - lastRssiAt >= RSSI_SAMPLE_INTERVAL_MS) {
            lastRssiAt = now;
            const int raw = at_mqtt_get_rssi_raw();
            setRssiRaw(raw);
            CellSignal cell{};
            at_cell_signal(cell);   // RSRP / RSRQ / SINR for the readings
            if (raw == 99) {
                lowSignalCount = recoveredSignalCount = 0;
                if (!signalUnknown && ++unknownSignalCount >= RSSI_CONSECUTIVE_COUNT) {
                    signalUnknown = true;
                    emitEvent(EVENT_GSM_SIGNAL_UNKNOWN);
                }
            } else if (raw >= 0 && raw < RSSI_LOW_RAW) {
                unknownSignalCount = recoveredSignalCount = 0;
                signalUnknown = false;
                if (!lowSignal && ++lowSignalCount >= RSSI_CONSECUTIVE_COUNT) {
                    lowSignal = true;
                    emitEvent(EVENT_GSM_LOW_SIGNAL);
                }
            } else if (raw >= RSSI_RECOVER_RAW) {
                unknownSignalCount = lowSignalCount = 0;
                signalUnknown = false;
                if (lowSignal && ++recoveredSignalCount >= RSSI_CONSECUTIVE_COUNT) {
                    lowSignal = false;
                    emitEvent(EVENT_GSM_RECOVERED);
                }
            }
        }

        // A command reply goes out ahead of telemetry: the sender is waiting on it.
        static char ack[CMD_ACK_MAX];
        if (commands_take_ack(ack, sizeof(ack)) || ota_take_ack(ack, sizeof(ack))) {
            at_mqtt_publish_to(MQTT_TOPIC_ACK, ack);
        }

        // A firmware update holds this task for the download (telemetry queues
        // to flash meanwhile). Wait a moment so the "accepted" reply goes out
        // first. ota_run() restarts on success; on failure the loop publishes
        // the failure reply.
        if (!ota_pending()) {
            otaSeenAt = 0;
        } else if (otaSeenAt == 0) {
            otaSeenAt = now ? now : 1;
        } else if (now - otaSeenAt >= 1000) {
            otaSeenAt = 0;
            ota_run(otaBlocked);
            if (at_mqtt_is_connected()) lastGoodMs = millis();   // the download kept the link busy, not dead
            continue;
        }

        QueueItem item{};
        const bool fromUrgent = xQueueReceive(urgentQueue, &item, 0) == pdPASS;
        const bool fromLive = !fromUrgent && xQueueReceive(liveQueue, &item, 0) == pdPASS;
        if (fromUrgent || fromLive) {
            if (publishRecord(item.record, DELIVERY_LIVE)) {
                delivered();
                if (item.ref.order) storage_mark_delivered(item.ref);   // no second (replay) copy
            } else {
                publishFailedOnce();
                if (storageReady && item.ref.order == 0) storage_append(item.record, fromUrgent);
            }
        } else if (storage_has_pending()) {
            if (!replayActive) {
                replayActive = true;
                emitEvent(EVENT_QUEUE_REPLAY_STARTED);
            }
            TelemetryRecord record{};
            if (storage_peek(record)) {
                if (publishRecord(record, DELIVERY_REPLAY)) {
                    delivered();
                    storage_pop();
                } else {
                    publishFailedOnce();
                }
            }
        } else if (replayActive) {
            replayActive = false;
            emitEvent(EVENT_QUEUE_REPLAY_COMPLETED);
        }
        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

}  // namespace

void setup() {
    Serial.setRxBufferSize(1536);   // a serial {"cmd":"ota",...} line is up to ~1.1 KB
    Serial.begin(115200);
    delay(250);
    DBGF("\n[MAIN] %s firmware %s\n", DEVICE_ID, FIRMWARE_VERSION);

    diag_boot();          // why this boot happened, before anything can restart again
    netlog_init(diag_info().boot_count);   // connection log in RTC RAM ({"cmd":"log"})
    ota_boot_check();     // may boot the previous image instead (failed update)
    const DiagBootInfo& boot = diag_info();

    bootId = esp_random();
    rate_init(limiter);
#if HAS_POWER_SENSE
    pinMode(PWR_FAIL_PIN, INPUT);
    pinMode(BACKUP_LOW_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(PWR_FAIL_PIN), powerFailIsr, CHANGE);
#endif

    time_service_init();
    calibration_init();
    pzem_init();
    storageReady = storage_init();
    if (!storageReady) DBGLN("[MAIN] SPI NOR queue unavailable");
#ifdef FLASH_SELFTEST
    storage_selftest(true);   // before the tasks start, so nothing else touches the chip
#endif

    // The previous boot's last sign of life (read before this boot writes one).
    const bool haveAlive = storageReady ? storage_last_alive(previousAlive) : aliveNvsRead(previousAlive);
    outagePending = haveAlive && reset_is_power_loss(boot.reset_reason);

    at_mqtt_set_safe_mode(boot.safe_mode);
    commands_init();
    at_mqtt_set_callback([](const char* topic, const char* payload) {
        ota_on_message(topic, payload);         // firmware-update stream blocks
        commands_on_message(topic, payload);    // <device>/cmd
    });

    liveQueue = xQueueCreate(LIVE_QUEUE_DEPTH, sizeof(QueueItem));
    urgentQueue = xQueueCreate(URGENT_QUEUE_DEPTH, sizeof(QueueItem));
    if (liveQueue == nullptr || urgentQueue == nullptr) {
        DBGLN("[MAIN] Queue allocation failed; restarting");
        diag_restart(RC_QUEUE_ALLOC);
    }

    // A full SPI-flash recovery scan is valid boot work. Start the watchdog
    // only after it has completed so a full queue cannot cause a reset loop.
    esp_task_wdt_init(TASK_WATCHDOG_TIMEOUT_S, true);
    esp_task_wdt_add(NULL);

    xTaskCreatePinnedToCore(meteringTask, "meter", 8192, nullptr, 2, &meterHandle, 1);
#if HAS_POWER_SENSE
    xTaskCreatePinnedToCore(powerTask, "power", 4096, nullptr, 3, &powerHandle, 1);
#endif
    xTaskCreatePinnedToCore(networkTask, "network", 12288, nullptr, 1, &networkHandle, 0);

    // ── what this boot says about the last one ──
    if (storageReady) {
        const StorageHealth h = storage_health();
        EventDetail detail{};
        detail.a = h.corrupt_slots;
        detail.b = h.chip_bytes / 1024;
        char text[sizeof(detail.text)];
        snprintf(text, sizeof(text), "jedec %02X %02X %02X", h.jedec[0], h.jedec[1], h.jedec[2]);
        setDetailText(detail, text);
        emitEvent(EVENT_STORAGE_READY, true, 0, &detail);
    } else {
        reportStorageUnavailable();
    }

    EventDetail bootDetail{};
    bootDetail.a = boot.prev_uptime_s;
    bootDetail.b = boot.prev_offline_s;
    if (haveAlive) {
        bootDetail.ts_ms = previousAlive.epoch_ms;
        for (uint8_t i = 0; i < PHASE_COUNT; ++i) bootDetail.v[i] = previousAlive.voltage[i];
    }
    char bootText[sizeof(bootDetail.text)];
    snprintf(bootText, sizeof(bootText), "%s %s", reset_reason_name(boot.reset_reason), restart_cause_name(boot.prev_cause));
    setDetailText(bootDetail, bootText);
    emitEvent(EVENT_BOOT, storageReady, 0, &bootDetail);

    if (boot.reset_reason == RR_TASK_WDT) emitEvent(EVENT_WATCHDOG_RESET, storageReady);
    if (reset_is_crash(boot.reset_reason)) emitEvent(EVENT_CRASH_RESET, storageReady, 0, &bootDetail);
    if (boot.reset_reason == RR_SOFTWARE && boot.prev_cause != RC_NONE) {
        EventDetail detail{};
        detail.a = boot.prev_offline_s;
        detail.b = boot.prev_uptime_s;
        setDetailText(detail, restart_cause_name(boot.prev_cause));
        emitEvent(EVENT_SELF_RESTART, storageReady, 0, &detail);
    }
    if (boot.safe_mode) emitEvent(EVENT_SAFE_MODE, storageReady);
}

void loop() {
    esp_task_wdt_reset();
#ifdef FLASH_SELFTEST
    static uint32_t lastStatusAt = 0;
    if (millis() - lastStatusAt >= 5000UL) {
        lastStatusAt = millis();
        storage_print_status();
        DBGF("[FLASHTEST] mqtt=%d\n", at_mqtt_is_connected());
    }
#endif
    delay(1000);
}
