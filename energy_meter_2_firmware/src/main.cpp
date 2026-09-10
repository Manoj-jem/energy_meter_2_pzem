


#include <Arduino.h>
#include <esp_system.h>
#include <esp_task_wdt.h>
#include <freertos/queue.h>

#include "at_mqtt.h"
#include "calibration.h"
#include "config.h"
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
constexpr uint8_t LIVE_QUEUE_DEPTH = 24;
constexpr uint8_t URGENT_QUEUE_DEPTH = 8;

QueueHandle_t liveQueue = nullptr;
QueueHandle_t urgentQueue = nullptr;
portMUX_TYPE stateMux = portMUX_INITIALIZER_UNLOCKED;

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
    health.storage_ready = stats.ready;
    health.mqtt_connected = at_mqtt_is_connected();
    health.pzem_valid_mask = validMask;
    health.pzem_alarm_mask = alarmMask;
    health.queue_depth = stats.depth;
    health.dropped_records = stats.dropped_records;
    return health;
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
    return record;
}

bool enqueueLive(const TelemetryRecord& record) {
    if (!at_mqtt_is_connected()) {
        return storageReady && storage_append(record);
    }
    if (xQueueSend(liveQueue, &record, 0) == pdPASS) return true;
    return storageReady && storage_append(record);
}

void emitEvent(EventCode event, bool writeAhead = false, uint8_t eventPhase = 0) {
    PzemReading readings[PHASE_COUNT]{};
    copyLastReadings(readings);
    TelemetryRecord record = makeRecord(RECORD_EVENT, event, readings, eventPhase);
    const bool mustStore = writeAhead || !at_mqtt_is_connected();
    if (mustStore && storageReady && !storage_append(record, writeAhead)) {
        DBGF("[EVENT] Failed to store %s\n", event_name(event));
    }
    if (at_mqtt_is_connected()) {
        if (xQueueSendToFront(urgentQueue, &record, 0) != pdPASS && !mustStore && storageReady) {
            storage_append(record);
        }
    }
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

bool publishRecord(const TelemetryRecord& record, DeliveryMode delivery) {
    char payload[MQTT_PAYLOAD_BUF_SIZE];
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

void IRAM_ATTR powerFailIsr() { powerFailInterrupt = true; }

void meteringTask(void*) {
    esp_task_wdt_add(NULL);
    TickType_t nextWake = xTaskGetTickCount();
    for (;;) {
        PzemReading readings[PHASE_COUNT]{};
        pzem_read_all(readings);
        updateLastReadings(readings);
        updatePzemHealth(readings);
        enqueueLive(makeRecord(RECORD_TELEMETRY, EVENT_NORMAL, readings));
        reportQueueHealth();
        esp_task_wdt_reset();
        vTaskDelayUntil(&nextWake, pdMS_TO_TICKS(SEND_INTERVAL_MS));
    }
}

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

uint32_t nextBackoff(uint32_t current) {
    if (current == 0) return 5000UL;
    const uint32_t doubled = current > CONNECT_BACKOFF_MAX_MS / 2 ? CONNECT_BACKOFF_MAX_MS : current * 2;
    return doubled + (esp_random() % 1000UL);
}

void networkTask(void*) {
    esp_task_wdt_add(NULL);
    bool modemInitialised = false;
    bool wasConnected = false;
    bool replayActive = false;
    uint32_t nextConnectAt = 0;
    uint32_t reconnectBackoff = 0;
    uint32_t lastRssiAt = 0;
    uint8_t lowSignalCount = 0;
    uint8_t recoveredSignalCount = 0;
    uint8_t unknownSignalCount = 0;
    bool lowSignal = false;
    bool signalUnknown = false;

    for (;;) {
        at_mqtt_poll();
        const uint32_t now = millis();

        if (!at_mqtt_is_connected()) {
            if (isMainsLost()) {
                // Hold-up time is reserved for the already-connected final
                // publish and flash checkpoint, never a fresh network attach.
                vTaskDelay(pdMS_TO_TICKS(100));
                continue;
            }
            if (wasConnected) {
                wasConnected = false;
                replayActive = false;
                emitEvent(EVENT_MQTT_DISCONNECTED);
            }
            if (now >= nextConnectAt) {
                setModemState(MODEM_BOOTING);
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
                    nextConnectAt = now;
                    time_service_sync_from_modem();
                    emitEvent(EVENT_MQTT_RECONNECTED);
                } else {
                    setModemState(MODEM_BACKOFF);
                    emitEvent(modemInitialised ? EVENT_NETWORK_DETACHED : EVENT_SIM_NOT_READY);
                    reconnectBackoff = nextBackoff(reconnectBackoff);
                    nextConnectAt = now + reconnectBackoff;
                    modemInitialised = false;
                }
            }
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        if (now - lastRssiAt >= RSSI_SAMPLE_INTERVAL_MS) {
            lastRssiAt = now;
            const int raw = at_mqtt_get_rssi_raw();
            setRssiRaw(raw);
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

        TelemetryRecord record{};
        bool fromUrgent = xQueueReceive(urgentQueue, &record, 0) == pdPASS;
        bool fromLive = !fromUrgent && xQueueReceive(liveQueue, &record, 0) == pdPASS;
        if (fromUrgent || fromLive) {
            if (!publishRecord(record, DELIVERY_LIVE) && storageReady) storage_append(record, fromUrgent);
        } else if (storage_has_pending()) {
            if (!replayActive) {
                replayActive = true;
                emitEvent(EVENT_QUEUE_REPLAY_STARTED);
            }
            if (storage_peek(record) && publishRecord(record, DELIVERY_REPLAY)) storage_pop();
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
    Serial.begin(115200);
    delay(250);
    bootId = esp_random();
    pinMode(PWR_FAIL_PIN, INPUT);
    pinMode(BACKUP_LOW_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(PWR_FAIL_PIN), powerFailIsr, CHANGE);

    time_service_init();
    calibration_init();
    pzem_init();
    storageReady = storage_init();
    if (!storageReady) DBGLN("[MAIN] SPI NOR queue unavailable");

    liveQueue = xQueueCreate(LIVE_QUEUE_DEPTH, sizeof(TelemetryRecord));
    urgentQueue = xQueueCreate(URGENT_QUEUE_DEPTH, sizeof(TelemetryRecord));
    if (liveQueue == nullptr || urgentQueue == nullptr) {
        DBGLN("[MAIN] Queue allocation failed; restarting");
        ESP.restart();
    }

    // A full SPI-flash recovery scan is valid boot work. Start the watchdog
    // only after it has completed so a full queue cannot cause a reset loop.
    // GSM registration and TLS setup are still blocking in the Phase 1 modem
    // driver and can take well over 45 seconds under poor signal. Keep all
    // service tasks supervised without resetting a healthy reconnect attempt.
    esp_task_wdt_init(TASK_WATCHDOG_TIMEOUT_S, true);
    esp_task_wdt_add(NULL);

    xTaskCreatePinnedToCore(meteringTask, "meter", 6144, nullptr, 2, nullptr, 1);
    xTaskCreatePinnedToCore(powerTask, "power", 4096, nullptr, 3, nullptr, 1);
    xTaskCreatePinnedToCore(networkTask, "network", 8192, nullptr, 1, nullptr, 0);
    emitEvent(storageReady ? EVENT_STORAGE_READY : EVENT_STORAGE_UNAVAILABLE, storageReady);
    if (esp_reset_reason() == ESP_RST_TASK_WDT) emitEvent(EVENT_WATCHDOG_RESET, storageReady);
    emitEvent(EVENT_BOOT, storageReady);
}

void loop() {
    esp_task_wdt_reset();
    delay(1000);
}
