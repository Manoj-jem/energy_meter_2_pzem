#include "commands.h"

#include <Arduino.h>
#include <ArduinoJson.h>
#include <esp_heap_caps.h>
#include <freertos/semphr.h>

#include "config.h"
#include <time.h>

#include "diag.h"
#include "netlog.h"
#include "ota.h"
#include "storage.h"
#include "telemetry.h"

namespace {

constexpr size_t ACK_MAX = CMD_ACK_MAX;
constexpr size_t LOG_PAGE = 12;   // lines per "log" reply; 12 x ~95 B fits ACK_MAX
constexpr uint32_t RESTART_DELAY_MS = 4000;   // lets the reply reach the broker first

SemaphoreHandle_t mutex = nullptr;
char pending[MQTT_RX_PAYLOAD_MAX + 1];
bool pendingSet = false;
char ack[ACK_MAX];
bool ackSet = false;
volatile uint32_t restartAtMs = 0;

char serialLine[MQTT_RX_PAYLOAD_MAX + 1];
size_t serialLen = 0;
bool serialOverflow = false;

void scheduleRestart() {
    const uint32_t at = millis() + RESTART_DELAY_MS;
    restartAtMs = at ? at : 1;   // 0 means "no restart pending"
}

void reply(JsonDocument& doc, const char* req, const char* cmd, bool ok, const char* message, char* out, size_t outLen) {
    doc["cmd"] = cmd;
    doc["ok"] = ok;
    if (req && req[0]) doc["req"] = req;
    if (message && message[0]) doc[ok ? "note" : "error"] = message;
    if (serializeJson(doc, out, outLen) >= outLen) {
        snprintf(out, outLen, "{\"cmd\":\"%s\",\"ok\":%s,\"error\":\"reply too long\"}", cmd, ok ? "true" : "false");
    }
}

// "#<seq> b<boot> +<uptime>s <MM-DDTHH:MM:SSZ|-> <text>"
void formatLogLine(const NetLogEntry& e, char* out, size_t outLen) {
    char when[20] = "-";
    if (e.epoch_s) {
        const time_t t = (time_t)e.epoch_s;
        struct tm tmv;
        gmtime_r(&t, &tmv);
        strftime(when, sizeof(when), "%m-%dT%H:%M:%SZ", &tmv);
    }
    snprintf(out, outLen, "#%lu b%u +%lus %s %s", (unsigned long)e.seq, (unsigned)e.boot,
             (unsigned long)e.uptime_s, when, e.text);
}

void addNet(JsonObject net, const NetSnapshot& snap, uint32_t takenAtS) {
    // The snapshot is taken after a FAILED connect attempt, so it describes
    // that moment, not the present. age_s says how old it is.
    net["age_s"] = takenAtS ? (int32_t)(diag_uptime_s() - takenAtS) : -1;
    net["at_ok"] = snap.at_ok;
    net["sim_ready"] = snap.sim_ready;
    net["creg"] = snap.creg;
    net["cereg"] = snap.cereg;
    net["csq"] = snap.csq;
    net["stage"] = snap.stage;
    net["last_err"] = snap.last_err;
}

void execute(const char* json, const RuntimeStatus& status, char* out, size_t outLen) {
    JsonDocument request;
    JsonDocument response;
    const DeserializationError parseError = deserializeJson(request, json);
    const char* req = request["req"] | "";
    const char* cmd = request["cmd"] | "";
    if (parseError || !cmd[0]) {
        reply(response, req, cmd[0] ? cmd : "?", false, "not a JSON command", out, outLen);
        return;
    }
    const DiagBootInfo& boot = diag_info();

    if (strcmp(cmd, "status") == 0) {
        response["fw"] = FIRMWARE_VERSION;
        response["uptime_s"] = diag_uptime_s();
        response["boot_count"] = boot.boot_count;
        response["reset_reason"] = reset_reason_name(boot.reset_reason);
        response["restart_cause"] = restart_cause_name(boot.prev_cause);
        response["safe_mode"] = boot.safe_mode;
        response["ota"] = ota_state_name();
        response["mqtt"] = status.mqttConnected;
        response["modem"] = modem_state_name((ModemState)status.modemState);
        response["rssi_raw"] = status.rssiRaw;
        response["offline_s"] = status.offlineS;
        response["ladder_step"] = status.ladderStep;
        response["pzem_valid_mask"] = status.pzemValidMask;
        const StorageStats stats = storage_stats();
        JsonObject flash = response["flash"].to<JsonObject>();
        flash["ready"] = stats.ready;
        flash["pending"] = stats.depth;
        flash["capacity"] = stats.capacity;
        flash["dropped"] = stats.dropped_records;
        reply(response, req, cmd, true, nullptr, out, outLen);
    } else if (strcmp(cmd, "diag") == 0) {
        JsonObject counters = response["counters"].to<JsonObject>();
        counters["boot_count"] = boot.boot_count;
        counters["fast_boots"] = boot.fast_boots;
        counters["power_ons"] = boot.power_ons;
        counters["brownouts"] = boot.brownouts;
        counters["crashes"] = boot.crashes;
        counters["net_restarts"] = boot.net_restarts;
        counters["publish_ok"] = status.publishOk;
        counters["publish_failed"] = status.publishFailed;
        counters["reconnects"] = status.reconnects;
        counters["down_total_s"] = status.downTotalS;
        const AtMqttStats ms = at_mqtt_stats();
        JsonObject mq = response["mqtt"].to<JsonObject>();
        mq["pub_ok"] = ms.pub_ok;                 // every publish incl. acks, events, replay
        mq["pub_failed"] = ms.pub_failed;
        mq["kept_after_fail"] = ms.kept_after_fail;
        mq["session_drops"] = ms.session_drops;
        mq["connlost"] = ms.connlost;
        mq["last_pub_ms"] = ms.last_pub_ms;
        mq["max_pub_ms"] = ms.max_pub_ms;
        mq["last_drop"] = ms.last_drop;
        mq["last_probe"] = ms.last_probe;
        mq["log_lines"] = netlog_count();
        JsonObject prev = response["previous_boot"].to<JsonObject>();
        prev["restart_cause"] = restart_cause_name(boot.prev_cause);
        prev["uptime_s"] = boot.prev_uptime_s;
        prev["offline_s"] = boot.prev_offline_s;
        addNet(response["net"].to<JsonObject>(), status.net, status.netAtS);
        const StorageHealth h = storage_health();
        JsonObject flash = response["flash"].to<JsonObject>();
        flash["ready"] = h.ready;
        char jedec[12];
        snprintf(jedec, sizeof(jedec), "%02X %02X %02X", h.jedec[0], h.jedec[1], h.jedec[2]);
        flash["jedec"] = jedec;
        flash["chip_kb"] = h.chip_bytes / 1024;
        flash["corrupt_slots"] = h.corrupt_slots;
        flash["init_attempts"] = h.init_attempts;
        if (h.fail_step[0]) flash["fail"] = h.fail_step;
        JsonObject mem = response["memory"].to<JsonObject>();
        mem["free_heap"] = ESP.getFreeHeap();
        mem["min_free_heap"] = ESP.getMinFreeHeap();
        mem["largest_block"] = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
        mem["stack_free_meter"] = status.stackFreeMeter;
        mem["stack_free_network"] = status.stackFreeNetwork;
        mem["stack_free_power"] = status.stackFreePower;
        reply(response, req, cmd, true, nullptr, out, outLen);
    } else if (strcmp(cmd, "log") == 0) {
        if (request["clear"] | false) {
            netlog_clear();
            reply(response, req, cmd, true, "log cleared", out, outLen);
            return;
        }
        static char lines[LOG_PAGE][NETLOG_TEXT_MAX + 48];
        const size_t total = netlog_count();
        const size_t pages = total ? (total + LOG_PAGE - 1) / LOG_PAGE : 1;
        const size_t page = request["page"] | 0;
        response["fw"] = FIRMWARE_VERSION;
        response["uptime_s"] = diag_uptime_s();
        response["total"] = total;
        response["page"] = page;
        response["pages"] = pages;
        JsonArray arr = response["lines"].to<JsonArray>();
        for (size_t i = 0; i < LOG_PAGE; ++i) {
            NetLogEntry e{};
            if (!netlog_get(page * LOG_PAGE + i, e)) break;
            formatLogLine(e, lines[i], sizeof(lines[i]));
            arr.add((const char*)lines[i]);
        }
        reply(response, req, cmd, true, nullptr, out, outLen);
    } else if (strcmp(cmd, "reboot") == 0) {
        scheduleRestart();
        reply(response, req, cmd, true, "restarting in 4 s", out, outLen);
    } else if (strcmp(cmd, "ota") == 0) {
        char err[96];
        if (ota_request(request, err, sizeof(err))) {
            response["version"] = request["version"] | "";
            reply(response, req, cmd, true, "accepted, downloading", out, outLen);
        } else {
            reply(response, req, cmd, false, err, out, outLen);
        }
    } else {
        reply(response, req, cmd, false, "unknown command (status, diag, log, reboot, ota)", out, outLen);
    }
}

void publishAck(const char* text) {
    xSemaphoreTake(mutex, portMAX_DELAY);
    strncpy(ack, text, sizeof(ack) - 1);
    ack[sizeof(ack) - 1] = '\0';
    ackSet = true;
    xSemaphoreGive(mutex);
}

void pollSerial(const RuntimeStatus& status) {
    while (Serial.available()) {
        const char c = (char)Serial.read();
        if (c == '\n' || c == '\r') {
            if (serialLen > 0 && !serialOverflow && serialLine[0] == '{') {
                serialLine[serialLen] = '\0';
                static char out[ACK_MAX];   // metering task only; keeps 1.4 KB off its stack
                execute(serialLine, status, out, sizeof(out));
                Serial.println(out);
            } else if (serialOverflow) {
                Serial.println("{\"ok\":false,\"error\":\"line too long\"}");
            }
            serialLen = 0;
            serialOverflow = false;
        } else if (serialLen < sizeof(serialLine) - 1) {
            serialLine[serialLen++] = c;
        } else {
            serialOverflow = true;
        }
    }
}

}  // namespace

void commands_init() { if (!mutex) mutex = xSemaphoreCreateMutex(); }

void commands_on_message(const char* topic, const char* payload) {
    if (!mutex || strcmp(topic, MQTT_TOPIC_SUB) != 0) return;
    xSemaphoreTake(mutex, portMAX_DELAY);
    if (!pendingSet) {
        strncpy(pending, payload, sizeof(pending) - 1);
        pending[sizeof(pending) - 1] = '\0';
        pendingSet = true;
    }   // a second command while one is queued is dropped; the sender retries on no ack
    xSemaphoreGive(mutex);
}

void commands_process(const RuntimeStatus& status) {
    if (!mutex) return;
    pollSerial(status);

    static char message[MQTT_RX_PAYLOAD_MAX + 1];
    bool have = false;
    xSemaphoreTake(mutex, portMAX_DELAY);
    if (pendingSet) {
        memcpy(message, pending, sizeof(message));
        pendingSet = false;
        have = true;
    }
    xSemaphoreGive(mutex);
    if (!have) return;

    static char out[ACK_MAX];
    execute(message, status, out, sizeof(out));
    publishAck(out);
}

bool commands_take_ack(char* out, size_t outLen) {
    if (!mutex) return false;
    bool have = false;
    xSemaphoreTake(mutex, portMAX_DELAY);
    if (ackSet) {
        strncpy(out, ack, outLen - 1);
        out[outLen - 1] = '\0';
        ackSet = false;
        have = true;
    }
    xSemaphoreGive(mutex);
    return have;
}

bool commands_restart_due() {
    const uint32_t at = restartAtMs;
    return at != 0 && (int32_t)(millis() - at) >= 0;
}
