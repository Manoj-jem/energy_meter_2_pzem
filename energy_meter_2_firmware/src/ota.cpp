#include "ota.h"

#include <Arduino.h>
#include <Preferences.h>
#include <Update.h>
#include <esp_ota_ops.h>
#include <esp_partition.h>
#include <esp_task_wdt.h>
#include <mbedtls/base64.h>
#include <mbedtls/sha256.h>

#include "at_mqtt.h"
#include "config.h"
#include "diag.h"
#include "storage.h"

// Every image carries its meter's marker; ota_run() refuses an image without
// this meter's. The terminating NUL is part of the match, so energy-meter-2
// never accepts an image for energy-meter-30. The prefix differs from the
// gateways' "DGGW-ID:", so a gateway image can never land on a meter either.
// Referenced below, so the linker keeps it.
extern const char OTA_IDENTITY[];
const char OTA_IDENTITY[] = "EMPZ-ID:" DEVICE_ID;

namespace {

constexpr const char* NS = "ota";
// A presigned S3 URL is ~450 characters (more with a session token); the whole
// command must still fit MQTT_RX_PAYLOAD_MAX.
constexpr size_t URL_MAX = 900;
constexpr size_t MARK_LEN = sizeof(OTA_IDENTITY);
constexpr uint32_t RETRY_PAUSE_MS = 3000;

struct Request {
    char     stream[65];      // AWS IoT stream id; empty = download from url
    uint16_t streamVer;
    char     url[URL_MAX];
    uint32_t size;
    uint8_t  sha256[32];
    char     version[24];
    char     req[48];
};

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
Request pendingReq;
volatile bool pendingSet = false;
volatile bool running = false;
volatile bool onTrial = false;

// Network task only.
char ackBuf[384];
bool ackSet = false;

int nibble(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

bool parseSha256(const char* hex, uint8_t out[32]) {
    if (strlen(hex) != 64) return false;
    for (size_t i = 0; i < 32; ++i) {
        const int hi = nibble(hex[2 * i]), lo = nibble(hex[2 * i + 1]);
        if (hi < 0 || lo < 0) return false;
        out[i] = (uint8_t)(hi << 4 | lo);
    }
    return true;
}

void queueAck(const char* req, bool ok, const char* message, const char* version = nullptr) {
    JsonDocument doc;
    doc["cmd"] = "ota";
    doc["ok"] = ok;
    if (req && req[0]) doc["req"] = req;
    doc["fw"] = FIRMWARE_VERSION;
    if (version && version[0]) doc["version"] = version;
    doc[ok ? "note" : "error"] = message;
    serializeJson(doc, ackBuf, sizeof(ackBuf));
    ackSet = true;
}

// ── download ─────────────────────────────────────────────────────

struct Download {
    mbedtls_sha256_context sha;
    uint32_t    size;
    uint32_t    received;
    uint8_t     kmp[MARK_LEN];    // failure table for the marker search
    size_t      matched;
    bool        markFound;
    const char* error;
};

void markerInit(Download& d) {
    d.kmp[0] = 0;
    size_t k = 0;
    for (size_t i = 1; i < MARK_LEN; ++i) {
        while (k > 0 && OTA_IDENTITY[i] != OTA_IDENTITY[k]) k = d.kmp[k - 1];
        if (OTA_IDENTITY[i] == OTA_IDENTITY[k]) ++k;
        d.kmp[i] = (uint8_t)k;
    }
}

void markerScan(Download& d, const uint8_t* data, size_t len) {
    for (size_t i = 0; i < len && !d.markFound; ++i) {
        const char c = (char)data[i];
        while (d.matched > 0 && c != OTA_IDENTITY[d.matched]) d.matched = d.kmp[d.matched - 1];
        if (c == OTA_IDENTITY[d.matched] && ++d.matched == MARK_LEN) d.markFound = true;
    }
}

bool sink(const uint8_t* data, size_t len, void* ctx) {
    Download& d = *static_cast<Download*>(ctx);
    if (d.received + len > d.size) { d.error = "server sent more bytes than size"; return false; }
    if (Update.write(const_cast<uint8_t*>(data), len) != len) { d.error = "flash write failed"; return false; }
    mbedtls_sha256_update(&d.sha, data, len);
    markerScan(d, data, len);
    d.received += len;
    return true;
}

char detail[80];

// ── fetch over HTTP(S) through the modem's HTTP client ───────────
// MQTT is closed first: an MQTT URC would land inside the binary stream.
const char* fetchHttp(const Request& r, Download& d, bool (*powerBad)()) {
    at_mqtt_disconnect();
    const char* error = nullptr;
    uint8_t failures = 0;
    while (!error && d.received < r.size) {
        esp_task_wdt_reset();
        if (powerBad()) { error = "paused: flash backlog must be sent first"; break; }
        const uint32_t from = d.received;
        const uint32_t to = min(from + (uint32_t)OTA_CHUNK_BYTES, r.size) - 1;
        const int status = at_http_get_range(r.url, from, to, sink, &d);
        if (d.error) { error = d.error; break; }
        if (status >= 400 && status < 600) {     // expired link, missing file: retrying cannot help
            snprintf(detail, sizeof(detail), "HTTP %d", status);
            error = detail;
            break;
        }
        if (d.received > to || d.received == r.size) {
            failures = 0;
            DBGF("[OTA] %lu / %lu bytes\n", (unsigned long)d.received, (unsigned long)r.size);
        } else if (++failures > OTA_CHUNK_RETRIES) {
            snprintf(detail, sizeof(detail), "download failed at byte %lu (status %d)",
                     (unsigned long)d.received, status);
            error = detail;
        } else {
            DBGF("[OTA] Chunk at %lu failed (status %d), retry %u\n",
                 (unsigned long)d.received, status, (unsigned)failures);
            delay(RETRY_PAUSE_MS);
        }
    }
    return error;
}

// ── fetch over MQTT: AWS IoT file streams ────────────────────────
// The image comes through the existing session to MQTT_BROKER_HOST, so the
// modem opens no second TLS connection (it cannot take S3's 4.4 KB chain).
// The device asks for STREAM_WINDOW blocks at a time on .../get/json and AWS
// answers on .../data/json, one JSON message per block with the bytes in
// base64. A missing block is asked for again.
// 512-byte blocks keep each message (~740 bytes) inside MQTT_RX_PAYLOAD_MAX.
//
// Nothing is written to flash while the modem is sending. A flash write stalls
// the CPU for 10-20 ms with interrupts blocked, and the modem keeps sending at
// ~11 KB/s into a 128-byte UART FIFO: bytes were lost, the message they
// belonged to was corrupt, and because only the next block in order is accepted
// the rest of the window was wasted until the timeout (~100 bytes/s). So the
// blocks of one request are collected in RAM and written once the burst is
// over and the modem is idle.
//
// One request costs three AT exchanges and a QoS 1 round trip (~3 s on the
// SIM), so the window sets the download time.
constexpr uint32_t STREAM_BLOCK = 512;
constexpr uint32_t STREAM_WINDOW = 16;
constexpr uint32_t STREAM_WAIT_MS = 20000;       // for the first block of a request
constexpr uint32_t STREAM_IDLE_MS = 3000;        // silence after a block = the burst is over
constexpr uint8_t  STREAM_DROPS = 4;                  // connection losses survived per update
constexpr uint8_t  STREAM_RECONNECT_ATTEMPTS = 4;     // per loss
constexpr uint32_t STREAM_RECONNECT_PAUSE_MS = 15000;
constexpr uint32_t STREAM_PROGRESS_BYTES = 65536;

struct StreamState {
    bool     active;
    char     dataTopic[128];
    char     rejectedTopic[128];
    uint32_t nextBlock;       // first block not yet written to flash
    uint32_t blocks;
    uint32_t winStart;        // first block of the current request
    uint32_t winGot;          // blocks received in order since winStart, held in `window`
    uint32_t lastRxMs;
    char     rejected[64];
    Download* d;
};
StreamState st;
uint8_t window[STREAM_WINDOW * STREAM_BLOCK];

void onStreamMessage(const char* topic, const char* payload) {
    if (!st.active) return;
    JsonDocument doc;
    if (deserializeJson(doc, payload)) return;
    if (strcmp(topic, st.rejectedTopic) == 0) {
        snprintf(st.rejected, sizeof(st.rejected), "%s %s", (const char*)(doc["o"] | "?"),
                 (const char*)(doc["m"] | ""));
        return;
    }
    if (strcmp(topic, st.dataTopic) != 0) return;
    const uint32_t block = doc["i"] | UINT32_MAX;
    if (block != st.winStart + st.winGot || st.winGot >= STREAM_WINDOW) return;   // duplicate / out of order
    const char* b64 = doc["p"] | "";
    uint8_t* slot = window + st.winGot * STREAM_BLOCK;
    size_t n = 0;
    if (mbedtls_base64_decode(slot, STREAM_BLOCK, &n, (const uint8_t*)b64, strlen(b64)) != 0) return;
    const uint32_t expect = block + 1 < st.blocks ? STREAM_BLOCK : st.d->size - block * STREAM_BLOCK;
    if (n != expect) return;
    ++st.winGot;
    st.lastRxMs = millis();
}

// Writes the blocks collected for the current request. Call only while the
// modem is quiet.
void flushWindow() {
    if (st.winGot == 0) return;
    const uint32_t from = st.winStart * STREAM_BLOCK;
    const uint32_t to = min((st.winStart + st.winGot) * STREAM_BLOCK, st.d->size);
    if (sink(window, to - from, st.d)) st.nextBlock = st.winStart + st.winGot;
    st.winGot = 0;
}

// Brings MQTT back after the modem lost the session or the network, and
// re-subscribes. The blocks already written stay valid.
bool reconnectStream() {
    for (uint8_t attempt = 1; attempt <= STREAM_RECONNECT_ATTEMPTS; ++attempt) {
        esp_task_wdt_reset();
        DBGF("[OTA] Reconnecting, attempt %u of %u\n", (unsigned)attempt, (unsigned)STREAM_RECONNECT_ATTEMPTS);
        bool ok = at_mqtt_connect();
        if (!ok && gsm_modem_init()) ok = at_mqtt_connect();   // the network may have detached
        ok = ok && at_mqtt_subscribe() && at_mqtt_subscribe_topic(st.dataTopic) &&
             at_mqtt_subscribe_topic(st.rejectedTopic);
        if (ok) return true;
        for (uint32_t waited = 0; waited < STREAM_RECONNECT_PAUSE_MS; waited += 500) {
            esp_task_wdt_reset();
            delay(500);
        }
    }
    return false;
}

// Progress for whoever sent the command; best effort, never fails the update.
void publishProgress(const Request& r, const Download& d) {
    char text[160];
    snprintf(text, sizeof(text),
             "{\"cmd\":\"ota\",\"ok\":true,\"req\":\"%s\",\"note\":\"progress\",\"bytes\":%lu,\"of\":%lu}",
             r.req, (unsigned long)d.received, (unsigned long)r.size);
    at_mqtt_publish_to(MQTT_TOPIC_ACK, text);
}

const char* fetchStream(const Request& r, Download& d, bool (*powerBad)()) {
    if (!at_mqtt_is_connected()) return "MQTT not connected";
    char getTopic[128];
    const char* base = "$aws/things/" DEVICE_ID "/streams/";
    snprintf(st.dataTopic, sizeof(st.dataTopic), "%s%s/data/json", base, r.stream);
    snprintf(st.rejectedTopic, sizeof(st.rejectedTopic), "%s%s/rejected/json", base, r.stream);
    snprintf(getTopic, sizeof(getTopic), "%s%s/get/json", base, r.stream);
    st.nextBlock = 0;
    st.winStart = 0;
    st.winGot = 0;
    st.lastRxMs = 0;
    st.blocks = (r.size + STREAM_BLOCK - 1) / STREAM_BLOCK;
    st.rejected[0] = '\0';
    st.d = &d;

    // AWS drops the whole connection on a subscribe the IoT policy does not
    // allow: a failure here usually means the stream policy is not attached.
    if (!at_mqtt_subscribe_topic(st.dataTopic) || !at_mqtt_subscribe_topic(st.rejectedTopic)) {
        return "cannot subscribe to the stream topics (IoT policy for $aws/things/<thing>/streams/*?)";
    }
    st.active = true;

    const char* error = nullptr;
    uint8_t failures = 0;
    uint8_t drops = 0;
    uint32_t lastLogged = 0;
    uint32_t lastReported = 0;
    while (!error && st.nextBlock < st.blocks) {
        esp_task_wdt_reset();
        if (powerBad()) { error = "paused: flash backlog must be sent first"; break; }
        const uint32_t start = st.nextBlock;
        const uint32_t count = min(STREAM_WINDOW, st.blocks - start);
        char request[128];
        snprintf(request, sizeof(request), "{\"c\":\"em\",\"s\":%u,\"f\":0,\"l\":%lu,\"o\":%lu,\"n\":%lu}",
                 (unsigned)r.streamVer, (unsigned long)STREAM_BLOCK, (unsigned long)start, (unsigned long)count);
        st.winStart = start;
        st.winGot = 0;
        if (at_mqtt_publish_to(getTopic, request)) {
            const uint32_t deadline = millis() + STREAM_WAIT_MS;
            while (st.winGot < count && !st.rejected[0] && at_mqtt_is_connected() &&
                   (int32_t)(millis() - deadline) < 0 &&
                   !(st.winGot > 0 && millis() - st.lastRxMs > STREAM_IDLE_MS)) {
                at_mqtt_poll();
                esp_task_wdt_reset();
                delay(2);
            }
        }
        flushWindow();                    // the burst is over: now it is safe to stall on flash
        if (d.error) { error = d.error; break; }
        if (st.rejected[0]) {
            snprintf(detail, sizeof(detail), "stream rejected: %s", st.rejected);
            error = detail;
        } else if (!at_mqtt_is_connected()) {
            // Resume where it stopped.
            DBGF("[OTA] MQTT lost at byte %lu (loss %u of %u)\n", (unsigned long)d.received,
                 (unsigned)(drops + 1), (unsigned)STREAM_DROPS);
            if (++drops > STREAM_DROPS) {
                snprintf(detail, sizeof(detail), "MQTT lost %u times, gave up at byte %lu",
                         (unsigned)drops, (unsigned long)d.received);
                error = detail;
            } else if (!reconnectStream()) {
                snprintf(detail, sizeof(detail), "MQTT lost at byte %lu, reconnect failed", (unsigned long)d.received);
                error = detail;
            } else {
                failures = 0;
            }
        } else if (st.nextBlock > start) {
            failures = 0;
            if (d.received - lastLogged >= 32768 || d.received == r.size) {
                lastLogged = d.received;
                DBGF("[OTA] %lu / %lu bytes\n", (unsigned long)d.received, (unsigned long)r.size);
            }
            if (d.received - lastReported >= STREAM_PROGRESS_BYTES && d.received < r.size) {
                lastReported = d.received;
                publishProgress(r, d);
            }
        } else if (++failures > OTA_CHUNK_RETRIES) {
            snprintf(detail, sizeof(detail), "no data from stream at byte %lu", (unsigned long)d.received);
            error = detail;
        } else {
            DBGF("[OTA] No block %lu from the stream, retry %u\n", (unsigned long)start, (unsigned)failures);
        }
    }
    st.active = false;
    at_mqtt_unsubscribe_topic(st.dataTopic);
    at_mqtt_unsubscribe_topic(st.rejectedTopic);
    return error;
}

// Downloads and installs the image. Returns only on failure, with the reason.
const char* install(const Request& r, bool (*powerBad)()) {
    static Download d;

    if (powerBad()) return "flash backlog too large: retry after the replay";
    DBGF("[OTA] %s -> %s: %lu bytes from %s %s\n", FIRMWARE_VERSION, r.version, (unsigned long)r.size,
         r.stream[0] ? "stream" : "url", r.stream[0] ? r.stream : r.url);

    const esp_partition_t* target = esp_ota_get_next_update_partition(nullptr);
    if (!target || !Update.begin(r.size)) {
        snprintf(detail, sizeof(detail), "cannot open the OTA slot: %s", Update.errorString());
        return detail;
    }

    d = Download{};
    d.size = r.size;
    markerInit(d);
    mbedtls_sha256_init(&d.sha);
    mbedtls_sha256_starts(&d.sha, 0);

    const char* error = r.stream[0] ? fetchStream(r, d, powerBad) : fetchHttp(r, d, powerBad);

    uint8_t digest[32];
    mbedtls_sha256_finish(&d.sha, digest);
    mbedtls_sha256_free(&d.sha);

    if (!error && memcmp(digest, r.sha256, sizeof(digest)) != 0) error = "sha256 mismatch";
    if (!error && !d.markFound) error = "image is not built for this meter (identity marker missing)";
    if (error) {
        Update.abort();
        return error;
    }
    if (!Update.end()) {        // also validates the image and selects it for the next boot
        snprintf(detail, sizeof(detail), "image rejected: %s", Update.errorString());
        return detail;
    }

    Preferences prefs;
    if (prefs.begin(NS, false)) {
        prefs.putBool("trial", true);
        prefs.putUChar("boots", 0);
        prefs.putString("ver", r.version);
        prefs.putString("part", target->label);
        prefs.putString("prev", esp_ota_get_running_partition()->label);
        prefs.putString("req", r.req);
        prefs.remove("rolled");
        prefs.end();
    }
    DBGF("[OTA] Verified and installed in %s: restarting into %s\n", target->label, r.version);
    diag_restart(RC_OTA_APPLY);
}

}  // namespace

void ota_boot_check() {
    Preferences prefs;
    if (!prefs.begin(NS, false)) return;
    if (!prefs.getBool("trial", false)) { prefs.end(); return; }

    char ver[24] = "", part[17] = "", prev[17] = "";
    prefs.getString("ver", ver, sizeof(ver));
    prefs.getString("part", part, sizeof(part));
    prefs.getString("prev", prev, sizeof(prev));

    // Not running the new image: the bootloader already went back, because the
    // new image crashed before Arduino could mark it valid.
    if (strcmp(esp_ota_get_running_partition()->label, part) != 0) {
        DBGF("[OTA] Firmware %s never started; the bootloader restored this image\n", ver);
        prefs.putBool("trial", false);
        prefs.putString("rolled", ver);
        prefs.end();
        return;
    }

    const uint8_t boots = prefs.getUChar("boots", 0) + 1;
    if (boots > OTA_TRIAL_MAX_BOOTS) {
        prefs.putBool("trial", false);
        const esp_partition_t* back =
            esp_partition_find_first(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_ANY, prev);
        if (back && esp_ota_set_boot_partition(back) == ESP_OK) {
            prefs.putString("rolled", ver);
            prefs.end();
            DBGF("[OTA] Firmware %s never reached AWS in %u boots: rolling back to %s\n",
                 ver, (unsigned)OTA_TRIAL_MAX_BOOTS, prev);
            diag_restart(RC_OTA_ROLLBACK);
        }
        prefs.end();
        DBGF("[OTA] Rollback impossible (%s not bootable): keeping %s\n", prev, ver);
        return;
    }
    prefs.putUChar("boots", boots);
    prefs.end();
    onTrial = true;
    DBGF("[OTA] Firmware %s on trial, boot %u of %u\n", ver, (unsigned)boots, (unsigned)OTA_TRIAL_MAX_BOOTS);
}

bool validStreamId(const char* id) {
    const size_t len = strlen(id);
    if (len == 0 || len >= sizeof(Request::stream)) return false;
    for (size_t i = 0; i < len; ++i) {
        if (!isalnum((unsigned char)id[i]) && id[i] != '_' && id[i] != '-') return false;
    }
    return true;
}

bool ota_request(JsonDocument& request, char* err, size_t errLen) {
    const char* stream = request["stream"] | "";
    const char* url = request["url"] | "";
    const uint32_t streamVer = request["stream_ver"] | 1U;
    const uint32_t size = request["size"] | 0U;
    const char* version = request["version"] | "";
    const bool force = request["force"] | false;
    const esp_partition_t* slot = esp_ota_get_next_update_partition(nullptr);

    Request r{};
    const char* problem = nullptr;
    if (stream[0]) {
        if (!validStreamId(stream)) problem = "stream id must be 1-64 of A-Z a-z 0-9 _ -";
        else if (streamVer == 0 || streamVer > 65535) problem = "stream_ver out of range";
    } else if (strncmp(url, "http://", 7) != 0 && strncmp(url, "https://", 8) != 0) {
        problem = "need \"stream\", or a url starting with http:// or https://";
    } else if (strlen(url) >= URL_MAX || strchr(url, '"')) {
        problem = "url too long or contains a quote";
    }
    if (problem) {
        // already explained
    } else if (!slot) {
        problem = "no OTA partition";
    } else if (size == 0 || size > slot->size) {
        problem = "size missing or larger than the OTA slot";
    } else if (!parseSha256(request["sha256"] | "", r.sha256)) {
        problem = "sha256 must be 64 hex characters";
    } else if (!version[0] || strlen(version) >= sizeof(r.version)) {
        problem = "version missing or too long";
    } else if (!force && strcmp(version, FIRMWARE_VERSION) == 0) {
        problem = "already running this version (add \"force\":true)";
    }
    if (problem) {
        snprintf(err, errLen, "%s", problem);
        return false;
    }

    strncpy(r.stream, stream, sizeof(r.stream) - 1);
    r.streamVer = (uint16_t)streamVer;
    if (!stream[0]) strncpy(r.url, url, sizeof(r.url) - 1);
    strncpy(r.version, version, sizeof(r.version) - 1);
    strncpy(r.req, request["req"] | "", sizeof(r.req) - 1);
    r.size = size;

    portENTER_CRITICAL(&mux);
    const bool busy = pendingSet || running;
    if (!busy) {
        pendingReq = r;
        pendingSet = true;
    }
    portEXIT_CRITICAL(&mux);
    if (busy) snprintf(err, errLen, "an update is already in progress");
    return !busy;
}

bool ota_pending() { return pendingSet; }

void ota_on_message(const char* topic, const char* payload) { onStreamMessage(topic, payload); }

void ota_run(bool (*powerBad)()) {
    static Request r;
    portENTER_CRITICAL(&mux);
    const bool have = pendingSet;
    if (have) {
        r = pendingReq;
        pendingSet = false;
        running = true;
    }
    portEXIT_CRITICAL(&mux);
    if (!have) return;

    const char* error = install(r, powerBad);
    DBGF("[OTA] Update to %s failed: %s\n", r.version, error);
    queueAck(r.req, false, error, r.version);
    running = false;
}

void ota_on_connected() {
    static bool reported = false;
    if (reported) return;
    reported = true;

    Preferences prefs;
    if (!prefs.begin(NS, false)) return;
    char req[48] = "";
    prefs.getString("req", req, sizeof(req));
    if (onTrial) {
        prefs.putBool("trial", false);
        onTrial = false;
        DBGF("[OTA] Firmware %s reached AWS: update confirmed\n", FIRMWARE_VERSION);
        queueAck(req, true, "running new firmware");
    } else if (prefs.isKey("rolled")) {
        char ver[24] = "";
        prefs.getString("rolled", ver, sizeof(ver));
        prefs.remove("rolled");
        char message[64];
        snprintf(message, sizeof(message), "rolled back from %s", ver);
        queueAck(req, false, message, ver);
    }
    prefs.end();
}

void ota_trial_tick() {
    if (onTrial && millis() > OTA_TRIAL_CONNECT_MS) {
        DBGLN("[OTA] Trial firmware has not reached AWS in time: restarting");
        diag_restart(RC_OTA_TRIAL_FAIL);
    }
}

bool ota_take_ack(char* out, size_t outLen) {
    if (!ackSet) return false;
    strncpy(out, ackBuf, outLen - 1);
    out[outLen - 1] = '\0';
    ackSet = false;
    return true;
}

const char* ota_state_name() {
    if (running) return "running";
    if (pendingSet) return "pending";
    if (onTrial) return "trial";
    return "idle";
}
