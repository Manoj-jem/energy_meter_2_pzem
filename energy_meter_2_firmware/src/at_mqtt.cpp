
#include <Arduino.h>
#include <Preferences.h>
#include "config.h"
#include "at_mqtt.h"
#include "certs_generated.h"

static HardwareSerial gsm(1);
static volatile bool _connected  = false;
static volatile bool _publishing = false;
static at_mqtt_message_cb _msgCallback = nullptr;


static unsigned long _lastPingMs = 0;
static const unsigned long PING_INTERVAL_MS = 45000UL;

static char   _rxTopic[256]   = {0};
static char   _rxPayload[512] = {0};
static size_t _rxTopicLen     = 0;
static size_t _rxPayloadLen   = 0;
static bool   _rxInProgress   = false;

const char* at_mqtt_err_string(int errcode) {
    switch (errcode) {
        case 0:  return "operation succeeded";
        case 1:  return "failed";
        case 3:  return "sock connect fail";
        case 11: return "no connection";
        case 12: return "invalid parameter";
        case 14: return "client is busy";
        case 17: return "timeout";
        case 19: return "client is used (slot already acquired)";
        case 20: return "client not acquired";
        case 25: return "DNS error";
        case 27: return "connection refused: protocol version";
        case 30: return "connection refused: bad credentials";
        case 31: return "connection refused: not authorized";
        case 32: return "handshake fail";
        case 33: return "not set certificate";
        case 34: return "open session failed";
        default: return "unknown error code";
    }
}

static bool _read_line(char* out, size_t outLen, uint32_t timeoutMs) {
    size_t   pos      = 0;
    uint32_t deadline = millis() + timeoutMs;
    while (millis() < deadline) {
        while (gsm.available()) {
            char c = (char)gsm.read();
            if (c == '\n') {
                out[pos] = '\0';
                if (pos > 0) { DBG("[AT] << "); DBGLN(out); return true; }
                pos = 0;
            } else if (c != '\r') {
                if (pos < outLen - 1) out[pos++] = c;
            }
        }
        delay(2);
    }
    return false;
}

static bool _at_cmd(const char* cmd, uint32_t timeoutMs,
                     const char* expectOk  = "OK",
                     const char* expectErr = "ERROR") {
    while (gsm.available()) gsm.read();
    DBG("[AT] >> "); DBGLN(cmd);
    gsm.print(cmd); gsm.print("\r\n");

    char line[256];
    uint32_t deadline = millis() + timeoutMs;
    while (millis() < deadline) {
        if (_read_line(line, sizeof(line), 200)) {
            if (strstr(line, expectOk))              return true;
            if (expectErr && strstr(line, expectErr)) return false;
        }
    }
    DBGF("[AT] Timeout after: %s\n", cmd);
    return false;
}

static void _parse_urc_err(char* line, int* errOut) {
    char* comma = strrchr(line, ',');
    if (comma) {
        *errOut = atoi(comma + 1);
        return;
    }
    char* colon = strrchr(line, ':');
    if (colon) {
        char* p = colon + 1;
        while (*p == ' ') p++;
        *errOut = atoi(p);
    } else {
        *errOut = -1;
    }
}

static bool _at_cmd_two_stage(const char* cmd, const char* urcPrefix,
                               uint32_t timeoutMs, int* errOut) {
    while (gsm.available()) gsm.read();
    DBG("[AT] >> "); DBGLN(cmd);
    gsm.print(cmd); gsm.print("\r\n");

    char line[256];
    uint32_t deadline = millis() + timeoutMs;
    bool gotOk  = false;
    bool gotUrc = false;

    while (millis() < deadline && !gotOk) {
        if (_read_line(line, sizeof(line), 200)) {
            // The URC can arrive *before* the final result code. A failed
            // CMQTTCONNECT emits "+CMQTTCONNECT: 0,<err>" and then ERROR —
            // capture the error code before the ERROR branch discards it,
            // otherwise a real diagnosis is reported as "no response".
            if (!gotUrc && strstr(line, urcPrefix)) {
                _parse_urc_err(line, errOut);
                gotUrc = true;
            }
            if (strstr(line, "ERROR")) {
                if (gotUrc) {
                    DBGF("[AT] ERROR after %s err=%d\n", urcPrefix, *errOut);
                    return true;
                }
                DBGLN("[AT] ERROR (stage 1)");
                return false;
            }
            if (strstr(line, "OK")) gotOk = true;
        }
    }
    if (gotUrc) { DBGF("[AT] two-stage result: err=%d\n", *errOut); return true; }
    if (!gotOk) { DBGLN("[AT] No OK (stage 1 timeout)"); return false; }

    while (millis() < deadline) {
        if (_read_line(line, sizeof(line), 200)) {
            if (strstr(line, urcPrefix)) {
                _parse_urc_err(line, errOut);
                DBGF("[AT] two-stage result: err=%d\n", *errOut);
                return true;
            }
        }
    }
    DBGF("[AT] Timeout for %s (stage 2)\n", urcPrefix);
    return false;
}

// ── Modem certificate store ─────────────────────────────────────────
//
// AT+CSSLCFG="cacert"/"clientcert"/"clientkey" only record a filename; they
// return OK whether or not the file exists in the module's cert store. A
// missing, misnamed or *stale* file surfaces much later as CMQTTCONNECT
// err=32 (handshake fail), which says nothing about which of a dozen causes
// it was. So the firmware owns the cert store outright: it checks what is
// there on every connect attempt and uploads the PEMs compiled into this
// build whenever they don't match.

// AT+CCERTLIST reports filenames only, never content, so a modem holding the
// WRONG certificate under the RIGHT filename is otherwise undetectable — the
// exact failure that stalled bring-up. Record which certificate we wrote, and
// re-provision when the build's certificate differs from it.
static const char* NVS_NAMESPACE   = "certs";
static const char* NVS_KEY_FP      = "fp";

// Consecutive TLS handshake failures (err 32/33/34) since the last success.
// The NVS fingerprint records which PEMs this firmware SENT, not that the
// modem still holds them intact, so after TLS_FAILS_BEFORE_REUPLOAD failures
// in a row the record is dropped and the next attempt rewrites all three.
static uint8_t       _tlsFailStreak = 0;
static const uint8_t TLS_FAILS_BEFORE_REUPLOAD = 2;

static void _invalidate_cert_record() {
    Preferences prefs;
    if (prefs.begin(NVS_NAMESPACE, false)) {
        prefs.remove(NVS_KEY_FP);
        prefs.end();
    }
}

static bool _cert_list(bool* haveCa, bool* haveCert, bool* haveKey) {
    *haveCa = *haveCert = *haveKey = false;

    while (gsm.available()) gsm.read();
    DBGLN("[AT] >> AT+CCERTLIST");
    gsm.print("AT+CCERTLIST\r\n");

    bool done = false;
    char line[256];
    uint32_t deadline = millis() + AT_DEFAULT_TIMEOUT_MS;
    while (millis() < deadline && !done) {
        if (!_read_line(line, sizeof(line), 200)) continue;
        if (strstr(line, CERT_FILENAME_CA))   *haveCa   = true;
        if (strstr(line, CERT_FILENAME_CERT)) *haveCert = true;
        if (strstr(line, CERT_FILENAME_KEY))  *haveKey  = true;
        if (strstr(line, "OK") || strstr(line, "ERROR")) done = true;
    }
    return done;
}

// AT+CCERTDOWN=<name>,<len> -> ">" prompt -> exactly <len> raw bytes -> OK.
static bool _cert_download(const char* filename, const char* pem, size_t len) {
    char cmd[96];

    // A file already under this name would otherwise be kept; the modem does
    // not overwrite. ERROR here just means "nothing to delete" — not a failure.
    snprintf(cmd, sizeof(cmd), "AT+CCERTDELE=\"%s\"", filename);
    _at_cmd(cmd, AT_DEFAULT_TIMEOUT_MS);

    snprintf(cmd, sizeof(cmd), "AT+CCERTDOWN=\"%s\",%u", filename, (unsigned)len);
    while (gsm.available()) gsm.read();
    DBG("[AT] >> "); DBGLN(cmd);
    gsm.print(cmd); gsm.print("\r\n");

    bool gotPrompt = false;
    uint32_t deadline = millis() + AT_DEFAULT_TIMEOUT_MS;
    while (millis() < deadline && !gotPrompt) {
        if (gsm.available() && (char)gsm.read() == '>') gotPrompt = true;
    }
    if (!gotPrompt) {
        DBGF("[CERT] %s: no '>' prompt\n", filename);
        return false;
    }

    DBGF("[CERT] %s: writing %u bytes...\n", filename, (unsigned)len);
    gsm.write((const uint8_t*)pem, len);
    gsm.flush();   // drain the ESP32 TX FIFO before waiting on the reply

    char line[256];
    deadline = millis() + AT_PUB_TIMEOUT_MS;   // flash write can take seconds
    while (millis() < deadline) {
        if (!_read_line(line, sizeof(line), 200)) continue;
        if (strstr(line, "OK"))    { DBGF("[CERT] %s: stored\n", filename); return true; }
        if (strstr(line, "ERROR")) break;
    }
    DBGF("[CERT] %s: write FAILED\n", filename);
    return false;
}

bool at_mqtt_provision_certs() {
    Preferences prefs;
    char storedFp[80] = {0};
    if (prefs.begin(NVS_NAMESPACE, true)) {          // read-only
        prefs.getString(NVS_KEY_FP, storedFp, sizeof(storedFp));
        prefs.end();
    }

    bool haveCa, haveCert, haveKey;
    if (!_cert_list(&haveCa, &haveCert, &haveKey)) {
        DBGLN("[CERT] AT+CCERTLIST gave no reply — modem not ready");
        return false;
    }

    const bool allPresent = haveCa && haveCert && haveKey;
    const bool fpMatches  = (strcmp(storedFp, CLIENT_CERT_SHA256) == 0);

    if (allPresent && fpMatches) {
        DBGF("[CERT] Store OK — %s serial=%s\n", CERT_DEVICE_NAME, CLIENT_CERT_SERIAL);
        DBGF("[CERT]   fingerprint %s\n", CLIENT_CERT_SHA256);
        return true;
    }

    DBGLN("[CERT] *** RE-PROVISIONING MODEM CERT STORE ***");
    if (!allPresent) {
        DBGF("[CERT]   reason: files missing — ca(%s)=%s cert(%s)=%s key(%s)=%s\n",
             CERT_FILENAME_CA,   haveCa   ? "found" : "MISSING",
             CERT_FILENAME_CERT, haveCert ? "found" : "MISSING",
             CERT_FILENAME_KEY,  haveKey  ? "found" : "MISSING");
    } else {
        DBGF("[CERT]   reason: stale certificate on modem\n");
        DBGF("[CERT]     modem has : %s\n", storedFp[0] ? storedFp : "(never provisioned by this firmware)");
        DBGF("[CERT]     build has : %s\n", CLIENT_CERT_SHA256);
    }
    DBGF("[CERT]   uploading %s serial=%s valid %s -> %s\n",
         CERT_DEVICE_NAME, CLIENT_CERT_SERIAL,
         CLIENT_CERT_NOT_BEFORE, CLIENT_CERT_NOT_AFTER);

    bool ok = _cert_download(CERT_FILENAME_CA,   ROOT_CA_PEM,     ROOT_CA_PEM_LEN);
    ok = _cert_download(CERT_FILENAME_CERT, CLIENT_CERT_PEM, CLIENT_CERT_PEM_LEN) && ok;
    ok = _cert_download(CERT_FILENAME_KEY,  CLIENT_KEY_PEM,  CLIENT_KEY_PEM_LEN)  && ok;
    if (!ok) {
        DBGLN("[CERT] One or more uploads failed — cert store not usable");
        return false;
    }

    // Trust the modem's own listing, not our three OKs, before recording success.
    if (!_cert_list(&haveCa, &haveCert, &haveKey) || !(haveCa && haveCert && haveKey)) {
        DBGLN("[CERT] Verification listing incomplete after upload");
        return false;
    }

    if (prefs.begin(NVS_NAMESPACE, false)) {         // read-write
        prefs.putString(NVS_KEY_FP, CLIENT_CERT_SHA256);
        prefs.end();
    } else {
        // Not fatal: the certs are on the modem and this connect will work.
        // It only means the next boot re-uploads them instead of skipping.
        DBGLN("[CERT] Warning: could not persist fingerprint to NVS");
    }

    DBGF("[CERT] Provisioned OK — fingerprint %s\n", CLIENT_CERT_SHA256);
    return true;
}

static void _mqtt_teardown() {
    DBGLN("[MQTT] Tearing down previous session...");
    _at_cmd("AT+CMQTTDISC=0,10", 3000);
    delay(200);
    _at_cmd("AT+CMQTTREL=0",     3000);
    delay(200);
    _at_cmd("AT+CMQTTSTOP",      3000);
    delay(500);
    DBGLN("[MQTT] Teardown done");
}

bool gsm_modem_init() {
    gsm.begin(GSM_UART_BAUD, SERIAL_8N1, GSM_RX_PIN, GSM_TX_PIN);
    delay(100);

    if (GSM_PWRKEY_PIN >= 0) {
        pinMode(GSM_PWRKEY_PIN, OUTPUT);
        digitalWrite(GSM_PWRKEY_PIN, HIGH); delay(100);
        digitalWrite(GSM_PWRKEY_PIN, LOW);  delay(1200);
        digitalWrite(GSM_PWRKEY_PIN, HIGH); delay(3000);
    }

    DBGLN("[GSM] Waking modem...");
    bool awake = false;
    for (int i = 0; i < 10 && !awake; i++) {
        awake = _at_cmd("AT", AT_DEFAULT_TIMEOUT_MS);
        if (!awake) delay(1000);
    }
    if (!awake) { DBGLN("[GSM] No AT response"); return false; }

    _at_cmd("AT+CMEE=2", AT_DEFAULT_TIMEOUT_MS);
    _at_cmd("ATI", AT_DEFAULT_TIMEOUT_MS);   // model + firmware revision, for support logs

    DBGLN("[GSM] Waiting for GPRS...");
    bool attached = false;
    char line[128];
    for (int i = 0; i < 15 && !attached; i++) {
        while (gsm.available()) gsm.read();
        gsm.print("AT+CGATT?\r\n");
        uint32_t d = millis() + AT_DEFAULT_TIMEOUT_MS;
        while (millis() < d) {
            if (_read_line(line, sizeof(line), 200)) {
                if (strstr(line, "+CGATT: 1")) { attached = true; break; }
                if (strstr(line, "OK") || strstr(line, "ERROR")) break;
            }
        }
        if (!attached) delay(1000);
    }
    if (!attached) { DBGLN("[GSM] GPRS not attached"); return false; }
    DBGLN("[GSM] GPRS attached");
    return true;
}

bool at_mqtt_connect() {
    char cmdBuf[300];
    int  err = -1;

    _mqtt_teardown();

    // Blocking: without a usable cert store the handshake cannot succeed, so
    // fail fast and let networkTask's backoff retry rather than burning the
    // 20 s CMQTTCONNECT timeout on a connection that is already doomed.
    if (!at_mqtt_provision_certs()) {
        DBGLN("[MQTT] Cert store not usable — aborting connect attempt");
        return false;
    }

    DBGF("[MQTT] Identity: client_id=%s thing=%s device_id=%s topic=%s\n",
         MQTT_CLIENT_ID, AWS_THING_NAME, DEVICE_ID, MQTT_TOPIC_PUB);

    DBGLN("[MQTT] Configuring SSL (mTLS)...");
    // 4 = "ALL": the module negotiates, and against AWS IoT that lands on
    // TLS 1.2. This is exactly what energy_meter_firmware (meter 001) sends in
    // production on the same SIMCom module family. Forcing 3 (TLS 1.2 only)
    // gave err 32 on every attempt during bring-up, against an endpoint that
    // accepts every TLS 1.1/1.2 cipher, group and signature algorithm and
    // serves the same chain as meter 001's (openssl s_client, 2026-09-11).
    // AWS does not reject a negotiating ClientHello.
    if (!_at_cmd("AT+CSSLCFG=\"sslversion\",0,4",   AT_DEFAULT_TIMEOUT_MS)) return false;
    if (!_at_cmd("AT+CSSLCFG=\"authmode\",0,2",     AT_DEFAULT_TIMEOUT_MS)) return false;
    if (!_at_cmd("AT+CSSLCFG=\"enableSNI\",0,1",    AT_DEFAULT_TIMEOUT_MS)) return false;
    // Without NITZ the modem RTC can sit in the past, which fails the server
    // cert's validity window and also presents as err 32. "ignorelocaltime" is
    // the SIM7600 spelling, "ignorertctime" the A76XX (A7670C/SIM7670C) one —
    // send both, non-fatal, and let the module ignore whichever it lacks.
    _at_cmd("AT+CSSLCFG=\"ignorelocaltime\",0,1",   AT_DEFAULT_TIMEOUT_MS);
    _at_cmd("AT+CSSLCFG=\"ignorertctime\",0,1",     AT_DEFAULT_TIMEOUT_MS);

    snprintf(cmdBuf, sizeof(cmdBuf), "AT+CSSLCFG=\"cacert\",0,\"%s\"",     CERT_FILENAME_CA);
    if (!_at_cmd(cmdBuf, AT_DEFAULT_TIMEOUT_MS)) { DBGLN("[MQTT] cacert failed");     return false; }
    snprintf(cmdBuf, sizeof(cmdBuf), "AT+CSSLCFG=\"clientcert\",0,\"%s\"", CERT_FILENAME_CERT);
    if (!_at_cmd(cmdBuf, AT_DEFAULT_TIMEOUT_MS)) { DBGLN("[MQTT] clientcert failed"); return false; }
    snprintf(cmdBuf, sizeof(cmdBuf), "AT+CSSLCFG=\"clientkey\",0,\"%s\"",  CERT_FILENAME_KEY);
    if (!_at_cmd(cmdBuf, AT_DEFAULT_TIMEOUT_MS)) { DBGLN("[MQTT] clientkey failed");  return false; }

    DBGLN("[MQTT] Starting CMQTT service...");
    if (_at_cmd_two_stage("AT+CMQTTSTART", "+CMQTTSTART:", AT_DEFAULT_TIMEOUT_MS * 2, &err)) {
        if (err != 0) { DBGF("[MQTT] CMQTTSTART failed err=%d\n", err); return false; }
        DBGLN("[MQTT] CMQTT service started");
    } else {
        DBGLN("[MQTT] CMQTTSTART ERROR — proceeding");
    }
    delay(300);

    snprintf(cmdBuf, sizeof(cmdBuf), "AT+CMQTTACCQ=0,\"%s\",1", MQTT_CLIENT_ID);
    if (!_at_cmd(cmdBuf, AT_DEFAULT_TIMEOUT_MS)) { DBGLN("[MQTT] CMQTTACCQ failed"); return false; }

    if (!_at_cmd("AT+CMQTTSSLCFG=0,0", AT_DEFAULT_TIMEOUT_MS)) { DBGLN("[MQTT] CMQTTSSLCFG failed"); return false; }

    snprintf(cmdBuf, sizeof(cmdBuf),
             "AT+CMQTTCONNECT=0,\"tcp://%s:%d\",%d,1",
             MQTT_BROKER_HOST, MQTT_BROKER_PORT, MQTT_KEEPALIVE_S);

    DBGLN("[MQTT] Connecting to AWS IoT...");
    if (!_at_cmd_two_stage(cmdBuf, "+CMQTTCONNECT:", AT_CONNECT_TIMEOUT_MS, &err)) {
        DBGLN("[MQTT] CMQTTCONNECT no response"); return false;
    }
    if (err != 0) {
        DBGF("[MQTT] Connect failed err=%d (%s)\n", err, at_mqtt_err_string(err));
        // err 32/33/34 all mean "the TLS handshake never completed", which has
        // several unrelated causes the modem cannot distinguish. Spell them out
        // so the next serial log is self-diagnosing.
        if (err == 32 || err == 33 || err == 34) {
            DBGLN("[MQTT]   TLS failed before MQTT started. Check, in order:");
            DBGF ("[MQTT]     1. cert on modem = %s (this build)\n", CLIENT_CERT_SHA256);
            DBGLN("[MQTT]        compare with the AWS IoT console's certificate fingerprint");
            DBGLN("[MQTT]     2. that certificate is ACTIVE and attached to a policy AND the thing");
            DBGF ("[MQTT]     3. modem clock — cert is not valid before %s\n", CLIENT_CERT_NOT_BEFORE);
            DBGLN("[MQTT]     4. endpoint is the -ats endpoint (chains to Amazon Root CA 1)");
            _at_cmd("AT+CCLK?", AT_DEFAULT_TIMEOUT_MS);   // modem clock, for item 3

            if (++_tlsFailStreak >= TLS_FAILS_BEFORE_REUPLOAD) {
                DBGF("[CERT] %u TLS failures in a row - discarding the NVS record so "
                     "the next attempt re-uploads all three files\n", (unsigned)_tlsFailStreak);
                _invalidate_cert_record();
                _tlsFailStreak = 0;
            }
        } else if (err == 30 || err == 31) {
            DBGLN("[MQTT]   TLS succeeded; the BROKER rejected the CONNECT. This is a");
            DBGLN("[MQTT]   policy/identity problem, not a certificate problem:");
            DBGF ("[MQTT]     client_id '%s' must match the thing name the IoT policy\n", MQTT_CLIENT_ID);
            DBGLN("[MQTT]     scopes iot:Connect to (client/${iot:Connection.Thing.ThingName})");
        }
        return false;
    }

    DBGLN("[MQTT] Connected to AWS IoT");
    _tlsFailStreak = 0;
    _connected  = true;
    _lastPingMs = millis();
    return true;
}

bool at_mqtt_publish(const char* payload) {
    if (!_connected) { DBGLN("[MQTT] Not connected"); return false; }
    
        // Flush any stale bytes in the modem UART buffer — these can
    // arrive from electrical noise during PZEM UART remapping
    while (gsm.available()) gsm.read();
    delay(20);   // brief settle before sending AT commands


    _publishing = true;

    char   cmdBuf[64];
    size_t topicLen   = strlen(MQTT_TOPIC_PUB);
    size_t payloadLen = strlen(payload);
    char   line[256];
    bool   result = false;
  
    snprintf(cmdBuf, sizeof(cmdBuf), "AT+CMQTTTOPIC=0,%u", (unsigned)topicLen);
    while (gsm.available()) gsm.read();
    DBG("[AT] >> "); DBGLN(cmdBuf);
    gsm.print(cmdBuf); gsm.print("\r\n");

    bool gotPrompt = false;
    uint32_t d = millis() + AT_DEFAULT_TIMEOUT_MS;
    while (millis() < d) {
        if (gsm.available() && (char)gsm.read() == '>') { gotPrompt = true; break; }
    }
    if (!gotPrompt) {
        DBGLN("[MQTT] No > for TOPIC — session dead, forcing reconnect");
        _connected = false;
        goto publish_done;
    }
    gsm.print(MQTT_TOPIC_PUB);
    if (!_read_line(line, sizeof(line), AT_DEFAULT_TIMEOUT_MS) || !strstr(line, "OK"))
        { DBGLN("[MQTT] Topic rejected"); _connected = false; goto publish_done; }


    snprintf(cmdBuf, sizeof(cmdBuf), "AT+CMQTTPAYLOAD=0,%u", (unsigned)payloadLen);
    while (gsm.available()) gsm.read();
    DBG("[AT] >> "); DBGLN(cmdBuf);
    gsm.print(cmdBuf); gsm.print("\r\n");

    gotPrompt = false;
    d = millis() + AT_DEFAULT_TIMEOUT_MS;
    while (millis() < d) {
        if (gsm.available() && (char)gsm.read() == '>') { gotPrompt = true; break; }
    }
    if (!gotPrompt) {
        DBGLN("[MQTT] No > for PAYLOAD — session dead, forcing reconnect");
        _connected = false;
        goto publish_done;
    }
    gsm.print(payload);
    if (!_read_line(line, sizeof(line), AT_DEFAULT_TIMEOUT_MS) || !strstr(line, "OK"))
        { DBGLN("[MQTT] Payload rejected"); goto publish_done; }

  
    {
        int err = -1;
        snprintf(cmdBuf, sizeof(cmdBuf), "AT+CMQTTPUB=0,%d,60", MQTT_QOS);
        if (!_at_cmd_two_stage(cmdBuf, "+CMQTTPUB:", AT_PUB_TIMEOUT_MS, &err)) {
            DBGLN("[MQTT] CMQTTPUB no response — session dead, forcing reconnect");
            _connected = false;
            goto publish_done;
        }
        if (err == 0) {
            DBGLN("[MQTT] Publish OK");
            _lastPingMs = millis();
            result = true;
        } else {
            DBGF("[MQTT] Publish failed err=%d (%s)\n", err, at_mqtt_err_string(err));
        }
    }

publish_done:
    _publishing = false;
    return result;
}

bool at_mqtt_subscribe() {
    if (!_connected) { DBGLN("[MQTT] Not connected"); return false; }

    char   cmdBuf[64];
    size_t topicLen = strlen(MQTT_TOPIC_SUB);
    char   line[256];

    snprintf(cmdBuf, sizeof(cmdBuf), "AT+CMQTTSUBTOPIC=0,%u,%d",
             (unsigned)topicLen, MQTT_QOS);
    while (gsm.available()) gsm.read();
    DBG("[AT] >> "); DBGLN(cmdBuf);
    gsm.print(cmdBuf); gsm.print("\r\n");

    bool gotPrompt = false;
    uint32_t d = millis() + AT_DEFAULT_TIMEOUT_MS;
    while (millis() < d) {
        if (gsm.available() && (char)gsm.read() == '>') { gotPrompt = true; break; }
    }
    if (!gotPrompt) { DBGLN("[MQTT] No > for SUBTOPIC"); return false; }
    gsm.print(MQTT_TOPIC_SUB);
    if (!_read_line(line, sizeof(line), AT_DEFAULT_TIMEOUT_MS) || !strstr(line, "OK"))
        { DBGLN("[MQTT] Subtopic rejected"); return false; }

    int err = -1;
    if (!_at_cmd_two_stage("AT+CMQTTSUB=0", "+CMQTTSUB:", AT_DEFAULT_TIMEOUT_MS, &err)) {
        DBGLN("[MQTT] CMQTTSUB no response"); return false;
    }
    if (err == 0) { DBGF("[MQTT] Subscribed to %s\n", MQTT_TOPIC_SUB); return true; }
    DBGF("[MQTT] Subscribe failed err=%d (%s)\n", err, at_mqtt_err_string(err));
    return false;
}

void at_mqtt_poll() {
    if (_publishing) return;

    static char  lineBuf[600];
    static size_t linePos = 0;

    while (gsm.available()) {
        char c = (char)gsm.read();
        if (c == '\n') {
            lineBuf[linePos] = '\0';
            linePos = 0;
            if (strlen(lineBuf) == 0) continue;
            DBG("[URC] "); DBGLN(lineBuf);

            if (strstr(lineBuf, "+CMQTTRXSTART:")) {
                _rxInProgress = true;
                _rxTopic[0] = '\0'; _rxPayload[0] = '\0';
                _rxTopicLen = 0;    _rxPayloadLen  = 0;
            }
            else if (strstr(lineBuf, "+CMQTTRXEND:")) {
                if (_msgCallback) _msgCallback(_rxTopic, _rxPayload);
                _rxInProgress = false;
            }
            else if (strstr(lineBuf, "CONNLOST") || strstr(lineBuf, "CONN_LOST")) {
                char* comma = strrchr(lineBuf, ',');
                int cause = comma ? atoi(comma + 1) : -1;
                DBGF("[MQTT] Connection lost, cause=%d (%s)\n",
                     cause,
                     cause == 1 ? "network error" :
                     cause == 2 ? "keepalive timeout" :
                     cause == 3 ? "broker closed connection" :
                     cause == 4 ? "heartbeat timeout" : "unknown");
                _connected = false;
            }
            else if (_rxInProgress && !strstr(lineBuf, "+CMQTT")) {
                if (strlen(_rxTopic) == 0 && _rxTopicLen == 0)
                    { strncpy(_rxTopic,   lineBuf, sizeof(_rxTopic)   - 1); _rxTopicLen   = strlen(_rxTopic); }
                else if (strlen(_rxPayload) == 0)
                    { strncpy(_rxPayload, lineBuf, sizeof(_rxPayload) - 1); _rxPayloadLen = strlen(_rxPayload); }
            }
        } else if (c != '\r') {
            if (linePos < sizeof(lineBuf) - 1) lineBuf[linePos++] = c;
        }
    }
}

void at_mqtt_set_callback(at_mqtt_message_cb cb) { _msgCallback = cb; }
bool at_mqtt_is_connected() { return _connected; }
bool at_mqtt_is_busy() { return _publishing; }

void at_mqtt_maintain() {
    at_mqtt_poll();

    if (!_connected) {
        DBGLN("[MQTT] Maintain: reconnecting...");
        at_mqtt_connect();
        if (_connected) at_mqtt_subscribe();
        return;
    }

    if (!_publishing && (millis() - _lastPingMs >= PING_INTERVAL_MS)) {
        DBGLN("[MQTT] Sending keepalive PING...");
        if (_at_cmd("AT+CMQTTPING=0", AT_DEFAULT_TIMEOUT_MS)) {
            DBGLN("[MQTT] PING OK");
        } else {

            DBGLN("[MQTT] PING failed — marking disconnected for reconnect");
            _connected = false;
        }
        _lastPingMs = millis();
    }
}

void at_mqtt_get_timestamp(char* outBuf, size_t outBufLen) {
    strncpy(outBuf, "1970-01-01T00:00:00+00:00", outBufLen - 1);
    outBuf[outBufLen - 1] = '\0';

    while (gsm.available()) gsm.read();
    gsm.print("AT+CCLK?\r\n");

    char line[128];
    uint32_t deadline = millis() + AT_DEFAULT_TIMEOUT_MS;
    while (millis() < deadline) {
        if (_read_line(line, sizeof(line), 200)) {
            char* p = strstr(line, "+CCLK:");
            if (!p) { if (strstr(line,"OK")||strstr(line,"ERROR")) break; continue; }
            p = strchr(p, '"');
            if (!p) break;
            p++;
            int yy,mo,dd,hh,mm,ss,tz; char sign='+';
            if (sscanf(p, "%2d/%2d/%2d,%2d:%2d:%2d%c%2d",
                       &yy,&mo,&dd,&hh,&mm,&ss,&sign,&tz) >= 6) {
                snprintf(outBuf, outBufLen,
                         "20%02d-%02d-%02dT%02d:%02d:%02d%c%02d:%02d",
                         yy,mo,dd,hh,mm,ss,sign,(tz*15)/60,(tz*15)%60);
                DBGF("[TIME] %s\n", outBuf);
                return;
            }
        }
    }
    DBGLN("[TIME] CCLK failed, using fallback");
}

int at_mqtt_get_rssi_raw() {
    while (gsm.available()) gsm.read();
    gsm.print("AT+CSQ\r\n");

    char line[128];
    uint32_t deadline = millis() + AT_DEFAULT_TIMEOUT_MS;
    while (millis() < deadline) {
        if (_read_line(line, sizeof(line), 200)) {
            char* p = strstr(line, "+CSQ:");
            if (!p) { if (strstr(line,"OK")||strstr(line,"ERROR")) break; continue; }
            int raw, ber;
            if (sscanf(p+5," %d,%d",&raw,&ber) >= 1) {
                DBGF("[RSSI] raw %d\n", raw);
                return raw;
            }
        }
    }
    return -1;
}

int at_mqtt_get_rssi() {
    const int raw = at_mqtt_get_rssi_raw();
    if (raw <= 0 || raw == 99) return 0;
    return -113 + raw * 2;
}
