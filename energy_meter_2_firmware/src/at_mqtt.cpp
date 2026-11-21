
#include <Arduino.h>
#include <Preferences.h>
#include <esp_task_wdt.h>
#include "config.h"
#include "at_mqtt.h"
#include "certs_generated.h"

static HardwareSerial gsm(1);
static volatile bool _connected  = false;
static volatile bool _publishing = false;
static at_mqtt_message_cb _msgCallback = nullptr;


static unsigned long _lastPingMs = 0;

// Where the last connection attempt stopped, for the recovery ladder's
// NET_RECOVERED report and {"cmd":"diag"} (see at_net_snapshot()).
static const char* _stage = "boot";
static int _lastConnErr = 0;
static bool _safeMode = false;
static const unsigned long PING_INTERVAL_MS = 45000UL;

// ── Incoming MQTT messages vs. AT replies ───────────────────────────
//
// An inbound message is a URC block that can arrive in the middle of any AT
// exchange:
//   +CMQTTRXSTART: 0,<topic_len>,<payload_len>
//   +CMQTTRXTOPIC: 0,<n>      then exactly n raw bytes
//   +CMQTTRXPAYLOAD: 0,<n>    then exactly n raw bytes (may repeat: chunks)
//   +CMQTTRXEND: 0
// Every byte read from the modem goes through _feed(), which cuts these blocks
// out (topic and payload by byte count, so their content can never look like
// "OK" or "ERROR") and hands only the remaining lines to the AT code. Without
// this, firmware-update blocks arriving during a publish were read as the
// publish's reply or flushed away, and the session was declared dead.
static char   _rxTopic[256]   = {0};
static char   _rxPayload[MQTT_RX_PAYLOAD_MAX + 1] = {0};
static size_t _rxTopicLen     = 0;
static size_t _rxPayloadLen   = 0;
static bool   _rxInProgress   = false;
static bool   _rxOverflow     = false;
enum RxRaw : uint8_t { RAW_NONE, RAW_TOPIC, RAW_PAYLOAD };
static RxRaw  _rxRaw          = RAW_NONE;
static size_t _rxRawLeft      = 0;

static char   _line[512];      // one AT reply / URC line (raw message bytes never land here)
static size_t _linePos        = 0;

static size_t _header_len(const char* line) {
    const char* comma = strrchr(line, ',');
    return comma ? (size_t)atoi(comma + 1) : 0;
}

// Handles a line that belongs to the inbound-message protocol. Returns false
// for anything else (an AT reply the caller is waiting for, or another URC).
static bool _mqtt_rx_line(const char* line) {
    if (strncmp(line, "+CMQTTRXSTART:", 14) == 0) {
        _rxInProgress = true;
        _rxOverflow = false;
        _rxTopic[0] = '\0';   _rxTopicLen = 0;
        _rxPayload[0] = '\0'; _rxPayloadLen = 0;
        return true;
    }
    if (strncmp(line, "+CMQTTRXTOPIC:", 14) == 0) {
        _rxRaw = RAW_TOPIC;
        _rxRawLeft = _header_len(line);
        return true;
    }
    if (strncmp(line, "+CMQTTRXPAYLOAD:", 16) == 0) {
        _rxRaw = RAW_PAYLOAD;
        _rxRawLeft = _header_len(line);
        return true;
    }
    if (strncmp(line, "+CMQTTRXEND:", 12) == 0) {
        if (_rxInProgress && !_rxOverflow && _msgCallback) _msgCallback(_rxTopic, _rxPayload);
        if (_rxOverflow) DBGF("[MQTT] Dropped a %u-byte message on %s (too large)\n", (unsigned)_rxPayloadLen, _rxTopic);
        _rxInProgress = false;
        return true;
    }
    if (strstr(line, "CONNLOST") || strstr(line, "CONN_LOST")) {
        const char* comma = strrchr(line, ',');
        const int cause = comma ? atoi(comma + 1) : -1;
        DBGF("[MQTT] Connection lost, cause=%d (%s)\n", cause,
             cause == 1 ? "network error" :
             cause == 2 ? "keepalive timeout" :
             cause == 3 ? "broker closed connection" :
             cause == 4 ? "heartbeat timeout" : "unknown");
        _connected = false;
        return true;
    }
    return false;
}

// Feeds one byte from the modem. Returns true when _line holds a complete line
// that is not part of an inbound message.
static bool _feed(char c) {
    if (_rxRawLeft > 0) {
        if (_rxRaw == RAW_TOPIC) {
            if (_rxTopicLen < sizeof(_rxTopic) - 1) { _rxTopic[_rxTopicLen++] = c; _rxTopic[_rxTopicLen] = '\0'; }
        } else if (_rxPayloadLen < sizeof(_rxPayload) - 1) {
            _rxPayload[_rxPayloadLen++] = c;
            _rxPayload[_rxPayloadLen] = '\0';
        } else {
            _rxOverflow = true;
        }
        if (--_rxRawLeft == 0) _rxRaw = RAW_NONE;
        return false;
    }
    if (c == '\r') return false;
    if (c != '\n') {
        if (_linePos < sizeof(_line) - 1) _line[_linePos++] = c;
        return false;
    }
    _line[_linePos] = '\0';
    const size_t len = _linePos;
    _linePos = 0;
    return len > 0 && !_mqtt_rx_line(_line);
}

// Replaces "discard whatever is buffered" before a command: inbound messages
// are still processed, stale reply lines are dropped. A partial line is kept
// only while a message is arriving.
static void _flush_rx() {
    while (gsm.available()) _feed((char)gsm.read());
    if (_linePos > 0 && !_rxInProgress) _linePos = 0;
}

// For loops waiting for the '>' data prompt: true when it arrives; any other
// byte goes through _feed().
static bool _prompt_char() {
    if (!gsm.available()) return false;
    const char c = (char)gsm.read();
    if (c == '>' && _linePos == 0 && _rxRawLeft == 0) return true;
    _feed(c);
    return false;
}

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

// Next complete line that is not part of an inbound MQTT message (those are
// dispatched to the message callback on the way).
static bool _read_line(char* out, size_t outLen, uint32_t timeoutMs) {
    uint32_t deadline = millis() + timeoutMs;
    while (millis() < deadline) {
        esp_task_wdt_reset();   // a bounded wait; keeps a slow attach from tripping the watchdog
        while (gsm.available()) {
            if (_feed((char)gsm.read())) {
                strncpy(out, _line, outLen - 1);
                out[outLen - 1] = '\0';
                DBG("[AT] << "); DBGLN(out);
                return true;
            }
        }
        delay(2);
    }
    return false;
}

static bool _at_cmd(const char* cmd, uint32_t timeoutMs,
                     const char* expectOk  = "OK",
                     const char* expectErr = "ERROR") {
    _flush_rx();
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
    _flush_rx();
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

    _flush_rx();
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
    _flush_rx();
    DBG("[AT] >> "); DBGLN(cmd);
    gsm.print(cmd); gsm.print("\r\n");

    bool gotPrompt = false;
    uint32_t deadline = millis() + AT_DEFAULT_TIMEOUT_MS;
    while (millis() < deadline && !gotPrompt) {
        if (_prompt_char()) gotPrompt = true;
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
    // Bundle hash (CA + cert + key), so replacing the CA alone -- e.g. moving
    // to the custom-domain server CA -- also re-provisions the modem.
    const bool fpMatches  = (strcmp(storedFp, CERT_BUNDLE_SHA256) == 0);

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
        DBGF("[CERT]     build has : %s (CA %s)\n", CERT_BUNDLE_SHA256, CERT_CA_SOURCE);
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
        prefs.putString(NVS_KEY_FP, CERT_BUNDLE_SHA256);
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

// ── Modem clock ─────────────────────────────────────────────────────
//
// The modem checks AWS's server certificate against its own RTC. The A7670C
// (seen on revision V11.0.01) boots at 1970-01-01 and, unless the operator
// sends network time, stays there. Every certificate then looks "not yet
// valid" and CMQTTCONNECT fails with err 32. AT+CSSLCFG="ignorelocaltime"
// returns OK on that revision but does not prevent it. So set the clock
// before every TLS attempt: network time if the operator provides it, NTP
// otherwise. The same clock stamps every payload (at_mqtt_get_timestamp).

// Two-digit year from AT+CCLK?, or -1.
static int _modem_year() {
    _flush_rx();
    DBGLN("[AT] >> AT+CCLK?");
    gsm.print("AT+CCLK?\r\n");

    char line[128];
    int  year = -1;
    uint32_t deadline = millis() + AT_DEFAULT_TIMEOUT_MS;
    while (millis() < deadline) {
        if (!_read_line(line, sizeof(line), 200)) continue;
        const char* p = strstr(line, "+CCLK:");
        if (p && (p = strchr(p, '"')) != nullptr) year = atoi(p + 1);
        if (strstr(line, "OK") || strstr(line, "ERROR")) break;
    }
    return year;
}

// CCLK years are two digits, and an unset RTC reads "70" (1970) -- so the
// check needs an upper bound as well as a lower one.
static bool _year_valid(int yy) { return yy >= 25 && yy < 70; }

static const char* const NTP_SERVERS[] = {
    "pool.ntp.org", "time.google.com", "time.cloudflare.com"
};

static bool _sync_modem_clock() {
    int yy = _modem_year();
    if (_year_valid(yy)) return true;

    DBGF("[TIME] Modem clock not set (year %02d) - TLS would reject AWS's certificate\n", yy);

    // Network time (NITZ). A persistent setting that applies on the
    // operator's next time update, so it usually cannot help this attempt.
    _at_cmd("AT+CTZU=1", AT_DEFAULT_TIMEOUT_MS);

    char cmd[80];
    for (const char* server : NTP_SERVERS) {
        snprintf(cmd, sizeof(cmd), "AT+CNTP=\"%s\",0", server);   // 0 = UTC
        if (!_at_cmd(cmd, AT_DEFAULT_TIMEOUT_MS)) continue;

        int err = -1;   // +CNTP: 0 = success; 1 unknown, 2 bad param, 3 bad time, 4 network
        if (_at_cmd_two_stage("AT+CNTP", "+CNTP:", 20000, &err) && err == 0) {
            yy = _modem_year();
            if (_year_valid(yy)) {
                DBGF("[TIME] Modem clock set via NTP (%s)\n", server);
                return true;
            }
        }
        DBGF("[TIME] NTP via %s failed (err=%d)\n", server, err);
    }
    DBGLN("[TIME] Could not set the modem clock - expect CMQTTCONNECT err 32");
    return false;
}

// ── IP bearer (NETOPEN) ──────────────────────────────────────────────
//
// AT+CGATT=1 means the module is attached to the packet-switched network at
// the signalling level -- it is NOT the same as having a usable IP bearer.
// The A7670C (this module, per ATI) is in the same AT-command family as the
// SIM7500/SIM7600/A7600 -- the family this firmware's CMQTT* commands and
// its -D TINY_GSM_MODEM_SIM7600 build flag were both written against -- and
// that whole family brings its data bearer up with AT+NETOPEN before CMQTT/
// CHTTP/CIP can open a socket. CGATT alone is not enough. Without an open
// bearer, CMQTTCONNECT's internal TCP connect can fail during the TLS
// phase, and this module reports that the same way it reports a bad
// certificate: err 32.
//
// (An earlier version of this function used AT+CNACT, which is the
// bearer command for a DIFFERENT SIMCom chip family -- SIM7070/7080/7090 --
// and does not apply to the A7670C. Left only as a fallback below, since a
// non-fatal ERROR here costs nothing.)
//
// This firmware re-provisions a verified-good certificate store and still
// fails identically every time, with a correct clock, on an endpoint
// independently proven to accept these exact files (see
// tools/iot_selftest.py) -- consistent with the socket never opening at all
// rather than a TLS-level rejection.
//
// Every step here is non-fatal and logged: if the module answers ERROR or a
// different shape than expected, gsm_modem_init() continues exactly as it
// did before this change. Send the log after a real attempt so the exchange
// can be corrected if the module's replies don't match what is parsed here.
static bool _bring_up_pdp() {
    char line[128];

    // AT+NETOPEN? -> "+NETOPEN: 1" if already open. A fresh boot answers 0.
    _flush_rx();
    DBGLN("[AT] >> AT+NETOPEN?");
    gsm.print("AT+NETOPEN?\r\n");
    uint32_t deadline = millis() + AT_DEFAULT_TIMEOUT_MS;
    while (millis() < deadline) {
        if (!_read_line(line, sizeof(line), 200)) continue;
        if (strstr(line, "+NETOPEN: 1")) { DBGLN("[GSM] Network bearer already open"); return true; }
        if (strstr(line, "OK") || strstr(line, "ERROR")) break;
    }

    char cmd[96];
    snprintf(cmd, sizeof(cmd), "AT+CGDCONT=1,\"IP\",\"%s\"", APN);
    _at_cmd(cmd, AT_DEFAULT_TIMEOUT_MS);   // non-fatal: a SIM-provisioned default APN may already be set

    // AT+NETOPEN -> OK, or "+IP ERROR: Network is already opened" (also fine).
    DBGLN("[AT] >> AT+NETOPEN");
    gsm.print("AT+NETOPEN\r\n");
    bool opened = false, alreadyOpen = false;
    deadline = millis() + 20000;   // bearer bring-up can take several seconds
    while (millis() < deadline) {
        if (!_read_line(line, sizeof(line), 200)) continue;
        if (strstr(line, "already opened")) { alreadyOpen = true; }
        if (strstr(line, "OK")) { opened = true; break; }
        if (strstr(line, "ERROR")) break;
    }
    if (!opened && !alreadyOpen) {
        DBGLN("[GSM] AT+NETOPEN failed -- trying AT+CNACT=1,1 as a fallback (different chip family)");
        _flush_rx();
        gsm.print("AT+CNACT=1,1\r\n");
        deadline = millis() + 20000;
        while (millis() < deadline) {
            if (!_read_line(line, sizeof(line), 200)) continue;
            if (strstr(line, "+APP PDP: 1,ACTIVE")) { DBGLN("[GSM] Bearer active (CNACT)"); return true; }
            if (strstr(line, "ERROR")) { DBGLN("[GSM] AT+CNACT=1,1 also failed"); return false; }
        }
        DBGLN("[GSM] Timed out waiting on CNACT fallback");
        return false;
    }

    // Confirm with an IP, not just OK -- AT+IPADDR -> "+IPADDR: <ip>".
    _flush_rx();
    DBGLN("[AT] >> AT+IPADDR");
    gsm.print("AT+IPADDR\r\n");
    deadline = millis() + AT_DEFAULT_TIMEOUT_MS;
    while (millis() < deadline) {
        if (!_read_line(line, sizeof(line), 200)) continue;
        if (strstr(line, "+IPADDR:") && !strstr(line, "0.0.0.0")) {
            DBGLN("[GSM] Network bearer open with an IP address");
            return true;
        }
        if (strstr(line, "OK") || strstr(line, "ERROR")) break;
    }
    DBGLN("[GSM] NETOPEN reported OK but no IP yet -- proceeding anyway");
    return true;
}

bool gsm_modem_init() {
    // The default 256-byte UART buffer overflows on a ~1 KB /cmd message that arrives
    // while the network task is busy publishing. The size only takes effect before
    // begin(), and this function re-runs after every failed attach, so end() first.
    gsm.end();
    // Firmware-update blocks arrive back to back while a flash sector erase
    // can stall the reader for ~100 ms; 8 KB holds a whole request window.
    gsm.setRxBufferSize(8192);
    gsm.begin(GSM_UART_BAUD, SERIAL_8N1, GSM_RX_PIN, GSM_TX_PIN);
    _linePos = 0;
    _rxRaw = RAW_NONE;
    _rxRawLeft = 0;
    _rxInProgress = false;
    delay(100);

    if (GSM_PWRKEY_PIN >= 0) {
        pinMode(GSM_PWRKEY_PIN, OUTPUT);
        digitalWrite(GSM_PWRKEY_PIN, HIGH); delay(100);
        digitalWrite(GSM_PWRKEY_PIN, LOW);  delay(1200);
        digitalWrite(GSM_PWRKEY_PIN, HIGH); delay(3000);
    }

    _stage = "at";
    DBGLN("[GSM] Waking modem...");
    bool awake = false;
    for (int i = 0; i < 10 && !awake; i++) {
        awake = _at_cmd("AT", AT_DEFAULT_TIMEOUT_MS);
        if (!awake) delay(1000);
    }
    if (!awake) { DBGLN("[GSM] No AT response"); return false; }

    _at_cmd("AT+CMEE=2", AT_DEFAULT_TIMEOUT_MS);
    _at_cmd("ATI", AT_DEFAULT_TIMEOUT_MS);   // model + firmware revision, for support logs

    _stage = "attach";
    DBGLN("[GSM] Waiting for GPRS...");
    bool attached = false;
    char line[128];
    for (int i = 0; i < 15 && !attached; i++) {
        _flush_rx();
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

    // Non-fatal on purpose -- see _bring_up_pdp()'s comment. If this module
    // turns out not to need it, logging a failed attempt here changes
    // nothing about what happens next.
    _bring_up_pdp();

    return true;
}

// ── One-shot TLS diagnosis ──────────────────────────────────────────
//
// CMQTTCONNECT reports a server-certificate check failure, a client-
// certificate problem, an AWS disconnect and a dead data path all as err 32.
// On the first err 32 of each boot this runs four connect attempts that each
// remove one variable, then prints a verdict. Nothing is published; every
// connection that succeeds is closed immediately. SSL context 0 keeps its
// cacert/clientcert/clientkey/SNI settings; only authmode is varied, and it
// is put back to 2 afterwards.
//
//   authmode: 0 = no certificates checked or sent
//             2 = verify AWS + present client cert (production)
//             3 = present client cert, do NOT verify AWS's certificate
static bool _tlsDiagDone = false;

static int _diag_attempt(const char* label, const char* url, int serverType, int authmode) {
    char cmd[200];
    int  err = -1;

    // Four attempts can take ~2 minutes; networkTask is on the task watchdog
    // (TASK_WATCHDOG_TIMEOUT_S), so feed it per attempt rather than let a
    // diagnosis reset the board halfway through.
    esp_task_wdt_reset();

    _mqtt_teardown();
    if (serverType == 1) {
        snprintf(cmd, sizeof(cmd), "AT+CSSLCFG=\"authmode\",0,%d", authmode);
        _at_cmd(cmd, AT_DEFAULT_TIMEOUT_MS);
    }
    int startErr = -1;
    _at_cmd_two_stage("AT+CMQTTSTART", "+CMQTTSTART:", AT_DEFAULT_TIMEOUT_MS * 2, &startErr);
    delay(300);

    snprintf(cmd, sizeof(cmd), "AT+CMQTTACCQ=0,\"%s-diag\",%d", MQTT_CLIENT_ID, serverType);
    if (!_at_cmd(cmd, AT_DEFAULT_TIMEOUT_MS)) {
        DBGF("[DIAG] %s -> CMQTTACCQ failed, test skipped\n", label);
        return -1;
    }
    if (serverType == 1) _at_cmd("AT+CMQTTSSLCFG=0,0", AT_DEFAULT_TIMEOUT_MS);

    snprintf(cmd, sizeof(cmd), "AT+CMQTTCONNECT=0,\"%s\",60,1", url);
    const uint32_t t0 = millis();
    if (!_at_cmd_two_stage(cmd, "+CMQTTCONNECT:", AT_CONNECT_TIMEOUT_MS, &err)) err = -1;
    // Duration matters: an instant failure means the modem gave up before
    // talking to the server at all; a second or more means a real exchange.
    DBGF("[DIAG] %s -> %s  err=%d (%s) after %lu ms\n",
         label, err == 0 ? "CONNECTED" : "failed", err,
         err >= 0 ? at_mqtt_err_string(err) : "no reply", (unsigned long)(millis() - t0));
    if (err == 0) _at_cmd("AT+CMQTTDISC=0,10", 5000);
    return err;
}

// AT+CDNSGIP="<host>" -> OK, then +CDNSGIP: 1,"<host>","<ip>"[,"<ip2>"]
// or +CDNSGIP: 0,<err>. Copies the last IP returned into ip.
static bool _modem_resolve(const char* host, char* ip, size_t ipLen) {
    char cmd[160];
    snprintf(cmd, sizeof(cmd), "AT+CDNSGIP=\"%s\"", host);
    _flush_rx();
    DBG("[AT] >> "); DBGLN(cmd);
    gsm.print(cmd); gsm.print("\r\n");

    char line[256];
    uint32_t deadline = millis() + 15000;
    while (millis() < deadline) {
        if (!_read_line(line, sizeof(line), 200)) continue;
        char* p = strstr(line, "+CDNSGIP:");
        if (p) {
            if (atoi(p + 9) != 1) return false;          // 0,<err> = lookup failed
            char* end = strrchr(line, '"');
            if (!end) return false;
            *end = '\0';
            char* start = strrchr(line, '"');
            if (!start) return false;
            strncpy(ip, start + 1, ipLen - 1);
            ip[ipLen - 1] = '\0';
            return true;
        }
        if (strstr(line, "ERROR")) return false;       // OK precedes the URC; keep waiting
    }
    return false;
}

// Fetches https://tls.peet.ws/api/clean with the modem's own HTTPS client,
// which uses the same TLS stack as CMQTT. The service answers with a
// fingerprint of the ClientHello it just received (JA3: TLS version, cipher
// suites, extensions, groups; peetprint: also signature algorithms), so the
// modem's exact hello can be replayed with openssl against both AWS
// endpoints. authmode 0: no certificate is sent, nothing else is transmitted.
static char _diagBuf[2048];

static void _diag_clienthello() {
    DBGLN("[DIAG] Capturing this modem's TLS ClientHello via https://tls.peet.ws/api/clean");
    _at_cmd("AT+HTTPTERM", 3000);                     // ERROR if not initialised: fine
    if (!_at_cmd("AT+HTTPINIT", 5000)) { DBGLN("[DIAG] HTTPINIT failed - capture skipped"); return; }
    _at_cmd("AT+CSSLCFG=\"authmode\",0,0", AT_DEFAULT_TIMEOUT_MS);
    _at_cmd("AT+HTTPPARA=\"URL\",\"https://tls.peet.ws/api/clean\"", 5000);
    _at_cmd("AT+HTTPPARA=\"SSLCFG\",0", 5000);       // use SSL context 0, as CMQTT does

    int method = -1, status = -1, len = -1;
    _flush_rx();
    DBGLN("[AT] >> AT+HTTPACTION=0");
    gsm.print("AT+HTTPACTION=0\r\n");
    uint32_t deadline = millis() + 30000;
    while (millis() < deadline) {
        if (!_read_line(_diagBuf, sizeof(_diagBuf), 200)) continue;
        const char* p = strstr(_diagBuf, "+HTTPACTION:");
        if (p && sscanf(p + 12, " %d,%d,%d", &method, &status, &len) == 3) break;
        if (strstr(_diagBuf, "ERROR")) break;
    }
    DBGF("[DIAG] HTTPS GET status=%d length=%d\n", status, len);

    if (status == 200 && len > 0) {
        char cmd[40];
        snprintf(cmd, sizeof(cmd), "AT+HTTPREAD=0,%d", len > 1900 ? 1900 : len);
        _flush_rx();
        DBG("[AT] >> "); DBGLN(cmd);
        gsm.print(cmd); gsm.print("\r\n");
        deadline = millis() + 10000;
        while (millis() < deadline) {                 // every line is echoed by _read_line
            if (!_read_line(_diagBuf, sizeof(_diagBuf), 200)) continue;
            if (strstr(_diagBuf, "+HTTPREAD: 0")) break;
        }
        DBGLN("[DIAG] ^ ClientHello fingerprint (ja3 / peetprint lines above) - send these");
    } else if (status > 0) {
        DBGF("[DIAG] HTTPS worked but returned status %d - capture incomplete\n", status);
    } else {
        DBGLN("[DIAG] HTTPS request failed - the modem's TLS could not reach tls.peet.ws either");
    }
    _at_cmd("AT+HTTPTERM", 3000);
}

// HTTPS GET through the modem's HTTP client (same TLS stack as CMQTT) with no
// client certificate. Returns the HTTP status: any real status (200, 403, ...)
// means the TLS handshake completed; 715 means the modem's TLS handshake
// failed; other 7xx are modem-side errors.
static int _diag_https_status(const char* label, const char* url) {
    esp_task_wdt_reset();
    _at_cmd("AT+HTTPTERM", 3000);                     // ERROR if not initialised: fine
    if (!_at_cmd("AT+HTTPINIT", 5000)) { DBGF("[DIAG] %s -> HTTPINIT failed\n", label); return -1; }
    _at_cmd("AT+CSSLCFG=\"authmode\",0,0", AT_DEFAULT_TIMEOUT_MS);

    char cmd[200];
    snprintf(cmd, sizeof(cmd), "AT+HTTPPARA=\"URL\",\"%s\"", url);
    _at_cmd(cmd, 5000);
    _at_cmd("AT+HTTPPARA=\"SSLCFG\",0", 5000);

    int method = -1, status = -1, len = -1;
    _flush_rx();
    DBGLN("[AT] >> AT+HTTPACTION=0");
    gsm.print("AT+HTTPACTION=0\r\n");
    const uint32_t t0 = millis();
    const uint32_t deadline = t0 + 40000;
    while (millis() < deadline) {
        if (!_read_line(_diagBuf, sizeof(_diagBuf), 200)) continue;
        const char* p = strstr(_diagBuf, "+HTTPACTION:");
        if (p && sscanf(p + 12, " %d,%d,%d", &method, &status, &len) == 3) break;
        if (strstr(_diagBuf, "ERROR")) break;
    }
    DBGF("[DIAG] %s -> HTTP status %d (%s) after %lu ms\n", label, status,
         status == 715 ? "modem: TLS handshake FAILED"
         : status >= 700 ? "modem-side error"
         : status > 0 ? "TLS handshake COMPLETED" : "no reply",
         (unsigned long)(millis() - t0));
    _at_cmd("AT+HTTPTERM", 3000);
    return status;
}

static void _run_tls_diagnostics() {
    // meter 001's endpoint (AWS account 571751567031), where a device IS
    // connecting in production. Used only as a comparison target.
    static const char* OTHER_AWS = "tcp://a1d3i8d08oi632-ats.iot.ap-south-1.amazonaws.com:8883";

    char aws[160];
    snprintf(aws, sizeof(aws), "tcp://%s:%d", MQTT_BROKER_HOST, MQTT_BROKER_PORT);

    DBGLN("[DIAG] ===== one-time TLS diagnosis (once per boot, nothing is published) =====");

    char ip[64] = {0};
    const bool resolved = _modem_resolve(MQTT_BROKER_HOST, ip, sizeof(ip));
    DBGF("[DIAG] DNS: %s -> %s\n", MQTT_BROKER_HOST, resolved ? ip : "LOOKUP FAILED");

    // Which TLS options this firmware revision supports, and SSL context 0's
    // current settings -- in case it offers a record-size / buffer option.
    _at_cmd("AT+CSSLCFG=?", AT_DEFAULT_TIMEOUT_MS);
    _at_cmd("AT+CSSLCFG?",  AT_DEFAULT_TIMEOUT_MS);

    // G first, so the answer is near the top of the log: the full production
    // connect (verify AWS + client cert) with SNI switched OFF. AWS requires
    // an exact host_name in SNI on endpoints with a configured TLS policy
    // (this account's iot:Data-ATS has one) and rejects a wrong one at the
    // ClientHello -- but accepts no SNI at all. Meter 001's endpoint and
    // test.mosquitto.org ignore SNI, which is why they connect.
    _at_cmd("AT+CSSLCFG=\"enableSNI\",0,0", AT_DEFAULT_TIMEOUT_MS);
    const int g = _diag_attempt("G: AWS, production certs, SNI OFF", aws, 1, 2);
    _at_cmd("AT+CSSLCFG=\"enableSNI\",0,1", AT_DEFAULT_TIMEOUT_MS);

    // The modem's exact ClientHello, so it can be replayed against AWS.
    _mqtt_teardown();
    esp_task_wdt_reset();
    _diag_clienthello();

    // K: the same AWS IoT host and the same 4,996-byte certificate record as
    // port 8883 -- but on 443, where AWS does NOT send a CertificateRequest.
    // This separates "cannot receive AWS's 5 KB chain" from "cannot handle
    // AWS's request for a client certificate" (openssl survey, 2026-09-11).
    char awsHttps[160];
    snprintf(awsHttps, sizeof(awsHttps), "https://%s:443/", MQTT_BROKER_HOST);
    const int k = _diag_https_status("K: AWS IoT :443 (same 5 KB chain, no cert request)", awsHttps);

    // L: the mirror image of K -- a server that DOES send a CertificateRequest
    // (client certificate optional) with a chain smaller than flespi's
    // 4,431 B, which this modem already handles. client.badssl.com: 4,074 B.
    const int l = _diag_https_status("L: client.badssl.com (4.07 KB chain + cert request)",
                                     "https://client.badssl.com/");
    // L2: a request shaped like AWS's -- no CA names, RSA-PSS-PSS (and EdDSA)
    // signature methods listed -- with a 4,073 B chain. badssl's (L) has
    // CA names and no PSS entries, so L vs L2 isolates the PSS entries.
    const int l2 = _diag_https_status("L2: certauth.idrix.fr (4.07 KB chain + PSS-style request)",
                                      "https://certauth.idrix.fr/");

    const int a = _diag_attempt("A: AWS, client cert, AWS cert NOT checked",   aws, 1, 3);
    const int b = _diag_attempt("B: AWS, no certificates at all",               aws, 1, 0);
    const int c = _diag_attempt("C: test.mosquitto.org, plain TCP (no TLS)",    "tcp://test.mosquitto.org:1883", 0, 0);
    const int d = _diag_attempt("D: test.mosquitto.org, TLS, nothing checked",  "tcp://test.mosquitto.org:8883", 1, 0);
    // Server certificate chain size is the suspected limit. Certificate
    // message sizes (openssl, 2026-09-11): D 933 B works; tls.peet.ws 4060 B
    // and AWS 4996 B fail. Port 8886 (Let's Encrypt, 3 certs) is 4094 B.
    const int h = _diag_attempt("H: test.mosquitto.org:8886, ~4.1 KB chain",    "tcp://test.mosquitto.org:8886", 1, 0);
    // Two more rungs on the size ladder. flespi needs a token for MQTT, so a
    // non-32 refusal there still means TLS itself completed.
    const int i = _diag_attempt("I: broker.hivemq.com, ~3.85 KB chain",         "tcp://broker.hivemq.com:8883", 1, 0);
    const int j = _diag_attempt("J: mqtt.flespi.io, ~4.43 KB chain",            "tcp://mqtt.flespi.io:8883", 1, 0);
    const int e = _diag_attempt("E: meter 001's AWS endpoint, no certificates", OTHER_AWS, 1, 0);

    int f = -2;   // -2 = not run (no IP to try)
    if (resolved && strchr(ip, '.')) {
        char byIp[96];
        snprintf(byIp, sizeof(byIp), "tcp://%s:%d", ip, MQTT_BROKER_PORT);
        f = _diag_attempt("F: AWS by IP address (no DNS), no certs", byIp, 1, 0);
    }

    // Leave the modem as the production connect expects it.
    _mqtt_teardown();
    _at_cmd("AT+CSSLCFG=\"authmode\",0,2", AT_DEFAULT_TIMEOUT_MS);

    DBGLN("[DIAG] ----- verdict -----");
    if (g == 0) {
        DBGLN("[DIAG] G CONNECTED with SNI OFF. This modem sends a malformed SNI host name,");
        DBGLN("[DIAG] which this AWS account's endpoint rejects. Certificates, clock and");
        DBGLN("[DIAG] network are all fine. Fix: production connect with enableSNI = 0.");
    } else if (a == 0) {
        DBGLN("[DIAG] A CONNECTED. AWS accepts this device's certificate and the link works.");
        DBGLN("[DIAG] The only failing step is the MODEM CHECKING AWS'S CERTIFICATE against");
        DBGLN("[DIAG] the CA file on the modem. Not the clock, not the device cert.");
    } else if (c != 0) {
        DBGLN("[DIAG] C failed: even plain MQTT over TCP does not work. This is the data path");
        DBGLN("[DIAG] (SIM data plan, APN, or the operator blocking the port), not TLS or certs.");
    } else if (d != 0) {
        DBGLN("[DIAG] C worked, D failed: the network is fine but this modem cannot complete");
        DBGLN("[DIAG] TLS with any server. Modem TLS/firmware problem, not certs, not AWS.");
    } else if (!resolved) {
        DBGLN("[DIAG] The modem cannot resolve the AWS endpoint name (DNS). TLS itself works (D).");
        DBGLN("[DIAG] Check the SIM operator's DNS, or configure public DNS on the modem.");
    } else if (f == 0) {
        DBGLN("[DIAG] AWS works BY IP (F) but not BY NAME (B): the modem's handling of the AWS");
        DBGLN("[DIAG] hostname is the problem (DNS answer or hostname length), not TLS or certs.");
    } else if (e == 0) {
        DBGLN("[DIAG] Meter 001's AWS endpoint works (E) but this one does not (B): something");
        DBGLN("[DIAG] specific to account 481665103941's endpoint. Send this log.");
    } else if (b != 32) {
        DBGLN("[DIAG] AWS got past TLS without certificates (B) but not with the client");
        DBGLN("[DIAG] certificate (A): the cert/key FILES on the modem are the problem.");
    } else {
        // err 32 = TLS failed; any other code (0, or an MQTT-level refusal)
        // means the TLS handshake itself completed.
        auto tls = [](int r) { return (r == 32 || r == -1) ? "TLS FAIL" : "TLS OK"; };
        DBGLN("[DIAG] Server certificate chain size vs TLS result on this modem:");
        DBGF ("[DIAG]   0.93 KB  test.mosquitto.org:8883  %s\n", tls(d));
        DBGF ("[DIAG]   3.85 KB  broker.hivemq.com        %s\n", tls(i));
        DBGF ("[DIAG]   4.09 KB  test.mosquitto.org:8886  %s\n", tls(h));
        DBGF ("[DIAG]   4.43 KB  mqtt.flespi.io           %s\n", tls(j));
        DBGF ("[DIAG]   5.00 KB  AWS IoT (both accounts)  %s\n", tls(b));
        DBGLN("[DIAG] If OK turns into FAIL as the size grows, this modem firmware cannot");
        DBGLN("[DIAG] receive a server certificate chain above that size -- AWS's is 5 KB.");
        DBGLN("[DIAG] Not certificates, clock, IoT policy, account or network.");
    }
    // K: AWS's 5 KB chain, no CertificateRequest.  L: 4 KB chain + CertificateRequest.
    auto httpsOk = [](int s) { return s > 0 && s < 700; };
    DBGLN("[DIAG] ----- chain size vs certificate request (K, L) -----");
    DBGF ("[DIAG]   K  AWS IoT :443     5.0 KB chain, no cert request  -> %s\n",
          httpsOk(k) ? "TLS OK" : (k == 715 ? "TLS FAIL" : "inconclusive"));
    DBGF ("[DIAG]   L  client.badssl.com 4.1 KB chain + cert request    -> %s\n",
          httpsOk(l) ? "TLS OK" : (l == 715 ? "TLS FAIL" : "inconclusive"));
    DBGF ("[DIAG]   L2 certauth.idrix.fr 4.1 KB chain + PSS-style req  -> %s\n",
          httpsOk(l2) ? "TLS OK" : (l2 == 715 ? "TLS FAIL" : "inconclusive"));
    if (httpsOk(k) && l == 715) {
        DBGLN("[DIAG] => The 5 KB chain is fine; ANY certificate request breaks this modem's TLS.");
    } else if (httpsOk(k) && httpsOk(l)) {
        DBGLN("[DIAG] => Chain size and certificate requests in general are fine; only AWS's");
        DBGLN("[DIAG]    particular CertificateRequest (port 8883) breaks this modem's TLS.");
    } else if (k == 715 && httpsOk(l)) {
        DBGLN("[DIAG] => Certificate requests are fine; this modem cannot receive AWS's");
        DBGLN("[DIAG]    4,996-byte certificate record.");
    } else if (k == 715 && l == 715) {
        DBGLN("[DIAG] => Both fail: AWS's 5 KB record AND certificate requests break this modem.");
    } else {
        DBGLN("[DIAG] => Inconclusive; send this log.");
    }
    if (httpsOk(l) && l2 == 715) {
        DBGLN("[DIAG] => L passed but L2 failed: a certificate request that lists RSA-PSS-PSS /");
        DBGLN("[DIAG]    EdDSA methods (as AWS's does) breaks this modem; a plain one does not.");
    }
    DBGF("[DIAG] codes: G=%d A=%d B=%d C=%d D=%d I=%d H=%d J=%d E=%d F=%d K=%d L=%d L2=%d  (0 = connected, -2 = not run)\n",
         g, a, b, c, d, i, h, j, e, f, k, l, l2);
    DBGLN("[DIAG] ==========================================================================");
}

bool at_mqtt_connect() {
    char cmdBuf[300];
    int  err = -1;

    _mqtt_teardown();

    // Blocking: without a usable cert store the handshake cannot succeed, so
    // fail fast and let networkTask's backoff retry rather than burning the
    // 20 s CMQTTCONNECT timeout on a connection that is already doomed.
    _stage = "certs";
    _lastConnErr = 0;
    if (!at_mqtt_provision_certs()) {
        DBGLN("[MQTT] Cert store not usable — aborting connect attempt");
        return false;
    }

    // Non-fatal: the attempt still runs, so a module that does honour
    // ignorelocaltime keeps working when no time source is reachable.
    _sync_modem_clock();

    DBGF("[MQTT] Identity: client_id=%s thing=%s device_id=%s topic=%s\n",
         MQTT_CLIENT_ID, AWS_THING_NAME, DEVICE_ID, MQTT_TOPIC_PUB);

    _stage = "ssl_cfg";
    DBGLN("[MQTT] Configuring SSL (mTLS)...");
    // 4 = "ALL": the module negotiates, and against AWS IoT that lands on
    // TLS 1.2. This is exactly what energy_meter_firmware (meter 001) sends in
    // production on the same SIMCom module family. Forcing 3 (TLS 1.2 only)
    // gave err 32 on every attempt during bring-up, against an endpoint that
    // accepts every TLS 1.1/1.2 cipher, group and signature algorithm and
    // serves the same chain as meter 001's (openssl s_client, 2026-09-11).
    // AWS does not reject a negotiating ClientHello.
    if (!_at_cmd("AT+CSSLCFG=\"sslversion\",0,4",   AT_DEFAULT_TIMEOUT_MS)) return false;
    // 2 = mutual TLS (client certificate required by AWS IoT on this port).
    // A diagnostic run with this forced to 1 (server-auth only, no client
    // cert presented) still failed with err 32 -- proof the certificate and
    // key are not the cause. See at_mqtt_connect()'s err==32 branch below.
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

    _stage = "mqtt_start";
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

    _stage = "connect";
    DBGLN("[MQTT] Connecting to AWS IoT...");
    if (!_at_cmd_two_stage(cmdBuf, "+CMQTTCONNECT:", AT_CONNECT_TIMEOUT_MS, &err)) {
        DBGLN("[MQTT] CMQTTCONNECT no response"); return false;
    }
    if (err != 0) {
        _lastConnErr = err;
        DBGF("[MQTT] Connect failed err=%d (%s)\n", err, at_mqtt_err_string(err));
        // err 32/33/34 all mean "the TLS handshake never completed", which has
        // several unrelated causes the modem cannot distinguish. Spell them out
        // so the next serial log is self-diagnosing.
        if (err == 32 || err == 33 || err == 34) {
            // Measurements, not a checklist. An earlier version printed fixed
            // reminders ("modem clock -- cert is not valid before <date>") on
            // every failure, which read as if the modem had found a clock
            // problem when it had not.
            DBGLN("[MQTT]   TLS handshake did not complete. Measured:");
            DBGF ("[MQTT]     cert on modem : %s (same as this build)\n", CLIENT_CERT_SHA256);
            const int yy = _modem_year();
            DBGF ("[MQTT]     modem clock   : year 20%02d -> %s\n", yy < 0 ? 0 : yy,
                  _year_valid(yy) ? "OK" : "WRONG - the modem would reject AWS's certificate");
            DBGLN("[MQTT]     (this device's own cert start date is checked by AWS, not by the modem)");

            if (!_tlsDiagDone && !_safeMode) {
                _tlsDiagDone = true;
                _run_tls_diagnostics();
            }

            if (!_safeMode && ++_tlsFailStreak >= TLS_FAILS_BEFORE_REUPLOAD) {
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

    _stage = "connected";
    DBGLN("[MQTT] Connected to AWS IoT");
    _tlsFailStreak = 0;
    _connected  = true;
    _lastPingMs = millis();
    return true;
}

bool at_mqtt_publish(const char* payload) { return at_mqtt_publish_to(MQTT_TOPIC_PUB, payload); }

bool at_mqtt_publish_to(const char* topic, const char* payload) {
    if (!_connected) { DBGLN("[MQTT] Not connected"); return false; }
    
        // Flush any stale bytes in the modem UART buffer — these can
    // arrive from electrical noise during PZEM UART remapping
    _flush_rx();
    delay(20);   // brief settle before sending AT commands


    _publishing = true;

    char   cmdBuf[64];
    size_t topicLen   = strlen(topic);
    size_t payloadLen = strlen(payload);
    char   line[256];
    bool   result = false;
  
    snprintf(cmdBuf, sizeof(cmdBuf), "AT+CMQTTTOPIC=0,%u", (unsigned)topicLen);
    _flush_rx();
    DBG("[AT] >> "); DBGLN(cmdBuf);
    gsm.print(cmdBuf); gsm.print("\r\n");

    bool gotPrompt = false;
    uint32_t d = millis() + AT_DEFAULT_TIMEOUT_MS;
    while (millis() < d) {
        if (_prompt_char()) { gotPrompt = true; break; }
    }
    if (!gotPrompt) {
        DBGLN("[MQTT] No > for TOPIC — session dead, forcing reconnect");
        _connected = false;
        goto publish_done;
    }
    gsm.print(topic);
    if (!_read_line(line, sizeof(line), AT_DEFAULT_TIMEOUT_MS) || !strstr(line, "OK"))
        { DBGLN("[MQTT] Topic rejected"); _connected = false; goto publish_done; }


    snprintf(cmdBuf, sizeof(cmdBuf), "AT+CMQTTPAYLOAD=0,%u", (unsigned)payloadLen);
    _flush_rx();
    DBG("[AT] >> "); DBGLN(cmdBuf);
    gsm.print(cmdBuf); gsm.print("\r\n");

    gotPrompt = false;
    d = millis() + AT_DEFAULT_TIMEOUT_MS;
    while (millis() < d) {
        if (_prompt_char()) { gotPrompt = true; break; }
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

bool at_mqtt_subscribe() { return at_mqtt_subscribe_topic(MQTT_TOPIC_SUB); }

bool at_mqtt_subscribe_topic(const char* topic) {
    if (!_connected) { DBGLN("[MQTT] Not connected"); return false; }

    char   cmdBuf[64];
    size_t topicLen = strlen(topic);
    char   line[256];

    snprintf(cmdBuf, sizeof(cmdBuf), "AT+CMQTTSUBTOPIC=0,%u,%d",
             (unsigned)topicLen, MQTT_QOS);
    _flush_rx();
    DBG("[AT] >> "); DBGLN(cmdBuf);
    gsm.print(cmdBuf); gsm.print("\r\n");

    bool gotPrompt = false;
    uint32_t d = millis() + AT_DEFAULT_TIMEOUT_MS;
    while (millis() < d) {
        if (_prompt_char()) { gotPrompt = true; break; }
    }
    if (!gotPrompt) { DBGLN("[MQTT] No > for SUBTOPIC"); return false; }
    gsm.print(topic);
    if (!_read_line(line, sizeof(line), AT_DEFAULT_TIMEOUT_MS) || !strstr(line, "OK"))
        { DBGLN("[MQTT] Subtopic rejected"); return false; }

    int err = -1;
    if (!_at_cmd_two_stage("AT+CMQTTSUB=0", "+CMQTTSUB:", AT_DEFAULT_TIMEOUT_MS, &err)) {
        DBGLN("[MQTT] CMQTTSUB no response"); return false;
    }
    if (err == 0) { DBGF("[MQTT] Subscribed to %s\n", topic); return true; }
    DBGF("[MQTT] Subscribe failed err=%d (%s)\n", err, at_mqtt_err_string(err));
    return false;
}

bool at_mqtt_unsubscribe_topic(const char* topic) {
    if (!_connected) return false;

    char cmdBuf[64];
    char line[256];
    snprintf(cmdBuf, sizeof(cmdBuf), "AT+CMQTTUNSUBTOPIC=0,%u", (unsigned)strlen(topic));
    _flush_rx();
    DBG("[AT] >> "); DBGLN(cmdBuf);
    gsm.print(cmdBuf); gsm.print("\r\n");

    bool gotPrompt = false;
    uint32_t d = millis() + AT_DEFAULT_TIMEOUT_MS;
    while (millis() < d) {
        if (_prompt_char()) { gotPrompt = true; break; }
    }
    if (!gotPrompt) { DBGLN("[MQTT] No > for UNSUBTOPIC"); return false; }
    gsm.print(topic);
    if (!_read_line(line, sizeof(line), AT_DEFAULT_TIMEOUT_MS) || !strstr(line, "OK")) return false;

    int err = -1;
    if (!_at_cmd_two_stage("AT+CMQTTUNSUB=0,0", "+CMQTTUNSUB:", AT_DEFAULT_TIMEOUT_MS, &err)) return false;
    DBGF("[MQTT] Unsubscribed from %s (err=%d)\n", topic, err);
    return err == 0;
}

void at_mqtt_poll() {
    if (_publishing) return;

    // Inbound messages are dispatched inside _feed(); other lines are just logged.
    while (gsm.available()) {
        if (_feed((char)gsm.read())) { DBG("[URC] "); DBGLN(_line); }
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

    _flush_rx();
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
    _flush_rx();
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

void at_mqtt_disconnect() {
    _mqtt_teardown();
    _connected = false;
}

// ── HTTP download (firmware update) ─────────────────────────────────
//
// AT+HTTPREAD=<offset>,<n> answers
//   OK
//   +HTTPREAD: <k>
//   <exactly k raw bytes>
//   +HTTPREAD: 0
// The body is binary, so the k bytes are read by count, never as lines.

static const size_t HTTP_READ_PIECE = 1024;   // well inside the 2304-byte UART RX buffer

static bool _read_exact(uint8_t* out, size_t len, uint32_t timeoutMs) {
    size_t got = 0;
    uint32_t deadline = millis() + timeoutMs;
    while (got < len && millis() < deadline) {
        const int avail = gsm.available();
        if (avail > 0) {
            got += gsm.readBytes(out + got, min((size_t)avail, len - got));
        } else {
            delay(1);
        }
    }
    return got == len;
}

// Reads one HTTPREAD piece at `offset` into `buf`. Returns the byte count, or -1.
static int _http_read_piece(uint32_t offset, size_t len, uint8_t* buf) {
    char cmd[48];
    snprintf(cmd, sizeof(cmd), "AT+HTTPREAD=%lu,%u", (unsigned long)offset, (unsigned)len);
    _flush_rx();
    gsm.print(cmd); gsm.print("\r\n");

    char line[64];
    int got = -1;
    uint32_t deadline = millis() + AT_PUB_TIMEOUT_MS;
    while (millis() < deadline) {
        if (!_read_line(line, sizeof(line), 200)) continue;
        if (strstr(line, "ERROR")) return -1;
        const char* p = strstr(line, "+HTTPREAD:");
        if (!p) continue;                               // OK, echo, blank
        const int n = atoi(p + 10);
        if (n <= 0) return got;                         // "+HTTPREAD: 0" closes the read
        if ((size_t)n > len || got >= 0) return -1;     // more than asked, or a second block
        if (!_read_exact(buf, (size_t)n, AT_PUB_TIMEOUT_MS)) return -1;
        got = n;
    }
    return -1;
}

int at_http_get_range(const char* url, uint32_t from, uint32_t to, at_http_sink sink, void* ctx) {
    static uint8_t piece[HTTP_READ_PIECE];
    static char cmd[960];                               // a presigned URL is long
    int status = -1;

    esp_task_wdt_reset();
    _at_cmd("AT+HTTPTERM", 3000);                       // ERROR if not initialised: fine
    if (!_at_cmd("AT+HTTPINIT", AT_DEFAULT_TIMEOUT_MS)) { DBGLN("[HTTP] HTTPINIT failed"); return -1; }

    if (snprintf(cmd, sizeof(cmd), "AT+HTTPPARA=\"URL\",\"%s\"", url) >= (int)sizeof(cmd) ||
        !_at_cmd(cmd, AT_DEFAULT_TIMEOUT_MS)) {
        DBGLN("[HTTP] URL rejected");
        goto done;
    }
    snprintf(cmd, sizeof(cmd), "AT+HTTPPARA=\"USERDATA\",\"Range: bytes=%lu-%lu\"",
             (unsigned long)from, (unsigned long)to);
    if (!_at_cmd(cmd, AT_DEFAULT_TIMEOUT_MS)) { DBGLN("[HTTP] Range header rejected"); goto done; }
    if (strncmp(url, "https://", 8) == 0) {
        // Context 1, no server verification: the image is checked against the
        // SHA-256 from the authenticated MQTT command instead.
        _at_cmd("AT+CSSLCFG=\"sslversion\",1,4", AT_DEFAULT_TIMEOUT_MS);
        _at_cmd("AT+CSSLCFG=\"authmode\",1,0",   AT_DEFAULT_TIMEOUT_MS);
        _at_cmd("AT+CSSLCFG=\"enableSNI\",1,1",  AT_DEFAULT_TIMEOUT_MS);
        _at_cmd("AT+CSSLCFG=\"ignorertctime\",1,1", AT_DEFAULT_TIMEOUT_MS);
        if (!_at_cmd("AT+HTTPPARA=\"SSLCFG\",1", AT_DEFAULT_TIMEOUT_MS)) goto done;
    }

    {
        int method = -1, len = -1;
        char line[96];
        _flush_rx();
        DBGLN("[AT] >> AT+HTTPACTION=0");
        gsm.print("AT+HTTPACTION=0\r\n");
        uint32_t deadline = millis() + OTA_HTTP_TIMEOUT_MS;
        while (millis() < deadline) {
            esp_task_wdt_reset();
            if (!_read_line(line, sizeof(line), 200)) continue;
            const char* p = strstr(line, "+HTTPACTION:");
            if (p && sscanf(p + 12, " %d,%d,%d", &method, &status, &len) == 3) break;
            if (strstr(line, "ERROR")) break;
        }
        DBGF("[HTTP] GET bytes %lu-%lu -> status %d, %d bytes\n",
             (unsigned long)from, (unsigned long)to, status, len);
        // 200 is the whole file from byte 0: only usable when that is what was asked.
        if (status != 206 && !(status == 200 && from == 0)) {
            if (status == 200) status = -1;
            goto done;
        }

        uint32_t offset = 0;
        while (offset < (uint32_t)len) {
            esp_task_wdt_reset();
            const size_t want = min((size_t)(len - offset), HTTP_READ_PIECE);
            const int n = _http_read_piece(offset, want, piece);
            if (n <= 0) { DBGF("[HTTP] HTTPREAD failed at %lu\n", (unsigned long)offset); status = -1; break; }
            if (!sink(piece, (size_t)n, ctx)) { status = -1; break; }
            offset += (uint32_t)n;
        }
    }

done:
    _at_cmd("AT+HTTPTERM", 3000);
    return status;
}

// ── Recovery ladder actions and network snapshot ────────────────────

void at_mqtt_set_safe_mode(bool on) { _safeMode = on; }

// Integer after "<prefix>" in the reply to `cmd` (the field-th comma-separated
// value, 0-based), or -1.
static int _query_int(const char* cmd, const char* prefix, int field) {
    _flush_rx();
    DBG("[AT] >> "); DBGLN(cmd);
    gsm.print(cmd); gsm.print("\r\n");
    char line[128];
    int value = -1;
    const uint32_t deadline = millis() + AT_DEFAULT_TIMEOUT_MS;
    while (millis() < deadline) {
        if (!_read_line(line, sizeof(line), 200)) continue;
        const char* p = strstr(line, prefix);
        if (p) {
            p += strlen(prefix);
            for (int i = 0; i < field && p; ++i) {
                p = strchr(p, ',');
                if (p) ++p;
            }
            if (p) {
                while (*p == ' ') ++p;
                value = atoi(p);
            }
            continue;   // wait for OK
        }
        if (strstr(line, "OK") || strstr(line, "ERROR")) break;
    }
    return value;
}

static int _query_sim_ready() {
    _flush_rx();
    gsm.print("AT+CPIN?\r\n");
    char line[128];
    int ready = -1;
    const uint32_t deadline = millis() + AT_DEFAULT_TIMEOUT_MS;
    while (millis() < deadline) {
        if (!_read_line(line, sizeof(line), 200)) continue;
        if (strstr(line, "+CPIN:")) { ready = strstr(line, "READY") ? 1 : 0; continue; }
        if (strstr(line, "OK")) break;
        if (strstr(line, "ERROR")) { if (ready < 0) ready = 0; break; }   // e.g. "SIM not inserted"
    }
    return ready;
}

void at_net_snapshot(NetSnapshot& snap) {
    snap = NetSnapshot{};
    snap.at_ok = _at_cmd("AT", 1500);
    snap.sim_ready = snap.at_ok ? (int8_t)_query_sim_ready() : -1;
    // stat: 0 not searching, 1 home, 2 searching, 3 denied, 4 unknown, 5 roaming
    snap.creg = snap.at_ok ? (int8_t)_query_int("AT+CREG?", "+CREG:", 1) : -1;
    snap.cereg = snap.at_ok ? (int8_t)_query_int("AT+CEREG?", "+CEREG:", 1) : -1;
    snap.csq = snap.at_ok ? (int8_t)_query_int("AT+CSQ", "+CSQ:", 0) : -1;
    snap.last_err = (int16_t)_lastConnErr;
    strncpy(snap.stage, _stage, sizeof(snap.stage) - 1);
    snprintf(snap.text, sizeof(snap.text), "sim=%d creg=%d cereg=%d csq=%d %s/%d", snap.sim_ready, snap.creg,
             snap.cereg, snap.csq, snap.stage, snap.last_err);
    DBGF("[NET] snapshot: at=%d %s\n", snap.at_ok, snap.text);
}

bool at_modem_radio_cycle() {
    DBGLN("[NET] Recovery step 1: MQTT teardown + radio off/on");
    _mqtt_teardown();
    _connected = false;
    _at_cmd("AT+CFUN=0", 15000);   // detach and power the RF down
    delay(3000);
    const bool ok = _at_cmd("AT+CFUN=1", 15000);
    delay(5000);                   // let it search before the next attach check
    return ok;
}

bool at_modem_reset() {
    DBGLN("[NET] Recovery step 2: modem reset (AT+CRESET)");
    _connected = false;
    if (!_at_cmd("AT+CRESET", 5000)) {
        DBGLN("[NET] CRESET not accepted - modem not answering AT");
        return false;
    }
    // The module reboots: it prints RDY (and later PB DONE) when ready.
    char line[128];
    const uint32_t deadline = millis() + 30000;
    while (millis() < deadline) {
        if (_read_line(line, sizeof(line), 500) && (strstr(line, "RDY") || strstr(line, "PB DONE"))) {
            DBGLN("[NET] Modem back after reset");
            return true;
        }
    }
    DBGLN("[NET] No RDY after CRESET (continuing)");
    return true;
}
