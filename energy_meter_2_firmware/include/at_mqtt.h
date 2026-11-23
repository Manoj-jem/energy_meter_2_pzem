#pragma once
// ================================================================
// at_mqtt.h 
// ================================================================
#include <Arduino.h>

bool gsm_modem_init();

// Brings the modem's certificate store in line with the PEMs compiled into
// this build, uploading them via AT+CCERTDOWN when they are missing or stale.
// Called automatically by at_mqtt_connect(); exposed for bring-up tools.
bool at_mqtt_provision_certs();

bool at_mqtt_connect();
bool at_mqtt_publish(const char* payload);                       // -> MQTT_TOPIC_PUB
bool at_mqtt_publish_to(const char* topic, const char* payload);  // e.g. MQTT_TOPIC_ACK
bool at_mqtt_subscribe();                         // MQTT_TOPIC_SUB
bool at_mqtt_subscribe_topic(const char* topic);
bool at_mqtt_unsubscribe_topic(const char* topic);
void at_mqtt_poll();
void at_mqtt_maintain();
bool at_mqtt_is_connected();
bool at_mqtt_is_busy();

void at_mqtt_get_timestamp(char* outBuf, size_t outBufLen);

int at_mqtt_get_rssi();
int at_mqtt_get_rssi_raw();

typedef void (*at_mqtt_message_cb)(const char* topic, const char* payload);
void at_mqtt_set_callback(at_mqtt_message_cb cb);

const char* at_mqtt_err_string(int errcode);

// Closes the MQTT session (CMQTTDISC/REL/STOP). networkTask reconnects as usual.
void at_mqtt_disconnect();

// HTTP(S) GET of bytes [from, to] through the modem's HTTP client, streaming the
// body to `sink` in pieces of at most 1 KB. https uses SSL context 1 without
// server verification (context 0 stays MQTT's mTLS); the caller verifies the
// content. MQTT must be disconnected: an MQTT URC would land inside the binary
// stream. Returns the HTTP status (206, or 200 when the server ignored the
// Range header), a 7xx modem error, or -1. The sink returning false aborts.
typedef bool (*at_http_sink)(const uint8_t* data, size_t len, void* ctx);
int at_http_get_range(const char* url, uint32_t from, uint32_t to, at_http_sink sink, void* ctx);

// ── Network recovery (fw 2.x) ────────────────────────────────────
// What the modem says about the network, captured after a failed attempt.
struct NetSnapshot {
    bool    at_ok;       // modem answers AT
    int8_t  sim_ready;   // 1 READY, 0 not ready / missing, -1 unknown
    int8_t  creg;        // 2G/3G registration stat, -1 unknown (1 home, 5 roaming, 2 searching, 3 denied)
    int8_t  cereg;       // LTE registration stat
    int8_t  csq;         // 0-31, 99 = unknown
    int16_t last_err;    // last CMQTTCONNECT error (32 = TLS, 0 = none)
    char    stage[12];   // where the last attempt stopped: at, attach, certs, ssl_cfg, mqtt_start, connect, connected
    char    text[28];    // compact form for event detail
};
void at_net_snapshot(NetSnapshot& snap);

// Recovery ladder step 1: MQTT teardown + AT+CFUN=0/1 (forces re-registration).
bool at_modem_radio_cycle();
// Recovery ladder step 2: AT+CRESET and wait for RDY. False if the modem does not answer AT.
bool at_modem_reset();

// fw 2.0.2: layer-by-layer test after a failed connect (MQTT must be down).
// Writes "ip<0|1> dns<0|1> http<status> <cell>", e.g.
//   "ip1 dns1 http204 B40 q-11 p-95 s12"
//   ip    the modem has an IP address on its data bearer
//   dns   iot.energywise.tech resolves
//   http  plain GET http://connectivitycheck.gstatic.com/generate_204:
//         204 = open internet; 200/30x = operator portal (data plan); 7xx/-1 = no TCP
//   cell  LTE band, RSRQ (q), RSRP (p), SINR (s) from AT+CPSI?, or the mode
// Takes up to ~50 s.
void at_net_probe(char* out, size_t outLen);

// Boot-loop safe mode: no one-shot TLS diagnosis, no cert re-upload loop.
void at_mqtt_set_safe_mode(bool on);

// ── Publish / session statistics (fw 2.0.1), since boot ──────────
struct AtMqttStats {
    uint32_t pub_ok;
    uint32_t pub_failed;
    uint32_t kept_after_fail;   // publish failed but the probe found the session alive: not dropped
    uint32_t session_drops;     // times this driver declared the session gone
    uint32_t connlost;          // of which: +CMQTTCONNLOST from the modem
    uint32_t last_pub_ms;
    uint32_t max_pub_ms;
    char     last_drop[40];     // why the session was last dropped
    char     last_probe[48];    // last at_net_probe() result (fw 2.0.2)
};
AtMqttStats at_mqtt_stats();
