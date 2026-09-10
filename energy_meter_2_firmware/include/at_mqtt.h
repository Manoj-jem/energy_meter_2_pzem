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
bool at_mqtt_publish(const char* payload);
bool at_mqtt_subscribe();
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
