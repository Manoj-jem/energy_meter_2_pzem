#pragma once
// ================================================================
// at_mqtt.h 
// ================================================================
#include <Arduino.h>

bool gsm_modem_init();    
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
