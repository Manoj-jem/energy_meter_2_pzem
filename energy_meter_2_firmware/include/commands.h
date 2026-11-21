#pragma once
// ================================================================
// commands.h - remote and serial commands (energy meter)
//
// Commands arrive on <device>/cmd (MQTT) or as a JSON line on the USB serial
// console. Replies go to <device>/cmd/ack, or back to the serial console.
//
//   {"cmd":"status"}   firmware, uptime, why it last restarted, network, flash, OTA state
//   {"cmd":"diag"}     counters, last network snapshot, flash health, heap and stacks
//   {"cmd":"reboot"}   restart (cause cmd_reboot), after the reply has gone out
//   {"cmd":"ota",...}  remote firmware update, see ota.h
// Any command may carry "req":"<text>", echoed in the reply.
//
// Threading: the modem callback only copies the message into a slot; the
// metering task executes it; the network task publishes the reply.
//
// Nothing here can change how the meter connects (APN, broker, credentials).
// ================================================================
#include <stddef.h>
#include <stdint.h>

#include "at_mqtt.h"

// Filled by main.cpp for each command.
struct RuntimeStatus {
    bool        mqttConnected;
    uint8_t     modemState;        // ModemState
    int         rssiRaw;
    uint32_t    offlineS;
    uint8_t     ladderStep;
    uint32_t    publishOk;
    uint32_t    publishFailed;
    uint32_t    reconnects;
    NetSnapshot net;               // after the last failed attempt
    uint8_t     pzemValidMask;
    uint32_t    stackFreeMeter;    // bytes never used, per task
    uint32_t    stackFreeNetwork;
    uint32_t    stackFreePower;
};

void commands_init();

// Modem callback (network task). Ignores anything not on MQTT_TOPIC_SUB.
void commands_on_message(const char* topic, const char* payload);

// Metering task: runs at most one pending MQTT or serial command.
void commands_process(const RuntimeStatus& status);

// Network task: fetches the reply to publish on MQTT_TOPIC_ACK, if any.
bool commands_take_ack(char* out, size_t outLen);

// A command asked for a reboot (delayed so the reply can go out first).
bool commands_restart_due();
