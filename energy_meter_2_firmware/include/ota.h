#pragma once
// ================================================================
// ota.h - remote firmware update over the GSM link
//
// Same mechanism as gateway_firmware (in use there since 1.1.2); only the
// identity marker ("EMPZ-ID:<thing>") and the abort condition differ.
//
//   {"cmd":"ota","stream":"energy-meter-27_2_1_0_a4ef05d9","stream_ver":1,"size":401234,
//    "sha256":"<64 hex>","version":"2.0.1","force":false,"req":"abc"}
//   ("url":"http://..." instead of "stream" downloads over HTTP; see below)
//
// The meter task validates the command (ota_request) and the network task,
// which owns the modem, runs it (ota_run). With "stream" the image comes as an
// AWS IoT file stream over the existing MQTT session to MQTT_BROKER_HOST
// (iot.energywise.tech): the A7670C cannot complete TLS with S3, whose 4.4 KB
// chain is as large as the default IoT endpoint's. With "url" MQTT is closed and
// the modem's HTTP client fetches Range chunks; that only works with a host whose
// TLS chain the modem accepts, or plain http. Either way the image goes straight
// into the spare OTA slot, and it is accepted
// only if its size and SHA-256 match the command and it carries this meter's
// identity marker (OTA_IDENTITY). A binary built for another meter branch (or a
// gateway) is refused, so an update can never clone another device's AWS identity.
//
// Trial and rollback. The new image boots on trial. Each boot on trial counts;
// the first MQTT connect ends the trial (ota_on_connected). After
// OTA_TRIAL_MAX_BOOTS boots without one, or if the bootloader itself rolled
// back (an image that crashes before setup), the previous image is booted and
// reports "rolled back" on its next connect. A trial image that has not
// connected within OTA_TRIAL_CONNECT_MS restarts, which counts as a boot.
//
// Telemetry is not lost during a download: with MQTT down, records go to the
// SPI-flash queue and are replayed after reconnect.
// ================================================================
#include <ArduinoJson.h>
#include <stddef.h>

// setup(), before the tasks start. May switch to the previous image and restart.
void ota_boot_check();

// Meter task (commands.cpp). Validates and queues an "ota" command.
bool ota_request(JsonDocument& request, char* err, size_t errLen);

// MQTT message callback (network task): takes the stream's data/rejected topics.
void ota_on_message(const char* topic, const char* payload);

// Network task. True while an accepted request waits for ota_run().
bool ota_pending();

// Network task. Downloads, verifies and installs the image, then restarts.
// Returns only on failure, with the failure ack queued for ota_take_ack().
// `powerBad` is polled throughout; true aborts the update. On the meter it
// means the flash backlog is above OTA_MAX_BACKLOG: data is sent first.
void ota_run(bool (*powerBad)());

// Network task, after every successful MQTT connect: ends a trial and queues
// the result ack (new firmware running, or rolled back).
void ota_on_connected();

// Network task, every loop: restarts a trial image that cannot connect.
void ota_trial_tick();

// Network task: fetches an OTA reply to publish on MQTT_TOPIC_ACK, if any.
bool ota_take_ack(char* out, size_t outLen);

// For the status command: "idle", "pending", "running" or "trial".
const char* ota_state_name();
