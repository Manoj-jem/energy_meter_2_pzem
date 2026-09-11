


#pragma once
// ================================================================
// config.h 
// ================================================================

// ── Device identity ──────────────────────────────────────────────
// One name everywhere: AWS IoT thing, MQTT client ID, topic prefix and the
// backend's device_id are all "energy-meter-002". Both macros stay string
// literals (tools/iot_selftest.py reads them straight out of this file).
//
//   AWS_THING_NAME / MQTT_CLIENT_ID: the IoT policy scopes this device's own
//   topics (/cmd) with ${iot:Connection.Thing.ThingName}, which AWS fills in
//   only when the client ID equals the thing attached to the certificate.
//
//   DEVICE_ID: the backend (energy_meter_iot_avaronn, mqtt_worker.py)
//   subscribes to "+/data" and takes device_id from the topic prefix, then
//   picks the decoder from public.device.payload_profile. The row for this
//   meter is "energy-meter-002" (payload_profile energywise_pzem_v1). Do NOT
//   use "energy_meter_002": that is a different row, registered as an MFM384
//   meter, and the MFM384 decoder turns a PZEM frame into all-null readings
//   without any error.
#define AWS_THING_NAME      "energy-meter-002"
#define DEVICE_ID           "energy-meter-002"
#define FIRMWARE_VERSION    "1.0.0"

#define PZEM_BAUD           9600
#define PZEM1_RX_PIN        16
#define PZEM1_TX_PIN        17
#define PZEM2_RX_PIN        4
#define PZEM2_TX_PIN        5
#define PZEM3_RX_PIN        18
#define PZEM3_TX_PIN        19
#define PZEM1_ADDR          0x01
#define PZEM2_ADDR          0x02
#define PZEM3_ADDR          0x03
#define PZEM_TIMEOUT_MS     500

// Phase 1 production storage and power-state signals.
#define FLASH_SCK_PIN       22
#define FLASH_MISO_PIN      21
#define FLASH_MOSI_PIN      23
#define FLASH_CS_PIN        25
#define FLASH_SIZE_BYTES    (8UL * 1024UL * 1024UL)
#define PWR_FAIL_PIN        34  // Active high, externally biased.
#define BACKUP_LOW_PIN      35  // Active high, externally biased.

#define GSM_RX_PIN          26
#define GSM_TX_PIN          27
#define GSM_UART_BAUD       115200
#define GSM_PWRKEY_PIN      -1
#define GSM_RESET_PIN       -1

#define APN                 "airtelgprs.com"
#define APN_USER            ""
#define APN_PASS            ""

// AWS account 481665103941, ap-south-1 — the account that ISSUED the
// certificate in ../certs/energy-meter-002/. This is not a free choice: a
// client certificate from one AWS account presented to a different account's
// broker can never authenticate, and the modem reports that only as
// CMQTTCONNECT err=32, indistinguishable from a missing cert file.
//
// The previous value was a1d3i8d08oi632-ats — account 571751567031, where
// energy_meter_001 lives. That mismatch, not the certificate files, is why
// this device never connected.
//
// The ingest backend for this account is energy_meter_iot_avaronn
// (AWS_IOT_ENDPOINT=a3nyhs5ft5gkz6-ats). energy_meter_001 stays on the other
// account and is ingested separately. See ../CERTIFICATES.md.
#define MQTT_BROKER_HOST    "a3nyhs5ft5gkz6-ats.iot.ap-south-1.amazonaws.com"
#define MQTT_BROKER_PORT    8883
#define MQTT_USERNAME       ""
#define MQTT_PASSWORD       ""
#define MQTT_CLIENT_ID      AWS_THING_NAME
// meter-energy-002-policy allows publish on */data (the backend's "+/data"
// subscription receives it) and publish/subscribe/receive on
// ${iot:Connection.Thing.ThingName}/*, which is where /cmd lives. AWS drops
// the whole connection on an unauthorised SUBSCRIBE or PUBLISH, so a topic
// outside those two patterns shows up as a reconnect loop, not an error.
#define MQTT_TOPIC_PUB      DEVICE_ID "/data"
#define MQTT_TOPIC_SUB      DEVICE_ID "/cmd"
#define MQTT_QOS            1
#define MQTT_KEEPALIVE_S    60

#define CERT_FILENAME_CA    "AmazonRootCA1.pem"
#define CERT_FILENAME_CERT  "em002_cert.pem"
#define CERT_FILENAME_KEY   "em002_key.pem"

#define AT_DEFAULT_TIMEOUT_MS   5000
#define AT_CONNECT_TIMEOUT_MS   20000
#define AT_PUB_TIMEOUT_MS       10000

#define SEND_INTERVAL_MS        2000
#define TASK_WATCHDOG_TIMEOUT_S 240
#define RSSI_SAMPLE_INTERVAL_MS 30000UL
#define RSSI_LOW_RAW            10
#define RSSI_RECOVER_RAW        14
#define RSSI_CONSECUTIVE_COUNT  3

#define CT_PRIMARY_A            300.0f
#define CT_SECONDARY_MA         100.0f
#define PZEM_REFERENCE_PRIMARY_A 100.0f
#define CT_NOMINAL_MULTIPLIER   (CT_PRIMARY_A / PZEM_REFERENCE_PRIMARY_A)
#define CT_PHASE1_GAIN          1.0f
#define CT_PHASE2_GAIN          1.0f
#define CT_PHASE3_GAIN          1.0f

#define MQTT_PAYLOAD_BUF_SIZE   1400

#define DEBUG_ENABLED
#ifdef DEBUG_ENABLED
  #define DBG(x)    Serial.print(x)
  #define DBGLN(x)  Serial.println(x)
  #define DBGF(...) Serial.printf(__VA_ARGS__)
#else
  #define DBG(x)
  #define DBGLN(x)
  #define DBGF(...)
#endif
