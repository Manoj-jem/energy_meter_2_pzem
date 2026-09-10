


#pragma once
// ================================================================
// config.h 
// ================================================================

// ── Device identity ──────────────────────────────────────────────
// Two DIFFERENT names, deliberately. They are not interchangeable:
//
//   AWS_THING_NAME is the AWS IoT *thing*. The IoT policy scopes
//   iot:Connect to client/${iot:Connection.Thing.ThingName}, so the MQTT
//   client ID must match it character for character or the broker denies
//   the CONNECT *after* a successful TLS handshake.
//
//   DEVICE_ID is the backend's identity for this meter. mqtt_worker.py
//   subscribes to "+/data" and takes device_id from the topic prefix, then
//   looks it up in the device registry. Changing DEVICE_ID without adding
//   the matching backend rows (devices, device_profile, device_ct_config)
//   sends every reading to S3 under unmapped/.
#define AWS_THING_NAME      "energy-meter-002"
#define DEVICE_ID           "energy_meter_002"
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
// NOTE: energy_meter_backend_iot still subscribes on 571751567031
// (app/services/mqtt_worker.py, AWS_IOT_ENDPOINT). Until it also points here,
// meter 002's data will reach AWS but not the backend. See ../CERTIFICATES.md.
#define MQTT_BROKER_HOST    "a3nyhs5ft5gkz6-ats.iot.ap-south-1.amazonaws.com"
#define MQTT_BROKER_PORT    8883
#define MQTT_USERNAME       ""
#define MQTT_PASSWORD       ""
#define MQTT_CLIENT_ID      AWS_THING_NAME
// Topics stay on DEVICE_ID so the existing backend registration keeps
// working. If the IoT policy also scopes iot:Publish/iot:Subscribe to
// topic/${iot:Connection.Thing.ThingName}/... rather than topic/*, these
// must move to AWS_THING_NAME and the backend must register that id too.
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
