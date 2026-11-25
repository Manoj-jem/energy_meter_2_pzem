# MQTT session drops: diagnosis and fix (meter fw 2.0.1, to port to the gateway)

This file is self-contained context for applying the same fix to `gateway_firmware`
(or to any other firmware built on the same `at_mqtt.cpp`).

- Reference implementation: branch `Firmware-26`, `energy_meter_2_firmware/src/at_mqtt.cpp`
  (`at_mqtt_publish_to()` and the helpers above it), `src/netlog.cpp`, `src/commands.cpp` (`log`).
- Applies to: every energy-meter firmware (1.0.0 and 2.0.0) and every gateway firmware up to the
  current `Gateway-0xx` branches. The publish code is the same in all of them, line for line.

---

## 1. Symptom and evidence

Bench meter energy-meter-26 (fw 2.0.0, A7670C, CSQ 31 = strongest signal), 3 h 26 min uptime,
`{"cmd":"diag"}`:

| counter | value | meaning |
|---|---|---|
| reconnects | 31 | the MQTT session was rebuilt about every 6.6 min |
| publish_failed | 33 | about 1 failed publish per reconnect |
| publish_ok | 4086 | about 6180 readings were taken in that time (one per 2 s) |
| net.at_ok | false, stage `connect` | at least one reconnect attempt found the modem not answering AT |

Memory, stacks and flash were healthy. Signal was perfect. So this is not RAM, coverage or the
SIM: the firmware itself throws away a working session.

Cost per drop: the session is torn down (CMQTTDISC/REL/STOP), then rebuilt (cert check, clock
sync, SSL config, CMQTTSTART, ACCQ, CONNECT, SUB). If that attempt fails, the full modem init runs
too. Each drop costs about 1–2 min offline, and while offline the meter keeps only 1 reading per
30 s. 30 drops × 1–2 min ≈ 30–40 % of the time offline at full signal. In the field, with
weaker signal, every step is slower and more likely to fail. That fits "meters keep going offline".

## 2. Root cause (code)

`at_mqtt_publish_to()` publishes in four AT steps: `CMQTTTOPIC` → `>` → topic → `OK`,
`CMQTTPAYLOAD` → `>` → payload → `OK`, then `CMQTTPUB` → `OK` … `+CMQTTPUB: 0,<err>`.

In the old code, **one** hiccup in any step set `_connected = false`, and the session was rebuilt:

| step | old behaviour | why it misfires on a healthy link |
|---|---|---|
| `>` prompt for TOPIC/PAYLOAD | not seen in 5 s → `_connected = false` ("session dead") | the modem is busy (a previous PUBACK, an inbound message, network registration work); 5 s is not proof of a dead session |
| reply after the topic | **exactly one line** read; if it is not `OK` → "Topic rejected", `_connected = false` | any unsolicited line arriving at that moment (`+CTZV`/`*PSUTTZ` network time, `+CGEV` PDP notices, `+CREG`, a late `+CMQTTPUB` from an earlier publish, SMS/CBM notices) is taken as "the reply" |
| reply after the payload | the same one-line read (did not drop, but abandoned the publish) | the same |
| `CMQTTPUB` | the modem was told `pub_timeout = 60` s; the firmware stopped listening after `AT_PUB_TIMEOUT_MS` = 10 s → `_connected = false` | a QoS 1 PUBACK over GSM can take 10+ s; the firmware then talks over a publish the modem is still running, which produces more garbage replies and more drops |

Gateway file positions (branch Gateway-007, `gateway_firmware/src/at_mqtt.cpp`):
- lines ~1095–1140: the publish steps;
- `AT+CMQTTPUB=0,%d,60` at ~1131 with `AT_PUB_TIMEOUT_MS` (10000, `include/config.h:150`);
- the `PING failed` path at ~1239 (only in the unused `at_mqtt_maintain()`).

Gateway-specific gap: the gateway's `networkTask` (`src/main.cpp` ~543–640) has **no** "N failed
publishes in a row → teardown" rule. It relies entirely on `at_mqtt` flipping `_connected`. After
the fix below, `at_mqtt` keeps the session on a single failure, so the gateway needs that rule
added (see 3.4). Otherwise a half-open session could keep failing without being rebuilt.

## 3. The fix

### 3.1 Principles
1. A failed publish is a failed publish, **not** a dead session.
2. Drop the session only on evidence:
   - the modem reports `+CMQTTCONNLOST: 0,<cause>`;
   - `+CMQTTPUB` returns err 11 ("no connection");
   - a probe confirms the session is gone: the modem does not answer `AT`, or `AT+CMQTTCONNECT?`
     no longer lists the broker;
   - the network task sees `PUBLISH_FAILS_OFFLINE` (3) failed publishes in a row.
3. Wait for replies properly: skip unrelated lines, and end the wait only on `OK`, `ERROR` or the
   expected result line.
4. Give the modem its own PUBACK timeout and wait **longer** than it, so the firmware always reads
   the modem's verdict and never talks over an unfinished publish.
5. Log every failure (which step, err, duration, probe verdict) to a log that can be read remotely.

### 3.2 config.h

```c
#define MQTT_PUB_ACK_TIMEOUT_S  20                                   // passed to AT+CMQTTPUB
#define AT_PUB_WAIT_MS          ((MQTT_PUB_ACK_TIMEOUT_S + 5) * 1000UL)
#define PUB_SLOW_LOG_MS         5000                                 // log publishes slower than this
// AT_PUB_TIMEOUT_MS stays 10000: still used for cert writes and HTTP reads.
```

### 3.3 at_mqtt.cpp

New helpers (place them above `at_mqtt_publish_to`):

```cpp
static bool _is_ok_line(const char* line)    { return strcmp(line, "OK") == 0; }
static bool _is_error_line(const char* line) { return strstr(line, "ERROR") != nullptr; }

// '>' prompt; false on timeout or an ERROR line; any other line is an unrelated URC.
static bool _wait_prompt(uint32_t timeoutMs) {
    const uint32_t deadline = millis() + timeoutMs;
    while ((int32_t)(millis() - deadline) < 0) {
        if (!gsm.available()) { esp_task_wdt_reset(); delay(1); continue; }
        const char c = (char)gsm.read();
        if (c == '>' && _linePos == 0 && _rxRawLeft == 0) return true;
        if (_feed(c) && _is_error_line(_line)) return false;
    }
    return false;
}

// 1 = OK, 0 = ERROR, -1 = timeout. Unrelated lines are skipped.
static int _wait_result(uint32_t timeoutMs) {
    char line[256];
    const uint32_t deadline = millis() + timeoutMs;
    while ((int32_t)(millis() - deadline) < 0) {
        if (!_read_line(line, sizeof(line), 200)) continue;
        if (_is_ok_line(line)) return 1;
        if (_is_error_line(line)) return 0;
    }
    return -1;
}

// Is the session still there? ESC first: it leaves data-input mode if a '>' was missed
// (it is ignored in command mode). Connected: +CMQTTCONNECT: 0,"tcp://host:8883",60,1
static bool _session_alive(char* why, size_t whyLen) {
    gsm.write((uint8_t)0x1B); delay(300); _flush_rx();
    if (!_at_cmd("AT", 1500) && !_at_cmd("AT", 1500)) { snprintf(why, whyLen, "modem not answering AT"); return false; }
    _flush_rx();
    gsm.print("AT+CMQTTCONNECT?\r\n");
    char line[256]; bool connected = false, answered = false;
    const uint32_t deadline = millis() + AT_DEFAULT_TIMEOUT_MS;
    while ((int32_t)(millis() - deadline) < 0) {
        if (!_read_line(line, sizeof(line), 200)) continue;
        if (strstr(line, "+CMQTTCONNECT:")) { answered = true; connected = strstr(line, "tcp://") != nullptr; continue; }
        if (_is_ok_line(line) || _is_error_line(line)) break;
    }
    if (!connected) snprintf(why, whyLen, answered ? "modem reports not connected" : "no CMQTTCONNECT? answer");
    return connected;
}
```

The publish then follows this flow (the full code is in the meter file):

```
TOPIC:   send AT+CMQTTTOPIC  -> _wait_prompt -> send topic   -> _wait_result == 1   else failedAt="topic prompt|reply"
PAYLOAD: send AT+CMQTTPAYLOAD-> _wait_prompt -> send payload -> _wait_result == 1   else failedAt="payload prompt|reply"
PUB:     AT+CMQTTPUB=0,<qos>,MQTT_PUB_ACK_TIMEOUT_S via _at_cmd_two_stage(..., AT_PUB_WAIT_MS)
         err 0 -> ok; err 11 -> session gone; other err -> soft failure
on failure:
         if err 11            -> _drop_session("pub err 11 no connection")
         else if probe alive  -> keep session (count kept_after_fail), return false
         else                 -> _drop_session(<probe reason>)
always:  record duration (last/max), log failures and slow publishes
```

`_drop_session(why)` is the **only** place that clears `_connected` for a live session. It counts
the drop, stores `last_drop`, and writes `mqtt drop: <why>` to the connection log. Use it in:
- the `+CMQTTCONNLOST` handler in `_mqtt_rx_line()`, with
  cause 1 = network, 2 = keepalive, 3 = broker closed, 4 = heartbeat;
- `at_mqtt_disconnect()`, as "teardown by network task".

Also changed: the `CMQTTSUBTOPIC` reply now uses `_wait_prompt`/`_wait_result` (the same one-line
bug). Connect failures (`cert store`, `CMQTTSTART err`, `connect err=N`, no result) are logged.

Stats exposed to `diag` (`AtMqttStats at_mqtt_stats()`): `pub_ok`, `pub_failed`,
`kept_after_fail`, `session_drops`, `connlost`, `last_pub_ms`, `max_pub_ms`, `last_drop`.

### 3.4 Network task (main.cpp)

The meter already had this rule; **the gateway must add it**:

```cpp
// after a failed live/replay publish:
if (++publishFailStreak >= PUBLISH_FAILS_OFFLINE && at_mqtt_is_connected()) {
    netlog_add("%u pub fails in a row: teardown", (unsigned)publishFailStreak);
    at_mqtt_disconnect();
}
// after a successful publish: publishFailStreak = 0;
```

Log lines added: `mqtt down after <n>s up: <last_drop>`,
`mqtt up: attempt <ms>[ (modem init)], down <s>`, `connect fail <ms> at=<0|1> <snapshot>`, and
`ladder step <n> after <s>s offline`. `diag` gained `down_total_s` and `net.age_s`: the net
snapshot is taken after a *failed* attempt, so it describes that moment, not the present.

Gateway: also change `if (now >= nextConnectAt)` to `if ((int32_t)(now - nextConnectAt) >= 0)`.
The current form misbehaves when `millis()` wraps after 49.7 days.

## 4. Remote log (`{"cmd":"log"}`)

The USB log needs a laptop on site. 2.0.1 keeps a connection log on the device that can be read
over MQTT:

- `src/netlog.cpp`: 40 entries × 68 bytes in `RTC_NOINIT` RAM, about 2.7 KB.
  - It **survives** software restarts, watchdog resets and crashes, so the lines leading up to a
    `net_loss` self-restart or a crash can still be read afterwards.
  - It is cleared on power-on.
  - It is independent of `DEBUG_ENABLED`, and thread-safe (portMUX).
- Entry: `seq` (continues across restarts), `boot` (low 16 bits of boot_count), `uptime_s`,
  `epoch_s` (0 before the clock is set), and 53 characters of text.
- Commands on `<thing>/cmd`; replies arrive on `<thing>/cmd/ack`:
  - `{"cmd":"log"}` returns page 0 (the oldest 12 lines); `{"cmd":"log","page":1}` the next 12,
    and so on;
  - each reply says `total`, `page` and `pages`;
  - `{"cmd":"log","clear":true}` empties the log (`seq` keeps counting).
- Line format: `#<seq> b<boot> +<uptime>s <MM-DDTHH:MM:SSZ|-> <text>`, for example:
  - `#57 b3 +8123s 11-21T10:02:03Z pub fail topic reply err=-1 5012ms -> session ok`
  - `#58 b3 +8410s 11-21T10:06:50Z mqtt drop: CONNLOST cause=3 (broker closed)`
- Reply buffer: `CMD_ACK_MAX` = 1400 (was 768). The command and network-task buffers are
  `static`, so they don't take task stack.

Gateway port: copy `include/netlog.h` and `src/netlog.cpp` unchanged (they need only `diag_info()`,
`diag_uptime_s()`, `reset_reason_name()`, `restart_cause_name()` and `time_service_now_ms()`; the
gateway has equivalents or they can be stubbed). Then add the `log` command to the gateway's
command handler.

## 5. How to read the results

| What `diag` / `log` shows | Meaning | Next step |
|---|---|---|
| `kept_after_fail` rising, `session_drops` ~0 | the fix is working: hiccups no longer cost a reconnect | none |
| `pub fail topic reply … -> session ok` | an unrelated URC or a slow modem; harmless now | none |
| `pub fail pub no result … ` / high `max_pub_ms` | the PUBACK path is slow (network latency) | consider QoS 0 for telemetry |
| `mqtt drop: CONNLOST cause=2/4` | keepalive lost: the radio link stalls | check signal and coverage; keepalive 60 s |
| `mqtt drop: CONNLOST cause=3` | **the broker closed the session**: a duplicate client ID (another board with the same thing name), a policy denial, or a payload over the limit | check AWS IoT logs and the client ID |
| `mqtt drop: modem not answering AT` | the modem hangs | the recovery ladder (radio cycle → CRESET → restart) handles it; hardware PWRKEY on the next PCB |
| `connect fail … (modem init)` repeated | reconnects are slow because each one re-initialises the modem | fix 2 below |

## 6. Not in this change (next)

- **Fast reconnect (point 2).** After a drop, retry only the MQTT layer (`CMQTTSTART`, `ACCQ`,
  `CONNECT`, `SUB`). Skip the cert check, clock sync and SSL configuration, which have not changed,
  and run `gsm_modem_init()` again only if the modem does not answer `AT`. Do this after the remote
  logs show what reconnects cost.
- Telemetry at QoS 0 with record-level de-duplication would remove the PUBACK wait altogether.
  Decide this with the backend.

## 7. Verification (meter 26, then the gateway)

1. Build and flash or OTA 2.0.1. `{"cmd":"diag"}` → `mqtt.session_drops`, `kept_after_fail`,
   `max_pub_ms`.
2. After 3–4 h, compare with 2.0.0's baseline (31 reconnects in 3.4 h). Expect a few drops at most,
   each explained in `{"cmd":"log"}`.
3. Check that `publish_ok` is close to `uptime_s / 2` (plus events).
4. Pull the antenna for 2 min: expect `pub fail … -> modem reports not connected` or
   `CONNLOST cause=1/2`, then `mqtt up …` after it returns. Check the log still reads correctly
   after `{"cmd":"reboot"}` (RTC survives).
5. The release tool needs the right AWS account:
   `AWS_PROFILE=energywise python tools/ota_release.py --env energy-meter-26 --bucket energywise-gateway-firmware [--send]`.
   The default profile on the dev PC is a different account (AccessDenied). Without `--send`, the
   tool still uploads the image to S3 and creates the stream; it only skips the publish.
