# Bring-up: flashing the controller and provisioning the GSM module

Target: ESP32 (`esp32dev`) + SIMCom A7670C / SIM7670C LTE module + 3× PZEM-004T v3.0.

## 0. Prerequisites

```bash
pip install platformio        # or use the PlatformIO IDE extension
```

Find the ESP32's serial port:

```bash
pio device list
```

On Windows it looks like `COM5`. If nothing appears, install the USB-UART
driver for your board (CP2102 or CH340 — the chip is printed next to the USB
socket).

## 1. Wiring

ESP32 pin numbers are GPIO numbers, from `include/config.h`. **RX goes to the
other device's TX** — the most common bring-up mistake is wiring RX→RX.

### LTE module

| ESP32 | Module | Notes |
|---|---|---|
| GPIO26 (`GSM_RX_PIN`) | module **TX** | |
| GPIO27 (`GSM_TX_PIN`) | module **RX** | |
| GND | GND | must be common with the module's supply |
| — | VCC | **3.4–4.2 V, 2 A peak.** See below. |

The LTE module does **not** run off the ESP32's 3V3 pin. Transmit bursts pull
~2 A for a few milliseconds; a weak supply browns out mid-handshake and looks
exactly like a firmware bug. Give it its own regulator plus a bulk capacitor
(≥1000 µF) at the module, and share ground with the ESP32.

`GSM_PWRKEY_PIN` is `-1`, meaning the firmware assumes the module powers on by
itself. If yours needs PWRKEY pulsed, wire it to a spare GPIO and set that pin
number in `config.h` — the firmware already handles the pulse sequence.

Insert a **data-enabled** SIM with an active plan. `APN` in `config.h` is
`airtelgprs.com`; change it if you are not on Airtel.

### PZEM-004T v3.0 (×3)

| Channel | ESP32 RX | ESP32 TX | Modbus addr |
|---|---|---|---|
| 1 | GPIO16 | GPIO17 | 0xF8 (general) |
| 2 | GPIO4 | GPIO5 | 0xF8 (general) |
| 3 | GPIO18 | GPIO19 | 0xF8 (general) |

Each PZEM is alone on its own pin pair, so all three are queried on the
general address `0xF8`. They answer it whatever address they are set to, so
new units (factory address `0x01`) work without any programming.

Each PZEM's TTL header needs **5 V** on its VCC pin (the 4-pin side header, not
the mains terminals) and a common ground with the ESP32. The TTL side is
optically isolated from the mains side; the mains terminals must still be
connected for voltage/current readings to be non-zero.

All three share UART2 — the firmware re-points the GPIO matrix per transaction
(`bind_channel()` in `src/pzem.cpp`).

### SPI flash (store-and-forward queue)

SCK GPIO22, MISO GPIO21, MOSI GPIO23, CS GPIO25.

## 2. Check the credentials before you touch hardware

This takes ten seconds and rules out the entire class of failure that the modem
reports only as the useless `err=32`:

```bash
cd energy_meter_2_pzem
python tools/iot_selftest.py \
    --subscribe energy-meter-002/cmd \
    --publish   energy-meter-002/selftest
```

You want `RESULT : PASS` — it exercises the same TLS 1.2 mutual handshake,
CONNECT, SUBSCRIBE and QoS1 PUBLISH the firmware does. If it fails, **fix that
first**: no amount of flashing will help, because the device does exactly what
this script does. The script prints a checklist for each failure mode.

Verified passing on 2026-09-11 against account 481665103941 (policy version 4,
client ID and topics `energy-meter-002`).

## 3. Flash the firmware

```bash
cd energy_meter_2_pzem/energy_meter_2_firmware
pio run                      # build (also regenerates certs_generated.h)
pio run -t upload            # flash
pio device monitor -b 115200 # watch
```

If upload fails with "Failed to connect to ESP32": hold the **BOOT** button,
tap **EN/RST**, release BOOT, and re-run. Some boards need this; many
auto-reset.

To pick a port explicitly: `pio run -t upload --upload-port COM5`.

One PlatformIO environment per meter; the default is energy-meter-002.

| Meter | Command | Topic |
|---|---|---|
| energy-meter-002 | `pio run -t upload` | `energy-meter-002/data` |
| energy-meter-3 | `pio run -e energy-meter-3 -t upload` | `energy-meter-3/data` |
| energy-meter-4 | `pio run -e energy-meter-4 -t upload` | `energy-meter-4/data` |
| energy-meter-5 | `pio run -e energy-meter-5 -t upload` | `energy-meter-5/data` |
| energy-meter-25 | `pio run -e energy-meter-25 -t upload` | `energy-meter-25/data` |

All use the certificate in `certs/energy-meter-002/`, attached to every AWS
thing. Never flash the same environment onto two meters: a shared client ID
makes AWS disconnect them alternately forever.

**There is no separate certificate-upload step.** The firmware carries the PEMs
and writes them into the modem itself on the first connect attempt.

## 4. What a good first boot looks like

```
[GSM] Waking modem...
[AT] >> AT
[AT] << OK
[GSM] Waiting for GPRS...
[GSM] GPRS attached
[MQTT] Tearing down previous session...
[AT] >> AT+CCERTLIST
[CERT] *** RE-PROVISIONING MODEM CERT STORE ***
[CERT]   reason: files missing — ...
[CERT]   uploading energy-meter-002 serial=46E4A726... valid 2026-09-10 -> 2049-12-31
[CERT] em002_ca.pem: writing 1188 bytes...
[CERT] em002_ca.pem: stored
[CERT] em002_cert.pem: writing 1220 bytes...
[CERT] em002_cert.pem: stored
[CERT] em002_key.pem: writing 1679 bytes...
[CERT] em002_key.pem: stored
[CERT] Provisioned OK — fingerprint 5efe96f3...
[MQTT] Identity: client_id=energy-meter-002 thing=energy-meter-002 ...
[MQTT] Configuring SSL (mTLS)...
[MQTT] Connecting to AWS IoT...
[AT] << +CMQTTCONNECT: 0,0
[MQTT] Connected
```

**Power-cycle and check the second boot** — it must print `[CERT] Store OK` and
*skip* the three uploads. That proves the NVS fingerprint works and the modem
isn't being rewritten on every boot.

## 5. Reading failures

| Log | Meaning |
|---|---|
| `[GSM] No AT response` | wiring (RX/TX swapped), module unpowered, or PWRKEY needed |
| `[GSM] GPRS not attached` | SIM inactive, no data plan, wrong `APN`, or poor signal |
| `[CERT] ... no '>' prompt` | module doesn't support `AT+CCERTDOWN` — check the model |
| `+CCLK: "70/01/01,...` or `[TIME] Could not set the modem clock` | modem clock is at 1970, so TLS rejects AWS's certificate (err 32). The firmware sets it via NTP before each connect; if every NTP server fails, check the data connection (APN, SIM data plan) |
| `Connect failed err=32` | TLS failed inside the modem. If `tools/iot_selftest.py` passes, the credentials and AWS are fine. After 2 failures in a row the firmware discards its cert record and re-uploads all three files on the next attempt (look for `RE-PROVISIONING`). If it still fails after that, send the `ATI` revision lines and the `+CCLK` line from the log |
| `Connect failed err=30/31` | TLS fine, broker refused — policy/client-ID problem |
| `[PZEM] chN: short response (0 bytes)` | nothing on that UART: 5 V missing, or RX/TX swapped |

Failures retry with exponential backoff (up to 5 min), so leave the monitor
running rather than resetting.

## 6. Setting the PZEM Modbus addresses

**Not needed.** Every PZEM has its own RX/TX pins, and the firmware queries
each one on the general address `0xF8`, which a v3.0 unit answers whatever
address it is set to. The log shows each unit's real address:
`[PZEM] CH2 OK (unit addr 0x01): ...`.

Only if units are ever wired onto one **shared** bus would they need distinct
addresses. In that case use `pzem_set_address(channel, oldAddr, newAddr)`
from `src/pzem.h`, one unit at a time, and set `PZEMn_ADDR` in `config.h` to
match.

If a channel still logs `received 0/25 bytes`, it is wiring, not
addressing. Check 5 V and GND on that PZEM's TTL header, and that the PZEM's
TX goes to the ESP32 RX pin in the table above.

## 7. Using cert_uploader (optional)

Only needed to inspect or repair a modem's certificate store out of band; the
firmware provisions itself. It prints what is on the modem, rewrites all three
files, then re-reads the listing to verify.

```bash
cd energy_meter_2_pzem/cert_uploader
pio run -t upload && pio device monitor -b 115200
```

Re-flash the main firmware afterwards — this sketch does nothing else.

## 8. Firmware 2.x: remote update (OTA) and diagnostics

From firmware **2.0.0** (branches `Firmware-26`, `Firmware-14`, `Firmware-6` and `Firmware-27` to `Firmware-35`, and every new meter
after it) a meter can be updated through its SIM, without a site visit. Meters on 1.0.0 have
no OTA: they need **one** USB flash with a 2.x build first. Installed 1.0.0 meters are left as
they are; the backend accepts both (`schema_version` 2 and 3).

The mechanism is the gateways' (gateway_firmware/FLASHING.md section 8): the image comes as an
**AWS IoT file stream** over the meter's existing MQTT session to `iot.energywise.tech`, in
512-byte blocks, because the A7670C cannot open TLS to S3. It goes into the spare app slot of the
default partition table (2 x 1.25 MB; the 2.0.0 image is ~390 KB).

**AWS: nothing to set up per meter.** The meters use the same certificate as the gateways, and
that certificate already carries `gateway-ota-stream-policy`
(`$aws/things/${ThingName}/streams/*`). Bucket `energywise-gateway-firmware` and role
`energywise-iot-ota-stream` are shared. A new meter only needs its thing:
`tools/create_iot_thing.sh energy-meter-NN`.

### Releasing an update

1. Bump `FIRMWARE_VERSION` in `energy_meter_2_firmware/include/config.h`. The meter refuses the
   version it already runs (unless `--force`).
2. `python energy_meter_2_firmware/tools/run_host_tests.py` and
   `cd energy_meter_2_firmware && pio run -e energy-meter-31`
3. Dry run (uploads to S3 and creates the stream, sends nothing):
   ```powershell
   $env:AWS_PROFILE = "em-481665103941"
   python tools/ota_release.py --env energy-meter-31 --bucket energywise-gateway-firmware
   ```
4. Send: the same with `--send`, or publish the printed JSON to `energy-meter-31/cmd`.
5. Watch `energy-meter-31/cmd/ack`: `accepted, downloading` -> `progress` every 64 KB ->
   (restart) -> `running new firmware`. About 7-10 minutes for 0.4 MB.

Telemetry is not lost during the download: it goes to the flash queue and is replayed after.
The image is accepted only if size and SHA-256 match and it carries this meter's identity marker
(`EMPZ-ID:energy-meter-31`): an image for another meter, or a gateway image, is refused.

**Trial and rollback.** The new image must reach AWS within 15 minutes, at most 3 boots;
otherwise the previous image boots again and reports `"error":"rolled back from <version>"`.
An update does not start while more than 1000 records wait in the flash queue (data first).

| Reply | Meaning |
|---|---|
| `cannot subscribe to the stream topics` | stream policy missing on this certificate |
| `stream rejected: ResourceNotFound` | wrong stream id, or the stream was deleted |
| `no data from stream at byte N` | poor signal, retry later |
| `sha256 mismatch` | image differs from the command: rerun the tool |
| `image is not built for this meter` | wrong branch / env |
| `flash backlog too large` | the meter is still replaying offline data: retry later |

### Commands (`energy-meter-31/cmd`, replies on `/cmd/ack`; also as a JSON line on USB serial)

| Command | Reply |
|---|---|
| `{"cmd":"status"}` | fw, uptime, boot_count, reset_reason, restart_cause, ota state, MQTT/modem, offline_s, ladder step, flash queue; from 2.1.0 also `signal` (quality, csq, rssi_dbm, rat, band, rsrp_dbm, rsrq_db, sinr_db) and `sim` (iccid, imsi, msisdn, operator) |
| `{"cmd":"diag"}` | counters (power-ons, brown-outs, crashes, network restarts, publishes), previous boot, last network snapshot (SIM, CREG/CEREG, CSQ, stage, error), flash health (JEDEC, size, corrupt slots), heap and task stacks; from 2.0.1 also `mqtt` (publish ok/failed, kept_after_fail, session_drops, connlost, last/max publish ms, last_drop) and `down_total_s` |
| `{"cmd":"log"}` / `{"cmd":"log","page":N}` | (2.0.1) connection log: the last 40 lines (publish failures, why the session dropped, reconnect times, recovery steps), 12 per reply, oldest first. Survives restarts and crashes, cleared by a power cut. `{"cmd":"log","clear":true}` empties it. See `MQTT_SESSION_DROPS.md` |
| `{"cmd":"reboot"}` | restarts after 4 s (cause `cmd_reboot`) |
| `{"cmd":"ota",...}` | see above |

### What 2.x tells the server (schema 3)

Every reading has a `diag` block: `uptime_s`, `boot_count`, `reset_reason`, `restart_cause`,
`fast_boots`, `offline_s`, `ladder_step`, `safe_mode`, `min_free_heap`, `events_suppressed`.
Events have a `detail` block (`ts`, `a`, `b`, `v`, `text`).

From **2.1.0** every reading's `health` block also has `signal`:
`{"quality":"good","csq":21,"rssi_dbm":-71,"rat":"lte","band":40,"rsrp_dbm":-98,"rsrq_db":-11,"sinr_db":8}`
(quality from RSRP on LTE: >= -90 excellent, >= -105 good, >= -115 fair, else poor; from CSQ
otherwise. Unknown values are `null`). Every event also has
`"sim":{"iccid":"...","imsi":"...","msisdn":"...","operator":"..."}`. `msisdn` is usually empty:
most SIMs do not store their own number, so match SIMs by the ICCID printed on the card.
The flash record format changed (version 4): unsent records from an older version are
discarded on the first boot of 2.1.0. Let a meter empty its backlog before flashing it.

| Question | Answer in the data |
|---|---|
| Was the power cut? | `BOOT` with `reset_reason` `power_on`, and `POWER_OUTAGE`: `detail.ts` = last alive (power still on, +-60 s), `event_ts` = power back, `detail.a` = duration s, `detail.v` = phase voltages at last alive. This board has no battery, so it cannot report *before* it dies; it proves the cut afterwards. |
| Did it crash / boot-loop? | `CRASH_RESET` (`panic`, `task_wdt`, `int_wdt`), `fast_boots` rising; at 5 short boots in a row `SAFE_MODE` (metering + MQTT + OTA only, so it can still be fixed remotely). |
| Was it powered but offline? | Replayed records with continuous `uptime_s` across the gap; `NET_RECOVERED` (`detail.a` = offline s, `detail.b` = highest recovery step, `detail.text` = last SIM/registration/error state); `SELF_RESTART` with `restart_cause` `net_loss`. |
| Is the flash working? | `STORAGE_READY` (JEDEC, size, corrupt slots) at every boot, or `STORAGE_UNAVAILABLE` with the failed step, re-sent every hour. |

**Network self-recovery** (offline = no acknowledged publish): 10 min -> MQTT teardown + radio
off/on (`AT+CFUN=0/1`); 20 min -> modem reset (`AT+CRESET`); 30 min -> ESP restart (cause
`net_loss`, the flash queue survives). After such a restart the next one waits 60 min. Each step
is logged (`NET_RECOVERY_STEP`). The modem module has no reset/power line to the ESP32, so a
modem that ignores AT commands can only be recovered by a power cycle; a PWRKEY or power-MOSFET
line on the next PCB revision would fix that. Bench build with 1/2/3-minute steps:
`pio run -e nettest -t upload`.

**While offline** one reading per 30 s goes to flash (about 11 days of buffer on 8 MB; live rate
stays 2 s). GPIO34/35 (PWR_FAIL / BACKUP_LOW) are not connected on this board and are ignored
(`HAS_POWER_SENSE=0`): they used to float and produce thousands of false "backup low" events.

**Flash format 3.** 2.x changed the record format. Data a 1.0.0 meter left on the chip is not
replayed; it is erased as the ring reaches it. Later versions must keep format 3 or migrate it,
because an OTA-updated meter keeps its flash.
