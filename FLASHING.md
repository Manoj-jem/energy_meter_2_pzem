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
| 1 | GPIO16 | GPIO17 | 0x01 |
| 2 | GPIO4 | GPIO5 | 0x02 |
| 3 | GPIO18 | GPIO19 | 0x03 |

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
[CERT] AmazonRootCA1.pem: writing 1188 bytes...
[CERT] AmazonRootCA1.pem: stored
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
| `Connect failed err=32` | TLS failed inside the modem. If `tools/iot_selftest.py` passes, the credentials and AWS are fine. After 2 failures in a row the firmware discards its cert record and re-uploads all three files on the next attempt (look for `RE-PROVISIONING`). If it still fails after that, send the `ATI` revision lines and the `+CCLK` line from the log |
| `Connect failed err=30/31` | TLS fine, broker refused — policy/client-ID problem |
| `[PZEM] chN: short response (0 bytes)` | nothing on that UART: 5 V missing, or RX/TX swapped |

Failures retry with exponential backoff (up to 5 min), so leave the monitor
running rather than resetting.

## 6. Setting the PZEM Modbus addresses

New PZEM-004T units all ship as address `0x01`, so channels 2 and 3 must be
programmed once before they can share a bus layout. Use
`pzem_set_address(channel, oldAddr, newAddr)` from `src/pzem.h`, one unit at a
time. To find a unit whose address you don't know, probe `0xF8` — v3.0 units
answer on it regardless of their configured address, which distinguishes "not
wired" from "wrong address".

## 7. Using cert_uploader (optional)

Only needed to inspect or repair a modem's certificate store out of band; the
firmware provisions itself. It prints what is on the modem, rewrites all three
files, then re-reads the listing to verify.

```bash
cd energy_meter_2_pzem/cert_uploader
pio run -t upload && pio device monitor -b 115200
```

Re-flash the main firmware afterwards — this sketch does nothing else.
