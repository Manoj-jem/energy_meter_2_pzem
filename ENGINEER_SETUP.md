# Engineer runbook — energy meter 002, from clone to publishing data

Start here. Wiring tables and the failure/cause table live in
[FLASHING.md](FLASHING.md); certificate background is in
[CERTIFICATES.md](CERTIFICATES.md).

Roughly 30 minutes, most of it the first PlatformIO build.

---

## Step 0 — Get the private key (do this first)

**The repository does not contain `device.private.key`.** It is a real
credential and is deliberately gitignored. Everything else you need is in the
repo; without this one file the build stops immediately with:

```
*** gen_certs FAILED ***
       missing file: ...\certs\energy-meter-002\device.private.key
```

Ask the project owner for `device.private.key` for `energy-meter-002` and
receive it out of band — a password manager, an encrypted archive, or a
direct transfer. **Not** email, Slack, or a commit.

You should end up with one file, 1679 bytes, starting with
`-----BEGIN RSA PRIVATE KEY-----`. Keep it; AWS issues a private key only once,
at certificate creation, and cannot re-issue it.

---

## Step 1 — Install the tools

| Tool | Notes |
|---|---|
| [VS Code](https://code.visualstudio.com/) | |
| **PlatformIO IDE** extension | Extensions panel → search "PlatformIO IDE" → Install. First launch downloads its toolchain; let it finish before building. |
| [Git](https://git-scm.com/downloads) | |
| Python 3.9+ | Only for `tools/iot_selftest.py`. The firmware build uses PlatformIO's own bundled Python. |
| USB-serial driver | See below. |

**USB driver.** Look at the chip beside the USB socket on the ESP32 board:

- **CP2102 / CP2104** → Silicon Labs CP210x VCP driver
- **CH340 / CH9102** → WCH CH34x driver

Without it the board never appears as a COM port. Install, then replug.

---

## Step 2 — Clone and open

```bash
git clone https://github.com/Manoj-jem/energy_meter_2_pzem.git
cd energy_meter_2_pzem
```

In VS Code: **File → Open Folder** → select `energy_meter_2_firmware`.

> Open `energy_meter_2_firmware`, **not** the repository root. PlatformIO looks
> for `platformio.ini` in the folder you open. The repo holds two PlatformIO
> projects (`energy_meter_2_firmware` and `cert_uploader`); open whichever one
> you are building.

Wait for the PlatformIO status bar (✓ ➜ 🔌 🗑 at the bottom) to appear.

---

## Step 3 — Drop the private key in

Copy the file from Step 0 to:

```
energy_meter_2_pzem/certs/energy-meter-002/device.private.key
```

That directory should now hold these files:

```
server_ca.pem          1188 bytes   the CA the modem trusts (iot.energywise.tech)
AmazonRootCA1.pem      1187 bytes   kept for reference; not used while server_ca.pem exists
device.cert.pem        1220 bytes
device.private.key     1679 bytes
```

The meter connects to `iot.energywise.tech`, not AWS's default endpoint,
because its modem cannot receive AWS's default certificate chain. See
CERTIFICATES.md.

Nothing else needs configuring — no endpoint to edit, no certificate to paste.
The build reads these three files and generates
`energy_meter_2_firmware/include/certs_generated.h` automatically.

---

## Step 4 — Verify the credentials before touching hardware

```bash
cd energy_meter_2_pzem
python tools/iot_selftest.py \
    --subscribe energy-meter-002/cmd \
    --publish   energy-meter-002/selftest
```

(Use `/selftest`, not `/data`. Anything on `/data` goes into the production
database.)

Expected:

```
TLS       : OK  TLSv1.2 / ECDHE-RSA-AES128-GCM-SHA256
CONNECT   : OK - CONNACK accepted
SUBSCRIBE : OK - SUBACK granted QoS 1
PUBLISH   : OK - PUBACK received
RESULT    : PASS
```

This performs exactly what the modem performs — same endpoint, same mutual
TLS 1.2 handshake, same client ID, same topics — from your laptop. **A PASS
here means any later failure is the modem, the SIM or the wiring, never the
credentials.** That distinction is worth ten seconds, because the modem reports
every certificate, policy, clock and endpoint fault as the same
`+CMQTTCONNECT: 0,32`.

If it fails, stop and fix it. The script prints a checklist per failure mode.
Do not proceed to flashing.

---

## Step 5 — Wire the hardware

Full tables in [FLASHING.md §1](FLASHING.md). The two mistakes that cost the
most time:

1. **RX goes to the other device's TX.** ESP32 GPIO26 ← module TX,
   GPIO27 → module RX. Wiring RX→RX gives `[GSM] No AT response`.
2. **The LTE module needs its own supply**: 3.4–4.2 V, **2 A peak**, with a
   ≥1000 µF capacitor at the module, ground shared with the ESP32. It will not
   run from the ESP32's 3V3 pin — transmit bursts brown it out mid-handshake and
   the symptom looks exactly like a firmware bug.

Insert a data-enabled SIM. The APN is `airtelgprs.com` (`config.h`); change it
if you are not on Airtel.

---

## Step 6 — Connect USB and find the port

```bash
pio device list
```

Note the port (`COM5` on Windows, `/dev/ttyUSB0` on Linux,
`/dev/cu.usbserial-*` on macOS). If nothing is listed, revisit the driver in
Step 1.

---

## Step 7 — Build and flash the microcontroller

```bash
cd energy_meter_2_pzem/energy_meter_2_firmware
pio run                 # build
pio run -t upload       # flash
```

Or use the VS Code status bar: **✓** to build, **➜** to upload.

**Which meter?** Plain `pio run -t upload` builds **energy-meter-002**. For the
third meter, pick its environment explicitly:

```bash
pio run -e energy-meter-3 -t upload
```

(In VS Code, choose the environment in the status bar first.) Both meters use
the same certificate files in `certs/energy-meter-002/`, and differ only in
thing name, client ID and topic (`energy-meter-3/data`). Double-check the
environment before flashing: two meters with the same client ID knock each
other off AWS in an endless reconnect loop. The boot log line
`[MQTT] Identity: client_id=...` shows which one you flashed.

To check meter 3's credentials first:
`python tools/iot_selftest.py --thing energy-meter-3 --publish energy-meter-3/selftest`

The first build downloads the ESP32 toolchain and takes several minutes;
later builds take about 30 seconds.

Expected sizes: RAM ~7.7%, Flash ~26.5%.

**"Failed to connect to ESP32: Timed out waiting for packet header"** — hold
**BOOT**, tap **EN/RST**, release **BOOT**, re-run. Some boards need the manual
sequence; many auto-reset.

To force a port: `pio run -t upload --upload-port COM5`

---

## Step 8 — Flashing the modem happens by itself

There is **no separate certificate-upload step, and no second sketch to flash.**

The firmware carries the three PEM files and provisions the modem over AT
commands on its first connect attempt. It records the certificate's SHA-256
fingerprint in the ESP32's NVS, so it uploads once and skips thereafter —
and re-uploads automatically if you ever rebuild with a different certificate.

Open the monitor and watch it happen:

```bash
pio device monitor -b 115200
```

```
[GSM] Waking modem...
[GSM] GPRS attached
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
[MQTT] Connecting to AWS IoT...
[AT] << +CMQTTCONNECT: 0,0
```

`+CMQTTCONNECT: 0,0` is success. Anything non-zero, see
[FLASHING.md §5](FLASHING.md).

**Then power-cycle the board.** The second boot must print:

```
[CERT] Store OK — energy-meter-002 serial=46E4A726...
```

and **skip** the three uploads. That confirms the modem retained the files and
isn't being rewritten every boot.

---

## Step 9 — Confirm data is arriving

1. **AWS.** IoT console (account **481665103941**, region **ap-south-1**) →
   **MQTT test client** → subscribe to `energy-meter-002/data`. Frames should
   appear every ~2 seconds (`SEND_INTERVAL_MS`).
2. **Backend.** `GET /mqtt/status` on energy_meter_iot_avaronn: `message_count`
   rises, `unmapped_count` stays 0, and `last_message.device_id` is
   `energy-meter-002`. After the next 5-minute boundary, `last_seen_at` on the
   `energy-meter-002` row in `public.device` updates and a `reading_5min_avg`
   row appears for it.

---

## Step 10 — PZEM meters

No PZEM address programming is needed. Each unit has its own RX/TX pins and
is queried on the general address `0xF8`, which a PZEM-004T v3.0 answers
whatever address it is set to (new units ship as `0x01`). A working channel
logs:

```
[PZEM] CH2 OK (unit addr 0x01): V=230.1V I=0.412A ...
```

If a channel logs `[PZEM] Timeout: received 0/25 bytes`, nothing is answering
on that pin pair. That is wiring, not addressing: check 5 V on the PZEM's
4-pin TTL header (not the mains terminals), the shared ground, and that the
PZEM's TX goes to the ESP32 RX pin for that channel (see FLASHING.md §1).

---

## Quick reference

```bash
git clone https://github.com/Manoj-jem/energy_meter_2_pzem.git
cd energy_meter_2_pzem
# copy device.private.key into certs/energy-meter-002/
python tools/iot_selftest.py --publish energy-meter-002/selftest   # expect PASS
cd energy_meter_2_firmware
pio run -t upload
pio device monitor -b 115200
```
