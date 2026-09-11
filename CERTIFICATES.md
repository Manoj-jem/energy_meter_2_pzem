# AWS IoT certificates — energy_meter_2_pzem

## Which AWS account

Meter 002 lives in AWS account **481665103941**, endpoint
`a3nyhs5ft5gkz6-ats.iot.ap-south-1.amazonaws.com` (`MQTT_BROKER_HOST`).

This is not a free choice. A client certificate issued by one AWS account can
never authenticate against another account's broker, and the modem reports that
only as `+CMQTTCONNECT: 0,32` — the same code it gives for a missing file, an
inactive certificate and a clock problem. The endpoint must belong to the
account that issued the files in `certs/energy-meter-002/`.

### Where the data goes

The ingest backend for this account is **energy_meter_iot_avaronn**
(`app/services/mqtt_worker.py`). It connects to `a3nyhs5ft5gkz6-ats`,
subscribes to `+/data`, takes device_id from the topic prefix and picks the
decoder from `public.device.payload_profile`. `energy_meter_001` stays on
account 571751567031 and is ingested separately.

### Verified AWS-side state (2026-09-11)

| item | value |
|---|---|
| certificate `5efe96f3…2c2b` | **ACTIVE**, attached to thing `energy-meter-002` |
| attached policy | `meter-energy-002-policy`, **version 4** |
| `iot:Connect` | `client/*` |
| `iot:Publish` | `*/data`, `${iot:Connection.Thing.ThingName}/*` |
| `iot:Subscribe` | `+/data`, `${iot:Connection.Thing.ThingName}/*` |
| `iot:Receive` | `*/data`, `${iot:Connection.Thing.ThingName}/*` |

**`+` is not a wildcard in a policy resource.** Inside a `topic/...` resource
it is a literal character; only `*` and `?` match. `topicfilter/+/data` is
correct, because it authorises subscribing to the literal filter `+/data`. But
`topic/+/data` matches no real topic. Version 3 of this policy used it for
Publish and Receive, which denied every meter's publish and every delivery to
the backend. AWS answers an unauthorised PUBLISH or SUBSCRIBE by closing the
connection, so this showed up as reconnect loops rather than errors. To roll
back: `aws iot set-default-policy-version --policy-name meter-energy-002-policy --policy-version-id <n>`.

The certificate originally had **no policy attached**, which is why every
connection was refused. A certificate with zero policies is denied
`iot:Connect`, and AWS signals that by closing the connection without sending a
CONNACK — indistinguishable at the modem from a bad certificate.

Do not attach `energy-meter-002-Policy`: it is the unmodified AWS sample policy
(`sdk/test/*`, `client/sdk-java`) and grants nothing this device needs.

Thing name, client ID, topic prefix and backend device_id are all
`energy-meter-002`. See [Device identity](#device-identity).

Verify the credentials at any time, without hardware — this does everything the
firmware's modem does, so a PASS here means any later failure is the modem, the
SIM or the wiring, never the credentials:

```bash
python tools/iot_selftest.py \
    --subscribe energy-meter-002/cmd \
    --publish   energy-meter-002/selftest
```

Publish to `/selftest`, not `/data`. The backend ingests everything on
`*/data` into the production database.

## How it works

There is **one** copy of the certificate material on disk:

```
energy_meter_2_pzem/certs/<thing-name>/
    AmazonRootCA1.pem     Amazon Root CA 1 (same for every device)
    device.cert.pem       the device certificate from AWS IoT
    device.private.key    its matching PRIVATE key
```

`energy_meter_2_firmware/scripts/gen_certs.py` runs before every build of
**both** projects and turns those three files into
`energy_meter_2_firmware/include/certs_generated.h`. Nothing is hand-pasted and
no length is hand-typed, so the firmware and the uploader cannot disagree about
what to write to the modem or under what filename.

The main firmware then provisions the modem itself. On every connect attempt
`at_mqtt_provision_certs()` runs `AT+CCERTLIST`, compares the result against
both the expected filenames and a fingerprint kept in NVS, and re-uploads via
`AT+CCERTDOWN` when anything is missing or stale.

**So the normal workflow is just: build and flash the main firmware.**

## Replacing a certificate

1. In the AWS IoT console, create or locate the certificate for the thing.
   Download the certificate **and the private key** — AWS offers the private
   key only at creation time and never again. The public key is not used.
2. Copy the two files into `certs/<thing-name>/`, renaming them to
   `device.cert.pem` and `device.private.key`.
3. Rebuild. The build fails if the key does not belong to the certificate, if
   the key is a public key, or if any file is missing or malformed.
4. Flash. The device notices the fingerprint no longer matches what it wrote
   last time and re-provisions the modem on the next connect attempt.

Confirm in the console that the certificate is **ACTIVE**, and that it is
attached to both a policy and the thing. A certificate that is inactive or
policy-less fails during the TLS handshake, indistinguishable at the modem from
a missing file — both surface as `+CMQTTCONNECT: 0,32`.

## Device identity

One name, `energy-meter-002`, is used everywhere:

| where | value | why it must match |
|---|---|---|
| AWS IoT thing | `energy-meter-002` | the certificate is attached to it |
| `MQTT_CLIENT_ID` | `energy-meter-002` | `${iot:Connection.Thing.ThingName}` is only filled in when client ID = thing name; the device's `/cmd` rights depend on it |
| `DEVICE_ID` / topic prefix | `energy-meter-002` | the backend takes device_id from the topic |
| `public.device` row | `energy-meter-002`, `payload_profile = energywise_pzem_v1` | selects the PZEM decoder |

There is a second, unrelated row, `energy_meter_002` (underscores), registered
with `payload_profile = mfm384_v1`. Publishing this meter under that name is
**not rejected**. The MFM384 decoder accepts the PZEM frame and produces
all-null voltage, current and energy, which then flows into the rollups
without any error. Keep `DEVICE_ID` on the hyphenated name.

## Adding another meter

Add `certs/<new-thing-name>/` with that device's three files and point the build
at it — no code changes:

```ini
custom_cert_device = energy-meter-003
```

in both `energy_meter_2_firmware/platformio.ini` and
`cert_uploader/platformio.ini`.

## cert_uploader

A recovery tool, not a required step — the firmware provisions itself. Use it to
inspect or repair a modem's certificate store out of band, or to provision a
module not yet wired to a meter. It shares the firmware's `include/` and the
same generated header; do not give it a local `config.h` or `certs.h`.

```
cd cert_uploader && pio run -t upload && pio device monitor
```

## Do not commit private keys

`certs/**/device.private.key` is a real credential. It is covered by
`.gitignore`; keep it that way, and distribute keys out of band.
