# AWS IoT certificates — energy_meter_2_pzem

## Which AWS account

Meter 002 lives in AWS account **481665103941**, endpoint
`a3nyhs5ft5gkz6-ats.iot.ap-south-1.amazonaws.com` (`MQTT_BROKER_HOST`).

This is not a free choice. A client certificate issued by one AWS account can
never authenticate against another account's broker, and the modem reports that
only as `+CMQTTCONNECT: 0,32` — the same code it gives for a missing file, an
inactive certificate and a clock problem. The endpoint must belong to the
account that issued the files in `certs/energy-meter-002/`.

> **Open item — the backend is still on the other account.**
> `energy_meter_001` lives in account **571751567031**
> (`a1d3i8d08oi632-ats…`), and `energy_meter_backend_iot` subscribes there:
> `app/services/mqtt_worker.py` (`AWS_IOT_ENDPOINT`) and `docker-compose.yml`.
> Until the backend also points at `a3nyhs5ft5gkz6-ats` with a subscriber
> certificate from 481665103941, meter 002's data reaches AWS but never reaches
> the backend. Either migrate meter 001 to the new account, or run a second
> subscriber. Changing the backend endpoint alone would break meter 001.

### Verified AWS-side state (2026-09-10)

| item | value |
|---|---|
| certificate `5efe96f3…2c2b` | **ACTIVE** |
| attached thing | `energy-meter-002` |
| attached policy | `meter-energy-002-policy` |
| `iot:Connect` | `client/*` (any client ID) |
| `iot:Publish` / `Subscribe` / `Receive` | `energy_meter_002/*` |

The certificate originally had **no policy attached**, which is why every
connection was refused. A certificate with zero policies is denied
`iot:Connect`, and AWS signals that by closing the connection without sending a
CONNACK — indistinguishable at the modem from a bad certificate.

Do not attach `energy-meter-002-Policy`: it is the unmodified AWS sample policy
(`sdk/test/*`, `client/sdk-java`) and grants nothing this device needs.

Note the topic scope uses **underscores** (`energy_meter_002/*`) while the thing
uses hyphens (`energy-meter-002`). That is why `MQTT_CLIENT_ID` and the topics
are separate macros in `config.h`.

Verify the credentials at any time, without hardware — this does everything the
firmware's modem does, so a PASS here means any later failure is the modem, the
SIM or the wiring, never the credentials:

```bash
python tools/iot_selftest.py \
    --subscribe energy_meter_002/cmd \
    --publish   energy_meter_002/selftest
```

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

`energy_meter_2_firmware/include/config.h` keeps two names apart on purpose:

| macro | value | must match |
|---|---|---|
| `AWS_THING_NAME` | `energy-meter-002` | the AWS IoT thing name; the IoT policy scopes `iot:Connect` to `client/${iot:Connection.Thing.ThingName}`, so `MQTT_CLIENT_ID` is derived from it |
| `DEVICE_ID` | `energy_meter_002` | the backend's id for this meter; `mqtt_worker.py` subscribes to `+/data` and takes `device_id` from the topic prefix |

Topics are built from `DEVICE_ID`, so the existing backend registration keeps
working. If the IoT policy also scopes `iot:Publish`/`iot:Subscribe` to
`topic/${iot:Connection.Thing.ThingName}/...` rather than `topic/*`, the topics
must move to `AWS_THING_NAME` **and** that id must be registered in the backend
(`devices`, `device_profile`, `device_ct_config`) or every reading is
quarantined to S3 under `unmapped/`.

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
