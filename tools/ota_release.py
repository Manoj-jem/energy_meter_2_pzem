#!/usr/bin/env python3
"""Release an energy-meter firmware build over the air (GSM).

Checks the built image, uploads it to a private S3 bucket, creates an AWS IoT
file stream for it and prints (or, with --send, publishes) the {"cmd":"ota",...}
command for the meter's <thing>/cmd topic. See FLASHING.md, "Remote update".
Same mechanism, bucket and IAM role as gateway_firmware/tools/ota_release.py.

    cd energy_meter_2_firmware && pio run -e energy-meter-30 && cd ..
    python tools/ota_release.py --env energy-meter-30 --bucket energywise-gateway-firmware          # dry run
    python tools/ota_release.py --env energy-meter-30 --bucket energywise-gateway-firmware --send   # publishes it

Only firmware 2.x meters can be updated this way (energy-meter-6, energy-meter-14, energy-meter-26 onwards);
meters on 1.0.0 have no OTA.

The meter receives the image over its existing MQTT connection to
iot.energywise.tech (AWS IoT MQTT file delivery), because its modem cannot
complete TLS with S3 directly. AWS IoT reads the object from S3 through the
IAM role --role (one-time setup, FLASHING.md). The image embeds the device's
private key (certs_generated.h), so the bucket must stay private.

--url downloads over HTTP instead (presigned S3 URL, or --url to a host). Over
https this only works with a host whose TLS chain the modem accepts; --http
sends the image, key included, unencrypted. Bench tests only.

Needs the AWS CLI with credentials for account 481665103941
(e.g. $env:AWS_PROFILE = "em-481665103941").
"""
import argparse
import hashlib
import json
import os
import re
import subprocess
import sys

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ROOT = os.path.join(REPO, "energy_meter_2_firmware")
MARKER = b"EMPZ-ID:"   # OTA_IDENTITY in energy_meter_2_firmware/src/ota.cpp
# Installed 1.0.0 meters and bench-only envs: never a release target from here.
NOT_RELEASABLE = {"energy-meter-002", "flashtest", "nettest"}
REGION = "ap-south-1"
ACCOUNT = "481665103941"
DEFAULT_ROLE = "energywise-iot-ota-stream"
RX_PAYLOAD_MAX = 1152   # MQTT_RX_PAYLOAD_MAX in include/config.h
URL_MAX = 900           # URL_MAX in src/ota.cpp


def fail(message):
    print(f"ota_release: {message}", file=sys.stderr)
    sys.exit(1)


def firmware_version():
    with open(os.path.join(ROOT, "include", "config.h"), encoding="utf-8") as f:
        match = re.search(r'#define\s+FIRMWARE_VERSION\s+"([^"]+)"', f.read())
    if not match:
        fail("FIRMWARE_VERSION not found in include/config.h")
    return match.group(1)


def aws(*args, check=True):
    result = subprocess.run(["aws", *args, "--region", REGION, "--output", "json"],
                            capture_output=True, text=True)
    if check and result.returncode != 0:
        fail(f"aws {' '.join(args[:2])} failed:\n{result.stderr.strip()}")
    return result


def ensure_stream(stream_id, bucket, key, role):
    """Creates the IoT stream, or reuses it (the id carries the image hash)."""
    existing = aws("iot", "describe-stream", "--stream-id", stream_id, check=False)
    if existing.returncode == 0:
        return json.loads(existing.stdout)["streamInfo"]["streamVersion"]
    role_arn = role if role.startswith("arn:") else f"arn:aws:iam::{ACCOUNT}:role/{role}"
    files = json.dumps([{"fileId": 0, "s3Location": {"bucket": bucket, "key": key}}])
    created = aws("iot", "create-stream", "--stream-id", stream_id, "--files", files,
                  "--role-arn", role_arn,
                  "--description", f"energy meter firmware {key}")
    return json.loads(created.stdout)["streamVersion"]


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--env", required=True, help="PlatformIO env = AWS thing name, e.g. energy-meter-30")
    parser.add_argument("--bucket", help="private S3 bucket for firmware images")
    parser.add_argument("--role", default=DEFAULT_ROLE, help="IAM role (name or ARN) AWS IoT uses to read the bucket")
    parser.add_argument("--url", nargs="?", const="presign",
                        help="download over HTTP instead of an IoT stream; without a value, a presigned S3 URL")
    parser.add_argument("--expires", type=int, default=6 * 3600, help="presigned URL lifetime, seconds")
    parser.add_argument("--http", action="store_true", help="with --url: plain http (sends the key unencrypted)")
    parser.add_argument("--force", action="store_true", help="reinstall even if the meter runs this version")
    parser.add_argument("--send", action="store_true", help="publish the command to <env>/cmd")
    args = parser.parse_args()

    use_url = args.url is not None
    if not args.bucket and not (use_url and args.url != "presign"):
        fail("--bucket is required (except with --url <address>)")
    if args.env in NOT_RELEASABLE:
        fail(f"{args.env} is not a releasable meter env")

    image_path = os.path.join(ROOT, ".pio", "build", args.env, "firmware.bin")
    if not os.path.isfile(image_path):
        fail(f"{image_path} not found: run  pio run -e {args.env}")
    with open(image_path, "rb") as f:
        image = f.read()

    # The meter refuses an image without its own marker; catch it here first.
    if MARKER + args.env.encode() + b"\0" not in image:
        fail(f"{image_path} was not built for {args.env} (identity marker missing)")

    version = firmware_version()
    sha256 = hashlib.sha256(image).hexdigest()
    print(f"image   : {image_path}")
    print(f"version : {version}")
    print(f"size    : {len(image)} bytes")
    print(f"sha256  : {sha256}")

    key = f"firmware/{args.env}/{version}-{sha256[:12]}.bin"
    if args.bucket:
        aws("s3", "cp", image_path, f"s3://{args.bucket}/{key}", "--only-show-errors")
        print(f"uploaded: s3://{args.bucket}/{key}")

    command = {"cmd": "ota"}
    if not use_url:
        stream_id = re.sub(r"[^A-Za-z0-9_-]", "_", f"{args.env}_{version}_{sha256[:8]}")
        stream_ver = ensure_stream(stream_id, args.bucket, key, args.role)
        print(f"stream  : {stream_id} (version {stream_ver})")
        command.update({"stream": stream_id, "stream_ver": stream_ver})
    else:
        url = args.url
        if url == "presign":
            url = aws("s3", "presign", f"s3://{args.bucket}/{key}",
                      "--expires-in", str(args.expires)).stdout.strip()
        if args.http:
            print("WARNING : plain http sends the device private key unencrypted", file=sys.stderr)
            url = re.sub(r"^https://", "http://", url)
        if len(url) >= URL_MAX or '"' in url:
            fail(f"URL is {len(url)} characters; the meter accepts < {URL_MAX}")
        command["url"] = url

    command.update({"size": len(image), "sha256": sha256, "version": version, "req": f"ota-{version}"})
    if args.force:
        command["force"] = True
    payload = json.dumps(command, separators=(",", ":"))
    if len(payload) >= RX_PAYLOAD_MAX:
        fail(f"command is {len(payload)} bytes; the meter accepts < {RX_PAYLOAD_MAX}")

    topic = f"{args.env}/cmd"
    if not args.send:
        print(f"\nPublish to {topic} (or rerun with --send):\n{payload}")
        return
    aws("iot-data", "publish", "--topic", topic, "--qos", "1",
        "--cli-binary-format", "raw-in-base64-out", "--payload", payload)
    print(f"\nSent to {topic}. Watch {args.env}/cmd/ack for \"accepted, downloading\",")
    print("then, after the reboot, \"running new firmware\" (or a failure / \"rolled back\").")


if __name__ == "__main__":
    main()
