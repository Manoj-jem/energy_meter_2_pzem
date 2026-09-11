#!/usr/bin/env python3
"""Prove the AWS IoT credentials work, without any hardware.

Does exactly what the firmware's modem does -- mutual TLS 1.2 to the endpoint
in config.h, using the PEM files in certs/<device>/, then a real MQTT CONNECT
and optionally a QoS1 PUBLISH -- and reports which step failed.

Worth running before every field trip: it separates "the credentials/policy are
wrong" from "the modem or the SIM is wrong", which the modem itself cannot do.
AT+CMQTTCONNECT reports every one of these failures as the same err=32.

Endpoint, client ID and cert directory are parsed out of the firmware's
config.h so this can never drift from what the device actually does.

  python tools/iot_selftest.py
  python tools/iot_selftest.py --publish energy-meter-002/selftest
  python tools/iot_selftest.py --thing energy-meter-3 --publish energy-meter-3/selftest
  python tools/iot_selftest.py --client-id some-other-id
"""
import argparse
import json
import os
import re
import socket
import ssl
import struct
import sys
from datetime import datetime, timezone

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
CONFIG_H = os.path.join(REPO, "energy_meter_2_firmware", "include", "config.h")

CONNACK_REASONS = {
    0: "Connection Accepted",
    1: "Refused: unacceptable protocol version",
    2: "Refused: identifier rejected",
    3: "Refused: server unavailable",
    4: "Refused: bad username or password",
    5: "Refused: not authorized",
}


def config_defines(path):
    """Pull the simple string #defines out of config.h."""
    out = {}
    with open(path, encoding="utf-8") as handle:
        for line in handle:
            m = re.match(r'\s*#define\s+(\w+)\s+"([^"]*)"', line)
            if m:
                out[m.group(1)] = m.group(2)
    return out


# ── minimal MQTT 3.1.1 ──────────────────────────────────────────

def remaining_length(n):
    out = bytearray()
    while True:
        b = n % 128
        n //= 128
        if n:
            b |= 0x80
        out.append(b)
        if not n:
            return bytes(out)


def mqtt_str(s):
    raw = s.encode("utf-8")
    return struct.pack("!H", len(raw)) + raw


def build_connect(client_id, keepalive=60):
    body = (mqtt_str("MQTT") + bytes([4]) + bytes([0x02])
            + struct.pack("!H", keepalive) + mqtt_str(client_id))
    return bytes([0x10]) + remaining_length(len(body)) + body


def build_publish(topic, payload, packet_id=1):
    body = mqtt_str(topic) + struct.pack("!H", packet_id) + payload
    return bytes([0x32]) + remaining_length(len(body)) + body   # QoS1


def build_subscribe(topic_filter, packet_id=2, qos=1):
    body = struct.pack("!H", packet_id) + mqtt_str(topic_filter) + bytes([qos])
    return bytes([0x82]) + remaining_length(len(body)) + body


def read_packet(sock):
    head = sock.recv(1)
    if not head:
        return None, None
    mult, length = 1, 0
    while True:
        b = sock.recv(1)
        if not b:
            return None, None
        length += (b[0] & 127) * mult
        if not (b[0] & 128):
            break
        mult *= 128
    data = b""
    while len(data) < length:
        chunk = sock.recv(length - len(data))
        if not chunk:
            break
        data += chunk
    return head[0], data


NOT_AUTHORIZED_HELP = """
  The TLS handshake completed but the broker hung up without a CONNACK.
  AWS IoT does that -- with no error detail -- for all of these, so check
  them in the console of the account that owns this endpoint:

    1. Is the endpoint the right ACCOUNT's endpoint? A certificate from
       account A can never authenticate against account B's broker.
       AWS IoT Core -> Settings -> Device data endpoint.
    2. Is the certificate registered there, and ACTIVE (not INACTIVE,
       not REVOKED)?  Security -> Certificates.
    3. Does it have a POLICY attached granting iot:Connect for this
       client ID?  A certificate with no policy fails exactly like this.
    4. If the policy scopes iot:Connect to
       client/${iot:Connection.Thing.ThingName}, the certificate must
       also be ATTACHED TO THE THING, and the client ID must equal the
       thing name exactly.

  Enable AWS IoT logs (Settings -> Logs, level INFO) and the CloudWatch
  log group AWSIotLogsV2 will name the exact reason.
"""


def main():
    cfg = config_defines(CONFIG_H)
    ap = argparse.ArgumentParser()
    ap.add_argument("--endpoint", default=cfg.get("MQTT_BROKER_HOST"))
    ap.add_argument("--thing", default=None,
                    help="act as this AWS thing, e.g. energy-meter-3 (sets the "
                         "client ID; the PlatformIO env of the same name builds it)")
    ap.add_argument("--client-id", default=cfg.get("AWS_THING_NAME"))
    ap.add_argument("--device", default="energy-meter-002",
                    help="subdirectory under certs/")
    ap.add_argument("--publish", metavar="TOPIC", default=None)
    ap.add_argument("--subscribe", metavar="FILTER", default=None,
                    help="topic filter to SUBSCRIBE to; the firmware subscribes "
                         "to MQTT_TOPIC_SUB after connecting")
    ap.add_argument("--timeout", type=float, default=20.0)
    args = ap.parse_args()
    if args.thing:
        args.client_id = args.thing

    certs = os.path.join(REPO, "certs", args.device)
    # Same rule as gen_certs.py: the custom-domain server CA written by
    # tools/make_iot_server_cert.sh replaces Amazon Root CA 1 when present.
    ca = os.path.join(certs, "server_ca.pem")
    if not os.path.isfile(ca):
        ca = os.path.join(certs, "AmazonRootCA1.pem")
    crt = os.path.join(certs, "device.cert.pem")
    key = os.path.join(certs, "device.private.key")
    for path in (ca, crt, key):
        if not os.path.isfile(path):
            print("missing: %s" % path)
            return 2

    print("endpoint  : %s:8883" % args.endpoint)
    print("client_id : %s" % args.client_id)
    print("certs     : certs/%s/" % args.device)

    ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_CLIENT)
    # Pinned to 1.2 to match the firmware's AT+CSSLCFG "sslversion",0,3.
    # Under TLS 1.3 the client finishes the handshake before the server
    # validates the client certificate, so a rejected cert would look like
    # a successful handshake followed by a mystery disconnect.
    ctx.minimum_version = ssl.TLSVersion.TLSv1_2
    ctx.maximum_version = ssl.TLSVersion.TLSv1_2
    ctx.load_verify_locations(cafile=ca)
    ctx.load_cert_chain(certfile=crt, keyfile=key)

    try:
        raw = socket.create_connection((args.endpoint, 8883), timeout=args.timeout)
    except OSError as exc:
        print("TCP       : FAILED - %s" % exc)
        return 2

    try:
        sock = ctx.wrap_socket(raw, server_hostname=args.endpoint)
    except ssl.SSLError as exc:
        print("TLS       : HANDSHAKE FAILED - %s" % exc)
        print("            The modem would report this as CMQTTCONNECT err=32.")
        return 2

    print("TLS       : OK  %s / %s" % (sock.version(), sock.cipher()[0]))
    print("            (a completed handshake does NOT mean AWS accepted the")
    print("             client certificate -- that shows up at CONNECT below)")

    sock.settimeout(args.timeout)
    sock.sendall(build_connect(args.client_id))
    ptype, data = read_packet(sock)

    if ptype is None:
        print("CONNECT   : FAILED - broker closed the connection, no CONNACK")
        print(NOT_AUTHORIZED_HELP)
        return 2
    if ptype >> 4 != 2 or len(data) < 2:
        print("CONNECT   : FAILED - unexpected packet 0x%02X" % ptype)
        return 2

    code = data[1]
    if code != 0:
        print("CONNACK   : rc=%d (%s)" % (code, CONNACK_REASONS.get(code, "unknown")))
        return 2
    print("CONNECT   : OK - CONNACK accepted")

    rc = 0
    if args.subscribe:
        print("SUBSCRIBE : %s (QoS1)" % args.subscribe)
        sock.sendall(build_subscribe(args.subscribe))
        ptype, data = read_packet(sock)
        if ptype is None:
            print("SUBSCRIBE : FAILED - closed instead of SUBACK")
            print("            iot:Subscribe is not allowed on that topicfilter.")
            rc = 2
        elif ptype >> 4 == 9 and len(data) >= 3:
            granted = data[2]
            if granted == 0x80:
                print("SUBSCRIBE : FAILED - SUBACK refused (0x80)")
                rc = 2
            else:
                print("SUBSCRIBE : OK - SUBACK granted QoS %d" % granted)
        else:
            print("SUBSCRIBE : FAILED - unexpected packet 0x%02X" % ptype)
            rc = 2

    if args.publish:
        payload = json.dumps({
            "device_id": args.client_id,   # firmware: DEVICE_ID == AWS_THING_NAME
            "timestamp": datetime.now(timezone.utc).isoformat(),
            "test": True,
            "note": "iot_selftest.py - synthetic frame, not a real reading",
        }).encode()
        print("PUBLISH   : %s (%d bytes, QoS1)" % (args.publish, len(payload)))
        sock.sendall(build_publish(args.publish, payload))
        ptype, _ = read_packet(sock)
        if ptype is None:
            print("PUBLISH   : FAILED - closed instead of PUBACK")
            print("            iot:Publish is not allowed on that topic by the policy.")
            rc = 2
        elif ptype >> 4 == 4:
            print("PUBLISH   : OK - PUBACK received")
        else:
            print("PUBLISH   : FAILED - unexpected packet 0x%02X" % ptype)
            rc = 2

    try:
        sock.sendall(bytes([0xE0, 0x00]))   # DISCONNECT
        sock.close()
    except OSError:
        pass

    print("\nRESULT    : %s" % ("PASS" if rc == 0 else "FAIL"))
    return rc


if __name__ == "__main__":
    sys.exit(main())
