# ================================================================
# gen_certs.py — generates include/certs_generated.h from the PEM
#                files under energy_meter_2_pzem/certs/<device>/
#
# WHY THIS EXISTS
#   The AWS IoT credentials used to be hand-pasted into a certs.h
#   with hand-typed length macros, in two separate PlatformIO
#   projects. They drifted: the uploader ended up writing one
#   certificate while the firmware expected another, and the only
#   symptom was "+CMQTTCONNECT: 0,32" (TLS handshake fail) — a error
#   code that says nothing about which of a dozen causes it was.
#
#   Now there is exactly one copy of the PEM bytes on disk, and both
#   the main firmware and the cert_uploader compile this same
#   generated header. Drift is not possible by construction.
#
# SAFETY CHECKS (all abort the build)
#   - client key's RSA modulus must equal the client certificate's,
#     i.e. the key really does belong to the cert. This is the exact
#     mistake that caused the outage: a *public* key had been placed
#     where the private key belongs.
#   - the key must not be a PUBLIC key.
#   - every file must be present and parse as PEM.
#
#   The modulus check is pure-Python DER parsing on purpose — no
#   `cryptography` package, no `openssl` binary. A build machine
#   without either must still be able to build the firmware.
#
# USAGE
#   PlatformIO:  extra_scripts = pre:scripts/gen_certs.py
#                custom_cert_device = energy-meter-002
#   Standalone:  python scripts/gen_certs.py [device-name]
# ================================================================

import base64
import hashlib
import inspect
import os
import sys

DEFAULT_DEVICE = "energy-meter-002"


def _script_path():
    """Absolute path of this file, under PlatformIO as well as standalone.

    PlatformIO runs extra scripts through SCons' SConscript(), which exec()s
    the source without defining __file__. The compiled code object still
    carries the real filename, so read it off the current frame instead.
    All the paths below are resolved relative to this file -- never the CWD --
    so that cert_uploader can run this same script from its own project dir.
    """
    try:
        return os.path.abspath(__file__)
    except NameError:
        return os.path.abspath(inspect.currentframe().f_code.co_filename)


SCRIPT_DIR = os.path.dirname(_script_path())
FIRMWARE_DIR = os.path.dirname(SCRIPT_DIR)                     # .../energy_meter_2_firmware
PZEM_DIR = os.path.dirname(FIRMWARE_DIR)                       # .../energy_meter_2_pzem
CERT_ROOT = os.path.join(PZEM_DIR, "certs")
OUT_HEADER = os.path.join(FIRMWARE_DIR, "include", "certs_generated.h")

CA_FILE = "AmazonRootCA1.pem"
# Written by tools/make_iot_server_cert.sh for the AWS IoT *custom domain*.
# When present it replaces Amazon Root CA 1: the custom domain presents our
# own short server certificate chain, because the A7670C modem cannot receive
# AWS's default 4,996-byte chain (see CERTIFICATES.md).
SERVER_CA_FILE = "server_ca.pem"
CERT_FILE = "device.cert.pem"
KEY_FILE = "device.private.key"
CONFIG_H = os.path.join(FIRMWARE_DIR, "include", "config.h")


class CertError(Exception):
    pass


# ── minimal DER reader ──────────────────────────────────────────
# Only what is needed to walk an X.509 certificate and a PKCS#1 /
# PKCS#8 RSA private key. Definite-length, low-tag-number form,
# which is all DER permits for these structures.

def der_read(buf, off):
    """Return (tag, content_bytes, offset_after_this_element)."""
    if off + 2 > len(buf):
        raise CertError("truncated DER element")
    tag = buf[off]
    length_byte = buf[off + 1]
    off += 2
    if length_byte < 0x80:
        length = length_byte
    else:
        n = length_byte & 0x7F
        if n == 0 or off + n > len(buf):
            raise CertError("bad DER length")
        length = int.from_bytes(buf[off:off + n], "big")
        off += n
    end = off + length
    if end > len(buf):
        raise CertError("DER element runs past end of buffer")
    return tag, buf[off:end], end


def der_children(content):
    """Split a constructed element's content into its child elements."""
    out = []
    off = 0
    while off < len(content):
        tag, body, off = der_read(content, off)
        out.append((tag, body))
    return out


def der_uint(body):
    """INTEGER content -> int, dropping the DER sign-padding byte."""
    return int.from_bytes(body, "big")


def pem_to_der(text, path):
    """Extract the first PEM block's DER bytes. Returns (der, label)."""
    lines = text.replace("\r\n", "\n").replace("\r", "\n").split("\n")
    label = None
    b64 = []
    for line in lines:
        s = line.strip()
        if s.startswith("-----BEGIN "):
            label = s[len("-----BEGIN "):].rstrip("-").strip()
            b64 = []
        elif s.startswith("-----END "):
            break
        elif label is not None and s:
            b64.append(s)
    if label is None or not b64:
        raise CertError("%s: no PEM block found" % path)
    try:
        return base64.b64decode("".join(b64)), label
    except Exception as exc:
        raise CertError("%s: base64 decode failed (%s)" % (path, exc))


# ── X.509 ───────────────────────────────────────────────────────

def parse_certificate(der, path):
    """Return dict with modulus, serial, not_before, not_after."""
    tag, cert_body, _ = der_read(der, 0)
    if tag != 0x30:
        raise CertError("%s: not a certificate (outer tag 0x%02X)" % (path, tag))

    tbs_tag, tbs, _ = der_read(cert_body, 0)
    if tbs_tag != 0x30:
        raise CertError("%s: malformed tbsCertificate" % path)

    fields = der_children(tbs)
    # version is [0] EXPLICIT and optional; everything after shifts by one.
    idx = 1 if fields and fields[0][0] == 0xA0 else 0

    serial = der_uint(fields[idx][1])
    validity = der_children(fields[idx + 3][1])
    spki = fields[idx + 5][1]

    # SubjectPublicKeyInfo ::= SEQUENCE { algorithm, subjectPublicKey BIT STRING }
    spki_fields = der_children(spki)
    bitstring = spki_fields[1][1]
    if not bitstring or bitstring[0] != 0x00:
        raise CertError("%s: unexpected unused-bit count in public key" % path)
    rsa_pub = der_children(der_read(bitstring[1:], 0)[1])
    modulus = der_uint(rsa_pub[0][1])

    return {
        "modulus": modulus,
        "serial": "%X" % serial,
        "not_before": fmt_time(validity[0]),
        "not_after": fmt_time(validity[1]),
    }


def fmt_time(element):
    """UTCTime (0x17) YYMMDDHHMMSSZ / GeneralizedTime (0x18) YYYYMMDD..."""
    tag, body = element
    s = body.decode("ascii", "replace")
    if tag == 0x17:
        yy = int(s[0:2])
        year = 2000 + yy if yy < 50 else 1900 + yy
        rest = s[2:]
    else:
        year = int(s[0:4])
        rest = s[4:]
    return "%04d-%s-%s %s:%s:%sZ" % (
        year, rest[0:2], rest[2:4], rest[4:6], rest[6:8], rest[8:10])


def parse_private_key(der, label, path):
    """Return the RSA modulus from a PKCS#1 or PKCS#8 private key."""
    if "PUBLIC KEY" in label:
        raise CertError(
            "%s is a PUBLIC key (%s).\n"
            "       The client key must be the PRIVATE key AWS gave you at\n"
            "       certificate-creation time (*-private.pem.key). A public key\n"
            "       cannot sign the TLS CertificateVerify message, which fails\n"
            "       the handshake as +CMQTTCONNECT err=32." % (path, label))

    tag, body, _ = der_read(der, 0)
    if tag != 0x30:
        raise CertError("%s: not a private key (outer tag 0x%02X)" % (path, tag))

    fields = der_children(body)
    # PKCS#1 "RSA PRIVATE KEY": SEQUENCE { version INTEGER, modulus INTEGER, ... }
    if len(fields) >= 3 and fields[0][0] == 0x02 and fields[1][0] == 0x02:
        return der_uint(fields[1][1])

    # PKCS#8 "PRIVATE KEY": SEQUENCE { version, algorithm, privateKey OCTET STRING }
    for tag_i, body_i in fields:
        if tag_i == 0x04:
            inner = der_children(der_read(body_i, 0)[1])
            if len(inner) >= 2 and inner[1][0] == 0x02:
                return der_uint(inner[1][1])

    raise CertError("%s: could not locate RSA modulus (label %r)" % (path, label))


# ── header emission ─────────────────────────────────────────────

def c_string_literal(text):
    """Emit PEM text as one C string literal per line, LF-normalised.

    A stray CR would be counted by sizeof() and sent to AT+CCERTDOWN,
    corrupting the file stored on the modem, so normalise here rather
    than trusting whatever line endings the PEM arrived with.
    """
    body = text.replace("\r\n", "\n").replace("\r", "\n")
    if not body.endswith("\n"):
        body += "\n"
    out = []
    for line in body.split("\n")[:-1]:
        out.append('    "%s\\n"' % line)
    return "\n".join(out)


HEADER_TEMPLATE = '''#pragma once
// ================================================================
// certs_generated.h — GENERATED FILE, DO NOT EDIT
//
// Produced by scripts/gen_certs.py from:
//     certs/{device}/{ca_file}
//     certs/{device}/{cert_file}
//     certs/{device}/{key_file}
//
// Edit those PEM files and rebuild; never edit this header. Both the
// main firmware and cert_uploader compile this same file, so the
// bytes written to the modem and the bytes the firmware expects can
// never disagree.
// ================================================================

#define CERT_DEVICE_NAME        "{device}"

// SHA-256 of the client certificate's DER. The firmware stores this in
// NVS after provisioning; AT+CCERTLIST reports filenames only, never
// content, so this fingerprint is the only way to tell that a modem is
// holding a *stale* certificate under the right filename.
#define CLIENT_CERT_SHA256      "{fingerprint}"
// SHA-256 over all three files exactly as uploaded (CA, cert, key). This is
// what the firmware records in NVS, so replacing ANY of them -- e.g. moving
// to the custom-domain server CA -- re-provisions the modem.
#define CERT_BUNDLE_SHA256      "{bundle}"
#define CERT_CA_SOURCE          "{ca_file}"
#define CLIENT_CERT_SERIAL      "{serial}"
#define CLIENT_CERT_NOT_BEFORE  "{not_before}"
#define CLIENT_CERT_NOT_AFTER   "{not_after}"

static const char ROOT_CA_PEM[] =
{ca_body};

static const char CLIENT_CERT_PEM[] =
{cert_body};

static const char CLIENT_KEY_PEM[] =
{key_body};

// Lengths are derived, never hand-typed — a hand-maintained constant
// that disagrees with the array truncates the file on the modem.
#define ROOT_CA_PEM_LEN     (sizeof(ROOT_CA_PEM) - 1)
#define CLIENT_CERT_PEM_LEN (sizeof(CLIENT_CERT_PEM) - 1)
#define CLIENT_KEY_PEM_LEN  (sizeof(CLIENT_KEY_PEM) - 1)
'''


def read_text(path):
    if not os.path.isfile(path):
        raise CertError("missing file: %s" % path)
    with open(path, "r", encoding="utf-8") as handle:
        return handle.read()


def generate(device):
    cert_dir = os.path.join(CERT_ROOT, device)
    if not os.path.isdir(cert_dir):
        raise CertError(
            "certificate directory not found: %s\n"
            "       Expected the three AWS IoT files for device %r there." % (cert_dir, device))

    custom_ca = os.path.isfile(os.path.join(cert_dir, SERVER_CA_FILE))
    ca_file = SERVER_CA_FILE if custom_ca else CA_FILE
    check_endpoint_matches_ca(custom_ca)

    ca_text = read_text(os.path.join(cert_dir, ca_file))
    cert_text = read_text(os.path.join(cert_dir, CERT_FILE))
    key_text = read_text(os.path.join(cert_dir, KEY_FILE))

    ca_der, ca_label = pem_to_der(ca_text, ca_file)
    cert_der, cert_label = pem_to_der(cert_text, CERT_FILE)
    key_der, key_label = pem_to_der(key_text, KEY_FILE)

    if "CERTIFICATE" not in ca_label:
        raise CertError("%s is %r, expected a CERTIFICATE" % (ca_file, ca_label))
    if "CERTIFICATE" not in cert_label:
        raise CertError("%s is %r, expected a CERTIFICATE" % (CERT_FILE, cert_label))

    info = parse_certificate(cert_der, CERT_FILE)
    key_modulus = parse_private_key(key_der, key_label, KEY_FILE)

    if key_modulus != info["modulus"]:
        raise CertError(
            "%s does not match %s - the private key belongs to a different\n"
            "       certificate. AWS IoT will fail the TLS handshake with\n"
            "       +CMQTTCONNECT err=32. Re-download both files from the same\n"
            "       certificate in the AWS IoT console, or create a new keypair.\n"
            "         cert modulus: %s\n"
            "         key  modulus: %s"
            % (KEY_FILE, CERT_FILE,
               modulus_hint(info["modulus"]), modulus_hint(key_modulus)))

    fingerprint = hashlib.sha256(cert_der).hexdigest()
    bundle = hashlib.sha256("\0".join(
        normalise(t) for t in (ca_text, cert_text, key_text)).encode("utf-8")).hexdigest()

    header = HEADER_TEMPLATE.format(
        device=device,
        ca_file=ca_file, cert_file=CERT_FILE, key_file=KEY_FILE,
        fingerprint=fingerprint,
        bundle=bundle,
        serial=info["serial"],
        not_before=info["not_before"],
        not_after=info["not_after"],
        ca_body=c_string_literal(ca_text),
        cert_body=c_string_literal(cert_text),
        key_body=c_string_literal(key_text),
    )

    # Only rewrite when the content actually changed, so an unchanged
    # certificate does not force a full rebuild on every invocation.
    existing = None
    if os.path.isfile(OUT_HEADER):
        with open(OUT_HEADER, "r", encoding="utf-8") as handle:
            existing = handle.read()
    if existing != header:
        os.makedirs(os.path.dirname(OUT_HEADER), exist_ok=True)
        with open(OUT_HEADER, "w", encoding="utf-8", newline="\n") as handle:
            handle.write(header)
        action = "written"
    else:
        action = "up to date"

    print("[gen_certs] %s (%s)" % (os.path.relpath(OUT_HEADER, FIRMWARE_DIR), action))
    print("[gen_certs]   device      : %s" % device)
    print("[gen_certs]   serial      : %s" % info["serial"])
    print("[gen_certs]   valid       : %s -> %s" % (info["not_before"], info["not_after"]))
    print("[gen_certs]   sha256      : %s" % fingerprint)
    print("[gen_certs]   key/cert    : modulus match OK")
    print("[gen_certs]   server CA   : %s%s" % (
        ca_file, "  (custom domain)" if custom_ca else "  (AWS default endpoint)"))
    return fingerprint


def normalise(text):
    """The exact bytes c_string_literal() emits, i.e. what reaches the modem."""
    body = text.replace("\r\n", "\n").replace("\r", "\n")
    return body if body.endswith("\n") else body + "\n"


def check_endpoint_matches_ca(custom_ca):
    """Fail the build when the trusted CA cannot possibly verify the endpoint.

    AWS's default endpoint (*-ats.iot.<region>.amazonaws.com) chains to Amazon
    Root CA 1; the custom domain presents a certificate from our own CA. Mixing
    them builds fine and then fails every connection with the same opaque
    +CMQTTCONNECT err=32 this project spent a week diagnosing.
    """
    import re
    with open(CONFIG_H, "r", encoding="utf-8") as handle:
        m = re.search(r'#define\s+MQTT_BROKER_HOST\s+"([^"]+)"', handle.read())
    if not m:
        return
    host = m.group(1)
    aws_default = host.endswith(".amazonaws.com")
    if custom_ca and aws_default:
        raise CertError(
            "%s is present, so the modem will trust only our own server CA,\n"
            "       but MQTT_BROKER_HOST is AWS's default endpoint (%s), whose\n"
            "       certificate comes from Amazon. Set MQTT_BROKER_HOST in config.h to\n"
            "       the custom domain, or remove %s to go back to Amazon Root CA 1."
            % (SERVER_CA_FILE, host, SERVER_CA_FILE))
    if not custom_ca and not aws_default:
        raise CertError(
            "MQTT_BROKER_HOST is %s (a custom domain), but certs/<device>/%s is\n"
            "       missing, so the modem would trust Amazon Root CA 1 and reject the\n"
            "       custom domain's certificate. Run tools/make_iot_server_cert.sh."
            % (host, SERVER_CA_FILE))


def modulus_hint(modulus):
    """Head and tail of the modulus, as openssl prints it.

    Deliberately NOT `openssl -modulus | openssl md5`: that digest covers
    openssl's own trailing line ending, so a Windows build (CRLF) and a
    Linux build (LF) give different answers for the same key. The raw hex
    is comparable everywhere -- it is literally what
    `openssl x509 -noout -modulus` puts on screen.
    """
    raw = modulus.to_bytes((modulus.bit_length() + 7) // 8, "big")
    hex_str = raw.hex().upper()
    return "%s...%s (%d bit)" % (hex_str[:24], hex_str[-16:], modulus.bit_length())


def main(device):
    try:
        generate(device)
    except CertError as exc:
        sys.stderr.write("\n*** gen_certs FAILED ***\n       %s\n\n" % exc)
        sys.exit(1)


# ── entry points ────────────────────────────────────────────────

try:
    Import("env")  # noqa: F821  (injected by PlatformIO/SCons)
except NameError:
    if __name__ == "__main__":
        main(sys.argv[1] if len(sys.argv) > 1 else DEFAULT_DEVICE)
else:
    main(env.GetProjectOption("custom_cert_device", DEFAULT_DEVICE))  # noqa: F821
