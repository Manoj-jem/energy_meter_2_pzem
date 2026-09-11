#!/usr/bin/env python3
"""
make_iot_server_cert.py -- private CA + AWS IoT custom-domain server certificate.

Why this exists: the SIMCom A7670C (firmware V11.0.01) cannot receive AWS
IoT's default server certificate chain (4 certificates, 4,996 bytes in one
TLS record) and fails every handshake with +CMQTTCONNECT err 32. The AWS IoT
custom domain (iot.energywise.tech) presents the ~0.9 KB certificate this
script issues instead. See CERTIFICATES.md.

The certificates are shaped for that modem, on purpose:
  * Every date is before 2038, so it is encoded as UTCTime. Amazon Root CA 1
    (notAfter 2038-01-17, UTCTime) loads fine on the modem. A first CA that
    expired in 2051 (encoded as GeneralizedTime, as X.509 requires after
    2049) made the modem fail with +CMQTTCONNECT: 0,34 "open session
    failed" before the handshake started.
  * notBefore is backdated one day, so a modem clock that lags a little
    cannot reject a freshly issued certificate.
  * The CA's extensions mirror Amazon Root CA 1: basicConstraints CA:TRUE
    (critical, no pathLen), keyUsage digitalSignature|keyCertSign|cRLSign
    (critical), subjectKeyIdentifier.

Files (all in --secrets, default ~/.energy-meter-iot-ca, never in the repo):
  ca.key / ca.pem         the CA. Anyone holding ca.key can impersonate the
                          broker to every meter: back it up privately.
  server.key / server.pem imported into AWS Certificate Manager.
  certs/<device>/server_ca.pem  gets a copy of ca.pem (public; commit it).

Usage:
  python tools/make_iot_server_cert.py <fqdn> [--device energy-meter-002]
                                       [--secrets DIR] [--new-ca]
By default the existing CA is reused (renewal: devices keep trusting it,
no firmware change). --new-ca moves the current CA and server files into
<secrets>/replaced-<timestamp>/ and creates a fresh CA; devices must then be
rebuilt and flashed with the new server_ca.pem.
"""

import argparse
import datetime as dt
import os
import shutil
import sys
from pathlib import Path

from cryptography import x509
from cryptography.hazmat.primitives import hashes, serialization
from cryptography.hazmat.primitives.asymmetric import padding, rsa
from cryptography.x509.oid import ExtendedKeyUsageOID, NameOID

UTC = dt.timezone.utc
# Before 2038-01-19 (32-bit time_t) and before 2050 (UTCTime), like Amazon Root CA 1.
CA_NOT_AFTER = dt.datetime(2037, 12, 31, 23, 59, 59, tzinfo=UTC)
LEAF_DAYS = 5 * 365
BACKDATE = dt.timedelta(days=1)
KEY_USAGE_NONE = dict(digital_signature=False, content_commitment=False, key_encipherment=False,
                      data_encipherment=False, key_agreement=False, key_cert_sign=False,
                      crl_sign=False, encipher_only=False, decipher_only=False)


def new_key():
    return rsa.generate_private_key(public_exponent=65537, key_size=2048)


def write_key(path, key):
    path.write_bytes(key.private_bytes(serialization.Encoding.PEM,
                                       serialization.PrivateFormat.TraditionalOpenSSL,
                                       serialization.NoEncryption()))
    try:
        os.chmod(path, 0o600)
    except OSError:
        pass


def write_cert(path, cert):
    path.write_bytes(cert.public_bytes(serialization.Encoding.PEM))   # LF line endings


def make_ca(key, now):
    name = x509.Name([x509.NameAttribute(NameOID.ORGANIZATION_NAME, "Energywise"),
                      x509.NameAttribute(NameOID.COMMON_NAME, "Energywise IoT Server CA")])
    return (x509.CertificateBuilder()
            .subject_name(name).issuer_name(name)
            .public_key(key.public_key())
            .serial_number(x509.random_serial_number())
            .not_valid_before(now - BACKDATE).not_valid_after(CA_NOT_AFTER)
            .add_extension(x509.BasicConstraints(ca=True, path_length=None), critical=True)
            .add_extension(x509.KeyUsage(**{**KEY_USAGE_NONE, "digital_signature": True,
                                            "key_cert_sign": True, "crl_sign": True}), critical=True)
            .add_extension(x509.SubjectKeyIdentifier.from_public_key(key.public_key()), critical=False)
            .sign(key, hashes.SHA256()))


def make_leaf(fqdn, key, ca_cert, ca_key, now):
    not_after = min(now + dt.timedelta(days=LEAF_DAYS), ca_cert.not_valid_after_utc)
    return (x509.CertificateBuilder()
            .subject_name(x509.Name([x509.NameAttribute(NameOID.COMMON_NAME, fqdn)]))
            .issuer_name(ca_cert.subject)
            .public_key(key.public_key())
            .serial_number(x509.random_serial_number())
            .not_valid_before(now - BACKDATE).not_valid_after(not_after)
            .add_extension(x509.BasicConstraints(ca=False, path_length=None), critical=True)
            .add_extension(x509.KeyUsage(**{**KEY_USAGE_NONE, "digital_signature": True,
                                            "key_encipherment": True}), critical=True)
            .add_extension(x509.ExtendedKeyUsage([ExtendedKeyUsageOID.SERVER_AUTH]), critical=False)
            .add_extension(x509.SubjectAlternativeName([x509.DNSName(fqdn)]), critical=False)
            .add_extension(x509.SubjectKeyIdentifier.from_public_key(key.public_key()), critical=False)
            .add_extension(x509.AuthorityKeyIdentifier.from_issuer_public_key(ca_key.public_key()),
                           critical=False)
            .sign(ca_key, hashes.SHA256()))


def modem_safe(cert):
    """True if both dates are before 2038 (UTCTime, 32-bit time_t safe)."""
    return cert.not_valid_after_utc < dt.datetime(2038, 1, 19, tzinfo=UTC)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("fqdn")
    ap.add_argument("--device", default="energy-meter-002")
    ap.add_argument("--secrets", default=str(Path.home() / ".energy-meter-iot-ca"))
    ap.add_argument("--new-ca", action="store_true", help="replace the CA (devices must be re-flashed)")
    args = ap.parse_args()

    repo = Path(__file__).resolve().parent.parent
    device_dir = repo / "certs" / args.device
    if not device_dir.is_dir():
        sys.exit("no such device directory: %s" % device_dir)
    secrets = Path(args.secrets).expanduser()
    secrets.mkdir(parents=True, exist_ok=True)
    now = dt.datetime.now(UTC).replace(microsecond=0)

    ca_key_p, ca_p = secrets / "ca.key", secrets / "ca.pem"
    srv_key_p, srv_p = secrets / "server.key", secrets / "server.pem"

    if args.new_ca and ca_key_p.exists():
        backup = secrets / ("replaced-" + now.strftime("%Y%m%dT%H%M%SZ"))
        backup.mkdir()
        for p in (ca_key_p, ca_p, srv_key_p, srv_p, secrets / "ca.srl"):
            if p.exists():
                shutil.move(str(p), str(backup / p.name))
        print("[ca] previous CA moved to %s" % backup)

    if ca_key_p.exists():
        ca_key = serialization.load_pem_private_key(ca_key_p.read_bytes(), password=None)
        ca_cert = x509.load_pem_x509_certificate(ca_p.read_bytes())
        print("[ca] reusing CA in %s (valid to %s)" % (secrets, ca_cert.not_valid_after_utc.date()))
        if not modem_safe(ca_cert):
            print("[ca] WARNING: this CA expires after 2038 -- the A7670C rejects it "
                  "(err 34). Re-run with --new-ca.", file=sys.stderr)
    else:
        ca_key = new_key()
        ca_cert = make_ca(ca_key, now)
        write_key(ca_key_p, ca_key)
        write_cert(ca_p, ca_cert)
        print("[ca] created CA in %s (valid %s -> %s)" % (
            secrets, ca_cert.not_valid_before_utc.date(), ca_cert.not_valid_after_utc.date()))

    key = new_key()
    leaf = make_leaf(args.fqdn, key, ca_cert, ca_key, now)
    # Sanity: the leaf really is signed by this CA.
    ca_cert.public_key().verify(leaf.signature, leaf.tbs_certificate_bytes,
                                padding.PKCS1v15(), leaf.signature_hash_algorithm)
    write_key(srv_key_p, key)
    write_cert(srv_p, leaf)
    shutil.copyfile(ca_p, device_dir / "server_ca.pem")

    der = leaf.public_bytes(serialization.Encoding.DER)
    print("[server] issued for %s (valid %s -> %s)" % (
        args.fqdn, leaf.not_valid_before_utc.date(), leaf.not_valid_after_utc.date()))
    print()
    print("  server certificate : %s  (%d bytes DER; TLS Certificate message %d bytes)"
          % (srv_p, len(der), len(der) + 6))
    print("  server private key : %s  (import into ACM; never commit)" % srv_key_p)
    print("  CA private key     : %s  (never commit; back it up)" % ca_key_p)
    print("  device CA file     : %s  (%d bytes)" % (device_dir / "server_ca.pem", ca_p.stat().st_size))
    print()
    print("Import into ACM (ap-south-1) -- no chain, the device already has the CA:")
    print("  aws acm import-certificate --region ap-south-1 \\")
    print("    --certificate fileb://%s --private-key fileb://%s" % (srv_p.as_posix(), srv_key_p.as_posix()))
    print("To renew in place, add: --certificate-arn <existing ARN>")


if __name__ == "__main__":
    main()
