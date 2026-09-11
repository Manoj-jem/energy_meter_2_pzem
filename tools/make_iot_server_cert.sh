#!/usr/bin/env bash
# ================================================================
# make_iot_server_cert.sh
#
# Creates the server certificate for the AWS IoT *custom domain* that
# meters connect to, plus the small private CA that signs it.
#
# Why this exists: the SIMCom A7670C (firmware V11.0.01) cannot receive
# AWS IoT's default server certificate chain -- 4 certificates, 4,996 bytes
# in one TLS record -- and fails every handshake with CMQTTCONNECT err 32.
# It handles chains up to at least 4.4 KB. A custom domain lets us choose
# the chain AWS presents: here it is a single ~1 KB leaf certificate.
#
#   CA        -> stays OUTSIDE the repo (default ~/.energy-meter-iot-ca).
#                Anyone with ca.key can impersonate the broker to meters.
#   server.*  -> imported into AWS Certificate Manager (ap-south-1); the
#                key is also kept outside the repo.
#   ca.pem    -> the only file devices need; copied into
#                certs/<device>/server_ca.pem (public, safe to commit).
#
# The CA is created once (25 years). Re-running the script re-issues only
# the server certificate (5 years); re-import it into the SAME ACM ARN and
# AWS IoT picks it up. Devices trust the CA, so no firmware change.
#
# Usage:  tools/make_iot_server_cert.sh <fqdn> [device] [secrets_dir]
#   e.g.  tools/make_iot_server_cert.sh iot.example.com energy-meter-002
# ================================================================
set -euo pipefail

# Git Bash on Windows rewrites any argument starting with "/" into a Windows
# path, which turns -subj "/O=..." into "C:/Program Files/Git/O=...". These
# are no-ops on Linux/macOS.
export MSYS_NO_PATHCONV=1
export MSYS2_ARG_CONV_EXCL="*"

FQDN=${1:?usage: $0 <fqdn> [device] [secrets_dir]}
DEVICE=${2:-energy-meter-002}
SECRETS=${3:-$HOME/.energy-meter-iot-ca}

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
DEVICE_DIR="$REPO_ROOT/certs/$DEVICE"
[ -d "$DEVICE_DIR" ] || { echo "no such device directory: $DEVICE_DIR" >&2; exit 1; }

mkdir -p "$SECRETS"
chmod 700 "$SECRETS" 2>/dev/null || true
cd "$SECRETS"

# Every extension comes from an explicit file passed to `openssl x509 -req`,
# which (unlike `req -x509`) adds nothing from the machine's openssl.cnf --
# so the certificates are identical whichever openssl builds them.
if [ ! -f ca.key ]; then
    echo "[ca] creating private CA in $SECRETS"
    openssl genrsa -out ca.key 2048 2>/dev/null
    openssl req -new -key ca.key -subj "/O=Energywise/CN=Energywise IoT Server CA" -out ca.csr
    cat > ca.ext <<EOF
basicConstraints=critical,CA:TRUE,pathlen:0
keyUsage=critical,keyCertSign,cRLSign
subjectKeyIdentifier=hash
EOF
    openssl x509 -req -in ca.csr -signkey ca.key -days 9125 -sha256 -extfile ca.ext -out ca.pem 2>/dev/null
else
    echo "[ca] reusing existing CA in $SECRETS"
fi

echo "[server] issuing certificate for $FQDN"
openssl genrsa -out server.key 2048 2>/dev/null
openssl req -new -key server.key -subj "/CN=$FQDN" -out server.csr
cat > server.ext <<EOF
basicConstraints=critical,CA:FALSE
keyUsage=critical,digitalSignature,keyEncipherment
extendedKeyUsage=serverAuth
subjectAltName=DNS:$FQDN
subjectKeyIdentifier=hash
authorityKeyIdentifier=keyid
EOF
openssl x509 -req -in server.csr -CA ca.pem -CAkey ca.key -CAcreateserial \
    -days 1825 -sha256 -extfile server.ext -out server.pem 2>/dev/null

# Sanity: the server cert must chain to the CA and name the FQDN.
openssl verify -CAfile ca.pem server.pem >/dev/null
openssl x509 -in server.pem -noout -ext subjectAltName | grep -q "DNS:$FQDN"

cp ca.pem "$DEVICE_DIR/server_ca.pem"

der_len=$(openssl x509 -in server.pem -outform DER | wc -c)
echo
echo "  server certificate : $SECRETS/server.pem  (${der_len} bytes DER -- the whole chain AWS will send)"
echo "  server private key : $SECRETS/server.key  (import into ACM; never commit)"
echo "  CA private key     : $SECRETS/ca.key      (never commit; back it up)"
echo "  device CA file     : $DEVICE_DIR/server_ca.pem"
echo
echo "Import into ACM (ap-south-1), no chain -- the device already has the CA:"
echo "  aws acm import-certificate --region ap-south-1 \\"
echo "    --certificate fileb://$SECRETS/server.pem --private-key fileb://$SECRETS/server.key"
echo "To renew later, add: --certificate-arn <existing ARN>"
