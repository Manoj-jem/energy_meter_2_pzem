#!/usr/bin/env bash
# create_iot_thing.sh - register a meter as an AWS IoT thing, the way energy-meter-3 and
# energy-meter-4 were: a new thing that carries the SAME device certificate as
# energy-meter-002, attached non-exclusively.
#
#   tools/create_iot_thing.sh energy-meter-17
#
# It changes the PRODUCTION AWS account (481665103941, ap-south-1). Run it deliberately, with
# credentials for that account. It is safe to re-run: an existing thing is left as it is.
#
# Why the certificate is shared: the IoT policy scopes each device's own topics with
# ${iot:Connection.Thing.ThingName}, which AWS fills in only when the MQTT client ID equals
# a thing attached to the certificate. Attaching one certificate to several things is
# allowed only as NON_EXCLUSIVE_THING. The client IDs must still differ, or AWS disconnects
# the devices from each other in a loop.
set -euo pipefail

THING="${1:?usage: $0 <ThingName>   e.g. energy-meter-17}"
REGION="ap-south-1"
ACCOUNT="481665103941"
CERT_DIR="$(cd "$(dirname "$0")/.." && pwd)/certs/energy-meter-002"

[ -f "$CERT_DIR/device.cert.pem" ] || { echo "missing $CERT_DIR/device.cert.pem"; exit 1; }

# An AWS certificate ID is the SHA-256 of the certificate's DER encoding.
CERT_ID="$(python - "$CERT_DIR/device.cert.pem" <<'PY'
import base64, hashlib, re, sys
pem = open(sys.argv[1]).read()
der = base64.b64decode(re.search(r"-----BEGIN CERTIFICATE-----(.*?)-----END CERTIFICATE-----", pem, re.S).group(1))
print(hashlib.sha256(der).hexdigest())
PY
)"
CERT_ARN="arn:aws:iot:${REGION}:${ACCOUNT}:cert/${CERT_ID}"

echo "account  : $(aws sts get-caller-identity --query Account --output text)"
echo "thing    : $THING"
echo "cert     : $CERT_ARN"
[ "$(aws sts get-caller-identity --query Account --output text)" = "$ACCOUNT" ] || {
  echo "refusing: these credentials are not for account $ACCOUNT"; exit 1; }

aws iot describe-certificate --region "$REGION" --certificate-id "$CERT_ID" \
    --query 'certificateDescription.status' --output text | grep -qx ACTIVE \
    || { echo "certificate $CERT_ID is not ACTIVE in $ACCOUNT"; exit 1; }

if aws iot describe-thing --region "$REGION" --thing-name "$THING" >/dev/null 2>&1; then
  echo "thing exists, leaving it"
else
  aws iot create-thing --region "$REGION" --thing-name "$THING"
fi

aws iot attach-thing-principal --region "$REGION" --thing-name "$THING" \
    --principal "$CERT_ARN" --thing-principal-type NON_EXCLUSIVE_THING

echo
echo "attached. Check:  aws iot list-thing-principals --region $REGION --thing-name $THING"
echo "Next: python tools/iot_selftest.py --thing $THING --publish $THING/selftest"
echo "Then add the backend row: device_id '$THING', payload_profile energywise_pzem_v1."
