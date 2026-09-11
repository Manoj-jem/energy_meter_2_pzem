#!/usr/bin/env bash
# ================================================================
# setup_iot_custom_domain.sh
#
# Puts an AWS IoT *custom domain* in front of the account's MQTT broker so
# that it presents OUR small server certificate (~1 KB, one certificate)
# instead of AWS's default 4-certificate, 4,996-byte chain, which the SIMCom
# A7670C (firmware V11.0.01) cannot receive. Devices keep their own X.509
# client certificates; IoT policies are unchanged.
#
# Stages (run in order):
#   certs   create CA + server cert (tools/make_iot_server_cert.sh), import the
#           server cert into ACM, request the ACM public certificate AWS uses
#           to validate domain ownership, and print the DNS records to add.
#   domain  once the validation certificate is ISSUED, create the IoT domain
#           configuration.
#   verify  show which chain AWS serves for the domain -- works before the
#           CNAME exists, because AWS selects the domain configuration by SNI.
#
# Usage:  tools/setup_iot_custom_domain.sh <fqdn> certs|domain|verify [device]
# Needs:  AWS CLI credentials for the account that owns the IoT endpoint.
# State:  ARNs are kept in <secrets_dir>/aws_state.env (outside the repo).
# ================================================================
set -euo pipefail
export MSYS_NO_PATHCONV=1
export MSYS2_ARG_CONV_EXCL="*"

FQDN=${1:?usage: $0 <fqdn> certs|domain|verify [device]}
STAGE=${2:?usage: $0 <fqdn> certs|domain|verify [device]}
DEVICE=${3:-energy-meter-002}
REGION=${AWS_DEFAULT_REGION:-ap-south-1}
SECRETS=${SECRETS_DIR:-$HOME/.energy-meter-iot-ca}
STATE="$SECRETS/aws_state.env"
DOMAIN_CONFIG_NAME="energywise-short-chain"
TOOLS="$(cd "$(dirname "$0")" && pwd)"

aws_() { aws --region "$REGION" "$@"; }
# Path conversion is off (above), so hand native Windows tools a Windows path.
winpath() { if command -v cygpath >/dev/null 2>&1; then cygpath -m "$1"; else printf '%s' "$1"; fi; }
save() { mkdir -p "$SECRETS"; grep -v "^$1=" "$STATE" 2>/dev/null > "$STATE.tmp" || true
         echo "$1=$2" >> "$STATE.tmp"; mv "$STATE.tmp" "$STATE"; }
[ -f "$STATE" ] && . "$STATE"

# Meter 002's certificates were issued by account 481665103941; a domain
# configuration created anywhere else can never authenticate them. This
# machine's default AWS profile may belong to the other account (the deploy
# .bat files configure 571751567031), so refuse to run against the wrong one.
EXPECT_ACCOUNT=${EXPECT_ACCOUNT:-481665103941}
ACCOUNT=$(aws_ sts get-caller-identity --query Account --output text)
echo "[aws] account $ACCOUNT, region $REGION"
if [ "$ACCOUNT" != "$EXPECT_ACCOUNT" ]; then
    echo "credentials are for account $ACCOUNT, expected $EXPECT_ACCOUNT -- aborting" >&2
    exit 1
fi

ATS=$(aws_ iot describe-endpoint --endpoint-type iot:Data-ATS --query endpointAddress --output text)

case "$STAGE" in
certs)
    if [ ! -f "$SECRETS/server.pem" ]; then
        SECRETS_DIR="$SECRETS" bash "$TOOLS/make_iot_server_cert.sh" "$FQDN" "$DEVICE" "$SECRETS"
    else
        echo "[certs] reusing $SECRETS/server.pem"
    fi

    if [ -z "${SERVER_CERT_ARN:-}" ]; then
        SERVER_CERT_ARN=$(aws_ acm import-certificate \
            --certificate "fileb://$(winpath "$SECRETS/server.pem")" \
            --private-key "fileb://$(winpath "$SECRETS/server.key")" \
            --tags Key=purpose,Value=iot-custom-domain Key=domain,Value="$FQDN" \
            --query CertificateArn --output text)
        save SERVER_CERT_ARN "$SERVER_CERT_ARN"
    fi
    echo "[acm] server certificate : $SERVER_CERT_ARN"

    if [ -z "${VALIDATION_CERT_ARN:-}" ]; then
        VALIDATION_CERT_ARN=$(aws_ acm request-certificate --domain-name "$FQDN" \
            --validation-method DNS --tags Key=purpose,Value=iot-custom-domain-validation \
            --query CertificateArn --output text)
        save VALIDATION_CERT_ARN "$VALIDATION_CERT_ARN"
        sleep 8   # ACM needs a moment before it publishes the validation record
    fi
    echo "[acm] validation cert    : $VALIDATION_CERT_ARN"

    read -r VNAME VVALUE < <(aws_ acm describe-certificate --certificate-arn "$VALIDATION_CERT_ARN" \
        --query 'Certificate.DomainValidationOptions[0].ResourceRecord.[Name,Value]' --output text | tr -d '\r')
    echo
    echo "Add these two DNS records at your DNS provider:"
    echo "  1) ACM ownership check   CNAME  ${VNAME%.}  ->  ${VVALUE%.}"
    echo "  2) Device endpoint       CNAME  $FQDN  ->  $ATS"
    echo
    echo "Then run:  $0 $FQDN domain"
    ;;

domain)
    : "${SERVER_CERT_ARN:?run the certs stage first}" "${VALIDATION_CERT_ARN:?run the certs stage first}"
    status=$(aws_ acm describe-certificate --certificate-arn "$VALIDATION_CERT_ARN" \
        --query Certificate.Status --output text)
    echo "[acm] validation certificate status: $status"
    [ "$status" = "ISSUED" ] || { echo "Not ISSUED yet -- check DNS record 1, then retry."; exit 2; }

    if aws_ iot describe-domain-configuration --domain-configuration-name "$DOMAIN_CONFIG_NAME" >/dev/null 2>&1; then
        echo "[iot] domain configuration $DOMAIN_CONFIG_NAME already exists"
    else
        aws_ iot create-domain-configuration --domain-configuration-name "$DOMAIN_CONFIG_NAME" \
            --service-type DATA --domain-name "$FQDN" \
            --server-certificate-arns "$SERVER_CERT_ARN" \
            --validation-certificate-arn "$VALIDATION_CERT_ARN" \
            --tags Key=purpose,Value=a7670-short-chain >/dev/null
        echo "[iot] created domain configuration $DOMAIN_CONFIG_NAME for $FQDN"
    fi
    aws_ iot describe-domain-configuration --domain-configuration-name "$DOMAIN_CONFIG_NAME" \
        --query '[domainName,domainConfigurationStatus,serverCertificates[0].serverCertificateStatus,serverCertificates[0].serverCertificateStatusDetail]' \
        --output text
    echo "AWS can take up to 60 minutes to start serving it. Then run:  $0 $FQDN verify"
    ;;

verify)
    echo "== chain AWS serves for SNI=$FQDN (via $ATS) =="
    out=$(openssl s_client -connect "$ATS:8883" -servername "$FQDN" -tls1_2 -msg -showcerts </dev/null 2>/dev/null || true)
    echo "$out" | awk '/^ *[0-9] s:/{print "   " $0} /^ *i:/{print "   " $0}'
    echo "$out" | grep -E '<<< TLS 1.2, Handshake.*Certificate$' | sed 's/^/   /'
    if echo "$out" | grep -q "CN = $FQDN\|CN=$FQDN"; then
        echo "   OK: custom certificate is being served."
    else
        echo "   Not yet: still AWS's default certificate (can take up to 60 min)."
    fi
    if getent hosts "$FQDN" >/dev/null 2>&1 || nslookup "$FQDN" >/dev/null 2>&1; then
        echo "   DNS: $FQDN resolves."
    else
        echo "   DNS: $FQDN does not resolve yet (DNS record 2)."
    fi
    ;;

*) echo "unknown stage: $STAGE (certs|domain|verify)" >&2; exit 1 ;;
esac
