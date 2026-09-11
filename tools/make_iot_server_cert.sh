#!/usr/bin/env bash
# ================================================================
# make_iot_server_cert.sh -- thin wrapper, kept so existing docs and
# tools/setup_iot_custom_domain.sh keep working. The certificates are made by
# make_iot_server_cert.py (Python 'cryptography'), because the A7670C modem
# needs exact control of their dates -- see that file's header.
#
# Usage:  tools/make_iot_server_cert.sh <fqdn> [device] [secrets_dir] [--new-ca]
# ================================================================
set -euo pipefail

here="$(cd "$(dirname "$0")" && pwd)"
fqdn=${1:?usage: $0 <fqdn> [device] [secrets_dir] [--new-ca]}
device=${2:-energy-meter-002}
secrets=${3:-}

# Python on Windows is a native program: give it Windows paths, whatever
# MSYS path conversion is doing in the calling shell.
if command -v cygpath >/dev/null 2>&1; then
    here="$(cygpath -m "$here")"
    [ -n "$secrets" ] && secrets="$(cygpath -m "$secrets")"
fi

args=("$fqdn" --device "$device")
[ -n "$secrets" ] && args+=(--secrets "$secrets")
[ "${4:-}" = "--new-ca" ] && args+=(--new-ca)

exec python "$here/make_iot_server_cert.py" "${args[@]}"
