#!/usr/bin/env bash
# Apply a persistent static IPv4 on the Ouster (rover 169.254.55.180/24).
# Run once while the sensor is reachable at its *current* address (link-local is fine).
#
# Usage:
#   ./ouster_set_rover_ipv4.sh 169.254.78.47
#   OUSTER_CURRENT_IP=169.254.78.47 ./ouster_set_rover_ipv4.sh
#
# Override target (must be free on the rover /24; avoid .1 and other hosts):
#   OUSTER_ROVER_IPV4_CIDR=169.254.55.180/24 ./ouster_set_rover_ipv4.sh 169.254.78.47
#
# Docs: PUT .../api/v1/system/network/ipv4/override (JSON string body "a.b.c.d/nn")

set -euo pipefail

CURRENT="${1:-${OUSTER_CURRENT_IP:-}}"
TARGET_CIDR="${OUSTER_ROVER_IPV4_CIDR:-169.254.55.180/24}"

if [[ -z "${CURRENT}" ]]; then
  echo "Usage: $0 <sensor-current-ipv4-or-hostname>" >&2
  echo "Example: $0 169.254.78.47" >&2
  exit 1
fi

echo "PUT ipv4/override on http://${CURRENT}/ → static ${TARGET_CIDR}"
curl -sS -i -X PUT "http://${CURRENT}/api/v1/system/network/ipv4/override" \
  -H 'Content-Type: application/json' \
  --data-raw "\"${TARGET_CIDR}\""
echo
echo "After ~1s the sensor uses ${TARGET_CIDR%/*}. This host must be on 169.254.55.x/24 to talk to it."
