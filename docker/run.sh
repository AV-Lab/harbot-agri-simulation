#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")/.."

if [[ -z "${DISPLAY:-}" ]]; then
    echo "DISPLAY is not set. Run this script in a terminal on your desktop." >&2
    exit 1
fi

# Copy the desktop's display-access key for the container's read-only mount.
HARBOT_XAUTHORITY=$(mktemp /tmp/harbot-xauth.XXXXXX)
export HARBOT_XAUTHORITY
trap 'rm -f -- "$HARBOT_XAUTHORITY"' EXIT

xauth nlist "$DISPLAY" | sed 's/^..../ffff/' | xauth -f "$HARBOT_XAUTHORITY" nmerge -
if [[ ! -s "$HARBOT_XAUTHORITY" ]]; then
    echo "No Xauthority key found for DISPLAY=$DISPLAY. Check your desktop session's Xauthority setup." >&2
    exit 1
fi

# Stay attached so Ctrl+C stops the simulation before the temporary key is removed.
docker compose up --no-build
