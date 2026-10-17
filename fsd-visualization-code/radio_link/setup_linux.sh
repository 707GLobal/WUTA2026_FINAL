#!/usr/bin/env bash
set -euo pipefail
RADIO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# ROS 2 Humble Python modules must remain visible; use its system Python 3.10.
/usr/bin/python3 -m venv --system-site-packages "$RADIO_ROOT/.venv"
"$RADIO_ROOT/.venv/bin/python" -m pip install -e "$RADIO_ROOT"
printf '%s\n' 'Ready. Source ROS 2 / existing FSD overlays, then bash start_linux.sh RADIO_SERIAL_PORT'
