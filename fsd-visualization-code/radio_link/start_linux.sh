#!/usr/bin/env bash
set -euo pipefail
RADIO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ $# -lt 1 || -z "$1" ]]; then
    printf '%s\n' 'Usage: bash start_linux.sh /dev/serial/by-id/RADIO_DEVICE [--mode native|bridge] [--use-sim-time]' >&2
    exit 2
fi
RADIO_PORT="$1"
shift
"$RADIO_ROOT/.venv/bin/python" -c 'import rclpy, tf2_ros, visualization_msgs.msg' || {
    printf '%s\n' 'Source ROS 2 Humble and the existing FSD overlay before launching.' >&2
    exit 2
}
exec "$RADIO_ROOT/.venv/bin/python" -m fsd_radio.cli --config "$RADIO_ROOT/radio.yaml" linux --port "$RADIO_PORT" "$@"
