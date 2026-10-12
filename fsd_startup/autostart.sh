#!/usr/bin/env bash
# GNOME login entry: open a real TTY for the interactive manager.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
sleep "${FSD_AUTOSTART_DELAY:-10}"
mkdir -p "$ROOT/.runtime/fsd_manager"
# Avoid opening a second window when the manager already owns its lock.
if ! flock -n "$ROOT/.runtime/fsd_manager/manager.lock" true; then
    exit 0
fi
exec /usr/bin/gnome-terminal --wait --title='FSD 启动管理' -- \
    "$ROOT/start_fsd_manager.sh" --skip-build "$@"
