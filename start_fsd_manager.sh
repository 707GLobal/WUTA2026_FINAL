#!/usr/bin/env bash
# Interactive hardware launcher; all runtime flags are forwarded to the old entry.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export PATH="/usr/bin:$PATH"
exec /usr/bin/python3 "$ROOT/fsd_startup/manager.py" "$@"
