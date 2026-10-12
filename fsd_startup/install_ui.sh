#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
/usr/bin/python3 -m pip install --upgrade --target "$ROOT/.runtime/fsd_manager/ui_deps" \
    -r "$ROOT/fsd_startup/requirements-ui.txt"
