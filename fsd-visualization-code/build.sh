#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/scripts/environment.sh"
cd "$FSD_VIS_ROOT"
colcon --log-base "$FSD_VIS_ROOT/log" build \
  --base-paths "$FSD_VIS_ROOT/src" --packages-up-to fsd_visualization \
  --build-base "$FSD_VIS_ROOT/build" --install-base "$FSD_VIS_ROOT/install" \
  --event-handlers console_direct+
