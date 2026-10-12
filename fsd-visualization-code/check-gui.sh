#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/scripts/environment.sh"
source "$FSD_VIS_ROOT/install/local_setup.bash"
export ROS_DOMAIN_ID="${FSD_VIS_TEST_DOMAIN:-180}" ROS_LOCALHOST_ONLY=1
# Requires a working graphical desktop; opens and closes one synthetic viewer.
/usr/bin/python3 "$FSD_VIS_ROOT/tests/gui_smoke.py" "$@"
