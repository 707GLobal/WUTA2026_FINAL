#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/scripts/environment.sh"
source "$FSD_VIS_ROOT/install/local_setup.bash"
# Deliberately use an isolated DDS domain; never inject test data into live FSD.
export ROS_DOMAIN_ID="${FSD_VIS_TEST_DOMAIN:-179}" ROS_LOCALHOST_ONLY=1
export PYTHONPATH="$FSD_VIS_ROOT/src/fsd_visualization${PYTHONPATH:+:$PYTHONPATH}"
/usr/bin/python3 -m pytest "$FSD_VIS_ROOT/tests" -q "$@"
