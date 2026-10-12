#!/usr/bin/env bash
# Source only; no source discovery, builds, simulator startup or control.
set -eo pipefail
FSD_VIS_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
FSD_VIS_ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
[[ -f "$FSD_VIS_ROS_SETUP" ]] || { echo "Missing ROS setup: $FSD_VIS_ROS_SETUP" >&2; return 2; }
source "$FSD_VIS_ROS_SETUP"
if [[ -n "${FSD_SETUP:-}" ]]; then
  [[ -f "$FSD_SETUP" ]] || { echo "Missing FSD_SETUP: $FSD_SETUP" >&2; return 2; }
  source "$FSD_SETUP"
fi
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
