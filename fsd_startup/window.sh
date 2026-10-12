#!/usr/bin/env bash
# ROS failures should appear inside the diagnostics window, not suppress the window.
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
for setup in /opt/ros/humble/setup.bash "$ROOT/humble-chcnav-cgi_ros2pkg/install/setup.bash" "$ROOT/WUTA-FSD/ros2_ws/install/setup.bash"; do
    [[ ! -f "$setup" ]] || source "$setup"
done
export PYTHONNOUSERSITE=1
STATE="$1"
shift
exec /usr/bin/python3 "$ROOT/fsd_startup/dashboard.py" --state "$STATE" "$@"
