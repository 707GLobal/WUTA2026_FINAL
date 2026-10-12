#!/usr/bin/env bash
# Default launch always opens RViz; extra arguments are ROS launch arguments.
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/scripts/environment.sh"
[[ -f "$FSD_VIS_ROOT/install/local_setup.bash" ]] || {
  echo 'Build the visualization package first: bash build.sh' >&2; exit 2;
}
source "$FSD_VIS_ROOT/install/local_setup.bash"
if [[ "${FSD_VIS_NVIDIA:-0}" == 1 ]]; then
  export __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia
fi
exec ros2 launch fsd_visualization view.launch.py \
  topics_config:="$FSD_VIS_ROOT/src/fsd_visualization/config/topics.yaml" \
  rviz_config:="$FSD_VIS_ROOT/src/fsd_visualization/config/fsd.rviz" "$@"
