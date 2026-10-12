#!/usr/bin/env bash
# 当前FSD + WUTA-SIM：隔离域、自动记录、限时测试，不启动任何真实硬件桥。
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ -z "${FSD_LOG_OWNER:-}" ]]; then
    export FSD_RUN_MODE=simulation
    exec /usr/bin/python3 "$ROOT/tools/run_logged.py" bash "$ROOT/start_wuta_sim_validation.sh" "$@"
fi
SIM_SETUP="${SIM_SETUP:-/home/wuta/lyx/WUTA/WUTA-SIM/install/setup.bash}"
export PATH=/usr/bin:/bin
export PYTHONNOUSERSITE=1
export ROS_DOMAIN_ID="${WUTA_SIM_DOMAIN:-157}"
[[ "$ROS_DOMAIN_ID" =~ ^[0-9]+$ ]] && (( ROS_DOMAIN_ID >= 101 && ROS_DOMAIN_ID <= 232 )) || { echo "仿真域须为101～232，禁止使用默认硬件域0" >&2; exit 2; }
mkdir -p "$ROOT/.runtime"
exec 9>"$ROOT/.runtime/wuta_sim_${ROS_DOMAIN_ID}.lock"
flock -n 9 || { echo "该仿真域已有本项目测试运行" >&2; exit 2; }
export ROS_LOCALHOST_ONLY=1
export ROS_LOG_DIR="$FSD_RUN_DIR/ros"
source /opt/ros/humble/setup.bash
source "$SIM_SETUP"
source "$ROOT/WUTA-FSD/ros2_ws/install/local_setup.bash"
exec /usr/bin/python3 "$ROOT/tools/run_wuta_sim_validation.py" "$@"
