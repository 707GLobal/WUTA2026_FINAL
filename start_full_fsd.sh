#!/usr/bin/env bash
# Start the hardware sensors, bridges, and the FSD runtime from this checkout.
set -Eeuo pipefail
# ROS Humble and generated message bindings use the system Python 3.10.
export PATH="/usr/bin:$PATH"
export CMAKE_BUILD_PARALLEL_LEVEL=2

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ -x "$ROOT/fsd_startup/sudo_askpass.local.sh" ]]; then
    export SUDO_ASKPASS="$ROOT/fsd_startup/sudo_askpass.local.sh"
    sudo() {
        local argument
        local -a arguments=()
        for argument in "$@"; do
            [[ "$argument" == -n ]] || arguments+=("$argument")
        done
        command sudo -A "${arguments[@]}"
    }
fi
FSD_WS="$ROOT/WUTA-FSD/ros2_ws"
VIS_WS="$ROOT/fsd-visualization-code"
CHC_WS="$ROOT/humble-chcnav-cgi_ros2pkg"
ZED_SETUP="${ZED_SETUP:-/home/wuta/WUTA/zed/install/setup.bash}"
RSLIDAR_SETUP="${RSLIDAR_SETUP:-/home/wuta/WUTA/rslidar_sdk/install/setup.bash}"
HARDWARE_DEPS="${HARDWARE_DEPS:-/home/wuta/lyx/WUTA/.hardware_deps}"
MODEL="${MODEL:-}"
CALIBRATION="${CALIBRATION:-$ROOT/config/camera_lidar.yaml}"
BASE_TO_ZED="${BASE_TO_ZED:-$ROOT/config/base_to_zed.yaml}"
CONTROLLER_CONFIG="$FSD_WS/src/control/controller/config/controller.yaml"
CAN_CONFIG="$FSD_WS/src/system/can_interface/config/can_interface.yaml"
INS_PORT="${INS_PORT:-/dev/ttyUSB0}"
LIDAR_IFACE="${LIDAR_IFACE:-enp8s0}"
LIDAR_IP="${LIDAR_IP:-192.168.1.102/24}"
CAN_IFACE="${CAN_IFACE:-}"
CAN_IFACE_EXPLICIT="$CAN_IFACE"
RUNTIME_DIR="$ROOT/.runtime/full_fsd"
SUPERVISOR_FILE="$RUNTIME_DIR/supervisor.pid"

ACTION=start
BUILD=auto
SENSOR_ONLY=0
RVIZ=1
REUSE_INS=0
CHILD_PIDS=()
CHILD_NAMES=()
OBSERVER_PIDS=()

# Progress messages distinguish a running process from verified sensor data.
item_label() {
    case "$1" in
        chcnav) echo '华测惯导驱动与数据桥' ;;
        zlg_can) echo 'ZLG CAN 硬件桥' ;;
        map_to_odom) echo 'map → odom 静态 TF' ;;
        base_to_zed) echo '车辆 → ZED 外参 TF' ;;
        perception) echo 'ZED / M1 驱动与感知算法组' ;;
        visualization) echo 'FSD 可视化：fsd-visualization-code（相机 / 雷达 / 赛道 / 规划 / 控制）' ;;
        ekf) echo 'EKF 定位滤波' ;;
        localization_manager) echo '定位管理' ;;
        cone_map_builder) echo '锥桶建图' ;;
        boundary_detector) echo '边界与中心线检测' ;;
        path_generator) echo '路径规划' ;;
        controller) echo '车辆控制器' ;;
        can_interface) echo 'VCU / RES CAN 接口' ;;
        mission_manager) echo '任务状态机' ;;
        mission_monitor) echo '任务门控与运行状态监控' ;;
        *) echo "$1" ;;
    esac
}

report_ok() { printf '[成功] %s\n' "$*"; }

show_failure_logs() {
    local name file
    for name in "$@"; do
        file="${LOG_DIR:-}/$name.log"
        [[ -f "$file" ]] || continue
        printf '\n--- %s：最近日志 (%s) ---\n' "$(item_label "$name")" "$file" >&2
        tail -30 "$file" >&2 || true
    done
}

fail() {
    local reason="$1"
    shift
    printf '[失败] %s\n' "$reason" >&2
    show_failure_logs "$@"
    exit 1
}

on_error() {
    local code="$1" line="$2" command="$3"
    printf '[失败] 命令执行失败：退出码=%s，脚本行=%s\n命令：%s\n' \
        "$code" "$line" "$command" >&2
    if [[ -n "${LOG_DIR:-}" ]]; then
        printf '本次日志目录：%s\n' "$LOG_DIR" >&2
    fi
    exit "$code"
}
trap 'on_error "$?" "$LINENO" "$BASH_COMMAND"' ERR

usage() {
    cat <<'EOF'
Usage: ./start_full_fsd.sh [--check|--build-only|--status|--stop]
       [--build|--skip-build] [--sensor-only] [--reuse-ins] [--rviz|--no-rviz] [--image-view]
       [--ins-port=/dev/ttyUSB0] [--model=FILE] [--calibration=FILE]
       [--base-to-zed=FILE] [--controller-config=FILE] [--can-config=FILE] [--print-config]

Default start runs all hardware and FSD runtime nodes. --sensor-only starts
the camera, M1, CAN bridge, INS bridge and perception without vehicle control.
RViz opens by default using fsd-visualization-code and its bundled display
and topic configuration. --no-rviz disables it;
--image-view aliases --rviz.
--reuse-ins uses an existing INS bridge and still verifies its odometry output.
An existing ZLG CAN bridge on the selected interface is verified and reused.
Otherwise a new bridge is started via sudo. External bridges survive FSD shutdown.
Ctrl-C stops only processes started by this script. Logs are in logs/<mission>/<run time>/ (pending until archive).
EOF
}

for arg in "$@"; do
    case "$arg" in
        --check) ACTION=check ;;
        --build-only) ACTION=build_only ;;
        --status) ACTION=status ;;
        --stop) ACTION=stop ;;
        --build) BUILD=yes ;;
        --skip-build) BUILD=no ;;
        --sensor-only) SENSOR_ONLY=1 ;;
        --reuse-can) echo '[提示] --reuse-can 已停用；每次启动都会通过 sudo 重连 CAN 盒。' ;;
        --reuse-ins) REUSE_INS=1 ;;
        --rviz) RVIZ=1 ;;
        --no-rviz) RVIZ=0 ;;
        --image-view) RVIZ=1 ;;
        --ins-port=*) INS_PORT="${arg#*=}" ;;
        --model=*) MODEL="${arg#*=}" ;;
        --calibration=*) CALIBRATION="${arg#*=}" ;;
        --base-to-zed=*) BASE_TO_ZED="${arg#*=}" ;;
        --vehicle-calibration=*) echo "统一参数覆盖已取消，请修改FSD模块config，或使用--controller-config/--can-config" >&2; exit 2 ;;
        --controller-config=*) CONTROLLER_CONFIG="${arg#*=}" ;;
        --can-config=*) CAN_CONFIG="${arg#*=}" ;;
        --print-config) ACTION=print_config ;;
        -h|--help) usage; exit 0 ;;
        *) echo "Unknown option: $arg" >&2; usage >&2; exit 2 ;;
    esac
done

# A single output owner creates the context before checks/builds can fail.
if [[ "$ACTION" != status && "$ACTION" != stop && "$ACTION" != print_config ]]; then
    if [[ -z "${FSD_LOG_OWNER:-}" ]]; then
        exec /usr/bin/python3 "$ROOT/tools/run_logged.py" bash "$ROOT/start_full_fsd.sh" "$@"
    fi
    : "${FSD_RUN_DIR:?日志收集入口必须传入运行目录}"
    RUN_DIR="$FSD_RUN_DIR"
    LOG_DIR="$RUN_DIR/modules"
    export ROS_LOG_DIR="$RUN_DIR/ros"
    mkdir -p "$LOG_DIR" "$ROS_LOG_DIR" "$RUN_DIR/configs" "$RUN_DIR/jrajectory" "$RUN_DIR/artifacts"
fi

load_module_settings() {
    [[ -z "${VEHICLE_CALIBRATION:-}" ]] || echo "[提示] VEHICLE_CALIBRATION 已停用，参数读取各模块 config。" >&2
    MODULE_JSON="$(PYTHONNOUSERSITE=1 /usr/bin/python3 "$ROOT/tools/check_fsd_module_configs.py" \
        --controller "$CONTROLLER_CONFIG" --can "$CAN_CONFIG" --model "$MODEL" --can-device "$CAN_IFACE")"
    mapfile -t MODULE_INFO < <(/usr/bin/python3 -c 'import json,sys; d=json.loads(sys.argv[1])["deployment"]; print("\n".join(str(d[k]) for k in ["model_path","can_device","lidar_topic","image_topic","fusion_topic","ins_topic"]))' "$MODULE_JSON")
    [[ "${#MODULE_INFO[@]}" == 6 ]] || fail '模块配置解析失败'
    MODEL="${MODULE_INFO[0]}"; CAN_IFACE="${MODULE_INFO[1]}"
    LIDAR_TOPIC="${MODULE_INFO[2]}"; IMAGE_TOPIC="${MODULE_INFO[3]}"
    FUSION_TOPIC="${MODULE_INFO[4]}"; INS_TOPIC="${MODULE_INFO[5]}"
}
if [[ "$ACTION" == print_config ]]; then
    load_module_settings
    printf '%s\n' "$MODULE_JSON"
    exit 0
fi

proc_start_time() {
    [[ -r "/proc/$1/stat" ]] && awk '{print $22}' "/proc/$1/stat" || true
}

supervisor_alive() {
    local pid expected actual
    [[ -r "$SUPERVISOR_FILE" ]] || return 1
    read -r pid expected < "$SUPERVISOR_FILE" || return 1
    [[ "$pid" =~ ^[1-9][0-9]*$ && "$expected" =~ ^[0-9]+$ ]] || return 1
    actual="$(proc_start_time "$pid")"
    [[ -n "$actual" && "$actual" == "$expected" ]]
}

if [[ "$ACTION" == status || "$ACTION" == stop ]]; then
    if ! supervisor_alive; then
        echo 'Full FSD supervisor is not running.'
        exit 0
    fi
    read -r supervisor_pid _ < "$SUPERVISOR_FILE"
    if [[ "$ACTION" == status ]]; then
        echo "Full FSD supervisor running: PID $supervisor_pid"
        exit 0
    fi
    kill -INT "$supervisor_pid"
    echo "Stop signal sent to full FSD supervisor $supervisor_pid."
    exit 0
fi

require_file() {
    [[ -f "$1" ]] || fail "缺少必需文件：$1"
}

can_bridge_pid() {
    # Match the interface actually passed to the bridge, not just its name.
    /usr/bin/python3 - "$CAN_IFACE" <<'PY'
import pathlib
import subprocess
import sys

result = subprocess.run(['pgrep', '-x', 'zlg_can_bridge'], capture_output=True, text=True)
matches = []
for pid in result.stdout.split():
    try:
        args = pathlib.Path(f'/proc/{pid}/cmdline').read_bytes().decode().split('\0')
    except (OSError, UnicodeError):
        continue
    iface = 'can0'
    for index, arg in enumerate(args):
        if arg == '--iface' and index + 1 < len(args):
            iface = args[index + 1]
        elif arg.startswith('--iface='):
            iface = arg.split('=', 1)[1]
    if iface == sys.argv[1]:
        matches.append(pid)
if len(matches) == 1 and len(result.stdout.split()) == 1:
    print(matches[0])
    sys.exit(0)
sys.exit(1)
PY
}

check_existing_can() {
    /usr/bin/python3 - "$CAN_IFACE" <<'PY'
import json
import subprocess
import sys

result = subprocess.run(['ip', '-j', '-details', 'link', 'show', 'dev', sys.argv[1]],
                        capture_output=True, text=True)
try:
    link = json.loads(result.stdout)[0]
    valid = (link.get('linkinfo', {}).get('info_kind') == 'vcan'
             and 'UP' in link.get('flags', []))
except (ValueError, IndexError, KeyError):
    valid = False
if result.returncode or not valid:
    print(f"[失败] CAN 桥要求 {sys.argv[1]} 是已启用的 vcan 接口", file=sys.stderr)
    if result.stderr:
        print(result.stderr.strip(), file=sys.stderr)
    sys.exit(1)
PY
}

check_prerequisites() {
    require_file /opt/ros/humble/setup.bash
    require_file "$ZED_SETUP"
    require_file "$RSLIDAR_SETUP"
    require_file "$MODEL"
    require_file "$CALIBRATION"
    require_file "$CONTROLLER_CONFIG"
    require_file "$CAN_CONFIG"
    require_file "$FSD_WS/src/perception/detection_fusion/launch/fusion_modules_hardware.launch.py"
    require_file "$CHC_WS/src/chcnav/scripts/start_chcnav_fsd.sh"
    require_file "$ROOT/zlgcan_bridge/start_zlg_bridge.sh"
    require_file "$ROOT/tools/run_privileged_group.py"
    if [[ "$RVIZ" == 1 ]]; then
        require_file "$VIS_WS/start.sh"
        require_file "$VIS_WS/src/fsd_visualization/config/fsd.rviz"
        require_file "$VIS_WS/src/fsd_visualization/config/topics.yaml"
        [[ -n "${DISPLAY:-}" || -n "${WAYLAND_DISPLAY:-}" ]] || fail '同屏可视化需要图形桌面；无桌面运行请加 --no-rviz'
    fi
    report_ok 'ROS、硬件驱动、模型与感知部署文件齐全'
    [[ -d "$HARDWARE_DEPS" ]] || { fail "缺少 Python 硬件依赖目录：$HARDWARE_DEPS"; }
    [[ -r "$INS_PORT" && -w "$INS_PORT" ]] || {
        echo "[失败] 华测串口不存在或缺少读写权限：$INS_PORT" >&2
        echo "See humble-chcnav-cgi_ros2pkg/README.md: sudo chmod 666 $INS_PORT (temporary), or join dialout and log in again." >&2
        exit 1;
    }
    report_ok "华测串口可读写：$INS_PORT"
    if [[ "$REUSE_INS" == 0 ]]; then
        /usr/bin/python3 "$CHC_WS/src/chcnav/scripts/check_serial_port.py" "$INS_PORT" || \
            fail '华测串口不可独占，尚未启动任何硬件进程'
    fi
    [[ "$(cat "/sys/class/net/$LIDAR_IFACE/carrier" 2>/dev/null || true)" == 1 ]] || {
        fail "M1 网口未连接：$LIDAR_IFACE";
    }
    ip -4 -o addr show dev "$LIDAR_IFACE" | grep -Fq " $LIDAR_IP " || {
        fail "M1 网口 $LIDAR_IFACE 未配置地址 $LIDAR_IP";
    }
    report_ok "M1 网口连接与地址检查通过：$LIDAR_IFACE / $LIDAR_IP"
    command -v lsusb >/dev/null || fail '缺少 lsusb 命令，无法检查 ZLG USB 设备'
    lsusb -d 0471:126a | grep -q . || {
        fail '未检测到 ZLG USBCAN-4E-U USB 设备 (0471:126a)';
    }
    report_ok 'ZLG USB 设备已检测到'
    local bridge_pid
    if bridge_pid="$(can_bridge_pid)"; then
        report_ok "检测到旧 ZLG CAN 桥：$CAN_IFACE（PID=$bridge_pid）；启动时验证接口并复用，退出 FSD 时保留该桥"
    elif pgrep -x zlg_can_bridge >/dev/null; then
        fail "已有 ZLG CAN 桥使用其他接口或存在重复实例，请检查桥参数与 CAN_IFACE=$CAN_IFACE"
    fi
    if [[ "$SENSOR_ONLY" == 0 ]]; then
        require_file "$BASE_TO_ZED"
        [[ -f "$ROOT/tools/monitor_mission.py" ]] || echo "[警告] 缺少只读任务监控；车辆模块仍可启动"
    fi
    report_ok '硬件连接与部署文件检查通过'
}

read_base_to_zed() {
    # Reject placeholders and non-finite values before publishing a vehicle TF.
    mapfile -t BASE_TF < <(/usr/bin/python3 - "$BASE_TO_ZED" <<'PY'
import math
import sys
import yaml

with open(sys.argv[1], encoding='utf-8') as stream:
    data = yaml.safe_load(stream) or {}
translation = data.get('translation')
quaternion = data.get('quaternion')
if not isinstance(translation, list) or len(translation) != 3:
    raise SystemExit('base_to_zed.yaml requires translation: [x, y, z]')
if not isinstance(quaternion, list) or len(quaternion) != 4:
    raise SystemExit('base_to_zed.yaml requires quaternion: [x, y, z, w]')
values = translation + quaternion
if any(not isinstance(value, (int, float)) or not math.isfinite(value) for value in values):
    raise SystemExit('base_to_zed.yaml contains an unset or non-finite value')
norm = math.sqrt(sum(value * value for value in quaternion))
if abs(norm - 1.0) > 0.01:
    raise SystemExit('base_to_zed.yaml quaternion must have unit length')
for value in values:
    print(value)
PY
    )
    [[ "${#BASE_TF[@]}" == 7 ]] || { fail '车辆到 ZED 外参无效，具体校验原因见上方 Python 输出'; }
    report_ok '车辆到 ZED 外参校验通过'
}

load_module_settings
if [[ -n "${RUN_DIR:-}" ]]; then
    printf '%s\n' "$MODULE_JSON" > "$RUN_DIR/configs/module_config_manifest.json"
    /usr/bin/python3 "$ROOT/tools/snapshot_run_configs.py" "$RUN_DIR/configs/module_config_manifest.json" "$RUN_DIR/configs" \
        "camera_lidar=$CALIBRATION" "base_to_zed=$BASE_TO_ZED" \
        "chcnav_bridge=$CHC_WS/src/chcnav/config/chcnav_fsd_bridge.yaml" \
        || echo '[警告] 配置快照未完整保存，源配置检查仍按原流程执行' >&2
fi

if [[ "$ACTION" == check ]]; then
    check_prerequisites
    [[ "$SENSOR_ONLY" == 1 ]] || read_base_to_zed
    report_ok '启动前检查全部完成'
    exit 0
fi

build_workspaces() {
    set +u
    source /opt/ros/humble/setup.bash
    set -u
    (cd "$CHC_WS" && colcon build --packages-select msg_interfaces chcnav \
        --cmake-clean-cache --cmake-args \
        -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3 \
        -DPYTHON_INCLUDE_DIR=/usr/include/python3.10 \
        -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.10.so \
        -DPYTHON_INSTALL_DIR=lib/python3.10/site-packages)
    report_ok '华测驱动与消息包构建完成'
    set +u
    source "$CHC_WS/install/setup.bash"
    set -u
    (cd "$FSD_WS" && colcon build --base-paths src --symlink-install --parallel-workers 2 \
        --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3)
    report_ok 'FSD 算法包构建完成'
}

build_visualization() {
    FSD_SETUP="$FSD_WS/install/setup.bash" bash "$VIS_WS/build.sh"
    report_ok 'fsd-visualization-code 可视化与安全图像插件构建完成'
}

if [[ "$BUILD" == yes || ( "$BUILD" == auto &&
    ( ! -f "$CHC_WS/install/setup.bash" || ! -f "$FSD_WS/install/setup.bash" ) ) ]]; then
    build_workspaces
fi
if [[ "$ACTION" == build_only ]]; then
    build_visualization
    (cd "$ROOT/zlgcan_bridge" && make -j2)
    report_ok '华测、FSD、ZLG CAN 桥构建全部完成'
    exit 0
fi
if [[ "$RVIZ" == 1 ]]; then
    if [[ "$BUILD" == yes || ( "$BUILD" == auto && ! -f "$VIS_WS/install/local_setup.bash" ) ]]; then
        build_visualization
    fi
    [[ -f "$VIS_WS/install/local_setup.bash" ]] || fail '可视化尚未构建，请先执行 ./start_full_fsd.sh --build-only，或移除 --skip-build'
fi
require_file "$CHC_WS/install/setup.bash"
require_file "$FSD_WS/install/setup.bash"
check_prerequisites
[[ "$SENSOR_ONLY" == 1 ]] || read_base_to_zed
if supervisor_alive; then
    echo '[失败] 已有全栈启动脚本正在运行。' >&2
    exit 1
fi

# Source every overlay once, then all child processes inherit one ROS domain.
set +u
source /opt/ros/humble/setup.bash
source "$ZED_SETUP"
source "$RSLIDAR_SETUP"
source "$CHC_WS/install/setup.bash"
source "$FSD_WS/install/setup.bash"
set -u
report_ok 'ROS 与硬件、算法工作空间环境加载完成'
export PATH="/usr/bin:$PATH"
export LD_LIBRARY_PATH="/usr/local/cuda-11.8/targets/x86_64-linux/lib:/home/wuta/WUTA/rslidar_sdk/install_msg/lib:${LD_LIBRARY_PATH:-}"
export PYTHONPATH="$HARDWARE_DEPS:${PYTHONPATH:-}"

mkdir -p "$RUNTIME_DIR" "$ROS_LOG_DIR"
mkdir -p "$LOG_DIR"
printf '%s %s\n' "$$" "$(proc_start_time "$$")" > "$SUPERVISOR_FILE"

cleanup() {
    local index pid
    trap - EXIT INT TERM
    # Observers share shutdown only; their failures never gate vehicle runtime.
    for pid in "${OBSERVER_PIDS[@]}"; do
        CHILD_PIDS+=("$pid")
        CHILD_NAMES+=(mission_observer)
    done
    for ((index=${#CHILD_PIDS[@]}-1; index>=0; index--)); do
        pid="${CHILD_PIDS[index]}"
        if [[ "${CHILD_NAMES[index]}" == zlg_can ]]; then
            # sudo forwards to the root supervisor, which signals the bridge group.
            kill -INT "$pid" 2>/dev/null || true
        else
            kill -INT -- "-$pid" 2>/dev/null || true
        fi
    done
    # Let ROS launch and the camera SDK close their children before escalating.
    local attempt alive
    for ((attempt=0; attempt<20; attempt++)); do
        alive=0
        for pid in "${CHILD_PIDS[@]}"; do
            kill -0 "$pid" 2>/dev/null && alive=1
        done
        [[ "$alive" == 0 ]] && break
        sleep 0.5
    done
    for ((index=${#CHILD_PIDS[@]}-1; index>=0; index--)); do
        pid="${CHILD_PIDS[index]}"
        if [[ "${CHILD_NAMES[index]}" == zlg_can ]]; then
            kill -TERM "$pid" 2>/dev/null || true
        else
            kill -TERM -- "-$pid" 2>/dev/null || true
        fi
        wait "$pid" 2>/dev/null || true
    done
    if supervisor_alive; then
        local recorded _
        read -r recorded _ < "$SUPERVISOR_FILE"
        [[ "$recorded" != "$$" ]] || rm -f "$SUPERVISOR_FILE"
    fi
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

report_child_failure() {
    local index="$1" phase="$2" code=0
    local name="${CHILD_NAMES[index]}" pid="${CHILD_PIDS[index]}"
    wait "$pid" || code=$?
    fail "$(item_label "$name") $phase（PID=$pid，退出码=$code）。驱动/节点输出如下：" "$name"
}

check_children() {
    local observer
    for observer in "${OBSERVER_PIDS[@]}"; do
        if ! kill -0 "$observer" 2>/dev/null && [[ "${OBSERVER_WARNED:-0}" == 0 ]]; then
            echo "[警告] 只读任务监控退出；车辆运行继续，赛项记录可能不完整" >&2
            OBSERVER_WARNED=1
        fi
    done
    local index
    for index in "${!CHILD_PIDS[@]}"; do
        if ! kill -0 "${CHILD_PIDS[index]}" 2>/dev/null; then
            report_child_failure "$index" '进程已退出'
        fi
    done
}

start_group() {
    local name="$1"
    shift
    printf '[启动] %s ...\n' "$(item_label "$name")"
    if [[ "$name" == zlg_can ]]; then
        # Elevate before creating the bridge session. The root supervisor waits
        # for the actual bridge and forwards shutdown to its whole process group.
        sudo -n /usr/bin/python3 "$ROOT/tools/run_privileged_group.py" "$@" > "$LOG_DIR/$name.log" 2>&1 &
    else
        setsid "$@" > "$LOG_DIR/$name.log" 2>&1 &
    fi
    local pid=$!
    CHILD_PIDS+=("$pid")
    CHILD_NAMES+=("$name")
    sleep 0.3
    check_children
    report_ok "$(item_label "$name")：进程已启动（PID=$pid）；日志：$LOG_DIR/$name.log"
}

wait_topic() {
    local topic="$1" seconds="${2:-15}" label="${3:-$1}"
    if (( $# >= 3 )); then shift 3; else shift "$#"; fi
    local diagnostic="$LOG_DIR/topic_${topic//\//_}.log" code=0
    local message_type
    case "$topic" in
        "$LIDAR_TOPIC") message_type=sensor_msgs/msg/PointCloud2 ;;
        "$IMAGE_TOPIC") message_type=sensor_msgs/msg/Image ;;
        "$FUSION_TOPIC") message_type=wuta_msgs/msg/ConeArray ;;
        "$INS_TOPIC") message_type=nav_msgs/msg/Odometry ;;
        /localization/pose) message_type=geometry_msgs/msg/PoseStamped ;;
        /system/mission_state) message_type=wuta_msgs/msg/MissionState ;;
        *) fail "未配置启动检查的话题类型：$topic" ;;
    esac
    printf '[检查] %s：等待 %s（最多 %s 秒）...\n' "$label" "$topic" "$seconds"
    # Preserve CLI errors, and accept either best-effort or reliable sensor publishers.
    # Explicit types let echo wait even before DDS discovers a publisher.
    timeout "$seconds" ros2 topic echo "$topic" "$message_type" --once --field header \
        --qos-reliability best_effort >"$diagnostic" 2>&1 || code=$?
    [[ "${TOPIC_CHECK_WORKER:-0}" == 1 ]] || check_children
    if [[ "$code" != 0 ]]; then
        printf '话题检查输出（%s）：\n' "$diagnostic" >&2
        tail -20 "$diagnostic" >&2 || true
        if [[ "$code" == 124 ]]; then
            fail "$label：${seconds} 秒内未收到 $topic 消息。可能为设备无数据、话题不匹配或 DDS/QoS 问题；相关日志如下：" "$@"
        fi
        fail "$label：ROS 话题检查失败（退出码=$code，话题=$topic）；相关日志如下：" "$@"
    fi
    report_ok "$label：已收到 $topic 消息"
}

wait_independent_sensor_topics() {
    local -a probes=()
    local pid failed=0
    # Probe subprocesses must not inherit the supervisor's EXIT cleanup.
    (trap - EXIT ERR INT TERM; TOPIC_CHECK_WORKER=1 wait_topic "$LIDAR_TOPIC" 20 'M1 激光雷达数据' perception) &
    probes+=("$!")
    (trap - EXIT ERR INT TERM; TOPIC_CHECK_WORKER=1 wait_topic "$IMAGE_TOPIC" 20 'ZED 相机图像' perception) &
    probes+=("$!")
    (trap - EXIT ERR INT TERM; TOPIC_CHECK_WORKER=1 wait_topic "$INS_TOPIC" 20 '华测惯导输出' chcnav) &
    probes+=("$!")
    for pid in "${probes[@]}"; do
        wait "$pid" || failed=1
    done
    check_children
    [[ "$failed" == 0 ]] || fail '独立传感器并行检查存在失败；各项完整原因见上方输出与 topic 日志' perception chcnav
}

# Authenticate in the foreground before stopping the old bridge or starting hardware.
if ! sudo -n -v 2>/dev/null; then
    [[ -t 0 ]] || fail '重连 CAN 盒需要 sudo 密码；请在交互终端运行 ./start_full_fsd.sh'
    echo '[认证] 重连 CAN 盒需要 sudo 权限，请在当前终端输入密码。'
    sudo -v || fail 'sudo 认证失败，尚未重连 CAN 盒或启动任何硬件进程'
fi

reconnect_can_bridge() {
    local bridge_pid expected attempt
    if bridge_pid="$(can_bridge_pid)"; then
        expected="$(proc_start_time "$bridge_pid")"
        [[ -n "$expected" ]] || fail '已有 CAN 桥在检查时退出，未打开第二个设备实例' zlg_can
        check_existing_can || fail "已有 CAN 桥的 $CAN_IFACE 接口未就绪，保留外部进程" zlg_can
        [[ "$(proc_start_time "$bridge_pid")" == "$expected" ]] || fail '已有 CAN 桥在检查时退出，未打开第二个设备实例' zlg_can
        # External bridge is deliberately not registered in CHILD_PIDS:
        # FSD shutdown must not stop a bridge owned by another terminal.
        printf '[复用] 外部 CAN 桥：PID=%s，接口=%s；退出 FSD 时保留该桥。\n' "$bridge_pid" "$CAN_IFACE" > "$LOG_DIR/zlg_can.log"
        report_ok "已复用外部 CAN 桥：$CAN_IFACE（PID=$bridge_pid）；接口已就绪，收报请查看独立桥终端或 candump $CAN_IFACE"
        return
    fi
    # Never open the same USB device twice; also catch changes since prerequisites.
    if pgrep -x zlg_can_bridge >/dev/null; then
        fail '仍有 ZLG CAN 桥进程，无法独占设备并重连'
    fi
    start_group zlg_can bash "$ROOT/zlgcan_bridge/start_zlg_bridge.sh" "--iface=$CAN_IFACE"
    for ((attempt=0; attempt<300; attempt++)); do
        check_children
        if grep -Fq '[main] 就绪:' "$LOG_DIR/zlg_can.log"; then
            check_existing_can || fail "CAN 桥初始化完成，但 $CAN_IFACE 接口未就绪" zlg_can
            report_ok "CAN 盒已重新打开并初始化：$CAN_IFACE；VCU 收报情况请查看 $LOG_DIR/zlg_can.log"
            return
        fi
        sleep 0.1
    done
    fail 'CAN 桥 30 秒内未完成设备初始化；接口存在不代表 CAN 盒已连接' zlg_can
}

reconnect_can_bridge
if [[ "$REUSE_INS" == 1 ]]; then
    report_ok '复用外部华测链路；将检查惯导输出话题'
else
    if [[ "${FSD_INS_DIAGNOSTICS:-1}" == 1 ]]; then
        export CHCNAV_DIAGNOSTIC_DIR="$RUN_DIR/artifacts/chcnav_diagnostic"
    fi
    start_group chcnav bash "$CHC_WS/src/chcnav/scripts/start_chcnav_fsd.sh" --no-build "--port=$INS_PORT" "--log-dir=$RUN_DIR/artifacts/chcnav"
fi

if [[ "$SENSOR_ONLY" == 0 ]]; then
    start_group map_to_odom ros2 run tf2_ros static_transform_publisher \
        --x 0 --y 0 --z 0 --qx 0 --qy 0 --qz 0 --qw 1 \
        --frame-id map --child-frame-id odom
    start_group base_to_zed ros2 run tf2_ros static_transform_publisher \
        --x "${BASE_TF[0]}" --y "${BASE_TF[1]}" --z "${BASE_TF[2]}" \
        --qx "${BASE_TF[3]}" --qy "${BASE_TF[4]}" --qz "${BASE_TF[5]}" --qw "${BASE_TF[6]}" \
        --frame-id base_link --child-frame-id zed_camera_link
fi

PERCEPTION_ARGS=(config_root:="$FSD_WS/src" model_path:="$MODEL" calibration_path:="$CALIBRATION"
    zed_publish_tf:=false zed_publish_map_tf:=false)
start_group perception ros2 launch detection_fusion fusion_modules_hardware.launch.py "${PERCEPTION_ARGS[@]}"
if [[ "${FSD_PARALLEL_CHECKS:-0}" == 1 ]]; then
    wait_independent_sensor_topics
    wait_topic "$FUSION_TOPIC" 20 '检测融合输出' perception
else
    wait_topic "$LIDAR_TOPIC" 20 'M1 激光雷达数据' perception
    wait_topic "$IMAGE_TOPIC" 20 'ZED 相机图像' perception
    wait_topic "$FUSION_TOPIC" 20 '检测融合输出' perception
    if [[ "$SENSOR_ONLY" == 0 || "$REUSE_INS" == 1 ]]; then
        wait_topic "$INS_TOPIC" 20 '华测惯导输出' chcnav
    fi
fi

if [[ "$SENSOR_ONLY" == 0 ]]; then
    start_group ekf ros2 run robot_localization ekf_node --ros-args \
        -r __node:=ekf_node \
        --params-file "$FSD_WS/src/localization/localization_manager/config/ekf.yaml" \
        -p "debug_out_file:=$RUN_DIR/modules/ekf_debug.txt"
    start_group localization_manager ros2 run localization_manager localization_manager_node
    wait_topic /localization/pose 20 '定位输出' ekf localization_manager chcnav
    start_group cone_map_builder ros2 run cone_map_builder cone_map_builder_node --ros-args \
        --params-file "$FSD_WS/src/mapping/cone_map_builder/config/cone_map_builder.yaml" \
        -p "map_save_path:=$RUN_DIR/artifacts/cone_map.yaml" \
        -r /perception/lidar/cones:=/perception/fused/cones
    start_group boundary_detector ros2 run boundary_detector boundary_detector_node --ros-args \
        --params-file "$FSD_WS/src/planning/boundary_detector/config/boundary_detector.yaml"
    start_group path_generator ros2 run path_generator path_generator_node --ros-args \
        --params-file "$FSD_WS/src/planning/path_generator/config/path_generator.yaml" \
        -p "record.run_dir:=$RUN_DIR"
    start_group controller ros2 run controller controller_node --ros-args \
        --params-file "$CONTROLLER_CONFIG"
    if [[ -f "$ROOT/tools/monitor_mission.py" ]]; then
        setsid /usr/bin/python3 -u "$ROOT/tools/monitor_mission.py" --run-dir "$RUN_DIR" &
        OBSERVER_PIDS+=("$!")
        report_ok '只读任务监控已启动，变化事件统一进入 manager.log'
    fi
    CAN_ARGS=()
    [[ -z "$CAN_IFACE_EXPLICIT" ]] || CAN_ARGS=(-p "can.device:=$CAN_IFACE")
    start_group can_interface ros2 run can_interface can_interface_node --ros-args \
        --params-file "$CAN_CONFIG" "${CAN_ARGS[@]}"
    start_group mission_manager ros2 run mission_manager mission_manager_node --ros-args \
        --params-file "$FSD_WS/src/system/mission_manager/config/mission_manager.yaml"
    wait_topic /system/mission_state 15 '任务状态广播' mission_manager
fi

if [[ "$RVIZ" == 1 ]]; then
    VIS_ARGS=(mode:=bridge use_sim_time:=false)
    if [[ "$SENSOR_ONLY" == 1 ]]; then
        # Sensor-only mode has no map/base_link TF or planning nodes.
        /usr/bin/python3 - "$VIS_WS/src/fsd_visualization/config/topics.yaml" "$RUN_DIR/configs/visualization_topics.yaml" <<'PY'
import sys
import yaml
with open(sys.argv[1], encoding='utf-8') as stream:
    settings = yaml.safe_load(stream)
settings['fixed_frame'] = settings['target_frame'] = 'rslidar'
for key in ('cone_map', 'centerline', 'final_waypoints', 'pose', 'velocity',
            'control_target', 'map_markers', 'centerline_markers',
            'path_markers', 'trajectory_markers'):
    settings['topics'][key] = ''
with open(sys.argv[2], 'w', encoding='utf-8') as stream:
    yaml.safe_dump(settings, stream, sort_keys=False)
PY
        VIS_ARGS+=(topics_config:="$RUN_DIR/configs/visualization_topics.yaml")
    fi
    start_group visualization env FSD_SETUP="$FSD_WS/install/setup.bash" \
        bash "$VIS_WS/start.sh" "${VIS_ARGS[@]}"
fi

report_ok "全栈启动检查完成，已启动 ${#CHILD_PIDS[@]} 个进程组。日志：$LOG_DIR"
if [[ "$SENSOR_ONLY" == 0 ]]; then
    echo '[提示] 任务仍需满足就绪条件、由 VCU 选择模式，再按 RES GO。'
fi
echo 'Press Ctrl-C to stop this stack.'
while true; do
    check_children
    sleep 2
done
