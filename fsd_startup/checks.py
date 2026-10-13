"""Independent preflight groups. Checks are read-only and collect all errors."""
from pathlib import Path
import os
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
from check_fsd_module_configs import manifest


def run(argv):
    result = subprocess.run(argv, capture_output=True, text=True, timeout=10)
    if result.returncode:
        raise RuntimeError(f"命令：{argv!r}\n退出码：{result.returncode}\n"
                           f"stdout：{result.stdout.strip()}\nstderr：{result.stderr.strip()}")
    return result.stdout


def require(paths):
    missing = [str(path) for path in paths if not Path(path).is_file()]
    if missing:
        raise RuntimeError('缺少文件：\n' + '\n'.join(missing))


def configuration(options):
    data = manifest(options.get('controller-config'), options.get('can-config'),
                    options.get('model', os.environ.get('MODEL', '')),
                    os.environ.get('CAN_IFACE', ''))
    require([data['deployment']['model_path']])
    return data


def dependencies(options):
    require(['/opt/ros/humble/setup.bash',
             os.environ.get('ZED_SETUP', '/home/wuta/WUTA/zed/install/setup.bash'),
             os.environ.get('RSLIDAR_SETUP', '/home/wuta/WUTA/rslidar_sdk/install/setup.bash'),
             ROOT / 'zlgcan_bridge/start_zlg_bridge.sh', ROOT / 'tools/run_privileged_group.py',
             ROOT / 'WUTA-FSD/ros2_ws/src/perception/detection_fusion/launch/fusion_modules_hardware.launch.py',
             ROOT / 'humble-chcnav-cgi_ros2pkg/src/chcnav/scripts/start_chcnav_fsd.sh'])
    deps = Path(os.environ.get('HARDWARE_DEPS', '/home/wuta/lyx/WUTA/.hardware_deps'))
    if not deps.is_dir():
        raise RuntimeError(f'缺少硬件 Python 依赖目录：{deps}')
    return 'ROS、驱动入口、Python 依赖目录存在'


def serial(options):
    port = options.get('ins-port', os.environ.get('INS_PORT', '/dev/ttyUSB0'))
    if not os.access(port, os.R_OK | os.W_OK):
        raise RuntimeError(f'{port} 不存在或当前用户不可读写；菜单 p 可设置当前用户 ACL')
    if 'reuse-ins' in options:
        return f'{port} 可读写；复用惯导，按原脚本跳过独占检查，启动时验证 odometry'
    run(['/usr/bin/python3', str(ROOT / 'humble-chcnav-cgi_ros2pkg/src/chcnav/scripts/check_serial_port.py'), port])
    return f'{port} 权限与独占检查通过'


def lidar(options):
    iface = os.environ.get('LIDAR_IFACE', 'enp8s0')
    ip = os.environ.get('LIDAR_IP', '192.168.1.102/24')
    if Path(f'/sys/class/net/{iface}/carrier').read_text().strip() != '1':
        raise RuntimeError(f'M1 网口 {iface} 未连接')
    output = run(['ip', '-4', '-o', 'addr', 'show', 'dev', iface])
    if f' {ip} ' not in output:
        raise RuntimeError(f'M1 地址期望 {ip}，实际：{output.strip()}')
    return f'{iface} / {ip} 已连接'


def can(options):
    output = run(['lsusb', '-d', '0471:126a'])
    if not output.strip():
        raise RuntimeError('未检测到 ZLG USBCAN-4E-U (0471:126a)')
    if not any(Path(p).is_file() for p in ('/usr/lib/libusbcan-4e.so', '/lib/libusbcan-4e.so')):
        raise RuntimeError('缺少 ZLG libusbcan-4e.so')
    return 'ZLG USB 与驱动库存在；启动时 sudo 初始化接口，收报由日志窗口观察'


def calibration(options):
    import math
    import yaml
    require([options.get('calibration', os.environ.get('CALIBRATION', str(ROOT / 'config/camera_lidar.yaml')))])
    if 'sensor-only' not in options:
        path = options.get('base-to-zed', os.environ.get('BASE_TO_ZED', str(ROOT / 'config/base_to_zed.yaml')))
        require([path])
        data = yaml.safe_load(Path(path).read_text()) or {}
        xyz, quat = data.get('translation', []), data.get('quaternion', [])
        if not isinstance(xyz, list) or not isinstance(quat, list) or len(xyz) != 3 or len(quat) != 4 or not all(isinstance(v, (int, float)) and math.isfinite(v) for v in xyz + quat):
            raise RuntimeError(f'{path}：translation/quaternion 结构或数值无效')
        if abs(math.sqrt(sum(v * v for v in quat)) - 1) > .01:
            raise RuntimeError(f'{path}：四元数未归一化')
    return ('相机雷达外参文件存在；sensor-only 不检查车辆 TF' if 'sensor-only' in options else
            '相机雷达外参文件与车辆 TF 检查通过；外参链由感知 launch 校验')


def visualization(options):
    if not options.get('_rviz', 'no-rviz' not in options):
        return '已禁用 RViz；按原脚本跳过可视化文件和桌面检查'
    require([ROOT / 'fsd-visualization-code/start.sh',
             ROOT / 'fsd-visualization-code/src/fsd_visualization/config/fsd.rviz',
             ROOT / 'fsd-visualization-code/src/fsd_visualization/config/topics.yaml'])
    if not (os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')):
        raise RuntimeError('同屏可视化需要图形桌面；无桌面运行请加 --no-rviz')
    return 'RViz 入口、显示配置、话题配置及图形桌面检查通过'


def build_artifacts(options):
    paths = [ROOT / 'humble-chcnav-cgi_ros2pkg/install/setup.bash',
             ROOT / 'WUTA-FSD/ros2_ws/install/setup.bash']
    if options.get('_rviz', 'no-rviz' not in options):
        paths.append(ROOT / 'fsd-visualization-code/install/local_setup.bash')
    missing = [str(path) for path in paths if not path.is_file()]
    mode = options.get('_build', 'no' if 'skip-build' in options else 'yes' if 'build' in options else 'auto')
    if missing and mode == 'no':
        raise RuntimeError('--skip-build 但缺少安装产物：\n' + '\n'.join(missing))
    if mode == 'yes':
        return '指定强制构建；构建完成与产物可用性在启动阶段由原脚本验证'
    if missing:
        return '需自动构建（尚未构建成功）：\n' + '\n'.join(missing)
    return '华测、FSD及启用的可视化安装入口存在；原脚本启动时加载'


def can_occupancy(options):
    data = manifest(options.get('controller-config'), options.get('can-config'),
                    options.get('model', os.environ.get('MODEL', '')), os.environ.get('CAN_IFACE', ''))
    selected = data['deployment']['can_device']
    result = subprocess.run(['pgrep', '-x', 'zlg_can_bridge'], capture_output=True, text=True, timeout=10)
    if result.returncode not in (0, 1):
        raise RuntimeError(f'CAN 桥进程查询失败：{result.stderr}')
    all_pids = result.stdout.split()
    matches = []
    for pid in all_pids:
        try:
            args = Path(f'/proc/{pid}/cmdline').read_bytes().decode().split('\0')
        except (OSError, UnicodeError):
            continue
        iface = 'can0'
        for index, argument in enumerate(args):
            if argument == '--iface' and index + 1 < len(args):
                iface = args[index + 1]
            elif argument.startswith('--iface='):
                iface = argument.split('=', 1)[1]
        if iface == selected:
            matches.append(pid)
    # This intentionally follows can_bridge_pid/check_prerequisites in the old script.
    if len(matches) == 1 and len(all_pids) == 1:
        return f'外部桥 PID={matches[0]}，接口={selected}；启动时验证并复用，退出 FSD 时保留'
    if all_pids:
        raise RuntimeError(f'已有 ZLG 桥使用其他接口或存在重复实例；CAN_IFACE={selected}，PID={all_pids}')
    return f'未发现旧桥；启动时初始化 {selected} 并验证 vcan/UP 和设备就绪日志'


def supervisor(options):
    from processes import identity
    path = ROOT / '.runtime/full_fsd/supervisor.pid'
    try:
        pid, expected = path.read_text().split()
        if int(pid) <= 0 or not expected.isdigit():
            return '原启动器 PID 文件无效；与原脚本一致视为未运行'
        actual = identity(int(pid))
    except (OSError, ValueError):
        return '原全栈启动器未运行'
    if actual and actual['start'] == expected:
        raise RuntimeError(f'已有全栈启动脚本正在运行：PID={pid}。不能启动第二套硬件链路')
    return '原全栈启动器未运行（记录 PID 已失效）'


def observer(options):
    if 'sensor-only' in options:
        return 'sensor-only 按原脚本不启动只读任务监控'
    if not (ROOT / 'tools/monitor_mission.py').is_file():
        return '警告：缺少只读任务监控；与原脚本一致不阻止车辆模块启动'
    return '只读任务监控脚本存在；其退出按原脚本仅告警'


CHECKS = {'模块配置': configuration, 'ROS与部署': dependencies, '华测串口': serial,
          'M1网络': lidar, 'CAN硬件': can, '外参与TF': calibration,
          'RViz部署与桌面': visualization, '工作空间产物': build_artifacts,
          'CAN桥占用': can_occupancy, '原启动器单实例': supervisor, '只读任务监控文件': observer}
