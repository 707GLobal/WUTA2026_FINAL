"""Display the existing shell launcher's verification; never invent extra readiness."""
from collections import OrderedDict
from pathlib import Path
import re

SOURCE = Path(__file__).resolve().parents[1] / 'start_full_fsd.sh'
PROCESS_NAMES = ('zlg_can', 'chcnav', 'map_to_odom', 'base_to_zed', 'perception', 'ekf',
                 'localization_manager', 'cone_map_builder', 'boundary_detector',
                 'path_generator', 'controller', 'can_interface', 'mission_manager', 'visualization')
FULL_ONLY = {'map_to_odom', 'base_to_zed', 'ekf', 'localization_manager', 'cone_map_builder',
             'boundary_detector', 'path_generator', 'controller', 'can_interface', 'mission_manager'}
TOPICS = ('M1 激光雷达数据', 'ZED 相机图像', '华测惯导输出', '检测融合输出', '定位输出', '任务状态广播')
BUILDS = {'华测构建': '华测驱动与消息包构建完成', 'FSD构建': 'FSD 算法包构建完成',
          '可视化构建': 'fsd-visualization-code 可视化与安全图像插件构建完成'}


class RuntimeChecks:
    def __init__(self, options):
        labels = dict(re.findall(r"^\s*(\w+)\) echo '([^']+)' ;;", SOURCE.read_text(), re.M))
        self.labels = {name: labels.get(name, name) for name in PROCESS_NAMES}
        self.rows = OrderedDict()
        self.options = options
        self.begun = False
        self.add('sudo', '权限', 'sudo 认证')
        for key in BUILDS:
            self.add('build:' + key, '构建', key)
        self.add('build:zlg', '构建', 'ZLG CAN 桥构建')
        self.add('environment', '环境', 'ROS 与工作空间环境加载')
        self.add('can_ready', '硬件', 'CAN 设备初始化与 vcan/UP')
        for name, label in self.labels.items():
            self.add('process:' + name, '进程', label)
        self.add('observer', '监控', '只读任务监控启动')
        for topic in TOPICS:
            self.add('topic:' + topic, '数据', topic)
        self.add('complete', '汇总', '全栈启动检查完成')
        sensor_only = 'sensor-only' in options
        build_only = 'build-only' in options
        if not build_only:
            self.skip('build:zlg', '正常启动由 ZLG 桥子脚本按需构建，设备初始化项验证最终结果')
        if sensor_only:
            for name in FULL_ONLY:
                self.skip('process:' + name, 'sensor-only 不启动此模块')
            for topic in ('定位输出', '任务状态广播'):
                self.skip('topic:' + topic, 'sensor-only 不检查此话题')
            self.skip('observer', 'sensor-only 不启动任务监控')
        if 'reuse-ins' in options:
            self.skip('process:chcnav', '复用外部惯导；数据输出仍由原脚本检查')
        if not options.get('_rviz', 'no-rviz' not in options) and not build_only:
            self.skip('process:visualization', '--no-rviz 不启动可视化')
            self.skip('build:可视化构建', '--no-rviz 不构建可视化')
        if build_only:
            for key in self.rows:
                if not key.startswith('build:') and key != 'complete':
                    self.skip(key, '--build-only 不启动硬件和运行节点')

    def add(self, key, phase, label):
        self.rows[key] = {'phase': phase, 'label': label, 'status': '待启动验证',
                          'ok': None, 'detail': '原脚本执行到此项后才会验证；预检查通过不代表此项通过'}

    def set(self, key, status, detail, ok=None):
        if key in self.rows and self.rows[key]['status'] != '跳过':
            self.rows[key].update(status=status, detail=detail, ok=ok)

    def skip(self, key, reason):
        self.rows[key].update(status='跳过', detail=reason, ok=None)

    def consume(self, line):
        if line.startswith('[启动] '):
            for name, label in self.labels.items():
                if line.startswith('[启动] ' + label + ' ...'):
                    self.set('process:' + name, '启动中', line)
        for name, label in self.labels.items():
            if line.startswith('[成功] ' + label + '：进程已启动'):
                self.set('process:' + name, '进程已启动', line, True)
            if line.startswith('[失败] ' + label):
                self.set('process:' + name, '失败', line, False)
        for label in TOPICS:
            key = 'topic:' + label
            if line.startswith('[检查] ' + label + '：等待 '):
                self.set(key, '等待数据', line)
            if line.startswith('[成功] ' + label + '：已收到 '):
                self.set(key, '已收到消息', line, True)
            if line.startswith('[失败] ' + label + '：'):
                self.set(key, '失败', line, False)
        for key, message in BUILDS.items():
            if line == '[成功] ' + message:
                self.set('build:' + key, '构建完成', line, True)
        if line.startswith('[成功] ROS 与硬件、算法工作空间环境加载完成'):
            self.set('environment', '环境已加载', line, True)
            for key in BUILDS:
                if self.rows['build:' + key]['ok'] is None:
                    self.set('build:' + key, '沿用安装产物', '原脚本加载安装环境；未执行本次构建', True)
        if line.startswith('[成功] CAN 盒已重新打开并初始化'):
            self.set('can_ready', '设备初始化通过', line, True)
        elif line.startswith('[重连]') or line.startswith('[启动] ' + self.labels['zlg_can']):
            self.set('can_ready', '初始化中', line)
        if line.startswith('[失败]') and any(word in line for word in ('CAN 桥', 'CAN 盒', '旧 CAN 桥', 'ZLG CAN 桥')):
            self.set('can_ready', '失败', line, False)
        if line.startswith('[成功] 只读任务监控已启动'):
            self.set('observer', '监控已启动', line, True)
        if line.startswith('[警告]') and '只读任务监控' in line:
            self.set('observer', '警告', line)
        if line.startswith('[成功] 全栈启动检查完成'):
            self.set('complete', '启动检查通过', line, True)
        if line.startswith('[成功] 华测、FSD、ZLG CAN 桥构建全部完成'):
            self.set('build:zlg', '构建完成', line, True)
            self.set('complete', '仅构建完成', line, True)

    def stopped(self, code):
        for key, item in self.rows.items():
            if key.startswith('process:') and item['ok'] is True:
                self.set(key, '已停止', item['detail'] + '\n本次运行已停止；以上是启动时的验证记录')
            elif item['status'] in ('启动中', '等待数据', '初始化中'):
                self.set(key, '未完成', item['detail'] + f'\n原启动器结束，退出码={code}')
        if self.rows['complete']['ok'] is None and self.rows['complete']['status'] != '跳过':
            self.set('complete', '启动未完成', f'原启动器结束，退出码={code}；未完成全部启动检查', False if code else None)
