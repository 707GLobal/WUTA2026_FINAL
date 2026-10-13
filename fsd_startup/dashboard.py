#!/usr/bin/env python3
"""Read-only text dashboard; no image, point-cloud or trajectory rendering."""
import argparse
from collections import deque
import json
from pathlib import Path
import sys
import threading
import time
import traceback

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
from mission_diagnostics import describe
from gnss_status import gnss_description, gnss_allows_start


def message_summary(key, msg):
    if key == 'GNSS':
        code = int(msg.stat[1])
        return f'GNSS 状态码={code}（{gnss_description(code)}）；' + (
            '定位门控通过：允许下一阶段' if gnss_allows_start(code) else
            '惯导原始数据在线，持续等待状态4；不判设备自检失败')
    if key == '惯导':
        p, v = msg.pose.pose.position, msg.twist.twist.linear
        return f'位置(m) x={p.x:.3f} y={p.y:.3f} z={p.z:.3f}；速度(m/s) x={v.x:.3f} y={v.y:.3f}'
    if key == '雷达':
        return f'帧={msg.header.frame_id} 点数={msg.width * msg.height} 字节={len(msg.data)}'
    if key == '相机':
        return f'帧={msg.header.frame_id} 分辨率={msg.width}×{msg.height} 编码={msg.encoding}'
    if key == '融合':
        return f'帧={msg.header.frame_id} 锥桶数量={len(msg.cones)}'
    if key == '控制请求':
        return f'speed={msg.speed:.3f} angle={msg.angle:.3f} throttle_brake={msg.throttle_brake:.3f}（请求值）'
    if key == '设备自检':
        return '通过' if msg.ok else '失败：' + '、'.join(msg.failures)
    if key == '任务':
        return '\n'.join(f'{k}：{v}' for k, v in describe(msg.state, msg.mission_mode, msg.localization_mode, msg.description).items())
    if key == '模式选择':
        return msg.data
    return str(msg.data)


class Telemetry:
    def __init__(self, state_path):
        self.state_path = state_path
        self.lock = threading.Lock()
        self.samples = {}
        self.error = ''
        self.quit = threading.Event()
        self.thread = threading.Thread(target=self.spin, daemon=True)
        self.thread.start()

    def record(self, key, msg):
        now = time.monotonic()
        with self.lock:
            item = self.samples.setdefault(key, {'times': deque(maxlen=1000)})
            item['times'].append(now)
            item['received'] = now
            item['detail'] = message_summary(key, msg)

    def lines(self, keys):
        now = time.monotonic()
        lines = []
        with self.lock:
            if self.error:
                lines += ['[错误] ROS监控：' + self.error, '']
            for key in keys:
                item = self.samples.get(key)
                if not item:
                    lines += [f'[等待] {key}：尚未收到消息', '']
                    continue
                age = now - item['received']
                times = [t for t in item['times'] if now - t <= 5]
                hz = (len(times) - 1) / (times[-1] - times[0]) if len(times) > 1 and times[-1] > times[0] else 0
                lines += [f'[{"超时" if age > 3 else "在线"}] {key}：接收频率 {hz:.1f} Hz，距末帧 {age:.1f}s',
                          item['detail'], '']
        return '\n'.join(lines)

    def spin(self):
        node = None
        try:
            import rclpy
            from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
            from rosidl_runtime_py.utilities import get_message
            rclpy.init()
            node = rclpy.create_node('fsd_startup_readonly_dashboard')
            from checks import configuration
            arguments = json.loads(Path(self.state_path).read_text()).get('args', [])
            options = {arg[2:].split('=', 1)[0]: arg.split('=', 1)[1] if '=' in arg else True
                       for arg in arguments if arg.startswith('--')}
            settings = configuration(options)
            deployment = settings['deployment']
            raw_topic = settings['parameters']['mission'].get('ins_heartbeat_topic', '/chcnav/devpvt')
            topics = [
                ('任务', '/system/mission_state', 'wuta_msgs/msg/MissionState'),
                ('设备自检', '/system/devices_inspection', 'wuta_msgs/msg/DevicesInspection'),
                ('模式选择', '/system/mission_mode_cmd', 'std_msgs/msg/String'),
                ('GO广播', '/system/start_command', 'std_msgs/msg/Bool'),
                ('急停广播', '/system/emergency', 'std_msgs/msg/Bool'),
                ('控制请求', '/control/command', 'autoware_msgs/msg/Command'),
                ('惯导', deployment.get('ins_topic', '/chcnav/odometry'), 'nav_msgs/msg/Odometry'),
                ('GNSS', raw_topic, 'msg_interfaces/msg/Hcinspvatzcb'),
                ('雷达', deployment.get('lidar_topic', '/rslidar_points'), 'sensor_msgs/msg/PointCloud2'),
                ('相机', deployment.get('image_topic', '/zed/zed_node/rgb/image_rect_color'), 'sensor_msgs/msg/Image'),
                ('融合', deployment.get('fusion_topic', '/perception/fused/cones'), 'wuta_msgs/msg/ConeArray'),
            ]
            subscriptions = []
            for key, topic, kind in topics:
                qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
                if key == '急停广播':
                    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
                subscriptions.append(node.create_subscription(get_message(kind), topic,
                                     lambda msg, key=key: self.record(key, msg), qos))
            while not self.quit.is_set():
                rclpy.spin_once(node, timeout_sec=.2)
        except Exception:
            with self.lock:
                self.error = traceback.format_exc()
        finally:
            if node:
                node.destroy_node()
            if 'rclpy' in locals() and rclpy.ok():
                rclpy.shutdown()


def tail(path, limit=40000):
    try:
        with Path(path).open('rb') as stream:
            stream.seek(0, 2)
            stream.seek(max(0, stream.tell() - limit))
            return stream.read().decode('utf-8', errors='replace')
    except OSError as exc:
        return f'[等待] {path}\n{exc}'


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--state', required=True)
    parser.add_argument('--telemetry-json', help='只读后台采集，供终端界面读取')
    args = parser.parse_args()
    if args.telemetry_json:
        import signal
        telemetry = Telemetry(args.state)
        signal.signal(signal.SIGINT, lambda *_: telemetry.quit.set())
        signal.signal(signal.SIGTERM, lambda *_: telemetry.quit.set())
        destination = Path(args.telemetry_json)
        try:
            while not telemetry.quit.is_set():
                data = {'updated': time.time(),
                        'vehicle': telemetry.lines(['任务', '设备自检', '模式选择', 'GO广播', '急停广播', '控制请求']),
                        'sensors': telemetry.lines(['GNSS', '惯导', '雷达', '相机', '融合'])}
                temporary = destination.with_suffix('.tmp')
                temporary.write_text(json.dumps(data, ensure_ascii=False), encoding='utf-8')
                temporary.replace(destination)
                telemetry.quit.wait(.5)
        finally:
            telemetry.quit.set()
            telemetry.thread.join(timeout=2)
        return
    import tkinter as tk
    from tkinter import ttk
    window = tk.Tk()
    window.title('FSD 实时日志 · 只读状态与传感器诊断')
    window.geometry('1080x720')
    window.configure(bg='#101820')
    style = ttk.Style(window)
    style.theme_use('clam')
    style.configure('TFrame', background='#101820')
    toolbar = ttk.Frame(window)
    toolbar.pack(fill='x', padx=10, pady=10)
    view = tk.StringVar(value='启动日志')
    selector = ttk.Combobox(toolbar, textvariable=view, state='readonly', width=20,
                            values=['启动日志', '车辆状态', '传感器参数', '模块配置', '组件日志'])
    selector.pack(side='left')
    component = tk.StringVar()
    files = ttk.Combobox(toolbar, textvariable=component, state='readonly', width=30)
    files.pack(side='left', padx=10)
    paused = tk.BooleanVar(value=False)
    ttk.Checkbutton(toolbar, text='暂停滚动', variable=paused).pack(side='left')
    status = tk.StringVar()
    tk.Label(window, textvariable=status, bg='#101820', fg='#66d9ef', anchor='w').pack(fill='x', padx=10)
    body = ttk.Frame(window)
    body.pack(fill='both', expand=True, padx=10, pady=10)
    text = tk.Text(body, bg='#101820', fg='#e9eef2', insertbackground='white', wrap='word', font=('Monospace', 11))
    scroll = ttk.Scrollbar(body, command=text.yview)
    text.configure(yscrollcommand=scroll.set)
    scroll.pack(side='right', fill='y')
    text.pack(fill='both', expand=True)
    for tag, color in [('error', '#ff6b6b'), ('warn', '#ffd166'), ('ok', '#8be9a8'), ('info', '#66d9ef')]:
        text.tag_configure(tag, foreground=color)
    telemetry = Telemetry(args.state)
    previous = ['']

    def refresh():
        try:
            state = json.loads(Path(args.state).read_text())
            status.set('管理状态：' + state['phase'] + '  |  只读监控；不发送模式、GO或控制指令')
            folder = Path(state.get('module_log_dir') or state['log_dir']) if state.get('log_dir') else None
            available = sorted(p.name for p in folder.glob('*.log')) if folder else []
            files['values'] = available
            if available and component.get() not in available:
                component.set(available[0])
            selected = view.get()
            if selected == '车辆状态':
                content = telemetry.lines(['任务', '设备自检', '模式选择', 'GO广播', '急停广播', '控制请求'])
                content += '\nCAN 原始收发与 VCU->FSD 计数见“组件日志 / zlg_can.log”。\n'
                content += 'GO与模式广播为最近事件，实际门控以任务状态诊断为准。\n'
            elif selected == '传感器参数':
                content = telemetry.lines(['GNSS', '惯导', '雷达', '相机', '融合'])
                content += '\n频率为窗口订阅端最近5秒的接收统计；不等同于设备标称频率。\n'
            elif selected == '模块配置':
                content = tail(ROOT / '.runtime/fsd_manager/manifest.json', 200000)
                content += '\n\n并行检查结果：\n' + json.dumps(state.get('checks', {}), ensure_ascii=False, indent=2)
            elif selected == '组件日志':
                content = tail(folder / component.get()) if folder and component.get() else '[等待] 尚无运行组件日志'
            else:
                content = tail(state['log'])
            if not paused.get() and content != previous[0]:
                text.configure(state='normal')
                text.delete('1.0', 'end')
                for line in content.splitlines(True):
                    tag = 'error' if any(s in line.lower() for s in ['失败', '错误', 'error', 'traceback', 'fatal']) else (
                        'warn' if any(s in line.lower() for s in ['超时', '等待', 'warn']) else 'ok' if any(s in line for s in ['成功', '在线', '通过']) else 'info')
                    text.insert('end', line, tag)
                if selected in ('启动日志', '组件日志'):
                    text.see('end')
                text.configure(state='disabled')
                previous[0] = content
        except Exception as exc:
            status.set(f'等待管理状态：{exc}')
        window.after(500, refresh)

    def close():
        telemetry.quit.set()
        telemetry.thread.join(timeout=1)
        window.destroy()

    window.protocol('WM_DELETE_WINDOW', close)
    refresh()
    window.mainloop()


if __name__ == '__main__':
    main()
