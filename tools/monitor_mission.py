#!/usr/bin/env python3
"""Read-only console monitor for the full FSD launcher."""
import argparse
import json
import time
from datetime import datetime

import os
from pathlib import Path

from mission_diagnostics import describe


class Console:
    def __init__(self):
        self.previous = {}

    def emit(self, text):
        line = f"[状态 {datetime.now().astimezone().strftime('%H:%M:%S')}] {text}"
        print(line, flush=True)

    def changed(self, key, value):
        if self.previous.get(key) != value:
            self.previous[key] = value
            self.emit(f'{key}：{value}')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--run-dir', default=os.environ.get('FSD_RUN_DIR'))
    args = parser.parse_args()
    console = Console()
    run_dir = Path(args.run_dir) if args.run_dir else None
    names = {0: 'trackdrive', 1: 'skidpad', 2: 'acceleration',
             3: 'inspection', 4: 'ebs_test'}
    reported_mismatch = set()

    def record_fallback_mission(mission):
        """Only used when the run started in pending because no VCU cache existed."""
        if run_dir is None:
            return
        path = run_dir / 'run.json'
        try:
            data = json.loads(path.read_text())
            current = data.get('mission')
        except (OSError, ValueError, AttributeError):
            return
        if data.get('mode') == 'manual_record':
            return  # 人工记录目录不按默认赛项重新归类
        if current == mission:
            return
        if current:
            if mission not in reported_mismatch:
                console.emit(f'本次日志已归为 {current}，忽略后续赛项 {mission}')
                reported_mismatch.add(mission)
            return
        data['mission'] = mission
        data['mission_source'] = 'fallback_observed'
        data['mission_observed_at'] = datetime.now().astimezone().isoformat()
        try:
            temporary = path.with_suffix('.tmp')
            temporary.write_text(json.dumps(data, ensure_ascii=False, indent=2) + '\n')
            temporary.replace(path)
            console.emit(f'补记赛项：{mission}')
        except OSError as exc:
            console.emit(f'赛项补记失败：{exc}')
    # Import here so formatting logic can be tested without ROS or hardware.
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import ExternalShutdownException
    from wuta_msgs.msg import MissionState, DevicesInspection

    class Monitor(Node):
        def __init__(self):
            super().__init__('full_fsd_mission_monitor')
            self.last_state = None
            self.started = time.monotonic()
            self.last_go = None
            self.sub = self.create_subscription(MissionState, '/system/mission_state', self.on_state, 10)
            self.inspection = self.create_subscription(
                DevicesInspection, '/system/devices_inspection', self.on_inspection, 10)
            self.timer = self.create_timer(1.0, self.tick)
            console.emit('只读任务监控已启动，等待状态机广播；不会发送模式、GO或控制命令')

        def on_state(self, msg):
            self.last_state = time.monotonic()
            tokens = dict(token.split('=', 1) for token in msg.description.split() if '=' in token)
            if tokens.get('mode_selected') == '1':
                mission = names.get(msg.mission_mode)
                if mission:
                    record_fallback_mission(mission)

            console.changed('状态广播', '在线')
            # The counter proves receipt even if GO was consumed between broadcasts.
            tokens = dict(token.split('=', 1) for token in msg.description.split() if '=' in token)
            try:
                count = int(tokens['go_count'])
            except (KeyError, ValueError):
                count = None
            if count is not None:
                if self.last_go is not None and count < self.last_go:
                    console.emit('GO计数已重置，任务机可能已重启')
                self.last_go = count
            for key, value in describe(msg.state, msg.mission_mode, msg.localization_mode, msg.description).items():
                console.changed(key, value)

        def on_inspection(self, msg):
            console.changed('设备自检', '通过' if msg.ok else '失败：' + '、'.join(msg.failures))

        def tick(self):
            if time.monotonic() - (self.last_state or self.started) > 3.0:
                console.changed('状态广播', '超过3秒未收到，请检查任务机进程与DDS连接')

    rclpy.init()
    node = None
    try:
        node = Monitor()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as exc:
        console.emit(f'监控失败：{type(exc).__name__}: {exc}')
        raise
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
