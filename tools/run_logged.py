#!/usr/bin/env python3
"""Standalone output owner; keep stdin interactive and preserve child exit codes."""
import argparse
import os
from pathlib import Path
import signal
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'fsd_startup'))
from log_context import RunLog, append_manager_event
import mode_cache


def main():
    command = sys.argv[1:]
    mode = os.environ.get('FSD_RUN_MODE', 'hardware')
    mode = 'manual_record' if '--manual-record' in command else 'brake_bench' if '--brake-bench' in command else 'sensor_only' if '--sensor-only' in command else mode
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument('--mission')
    parser.add_argument('--output')
    parser.add_argument('--can-config')
    options, _ = parser.parse_known_args(command)
    mission = options.mission if mode == 'simulation' else None
    mission_source = 'cli' if mission else None
    vcu_mode = None
    if mode == 'hardware':
        config = options.can_config or (
            ROOT / 'WUTA-FSD/ros2_ws/src/system/can_interface/config/can_interface.yaml')
        info, _ = mode_cache.read_mission(config)
        if info:
            mission = info['mission']
            mission_source = 'vcu_mode_cache'
            vcu_mode = info['vcu_mode']
        else:
            mission_source = 'fallback_pending'
            print('[日志警告] 未读到有效 VCU 模式缓存，本次日志先落 pending。', file=sys.stderr)
    run = RunLog.create(ROOT, mode=mode, args=command, mission=mission,
                        mission_source=mission_source, vcu_mode=vcu_mode,
                        path=options.output if mode == 'simulation' else None)
    environment = dict(os.environ, FSD_RUN_DIR=str(run.path), FSD_LOG_OWNER='collector')
    with (run.path / 'manager.log').open('a', buffering=1) as log:
        child = subprocess.Popen(command, env=environment, stdout=subprocess.PIPE,
                                 stderr=subprocess.STDOUT, text=True, errors='replace',
                                 start_new_session=False)
        def forward(signum, _frame):
            # Foreground terminal SIGINT already reaches the child group.
            # Keep its controlling terminal for the existing sudo flow.
            if signum == signal.SIGINT and sys.stdin.isatty():
                return
            if child.poll() is None:
                child.send_signal(signum)
        signal.signal(signal.SIGINT, forward)
        signal.signal(signal.SIGTERM, forward)
        for line in child.stdout:
            log.write(line)
            append_manager_event(run.path / 'manager.log', line, 'info')
            try:
                print(line, end='', flush=True)
            except BrokenPipeError:
                pass
        code = child.wait()
        ended = f'[管理] 运行结束，退出码={code}\n'
        log.write(ended)
        append_manager_event(run.path / 'manager.log', ended, 'error' if code else 'info')
    try:
        path = run.finish(code, 'standalone_exit')
        print(f'本次日志目录：{path}', flush=True)
    except Exception as exc:
        # Archive errors must not mask the runtime exit status or remove records.
        print(f'[日志警告] 归档未完成，原目录保留：{run.path}：{exc}', file=sys.stderr)
    return code if code >= 0 else 128 - code


if __name__ == '__main__':
    raise SystemExit(main())
