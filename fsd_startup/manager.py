#!/usr/bin/env python3
"""Interactive launcher. Runtime failures return to the menu, never auto-restart control."""
import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
from datetime import datetime
import fcntl
import json
import os
from pathlib import Path
import queue
import re
import select
import signal
import stat
import subprocess
import sys
import threading
import time
import traceback

from checks import CHECKS, configuration
from processes import cleanup, descendants, identity, matches, send
from privilege import sudo_command, sudo_environment
from log_context import RunLog, new_directory
from runtime_checks import RuntimeChecks

ROOT = Path(__file__).resolve().parents[1]
RUNTIME = ROOT / '.runtime/fsd_manager'
COLORS = {'ok': '\033[38;2;166;227;161m', 'error': '\033[38;2;243;139;168m',
          'warn': '\033[38;2;249;226;175m', 'info': '\033[38;2;137;180;250m',
          'title': '\033[1;38;2;203;166;247m'}


def save(path, data):
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(data, ensure_ascii=False, indent=2))
    temporary.replace(path)


def options_from(args):
    options = {arg[2:].split('=', 1)[0]: arg.split('=', 1)[1] if '=' in arg else True
               for arg in args if arg.startswith('--')}
    options['_rviz'] = True
    options['_build'] = 'auto'
    for arg in args:
        if arg in ('--rviz', '--image-view', '--no-rviz'):
            options['_rviz'] = arg != '--no-rviz'
        if arg in ('--build', '--skip-build'):
            options['_build'] = 'yes' if arg == '--build' else 'no'
    return options


class Manager:
    def __init__(self, args, color=True):
        self.args = args
        self.options = options_from(args)
        if 'build-only' in self.options and 'skip-build' not in self.options and 'build' not in self.options:
            self.args = [*args, '--build']
        self.color = color
        self.child = None
        self.dashboard = None
        self.records = {}
        self.events = queue.Queue()
        self.phase = '等待操作'
        self.log_dir = ''
        folder = new_directory(ROOT / 'logs/sessions')
        self.session_log_path = folder / 'manager.log'
        self.run_log = None
        self.output_thread = None
        self.runtime_exit_code = None
        self.log = (folder / 'manager.log').open('a', buffering=1)
        self.log_path = folder / 'manager.log'
        self.state_path = RUNTIME / 'state.json'
        self.previous_run_dir = None
        if self.state_path.exists():
            try:
                previous = json.loads(self.state_path.read_text())
                candidate = Path(previous.get('log_dir') or '/')
                if candidate.is_relative_to(ROOT / 'logs') and (candidate / 'run.json').exists():
                    self.previous_run_dir = candidate
            except (OSError, ValueError):
                pass
        self.registry_path = RUNTIME / 'processes.json'
        self.results = {}
        self.failures = []
        self.runtime_checks = RuntimeChecks(self.options)
        self.ui_events = queue.Queue()
        self.silent = False
        self.publish()

    def emit(self, text, level='info'):
        line = f'[{datetime.now().astimezone().strftime("%H:%M:%S")}] {text}'
        if not self.silent:
            print((COLORS[level] if self.color else '') + line + ('\033[0m' if self.color else ''), flush=True)
        self.ui_events.put(('log', (line, level)))
        self.log.write(line + '\n')

    def publish(self):
        state = {'manager': identity(os.getpid()), 'phase': self.phase,
                              'log': str(self.log_path), 'log_dir': self.log_dir,
                              'module_log_dir': str(Path(self.log_dir) / 'modules') if self.log_dir else '',
                              'run_id': self.run_log.data['run_id'] if self.run_log else None,
                              'checks': dict(self.results),
                              'runtime_checks': {key: dict(item) for key, item in self.runtime_checks.rows.items()},
                              'failures': list(self.failures), 'args': self.args}
        save(self.state_path, state)
        self.ui_events.put(('state', state))

    def checks(self):
        self.emit('并行预检查：' + ' / '.join(CHECKS), 'title')
        self.results = {}
        self.phase = '并行检查中'
        self.publish()
        with ThreadPoolExecutor(max_workers=len(CHECKS)) as pool:
            jobs = {pool.submit(function, self.options): name for name, function in CHECKS.items()}
            for job in as_completed(jobs):
                name = jobs[job]
                try:
                    result = job.result()
                    if isinstance(result, dict):
                        save(RUNTIME / 'manifest.json', result)
                        if self.run_log is not None:
                            try:
                                self.run_log.snapshot(result['files'])
                                save(self.run_log.path / 'configs/module_config_manifest.json', result)
                            except OSError as exc:
                                self.emit(f'配置快照未完整保存：{exc}', 'warn')
                        result = json.dumps(result['deployment'], ensure_ascii=False)
                    status = '通过'
                    if (name == 'RViz部署与桌面' and not self.options.get('_rviz', True)) or (
                            name == '只读任务监控文件' and 'sensor-only' in self.options):
                        status = '跳过'
                    elif str(result).startswith('警告'):
                        status = '警告'
                    elif name == '工作空间产物' and str(result).startswith(('需自动构建', '指定强制构建')):
                        status = '待构建'
                    self.results[name] = {'ok': True, 'status': status, 'detail': result}
                    self.emit(f'[{"成功" if status == "通过" else status}] {name}：{result}',
                              'ok' if status == '通过' else 'warn')
                except Exception:
                    detail = traceback.format_exc()
                    self.results[name] = {'ok': False, 'detail': detail}
                    self.emit(f'[失败] {name}\n{detail}', 'error')
                self.publish()
        ok = all(item['ok'] for item in self.results.values())
        self.phase = '预检查通过' if ok else '预检查失败：可修复后重试'
        self.publish()
        return ok

    def permissions(self):
        self.emit('认证 sudo：用于 CAN 初始化及必要的华测串口 ACL', 'info')
        command = sudo_command('-v')
        authentication = subprocess.run(command, env=sudo_environment(), timeout=15,
                                        capture_output=True, text=True)
        if authentication.returncode:
            self.runtime_checks.set('sudo', '失败', authentication.stderr.strip(), False)
            self.emit(f'sudo 认证失败，退出码 {authentication.returncode}：{authentication.stderr.strip()}。可输入 p 重试。', 'error')
            return False
        self.runtime_checks.set('sudo', '认证通过', 'sudo 自动认证通过；CAN初始化和串口权限处理使用此权限', True)
        if 'reuse-ins' in self.options:
            return True
        port = self.options.get('ins-port', os.environ.get('INS_PORT', '/dev/ttyUSB0'))
        device = Path(port).resolve(strict=True)
        if not stat.S_ISCHR(device.stat().st_mode):
            raise ValueError(f'{device} 不是串口字符设备')
        if not os.access(device, os.R_OK | os.W_OK):
            command = sudo_command('setfacl', '-m', f'u:{os.getuid()}:rw', '--', str(device))
            result = subprocess.run(command, env=sudo_environment(), capture_output=True, text=True, timeout=10)
            if result.returncode or not os.access(device, os.R_OK | os.W_OK):
                self.emit(f'串口 ACL 设置失败：{command!r}\n{result.stdout}\n{result.stderr}', 'error')
                return False
            self.emit(f'已授予当前用户 {device} 读写 ACL（设备重插后可能需重设）', 'ok')
        else:
            self.emit(f'华测串口权限已满足：{device}', 'ok')
        return True

    def remember(self, pid):
        for item in descendants(pid):
            self.records[(item['pid'], item['start'])] = item
        save(self.registry_path, list(self.records.values()))

    def recover(self):
        stored = json.loads(self.registry_path.read_text()) if self.registry_path.exists() else []
        # Expand still-live saved parents before cleaning a previous crashed session.
        for item in stored:
            if matches(item):
                self.records[(item['pid'], item['start'])] = item
                self.remember(item['pid'])
        if self.records:
            self.emit('发现本启动包上次会话残留，将按 PID 与启动时间清理', 'warn')
            if not self.stop(include_dashboard=True):
                return
        if self.previous_run_dir is not None:
            try:
                data = json.loads((self.previous_run_dir / 'run.json').read_text())
                if data.get('ended_at') is None:
                    run = RunLog(ROOT, self.previous_run_dir, data)
                    self.log_dir = str(run.finish(1, 'recovered_after_manager_exit'))
                    self.emit(f'上次中断运行已标记并归档：{self.log_dir}', 'warn')
            except Exception:
                self.emit('上次运行归档未完成，保留原目录：\n' + traceback.format_exc(), 'warn')
            self.previous_run_dir = None
            self.publish()

    def read_output(self, child):
        for line in child.stdout:
            self.events.put(line.rstrip('\n'))

    def use_log(self, path):
        path = Path(path)
        stream = path.open('a', buffering=1)
        self.log.close()
        self.log_path = path
        self.log = stream

    def finish_run(self, code, reason):
        if self.run_log is None:
            return
        self.drain_events()
        if reason in ('precheck_failed', 'startup_exception'):
            self.runtime_checks.stopped(code)
        self.emit(f'[管理] 本次运行结束：{reason}，退出码={code}')
        self.log.close()
        try:
            self.log_dir = str(self.run_log.finish(code, reason))
        except Exception:
            self.log_dir = str(self.run_log.path)
            with (self.run_log.path / 'manager.log').open('a') as stream:
                stream.write('[日志警告] 归档失败，保留原目录：\n' + traceback.format_exc())
        self.run_log = None
        self.runtime_exit_code = None
        self.log_path = self.session_log_path
        self.log = self.log_path.open('a', buffering=1)
        self.emit(f'运行记录：{self.log_dir}')
        self.publish()

    def drain_events(self):
        while not self.events.empty():
            line = self.events.get_nowait()
            # Replace the legacy static hint at the manager presentation boundary.
            if line == '[提示] 任务仍需满足就绪条件、由 VCU 选择模式，再按 RES GO。':
                line = '[提示] 当前任务模式与尚未满足的启动条件，请查看任务监控最新的“任务选择”和“下一步”；全栈启动完成不代表车辆已获准发车。'
            self.runtime_checks.consume(line)
            level = 'error' if re.search(r'失败|ERROR|Traceback|FATAL', line, re.I) else (
                'ok' if '[成功]' in line else 'warn' if re.search(r'WARN|警告', line, re.I) else 'info')
            self.emit(line, level)
            if '全栈启动检查完成' in line:
                self.phase = '运行中'

    def start(self):
        try:
            self.start_runtime()
        except Exception:
            self.emit('启动操作失败：\n' + traceback.format_exc(), 'error')
            if self.child is None and self.run_log is not None:
                self.finish_run(1, 'startup_exception')
            self.phase = '启动失败：可修复后重试'
            self.publish()

    def start_runtime(self):
        if self.child and self.child.poll() is None:
            self.emit('当前启动流程仍在运行；先输入 x 停止后再重试。', 'warn')
            return
        window_pids = {item['pid'] for item in descendants(self.dashboard.pid)} if (
            self.dashboard and self.dashboard.poll() is None) else set()
        if any(matches(item) for item in self.records.values() if item['pid'] not in window_pids):
            self.emit('存在待清理的已登记进程，先输入 x 清理。', 'error')
            return
        if self.run_log is not None:
            self.emit('上一轮日志尚未结束，请先停止并清理。', 'warn')
            return
        self.runtime_checks = RuntimeChecks(self.options)
        self.runtime_checks.begun = True
        mode = 'sensor_only' if 'sensor-only' in self.options else 'hardware'
        self.output_thread = None
        self.runtime_exit_code = None
        self.run_log = RunLog.create(ROOT, mode=mode, args=self.args)
        self.log_dir = str(self.run_log.path)
        self.use_log(self.run_log.path / 'manager.log')
        self.publish()
        if 'build-only' not in self.options:
            if not self.permissions() or not self.checks():
                self.phase = '预检查失败：可修复后重试'
                self.finish_run(1, 'precheck_failed')
                return
        self.phase = '构建中' if 'build-only' in self.options else '启动中'
        # Keep terminal stdin for sudo authentication; stdout/stderr go to the event log.
        environment = dict(os.environ, FSD_PARALLEL_CHECKS='1',
                           FSD_RUN_DIR=str(self.run_log.path), FSD_LOG_OWNER='manager')
        self.child = subprocess.Popen(['bash', str(ROOT / 'start_full_fsd.sh'), *self.args], env=environment,
                                      stdin=subprocess.DEVNULL if self.silent else None,
                                      stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                      text=True, errors='replace', start_new_session=True)
        self.remember(self.child.pid)
        self.output_thread = threading.Thread(target=self.read_output, args=(self.child,), daemon=True)
        self.output_thread.start()
        self.publish()
        self.emit('启动任务已提交；失败后保留管理菜单，不自动重启控制器。', 'ok')

    def window(self):
        if self.dashboard and self.dashboard.poll() is None:
            self.emit('实时日志窗口已打开。', 'info')
            return
        if not (os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')):
            self.emit('没有图形桌面，保留彩色终端；可用 l 查看详细日志。', 'warn')
            return
        self.dashboard = subprocess.Popen(['bash', str(ROOT / 'fsd_startup/window.sh'), str(self.state_path)],
                                          stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, errors='replace',
                                          start_new_session=True)
        threading.Thread(target=self.read_output, args=(self.dashboard,), daemon=True).start()
        self.remember(self.dashboard.pid)
        self.emit('已打开实时日志窗口：车辆状态 / 传感器参数 / 模块配置 / 组件日志。', 'ok')

    def telemetry(self):
        """Keep Textual dependencies separate from ROS subscriber imports."""
        if self.dashboard and self.dashboard.poll() is None:
            return
        (RUNTIME / 'telemetry.json').unlink(missing_ok=True)
        self.dashboard = subprocess.Popen(
            ['bash', str(ROOT / 'fsd_startup/window.sh'), str(self.state_path),
             '--telemetry-json', str(RUNTIME / 'telemetry.json')],
            stdin=subprocess.DEVNULL, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            text=True, errors='replace', start_new_session=True)
        threading.Thread(target=self.read_output, args=(self.dashboard,), daemon=True).start()
        self.remember(self.dashboard.pid)

    def pump(self):
        self.drain_events()
        if self.child:
            if self.child.poll() is None:
                self.remember(self.child.pid)
            else:
                code = self.child.wait()
                self.runtime_exit_code = code
                self.child = None
                outcome = f'启动任务结束，退出码 {code}；菜单仍可操作'
                self.emit(outcome, 'ok' if code == 0 else 'error')
                if code:
                    self.failures.append(outcome)
                self.stop()
                self.phase = outcome
                self.menu()
        if self.dashboard and self.dashboard.poll() is None:
            self.remember(self.dashboard.pid)
        self.publish()

    def stop(self, include_dashboard=False):
        if self.child and self.child.poll() is None:
            self.remember(self.child.pid)
            send([identity(self.child.pid)], signal.SIGINT)
            deadline = time.monotonic() + 12
            while self.child.poll() is None and time.monotonic() < deadline:
                self.remember(self.child.pid)
                time.sleep(.2)
        window_pids = {item['pid'] for item in descendants(self.dashboard.pid)} if (
            self.dashboard and self.dashboard.poll() is None) else set()
        records = [item for item in self.records.values() if include_dashboard or item['pid'] not in window_pids]
        remaining, errors = cleanup(records)
        kept = [item for item in self.records.values() if not include_dashboard and item['pid'] in window_pids and matches(item)]
        self.records = {(item['pid'], item['start']): item for item in remaining + kept}
        save(self.registry_path, list(self.records.values()))
        if self.child and self.child.poll() is not None:
            self.runtime_exit_code = self.child.wait()
            self.child = None
        if include_dashboard and self.dashboard and self.dashboard.poll() is not None:
            self.dashboard.wait()
            self.dashboard = None
        if remaining:
            self.emit(f'残留未能清除：{remaining}\n' + '\n'.join(errors), 'error')
            self.phase = '清理未完成；请 p 认证后重试 x / q'
        else:
            self.emit('本启动包登记的运行进程已清理。', 'ok')
            self.phase = '已停止'
        if not remaining and self.child is None and self.run_log is not None:
            if self.output_thread is not None:
                self.output_thread.join(timeout=2)
            if self.output_thread is None or not self.output_thread.is_alive():
                self.drain_events()
                self.runtime_checks.stopped(self.runtime_exit_code if self.runtime_exit_code is not None else 0)
                self.finish_run(self.runtime_exit_code if self.runtime_exit_code is not None else 0,
                                'runtime_stopped')
            else:
                self.emit('输出仍在收集，暂缓日志归档；再次停止可重试。', 'warn')
        self.publish()
        return not remaining

    def menu(self):
        self.emit('s 启动/重试 | c 并行检查 | p 权限处理 | w 日志窗口 | l 日志尾部 | x 停止并清残留 | q 清理并退出 | h 菜单', 'title')

    def command(self, command):
        if command == 's':
            self.start()
        elif command == 'c':
            if self.child and self.child.poll() is None:
                self.emit('运行期间请在日志窗口查看状态；串口独占预检查需先停止。', 'warn')
            else:
                self.checks()
        elif command == 'p':
            self.permissions()
        elif command == 'w':
            self.window()
        elif command == 'x':
            self.stop()
        elif command == 'l':
            print(self.log_path.read_text()[-12000:], flush=True)
        elif command == 'q':
            return self.stop(include_dashboard=True)
        else:
            self.menu()
        return False


def main():
    parser = argparse.ArgumentParser(description='FSD 彩色交互启动包；未列出的参数转发给原启动脚本')
    parser.add_argument('--menu', action='store_true', help='只打开菜单，不立即启动车辆节点')
    parser.add_argument('--no-window', action='store_true')
    parser.add_argument('--plain', action='store_true', help='旧逐行菜单；默认使用 Textual 单终端界面')
    parser.add_argument('--no-color', action='store_true')
    parser.add_argument('--check', action='store_true', help='只做并行只读检查')
    parser.add_argument('--stop', action='store_true', help='请求正在运行的管理器清理并退出')
    parser.add_argument('--status', action='store_true')
    args, forwarded = parser.parse_known_args()
    RUNTIME.mkdir(parents=True, exist_ok=True)
    state_path = RUNTIME / 'state.json'
    if args.stop or args.status:
        data = json.loads(state_path.read_text()) if state_path.exists() else {}
        record = data.get('manager')
        alive = record and matches(record)
        print(json.dumps(data, ensure_ascii=False, indent=2) if args.status else '请求清理并退出' if alive else '管理器未运行')
        if args.stop:
            if alive:
                os.kill(record['pid'], signal.SIGTERM)
                deadline = time.monotonic() + 30
                while matches(record) and time.monotonic() < deadline:
                    time.sleep(.2)
                if matches(record):
                    print('管理器尚未退出；可能存在权限不足的残留，请在原终端输入 p 后 q。')
                    return 1
                print('管理器已退出。')
            else:
                print('如有崩溃残留，执行 --menu 会按登记身份清理。')
        return 0
    lock = (RUNTIME / 'manager.lock').open('w')
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        print('启动管理器已运行；可用 --status / --stop。')
        return 1
    manager = Manager(forwarded, not args.no_color)
    if args.check:
        return 0 if manager.checks() else 1
    if not sys.stdin.isatty():
        manager.emit('交互启动需要终端。只读验证可执行 --check；桌面使用终端运行本入口。', 'error')
        return 2
    requested = []
    signal.signal(signal.SIGINT, lambda *_: requested.append('x'))
    signal.signal(signal.SIGTERM, lambda *_: requested.append('q'))
    try:
        if not args.plain:
            dependency_dir = RUNTIME / 'ui_deps'
            if dependency_dir.is_dir():
                sys.path.insert(0, str(dependency_dir))
            try:
                from terminal_ui import FsdApp
            except ImportError as exc:
                manager.emit(f'终端界面依赖未安装：{exc}。执行 fsd_startup/install_ui.sh 或使用 --plain。', 'error')
                return 2
            manager.silent = True
            app = FsdApp(manager, auto_start=not args.menu, requested=requested,
                         telemetry=not args.no_window, color=not args.no_color)
            app.run(mouse=False)
            return 0
        try:
            manager.recover()
            if not args.no_window:
                manager.window()
        except Exception:
            manager.emit('会话恢复/窗口启动失败，保留菜单：\n' + traceback.format_exc(), 'error')
        manager.menu()
        if not args.menu:
            try:
                manager.start()
            except Exception:
                manager.emit(traceback.format_exc(), 'error')
        while True:
            try:
                manager.pump()
                command = requested.pop(0) if requested else None
                if command is None and select.select([sys.stdin], [], [], .25)[0]:
                    line = sys.stdin.readline()
                    command = line.strip().lower() if line else 'q'
                if command is not None and manager.command(command):
                    return 0
            except Exception:
                manager.emit('操作失败，保留菜单：\n' + traceback.format_exc(), 'error')
                manager.menu()
                time.sleep(.2)
    finally:
        if manager.run_log is not None or any(matches(item) for item in manager.records.values()):
            manager.stop(include_dashboard=True)
        manager.log.close()


if __name__ == '__main__':
    sys.exit(main())
