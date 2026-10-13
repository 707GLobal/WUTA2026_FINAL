"""Rich + Textual front end. One worker owns all manager mutations."""
from collections import deque
import asyncio
import json
import os
from pathlib import Path
import queue
import threading
import time
import traceback

from rich.text import Text
from textual.app import App, ComposeResult
from textual.binding import Binding
from textual.theme import Theme
from textual.containers import Horizontal, Vertical
from textual.widgets import DataTable, OptionList, RichLog, Select, Static
from textual.widgets.option_list import Option

from checks import CHECKS
from dashboard import tail

VIEWS = [('checks', '01  启动检查'), ('vehicle', '02  车辆状态'),
         ('sensors', '03  传感器参数'), ('logs', '04  实时日志'),
         ('modules', '05  组件日志'), ('config', '06  模块配置')]
STYLES = {'ok': '#a6e3a1', 'error': '#f38ba8', 'warn': '#f9e2af',
          'info': '#89b4fa', 'title': 'bold #cba6f7', 'muted': '#a6adc8'}


def summary(detail):
    """Keep the error cause in the table, the full traceback in its detail pane."""
    lines = [line.strip() for line in str(detail).splitlines() if line.strip()]
    return lines[-1] if lines else '无详情'


def colored_lines(content, color=True):
    result = Text()
    for line in content.splitlines(True):
        lowered = line.lower()
        level = 'error' if any(word in lowered for word in ('失败', '错误', 'error', 'traceback', 'fatal')) else (
            'warn' if any(word in lowered for word in ('等待', '超时', 'warn')) else
            'ok' if any(word in line for word in ('成功', '在线', '通过')) else 'info')
        part = Text.from_ansi(line)
        if color:
            part.stylize(STYLES[level])
        result.append(part)
    return result


class FsdApp(App):
    TITLE = 'WUTA / FSD'
    ENABLE_COMMAND_PALETTE = False
    CSS = """
    Screen { background: #1e1e2e; color: #cdd6f4; }
    #brand { height: 3; padding: 1 2 0 2; color: #cba6f7; text-style: bold; }
    #status { height: 2; padding: 0 2; color: #a6adc8; text-style: dim; }
    #workspace { height: 1fr; margin: 0 2; }
    #sidebar { width: 23; margin-right: 2; }
    #views { height: 1fr; border: none; background: #181825; color: #cdd6f4; }
    #views > .option-list--option-highlighted { background: #313244; color: #cdd6f4; text-style: bold; }
    #views:focus > .option-list--option-highlighted { background: #45475a; color: #cba6f7; text-style: bold; }
    #side-note { height: 6; padding: 1; color: #a6adc8; text-style: dim; }
    #main { width: 1fr; }
    #heading { height: 2; color: #94e2d5; text-style: bold; }
    #check-table { height: 10; background: #1e1e2e; color: #cdd6f4; }
    #check-table > .datatable--header { background: #181825; color: #a6adc8; text-style: bold; }
    #check-table > .datatable--cursor { background: #313244; color: #cdd6f4; text-style: bold; }
    #check-table:focus > .datatable--cursor { background: #45475a; color: #cdd6f4; text-style: bold; }
    #component { height: 3; margin-bottom: 1; }
    #detail { height: 1fr; border: round #45475a; background: #1e1e2e; color: #cdd6f4; padding: 0 1; border-title-color: #a6adc8; }
    #detail:focus { border: round #89b4fa; }
    #actions { height: 2; margin: 1 2 0 2; color: #94e2d5; }
    #keys { height: 1; padding: 0 2; background: #11111b; color: #a6adc8; text-style: dim; }
    Select, SelectCurrent, SelectOverlay { background: #181825; color: #cdd6f4; border: round #45475a; }
    * { scrollbar-background: #181825; scrollbar-color: #45475a; scrollbar-color-active: #89b4fa; }
    """
    BINDINGS = [Binding('c', 'operate("c")', '检查'), Binding('s', 'operate("s")', '启动'),
                Binding('p', 'operate("p")', '权限'), Binding('x', 'operate("x")', '停止'),
                Binding('q', 'quit', '清理退出'), Binding('ctrl+q', 'quit', show=False, priority=True),
                Binding('ctrl+c', 'operate("x")', show=False, priority=True),
                Binding('space', 'pause_log', '暂停日志')]

    def __init__(self, manager, *, auto_start=False, requested=None, telemetry=True, color=True):
        # Color is an explicit launcher preference, independent of inherited NO_COLOR.
        previous = os.environ.pop('NO_COLOR', None)
        if not color:
            os.environ['NO_COLOR'] = '1'
        try:
            super().__init__(ansi_color=False)
        finally:
            os.environ.pop('NO_COLOR', None)
            if previous is not None:
                os.environ['NO_COLOR'] = previous
        self.register_theme(Theme(
            name='fsd-mocha', primary='#cba6f7', secondary='#89b4fa', accent='#94e2d5',
            background='#1e1e2e', surface='#181825', panel='#313244', foreground='#cdd6f4',
            success='#a6e3a1', warning='#f9e2af', error='#f38ba8', dark=True, ansi=False,
            variables={'border': '#45475a', 'border-blurred': '#313244',
                       'block-cursor-background': '#45475a', 'block-cursor-foreground': '#cdd6f4',
                       'block-cursor-text-style': 'bold', 'scrollbar-corner-color': '#181825'}))
        self.theme = 'fsd-mocha'
        self.manager = manager
        self.manager.silent = True
        self.auto_start = auto_start
        self.requested = requested if requested is not None else []
        self.use_telemetry = telemetry
        self.use_color = color
        self.commands = queue.Queue()
        self.done = threading.Event()
        self.busy = False
        self.state_data = {}
        self.history = deque(maxlen=2000)
        self.view = 'checks'
        self.selected_check = next(iter(CHECKS))
        self.paused = False
        self.last_content = None
        self.last_checks = None
        self.components = []

    def compose(self) -> ComposeResult:
        yield Static('WUTA / FSD    ›    启动与诊断', id='brand')
        yield Static('等待操作', id='status', markup=False)
        with Horizontal(id='workspace'):
            with Vertical(id='sidebar'):
                yield OptionList(*(Option(label, id=key) for key, label in VIEWS), id='views')
                yield Static('↑↓ 选择 · Enter 打开\nTab 切换焦点\n检查表 Enter 查看详情', id='side-note')
            with Vertical(id='main'):
                yield Static('启动检查  /  独立模块并行执行', id='heading')
                yield DataTable(id='check-table', cursor_type='row')
                yield Select([], prompt='选择组件日志', id='component')
                yield RichLog(id='detail', wrap=True, markup=False, max_lines=3000, min_width=1)
        yield Static('C 检查   S 启动   P 权限   X 停止   Q 清理退出', id='actions')
        yield Static('↑↓ 选择   Enter 打开   Tab 切换区域   Space 暂停日志', id='keys')

    def on_mount(self):
        table = self.query_one('#check-table', DataTable)
        for title, width in [('阶段', 6), ('检查项目', 24), ('状态', 16), ('结果摘要', 45)]:
            table.add_column(title, width=width)
        self.query_one('#component').display = False
        self.query_one('#views', OptionList).focus()
        self.query_one('#views', OptionList).highlighted = 0
        self.owner = threading.Thread(target=self.manager_loop, daemon=True)
        self.owner.start()
        self.set_interval(.2, self.refresh_data)
        self.refresh_data()

    async def on_unmount(self):
        if hasattr(self, 'owner') and self.owner.is_alive():
            self.commands.put('q')
            # Never race a cleanup against the manager's active operation.
            await asyncio.to_thread(self.owner.join)

    def on_resize(self, event):
        if not self.is_mounted:
            return
        compact = event.size.height < 30
        self.query_one('#brand').styles.height = 2 if compact else 3
        self.query_one('#status').styles.height = 1 if compact else 2
        self.query_one('#check-table').styles.height = 7 if compact else 10
        self.query_one('#side-note').display = not compact
        self.query_one('#actions').styles.margin = (0 if compact else 1, 2, 0, 2)
        self.query_one('#sidebar').styles.width = 23 if event.size.width >= 100 else 18

    def manager_loop(self):
        """Serialize start/check/cleanup and pumping; terminal rendering stays responsive."""
        try:
            try:
                self.busy = True
                self.manager.recover()
                if self.use_telemetry:
                    self.manager.telemetry()
                self.manager.emit('终端已就绪。按 C 检查；Tab 切换区域，方向键选择、Enter 打开页面。', 'title')
                if self.auto_start:
                    self.manager.start()
            except Exception:
                self.manager.emit(traceback.format_exc(), 'error')
            finally:
                self.busy = False
            while not self.done.is_set():
                try:
                    command = self.commands.get(timeout=.2)
                except queue.Empty:
                    command = None
                try:
                    self.busy = True
                    if command == 'q':
                        if self.manager.command('q'):
                            self.done.set()
                            self.manager.ui_events.put(('exit', None))
                            break
                    elif command:
                        self.manager.command(command)
                    self.manager.pump()
                except Exception:
                    self.manager.emit('操作失败，界面继续保留：\n' + traceback.format_exc(), 'error')
                finally:
                    self.busy = False
        finally:
            self.done.set()

    def action_operate(self, command):
        if self.busy and command not in ('q', 'x'):
            self.notify('当前操作仍在进行，可继续切换页面查看状态。', severity='warning')
            return
        # Stop and quit wait behind the current operation, never run concurrently.
        if command not in list(self.commands.queue):
            self.commands.put(command)
        if command == 'c':
            self.change_view('checks')

    def action_quit(self):
        self.action_operate('q')

    def action_pause_log(self):
        self.paused = not self.paused
        self.last_content = None

    def on_option_list_option_selected(self, event: OptionList.OptionSelected):
        self.change_view(event.option.id)

    def change_view(self, view):
        self.view = view
        self.last_content = None
        self.query_one('#check-table').display = view == 'checks'
        self.query_one('#component').display = view == 'modules'
        title = dict(VIEWS)[view][4:]
        self.query_one('#heading', Static).update(title + ('  /  Enter 查看完整检查原因' if view == 'checks' else ''))
        self.render_content()

    def on_data_table_row_selected(self, event: DataTable.RowSelected):
        self.selected_check = str(event.row_key.value)
        self.last_content = None
        self.render_content()

    def on_select_changed(self, event: Select.Changed):
        self.last_content = None
        self.render_content()

    def refresh_data(self):
        while self.requested:
            self.action_operate(self.requested.pop(0))
        while not self.manager.ui_events.empty():
            kind, payload = self.manager.ui_events.get_nowait()
            if kind == 'exit':
                self.exit()
                return
            if kind == 'state':
                self.state_data = payload
            else:
                self.history.append(payload)
        checks = self.state_data.get('checks', {})
        runtime = self.state_data.get('runtime_checks', {})
        signature = (json.dumps([checks, runtime], ensure_ascii=False), self.state_data.get('phase'))
        if signature != self.last_checks:
            table = self.query_one('#check-table', DataTable)
            cursor = table.cursor_row
            table.clear()
            for name in CHECKS:
                item = checks.get(name)
                status = item.get('status', '通过') if item and item['ok'] else '失败' if item else (
                    '检查中' if self.state_data.get('phase') == '并行检查中' else '未检查')
                style = STYLES['ok'] if status == '通过' else STYLES['error'] if status == '失败' else STYLES['muted'] if status == '跳过' else STYLES['warn']
                table.add_row('预检查', name, Text(status, style=style if self.use_color else ''),
                              summary(item['detail']) if item else '—', key=name)
            for key, item in runtime.items():
                status = item['status']
                style = STYLES['error'] if item['ok'] is False else STYLES['ok'] if item['ok'] is True else (
                    STYLES['muted'] if status in ('跳过', '待启动验证', '已停止') else STYLES['warn'])
                table.add_row(item['phase'], item['label'], Text(status, style=style if self.use_color else ''),
                              summary(item['detail']), key='rt:' + key)
            table.move_cursor(row=min(cursor, table.row_count - 1))
            self.last_checks = signature
        ok = sum(item['ok'] and item.get('status', '通过') == '通过' for item in checks.values())
        failed = sum(not item['ok'] for item in checks.values())
        verified = sum(item['ok'] is True for item in runtime.values())
        status = f'{self.state_data.get("phase", "等待操作")}   ·   预检查 {ok}/{len(CHECKS)} 失败 {failed}   ·   启动验证 {verified} 项'
        if self.busy:
            status += '   ·   操作处理中'
        if self.paused:
            status += '   ·   日志已暂停'
        self.query_one('#status', Static).update(status)
        folder = self.state_data.get('module_log_dir') or self.state_data.get('log_dir')
        available = sorted(str(path) for path in Path(folder).glob('*.log')) if folder else []
        if available != self.components:
            self.components = available
            selector = self.query_one('#component', Select)
            selector.set_options([(Path(path).name, path) for path in available])
            if available:
                selector.value = available[0]
        self.render_content()

    def content(self):
        if self.view == 'checks':
            if self.selected_check.startswith('rt:'):
                item = self.state_data.get('runtime_checks', {}).get(self.selected_check[3:])
                if item:
                    return f'{item["phase"]} / {item["label"]}\n状态：{item["status"]}\n' + '─' * 28 + '\n' + item['detail']
            item = self.state_data.get('checks', {}).get(self.selected_check)
            heading = f'{self.selected_check}\n' + '─' * 28 + '\n'
            if item:
                return heading + ('[' + item.get('status', '成功') + '] ' if item['ok'] else '[失败] ') + item['detail']
            return heading + '选择“检查”开始。独立模块并行执行，结果逐项更新。\n\n↑↓ 选择模块，Enter 展开完整详情。\n失败后可处理权限或修复设备，再重新检查。'
        if self.view in ('vehicle', 'sensors'):
            path = self.manager.state_path.parent / 'telemetry.json'
            try:
                data = json.loads(path.read_text())
                if time.time() - data['updated'] > 3:
                    return '[超时] 只读采集进程没有更新，请查看实时日志。'
                extra = '\n控制请求值不代表执行器反馈；GO/模式门控以任务状态为准。' if self.view == 'vehicle' else '\n频率为最近5秒订阅接收统计；图像、点云与轨迹仍由 RViz 显示。'
                return data[self.view] + extra
            except (OSError, ValueError, KeyError):
                return '[等待] 只读 ROS 采集尚未就绪。' if self.use_telemetry else '[等待] 当前使用 --no-window 禁用了状态采集。'
        if self.view == 'config':
            return tail(self.manager.state_path.parent / 'manifest.json', 200000)
        if self.view == 'modules':
            value = self.query_one('#component', Select).value
            return tail(value) if isinstance(value, str) else '[等待] 尚无组件日志，启动后可选择。'
        return '\n'.join(line for line, _ in self.history)

    def render_content(self):
        if self.paused and self.view in ('logs', 'modules'):
            return
        content = self.content()
        if content == self.last_content:
            return
        panel = self.query_one('#detail', RichLog)
        offset = panel.scroll_y
        panel.clear()
        panel.write(colored_lines(content, self.use_color), scroll_end=self.view in ('logs', 'modules'))
        if self.view not in ('logs', 'modules'):
            panel.scroll_to(y=offset, animate=False)
        panel.border_title = '› ' + dict(VIEWS)[self.view][4:]
        self.last_content = content
