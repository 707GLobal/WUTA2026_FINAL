"""Drive the keyboard-only Textual interface without starting vehicle hardware."""
import asyncio
import sys
from pathlib import Path
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / '.runtime/fsd_manager/ui_deps'))
sys.path.insert(0, str(Path(__file__).parent))
from manager import Manager
from terminal_ui import FsdApp
from textual.widgets import Button, DataTable, Input, OptionList


def test_menu_keyboard_check_errors_views_and_exit(tmp_path):
    import manager
    import terminal_ui

    def failure(_):
        raise PermissionError('serial ACL test failure')

    async def scenario():
        instance = Manager([], color=False)
        app = FsdApp(instance, telemetry=False)
        try:
            async with app.run_test(size=(110, 36)) as pilot:
                await pilot.pause(.4)
                assert not list(app.query(Button))
                assert app.no_color is False
                await pilot.press('c')
                for _ in range(30):
                    await pilot.pause(.1)
                    if app.state_data.get('checks'):
                        break
                assert app.state_data['checks']['华测串口']['ok'] is False
                assert app.query_one('#check-table', DataTable).row_count == len(manager.CHECKS) + len(instance.runtime_checks.rows)
                table = app.query_one('#check-table', DataTable)
                table.focus()
                await pilot.press('enter')
                assert 'serial ACL test failure' in app.content()
                assert 'Traceback' in app.content()
                table.move_cursor(row=len(manager.CHECKS))
                await pilot.press('enter')
                assert app.selected_check == 'rt:sudo'
                assert '待启动验证' in app.content()
                await pilot.press('/')
                command = app.query_one('#command', Input)
                assert command.value == '/'
                await pilot.press(*'sensors', 'enter')
                assert app.view == 'sensors'
                assert not table.display
                app.query_one('#views', OptionList).focus()
                await pilot.press('down', 'enter')
                assert app.view == 'vehicle'
                app.change_view('logs')
                app.action_pause_log()
                assert app.paused
                app.action_pause_log()
                app.change_view('modules')
                assert app.query_one('#component').display
                await pilot.resize_terminal(80, 24)
                await pilot.pause(.2)
                assert app.query_one('#command').region.bottom <= 24
                app.query_one('#views', OptionList).focus()
                await pilot.press('q')
                for _ in range(30):
                    await pilot.pause(.1)
                    if app.done.is_set():
                        break
                assert app.done.is_set()
            assert not app.owner.is_alive()
        finally:
            instance.log.close()

    with patch.object(manager, 'ROOT', tmp_path), patch.object(manager, 'RUNTIME', tmp_path), \
            patch.object(manager, 'CHECKS', {'华测串口': failure}), \
            patch.object(terminal_ui, 'CHECKS', {'华测串口': failure}):
        asyncio.run(scenario())
