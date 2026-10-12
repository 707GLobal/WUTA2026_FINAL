"""Verify check coverage and mode semantics against the existing hardware entry."""
from pathlib import Path
import re
import subprocess
import sys
from unittest.mock import patch
import pytest

sys.path.insert(0, str(Path(__file__).parent))
import checks
from manager import options_from
from runtime_checks import RuntimeChecks, PROCESS_NAMES, TOPICS

SOURCE = (Path(__file__).resolve().parents[1] / 'start_full_fsd.sh').read_text()


def test_runtime_catalog_covers_original_processes_and_topics():
    launched = set(re.findall(r'^\s*start_group (\w+) ', SOURCE, re.M))
    assert launched == set(PROCESS_NAMES)
    labels = set(re.findall(r"wait_topic .*? \d+ '([^']+)'", SOURCE))
    assert labels == set(TOPICS)
    rows = RuntimeChecks({}).rows
    assert all('process:' + name in rows for name in launched)
    assert all('topic:' + name in rows for name in labels)


def test_topic_success_never_follows_only_process_success():
    progress = RuntimeChecks({})
    progress.consume('[启动] ZED / M1 驱动与感知算法组 ...')
    assert progress.rows['process:perception']['status'] == '启动中'
    progress.consume('[成功] ZED / M1 驱动与感知算法组：进程已启动（PID=123）；日志：/tmp/perception.log')
    assert progress.rows['process:perception']['ok'] is True
    assert progress.rows['topic:检测融合输出']['ok'] is None
    progress.consume('[检查] 检测融合输出：等待 /custom/cones（最多 20 秒）...')
    assert progress.rows['topic:检测融合输出']['status'] == '等待数据'
    progress.consume('[失败] 检测融合输出：20 秒内未收到 /custom/cones 消息。')
    assert progress.rows['topic:检测融合输出']['ok'] is False
    assert progress.rows['complete']['ok'] is None


def test_runtime_can_initialization_and_shutdown():
    progress = RuntimeChecks({})
    progress.consume('[成功] ZLG CAN 硬件桥：进程已启动（PID=123）；日志：/tmp/zlg.log')
    assert progress.rows['can_ready']['ok'] is None
    progress.consume('[成功] CAN 盒已重新打开并初始化：can0；VCU 收报情况请查看日志')
    assert progress.rows['can_ready']['ok'] is True
    progress.stopped(1)
    assert progress.rows['process:zlg_can']['status'] == '已停止'
    assert progress.rows['complete']['ok'] is False


def test_modes_and_last_flag_wins():
    options = options_from(['--no-rviz', '--rviz', '--skip-build', '--build'])
    assert options['_rviz'] is True and options['_build'] == 'yes'
    options = options_from(['--rviz', '--no-rviz', '--build', '--skip-build', '--sensor-only', '--reuse-ins'])
    progress = RuntimeChecks(options)
    for name in ('visualization', 'controller', 'ekf', 'chcnav'):
        assert progress.rows['process:' + name]['status'] == '跳过'
    assert progress.rows['topic:定位输出']['status'] == '跳过'
    assert progress.rows['topic:华测惯导输出']['status'] != '跳过'
    assert options['_rviz'] is False and options['_build'] == 'no'
    built = RuntimeChecks({'build-only': True})
    assert built.rows['process:zlg_can']['status'] == '跳过'
    built.consume('[成功] 华测、FSD、ZLG CAN 桥构建全部完成')
    assert built.rows['build:zlg']['ok'] is True


def test_reuse_ins_still_requires_serial_permission():
    with patch.object(checks.os, 'access', return_value=False):
        with pytest.raises(RuntimeError, match='不可读写'):
            checks.serial({'reuse-ins': True})
    with patch.object(checks.os, 'access', return_value=True), patch.object(checks, 'run') as run:
        assert '跳过独占' in checks.serial({'reuse-ins': True})
        run.assert_not_called()


def test_skip_build_requires_artifacts_but_auto_reports_pending(tmp_path):
    with patch.object(checks, 'ROOT', tmp_path):
        with pytest.raises(RuntimeError, match='缺少安装产物'):
            checks.build_artifacts(options_from(['--skip-build']))
        assert '尚未构建成功' in checks.build_artifacts({})


def test_rviz_mode_requires_same_files_and_desktop(tmp_path):
    with patch.object(checks, 'ROOT', tmp_path):
        assert '跳过' in checks.visualization(options_from(['--no-rviz']))
        with pytest.raises(RuntimeError, match='缺少文件'):
            checks.visualization({})
        for relative in ('start.sh', 'src/fsd_visualization/config/fsd.rviz', 'src/fsd_visualization/config/topics.yaml'):
            file = tmp_path / 'fsd-visualization-code' / relative
            file.parent.mkdir(parents=True, exist_ok=True)
            file.touch()
        with patch.dict(checks.os.environ, {'DISPLAY': '', 'WAYLAND_DISPLAY': ''}):
            with pytest.raises(RuntimeError, match='图形桌面'):
                checks.visualization({})


def test_missing_launch_and_chcnav_entry_are_not_silently_accepted(tmp_path):
    with patch.object(checks, 'ROOT', tmp_path):
        with pytest.raises(RuntimeError) as failure:
            checks.dependencies({})
        detail = str(failure.value)
        assert 'fusion_modules_hardware.launch.py' in detail
        assert 'start_chcnav_fsd.sh' in detail


def test_existing_supervisor_blocks_a_second_stack(tmp_path):
    from processes import identity
    import os
    record = identity(os.getpid())
    file = tmp_path / '.runtime/full_fsd/supervisor.pid'
    file.parent.mkdir(parents=True)
    file.write_text(f'{record["pid"]} {record["start"]}\n')
    with patch.object(checks, 'ROOT', tmp_path):
        with pytest.raises(RuntimeError, match='全栈启动脚本正在运行'):
            checks.supervisor({})
        file.write_text(f'{record["pid"]} 0\n')
        assert '已失效' in checks.supervisor({})
