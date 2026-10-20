"""GNSS presentation and ROS topic-only startup regression checks."""
from pathlib import Path
import sys
from types import SimpleNamespace
from unittest.mock import patch
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
sys.path.insert(0, str(ROOT / 'fsd_startup'))
from gnss_status import gnss_allows_start, gnss_description
from dashboard import message_summary, Telemetry


@pytest.mark.parametrize('code', range(10))
def test_only_four_allows_next_stage(code):
    assert gnss_allows_start(code) is (code == 4)
    assert gnss_description(code) != '未知状态'
    text = message_summary('GNSS', SimpleNamespace(stat=[1, code]))
    assert f'状态码={code}' in text and gnss_description(code) in text
    assert '门控' not in text and '允许下一阶段' not in text


def test_status_zero_is_online_when_messages_arrive():
    with patch.object(Telemetry, 'spin', return_value=None):
        monitor = Telemetry('/unused')
        monitor.thread.join()
        monitor.record('GNSS', SimpleNamespace(stat=[1, 0]))
        lines = monitor.lines(['GNSS'])
        assert '[在线]' in lines
        assert '未知（等待任务机诊断）' in lines
        assert '不判设备自检失败' in lines


def test_cli_retains_odometry_probe_and_no_extra_serial_gate():
    source = (ROOT / 'start_full_fsd.sh').read_text()
    assert 'wait_ins_gnss.py' not in source
    assert 'INS_RAW_TOPIC' not in source
    assert 'wait_topic "$INS_TOPIC" 20 \'华测惯导输出\'' in source


@pytest.mark.parametrize('mode,target', [(0, '状态4'), (1, '状态4'), (2, '状态4'), (4, '状态4')])
def test_mission_wait_text_uses_mode_gate(mode, target):
    from mission_diagnostics import describe
    from runtime_checks import RuntimeChecks
    description = ('mode_selected=1 go_count=0 go_pending=0 lidar_ready=1 '
                   'localization_ready=1 gnss_status=0 gnss_ready=0 wait=gnss,go')
    lines = describe(0, mode, 0, description)
    assert '等待' + target in lines['GNSS']
    assert 'GNSS' + target in lines['下一步']
    checks = RuntimeChecks({})
    checks.consume('[状态] GNSS：' + lines['GNSS'])
    assert checks.rows['gnss']['status'] == '等待' + target


@pytest.mark.parametrize('code', range(10))
def test_runtime_accepts_authoritative_inspection_gate(code):
    from mission_diagnostics import describe
    from runtime_checks import RuntimeChecks
    description = ('mode_selected=1 go_count=0 go_pending=0 lidar_ready=1 '
                   f'localization_ready=1 gnss_status={code} gnss_ready=1 wait=go')
    lines = describe(1, 3, 0, description)
    assert '门控=不要求（车检）' in lines['GNSS']
    assert lines['下一步'] == '等待：本次任务GO'
    checks = RuntimeChecks({})
    checks.consume('[状态] GNSS：' + lines['GNSS'])
    assert checks.rows['gnss']['ok'] is True
    assert checks.rows['gnss']['status'] == '不要求（车检）'


def test_sensor_page_follows_mission_gate_and_expires_diagnostics():
    with patch.object(Telemetry, 'spin', return_value=None):
        monitor = Telemetry('/unused')
        monitor.thread.join()
        monitor.record('GNSS', SimpleNamespace(stat=[1, 6]))
        for mode, ready, expected in [(3, 1, '不要求（车检）'), (2, 0, '等待状态4'),
                                      (2, 1, '通过')]:
            monitor.record('任务', SimpleNamespace(state=1, mission_mode=mode, localization_mode=0,
                description=('mode_selected=1 go_count=0 go_pending=0 lidar_ready=1 '
                             f'localization_ready=1 gnss_status=6 gnss_ready={ready} wait=go')))
            assert '任务机GNSS门控：' + expected in monitor.lines(['GNSS'])
        monitor.samples['任务']['received'] -= 4
        assert '未知（任务机诊断超时）' in monitor.lines(['GNSS'])
