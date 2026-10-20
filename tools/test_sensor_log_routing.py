"""Exercise real launcher failure summaries with a simulated topic timeout."""
import os
from pathlib import Path
import re
import shutil
import subprocess

import pytest

SOURCE = (Path(__file__).resolve().parents[1] / 'start_full_fsd.sh').read_text(encoding='utf-8')


def git_bash():
    if os.name != 'nt':
        return Path('/bin/bash')
    git = shutil.which('git')
    return Path(git).parents[1] / 'bin/bash.exe' if git else None


@pytest.mark.parametrize('topic,label', [('/lidar', 'M1 激光雷达数据'), ('/image', 'ZED 相机图像')])
def test_sensor_timeout_prints_driver_log(topic, label, tmp_path):
    bash = git_bash()
    if bash is None or not bash.is_file():
        pytest.skip('Native Bash/Git Bash is unavailable; WSL is never used')
    external = tmp_path / 'external_log'
    external.mkdir()
    (external / 'perception_drivers.log').write_text('REAL_DRIVER_DIAGNOSTIC\n', encoding='utf-8')
    algorithm = tmp_path / 'fsd_modules/perception'
    algorithm.mkdir(parents=True)
    (algorithm / 'perception.log').write_text('WRONG_ALGORITHM_LOG\n', encoding='utf-8')
    functions = '\n'.join(re.search(rf'(?ms)^{name}\(\) \{{\n.*?^\}}', SOURCE)[0]
                          for name in ('item_label', 'log_path_for', 'show_failure_logs', 'fail', 'wait_topic'))
    harness = '''
set -eu
FSD_MODULES_DIR="$PWD/fsd_modules"
EXTERNAL_LOG_DIR="$PWD/external_log"
ZLG_LOG_DIR="$PWD/zlgCAN_log"
CHCNAV_LOG_DIR="$PWD/chcnav_log"
LIDAR_TOPIC=/lidar; IMAGE_TOPIC=/image; INS_TOPIC=/ins; FUSION_TOPIC=/fusion
timeout() { return 124; }
check_children() { :; }
'''
    result = subprocess.run([str(bash), '-c', harness + functions +
                             f'\nwait_topic "{topic}" 1 "{label}" perception_drivers\n'],
                            cwd=tmp_path, capture_output=True, text=True, encoding='utf-8', timeout=10)
    assert result.returncode == 1, result.stdout + result.stderr
    assert 'REAL_DRIVER_DIAGNOSTIC' in result.stderr
    assert 'WRONG_ALGORITHM_LOG' not in result.stderr


def test_serial_and_parallel_sensor_probes_route_to_driver_group():
    probes = [line for line in SOURCE.splitlines()
              if re.search(r'wait_topic "\$(LIDAR|IMAGE)_TOPIC"', line)]
    assert len(probes) == 4
    assert all(re.search(r'perception_drivers(?:\)|\s*$)', line) for line in probes)
