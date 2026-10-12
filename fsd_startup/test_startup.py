"""Hardware-free concurrency, failure recovery and scoped cleanup tests."""
import json
import os
from pathlib import Path
import subprocess
import sys
import threading
import time
from unittest.mock import patch
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).parent))
from processes import cleanup, descendants, identity, matches
from manager import Manager


def test_checks_run_concurrently_and_collect_errors(tmp_path):
    import manager
    barrier = threading.Barrier(3, timeout=2)

    def good(_):
        barrier.wait()
        return 'ready'

    def bad(_):
        barrier.wait()
        raise RuntimeError('injected serial permission error')

    with patch.object(manager, 'RUNTIME', tmp_path), patch.object(manager, 'ROOT', tmp_path), \
            patch.object(manager, 'CHECKS', {'one': good, 'two': good, 'serial': bad}):
        instance = Manager([], color=False)
        try:
            assert not instance.checks()
            assert len(instance.results) == 3
            assert instance.results['one']['ok']
            assert 'injected serial permission error' in instance.results['serial']['detail']
            assert instance.command('h') is False
        finally:
            instance.log.close()


def test_pid_identity_rejects_reuse():
    record = identity(os.getpid())
    assert matches(record)
    assert not matches(dict(record, start='wrong-start-time'))
    assert not matches(None)


def test_cleanup_kills_detached_descendant_but_preserves_unrelated(tmp_path):
    unrelated = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(30)'])
    marker = tmp_path / 'ready'
    child_code = ('import signal,time,pathlib,sys; '
                  'signal.signal(signal.SIGINT,signal.SIG_IGN); '
                  'signal.signal(signal.SIGTERM,signal.SIG_IGN); '
                  'pathlib.Path(sys.argv[1]).touch(); time.sleep(30)')
    parent_code = ('import subprocess,sys,time; '
                   'subprocess.Popen([sys.executable,"-c",sys.argv[1],sys.argv[2]],start_new_session=True); '
                   'time.sleep(30)')
    parent = subprocess.Popen([sys.executable, '-c', parent_code, child_code, str(marker)],
                              stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    records = []
    try:
        deadline = time.monotonic() + 3
        while not marker.exists() and time.monotonic() < deadline:
            time.sleep(.02)
        assert marker.exists()
        records = descendants(parent.pid)
        assert len(records) >= 2
        remaining, _ = cleanup(records, grace=.1)
        parent.wait(timeout=3)
        assert not remaining
        assert unrelated.poll() is None
    finally:
        cleanup(records, grace=.1)
        if parent.poll() is None:
            parent.kill()
        parent.wait()
        unrelated.terminate()
        unrelated.wait()


def test_parallel_topic_probes_collect_failures_without_supervisor_cleanup(tmp_path):
    source = (Path(__file__).resolve().parents[1] / 'start_full_fsd.sh').read_text()
    function = 'wait_independent_sensor_topics() {' + source.split(
        'wait_independent_sensor_topics() {', 1)[1].split('\n}\n', 1)[0] + '\n}\n'
    harness = '''
set -Eeuo pipefail
LIDAR_TOPIC=lidar; IMAGE_TOPIC=image; INS_TOPIC=ins; SENSOR_ONLY=0; REUSE_INS=0
trap 'echo cleanup >> events' EXIT
wait_topic() {
    [[ "$TOPIC_CHECK_WORKER" == 1 ]]
    echo "start:$1" >> events
    # All checks must start before any completes.
    for ((i=0;i<100;i++)); do
        [[ "$(wc -l < events)" -ge 3 ]] && break
        sleep .01
    done
    [[ "$i" -lt 100 ]] || exit 5
    [[ "$1" != image ]] || exit 7
}
check_children() { echo checked >> events; }
fail() { echo failed >> events; exit 1; }
'''
    result = subprocess.run(['bash', '-c', harness + function + 'wait_independent_sensor_topics'],
                            cwd=tmp_path, capture_output=True, text=True, timeout=5)
    assert result.returncode == 1, result.stderr
    events = (tmp_path / 'events').read_text().splitlines()
    assert set(events[:3]) == {'start:lidar', 'start:image', 'start:ins'}
    assert events.count('cleanup') == 1
    assert 'checked' in events and 'failed' in events


def test_failed_runtime_returns_to_menu(tmp_path):
    import manager
    with patch.object(manager, 'RUNTIME', tmp_path), patch.object(manager, 'ROOT', tmp_path):
        instance = Manager([], color=False)
        instance.child = subprocess.Popen([sys.executable, '-c', 'raise SystemExit(17)'])
        instance.remember(instance.child.pid)
        instance.child.wait()
        try:
            instance.pump()
            assert instance.child is None
            assert instance.command('h') is False
            assert instance.command('q') is True
            assert json.loads(instance.state_path.read_text())['phase'] == '已停止'
            assert '退出码 17' in instance.log_path.read_text()
        finally:
            instance.log.close()


def test_permission_repair_grants_only_current_user_acl(tmp_path):
    import manager
    port = tmp_path / 'serial'
    port.touch()
    helper = tmp_path / 'sudo_askpass.local.sh'
    helper.write_text('# test fixture; no real credential\n')
    with patch.object(manager, 'RUNTIME', tmp_path), patch.object(manager, 'ROOT', tmp_path), \
            patch('privilege.ASKPASS', helper):
        instance = Manager([f'--ins-port={port}'], color=False)
        try:
            with patch.object(manager.stat, 'S_ISCHR', return_value=True), \
                    patch.object(manager.os, 'access', side_effect=[False, True]), \
                    patch.object(manager.subprocess, 'run', return_value=SimpleNamespace(
                        returncode=0, stdout='', stderr='')) as run:
                assert instance.permissions()
                calls = [item.args[0] for item in run.call_args_list]
                assert calls[-1] == ['sudo', '-A', 'setfacl', '-m', f'u:{os.getuid()}:rw', '--', str(port)]
                assert run.call_args.kwargs['env']['SUDO_ASKPASS'].endswith('sudo_askpass.local.sh')
                assert not any('chmod' in call for call in calls)
        finally:
            instance.log.close()


def test_dashboard_reports_sensor_age_and_request_values():
    from dashboard import Telemetry
    with patch.object(Telemetry, 'spin', return_value=None):
        telemetry = Telemetry('/unused/state.json')
        telemetry.thread.join()
        telemetry.record('相机', SimpleNamespace(header=SimpleNamespace(frame_id='camera'),
                                                width=1280, height=768, encoding='rgb8'))
        assert '1280×768' in telemetry.lines(['相机'])
        telemetry.samples['相机']['received'] -= 4
        assert '[超时]' in telemetry.lines(['相机'])
        telemetry.record('控制请求', SimpleNamespace(speed=2., angle=3., throttle_brake=.4))
        assert '请求值' in telemetry.lines(['控制请求'])
