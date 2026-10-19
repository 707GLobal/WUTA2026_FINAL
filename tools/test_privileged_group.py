"""Real subprocess lifecycle tests; no sudo, USB access, or CAN transmission."""
from pathlib import Path
import os
import signal
import subprocess
import sys
import tempfile
import time
import unittest


ROOT = Path(__file__).resolve().parents[1]
HELPER = ROOT / 'tools/run_privileged_group.py'


class PrivilegedGroupTests(unittest.TestCase):
    def test_child_exit_code_is_preserved(self):
        result = subprocess.run([sys.executable, str(HELPER), sys.executable,
                                 '-c', 'raise SystemExit(17)'], timeout=5)
        self.assertEqual(result.returncode, 17)

    def test_missing_command_fails(self):
        result = subprocess.run([sys.executable, str(HELPER), '/missing/can-bridge'],
                                capture_output=True, timeout=5)
        self.assertEqual(result.returncode, 127)

    def test_supervisor_stays_alive_and_forwards_shutdown(self):
        for signum in (signal.SIGINT, signal.SIGTERM):
            with self.subTest(signum=signum), tempfile.TemporaryDirectory() as tmp:
                path = Path(tmp) / 'ready'
                code = ('import signal,time,pathlib,sys; '
                        'signal.signal(signal.SIGINT,lambda *_: sys.exit(0)); '
                        'signal.signal(signal.SIGTERM,lambda *_: sys.exit(0)); '
                        'pathlib.Path(sys.argv[1]).write_text("ready"); time.sleep(30)')
                process = subprocess.Popen([sys.executable, str(HELPER),
                                            sys.executable, '-c', code, str(path)])
                try:
                    deadline = time.monotonic() + 3
                    while not path.exists() and time.monotonic() < deadline:
                        time.sleep(.02)
                    self.assertTrue(path.exists())
                    self.assertIsNone(process.poll())
                    process.send_signal(signum)
                    self.assertEqual(process.wait(timeout=3), 0)
                finally:
                    if process.poll() is None:
                        process.terminate()
                        process.wait(timeout=3)

    def test_actual_start_group_can_branch_tracks_child(self):
        source = (ROOT / 'start_full_fsd.sh').read_text()
        function = 'start_group() {' + source.split('start_group() {', 1)[1].split(
            '\n}\n\nwait_topic()', 1)[0] + '\n}\n'
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)
            mock_sudo = path / 'sudo'
            mock_sudo.write_text('#!/bin/bash\n[[ "$1" != -n ]] || shift\nexec "$@"\n')
            mock_sudo.chmod(0o755)
            harness = r'''
set -euo pipefail
CHILD_PIDS=(); CHILD_NAMES=()
item_label() { echo "$1"; }
report_ok() { :; }
check_children() { kill -0 "${CHILD_PIDS[0]}"; }
log_path_for() { echo "$LOG_DIR/$1.log"; }
'''
            harness += function + r'''
start_group zlg_can /usr/bin/python3 -c 'import time; time.sleep(30)'
kill -TERM "${CHILD_PIDS[0]}"
code=0
wait "${CHILD_PIDS[0]}" || code=$?
[[ "$code" == 143 ]]
'''
            env = dict(os.environ, ROOT=str(ROOT), LOG_DIR=str(path),
                       PATH=str(path) + ':' + os.environ['PATH'])
            result = subprocess.run(['bash', '-c', harness], env=env,
                                    capture_output=True, text=True, timeout=5)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)


if __name__ == '__main__':
    unittest.main()
