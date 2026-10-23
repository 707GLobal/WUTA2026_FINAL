"""Exercise startup reconnect sequencing without opening USB or sending CAN frames."""
from pathlib import Path
import os
import subprocess
import tempfile
import unittest


SOURCE = (Path(__file__).resolve().parents[1] / 'start_full_fsd.sh').read_text()
RECONNECT = SOURCE.split('reconnect_can_bridge() {', 1)[1].split(
    '\n}\n\nreconnect_can_bridge', 1)[0]
RECONNECT = 'reconnect_can_bridge() {' + RECONNECT + '\n}\n'


class CanReconnectTests(unittest.TestCase):
    def run_reconnect(self, *, old=False, ready=True, stuck=False,
                      kill_error=False, valid=True, foreign=False, manual=False):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            if old:
                (root / 'running').touch()
            harness = r'''
set -Eeuo pipefail
ROOT="$PWD"; LOG_DIR="$PWD"; ZLG_LOG_DIR="$PWD"; CAN_IFACE=can0
fail() { echo "FAIL $*" >> events; exit 1; }
report_ok() { echo "OK $*" >> events; }
can_bridge_pid() { [[ -f running ]] && echo 12345; }
proc_start_time() { [[ ! -f running ]] || echo 100; }
awk() { echo S; }
sleep() { :; }
sudo() {
    echo "SUDO $*" >> events
    [[ "$KILL_ERROR" == 0 ]] || return 1
    if [[ "$*" == *-KILL* || "$STUCK" == 0 ]]; then rm -f running; fi
}
pgrep() { [[ -f running || "$FOREIGN" == 1 ]]; }
start_group() {
    echo "START $*" >> events
    : > "$ZLG_LOG_DIR/bridge.log"
    if [[ "$READY" == 1 ]]; then echo '[INFO ] [main] 就绪:' > "$ZLG_LOG_DIR/bridge.log"; fi
}
check_children() { :; }
check_existing_can() { [[ "$VALID" == 1 ]]; }
'''
            script = harness + RECONNECT + '\nreconnect_can_bridge\n'
            env = dict(os.environ,
                       READY=str(int(ready)), STUCK=str(int(stuck)),
                       KILL_ERROR=str(int(kill_error)), VALID=str(int(valid)),
                       FOREIGN=str(int(foreign)), MANUAL_RECORD=str(int(manual)))
            result = subprocess.run(['bash', '-c', script], cwd=root, env=env,
                                    text=True, capture_output=True, timeout=5)
            events = (root / 'events').read_text() if (root / 'events').exists() else result.stderr
            return result.returncode, events

    def test_manual_record_starts_receive_only_with_frame_logging(self):
        code, events = self.run_reconnect(manual=True)
        self.assertEqual(code, 0, events)
        self.assertIn('--receive-only --log-frames=on', events)

    def test_manual_record_rejects_external_bridge_without_stopping_it(self):
        code, events = self.run_reconnect(manual=True, old=True)
        self.assertNotEqual(code, 0)
        self.assertIn('禁止复用', events)
        self.assertNotIn('SUDO', events)
        self.assertNotIn('START', events)

    def test_external_bridge_reused_without_shutdown_or_second_open(self):
        code, events = self.run_reconnect(old=True)
        self.assertEqual(code, 0, events)
        self.assertNotIn('SUDO', events)
        self.assertNotIn('START', events)
        self.assertIn('已复用外部 CAN 桥', events)

    def test_invalid_external_interface_blocks_reuse(self):
        code, events = self.run_reconnect(old=True, valid=False)
        self.assertNotEqual(code, 0)
        self.assertNotIn('SUDO', events)
        self.assertNotIn('START', events)

    def test_first_start_still_initializes(self):
        code, events = self.run_reconnect()
        self.assertEqual(code, 0, events)
        self.assertNotIn('kill', events)
        self.assertIn('START zlg_can', events)

    def test_interface_alone_cannot_pass_initialization(self):
        code, events = self.run_reconnect(ready=False)
        self.assertNotEqual(code, 0)
        self.assertIn('30 秒内未完成设备初始化', events)

    def test_ready_message_requires_enabled_interface(self):
        code, events = self.run_reconnect(valid=False)
        self.assertNotEqual(code, 0)
        self.assertIn('接口未就绪', events)

    def test_other_bridge_prevents_duplicate_open(self):
        code, events = self.run_reconnect(foreign=True)
        self.assertNotEqual(code, 0)
        self.assertNotIn('START', events)


if __name__ == '__main__':
    unittest.main()
