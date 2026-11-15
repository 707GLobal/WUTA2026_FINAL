"""Exercise startup reconnect sequencing without opening USB or sending CAN frames."""
from pathlib import Path
import os
import subprocess
import tempfile
import unittest


SOURCE = (Path(__file__).resolve().parents[1] / 'start_full_fsd.sh').read_text()
RECONNECT = SOURCE.split('reconnect_can_bridge() {', 1)[1].split('\n}\n', 1)[0]
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
    [[ "$STUCK" == 0 ]] || return 1
    rm -f running
}
pgrep() { [[ -f running || "$FOREIGN" == 1 ]]; }
can_bridge_active() { pgrep; }
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

    def test_manual_record_replaces_old_bridge_with_receive_only_bridge(self):
        code, events = self.run_reconnect(manual=True, old=True)
        self.assertEqual(code, 0, events)
        self.assertIn('--stop-bridge 12345 100 can0', events)
        self.assertIn('--receive-only --log-frames=on', events)

    def test_old_bridge_stopped_before_fresh_initialization(self):
        code, events = self.run_reconnect(old=True)
        self.assertEqual(code, 0, events)
        self.assertIn('--stop-bridge 12345 100 can0', events)
        self.assertLess(events.index('SUDO'), events.index('START'))
        self.assertIn('CAN 盒已重新打开并初始化', events)

    def test_unready_old_interface_does_not_skip_reinitialization(self):
        code, events = self.run_reconnect(old=True, valid=False)
        self.assertNotEqual(code, 0)
        self.assertIn('SUDO', events)
        self.assertIn('START', events)

    def test_failed_shutdown_does_not_open_second_device(self):
        code, events = self.run_reconnect(old=True, kill_error=True)
        self.assertNotEqual(code, 0)
        self.assertNotIn('START', events)

    def test_stuck_bridge_does_not_open_second_device(self):
        code, events = self.run_reconnect(old=True, stuck=True)
        self.assertNotEqual(code, 0)
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
