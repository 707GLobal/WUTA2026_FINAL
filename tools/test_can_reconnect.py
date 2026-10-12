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
                      kill_error=False, valid=True, foreign=False):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            if old:
                (root / 'running').touch()
            harness = r'''
set -Eeuo pipefail
ROOT="$PWD"; LOG_DIR="$PWD"; CAN_IFACE=can0
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
    : > "$LOG_DIR/zlg_can.log"
    if [[ "$READY" == 1 ]]; then echo '[INFO ] [main] 就绪:' > "$LOG_DIR/zlg_can.log"; fi
}
check_children() { :; }
check_existing_can() { [[ "$VALID" == 1 ]]; }
'''
            script = harness + RECONNECT + '\nreconnect_can_bridge\n'
            env = dict(os.environ,
                       READY=str(int(ready)), STUCK=str(int(stuck)),
                       KILL_ERROR=str(int(kill_error)), VALID=str(int(valid)),
                       FOREIGN=str(int(foreign)))
            result = subprocess.run(['bash', '-c', script], cwd=root, env=env,
                                    text=True, capture_output=True, timeout=5)
            events = (root / 'events').read_text()
            return result.returncode, events

    def test_old_bridge_stopped_before_new_bridge(self):
        code, events = self.run_reconnect(old=True)
        self.assertEqual(code, 0, events)
        self.assertLess(events.index('SUDO -n kill -TERM'), events.index('START'))
        self.assertIn('CAN 盒已重新打开并初始化', events)

    def test_first_start_still_initializes(self):
        code, events = self.run_reconnect()
        self.assertEqual(code, 0, events)
        self.assertNotIn('kill', events)
        self.assertIn('START zlg_can', events)

    def test_stuck_bridge_is_killed_before_reopening(self):
        code, events = self.run_reconnect(old=True, stuck=True)
        self.assertEqual(code, 0, events)
        self.assertLess(events.index('kill -KILL'), events.index('START'))

    def test_permission_failure_does_not_open_second_bridge(self):
        code, events = self.run_reconnect(old=True, kill_error=True)
        self.assertNotEqual(code, 0)
        self.assertNotIn('START', events)

    def test_interface_alone_cannot_pass_initialization(self):
        code, events = self.run_reconnect(ready=False)
        self.assertNotEqual(code, 0)
        self.assertIn('10 秒内未完成设备初始化', events)

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
