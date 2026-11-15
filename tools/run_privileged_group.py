#!/usr/bin/env python3
"""Keep sudo's child alive and forward shutdown to a separate process group."""
import os
from pathlib import Path
import select
import signal
import subprocess
import sys


def stop_bridge(pid, expected_start, iface):
    """Signal only the inspected bridge, even if its numeric PID is reused."""
    try:
        fd = os.pidfd_open(pid)
    except ProcessLookupError:
        return 0
    try:
        proc = Path(f'/proc/{pid}')
        try:
            fields = (proc / 'stat').read_text().rsplit(')', 1)[1].split()
            if fields[0] == 'Z':
                return 0
            start = fields[19]
            executable = (proc / 'exe').readlink().name.removesuffix(' (deleted)')
            args = (proc / 'cmdline').read_bytes().decode().split('\0')
        except FileNotFoundError:
            return 0
        selected = 'can0'
        for index, arg in enumerate(args):
            if arg == '--iface' and index + 1 < len(args):
                selected = args[index + 1]
            elif arg.startswith('--iface='):
                selected = arg.split('=', 1)[1]
        if start != str(expected_start) or executable != 'zlg_can_bridge' or selected != iface:
            print('Bridge identity changed; refusing to signal another process.', file=sys.stderr)
            return 1
        poller = select.poll()
        poller.register(fd, select.POLLIN)
        signal.pidfd_send_signal(fd, signal.SIGTERM)
        if not poller.poll(5000):
            print(f'Bridge PID={pid} did not exit; sending SIGKILL.', flush=True)
            signal.pidfd_send_signal(fd, signal.SIGKILL)
            if not poller.poll(2000):
                print('Bridge has not exited; refusing a second device open.', file=sys.stderr)
                return 1
        return 0
    except ProcessLookupError:
        return 0
    finally:
        os.close(fd)


def main(argv):
    if argv and argv[0] == '--stop-bridge':
        if len(argv) != 4:
            return 2
        try:
            return stop_bridge(int(argv[1]), argv[2], argv[3])
        except (OSError, ValueError, UnicodeError) as error:
            print(f'Cannot stop old CAN bridge: {error}', file=sys.stderr)
            return 1
    if not argv:
        print('Missing child command', file=sys.stderr)
        return 2
    child = None
    pending = []

    def forward(signum, _frame):
        if child is None:
            pending.append(signum)
            return
        try:
            os.killpg(child.pid, signum)
        except ProcessLookupError:
            pass

    signal.signal(signal.SIGINT, forward)
    signal.signal(signal.SIGTERM, forward)
    try:
        child = subprocess.Popen(argv, start_new_session=True)
    except OSError as error:
        print(f'Cannot start CAN bridge: {error}', file=sys.stderr)
        return 127
    for signum in pending:
        forward(signum, None)
    code = child.wait()
    return code if code >= 0 else 128 - code


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
