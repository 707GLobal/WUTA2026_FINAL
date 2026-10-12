#!/usr/bin/env python3
"""Keep sudo's child alive and forward shutdown to a separate process group."""
import os
import signal
import subprocess
import sys


def main(argv):
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
