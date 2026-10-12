"""PID identities prevent cleanup from targeting unrelated reused PIDs."""
import os
from pathlib import Path
import signal
import subprocess
import time
from privilege import sudo_command, sudo_environment


def identity(pid, proc=Path('/proc')):
    try:
        raw = (proc / str(pid) / 'stat').read_text()
        fields = raw.rsplit(')', 1)[1].split()
        return {'pid': int(pid), 'start': fields[19], 'ppid': int(fields[1]),
                'group': int(fields[2]), 'state': fields[0]}
    except (OSError, ValueError, IndexError):
        return None


def matches(record):
    if not record:
        return False
    current = identity(record['pid'])
    return current is not None and current['start'] == record['start'] and current['state'] != 'Z'


def descendants(root):
    entries = {}
    for path in Path('/proc').iterdir():
        if path.name.isdigit():
            item = identity(path.name)
            if item:
                entries[item['pid']] = item
    selected = {root}
    while True:
        extra = {pid for pid, item in entries.items() if item['ppid'] in selected}
        if extra.issubset(selected):
            break
        selected.update(extra)
    return [entries[pid] for pid in selected if pid in entries]


def send(records, signum):
    errors = []
    for item in records:
        if not matches(item):
            continue
        try:
            os.kill(item['pid'], signum)
        except ProcessLookupError:
            pass
        except PermissionError:
            try:
                if not matches(item):
                    continue
                result = subprocess.run(sudo_command('kill', '-s', str(signum), '--', str(item['pid'])),
                                        env=sudo_environment(), capture_output=True, text=True, timeout=3)
                if result.returncode and matches(item):
                    errors.append(f"PID={item['pid']}：{result.stderr.strip()}")
            except (OSError, subprocess.TimeoutExpired) as exc:
                errors.append(f"PID={item['pid']}：{exc}")
    return errors


def cleanup(records, grace=5):
    errors = []
    for sig, duration in ((signal.SIGINT, grace), (signal.SIGTERM, 2), (signal.SIGKILL, 1)):
        live = [item for item in records if matches(item)]
        if not live:
            break
        errors.extend(send(live, sig))
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline and any(matches(item) for item in live):
            time.sleep(0.1)
    remaining = [item for item in records if matches(item)]
    return remaining, errors
