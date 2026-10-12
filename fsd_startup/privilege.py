"""Use a local askpass helper without placing credentials in argv or logs."""
import os
from pathlib import Path

ASKPASS = Path(__file__).with_name('sudo_askpass.local.sh')


def sudo_command(*arguments):
    return ['sudo', '-A' if ASKPASS.is_file() else '-n', *arguments]


def sudo_environment():
    environment = dict(os.environ)
    if ASKPASS.is_file():
        environment['SUDO_ASKPASS'] = str(ASKPASS)
    return environment
