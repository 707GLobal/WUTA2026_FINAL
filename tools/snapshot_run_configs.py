#!/usr/bin/env python3
"""Copy declared input configs; never claim these are queried node parameters."""
import json
from pathlib import Path
import shutil
import sys


def snapshot(manifest_path, destination, *extra):
    manifest = json.loads(Path(manifest_path).read_text())
    destination = Path(destination)
    files = dict(manifest['files'])
    files.update(item.split('=', 1) for item in extra)
    for name, source in files.items():
        source = Path(source)
        shutil.copy2(source, destination / (name + source.suffix))


if __name__ == '__main__':
    snapshot(*sys.argv[1:])
