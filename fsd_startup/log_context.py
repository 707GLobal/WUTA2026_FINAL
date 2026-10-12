"""Filesystem-only run lifecycle. No ROS, hardware or process-management logic."""
from datetime import datetime
import json
import os
from pathlib import Path
import shutil

MISSIONS = {0: 'trackdrive', 1: 'skidpad', 2: 'acceleration',
            3: 'inspection', 4: 'ebs_test'}


def timestamp():
    return datetime.now().astimezone().isoformat()


def write_json(path, data):
    path = Path(path)
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(data, ensure_ascii=False, indent=2) + '\n')
    temporary.replace(path)


def new_directory(parent):
    parent = Path(parent)
    parent.mkdir(parents=True, exist_ok=True)
    for _ in range(100):
        path = parent / datetime.now().astimezone().strftime('%Y%m%d_%H%M%S_%f')
        try:
            path.mkdir()
            return path
        except FileExistsError:
            continue
    raise RuntimeError('无法分配唯一运行目录')


class RunLog:
    def __init__(self, root, path, data):
        self.root, self.path, self.data = Path(root), Path(path), data

    @classmethod
    def create(cls, root, mode='hardware', args=(), mission=None, path=None):
        root = Path(root).resolve()
        category = mission or ('sensor_only' if mode == 'sensor_only' else 'pending')
        if category not in {*MISSIONS.values(), 'sensor_only', 'pending'}:
            raise ValueError('未知赛项：' + category)
        if path is None:
            path = new_directory(root / 'logs' / category)
        else:
            path = Path(path).resolve()
            path.mkdir(parents=True, exist_ok=False)
        for folder in ('modules', 'configs', 'jrajectory', 'ros', 'artifacts', 'validation'):
            (path / folder).mkdir()
        data = dict(schema_version=1, run_id=path.name, started_at=timestamp(),
                    ended_at=None, mode=mode, args=list(args), mission=mission,
                    status='starting', path=str(path), archive_state='pending',
                    paths={name: name for name in ('modules', 'configs', 'jrajectory', 'ros', 'artifacts', 'validation')},
                    manager_log='manager.log')
        run = cls(root, path, data)
        run.save()
        return run

    def save(self):
        write_json(self.path / 'run.json', self.data)

    def snapshot(self, files):
        manifest = {}
        for label, source in files.items():
            source = Path(source)
            destination = self.path / 'configs' / (label + source.suffix)
            shutil.copy2(source, destination)
            manifest[label] = {'source': str(source), 'snapshot': destination.name}
        write_json(self.path / 'configs' / 'manifest.json', manifest)

    def finish(self, exit_code, reason):
        """Called by the owner only after writers and runtime processes stop."""
        history_path = self.path / 'mission_history.json'
        history = json.loads(history_path.read_text()) if history_path.exists() else []
        missions = list(dict.fromkeys(item['mission'] for item in history))
        if self.data['mission'] and self.data['mission'] not in missions:
            missions.insert(0, self.data['mission'])
        category = ('sensor_only' if self.data['mode'] == 'sensor_only' else
                    'multi_mission' if len(missions) > 1 else
                    missions[0] if missions else 'pending')
        if category not in {*MISSIONS.values(), 'sensor_only', 'multi_mission', 'pending'}:
            raise ValueError('非法归档类别')
        self.data.update(ended_at=timestamp(), exit_code=exit_code, reason=reason,
                         status='finished' if exit_code == 0 else 'failed', missions=missions,
                         mission=missions[0] if len(missions) == 1 else None, category=category)
        # Explicit user output locations (e.g. regression batches) stay in place.
        if self.path.parent == self.root / 'logs' / 'pending' and category != 'pending':
            destination = self.root / 'logs' / category / self.path.name
            destination.parent.mkdir(parents=True, exist_ok=True)
            if destination.exists():
                raise FileExistsError(destination)
            # Driver loggers can create absolute latest links within the run.
            # Make only internal links relative before moving the directory.
            for link in self.path.rglob('*'):
                if link.is_symlink():
                    target = Path(os.readlink(link))
                    if target.is_absolute() and target.is_relative_to(self.path):
                        relative = os.path.relpath(target, link.parent)
                        link.unlink()
                        link.symlink_to(relative)
            self.path.rename(destination)
            self.path = destination
        self.data.update(path=str(self.path), archive_state='complete')
        self.save()
        return self.path
