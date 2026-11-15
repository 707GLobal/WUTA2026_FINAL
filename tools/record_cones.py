#!/usr/bin/env python3
"""Read-only cone message journal. RunLog owns run creation, cleanup and archival."""
import argparse
import copy
import csv
import uuid
import fcntl
import json
import math
import os
from pathlib import Path
import queue
import signal
import threading
import time
import traceback

COLORS = ('RED', 'YELLOW', 'BLUE', 'ORANGE', 'UNKNOWN')
TYPES = {'camera': 'CameraConeDetectionArray', 'camera_3d': 'CameraConeDetectionArray',
         'lidar': 'ConeArray', 'fused': 'ConeArray', 'map': 'ConeMap'}


def sources_from_manifest(path, sensor_only=False):
    """Use the exact source configuration snapshot selected by the launcher."""
    parameters = json.loads(Path(path).read_text())['parameters']
    result = {key: {'topic': parameters[module]['output_topic'], 'type': TYPES[key]}
              for key, module in (('camera', 'camera'), ('camera_3d', 'adapter'),
                                  ('lidar', 'lidar'), ('fused', 'fusion'))}
    if not sensor_only:
        # cone_map_builder publishes this authoritative topic without a parameter.
        result['map'] = {'topic': '/mapping/cone_map', 'type': TYPES['map']}
    if any(not isinstance(item['topic'], str) or not item['topic'].startswith('/') for item in result.values()):
        raise ValueError('锥桶记录话题必须为配置中的绝对ROS话题')
    return result


def cone_count(message, kind):
    if kind == 'CameraConeDetectionArray':
        return len(message.detections)
    if kind == 'ConeArray':
        return len(message.cones)
    return sum(len(getattr(message, color.lower() + '_cones')) for color in COLORS)


def serialize(message, kind):
    """Preserve source fields; mark nonfinite scalars as null with their field paths."""
    invalid = []

    def number(value, path):
        value = float(value)
        if math.isfinite(value):
            return value
        invalid.append(path)
        return None

    def position(point, path):
        return {axis: number(getattr(point, axis), path + '.' + axis) for axis in ('x', 'y', 'z')}

    def cone(item, path):
        color = int(item.color)
        return {'position': position(item.position, path + '.position'), 'color': color,
                'color_name': COLORS[color] if 0 <= color < len(COLORS) else 'UNRECOGNIZED',
                'confidence': number(item.confidence, path + '.confidence')}

    header = message.header
    result = {'header': {'stamp': {'sec': int(header.stamp.sec), 'nanosec': int(header.stamp.nanosec)},
                         'stamp_ns': str(int(header.stamp.sec) * 10**9 + int(header.stamp.nanosec)),
                         'frame_id': header.frame_id}}
    if kind == 'CameraConeDetectionArray':
        detections = []
        for index, item in enumerate(message.detections):
            path = f'detections[{index}]'
            detections.append({'detection_id': int(item.detection_id),
                'bbox_xyxy': [number(v, f'{path}.bbox_xyxy[{i}]') for i, v in enumerate(item.bbox_xyxy)],
                'color_probabilities': [number(v, f'{path}.color_probabilities[{i}]') for i, v in enumerate(item.color_probabilities)],
                'confidence': number(item.confidence, path + '.confidence'),
                'position_valid': bool(item.position_valid), 'position': position(item.position, path + '.position'),
                'position_covariance': [number(v, f'{path}.position_covariance[{i}]') for i, v in enumerate(item.position_covariance)]})
        result['detections'] = detections
    elif kind == 'ConeArray':
        result['cones'] = [cone(item, f'cones[{i}]') for i, item in enumerate(message.cones)]
    else:
        result['is_closed'] = bool(message.is_closed)
        for color in COLORS:
            key = color.lower() + '_cones'
            result[key] = [cone(item, f'{key}[{i}]') for i, item in enumerate(getattr(message, key))]
    result['invalid_fields'] = invalid
    return result


def atomic_json(path, data):
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(data, ensure_ascii=False, allow_nan=False) + '\n', encoding='utf-8')
    temporary.replace(path)


class Recorder:
    def __init__(self, run_dir, sources, queue_size=256, max_bytes=128*1024*1024, max_frame_bytes=8*1024*1024):
        if min(queue_size, max_bytes, max_frame_bytes) <= 0:
            raise ValueError('记录队列及文件大小必须为正数')
        self.folder = Path(run_dir) / 'perception/cones'
        self.folder.mkdir(parents=True, exist_ok=True)
        self.file_lock = (self.folder / '.writer.lock').open('a')
        try:
            fcntl.flock(self.file_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError:
            self.file_lock.close()
            raise RuntimeError('本次运行已有锥桶记录器')
        self.session_id = uuid.uuid4().hex
        old_summary = self.folder / "summary.json"
        if old_summary.exists():
            old_summary.rename(self.folder / ("summary_previous_" + self.session_id + ".json"))
        self.sources = sources
        self.queue = queue.Queue(maxsize=queue_size)
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.max_bytes, self.max_frame_bytes = max_bytes, min(max_frame_bytes, max_bytes)
        self.files, self.parts, self.sizes, self.pending = {}, {}, {}, {}
        self.state, self.error, self.accepting = 'starting', '', True
        self.stats = {key: dict(item, received=0, written=0, dropped=0, failed=0, empty=0,
                               last_receive_ns=None, last_write_ns=None, last_count=None,
                               file=None, error='') for key, item in sources.items()}
        self.started_at = time.time()
        self.process = {'pid': os.getpid(), 'start': Path('/proc/self/stat').read_text().rsplit(')', 1)[1].split()[19]}
        self.snapshot()
        self.worker = threading.Thread(target=self.write_loop, name='cone_journal', daemon=True)
        self.worker.start()

    def submit(self, source, message):
        with self.lock:
            if not self.accepting:
                return
            item = self.stats[source]
            item['received'] += 1
            item['last_receive_ns'] = str(time.time_ns())
            item['last_count'] = cone_count(message, item['type'])
            item['empty'] += int(item['last_count'] == 0)
            job = (source, item['received'], item['last_receive_ns'], item['last_count'], message)
            try:
                self.queue.put_nowait(job)
            except queue.Full:
                item['dropped'] += 1

    def snapshot(self):
        with self.lock:
            data = {'schema': 1, 'session_id': self.session_id, 'state': self.state, 'error': self.error, 'process': self.process,
                    'started_at': self.started_at, 'updated_at': time.time(),
                    'output': 'perception/cones', 'qos': 'reliable/volatile/keep_last_10',
                    'queue_capacity': self.queue.maxsize, 'queue_pending': self.queue.qsize(),
                    'max_file_bytes': self.max_bytes, 'max_frame_bytes': self.max_frame_bytes,
                    'streams': copy.deepcopy(self.stats)}
        atomic_json(self.folder / 'summary.json', data)
        return data

    def fail(self, error):
        with self.lock:
            self.error = str(error)
            self.state = 'error'
        print('[锥桶记录异常] ' + str(error), flush=True)

    def flush_files(self):
        # Only acknowledge rows after flush succeeds. No fsync/power-loss guarantee.
        for source, stream in self.files.items():
            stream.flush()
            count = self.pending.pop(source, 0)
            if count:
                with self.lock:
                    self.stats[source]['written'] += count
                    self.stats[source]['last_write_ns'] = str(time.time_ns())

    def write_job(self, job):
        source, index, received_ns, count, message = job
        kind = self.sources[source]['type']
        frame = dict(schema=1, session_id=self.session_id, source=source, topic=self.sources[source]['topic'],
                     message_type='wuta_msgs/msg/' + kind, frame_index=index,
                     received_ns=received_ns, count=count, **serialize(message, kind))
        line = (json.dumps(frame, ensure_ascii=False, allow_nan=False) + '\n').encode('utf-8')
        if len(line) > self.max_frame_bytes:
            raise ValueError(f'{source} 单帧超过 {self.max_frame_bytes} 字节上限')
        if source not in self.files or self.sizes[source] + len(line) > self.max_bytes:
            if source in self.files:
                self.flush_files()
                self.files.pop(source).close()
            part = self.parts.get(source, 0) + 1
            while True:
                path = self.folder / f'{source}_{part:06d}.jsonl'
                try:
                    stream = path.open('xb')
                    break
                except FileExistsError:
                    part += 1
            self.parts[source], self.files[source], self.sizes[source] = part, stream, 0
            with self.lock:
                self.stats[source]['file'] = path.name
        self.files[source].write(line)
        self.sizes[source] += len(line)
        self.pending[source] = self.pending.get(source, 0) + 1

    def write_loop(self):
        with self.lock:
            if not self.error:
                self.state = 'running'
        last_flush = time.monotonic()
        try:
            while not self.stop.is_set() or not self.queue.empty():
                try:
                    job = self.queue.get(timeout=.1)
                except queue.Empty:
                    job = None
                if job:
                    source = job[0]
                    try:
                        self.write_job(job)
                    except ValueError as exc:
                        with self.lock:
                            self.stats[source]['failed'] += 1
                            self.stats[source]['error'] = str(exc)
                        print('[锥桶记录异常] ' + str(exc), flush=True)
                    except OSError:
                        with self.lock:
                            self.stats[source]['failed'] += 1
                        raise
                if time.monotonic() - last_flush >= .5:
                    self.flush_files()
                    self.snapshot()
                    last_flush = time.monotonic()
            self.flush_files()
        except Exception as exc:
            self.fail(exc)
            with self.lock:
                self.accepting = False
                for source, count in self.pending.items():
                    self.stats[source]['failed'] += count
                self.pending.clear()
            while True:
                try:
                    source = self.queue.get_nowait()[0]
                    with self.lock:
                        self.stats[source]['failed'] += 1
                except queue.Empty:
                    break
        finally:
            for stream in self.files.values():
                try:
                    stream.close()
                except OSError as exc:
                    self.fail(exc)
            with self.lock:
                self.state = 'error' if self.error else 'closed'
            try:
                self.snapshot()
            except OSError as exc:
                print('[锥桶记录异常] 状态摘要无法写入：' + str(exc), flush=True)

    def close(self):
        with self.lock:
            self.accepting = False
        self.stop.set()
        self.worker.join()
        self.file_lock.close()


def export_csv(folder, destination):
    fields = ('session_id', 'source', 'topic', 'frame_index', 'stamp_ns', 'received_ns',
              'frame_id', 'frame_count', 'index', 'group', 'color', 'color_name',
              'x', 'y', 'z', 'confidence', 'detection_id', 'position_valid',
              'bbox_xyxy', 'color_probabilities', 'position_covariance', 'is_closed', 'invalid_fields')
    with Path(destination).open('x', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fields)
        writer.writeheader()
        for path in sorted(Path(folder).glob('*.jsonl')):
            with path.open(encoding='utf-8') as source:
                for line in source:
                    try:
                        frame = json.loads(line)
                    except ValueError as exc:
                        raise ValueError(f'{path}: JSONL包含不完整/损坏记录，CSV导出不完整') from exc
                    groups = [('detections', frame['detections'])] if 'detections' in frame else (
                        [('cones', frame['cones'])] if 'cones' in frame else
                        [(color.lower() + '_cones', frame[color.lower() + '_cones']) for color in COLORS])
                    entries = [(group, i, cone) for group, cones in groups for i, cone in enumerate(cones)]
                    # Keep an explicit row for empty frames too.
                    for group, index, cone in entries or [('', '', {})]:
                        row = {key: frame.get(key, '') for key in ('session_id', 'source', 'topic', 'frame_index', 'received_ns', 'is_closed')}
                        row.update(stamp_ns=frame['header']['stamp_ns'], frame_id=frame['header']['frame_id'],
                                   frame_count=frame['count'], index=index, group=group)
                        row.update({key: cone.get(key, '') for key in ('color', 'color_name', 'confidence', 'detection_id', 'position_valid')})
                        row.update({axis: cone.get('position', {}).get(axis) for axis in ('x', 'y', 'z')})
                        for key in ('bbox_xyxy', 'color_probabilities', 'position_covariance'):
                            row[key] = json.dumps(cone[key]) if key in cone else ''
                        row['invalid_fields'] = json.dumps(frame['invalid_fields'])
                        writer.writerow(row)


def dependencies():
    import rclpy
    from wuta_msgs.msg import ConeArray, CameraConeDetectionArray, ConeMap
    return rclpy, {cls.__name__: cls for cls in (ConeArray, CameraConeDetectionArray, ConeMap)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--export-csv', type=Path, help='离线导出指定cones目录的JSONL')
    parser.add_argument('--output', type=Path, help='CSV目标文件，须为新文件')
    parser.add_argument('--check-dependencies', action='store_true')
    parser.add_argument('--run-dir', type=Path)
    parser.add_argument('--manifest', type=Path)
    parser.add_argument('--sensor-only', action='store_true')
    parser.add_argument('--queue-size', type=int, default=256)
    parser.add_argument('--max-file-mb', type=float, default=128)
    args = parser.parse_args()
    if args.export_csv:
        if not args.output:
            parser.error('--export-csv 需要 --output')
        export_csv(args.export_csv, args.output)
        return 0
    if args.check_dependencies:
        dependencies()
        print('rclpy / ConeArray / CameraConeDetectionArray / ConeMap 导入通过')
        return 0
    if not args.run_dir or not args.manifest:
        parser.error('--run-dir 和 --manifest 必填')
    recorder = None
    node = None
    rclpy = None
    quit_requested = False
    def request_stop(*_):
        nonlocal quit_requested
        quit_requested = True
    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    try:
        recorder = Recorder(args.run_dir, sources_from_manifest(args.manifest, args.sensor_only),
                            queue_size=args.queue_size, max_bytes=int(args.max_file_mb * 1024**2))
        rclpy, messages = dependencies()
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        node = rclpy.create_node('fsd_cone_recorder', enable_rosout=False,
                                 start_parameter_services=False)
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        subscriptions = [node.create_subscription(messages[item['type']], item['topic'],
                         lambda message, key=key: recorder.submit(key, message), qos)
                         for key, item in recorder.sources.items()]
        print('[锥桶记录] 已订阅，等待数据；输出：' + str(recorder.folder), flush=True)
        while not quit_requested and rclpy.ok() and recorder.worker.is_alive():
            rclpy.spin_once(node, timeout_sec=.1)
    except Exception as exc:
        if recorder:
            recorder.fail(exc)
        traceback.print_exc()
        return 1
    finally:
        if node:
            node.destroy_node()
        if recorder:
            recorder.close()
        if rclpy and rclpy.ok():
            rclpy.shutdown()
    return 1 if recorder.error else 0


if __name__ == '__main__':
    raise SystemExit(main())
