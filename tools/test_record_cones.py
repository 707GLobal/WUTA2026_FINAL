"""Pure file/serializer tests; no ROS graph, sensor or CAN device."""
import csv
import json
from pathlib import Path
import sys
import threading
import time
from types import SimpleNamespace as NS
import pytest
sys.path.insert(0, str(Path(__file__).parent))
from record_cones import Recorder, serialize, sources_from_manifest, export_csv


def header():
    return NS(stamp=NS(sec=1780123456, nanosec=987654321), frame_id='rslidar')


def cone():
    return NS(position=NS(x=1.25, y=-.75, z=.1), color=2, confidence=.85)


def frame(empty=False):
    return NS(header=header(), cones=[] if empty else [cone()])


def sources():
    return {'fused': {'topic': '/custom/fused', 'type': 'ConeArray'}}


def test_serialize_all_fields_and_nonfinite():
    item = cone(); item.position.x = float('nan'); item.color = 99
    value = serialize(NS(header=header(), cones=[item]), 'ConeArray')
    assert value['header']['stamp_ns'] == '1780123456987654321'
    assert value['cones'][0]['position']['x'] is None
    assert value['cones'][0]['color'] == 99
    assert value['cones'][0]['color_name'] == 'UNRECOGNIZED'
    assert value['invalid_fields'] == ['cones[0].position.x']
    detection = NS(detection_id=17, bbox_xyxy=[1, 2, 3, 4], color_probabilities=[.1, .2, .3, .15, .25],
                   position_valid=False, position=cone().position, position_covariance=list(range(9)), confidence=.8)
    camera = serialize(NS(header=header(), detections=[detection]), 'CameraConeDetectionArray')['detections'][0]
    assert camera['position_valid'] is False and camera['detection_id'] == 17
    assert camera['bbox_xyxy'] == [1., 2., 3., 4.]
    assert len(camera['color_probabilities']) == 5 and len(camera['position_covariance']) == 9
    message = NS(header=header(), is_closed=True, red_cones=[], yellow_cones=[], blue_cones=[cone()], orange_cones=[], unknown_cones=[])
    result = serialize(message, 'ConeMap')
    assert result['is_closed'] and result['blue_cones'][0]['color_name'] == 'BLUE'


def test_rotation_restart_empty_frames_and_csv(tmp_path):
    recorder = Recorder(tmp_path, sources(), max_bytes=700)
    for _ in range(4):
        recorder.submit('fused', frame())
    recorder.submit('fused', frame(True))
    recorder.close()
    folder = tmp_path/'perception/cones'
    original_files = {p.name: p.read_bytes() for p in folder.glob('*.jsonl')}
    assert len(original_files) >= 2
    data = json.loads((folder/'summary.json').read_text())
    assert data['state'] == 'closed'
    assert data['streams']['fused']['written'] == 5
    assert data['streams']['fused']['empty'] == 1
    again = Recorder(tmp_path, sources(), max_bytes=700)
    again.submit('fused', frame()); again.close()
    assert all((folder/name).read_bytes() == payload for name, payload in original_files.items())
    assert list(folder.glob('summary_previous_*.json'))
    export_csv(folder, tmp_path/'cones.csv')
    with (tmp_path/'cones.csv').open() as stream:
        rows = list(csv.DictReader(stream))
    assert len(rows) == 6
    assert any(row['frame_count'] == '0' and row['index'] == '' for row in rows)
    assert rows[0]['stamp_ns'] == '1780123456987654321'


def test_queue_overflow_is_counted_and_drained(tmp_path):
    recorder = Recorder(tmp_path, sources(), queue_size=1)
    entered, release = threading.Event(), threading.Event()
    real = recorder.write_job
    def paused(job):
        entered.set(); assert release.wait(3); real(job)
    recorder.write_job = paused
    recorder.submit('fused', frame()); assert entered.wait(2)
    recorder.submit('fused', frame()); recorder.submit('fused', frame())
    release.set(); recorder.close()
    item = json.loads((recorder.folder/'summary.json').read_text())['streams']['fused']
    assert item['received'] == 3 and item['written'] == 2 and item['dropped'] == 1


def test_disk_failure_visible_and_duplicate_writer_rejected(tmp_path):
    recorder = Recorder(tmp_path, sources())
    with pytest.raises(RuntimeError, match='已有'):
        Recorder(tmp_path, sources())
    def disk_full(job):
        raise OSError('No space left on device')
    recorder.write_job = disk_full
    recorder.submit('fused', frame()); recorder.close()
    data = json.loads((recorder.folder/'summary.json').read_text())
    assert data['state'] == 'error' and 'space' in data['error']
    assert data['streams']['fused']['written'] == 0
    assert data['streams']['fused']['failed'] == 1


def test_manifest_uses_actual_sources_and_skips_map_for_sensor_only(tmp_path):
    path = tmp_path/'manifest.json'
    path.write_text(json.dumps({'parameters': {key: {'output_topic': '/selected/' + key}
                                             for key in ('camera', 'adapter', 'lidar', 'fusion')}}))
    values = sources_from_manifest(path)
    assert values['fused']['topic'] == '/selected/fusion'
    assert values['camera_3d']['topic'] == '/selected/adapter'
    assert values['map']['topic'] == '/mapping/cone_map'
    assert 'map' not in sources_from_manifest(path, sensor_only=True)


@pytest.mark.parametrize('sensor_only', [0, 1])
def test_launcher_owns_recorder_shutdown_without_drivers(tmp_path, sensor_only):
    import subprocess
    source = (Path(__file__).resolve().parents[1]/'start_full_fsd.sh').read_text()
    start = 'start_cone_recorder() {' + source.split('start_cone_recorder() {', 1)[1].split('\n}\n', 1)[0] + '\n}\n'
    cleanup = 'cleanup() {' + source.split('cleanup() {', 1)[1].split('\n}\n', 1)[0] + '\n}\n'
    script = tmp_path/'tools/record_cones.py'; script.parent.mkdir()
    script.write_text('''import json,signal,sys,time
from pathlib import Path
stop=False
def done(*_):
    global stop
    stop=True
signal.signal(signal.SIGINT,done)
signal.signal(signal.SIGTERM,done)
Path('arguments.json').write_text(json.dumps(sys.argv[1:]))
while not stop: time.sleep(.05)
Path('closed').touch()
''')
    harness = f'''set -eu
ROOT="$PWD"
RUN_DIR="$PWD/run"
SENSOR_ONLY={sensor_only}
BRAKE_BENCH=0
CHILD_PIDS=()
CHILD_NAMES=()
OBSERVER_PIDS=()
log_path_for() {{ echo "$PWD/recorder.log"; }}
supervisor_alive() {{ return 1; }}
'''
    tail = '''
start_cone_recorder
for ((i=0;i<50;i++)); do [[ ! -f arguments.json ]] || break; sleep .02; done
cleanup
'''
    result = subprocess.run(['bash', '-c', harness + start + cleanup + tail], cwd=tmp_path,
                            capture_output=True, text=True, timeout=8)
    assert result.returncode == 0, result.stderr
    assert (tmp_path/'closed').exists()
    args = json.loads((tmp_path/'arguments.json').read_text())
    assert ('--sensor-only' in args) == bool(sensor_only)
    assert args[args.index('--manifest')+1] == str(tmp_path/'run/configs/module_config_manifest.json')


def test_stopped_recorder_warns_without_gating_vehicle(tmp_path):
    import subprocess
    source = (Path(__file__).resolve().parents[1]/'start_full_fsd.sh').read_text()
    check = 'check_children() {' + source.split('check_children() {', 1)[1].split('\n}\n', 1)[0] + '\n}\n'
    result = subprocess.run(['bash', '-c', '''set -eu
CONE_RECORDER_PID=99999999
OBSERVER_PIDS=()
CHILD_PIDS=()
report_child_failure() { exit 9; }
''' + check + 'check_children; check_children'], capture_output=True, text=True, timeout=3)
    assert result.returncode == 0
    assert result.stderr.count('锥桶记录器退出') == 1
