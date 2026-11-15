"""Isolated graph tests: only synthetic cone messages, no drivers or CAN."""
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import rclpy
from wuta_msgs.msg import Cone, ConeArray, CameraConeDetection, CameraConeDetectionArray, ConeMap

ROOT = Path(__file__).resolve().parents[1]


def test_all_streams_record_without_go_and_no_business_publishers(tmp_path):
    assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
    assert os.environ.get('ROS_DOMAIN_ID') not in (None, '0')
    manifest = tmp_path/'manifest.json'
    manifest.write_text(json.dumps({'parameters': {key: {'output_topic': '/record_test/' + key}
                                                   for key in ('camera', 'adapter', 'lidar', 'fusion')}}))
    log = (tmp_path/'recorder.log').open('w')
    process = subprocess.Popen(['/usr/bin/python3', str(ROOT/'tools/record_cones.py'),
                               '--run-dir', str(tmp_path), '--manifest', str(manifest), '--max-file-mb', '.002'],
                               stdout=log, stderr=subprocess.STDOUT)
    rclpy.init()
    probe = rclpy.create_node('cone_record_test')
    publishers = {
        'camera': probe.create_publisher(CameraConeDetectionArray, '/record_test/camera', 10),
        'camera_3d': probe.create_publisher(CameraConeDetectionArray, '/record_test/adapter', 10),
        'lidar': probe.create_publisher(ConeArray, '/record_test/lidar', 10),
        'fused': probe.create_publisher(ConeArray, '/record_test/fusion', 10),
        'map': probe.create_publisher(ConeMap, '/mapping/cone_map', 10)}
    summary = tmp_path/'perception/cones/summary.json'
    try:
        end = time.monotonic() + 8
        while not all(pub.get_subscription_count() for pub in publishers.values()) and time.monotonic() < end:
            rclpy.spin_once(probe, timeout_sec=.1)
            assert process.poll() is None, (tmp_path/'recorder.log').read_text()
        assert all(pub.get_subscription_count() for pub in publishers.values())
        one = Cone(); one.position.x = 2.25; one.color = 2; one.confidence = .875
        camera_one = CameraConeDetection(); camera_one.detection_id = 19
        camera_one.bbox_xyxy = [1., 2., 3., 4.]; camera_one.confidence = .75
        camera_one.color_probabilities = [.1, .2, .3, .2, .2]; camera_one.position_valid = False
        for index in range(12):
            for key, pub in publishers.items():
                message = CameraConeDetectionArray() if key.startswith('camera') else ConeMap() if key == 'map' else ConeArray()
                message.header.stamp.sec = 1780123456
                message.header.stamp.nanosec = 987654321
                message.header.frame_id = 'map' if key == 'map' else 'rslidar'
                if index % 2:
                    if key.startswith('camera'): message.detections = [camera_one]
                    elif key == 'map': message.blue_cones = [one]; message.is_closed = True
                    else: message.cones = [one]
                pub.publish(message)
            rclpy.spin_once(probe, timeout_sec=.05); time.sleep(.05)
        end = time.monotonic() + 5
        while time.monotonic() < end:
            data = json.loads(summary.read_text())
            if all(item['written'] >= 10 for item in data['streams'].values()): break
            rclpy.spin_once(probe, timeout_sec=.1)
        assert all(item['written'] >= 10 for item in data['streams'].values())
        assert all(item['empty'] >= 1 for item in data['streams'].values())
        for topic in ('/control/command', '/system/mission_state', '/system/start_command', '/system/lap_count'):
            assert probe.count_publishers(topic) == 0
        assert probe.count_publishers('/record_test/fusion') == 1  # Only our fixture publisher.
        process.send_signal(signal.SIGINT); process.send_signal(signal.SIGINT)
        assert process.wait(timeout=5) == 0
    finally:
        if process.poll() is None:
            process.terminate()
            try: process.wait(timeout=5)
            except subprocess.TimeoutExpired: process.kill(); process.wait()
        log.close(); probe.destroy_node(); rclpy.shutdown()
    data = json.loads(summary.read_text())
    assert data['state'] == 'closed'
    folder = summary.parent
    for key in publishers:
        frames = [json.loads(line) for path in folder.glob(key + '_[0-9][0-9][0-9][0-9][0-9][0-9].jsonl') for line in path.read_text().splitlines()]
        assert len(frames) == data['streams'][key]['written']
        assert all(frame['header']['stamp_ns'] == '1780123456987654321' for frame in frames)
        assert any(frame['count'] == 0 for frame in frames)
    camera_frames = [json.loads(line) for path in folder.glob('camera_3d_*.jsonl') for line in path.read_text().splitlines()]
    assert any(frame['detections'] and not frame['detections'][0]['position_valid'] for frame in camera_frames)
