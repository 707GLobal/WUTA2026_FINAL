"""Camera QoS and depth matching regression; run with ROS_DOMAIN_ID=177.

Synthetic inputs only. No camera driver, CAN bridge or controller is started.
"""
import os
import signal
import struct
import subprocess
import sys
import time
from pathlib import Path

import pytest
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, CameraInfo
from wuta_msgs.msg import CameraConeDetection, CameraConeDetectionArray

ROOT = Path(__file__).resolve().parents[1]
WS = ROOT / 'WUTA-FSD/ros2_ws'
pytestmark = pytest.mark.skipif(int(os.environ.get('ROS_DOMAIN_ID', '0')) < 100,
                                reason='Requires an isolated ROS_DOMAIN_ID >= 100')
QOS = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)


def spin_until(nodes, predicate, timeout=8):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        for node in nodes:
            rclpy.spin_once(node, timeout_sec=.01)
        if predicate():
            return
    raise AssertionError('ROS camera check timed out')


@pytest.mark.parametrize('executable', ['stereo_detection_adapter_cpp', 'stereo_detection_adapter'])
def test_adapter_reliable_inputs_preserve_exposure_and_produce_depth(executable, tmp_path):
    rclpy.init()
    probe = Node('camera_reliable_adapter_probe')
    inputs = {
        '/audit/camera/boxes': CameraConeDetectionArray,
        '/audit/camera/depth': Image,
        '/audit/camera/info': CameraInfo,
    }
    pubs = {topic: probe.create_publisher(typ, topic, QOS) for topic, typ in inputs.items()}
    output = []
    sub = probe.create_subscription(CameraConeDetectionArray, '/perception/camera/cones',
                                    output.append, QOS)
    seen_stamps = set()
    def publish():
        stamp = probe.get_clock().now().to_msg()
        info = CameraInfo()
        info.header.frame_id = 'camera_optical'
        info.header.stamp = stamp
        info.width = info.height = 100
        info.p = [100., 0., 50., 0., 0., 100., 50., 0., 0., 0., 1., 0.]
        depth = Image()
        depth.header = info.header
        depth.width = depth.height = 100
        depth.encoding = '32FC1'
        depth.step = 400
        depth.data = struct.pack('<f', 5.) * 10000
        boxes = CameraConeDetectionArray()
        boxes.header = info.header
        detection = CameraConeDetection()
        detection.bbox_xyxy = [30., 30., 70., 70.]
        detection.confidence = 1.
        detection.color_probabilities = [1., 0., 0., 0., 0.]
        boxes.detections = [detection]
        seen_stamps.add((stamp.sec, stamp.nanosec))
        pubs['/audit/camera/info'].publish(info)
        pubs['/audit/camera/depth'].publish(depth)
        pubs['/audit/camera/boxes'].publish(boxes)
    timer = probe.create_timer(.1, publish)
    with (tmp_path / 'adapter.log').open('w') as log:
        command = [str(WS / 'install/camera_detection/lib/camera_detection' / executable),
            '--ros-args', '-p', 'boxes_topic:=/audit/camera/boxes',
            '-p', 'depth_topic:=/audit/camera/depth', '-p', 'info_topic:=/audit/camera/info']
        if executable == 'stereo_detection_adapter':
            command.insert(0, sys.executable)  # Match CLI's system-Python runtime.
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)
        try:
            spin_until([probe], lambda: any(m.detections and m.detections[0].position_valid for m in output))
            assert process.poll() is None
            for topic in inputs:
                endpoints = probe.get_subscriptions_info_by_topic(topic)
                assert endpoints
                assert all(x.qos_profile.reliability == ReliabilityPolicy.RELIABLE for x in endpoints)
            result = next(m for m in output if m.detections and m.detections[0].position_valid)
            assert (result.header.stamp.sec, result.header.stamp.nanosec) in seen_stamps
            assert result.header.frame_id == 'camera_optical'
            assert result.detections[0].position.z == pytest.approx(5., abs=.05)
        finally:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill(); process.wait()
            probe.destroy_node()
            rclpy.shutdown()


def test_yolo_input_and_annotated_output_are_reliable(monkeypatch):
    from camera_detection import yolo_node
    class Model:
        names = {0: 'red', 1: 'yellow', 2: 'blue'}
        backend = 'test'
        device = 'cpu'
        providers = []
        def __init__(self, *args, **kwargs):
            pass
        def predict(self, *args):
            return []
    monkeypatch.setattr(yolo_node, 'YoloModel', Model)
    rclpy.init()
    detector = yolo_node.YoloNode()
    probe = Node('camera_reliable_yolo_probe')
    images = []
    sub = probe.create_subscription(Image, detector.cfg['annotated_topic'], images.append, QOS)
    pub = probe.create_publisher(Image, detector.cfg['image_topic'], QOS)
    def publish():
        image = Image()
        image.header.frame_id = 'camera_optical'
        image.header.stamp = probe.get_clock().now().to_msg()
        image.width = image.height = 100
        image.encoding = 'bgr8'
        image.step = 300
        image.data = bytes(30000)
        pub.publish(image)
    timer = probe.create_timer(.1, publish)
    try:
        spin_until([detector, probe], lambda: bool(images))
        subscriptions = probe.get_subscriptions_info_by_topic(detector.cfg['image_topic'])
        assert all(x.qos_profile.reliability == ReliabilityPolicy.RELIABLE for x in subscriptions)
        publishers = probe.get_publishers_info_by_topic(detector.cfg['annotated_topic'])
        assert publishers and all(x.qos_profile.reliability == ReliabilityPolicy.RELIABLE for x in publishers)
        assert images[0].header.frame_id == 'camera_optical'
    finally:
        detector.destroy_node()
        probe.destroy_node()
        rclpy.shutdown()
