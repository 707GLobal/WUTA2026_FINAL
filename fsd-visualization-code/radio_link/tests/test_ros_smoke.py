"""Optional isolated-domain ROS 2 process smoke test; never opens real hardware."""

import os
import json
import select
import signal
import socket
import subprocess
import sys
import threading
import time
from copy import deepcopy
import pytest
import yaml

rclpy = pytest.importorskip("rclpy", reason="requires Linux ROS 2 Humble environment")
from geometry_msgs.msg import PoseStamped, TwistStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String
from rclpy.node import Node
from fsd_radio.link import Endpoint
from fsd_radio.protocol import SLAVE, Framer


class AcceptedSocket:
    def __init__(self, connection):
        self.connection = connection
        self.tx_bytes = self.rx_bytes = 0
        self.framer = Framer()
        self.packets = []

    def write(self, data):
        self.connection.sendall(data)
        self.tx_bytes += len(data)

    def receive(self, timeout):
        if self.packets:
            return self.packets.pop(0)
        ready, _, _ = select.select([self.connection], [], [], timeout)
        if not ready:
            return None
        data = self.connection.recv(2048)
        self.rx_bytes += len(data)
        self.packets.extend(self.framer.feed(data))
        return self.packets.pop(0) if self.packets else None


def test_real_ros_subscriptions_and_extension_round_trip(config, tmp_path, monkeypatch):
    monkeypatch.setenv("ROS_DOMAIN_ID", "191")
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    c = deepcopy(config)
    c["link"]["bytes_per_second"] = 240
    filename = tmp_path / "radio.yaml"
    filename.write_text(yaml.safe_dump(c), encoding="utf-8")
    listener = socket.socket()
    listener.bind(("127.0.0.1", 0))
    listener.listen(1)
    listener.settimeout(15)
    process = subprocess.Popen(
        [
            sys.executable,
            "-m",
            "fsd_radio.cli",
            "--config",
            str(filename),
            "linux",
            "--port",
            f"socket://127.0.0.1:{listener.getsockname()[1]}",
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        env=os.environ.copy(),
    )
    node = None
    connection = None
    thread = None
    stop = threading.Event()
    try:
        connection, _ = listener.accept()
        received = []
        peer = Endpoint(SLAVE, AcceptedSocket(connection), c, received.append)
        peer.outbox.extension({"type": "ping", "id": "isolated-ros-test"})
        thread = threading.Thread(target=peer.run, args=(stop,))
        thread.start()
        rclpy.init(args=[])
        node = Node("fsd_radio_isolated_test")
        path_pub = node.create_publisher(MarkerArray, c["resolved_layers"]["path"], 1)
        pose_pub = node.create_publisher(PoseStamped, c["pose_topic"], 1)
        velocity_pub = node.create_publisher(TwistStamped, c["velocity_topic"], 1)
        extensions = []
        node.create_subscription(
            String,
            "/fsd_radio/extension/rx",
            lambda msg: extensions.append(json.loads(msg.data)),
            10,
        )
        end = time.monotonic() + 25
        while time.monotonic() < end:
            stamp = node.get_clock().now().to_msg()
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = stamp
            marker.ns = "smoke"
            marker.type = Marker.LINE_STRIP
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.1
            marker.color.g = 1.0
            marker.color.a = 1.0
            marker.points = [Point(x=0.0), Point(x=2.0)]
            path_pub.publish(MarkerArray(markers=[marker]))
            pose = PoseStamped()
            pose.header = marker.header
            pose.pose.orientation.w = 1.0
            pose_pub.publish(pose)
            speed = TwistStamped()
            speed.twist.linear.x = 3.0
            speed.twist.linear.y = 4.0
            velocity_pub.publish(speed)
            rclpy.spin_once(node, timeout_sec=0.05)
            if (
                any(
                    m["k"] == "scene" and m.get("l") == "path" and m["o"]
                    for m in received
                )
                and any(m["k"] == "vehicle" and m["v"] == 5.0 for m in received)
                and any(
                    m["k"] == "extension" and m["body"].get("type") == "pong"
                    for m in received
                )
                and extensions
            ):
                break
        assert any(
            m["k"] == "scene" and m.get("l") == "path" and m["o"] for m in received
        )
        assert any(m["k"] == "vehicle" and m["v"] == 5.0 for m in received)
        assert any(
            m["k"] == "extension" and m["body"].get("type") == "pong" for m in received
        )
        assert extensions[0]["id"] == "isolated-ros-test"
    finally:
        stop.set()
        if thread:
            thread.join(2)
        if connection:
            connection.close()
        listener.close()
        process.send_signal(signal.SIGINT)
        try:
            process.communicate(timeout=8)
        except subprocess.TimeoutExpired:
            process.kill()
            process.communicate()
        if node:
            node.destroy_node()
        rclpy.try_shutdown()
