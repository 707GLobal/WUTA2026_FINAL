"""Verify the delivered launcher, installed entry point and clean process exit."""
from pathlib import Path
import os
import signal
import subprocess
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String


def test_installed_headless_launch(tmp_path):
    root = Path(__file__).resolve().parents[1]
    log_path = tmp_path/'launch.log'
    rclpy.init(args=[])
    probe = Node('fsd_launch_probe')
    messages = []
    subscription = probe.create_subscription(String, '/fsd_visualization/status', messages.append,
                                             qos_profile_sensor_data)
    with log_path.open('w') as stream:
        process = subprocess.Popen(['bash', str(root/'start.sh'), 'mode:=bridge', 'rviz:=false'],
                                   stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            deadline = time.monotonic()+10
            while time.monotonic()<deadline and not messages:
                assert process.poll() is None, log_path.read_text()
                rclpy.spin_once(probe,timeout_sec=.1)
            assert messages, log_path.read_text()
        finally:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=6)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
                raise AssertionError('Launcher did not shut down cleanly')
            probe.destroy_node()
            rclpy.try_shutdown()
    assert 'Traceback' not in log_path.read_text(), log_path.read_text()
