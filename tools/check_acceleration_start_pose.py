"""Exercise the installed planner in an isolated ROS domain; no vehicle nodes.

Run with system Python after sourcing ROS Humble and the FSD install workspace.
"""
import math
import os
from pathlib import Path
import subprocess
import tempfile
import time

os.environ['ROS_DOMAIN_ID'] = '227'
os.environ['ROS_LOCALHOST_ONLY'] = '1'

import rclpy
from autoware_msgs.msg import Lane
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from wuta_msgs.msg import MissionState

ROOT = Path(__file__).resolve().parents[1]
WS = ROOT / 'WUTA-FSD/ros2_ws'
PREFIX = '/test_acceleration_anchor'
rclpy.init()
node = rclpy.create_node('check_acceleration_start_pose')
received = []
node.create_subscription(Lane, PREFIX + '/path', received.append, 10)
pose_pub = node.create_publisher(PoseStamped, PREFIX + '/pose', 10)
ready_pub = node.create_publisher(Bool, PREFIX + '/ready', 10)
state_pub = node.create_publisher(MissionState, PREFIX + '/state', 10)


def spin(seconds=0.12):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.02)


def state(active=True):
    msg = MissionState()
    msg.mission_mode = MissionState.MISSION_ACCELERATION
    msg.state = MissionState.RACE if active else MissionState.READY
    state_pub.publish(msg)
    spin()


def pose(x, y, yaw, valid=True):
    msg = PoseStamped()
    msg.header.frame_id = 'map'
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.pose.position.x, msg.pose.position.y = float(x), float(y)
    if valid:
        msg.pose.orientation.z = math.sin(yaw / 2)
        msg.pose.orientation.w = math.cos(yaw / 2)
    else:
        msg.pose.orientation.w = 0.0
    pose_pub.publish(msg)
    spin()


def ready(value):
    ready_pub.publish(Bool(data=value))
    spin()


def request_path():
    received.clear()
    state()
    assert received, 'No acceleration path received'
    return received[-1]


def check_path(lane, x0, y0, yaw):
    assert lane.header.frame_id == 'map'
    assert len(lane.waypoints) > 3
    distances = []
    for wp in lane.waypoints:
        p = wp.pose.pose.position
        dx, dy = p.x - x0, p.y - y0
        forward = math.cos(yaw) * dx + math.sin(yaw) * dy
        lateral = -math.sin(yaw) * dx + math.cos(yaw) * dy
        assert abs(lateral) < 1e-6, (forward, lateral)
        q = wp.pose.pose.orientation
        heading = math.atan2(2 * q.w * q.z, 1 - 2 * q.z * q.z)
        delta = math.atan2(math.sin(heading - yaw), math.cos(heading - yaw))
        assert abs(delta) < 1e-6
        distances.append(forward)
    assert abs(distances[0] + 0.3) < 1e-6
    assert abs(distances[-1] - 20) < 1e-6
    assert all(b >= a - 1e-6 for a, b in zip(distances, distances[1:]))
    assert lane.waypoints[-1].twist.twist.linear.x == 0


def geometry(lane):
    return [(w.pose.pose, w.twist.twist) for w in lane.waypoints]


def run(frame):
    remaps = {
        '/system/mission_state': PREFIX + '/state',
        '/localization/pose': PREFIX + '/pose',
        '/system/localization_ready': PREFIX + '/ready',
        '/planning/final_waypoints': PREFIX + '/path',
        '/planning/final_waypoints_viz': PREFIX + '/viz',
        '/planning/driven_trajectory_viz': PREFIX + '/driven_viz',
    }
    cmd = [str(WS / 'install/path_generator/lib/path_generator/path_generator_node'),
           '--ros-args', '--params-file',
           str(WS / 'src/planning/path_generator/config/path_generator.yaml'),
           '-p', 'record.enabled:=false', '-p', f'acceleration.anchor_frame:={frame}']
    for src, dst in remaps.items():
        cmd += ['-r', f'{src}:={dst}']
    with tempfile.TemporaryFile(mode='w+') as log:
        process = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT)
        try:
            deadline = time.monotonic() + 8
            while state_pub.get_subscription_count() == 0:
                assert process.poll() is None, 'Planner exited before discovery'
                assert time.monotonic() < deadline, 'ROS discovery timed out'
                spin(0.1)
            spin(0.3)
            state(False)
            received.clear()
            if frame == 'map':
                check_path(request_path(), 0, 0, 0)
                print('PASS: map compatibility')
                return
            state()
            assert not received, 'Path generated without localization'
            pose(0.75, -0.29, math.radians(-173))
            ready(False)
            state()
            assert not received, 'Path generated when localization not ready'
            ready(True)
            pose(0.75, -0.29, 0, valid=False)
            state()
            assert not received, 'Path generated with zero quaternion'
            pose(0.75, -0.29, math.radians(-173))
            spin(0.65)
            state()
            assert not received, 'Path generated from stale localization'
            pose(0.75, -0.29, math.radians(-173))
            first = request_path()
            check_path(first, 0.75, -0.29, math.radians(-173))
            pose(8, 9, 0.5)
            assert geometry(first) == geometry(request_path()), 'Path moved during mission'
            print('PASS: -173 degrees, invalid/stale pose guards, frozen path')
            for degrees in (0, 90, 180, -90):
                state(False)
                yaw = math.radians(degrees)
                pose(3, 4, yaw)
                check_path(request_path(), 3, 4, yaw)
                print(f'PASS: new mission reanchors at {degrees} degrees')
        except BaseException:
            log.seek(0)
            print(log.read())
            raise
        finally:
            process.terminate()
            try:
                process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
            spin(0.3)


try:
    run('start_pose')
    run('map')
finally:
    node.destroy_node()
    rclpy.shutdown()
