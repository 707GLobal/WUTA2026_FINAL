"""Verify fixed-path start anchoring and Trackdrive rotation equivalence.

Exercises the installed planner in isolated ROS domain 227; no vehicle nodes.

Run with system Python after sourcing ROS Humble and the FSD install workspace.
"""
import csv
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
from std_msgs.msg import Bool, Float32
from wuta_msgs.msg import MissionState
from rclpy.qos import QoSProfile, DurabilityPolicy

ROOT = Path(__file__).resolve().parents[1]
WS = ROOT / 'WUTA-FSD/ros2_ws'
PREFIX = '/test_fixed_path_anchor'
mode = MissionState.MISSION_ACCELERATION
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
    msg.mission_mode = mode
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
    assert received, 'No path received'
    return received[-1]


def compare_path(lane, baseline, x0, y0, yaw):
    assert lane.header.frame_id == 'map'
    assert len(lane.waypoints) == len(baseline.waypoints)
    for wp, original in zip(lane.waypoints, baseline.waypoints):
        p = wp.pose.pose.position
        dx, dy = p.x - x0, p.y - y0
        forward = math.cos(yaw) * dx + math.sin(yaw) * dy
        lateral = -math.sin(yaw) * dx + math.cos(yaw) * dy
        assert abs(forward - original.pose.pose.position.x) < 1e-6
        assert abs(lateral - original.pose.pose.position.y) < 1e-6
        q, q0 = wp.pose.pose.orientation, original.pose.pose.orientation
        heading = math.atan2(2*q.w*q.z, 1-2*q.z*q.z)
        heading0 = math.atan2(2*q0.w*q0.z, 1-2*q0.z*q0.z)
        delta = heading - heading0 - yaw
        assert abs(math.atan2(math.sin(delta), math.cos(delta))) < 1e-6
        assert wp.twist.twist == original.twist.twist


def geometry(lane):
    return [(w.pose.pose, w.twist.twist) for w in lane.waypoints]


def run(frame, mission, baseline=None, u_turn=False):
    global mode
    mode = getattr(MissionState, "MISSION_EBS_TEST" if mission == "ebs" else "MISSION_" + mission.upper())
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
           '-p', 'record.enabled:=true', '-p', f'{mission}.anchor_frame:={frame}',
           '-p', f'skidpad.u_turn_test.enabled:={str(u_turn).lower()}']
    for src, dst in remaps.items():
        cmd += ['-r', f'{src}:={dst}']
    with tempfile.TemporaryDirectory(prefix='fixed_path_record_') as record_dir, \
            tempfile.TemporaryFile(mode='w+') as log:
        cmd += ['-p', f'record.run_dir:={record_dir}']
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
                result = request_path()
                assert result.waypoints[-1].twist.twist.linear.x == 0
                if mission == 'skidpad':
                    assert abs(result.waypoints[0].pose.pose.position.x) < 1e-6
                    if u_turn:
                        assert abs(result.waypoints[-1].pose.pose.position.y - 18.25) < 1e-6
                    else:
                        assert abs(result.waypoints[-1].pose.pose.position.x - 40) < 1e-6
                        phases = [w.pose.pose.position.y for w in result.waypoints]
                        assert min(phases) < -18 and max(phases) > 18
                print(f'PASS: {mission} u_turn={u_turn} map compatibility')
                return result
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
            compare_path(first, baseline, 0.75, -0.29, math.radians(-173))
            with (Path(record_dir) / 'trajectory/planned.csv').open() as f:
                records = list(csv.DictReader(f))
            assert len(records) == len(first.waypoints)
            for row, waypoint in zip(records, first.waypoints):
                p, q = waypoint.pose.pose.position, waypoint.pose.pose.orientation
                assert abs(float(row['x_m']) - p.x) < 1e-6
                assert abs(float(row['y_m']) - p.y) < 1e-6
                heading = math.atan2(2*q.w*q.z, 1-2*q.z*q.z)
                delta = float(row['yaw_rad']) - heading
                assert abs(math.atan2(math.sin(delta), math.cos(delta))) < 1e-6
                assert abs(float(row['target_speed_mps']) - waypoint.twist.twist.linear.x) < 1e-6
            pose(8, 9, 0.5)
            assert geometry(first) == geometry(request_path()), 'Path moved during mission'
            print(f'PASS: {mission} u_turn={u_turn}: -173 degrees, invalid/stale guards, frozen path and map CSV')
            for degrees in (0, 90, 180, -90):
                state(False)
                yaw = math.radians(degrees)
                pose(3, 4, yaw)
                compare_path(request_path(), baseline, 3, 4, yaw)
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


# Trackdrive consumes an already-map Lane; rotating both pose and centerline
# must preserve its speeds and local/global geometry without another anchoring.
center_pub = node.create_publisher(Lane, '/planning/centerline', 10)
confidence_pub = node.create_publisher(Float32, '/system/localization_confidence', 10)
path_conf_pub = node.create_publisher(Float32, '/planning/path_confidence',
    QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))


def run_trackdrive(degrees, global_lane=False, reverse=False):
    global mode
    mode = MissionState.MISSION_TRACKDRIVE
    cmd = [str(WS / 'install/path_generator/lib/path_generator/path_generator_node'),
           '--ros-args', '--params-file', str(WS / 'src/planning/path_generator/config/path_generator.yaml'),
           '-p', 'record.enabled:=false']
    for src, dst in {'/system/mission_state': PREFIX+'/state',
                     '/localization/pose': PREFIX+'/pose',
                     '/system/localization_ready': PREFIX+'/ready',
                     '/planning/final_waypoints': PREFIX+'/path',
                     '/planning/final_waypoints_viz': PREFIX+'/viz',
                     '/planning/driven_trajectory_viz': PREFIX+'/driven_viz'}.items():
        cmd += ['-r', f'{src}:={dst}']
    with tempfile.TemporaryFile(mode='w+') as log:
        proc = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT)
        try:
            deadline = time.monotonic() + 8
            while center_pub.get_subscription_count() == 0:
                assert proc.poll() is None
                assert time.monotonic() < deadline
                spin(0.1)
            spin(0.3)
            yaw = math.radians(degrees)
            x0, y0 = 3.0, -4.0
            state()
            pose(x0, y0, yaw)
            ready(True)
            confidence_pub.publish(Float32(data=1.0))
            path_conf_pub.publish(Float32(data=1.0))
            spin(0.05)
            lane = Lane()
            lane.header.frame_id = 'map'
            from autoware_msgs.msg import Waypoint
            for i in range(48 if global_lane else 12):
                # Closed counterclockwise ring, starting tangentially along +X.
                if global_lane:
                    angle = 2*math.pi*i/48
                    x, y = 12*math.sin(angle), 12*(1-math.cos(angle))
                    heading = angle
                else:
                    x = float(-i-2 if reverse else i+2)
                    y = 0.015*x*x
                    heading = math.atan2(0.03*x, 1)
                wp = Waypoint()
                wp.pose.pose.position.x = x0 + math.cos(yaw)*x - math.sin(yaw)*y
                wp.pose.pose.position.y = y0 + math.sin(yaw)*x + math.cos(yaw)*y
                wp.pose.pose.orientation.z = math.sin((yaw+heading)/2)
                wp.pose.pose.orientation.w = math.cos((yaw+heading)/2)
                wp.twist.twist.linear.x = 5.0
                lane.waypoints.append(wp)
            if global_lane:
                msg = MissionState()
                msg.mission_mode, msg.state = mode, MissionState.RACE
                state_pub.publish(msg)
                spin(0.05)
            received.clear()
            center_pub.publish(lane)
            spin(0.2)
            if reverse:
                assert not received, 'Trackdrive accepted entirely rearward lane'
                print(f'PASS: trackdrive rearward rejection at {degrees} degrees')
                return None
            assert received, 'No trackdrive path'
            result = received[-1]
            # Transform back solely for comparison; production stays in map.
            for w in result.waypoints:
                p = w.pose.pose.position
                dx, dy = p.x-x0, p.y-y0
                p.x, p.y = math.cos(yaw)*dx+math.sin(yaw)*dy, -math.sin(yaw)*dx+math.cos(yaw)*dy
                q = w.pose.pose.orientation
                heading = math.atan2(2*q.w*q.z, 1-2*q.z*q.z) - yaw
                q.z, q.w = math.sin(heading/2), math.cos(heading/2)
            print(f'PASS: trackdrive {"global" if global_lane else "local"} at {degrees} degrees')
            return result
        except BaseException:
            log.seek(0)
            print(log.read())
            raise
        finally:
            proc.terminate()
            proc.wait(timeout=3)
            spin(0.3)


try:
    for mission, u_turn in [('acceleration', False), ('ebs_test', False),
                            ('skidpad', True), ('skidpad', False)]:
        # ROS MissionState calls the mode EBS_TEST but the parameter namespace ebs.
        parameter_mission = 'ebs' if mission == 'ebs_test' else mission
        baseline = run('map', parameter_mission, u_turn=u_turn)
        run('start_pose', parameter_mission, baseline, u_turn=u_turn)
    for global_lane in (False, True):
        baseline = run_trackdrive(0, global_lane)
        for degrees in (-173, 90, 180, -90):
            result = run_trackdrive(degrees, global_lane)
            compare_path(result, baseline, 0, 0, 0)
    for degrees in (0, -173):
        run_trackdrive(degrees, reverse=True)
finally:
    node.destroy_node()
    rclpy.shutdown()
