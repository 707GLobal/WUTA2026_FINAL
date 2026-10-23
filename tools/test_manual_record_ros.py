"""Isolated ROS replay: real mission/trajectory nodes, no hardware or CAN socket."""
import csv
import os
from pathlib import Path
import subprocess
import time
import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import Bool, String
from wuta_msgs.msg import MissionState

ROOT = Path(__file__).resolve().parents[1]
WS = ROOT / 'WUTA-FSD/ros2_ws'


def test_manual_record_ignores_go_and_completion_but_records_driving(tmp_path):
    assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
    assert os.environ.get('ROS_DOMAIN_ID') not in (None, '0')
    rclpy.init()
    node = rclpy.create_node('manual_record_replay')
    processes, logs = [], []
    states = []
    node.create_subscription(MissionState, '/system/mission_state', states.append, 10)
    poses = node.create_publisher(PoseStamped, '/localization/pose', 10)
    speeds = node.create_publisher(TwistStamped, '/chcnav/velocity', 10)
    modes = node.create_publisher(String, '/system/mission_mode_cmd', 10)
    go = node.create_publisher(Bool, '/system/start_command', 10)
    complete = node.create_publisher(Bool, '/system/mission_complete', 10)
    try:
        for package, arguments in [
            ('mission_manager', ['-p', 'manual_record_mode:=true', '-p', 'selfcheck_grace_sec:=60.0']),
            ('path_generator', ['--params-file', str(WS/'src/planning/path_generator/config/path_generator.yaml'),
                                '-p', 'record.run_dir:=' + str(tmp_path)])]:
            log = (tmp_path / (package + '.log')).open('w'); logs.append(log)
            processes.append(subprocess.Popen([str(WS/'install'/package/'lib'/package/(package+'_node')),
                                                '--ros-args', *arguments], stdout=log, stderr=subprocess.STDOUT))
        end = time.monotonic() + 6
        step = 0
        while time.monotonic() < end:
            pose = PoseStamped(); pose.header.stamp = node.get_clock().now().to_msg()
            pose.header.frame_id = 'map'; pose.pose.orientation.w = 1.0
            pose.pose.position.x = step * .1; step += 1
            speed = TwistStamped(); speed.header = pose.header; speed.twist.linear.x = 2.0
            speeds.publish(speed); poses.publish(pose)
            modes.publish(String(data='inspection')); go.publish(Bool(data=True)); complete.publish(Bool(data=True))
            rclpy.spin_once(node, timeout_sec=.05)
            time.sleep(.03)
            assert all(p.poll() is None for p in processes)
        assert states and all(s.state == s.IDLE for s in states)
        assert all('manual_record=1' in s.description and 'wait=manual_record' in s.description for s in states)
        assert node.count_publishers('/control/command') == 0
    finally:
        for process in processes:
            process.terminate()
        for process in processes:
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill(); process.wait()
        for log in logs:
            log.close()
        node.destroy_node(); rclpy.shutdown()
    files = list((tmp_path/'trajectory').rglob('driven.csv'))
    assert len(files) == 1
    with files[0].open() as stream:
        rows = list(csv.DictReader(stream))
    assert len(rows) > 10
    assert float(rows[-1]['x_m']) > float(rows[0]['x_m'])
    assert any(row['speed_mps'] == '2.000000' for row in rows)
    assert all(row[key] == 'nan' for row in rows for key in ('cmd_speed_mps', 'cmd_angle_deg', 'cmd_throttle_brake'))
