"""Isolated ROS nodes only. Run with ROS_LOCALHOST_ONLY=1 and a dedicated domain.
No CAN node, hardware bridge, sensor driver or full launcher is started here.
"""
import os
from pathlib import Path
import subprocess
import time

import pytest
import yaml
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Bool, String
from wuta_msgs.msg import MissionState, DevicesInspection
from autoware_msgs.msg import Command

ROOT = Path(__file__).resolve().parents[1]
WS = ROOT / 'WUTA-FSD/ros2_ws'


@pytest.fixture
def rig(tmp_path):
    assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
    assert os.environ.get('ROS_DOMAIN_ID') not in (None, '0'), 'Use an isolated ROS domain'
    rclpy.init()
    node = rclpy.create_node('brake_bench_test')
    processes, logs = [], []

    def launch(package, bench=True, short=True):
        args = [str(WS / 'install' / package / 'lib' / package / (package + '_node')),
                '--ros-args', '-p', 'brake_bench_mode:=' + str(bench).lower()]
        if package == 'controller':
            config = WS / 'src/control/controller/config/controller.yaml'
            if short:
                document = yaml.safe_load(config.read_text())
                document['controller_node']['ros__parameters']['acceleration']['bench_brake'].update(
                    delay_sec=0.2, hold_sec=0.4)
                config = tmp_path / 'controller.yaml'
                config.write_text(yaml.safe_dump(document))
            args += ['--params-file', str(config)]
        log = (tmp_path / f'{package}_{len(processes)}.log').open('w')
        logs.append(log)
        proc = subprocess.Popen(args, stdout=log, stderr=subprocess.STDOUT)
        processes.append(proc)
        return proc

    def spin(seconds):
        until = time.monotonic() + seconds
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=min(0.02, until - time.monotonic()))
        for proc in processes:
            assert proc.poll() is None, f'Node exited: {proc.args}'

    def until(predicate, timeout=4):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            spin(.02)
        assert predicate(), 'ROS condition timed out'

    yield node, launch, spin, until
    for proc in processes:
        if proc.poll() is None:
            proc.terminate()
    for proc in processes:
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait()
    for log in logs:
        log.close()
    node.destroy_node()
    rclpy.shutdown()


def test_actual_5s_10s_sequence_without_sensors_and_one_shot(rig):
    node, launch, spin, until = rig
    commands, states, inspections = [], [], []
    node.create_subscription(Command, '/control/command', lambda m: commands.append((time.monotonic(), m)), 10)
    node.create_subscription(MissionState, '/system/mission_state', states.append, 10)
    node.create_subscription(DevicesInspection, '/system/devices_inspection', inspections.append, 10)
    modes = node.create_publisher(String, '/system/mission_mode_cmd', 10)
    go = node.create_publisher(Bool, '/system/start_command', 10)
    launch('controller', short=False)
    launch('mission_manager')
    until(lambda: states and commands and inspections)
    assert inspections[-1].ok
    modes.publish(String(data='skidpad')); spin(.2)
    go.publish(Bool(data=True)); spin(.2)
    assert states[-1].state == MissionState.IDLE
    modes.publish(String(data='acceleration'))
    until(lambda: states[-1].state == MissionState.READY)
    assert all(m.throttle_brake == 0 for _, m in commands)
    go.publish(Bool(data=True))
    until(lambda: states[-1].state == MissionState.EXPLORE)
    started = time.monotonic()
    until(lambda: any(m.throttle_brake == -.5 for _, m in commands), timeout=6)
    braking_at = next(t for t, m in commands if m.throttle_brake == -.5)
    assert 4.7 < braking_at - started < 5.3
    until(lambda: states[-1].state == MissionState.FINISH, timeout=11)
    released_at = next(t for t, m in commands if t > braking_at and m.throttle_brake == 0)
    assert 9.7 < released_at - braking_at < 10.3
    go.publish(Bool(data=True)); spin(.4)
    assert states[-1].state == MissionState.FINISH
    assert all(m.speed == 0 and m.angle == 0 and m.throttle_brake in (0, -.5) for _, m in commands)
    assert all(m.throttle_brake == 0 for t, m in commands if t >= released_at)
    assert states[-1].brake_bench_mode


@pytest.mark.parametrize('scenario', ['emergency', 'short_sequence', 'mismatch', 'late_start'])
def test_controller_interrupts_never_fall_into_driving(rig, scenario):
    node, launch, spin, until = rig
    commands = []
    node.create_subscription(Command, '/control/command', commands.append, 10)
    states = node.create_publisher(MissionState, '/system/mission_state', 10)
    emergency = node.create_publisher(Bool, '/system/emergency',
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    if scenario == 'emergency':
        # Also verify a controller starting after a latched RES emergency stays neutral.
        emergency.publish(Bool(data=True))
    launch('controller')
    until(lambda: states.get_subscription_count() > 0 and commands)
    state = MissionState(brake_bench_mode=True, mission_mode=MissionState.MISSION_ACCELERATION)
    if scenario != 'late_start':
        state.state = MissionState.READY
        states.publish(state); spin(.1)
    state.state = MissionState.EXPLORE
    if scenario == 'mismatch':
        state.brake_bench_mode = False
    for _ in range(20):
        states.publish(state); spin(.04)
    if scenario == 'short_sequence':
        assert any(m.throttle_brake == -.5 for m in commands)
        assert commands[-1].throttle_brake == 0
    else:
        assert all(m.throttle_brake == 0 for m in commands)


def test_mission_timeout_during_braking_aborts_and_does_not_resume(rig):
    node, launch, spin, until = rig
    commands = []
    node.create_subscription(Command, '/control/command', commands.append, 10)
    states = node.create_publisher(MissionState, '/system/mission_state', 10)
    launch('controller', short=False)
    until(lambda: commands and states.get_subscription_count() > 0)
    state = MissionState(brake_bench_mode=True, mission_mode=MissionState.MISSION_ACCELERATION,
                         state=MissionState.READY)
    states.publish(state); spin(.1)
    state.state = MissionState.EXPLORE
    for _ in range(55):
        states.publish(state); spin(.1)
    assert commands[-1].throttle_brake == -.5
    spin(.7)
    assert commands[-1].throttle_brake == 0
    for _ in range(8):
        states.publish(state); spin(.1)
    assert commands[-1].throttle_brake == 0


def test_normal_mode_still_requires_gnss_and_inputs(rig):
    node, launch, spin, until = rig
    states = []
    node.create_subscription(MissionState, '/system/mission_state', states.append, 10)
    modes = node.create_publisher(String, '/system/mission_mode_cmd', 10)
    go = node.create_publisher(Bool, '/system/start_command', 10)
    launch('mission_manager', bench=False)
    until(lambda: states)
    modes.publish(String(data='acceleration')); spin(.1)
    go.publish(Bool(data=True)); spin(.3)
    assert states[-1].state == MissionState.IDLE
    assert not states[-1].brake_bench_mode
    assert 'gnss' in states[-1].description and 'localization' in states[-1].description
