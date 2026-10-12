"""Real ROS interface checks using synthetic data in an isolated domain."""
import json
import math
from pathlib import Path
import time
import yaml
import pytest
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String
from wuta_msgs.msg import ConeArray, Cone
from autoware_msgs.msg import Lane, Waypoint
from fsd_visualization.bridge import DisplayBridge
from fsd_visualization.configuration import render

ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT/'src/fsd_visualization/config'


def test_configuration_profiles():
    settings = yaml.safe_load((CONFIG/'topics.yaml').read_text())
    template = yaml.safe_load((CONFIG/'fsd.rviz').read_text())
    native = render(template, settings, 'native')
    portable = render(template, settings, 'bridge')
    native_topics = [d.get('Topic', {}).get('Value', '') for d in native['Visualization Manager']['Displays']]
    bridge_topics = [d.get('Topic', {}).get('Value', '') for d in portable['Visualization Manager']['Displays']]
    assert '/perception/fused/cones_viz' in native_topics
    assert '/fsd_visualization/fused_cones' in bridge_topics
    assert '/sim/ground_truth' not in native_topics
    settings['topics']['camera_image'] = ''
    custom = render(template, settings, 'native')
    assert not next(d for d in custom['Visualization Manager']['Displays'] if d['Name']=='Camera detections')['Enabled']
    settings['stale_seconds'] = float('nan')
    with pytest.raises(ValueError):
        render(template, settings, 'bridge')


def spin_until(nodes, predicate, timeout=4):
    deadline = time.monotonic()+timeout
    while time.monotonic() < deadline:
        for node in nodes:
            rclpy.spin_once(node, timeout_sec=.01)
        if predicate():
            return
    raise AssertionError('ROS interface timed out')


def test_bridge_messages_stale_speed_and_reset():
    rclpy.init(args=['--ros-args', '-p', f'topics_config:={CONFIG / "topics.yaml"}',
                     '-p', 'bridge_mode:=true', '-p', 'use_sim_time:=true'])
    bridge = DisplayBridge()
    probe = Node('fsd_visualization_probe', use_global_arguments=False)
    observed = {}
    subscribers = []
    outputs = {'fused_cones': MarkerArray, 'cone_map': MarkerArray,
               'centerline': MarkerArray, 'planned_path': MarkerArray,
               'trajectory': MarkerArray, 'speed': Marker, 'status': String}
    for name, typ in outputs.items():
        subscribers.append(probe.create_subscription(
            typ, '/fsd_visualization/'+name,
            lambda message, key=name: observed.__setitem__(key, message), bridge.qos))
    publishers = {}
    for name, typ in [('velocity', TwistStamped), ('fused_cones', ConeArray),
                      ('cone_map', ConeArray), ('centerline', Lane),
                      ('final_waypoints', Lane), ('pose', PoseStamped)]:
        publishers[name] = probe.create_publisher(typ, bridge.topics[name], bridge.qos)
    nodes = [bridge, probe]
    try:
        spin_until(nodes, lambda: all(p.get_subscription_count() for p in publishers.values()))
        # Wall timer must run and mark stale even while /clock is absent/frozen.
        spin_until(nodes, lambda: 'status' in observed)
        assert json.loads(observed['status'].data)['speed_stale']
        cones = ConeArray()
        cones.header.frame_id = 'lidar'
        cones.header.stamp.sec = 7
        cone = Cone()
        cone.position.x, cone.position.y = 2., 1.
        cone.color = Cone.COLOR_BLUE
        cones.cones = [cone]
        lane = Lane()
        lane.header.frame_id = 'map'
        for x in (0., 1., 2.):
            point = Waypoint()
            point.pose.pose.position.x = x
            lane.waypoints.append(point)
        velocity = TwistStamped()
        velocity.twist.linear.x, velocity.twist.linear.y = 3., 4.
        for name in ('fused_cones', 'cone_map'):
            publishers[name].publish(cones)
        for name in ('centerline', 'final_waypoints'):
            publishers[name].publish(lane)
        publishers['velocity'].publish(velocity)
        spin_until(nodes, lambda: all(name in observed for name in ('fused_cones','cone_map','centerline','planned_path')) and
                   '5.00 m/s' in observed.get('speed', Marker()).text)
        fused = observed['fused_cones'].markers
        assert fused[0].action == Marker.DELETEALL
        assert fused[1].header.frame_id == 'lidar' and fused[1].header.stamp.sec == 7
        assert fused[1].color.b == 1.
        assert len(observed['planned_path'].markers[1].points) == 3
        lane.waypoints[0].pose.header.frame_id = 'different_frame'
        publishers['final_waypoints'].publish(lane)
        spin_until(nodes, lambda: len(observed['planned_path'].markers)==1)
        # Empty outputs actively delete previous objects.
        cones.cones = []
        publishers['fused_cones'].publish(cones)
        spin_until(nodes, lambda: len(observed['fused_cones'].markers)==1)
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp.sec = 1
        publishers['pose'].publish(pose)
        spin_until(nodes, lambda: bridge.pose_stamp==10**9)
        pose.header.stamp.sec = 2
        pose.pose.position.x = 1.
        publishers['pose'].publish(pose)
        spin_until(nodes, lambda: len(observed.get('trajectory', MarkerArray()).markers)==2)
        assert len(observed['trajectory'].markers[1].points)==2
        # Backward time starts a new trajectory, never connects bag loops.
        pose.header.stamp.sec = 1
        publishers['pose'].publish(pose)
        spin_until(nodes, lambda: len(observed['trajectory'].markers)==1)
        pose.header.stamp.sec = 3
        pose.pose.position.x = 100.
        publishers['pose'].publish(pose)
        spin_until(nodes, lambda: bridge.pose_stamp==3*10**9)
        assert len(bridge.points)==1
        publishers['velocity'].publish(TwistStamped())
        spin_until(nodes, lambda: '0.00 m/s' in observed['speed'].text)
        spin_until(nodes, lambda: 'STALE' in observed['speed'].text and
                   json.loads(observed['status'].data)['speed_stale'], timeout=2.)
        assert json.loads(observed['status'].data)['speed_stale']
        # All visualization-owned publishers stay in the display namespace.
        for name, namespace in probe.get_node_names_and_namespaces():
            if name == 'fsd_display_bridge':
                published = probe.get_publisher_names_and_types_by_node(name, namespace)
                assert all(topic.startswith('/fsd_visualization/') or topic in ('/rosout','/parameter_events')
                           for topic, _ in published)
    finally:
        bridge.destroy_node()
        probe.destroy_node()
        rclpy.try_shutdown()


def test_odometry_speed_and_invalid_values():
    from nav_msgs.msg import Odometry
    rclpy.init(args=['--ros-args', '-p', f'topics_config:={CONFIG / "topics.yaml"}',
                     '-p', 'velocity_type:=odometry'])
    bridge = DisplayBridge()
    probe = Node('fsd_odometry_probe', use_global_arguments=False)
    result = []
    subscription = probe.create_subscription(Marker, '/fsd_visualization/speed', result.append, bridge.qos)
    publisher = probe.create_publisher(Odometry, bridge.topics['odometry'], bridge.qos)
    try:
        spin_until([bridge,probe], lambda: publisher.get_subscription_count()>0)
        message = Odometry()
        message.twist.twist.linear.x, message.twist.twist.linear.y = -3., 4.
        publisher.publish(message)
        spin_until([bridge,probe], lambda: result and '5.00 m/s' in result[-1].text)
        message.twist.twist.linear.x = math.nan
        publisher.publish(message)
        spin_until([bridge,probe], lambda: result and 'STALE' in result[-1].text)
    finally:
        bridge.destroy_node()
        probe.destroy_node()
        rclpy.try_shutdown()
