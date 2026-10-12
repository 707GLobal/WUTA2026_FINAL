"""Isolated graphical check; synthetic sources never include control commands."""
import argparse
import os
from pathlib import Path
import signal
import subprocess
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, ReliabilityPolicy
from sensor_msgs.msg import Image, PointCloud2, PointField
from std_msgs.msg import Header
from sensor_msgs_py import point_cloud2
from geometry_msgs.msg import TwistStamped, TransformStamped, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import StaticTransformBroadcaster
from wuta_msgs.msg import ConeArray, Cone
from autoware_msgs.msg import Lane, Waypoint


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--mode', choices=['native','bridge'], default='native')
    parser.add_argument('--dashboard', action='store_true', help='Check the full-stack same-screen layout')
    parser.add_argument('--screenshot', help='Save a desktop capture while the viewer is open')
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    log = root/'log'/f'gui-{args.mode}.log'
    log.parent.mkdir(exist_ok=True)
    rclpy.init()
    node = Node('isolated_fsd_gui_probe')
    broadcaster = StaticTransformBroadcaster(node)
    transforms = []
    for frame in ('base_link','lidar', 'rslidar'):
        transform = TransformStamped()
        transform.header.frame_id = 'map'
        transform.child_frame_id = frame
        transform.transform.rotation.w = 1.
        transforms.append(transform)
    broadcaster.sendTransform(transforms)
    image = node.create_publisher(Image, '/camera/yolo/image_annotated', qos_profile_sensor_data)
    velocity = node.create_publisher(TwistStamped, '/localization/velocity', qos_profile_sensor_data)
    cloud = node.create_publisher(PointCloud2, '/rslidar_points', qos_profile_sensor_data) if args.dashboard else None
    native_topics = ['/perception/fused/guided_clusters_viz', '/control/target_viz']
    if args.dashboard:
        native_topics = ['/perception/lidar/cones_viz', '/control/target_viz']
    if args.mode == 'native':
        native_topics += ['/perception/fused/cones_viz','/mapping/cone_map_viz',
                          '/planning/centerline_viz','/planning/final_waypoints_viz',
                          '/planning/driven_trajectory_viz']
    publishers = [node.create_publisher(MarkerArray, topic, qos_profile_sensor_data) for topic in native_topics]
    extra = {}
    if args.mode == 'bridge':
        for topic, typ in [('/perception/fused/cones',ConeArray),('/mapping/cone_map',ConeArray),
                           ('/planning/centerline',Lane),('/planning/final_waypoints',Lane),
                           ('/localization/pose',PoseStamped)]:
            extra[topic] = node.create_publisher(typ, topic, qos_profile_sensor_data)
    with log.open('w') as stream:
        command = ['bash', str(root/'start.sh'), f'mode:={args.mode}']
        if args.dashboard:
            command += [f'rviz_config:={root.parent}/config/full_fsd.rviz',
                        f'topics_config:={root.parent}/config/visualization_topics.yaml']
        process = subprocess.Popen(command,
                                   stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
        try:
            deadline = time.monotonic()+10
            while time.monotonic()<deadline:
                assert process.poll() is None, log.read_text()
                stamp = node.get_clock().now().to_msg()
                im = Image()
                im.header.frame_id, im.header.stamp = 'base_link', stamp
                im.height, im.width, im.encoding, im.step = 120, 160, 'rgb8', 480
                im.data = bytes([40,100,180])*19200
                image.publish(im)
                if cloud:
                    header = Header(frame_id='rslidar', stamp=stamp)
                    fields = [PointField(name=name, offset=i*4, datatype=PointField.FLOAT32, count=1)
                              for i, name in enumerate(('x', 'y', 'z'))]
                    cloud.publish(point_cloud2.create_cloud(header, fields,
                        [(x*.1, y*.2, (x%5)*.08) for x in range(100) for y in range(-25, 26)]))
                speed = TwistStamped()
                speed.twist.linear.x = 2.
                velocity.publish(speed)
                marker = Marker()
                marker.header.frame_id, marker.header.stamp = 'map', stamp
                marker.type = Marker.SPHERE
                marker.pose.orientation.w = 1.
                marker.scale.x = marker.scale.y = marker.scale.z = .3
                marker.color.g = marker.color.a = 1.
                for pub in publishers:
                    pub.publish(MarkerArray(markers=[marker]))
                if extra:
                    cones = ConeArray()
                    cones.header = marker.header
                    cone = Cone()
                    cone.position.x, cone.color = 2., Cone.COLOR_BLUE
                    cones.cones = [cone]
                    lane = Lane()
                    lane.header = marker.header
                    for x in (0.,1.,2.):
                        waypoint = Waypoint()
                        waypoint.pose.pose.position.x = x
                        lane.waypoints.append(waypoint)
                    pose = PoseStamped()
                    pose.header = marker.header
                    pose.pose.position.x = 2.*(10.-(deadline-time.monotonic()))
                    for topic,pub in extra.items():
                        pub.publish(cones if 'cones' in topic or 'cone_map' in topic
                                    else pose if topic.endswith('/pose') else lane)
                rclpy.spin_once(node,timeout_sec=.05)
                time.sleep(.05)
            expected = ['/camera/yolo/image_annotated',*native_topics,'/fsd_visualization/speed']
            if cloud:
                expected += ['/rslidar_points']
            if extra:
                expected += ['/fsd_visualization/fused_cones','/fsd_visualization/cone_map',
                             '/fsd_visualization/centerline','/fsd_visualization/planned_path',
                             '/fsd_visualization/trajectory']
            for topic in expected:
                endpoints = [entry for entry in node.get_subscriptions_info_by_topic(topic)
                             if entry.node_name == 'fsd_rviz']
                assert endpoints, f'No RViz subscription: {topic}'
                assert all(entry.qos_profile.reliability == ReliabilityPolicy.BEST_EFFORT for entry in endpoints), topic
            if args.screenshot:
                from PyQt5.QtWidgets import QApplication
                app = QApplication([])
                assert app.primaryScreen().grabWindow(0).save(args.screenshot), 'Screenshot failed'
        finally:
            if process.poll() is None:
                os.kill(process.pid,signal.SIGINT)
            try:
                process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid,signal.SIGKILL)
                process.wait()
                raise
            node.destroy_node()
            rclpy.try_shutdown()
    output = log.read_text()
    assert 'process has died' not in output, output
    assert 'Traceback' not in output, output
    assert 'Message Filter dropping' not in output, output
    print(f'PASS: {args.mode} RViz; synthetic image/markers; final Best Effort endpoints; '
          f'10-second survival; clean shutdown; domain {os.environ.get("ROS_DOMAIN_ID")}.')
    print(log)


if __name__ == '__main__':
    main()
