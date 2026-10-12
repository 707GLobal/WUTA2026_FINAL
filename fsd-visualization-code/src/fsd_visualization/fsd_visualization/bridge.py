"""Consume FSD outputs and publish display messages only."""
from collections import deque
from copy import deepcopy
import math
from pathlib import Path
import time
import signal
import yaml
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import TwistStamped, PoseStamped
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String
from .configuration import validate


def finite_point(point):
    return all(math.isfinite(value) for value in (point.x, point.y, point.z))


def main():
    rclpy.init()
    node = None
    try:
        node = DisplayBridge()
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        # Terminal SIGINT and launch forwarding can arrive twice during teardown.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


class DisplayBridge(Node):
    def __init__(self):
        super().__init__('fsd_display_bridge')
        self.declare_parameter('topics_config', '')
        self.declare_parameter('bridge_mode', False)
        self.declare_parameter('velocity_type', 'twist')
        self.settings = yaml.safe_load(Path(self.get_parameter('topics_config').value).read_text())
        validate(self.settings)
        self.topics = self.settings['topics']
        self.qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.subscriptions_owned = []
        self.last_speed = None
        self.last_received = None
        self.received = {}
        self.frame = self.settings['fixed_frame']
        self.points = deque(maxlen=int(self.settings['trajectory_max_points']))
        self.pose_frame = None
        self.pose_stamp = None
        self.last_pose_received = None
        self.last_ros_time = self.get_clock().now().nanoseconds
        self.speed_pub = self.create_publisher(Marker, '/fsd_visualization/speed', self.qos)
        self.status_pub = self.create_publisher(String, '/fsd_visualization/status', self.qos)
        velocity_type = self.get_parameter('velocity_type').value
        if velocity_type not in ('twist', 'odometry'):
            raise ValueError('velocity_type must be twist or odometry')
        self.velocity_topic = self.topics['velocity' if velocity_type == 'twist' else 'odometry']
        if self.velocity_topic:
            self.subscribe(TwistStamped if velocity_type == 'twist' else Odometry,
                           self.velocity_topic, self.speed)
        self.bridge_mode = self.get_parameter('bridge_mode').value
        if self.bridge_mode:
            # Native mode needs no custom message imports at runtime.
            from wuta_msgs.msg import ConeArray
            from autoware_msgs.msg import Lane
            for name, topic_key in [('fused_cones', 'fused_cones'), ('cone_map', 'cone_map')]:
                publisher = self.create_publisher(MarkerArray, f'/fsd_visualization/{name}', self.qos)
                if self.topics[topic_key]:
                    self.subscribe(ConeArray, self.topics[topic_key],
                                   lambda msg, p=publisher, n=name: self.cones(msg, p, n))
            for name, key, color in [('centerline', 'centerline', (0., 1., 0.)),
                                     ('planned_path', 'final_waypoints', (0., .8, 1.))]:
                publisher = self.create_publisher(MarkerArray, f'/fsd_visualization/{name}', self.qos)
                if self.topics[key]:
                    self.subscribe(Lane, self.topics[key],
                                   lambda msg, p=publisher, n=name, c=color: self.lane(msg, p, n, c))
            self.trajectory_pub = self.create_publisher(MarkerArray, '/fsd_visualization/trajectory', self.qos)
            if self.topics['pose']:
                self.subscribe(PoseStamped, self.topics['pose'], self.pose)
        self.timer = self.create_timer(.1, self.render_speed,
                                      clock=Clock(clock_type=ClockType.STEADY_TIME))

    def subscribe(self, message_type, topic, callback):
        self.received[topic] = None
        def receive(message):
            self.received[topic] = time.monotonic()
            callback(message)
        self.subscriptions_owned.append(self.create_subscription(message_type, topic, receive, self.qos))

    def marker(self, header, namespace, marker_type, marker_id=0):
        marker = Marker()
        marker.header = deepcopy(header)
        marker.ns, marker.id = namespace, marker_id
        marker.type, marker.action = marker_type, Marker.ADD
        marker.pose.orientation.w = 1.
        marker.color.a = 1.
        # Live observations expire in ROS time; map/path/trajectory persist.
        return marker

    def cleared(self, header):
        clear = Marker()
        clear.header = deepcopy(header)
        clear.action = Marker.DELETEALL
        return MarkerArray(markers=[clear])

    def cones(self, message, publisher, namespace):
        if not message.header.frame_id:
            self.get_logger().warning(f'{namespace}: empty frame_id; message dropped')
            return
        array = self.cleared(message.header)
        colors = {0: (1., 0., 0.), 1: (1., .9, 0.), 2: (0., .4, 1.), 3: (1., .5, 0.)}
        for index, cone in enumerate(message.cones):
            if not finite_point(cone.position):
                continue
            marker = self.marker(message.header, namespace, Marker.CYLINDER, index)
            marker.pose.position = deepcopy(cone.position)
            marker.scale.x = marker.scale.y = .25
            marker.scale.z = .4
            marker.color.r, marker.color.g, marker.color.b = colors.get(cone.color, (1., 1., 1.))
            if namespace == 'fused_cones':
                marker.lifetime.sec = 1
            array.markers.append(marker)
        publisher.publish(array)

    def lane(self, message, publisher, namespace, color):
        if not message.header.frame_id:
            self.get_logger().warning(f'{namespace}: empty frame_id; message dropped')
            return
        array = self.cleared(message.header)
        marker = self.marker(message.header, namespace, Marker.LINE_STRIP)
        marker.scale.x = .09
        marker.color.r, marker.color.g, marker.color.b = color
        for waypoint in message.waypoints:
            frame = waypoint.pose.header.frame_id
            if frame and frame != message.header.frame_id:
                self.get_logger().warning(f'{namespace}: waypoint frame mismatch; display cleared')
                publisher.publish(array)
                return
            if finite_point(waypoint.pose.pose.position):
                marker.points.append(deepcopy(waypoint.pose.pose.position))
        if len(marker.points) >= 2:
            array.markers.append(marker)
        publisher.publish(array)

    def pose(self, message):
        point = message.pose.position
        if not message.header.frame_id or not finite_point(point):
            return
        stamp = message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
        reset = self.pose_frame != message.header.frame_id or (self.pose_stamp is not None and stamp < self.pose_stamp)
        if self.points and math.dist((point.x, point.y, point.z),
                                    (self.points[-1].x, self.points[-1].y, self.points[-1].z)) > self.settings['trajectory_max_step']:
            reset = True
        if reset:
            self.points.clear()
        self.pose_frame, self.pose_stamp = message.header.frame_id, stamp
        self.last_pose_received = time.monotonic()
        if not self.points or math.dist((point.x, point.y, point.z),
                                       (self.points[-1].x, self.points[-1].y, self.points[-1].z)) >= self.settings['trajectory_min_distance']:
            self.points.append(deepcopy(point))
        array = self.cleared(message.header)
        if len(self.points) >= 2:
            marker = self.marker(message.header, 'localization_trajectory', Marker.LINE_STRIP)
            marker.points = list(self.points)
            marker.scale.x = .06
            marker.color.r, marker.color.g, marker.color.b = 1., .85, 0.
            array.markers.append(marker)
        self.trajectory_pub.publish(array)

    def speed(self, message):
        velocity = message.twist.linear if isinstance(message, TwistStamped) else message.twist.twist.linear
        speed = math.hypot(velocity.x, velocity.y)
        if math.isfinite(speed):
            self.last_speed, self.last_received = speed, time.monotonic()
        else:
            self.last_speed, self.last_received = None, None

    def render_speed(self):
        now = time.monotonic()
        ros_time = self.get_clock().now()
        if ros_time.nanoseconds < self.last_ros_time:
            self.points.clear()
            self.pose_stamp = None
            self.last_speed, self.last_received = None, None
            if self.bridge_mode and self.pose_frame:
                from std_msgs.msg import Header
                self.trajectory_pub.publish(self.cleared(Header(frame_id=self.pose_frame)))
        self.last_ros_time = ros_time.nanoseconds
        marker = Marker()
        # Follow the same real TF frame as the RViz camera; never publish a fake TF.
        marker.header.frame_id = self.settings['target_frame']
        marker.header.stamp = ros_time.to_msg()
        marker.ns, marker.id = 'fsd_velocity', 0
        marker.type, marker.action = Marker.TEXT_VIEW_FACING, Marker.ADD
        marker.pose.orientation.w = 1.
        marker.pose.position.z = 2.5
        marker.frame_locked = True
        marker.scale.z = .5
        marker.color.a = 1.
        stale = self.last_received is None or now - self.last_received > self.settings['stale_seconds']
        if stale:
            marker.text = 'FSD speed: -- (STALE)'
            marker.color.r, marker.color.g = 1., .4
        else:
            marker.text = f'FSD speed: {self.last_speed:.2f} m/s | {self.last_speed * 3.6:.1f} km/h'
            marker.color.g, marker.color.b = 1., .8
        self.speed_pub.publish(marker)
        import json
        self.status_pub.publish(String(data=json.dumps({
            'speed_source': self.velocity_topic, 'speed_stale': stale,
            'speed_m_s': None if stale else self.last_speed,
            'input_wall_age_s': {topic: None if received is None else now - received
                                 for topic, received in self.received.items()},
            'trajectory_points': len(self.points), 'bridge_mode': bool(self.bridge_mode),
        })))
