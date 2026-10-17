"""Linux ROS 2 consumer. ROS imports are isolated from the portable/PC code."""

import json
import logging
import math
import queue
import time
from .geometry import (
    MarkerCache,
    scene_snapshot,
    sample_indices,
    quaternion,
    transform_point,
    multiply,
)
from .protocol import MASTER
from .worker import RadioWorker

LOG = logging.getLogger(__name__)


def run_linux(config, port, use_sim_time=False):
    import rclpy
    from rclpy.node import Node
    from rclpy.time import Time
    from rclpy.clock import Clock, ClockType
    from rclpy.duration import Duration
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from tf2_ros import Buffer, TransformListener, TransformException
    from visualization_msgs.msg import MarkerArray
    from geometry_msgs.msg import PoseStamped, TwistStamped
    from std_msgs.msg import String

    incoming = queue.Queue(maxsize=64)

    def receive(message):
        incoming.put_nowait(message)

    worker = RadioWorker(MASTER, port, config, receive)

    class Sender(Node):
        def __init__(self):
            super().__init__(
                "fsd_radio_sender",
                parameter_overrides=[
                    rclpy.parameter.Parameter("use_sim_time", value=use_sim_time)
                ],
            )
            self.qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
            self.tf = Buffer(cache_time=Duration(seconds=30))
            self.tf_listener = TransformListener(self.tf, self)
            self.caches = {
                name: MarkerCache(config["geometry"]["cache_markers"])
                for name in config["resolved_layers"]
            }
            self.seen = set()
            self.subs = []
            self.pose_message = None
            self.speed = None
            self.speed_received = 0
            self.pose_received = 0
            self.input_received = {}
            self.last_ros_now = None
            self.snapshots_last = {}
            for name, topic in config["resolved_layers"].items():
                if topic:
                    self.subs.append(
                        self.create_subscription(
                            MarkerArray,
                            topic,
                            lambda msg, key=name: self.markers(key, msg),
                            self.qos,
                        )
                    )
            if config["pose_topic"]:
                self.subs.append(
                    self.create_subscription(
                        PoseStamped, config["pose_topic"], self.pose_input, self.qos
                    )
                )
            if config["velocity_topic"]:
                self.subs.append(
                    self.create_subscription(
                        TwistStamped, config["velocity_topic"], self.velocity, self.qos
                    )
                )
            self.extension_pub = self.create_publisher(
                String, "/fsd_radio/extension/rx", 10
            )
            self.extension_sub = self.create_subscription(
                String, "/fsd_radio/extension/tx", self.extension, 10
            )
            self.status_pub = self.create_publisher(String, "/fsd_radio/status", 10)
            self.timer = self.create_timer(
                0.5, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME)
            )

        def lookup(self, frame, stamp):
            if frame == config["fixed_frame"]:
                return (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)
            try:
                transform = self.tf.lookup_transform(
                    config["fixed_frame"], frame, Time(nanoseconds=stamp)
                )
            except TransformException as exc:
                raise LookupError(str(exc)) from exc
            t, r = transform.transform.translation, transform.transform.rotation
            return (t.x, t.y, t.z), quaternion((r.x, r.y, r.z, r.w))

        def markers(self, name, message):
            converted = []

            def rgba(c):
                values = (c.r, c.g, c.b, c.a)
                if not all(math.isfinite(v) for v in values):
                    raise ValueError("non-finite color")
                return [max(0, min(255, round(v * 255))) for v in values]

            for marker in message.markers:
                # Cap retained point arrays before storing them, independent of ROS input size.
                indices = sample_indices(len(marker.points), 256, marker.type == 5)
                try:
                    converted.append(
                        {
                            "ns": marker.ns[:64],
                            "id": marker.id,
                            "type": marker.type,
                            "action": marker.action,
                            "stamp": marker.header.stamp.sec * 10**9
                            + marker.header.stamp.nanosec,
                            "frame": marker.header.frame_id,
                            "frame_locked": marker.frame_locked,
                            "lifetime_ns": marker.lifetime.sec * 10**9
                            + marker.lifetime.nanosec,
                            "position": (
                                marker.pose.position.x,
                                marker.pose.position.y,
                                marker.pose.position.z,
                            ),
                            "orientation": (
                                marker.pose.orientation.x,
                                marker.pose.orientation.y,
                                marker.pose.orientation.z,
                                marker.pose.orientation.w,
                            ),
                            "scale": (marker.scale.x, marker.scale.y, marker.scale.z),
                            "color": rgba(marker.color),
                            "points": [
                                (
                                    marker.points[i].x,
                                    marker.points[i].y,
                                    marker.points[i].z,
                                )
                                for i in indices
                            ],
                            "source_points": len(marker.points),
                            "colors": [rgba(marker.colors[i]) for i in indices]
                            if len(marker.colors) == len(marker.points)
                            else [],
                            "text": marker.text[: config["geometry"]["text_chars"]],
                        }
                    )
                except ValueError as exc:
                    LOG.warning("invalid %s marker ignored: %s", name, exc)
            self.caches[name].apply(converted, self.get_clock().now().nanoseconds)
            self.seen.add(name)
            self.input_received[name] = time.monotonic()

        def pose_input(self, message):
            self.pose_message = message
            self.pose_received = time.monotonic()

        def velocity(self, message):
            v = message.twist.linear
            speed = math.hypot(v.x, v.y)
            self.speed = speed if math.isfinite(speed) else None
            self.speed_received = time.monotonic()

        def extension(self, message):
            try:
                if len(message.data.encode()) > 512:
                    raise ValueError("extension JSON exceeds 512 bytes")
                worker.outbox.extension(json.loads(message.data))
            except (ValueError, TypeError) as exc:
                LOG.warning("ROS extension rejected: %s", exc)

        def tick(self):
            now_ns = self.get_clock().now().nanoseconds
            if self.last_ros_now is not None and now_ns < self.last_ros_now:
                for cache in self.caches.values():
                    cache.items.clear()
                    cache.source_stamp = 0
                self.pose_message = None
            self.last_ros_now = now_ns
            wall = time.monotonic()
            for name in self.seen:
                # Snapshot preparation is bounded and stays on the ROS thread.
                if wall - self.snapshots_last.get(name, -1e9) < min(
                    config["layers"][name]["interval"], 2
                ):
                    continue
                try:
                    snapshot = scene_snapshot(
                        name,
                        self.caches[name],
                        self.lookup,
                        config,
                        now_ns,
                        wall - self.input_received[name],
                    )
                    worker.outbox.update(name, snapshot)
                    self.snapshots_last[name] = wall
                except ValueError as exc:
                    LOG.warning("snapshot %s rejected: %s", name, exc)
            if self.pose_message:
                message = self.pose_message
                p, r = message.pose.position, message.pose.orientation
                try:
                    stamp = (
                        message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
                    )
                    if not message.header.frame_id:
                        raise ValueError("pose has no frame")
                    point = (p.x, p.y, p.z)
                    if not all(math.isfinite(v) for v in point):
                        raise ValueError("pose has non-finite coordinates")
                    transform = self.lookup(message.header.frame_id, stamp)
                    step = config["geometry"]["quantization_m"]
                    worker.outbox.update(
                        "vehicle",
                        {
                            "k": "vehicle",
                            "f": config["fixed_frame"],
                            "t": stamp,
                            "q": step,
                            "p": [
                                round(v / step)
                                for v in transform_point(transform, point)
                            ],
                            "r": [
                                round(v, 4)
                                for v in multiply(
                                    transform[1], quaternion((r.x, r.y, r.z, r.w))
                                )
                            ],
                            "a": round(wall - self.pose_received, 1),
                            "v": round(self.speed, 2)
                            if self.speed is not None and wall - self.speed_received < 2
                            else None,
                        },
                    )
                except (ValueError, LookupError) as exc:
                    LOG.debug("pose not available: %s", exc)
            while not incoming.empty():
                message = incoming.get_nowait()
                if message["k"] != "extension":
                    LOG.warning("ignored non-extension uplink")
                    continue
                body = message["body"]
                self.extension_pub.publish(
                    String(data=json.dumps(body, ensure_ascii=False))
                )
                if body.get("type") == "ping":
                    try:
                        worker.outbox.extension(
                            {
                                "type": "pong",
                                "id": body.get("id"),
                                "linux_time_ns": time.time_ns(),
                            }
                        )
                    except ValueError as exc:
                        LOG.warning("pong queue full: %s", exc)
            self.status_pub.publish(
                String(
                    data=json.dumps(
                        {
                            **worker.status(),
                            "unseen_layers": [
                                name
                                for name, topic in config["resolved_layers"].items()
                                if topic and name not in self.seen
                            ],
                        }
                    )
                )
            )

    rclpy.init(args=[])
    node = None
    try:
        node = Sender()
        worker.start()
        LOG.info("ROS inputs: %s", config["resolved_layers"])
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if worker.thread.is_alive():
            worker.close()
        if node:
            node.destroy_node()
        rclpy.try_shutdown()
