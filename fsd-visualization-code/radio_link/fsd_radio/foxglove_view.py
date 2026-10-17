"""Official SDK typed SceneUpdate channels; the PC does not require ROS."""

import json
import math
import queue
import threading
import time
import foxglove
from foxglove import messages as m
from foxglove.channels import SceneUpdateChannel
from foxglove.websocket import Capability, ServerListener
from .geometry import multiply, rotate

TX_TOPIC = "/fsd_radio/extension/tx"


def timestamp(ns):
    return m.Timestamp(sec=ns // 10**9, nsec=ns % 10**9)


def vector(values):
    return m.Vector3(x=values[0], y=values[1], z=values[2])


def pose(position=(0, 0, 0), rotation=(0, 0, 0, 1)):
    return m.Pose(
        position=vector(position),
        orientation=m.Quaternion(
            x=rotation[0], y=rotation[1], z=rotation[2], w=rotation[3]
        ),
    )


def color(values):
    return m.Color(
        r=values[0] / 255, g=values[1] / 255, b=values[2] / 255, a=values[3] / 255
    )


def arrow_rotation(direction):
    length = math.sqrt(sum(v * v for v in direction))
    if length < 1e-8:
        return (0, 0, 0, 1), 0.0
    x, y, z = (v / length for v in direction)
    if x < -0.999999:
        return (0, 0, 1, 0), length
    norm = math.sqrt(2 * (1 + x))
    return (0, -z / norm, y / norm, norm / 2), length


def entity_from_record(record, frame, step, now_ns, lifetime, source_stamp):
    ident, typ, p, q, scale, rgba, points, point_colors, text = record
    position = [v * step for v in p]
    points = [[v * step for v in point] for point in points]
    origin = pose(position, q)
    c = color(rgba)
    fields = dict(
        timestamp=timestamp(now_ns),
        frame_id=frame,
        id=ident,
        lifetime=m.Duration(sec=int(lifetime), nsec=int(lifetime % 1 * 1e9)),
        metadata=[
            m.KeyValuePair(key="source_ros_stamp_ns", value=str(source_stamp)),
            m.KeyValuePair(key="display", value="quantized low-bandwidth sample"),
        ],
    )
    if typ in (4, 5):
        fields["lines"] = [
            m.LinePrimitive(
                type=m.LinePrimitiveLineType.LineStrip
                if typ == 4
                else m.LinePrimitiveLineType.LineList,
                pose=origin,
                thickness=max(scale[0], 0.01),
                points=[m.Point3(x=p[0], y=p[1], z=p[2]) for p in points],
                color=c,
                colors=[color(v) for v in point_colors],
            )
        ]
    elif typ in (6, 7, 8):
        primitives = []
        for index, point in enumerate(points):
            world = [a + b for a, b in zip(position, rotate(q, point))]
            cls = m.CubePrimitive if typ == 6 else m.SpherePrimitive
            primitives.append(
                cls(
                    pose=pose(world, q),
                    size=vector([max(v, 0.03) for v in scale]),
                    color=color(point_colors[index]) if point_colors else c,
                )
            )
        if typ == 6:
            fields["cubes"] = primitives
        else:
            fields["spheres"] = primitives
    elif typ == 1:
        fields["cubes"] = [m.CubePrimitive(pose=origin, size=vector(scale), color=c)]
    elif typ == 2:
        fields["spheres"] = [
            m.SpherePrimitive(pose=origin, size=vector(scale), color=c)
        ]
    elif typ == 3:
        fields["cylinders"] = [
            m.CylinderPrimitive(
                pose=origin, size=vector(scale), bottom_scale=1, top_scale=1, color=c
            )
        ]
    elif typ == 9:
        fields["texts"] = [
            m.TextPrimitive(
                pose=origin,
                billboard=True,
                font_size=max(scale[2], 0.1),
                color=c,
                text=text,
            )
        ]
    elif typ == 0:
        if len(points) >= 2:
            rotation, length = arrow_rotation(
                [b - a for a, b in zip(points[0], points[1])]
            )
            start = [a + b for a, b in zip(position, rotate(q, points[0]))]
            origin = pose(start, multiply(q, rotation))
            head_length = min(scale[2] or length * 0.23, length)
            shaft_diameter, head_diameter = scale[:2]
        else:
            length, shaft_diameter, head_diameter = scale
            head_length = length * 0.23
        fields["arrows"] = [
            m.ArrowPrimitive(
                pose=origin,
                shaft_length=max(0, length - head_length),
                shaft_diameter=max(0.02, shaft_diameter),
                head_length=head_length,
                head_diameter=max(0.04, head_diameter),
                color=c,
            )
        ]
    return m.SceneEntity(**fields)


class PublishListener(ServerListener):
    def __init__(self, enqueue):
        self.enqueue = enqueue
        self.channels = {}
        self.errors = queue.Queue(maxsize=32)

    def on_client_advertise(self, client, channel):
        if channel.topic == TX_TOPIC and channel.encoding == "json":
            self.channels[(client.id, channel.id)] = True

    def on_client_unadvertise(self, client, client_channel_id):
        self.channels.pop((client.id, client_channel_id), None)

    def on_message_data(self, client, client_channel_id, data):
        try:
            if (client.id, client_channel_id) not in self.channels:
                raise ValueError(f"only {TX_TOPIC} with json encoding is accepted")
            if len(data) > 512:
                raise ValueError("extension exceeds 512 bytes")
            self.enqueue(json.loads(data))
        except (ValueError, TypeError) as exc:
            try:
                self.errors.put_nowait(str(exc))
            except queue.Full:
                pass


class FoxgloveView:
    def __init__(self, config, enqueue):
        self.config = config
        self.context = foxglove.Context()
        self.listener = PublishListener(enqueue)
        self.channels = {
            name: SceneUpdateChannel("/radio/scene/" + name, context=self.context)
            for name in config["layers"]
        }
        self.json_channels = {
            name: foxglove.Channel("/radio/" + name, context=self.context)
            for name in ("state", "link", "layers", "extension/rx", "extension/status")
        }
        self.snapshots = {}
        self.lock = threading.RLock()
        self.server = foxglove.start_server(
            name="FSD E22 2.4kbps",
            host=config["foxglove"]["host"],
            port=config["foxglove"]["port"],
            context=self.context,
            capabilities=[Capability.ClientPublish],
            supported_encodings=["json"],
            server_listener=self.listener,
        )

    def log(self, name, value):
        self.json_channels[name].log(value)

    def clear(self):
        with self.lock:
            self.snapshots.clear()
        now = time.time_ns()
        for channel in self.channels.values():
            channel.log(
                m.SceneUpdate(
                    deletions=[
                        m.SceneEntityDeletion(
                            timestamp=timestamp(now), type=m.SceneEntityDeletionType.All
                        )
                    ]
                )
            )

    def accept(self, message):
        if message["k"] == "extension":
            self.log("extension/rx", message["body"])
            return
        key = message.get("l", "vehicle")
        if key not in self.channels:
            raise ValueError("unknown scene layer")
        with self.lock:
            self.snapshots[key] = (message, time.monotonic())
        self.render(key, message, 0.0)

    def render(self, key, message, age):
        now = time.time_ns()
        lifetime = self.config["foxglove"]["stale_seconds"]
        frame = message["f"]
        source_age = age + message.get("a", 0)
        if message["k"] == "vehicle":
            if source_age > lifetime:
                self.log(
                    "state", {"stale": True, "speed_m_s": None, "receive_age_s": age}
                )
                records = []
            else:
                self.log(
                    "state",
                    {
                        "stale": False,
                        "speed_m_s": message.get("v"),
                        "source_ros_stamp_ns": message["t"],
                        "receive_age_s": round(age, 2),
                        "pose": message["p"],
                        "orientation": message["r"],
                        "frame_id": frame,
                    },
                )
                records = [
                    [
                        "vehicle",
                        0,
                        message["p"],
                        message["r"],
                        [1.8, 0.15, 0.35],
                        [40, 230, 180, 255],
                        [],
                        [],
                        "",
                    ]
                ]
        else:
            records = message["o"]
            # Live observations clear on stale; map/planned/trajectory remain, with age in /radio/layers.
            if key in ("guided", "fused", "control") and source_age > lifetime:
                records = []
        entities = [
            entity_from_record(
                r,
                frame,
                message["q"],
                now,
                lifetime if key in ("guided", "fused", "control", "vehicle") else 0,
                message["t"],
            )
            for r in records
        ]
        update = m.SceneUpdate(
            deletions=[
                m.SceneEntityDeletion(
                    timestamp=timestamp(now - 1), type=m.SceneEntityDeletionType.All
                )
            ],
            entities=entities,
        )
        self.channels[key].log(update)

    def refresh(self):
        # Re-emit cached geometry for a Foxglove client opened after radio reception.
        # The original source stamp is unchanged and age is ALWAYS exposed separately.
        now = time.monotonic()
        ages = {key: {"received": False, "stale": True} for key in self.channels}
        with self.lock:
            snapshots = list(self.snapshots.items())
        for key, (message, received) in snapshots:
            age = now - received
            self.render(key, message, age)
            ages[key] = {
                "received": True,
                "receive_age_s": round(age, 2),
                "source_age_lower_bound_s": round(age + message.get("a", 0), 2),
                "stale": age + message.get("a", 0)
                > self.config["foxglove"]["stale_seconds"],
                "source_ros_stamp_ns": message["t"],
                "sampling": message.get("s", {}),
            }
        self.log("layers", ages)
        if "vehicle" not in self.snapshots:
            self.log("state", {"stale": True, "speed_m_s": None, "receive_age_s": None})
        while not self.listener.errors.empty():
            self.log(
                "extension/status",
                {"accepted": False, "error": self.listener.errors.get_nowait()},
            )

    def close(self):
        self.server.stop()
