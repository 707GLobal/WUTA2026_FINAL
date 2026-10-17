"""PC receiver, Foxglove publishing, and a loopback-only JSON extension endpoint."""

import json
import logging
import queue
import socket
import socketserver
import threading
import time
from .protocol import SLAVE
from .worker import RadioWorker

LOG = logging.getLogger(__name__)


def make_control_server(port, enqueue):
    class Handler(socketserver.StreamRequestHandler):
        def handle(self):
            self.connection.settimeout(2)
            try:
                raw = self.rfile.readline(514)
                if len(raw) > 512 or not raw.endswith(b"\n"):
                    raise ValueError(
                        "one JSON object per line; at most 511 bytes plus newline"
                    )
                body = json.loads(raw)
                enqueue(body)
                result = {
                    "queued": True,
                    "id": body.get("id"),
                    "note": "queued locally; NOT a peer delivery acknowledgement",
                }
            except (ValueError, TypeError, OSError) as exc:
                result = {"queued": False, "error": str(exc)}
            self.wfile.write(json.dumps(result).encode() + b"\n")

    # Serial handling is never performed by request handlers.
    server = socketserver.TCPServer(("127.0.0.1", port), Handler)
    server.timeout = 0.1
    return server


def send_extension(host, port, message):
    raw = (
        json.dumps(
            message, ensure_ascii=True, allow_nan=False, separators=(",", ":")
        ).encode()
        + b"\n"
    )
    if len(raw) > 512:
        raise ValueError("extension JSON too large")
    with socket.create_connection((host, port), timeout=3) as connection:
        connection.sendall(raw)
        reply = connection.makefile("rb").readline(2048)
    return json.loads(reply)


def run_pc(config, port, demo=False, duration=None):
    from .foxglove_view import FoxgloveView

    incoming = queue.Queue(maxsize=64)
    peer_changed = threading.Event()
    worker = RadioWorker(
        SLAVE, port, config, incoming.put_nowait, lambda session: peer_changed.set()
    )

    def enqueue(body):
        worker.outbox.extension(body)

    view = FoxgloveView(config, enqueue)
    control = None
    try:
        control = make_control_server(config["foxglove"]["control_port"], enqueue)
        control_thread = threading.Thread(
            target=control.serve_forever, kwargs={"poll_interval": 0.1}, daemon=True
        )
        control_thread.start()
        if not demo:
            worker.start()
        LOG.info(
            "Foxglove: ws://%s:%d (3D fixed frame %s)",
            config["foxglove"]["host"],
            view.server.port,
            config["fixed_frame"],
        )
        LOG.info(
            "extension API: 127.0.0.1:%d; no ROS required on PC",
            control.server_address[1],
        )
        began, last_refresh, last_demo = time.monotonic(), 0, 0
        while duration is None or time.monotonic() - began < duration:
            now = time.monotonic()
            if peer_changed.is_set():
                view.clear()
                peer_changed.clear()
            if demo and now - last_demo > 0.5:
                x = math_sine(now - began)
                view.accept(
                    {
                        "k": "vehicle",
                        "f": config["fixed_frame"],
                        "t": time.time_ns(),
                        "q": 0.05,
                        "p": [round(x / 0.05), 0, 0],
                        "r": [0, 0, 0, 1],
                        "v": 2.5,
                    }
                )
                view.accept(
                    {
                        "k": "scene",
                        "l": "path",
                        "f": config["fixed_frame"],
                        "t": time.time_ns(),
                        "q": 0.05,
                        "o": [
                            [
                                "demo:0",
                                4,
                                [0, 0, 0],
                                [0, 0, 0, 1],
                                [0.1, 0, 0],
                                [0, 210, 255, 255],
                                [[0, 0, 0], [80, 20, 0], [160, 0, 0]],
                                [],
                                "",
                            ]
                        ],
                        "s": {"demo": True},
                    }
                )
                last_demo = now
            while not incoming.empty():
                message = incoming.get_nowait()
                try:
                    view.accept(message)
                    if message["k"] == "extension":
                        LOG.info(
                            "extension RX: %s",
                            json.dumps(message["body"], ensure_ascii=False),
                        )
                except (ValueError, TypeError, KeyError, IndexError) as exc:
                    LOG.warning("invalid received display message: %s", exc)
            if now - last_refresh >= 1:
                view.refresh()
                view.log("link", {"demo": demo, **worker.status()})
                last_refresh = now
            time.sleep(0.02)
    except KeyboardInterrupt:
        pass
    finally:
        if worker.thread.is_alive():
            worker.close()
        if control:
            control.shutdown()
            control.server_close()
        view.close()


def math_sine(value):
    import math

    return 4 * math.sin(value / 3)
