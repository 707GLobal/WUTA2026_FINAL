"""Actual pyserial socket transports, fragmented TCP delivery, and both endpoints."""

from copy import deepcopy
import select
import socket
import threading
import time
from fsd_radio.geometry import MarkerCache, scene_snapshot
from fsd_radio.link import Endpoint
from fsd_radio.protocol import MASTER, SLAVE
from fsd_radio.transport import SerialTransport
from test_geometry import marker


def test_pyserial_bidirectional_scene_transfer(config):
    listener = socket.socket()
    listener.bind(("127.0.0.1", 0))
    listener.listen(2)
    address = f"socket://127.0.0.1:{listener.getsockname()[1]}"
    first = SerialTransport(address)
    left, _ = listener.accept()
    second = SerialTransport(address)
    right, _ = listener.accept()
    stop = threading.Event()

    def relay():
        try:
            while not stop.is_set():
                ready, _, _ = select.select([left, right], [], [], 0.05)
                for source in ready:
                    # Tiny reads deliberately break up serial application frames.
                    data = source.recv(7)
                    if not data:
                        return
                    (right if source is left else left).sendall(data)
        except OSError:
            if not stop.is_set():
                raise

    relay_thread = threading.Thread(target=relay)
    relay_thread.start()
    c = deepcopy(config)
    c["link"].update(
        bytes_per_second=10000, response_timeout=0.5, turnaround_seconds=0.002
    )
    cache = MarkerCache()
    cache.apply([marker(frame="map")], 1)
    scene = scene_snapshot("path", cache, lambda *args: ((0, 0, 0), (0, 0, 0, 1)), c, 1)
    received_down, received_up = [], []
    master = Endpoint(MASTER, first, c, received_up.append)
    slave = Endpoint(SLAVE, second, c, received_down.append)
    master.outbox.update("path", scene)
    slave.outbox.extension({"type": "ping", "id": "serial-test"})
    threads = [threading.Thread(target=e.run, args=(stop,)) for e in (master, slave)]
    try:
        for thread in threads:
            thread.start()
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline and not (received_down and received_up):
            time.sleep(0.01)
        assert received_down[0] == scene
        assert received_up[0]["body"]["id"] == "serial-test"
        assert first.tx_bytes and first.rx_bytes and second.tx_bytes and second.rx_bytes
    finally:
        stop.set()
        for thread in threads:
            thread.join(2)
        first.close()
        second.close()
        left.close()
        right.close()
        listener.close()
        relay_thread.join(1)
