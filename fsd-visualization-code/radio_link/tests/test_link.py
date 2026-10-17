from copy import deepcopy
import queue
import threading
import time
import random
from fsd_radio.link import Endpoint, Outbox
from fsd_radio.protocol import MASTER, SLAVE, Framer, Packet


class MemoryTransport:
    def __init__(self):
        self.q = queue.Queue()
        self.other = None
        self.drop = 0
        self.tx_bytes = self.rx_bytes = 0
        self.framer = Framer()

    def write(self, data):
        self.tx_bytes += len(data)
        if self.drop:
            self.drop -= 1
            return
        self.other.q.put(data)

    def receive(self, timeout):
        try:
            data = self.q.get(timeout=timeout)
        except queue.Empty:
            return None
        self.rx_bytes += len(data)
        packets = self.framer.feed(data)
        return packets[0] if packets else None


def pair(config, on_master, on_slave):
    a, b = MemoryTransport(), MemoryTransport()
    a.other, b.other = b, a
    return Endpoint(MASTER, a, config, on_master), Endpoint(SLAVE, b, config, on_slave)


def test_retry_duplicate_ack_and_session_guard(config):
    rx = []
    a, b = pair(config, lambda msg: None, rx.append)
    a.outbox.extension({"id": "one", "text": "hello"})
    packet = a.packet()
    assert b.receive(packet)
    assert b.receive(packet)
    assert len(rx) == 1
    reply = b.packet()
    assert a.receive(reply) and a.pending is None
    assert not a.acknowledged(reply)
    assert b.receive(Packet(MASTER, 999, 1))
    assert not b.receive(packet)  # Never resurrect the retired sender session.


def test_live_bidirectional_fragments_with_dropped_ack(config):
    c = deepcopy(config)
    c["link"].update(
        bytes_per_second=10000, response_timeout=0.08, turnaround_seconds=0.002
    )
    c["link"]["fragment_bytes"] = 32
    master_rx, slave_rx = [], []
    a, b = pair(c, master_rx.append, slave_rx.append)
    # incompressible-ish multi-fragment messages in both directions.
    text = "".join(
        random.Random(22).choices("abcdefghijklmnopqrstuvwxyz0123456789", k=280)
    )
    a.outbox.extension({"id": "down", "text": text})
    b.outbox.extension({"id": "up", "text": text})
    b.transport.drop = 1
    stop = threading.Event()
    threads = [threading.Thread(target=e.run, args=(stop,)) for e in (a, b)]
    for thread in threads:
        thread.start()
    try:
        end = time.monotonic() + 4
        while time.monotonic() < end and not (master_rx and slave_rx):
            time.sleep(0.01)
        assert len(master_rx) == len(slave_rx) == 1
        assert master_rx[0]["body"]["id"] == "up"
        assert slave_rx[0]["body"]["id"] == "down"
        assert a.stats["retries"] >= 1
    finally:
        stop.set()
        for thread in threads:
            thread.join(2)
    assert not any(t.is_alive() for t in threads)


def test_no_unsolicited_pc_transmit_and_queue_bound(config):
    a, b = pair(config, lambda msg: None, lambda msg: None)
    b.outbox.extension({"id": "waiting"})
    stop = threading.Event()
    thread = threading.Thread(target=b.run, args=(stop,))
    thread.start()
    time.sleep(0.15)
    stop.set()
    thread.join(1)
    assert b.transport.tx_bytes == 0
    box = Outbox(config)
    for i in range(config["link"]["extension_queue"]):
        box.extension({"id": i})
    import pytest

    with pytest.raises(ValueError):
        box.extension({"id": "overflow"})


def test_successful_last_retry_does_not_fail_next_packet(config):
    a, b = pair(config, lambda message: None, lambda message: None)
    a.outbox.extension({"id": "last-attempt"})
    for _ in range(3):
        a.transmit()
    assert b.receive(a.packet())
    assert a.receive(b.packet())
    assert a.pending is None and a.attempts == 3
    a.transmit()  # Previously indexed None after a successful third attempt.
    assert a.pending is not None and a.attempts == 1


def test_sequence_rollover_restarts_partial_snapshot(config):
    c = deepcopy(config)
    c["link"]["fragment_bytes"] = 32
    a, b = pair(c, lambda message: None, lambda message: None)
    data = "".join(random.Random(17).choices("abcdef0123456789", k=250))
    a.outbox.update("path", {"k": "scene", "l": "path", "data": data})
    a.sequence = 65534
    first = a.packet()
    assert first.index == 0 and first.count > 1
    b.receive(first)
    a.receive(b.packet())
    second = a.packet()
    assert second.session != first.session and second.seq == 1 and second.index == 0


def test_aggregate_byte_pacing_both_directions(config):
    a, b = pair(config, lambda msg: None, lambda msg: None)
    for i in range(4):
        a.outbox.extension({"id": i, "text": "x" * 80})
        b.outbox.extension({"id": i, "text": "y" * 80})
    stop = threading.Event()
    start = time.monotonic()
    threads = [threading.Thread(target=e.run, args=(stop,)) for e in (a, b)]
    for t in threads:
        t.start()
    time.sleep(2.5)
    stop.set()
    for t in threads:
        t.join(2)
    elapsed = time.monotonic() - start
    # Initial one-frame burst is bounded; no independent per-direction full-rate budget.
    assert (
        a.transport.tx_bytes + b.transport.tx_bytes
        <= elapsed * config["link"]["bytes_per_second"] + 162
    )
