"""Linux is the master; PC only transmits in response to a valid poll.

Stop-and-wait per direction, latest telemetry, bounded extension FIFO, byte pacing.
The delivery callback must only enqueue work; it must not block the radio thread.
"""

from collections import deque
from dataclasses import dataclass
import logging
import queue
import secrets
import threading
import time
from .protocol import (
    MASTER,
    SLAVE,
    MAX_WIRE,
    MAX_FRAGMENTS,
    Packet,
    Reassembler,
    pack_message,
)

LOG = logging.getLogger(__name__)


@dataclass
class Job:
    key: str
    message_id: int
    pieces: list
    born: float
    index: int = 0


class Outbox:
    def __init__(self, config):
        self.config = config
        self.lock = threading.RLock()
        self.latest, self.active, self.last = {}, {}, {}
        self.extensions = deque()
        self.counter = 0
        self.sent = 0

    def update(self, key, message):
        # Factories are evaluated on the ROS thread, never inside radio I/O.
        packed = pack_message(message, self.config["link"]["max_message_bytes"])
        if key not in self.config["layers"]:
            raise ValueError("unknown telemetry layer")
        if len(packed) > self.config["link"]["fragment_bytes"] * MAX_FRAGMENTS:
            raise ValueError("message exceeds fragment count limit")
        with self.lock:
            self.latest[key] = packed

    def extension(self, body):
        if not isinstance(body, dict):
            raise ValueError("extension must be a JSON object")
        raw = pack_message({"k": "extension", "body": body}, 512)
        with self.lock:
            if len(self.extensions) >= self.config["link"]["extension_queue"]:
                raise ValueError("extension queue full")
            self.extensions.append(raw)

    def next_piece(self, now):
        with self.lock:
            candidates = []
            for key in self.latest:
                layer = self.config["layers"][key]
                if (
                    key in self.active
                    or now - self.last.get(key, -1e9) >= layer["interval"]
                ):
                    candidates.append(key)
            # Every fifth fragment can be extension traffic, never an unbounded priority flood.
            if self.extensions and (not candidates or self.sent % 5 == 0):
                key = "_extension"
            elif candidates:
                # Weighted least-recently-served scheduling also interleaves large snapshots.
                key = max(
                    candidates,
                    key=lambda k: (
                        (now - self.last.get(k, now - 60))
                        * self.config["layers"][k]["weight"]
                    ),
                )
            else:
                return None
            if key not in self.active:
                self.counter = (self.counter + 1) % 65536
                raw = self.extensions[0] if key == "_extension" else self.latest[key]
                size = self.config["link"]["fragment_bytes"]
                self.active[key] = Job(
                    key,
                    self.counter,
                    [raw[i : i + size] for i in range(0, len(raw), size)],
                    now,
                )
            job = self.active[key]
            if now - job.born > self.config["link"]["assembly_timeout"] * 0.75:
                self.finish_piece(job, False)
                return None
            self.last[key] = now
            self.sent += 1
            return job

    def finish_piece(self, job, success):
        with self.lock:
            if not success:
                LOG.warning(
                    "radio message failed: %s message=%d", job.key, job.message_id
                )
            job.index += 1
            if not success or job.index == len(job.pieces):
                self.active.pop(job.key, None)
                if job.key == "_extension":
                    self.extensions.popleft()


class Endpoint:
    def __init__(self, role, transport, config, on_message, on_peer=None):
        self.role, self.transport, self.config = role, transport, config
        self.on_message, self.on_peer = on_message, on_peer or (lambda session: None)
        self.outbox = Outbox(config)
        self.session, self.sequence = secrets.randbelow(0xFFFFFFFF) + 1, 0
        self.peer_session, self.peer_seq = 0, 0
        self.retired_peers = deque(maxlen=16)
        self.reassembler = Reassembler(
            config["link"]["assembly_timeout"], config["link"]["max_message_bytes"]
        )
        self.pending = None
        self.attempts = 0
        self.started = time.monotonic()
        self.last_received = None
        self.stats = {
            "packets_tx": 0,
            "packets_rx": 0,
            "retries": 0,
            "failed": 0,
            "bad_messages": 0,
        }

    def reset_outbox(self):
        with self.outbox.lock:
            if self.outbox.extensions:
                LOG.warning(
                    "peer/session changed: extension delivery unknown; queue discarded"
                )
            self.outbox.active.clear()
            self.outbox.extensions.clear()
            self.outbox.last.clear()
        self.pending = None

    def _new(self):
        self.sequence += 1
        if self.sequence > 65535:
            self.sequence = 1
            self.session = secrets.randbelow(0xFFFFFFFF) + 1
            self.reset_outbox()
        job = self.outbox.next_piece(time.monotonic())
        self.pending = (self.sequence, job)
        self.attempts = 0

    def packet(self):
        if self.pending is None:
            self._new()
        seq, job = self.pending
        return Packet(
            self.role,
            self.session,
            seq,
            self.peer_session,
            self.peer_seq,
            job.message_id if job else 0,
            job.index if job else 0,
            len(job.pieces) if job else 0,
            job.pieces[job.index] if job else b"",
        )

    def acknowledged(self, packet):
        if (
            self.pending
            and packet.ack_session == self.session
            and packet.ack_seq == self.pending[0]
        ):
            job = self.pending[1]
            if job:
                self.outbox.finish_piece(job, True)
            self.pending = None
            return True
        return False

    def receive(self, packet):
        if packet.role == self.role or packet.session in self.retired_peers:
            return False
        if packet.session != self.peer_session:
            if self.peer_session:
                self.retired_peers.append(self.peer_session)
                self.reset_outbox()
            self.peer_session, self.peer_seq = packet.session, 0
            self.reassembler.active.clear()
            self.on_peer(packet.session)
        self.last_received = time.monotonic()
        self.stats["packets_rx"] += 1
        self.acknowledged(packet)
        if packet.seq <= self.peer_seq:
            return True  # A retry is acknowledged but never delivered twice.
        self.peer_seq = packet.seq
        try:
            message = self.reassembler.accept(packet)
            if message is not None:
                self.on_message(message)
        except (ValueError, TypeError, KeyError, queue.Full) as exc:
            self.stats["bad_messages"] += 1
            LOG.warning("message rejected: %s", exc)
        return True

    def transmit(self):
        if self.pending is not None and self.attempts > self.config["link"]["retries"]:
            job = self.pending[1]
            if job:
                self.outbox.finish_piece(job, False)
            self.stats["failed"] += 1
            self.pending = None
        wire = self.packet().encode()
        if self.attempts:
            self.stats["retries"] += 1
        self.transport.write(wire)
        self.attempts += 1
        self.stats["packets_tx"] += 1
        return len(wire)

    def status(self):
        now = time.monotonic()
        return {
            **self.stats,
            "role": "linux" if self.role == MASTER else "pc",
            "peer_age_s": None
            if self.last_received is None
            else round(now - self.last_received, 2),
            "session": self.session,
            "peer_session": self.peer_session,
            "tx_bytes": self.transport.tx_bytes,
            "rx_bytes": self.transport.rx_bytes,
            "tx_Bps_average": round(
                self.transport.tx_bytes / max(now - self.started, 0.01), 2
            ),
            "aggregate_Bps_average": round(
                (self.transport.tx_bytes + self.transport.rx_bytes)
                / max(now - self.started, 0.01),
                2,
            ),
            "framing_errors": self.transport.framer.errors,
            "extension_queue": len(self.outbox.extensions),
        }

    def run(self, stop):
        settings = self.config["link"]
        while not stop.is_set():
            if self.role == MASTER:
                began = time.monotonic()
                sent = self.transmit()
                expected = self.pending[0]
                rx_before = self.transport.rx_bytes
                deadline = time.monotonic() + settings["response_timeout"]
                while not stop.is_set() and time.monotonic() < deadline:
                    reply = self.transport.receive(
                        min(0.1, max(0, deadline - time.monotonic()))
                    )
                    if (
                        reply
                        and reply.role == SLAVE
                        and reply.ack_session == self.session
                        and reply.ack_seq == expected
                    ):
                        if self.receive(reply):
                            break
                # Reserve even the largest reverse frame on timeout; all retry bytes count.
                received = max(
                    self.transport.rx_bytes - rx_before, MAX_WIRE if self.pending else 0
                )
                elapsed = time.monotonic() - began
                stop.wait(
                    max(0, (sent + received) / settings["bytes_per_second"] - elapsed)
                    + settings["turnaround_seconds"]
                )
            else:
                poll = self.transport.receive(0.1)
                if poll and poll.role == MASTER and self.receive(poll):
                    if stop.wait(settings["turnaround_seconds"]):
                        break
                    self.transmit()
