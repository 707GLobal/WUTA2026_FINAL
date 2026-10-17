"""Reconnect only the selected radio, with bounded queues and visible failures."""

import logging
import threading
from .link import Endpoint, Outbox
from .transport import SerialTransport

LOG = logging.getLogger(__name__)


class RadioWorker:
    def __init__(self, role, port, config, on_message, on_peer=None):
        self.role, self.port, self.config = role, port, config
        self.on_message, self.on_peer = on_message, on_peer
        self.outbox = Outbox(config)
        self.endpoint = None
        self.error = None
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self.run, name="e22-radio", daemon=True)

    def start(self):
        self.thread.start()

    def run(self):
        while not self.stop_event.is_set():
            transport = None
            try:
                transport = SerialTransport(self.port, self.config["serial"]["baud"])
                endpoint = Endpoint(
                    self.role, transport, self.config, self.on_message, self.on_peer
                )
                endpoint.outbox = self.outbox
                self.endpoint, self.error = endpoint, None
                LOG.info("radio connected: %s", self.port)
                endpoint.run(self.stop_event)
            except (OSError, ValueError) as exc:
                self.error = str(exc)
                LOG.warning("radio disconnected: %s", exc)
                # Old fragments cannot be completed in a new session. Extensions may
                # have reached the peer: never silently replay them after reconnect.
                with self.outbox.lock:
                    if self.outbox.extensions:
                        LOG.warning(
                            "extensions discarded after disconnect; delivery is unknown"
                        )
                    self.outbox.extensions.clear()
                    self.outbox.active.clear()
                    self.outbox.last.clear()
                self.endpoint = None
            finally:
                if transport:
                    transport.close()
            self.stop_event.wait(1)

    def status(self):
        return {
            "connected": self.endpoint is not None,
            "port": self.port,
            "error": self.error,
            **(self.endpoint.status() if self.endpoint else {}),
        }

    def close(self):
        self.stop_event.set()
        self.thread.join(timeout=6)
