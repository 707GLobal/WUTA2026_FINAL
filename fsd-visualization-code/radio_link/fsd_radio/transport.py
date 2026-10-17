"""One owner for each serial port; partial writes and disconnects are explicit."""

import time
import serial
from .protocol import Framer


class SerialTransport:
    def __init__(self, port, baud=9600):
        if not port:
            raise ValueError(
                "Linux radio port is required; use --port /dev/serial/by-id/..."
            )
        self.serial = serial.serial_for_url(
            port,
            baudrate=baud,
            bytesize=8,
            parity="N",
            stopbits=1,
            timeout=0.05,
            write_timeout=2,
            xonxoff=False,
            rtscts=False,
            dsrdtr=False,
        )
        self.framer = Framer()
        self.received = []
        self.tx_bytes = self.rx_bytes = 0

    def write(self, data):
        offset, deadline = 0, time.monotonic() + 2
        while offset < len(data):
            count = self.serial.write(data[offset:])
            if not count or time.monotonic() > deadline:
                raise OSError("serial write did not complete")
            offset += count
            self.tx_bytes += count
        self.serial.flush()

    def receive(self, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.received:
                return self.received.pop(0)
            data = self.serial.read(min(max(self.serial.in_waiting, 1), 1024))
            self.rx_bytes += len(data)
            self.received.extend(self.framer.feed(data))
        return self.received.pop(0) if self.received else None

    def close(self):
        self.serial.close()
