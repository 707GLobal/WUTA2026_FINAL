import random
import zlib
import pytest
from fsd_radio.protocol import (
    Packet,
    Framer,
    MASTER,
    MAX_WIRE,
    pack_message,
    unpack_message,
    Reassembler,
)


@pytest.mark.parametrize("size", [0, 1, 2, 20, 95, 96])
def test_binary_payload_split_and_coalesced(size):
    rng = random.Random(size)
    payload = bytes(rng.randrange(256) for _ in range(size))
    packet = Packet(MASTER, 123, 42, 12, 8, 5, 0, 1, payload)
    wire = packet.encode()
    assert len(wire) <= MAX_WIRE < 240
    assert all(b < 128 for b in wire)
    assert Packet.decode(wire) == packet
    framer = Framer()
    result = []
    for byte in b"noise\x00\xff" + wire + wire:
        result += framer.feed(bytes([byte]))
    assert result == [packet, packet]


def test_corrupt_crc_oversized_noise_and_resync():
    packet = Packet(MASTER, 1, 1)
    wire = packet.encode()
    damaged = bytearray(wire)
    damaged[5] = ord("B") if damaged[5] != ord("B") else ord("C")
    f = Framer()
    assert f.feed(bytes(damaged)) == []
    assert f.feed(b"~" + b"a" * 1000 + wire) == [packet]
    assert f.errors >= 2
    with pytest.raises(ValueError):
        Packet(MASTER, 1, 1, count=2, index=2).encode()


def test_message_limits_decompression_bomb_and_nonobject():
    message = {"k": "extension", "body": {"text": "中文", "n": 2}}
    assert unpack_message(pack_message(message)) == message
    with pytest.raises(ValueError):
        pack_message({"k": "extension", "body": {"data": "x" * 10000}})
    with pytest.raises(ValueError):
        unpack_message(b"Z" + zlib.compress(b"A" * 100000))
    with pytest.raises(ValueError):
        unpack_message(b"J[]")
    with pytest.raises(ValueError):
        pack_message({"k": "vehicle", "n": float("nan")})


def test_atomic_reassembly_out_of_order_duplicate_timeout_and_restart():
    raw = pack_message({"k": "extension", "body": {"text": "test"}})
    pieces = [raw[i : i + 7] for i in range(0, len(raw), 7)]
    packets = [
        Packet(MASTER, 11, i + 1, message_id=3, index=i, count=len(pieces), payload=p)
        for i, p in enumerate(pieces)
    ]
    r = Reassembler(timeout=10)
    for p in reversed(packets[1:]):
        assert r.accept(p, now=1) is None
    result = r.accept(packets[0], now=2)
    assert result["body"]["text"] == "test"
    assert r.accept(packets[0], now=3) is None
    assert (
        r.accept(
            Packet(MASTER, 22, 1, message_id=3, index=0, count=1, payload=raw), now=4
        )
        == result
    )
    r = Reassembler(timeout=1)
    assert r.accept(packets[0], now=0) is None
    for p in packets[1:]:
        assert r.accept(p, now=3) is None
