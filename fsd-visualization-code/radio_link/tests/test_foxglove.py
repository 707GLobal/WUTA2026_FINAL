from copy import deepcopy
import asyncio
import json
import struct
import threading
import websockets
from fsd_radio.foxglove_view import FoxgloveView, entity_from_record
from fsd_radio.pc_receiver import make_control_server, send_extension


def test_all_marker_types_sdk_serialization(config):
    from foxglove.channels import SceneUpdateChannel
    from foxglove import messages as m
    import foxglove

    context = foxglove.Context()
    channel = SceneUpdateChannel("/test/types", context=context)
    for typ in range(10):
        record = [
            "n:0",
            typ,
            [0, 0, 0],
            [0, 0, 0, 1],
            [1, 0.2, 0.3],
            [255, 0, 0, 255],
            [[0, 0, 0], [20, 0, 0]],
            [],
            "test",
        ]
        entity = entity_from_record(record, "map", 0.05, 123000000000, 5, 9)
        channel.log(m.SceneUpdate(entities=[entity]))


def test_real_sdk_websocket_advertise_subscribe_and_client_publish(config):
    c = deepcopy(config)
    c["foxglove"]["port"] = 0
    sent = []
    view = FoxgloveView(c, sent.append)

    async def client():
        async with websockets.connect(
            f"ws://127.0.0.1:{view.server.port}",
            subprotocols=["foxglove.sdk.v1", "foxglove.websocket.v1"],
        ) as ws:
            channels = []
            while not channels:
                frame = await asyncio.wait_for(ws.recv(), 3)
                if isinstance(frame, str):
                    obj = json.loads(frame)
                    if obj["op"] == "advertise":
                        channels = obj["channels"]
            assert any(ch["schemaName"] == "foxglove.SceneUpdate" for ch in channels)
            state = next(ch for ch in channels if ch["topic"] == "/radio/state")
            await ws.send(
                json.dumps(
                    {
                        "op": "subscribe",
                        "subscriptions": [{"id": 17, "channelId": state["id"]}],
                    }
                )
            )
            await asyncio.sleep(0.05)
            view.accept(
                {
                    "k": "vehicle",
                    "f": "map",
                    "t": 7,
                    "q": 0.05,
                    "p": [1, 2, 0],
                    "r": [0, 0, 0, 1],
                    "v": 3.2,
                }
            )
            while True:
                frame = await asyncio.wait_for(ws.recv(), 3)
                if (
                    isinstance(frame, bytes)
                    and frame[0] == 1
                    and struct.unpack_from("<I", frame, 1)[0] == 17
                ):
                    value = json.loads(frame[13:])
                    assert value["speed_m_s"] == 3.2
                    break
            await ws.send(
                json.dumps(
                    {
                        "op": "advertise",
                        "channels": [
                            {
                                "id": 22,
                                "topic": "/fsd_radio/extension/tx",
                                "encoding": "json",
                                "schemaName": "Extension",
                            }
                        ],
                    }
                )
            )
            await ws.send(
                b"\x01" + struct.pack("<I", 22) + b'{"type":"ping","id":"ws"}'
            )
            for _ in range(50):
                if sent:
                    break
                await asyncio.sleep(0.01)
            assert sent == [{"type": "ping", "id": "ws"}]

    try:
        asyncio.run(client())
    finally:
        view.close()


def test_local_extension_command_reports_queued_only():
    received = []
    server = make_control_server(0, received.append)
    thread = threading.Thread(
        target=server.serve_forever, kwargs={"poll_interval": 0.05}
    )
    thread.start()
    try:
        reply = send_extension(
            "127.0.0.1", server.server_address[1], {"type": "ping", "id": "cli"}
        )
        assert reply["queued"] and "NOT" in reply["note"]
        assert received == [{"type": "ping", "id": "cli"}]
    finally:
        server.shutdown()
        server.server_close()
        thread.join(1)
