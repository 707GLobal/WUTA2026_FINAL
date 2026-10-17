import math
import pytest
from fsd_radio.geometry import (
    MarkerCache,
    compact_marker,
    scene_snapshot,
    sample_indices,
    transform_point,
)
from fsd_radio.protocol import pack_message


def marker(**updates):
    data = {
        "ns": "test",
        "id": 1,
        "type": 4,
        "action": 0,
        "stamp": 1,
        "frame": "sensor",
        "frame_locked": False,
        "lifetime_ns": 0,
        "position": (1.0, 0.0, 0.0),
        "orientation": (0.0, 0.0, 0.0, 1.0),
        "scale": (0.1, 0.1, 0.1),
        "color": [255, 0, 0, 255],
        "points": [(float(i), 0.0, 0.0) for i in range(100)],
        "colors": [],
        "text": "",
    }
    data.update(updates)
    return data


def test_transform_quantization_and_line_endpoints():
    transform = ((10.0, 0.0, 0.0), (0, 0, math.sqrt(0.5), math.sqrt(0.5)))
    record = compact_marker(marker(), lambda frame, stamp: transform, 0.05, 8, 64)
    assert record[2] == [200, 20, 0]
    assert record[6][0] == [0, 0, 0]
    assert record[6][-1] == [1980, 0, 0]
    assert transform_point(transform, (1, 0, 0)) == pytest.approx((10, 1, 0))
    pairs = sample_indices(100, 12, pairs=True)
    assert all(pairs[i + 1] == pairs[i] + 1 for i in range(0, len(pairs), 2))


def test_marker_delete_expiry_backward_clock_and_capacity():
    cache = MarkerCache(2)
    cache.apply([marker(lifetime_ns=10)], 100)
    assert cache.live(109)
    assert not cache.live(110)
    cache.apply([marker(id=i, stamp=200) for i in range(3)], 200)
    assert len(cache.live(200)) == 2 and cache.evicted == 1
    cache.apply([marker(id=2, action=2, stamp=201)], 201)
    assert len(cache.live(201)) == 1
    cache.apply([marker(stamp=1)], 202)
    assert len(cache.live(202)) == 1
    cache.apply([marker(action=3, stamp=2)], 202)
    assert cache.live(202) == []


def test_bounded_scene_and_tf_failure_clear(config):
    cache = MarkerCache(100)
    cache.apply([marker(id=i) for i in range(70)], 1)
    lookup = lambda frame, stamp: ((0, 0, 0), (0, 0, 0, 1))
    result = scene_snapshot("map", cache, lookup, config, 1)
    assert len(pack_message(result)) <= config["geometry"]["snapshot_bytes"]
    assert result["s"]["omitted_markers"] > 0
    assert result["s"]["source_points"] == 7000

    def missing(*args):
        raise LookupError("no TF")

    result = scene_snapshot("map", cache, missing, config, 1)
    assert result["o"] == [] and result["s"]["tf_or_geometry_drops"] > 0


def test_no_false_line_connection_across_nan_and_unsupported_mesh():
    lookup = lambda *args: ((0, 0, 0), (0, 0, 0, 1))
    with pytest.raises(ValueError):
        compact_marker(
            marker(points=[(0, 0, 0), (float("nan"), 0, 0), (1, 0, 0)]),
            lookup,
            0.05,
            12,
            64,
        )
    with pytest.raises(ValueError):
        compact_marker(marker(type=10), lookup, 0.05, 12, 64)
