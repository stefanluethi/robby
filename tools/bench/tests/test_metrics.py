from __future__ import annotations

import pytest

from robby_bench import blip
from robby_bench.metrics import Metrics, interval_stats

MS = 1_000_000
T0 = 1_793_045_673_000_000_000


def add_values(m: Metrics, path: str, times: list[int], value: object = 1.0) -> None:
    for t in times:
        m.add_message(blip.value(t, path, value), len(blip.encode(blip.value(t, path, value))))


def test_interval_stats_and_gap() -> None:
    times = [T0 + i * MS for i in range(100)] + [T0 + (99 + 10 + i) * MS for i in range(100)]
    s = interval_stats(times)
    assert s["interval_ms"]["median"] == pytest.approx(1.0)
    assert s["interval_ms"]["max"] == pytest.approx(10.0)
    assert s["gaps"]["count"] == 1
    assert s["gaps"]["max_ms"] == pytest.approx(10.0)
    assert s["gaps"]["list"][0]["after_ns"] == T0 + 99 * MS
    assert s["non_increasing"] == 0


def test_interval_stats_counts_repeated_timestamps() -> None:
    s = interval_stats([T0, T0 + MS, T0 + MS, T0 + 2 * MS, T0 + MS])
    assert s["non_increasing"] == 2
    assert s["interval_ms"]["mean"] == pytest.approx(1.0)
    assert interval_stats([T0])["interval_ms"] is None


def test_per_axis_and_record_streams() -> None:
    m = Metrics()
    times = [T0 + i * 2 * MS for i in range(50)]
    for axis in "xyz":
        add_values(m, f"imu/acc/{axis}", times)
    add_values(m, "imu/gyr", times, {"x": 1.0, "y": 2.0, "z": 3.0})
    add_values(m, "tof/distance/x", times[:5])  # no y/z siblings: not a stream
    r = m.result(1.0)
    acc = r["streams"]["imu/acc"]
    assert acc["style"] == "per-axis"
    assert acc["samples"] == 50
    assert acc["bytes_per_sample"] == pytest.approx(3 * r["paths"]["imu/acc/x"]["bytes_per_msg"])
    assert r["streams"]["imu/gyr"]["style"] == "record"
    assert r["paths"]["imu/gyr"]["scalars_per_msg"] == 3
    assert "tof/distance" not in r["streams"]
    assert sum(p["bytes_share"] for p in r["paths"].values()) == pytest.approx(1.0)


def test_counters_delta_reset_and_drops() -> None:
    m = Metrics()
    for i, (drops, nbytes) in enumerate([(0, 100), (2, 200), (5, 300), (1, 400)]):
        t = T0 + i * 1000 * MS
        add_values(m, "outpost/queue_drops", [t], drops)
        add_values(m, "outpost/ws", [t], {"bytes": nbytes, "frames": i})
    r = m.result(4.0)
    drops = r["counters"]["outpost/queue_drops"]
    assert drops["drop"]
    assert drops["delta"] == 5 + 1  # 0 -> 5, then a reset and 1 more
    assert drops["resets"] == 1
    ws = r["counters"]["outpost/ws.bytes"]
    assert not ws["drop"]
    assert ws["per_s"] == pytest.approx(100.0)  # 300 B over the 3 s between the samples
    assert r["drops"] == {"total": 6, "by_counter": {"outpost/queue_drops": 6}}
    assert "outpost/ws" not in r["streams"]  # counters aren't sample streams


def test_counter_start_counts_increments_before_the_window() -> None:
    m = Metrics(counter_start={"link/drops": 3})
    add_values(m, "link/drops", [T0], 7)
    r = m.result(1.0)
    assert r["counters"]["link/drops"]["delta"] == 4
    assert r["drops"]["total"] == 4
    assert m.counters_last() == {"link/drops": 7}


def test_frames_and_decode_errors() -> None:
    m = Metrics()
    good = blip.encode(blip.value(T0, "a", 1))
    m.add_frame(good + good)
    m.add_frame(good + good[:5])
    m.add_text_frame()
    r = m.result(2.0)
    assert r["ws"]["frames"] == 2
    assert r["ws"]["bytes"] == 3 * len(good) + 5
    assert r["ws"]["text_frames"] == 1
    assert r["messages"]["count"] == 3
    assert r["messages"]["decode_errors"] == 1
    assert r["messages"]["per_frame_mean"] == 1.5
