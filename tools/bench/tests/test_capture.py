"""capture against the simulated bridge: metrics and the recorded file."""

from __future__ import annotations

import asyncio
import json
import subprocess
import sys
from pathlib import Path
from typing import Any

import pytest
from conftest import CAPTURE_S, SimFactory

from robby_bench import blip
from robby_bench.capture import capture
from robby_bench.recording import read_recording
from robby_bench.sim import OUTPOST_IMU_RATE, SONDE_IMU_RATE, UART_BYTES_PER_SAMPLE, SimConfig

BENCH_DIR = Path(__file__).parents[1]


def check_recording(result: dict[str, Any], blip_path: Path) -> None:
    """The .blip holds every received message, byte for byte."""
    rec = read_recording(blip_path)
    assert rec.header == {"format": "blip recording", "version": 1}
    assert rec.error is None
    assert len(rec.items) == result["messages"]["count"]
    assert sum(i.size for i in rec.items) == result["ws"]["bytes"]
    types: dict[str, int] = {}
    for item in rec.items:
        t = blip.message_type(item.obj)
        types[t] = types.get(t, 0) + 1
    assert types == result["messages"]["by_type"]


def test_sonde_metrics(sonde_run: dict[str, Any], runs_dir: Path) -> None:
    r = sonde_run
    assert r["kind"] == "capture"
    assert r["duration_s"] == pytest.approx(CAPTURE_S, abs=0.05)
    for axis in "xyz":
        p = r["paths"][f"imu/acc/{axis}"]
        assert p["rate_hz"] == pytest.approx(SONDE_IMU_RATE, rel=0.03)
        assert p["timestamp_rate_hz"] == pytest.approx(SONDE_IMU_RATE, rel=0.001)
        assert p["bytes_per_msg"] == pytest.approx(53, abs=0.1)  # a few floats fit in float16
        assert p["interval_ms"]["mean"] == pytest.approx(1000 / SONDE_IMU_RATE, rel=0.001)
        assert p["interval_ms"]["stddev"] < 0.001
        assert p["gaps"]["count"] == 0
    acc = r["streams"]["imu/acc"]
    assert acc["style"] == "per-axis"
    assert acc["bytes_per_sample"] == pytest.approx(159, abs=0.2)
    assert r["paths"]["tof/distance"]["rate_hz"] == pytest.approx(6, abs=1)
    assert r["ws"]["frame_bytes_max"] <= 1024
    assert r["ws"]["bytes_per_s"] == pytest.approx(r["ws"]["bytes"] / r["duration_s"])
    # about the 62 kB/s of the IMU on today's link
    assert 60_000 < r["ws"]["bytes_per_s"] < 70_000
    assert r["messages"]["per_s"] == pytest.approx(r["messages"]["count"] / r["duration_s"])
    assert r["messages"]["decode_errors"] == 0
    assert r["counters"] == {}
    assert r["drops"]["total"] == 0
    assert r["meta"] == {"rssi": "-60"}
    assert json.loads((runs_dir / "h0-a.json").read_text())["ws"] == r["ws"]
    check_recording(r, runs_dir / "h0-a.blip")


def test_outpost_metrics(outpost_run: dict[str, Any], runs_dir: Path) -> None:
    r = outpost_run
    acc = r["streams"]["imu/acc"]
    assert acc["style"] == "record"
    assert acc["rate_hz"] == pytest.approx(OUTPOST_IMU_RATE, rel=0.03)
    assert acc["interval_ms"]["mean"] == pytest.approx(1000 / OUTPOST_IMU_RATE, rel=0.001)
    assert r["paths"]["imu/acc"]["scalars_per_msg"] == 3
    assert r["paths"]["imu/acc"]["bytes_share"] > 0.9
    assert r["messages"]["by_type"]["Actions"] == 1
    c = r["counters"]
    assert c["outpost/queue_drops"]["last"] == 0
    assert c["outpost/queue_drops"]["drop"]
    assert c["outpost/seq_gaps"]["drop"]
    assert not c["link/tx_bytes"]["drop"]
    assert c["link/tx_bytes"]["per_s"] == pytest.approx(
        UART_BYTES_PER_SAMPLE * OUTPOST_IMU_RATE, rel=0.03
    )
    assert c["outpost/ws.frames"]["last"] > 0
    assert c["outpost/rssi"]["last"] == -58
    assert r["drops"]["total"] == 0
    # several messages per frame, as the outpost batches them
    assert r["messages"]["per_frame_mean"] > 1.5
    check_recording(r, runs_dir / "h1-a.blip")


def test_gap_is_found(sim: SimFactory, tmp_path: Path) -> None:
    server = sim(SimConfig(style="sonde", imu_gaps=((0.8, 0.1),)))
    r = asyncio.run(capture(server.url, 1.5, tmp_path / "gap.blip"))
    gaps = r["paths"]["imu/acc/x"]["gaps"]
    assert gaps["count"] == 1
    assert gaps["max_ms"] == pytest.approx(100 + 1000 / SONDE_IMU_RATE, abs=2.6)
    assert r["streams"]["imu/acc"]["gaps"] == 1
    assert r["paths"]["imu/acc/x"]["interval_ms"]["max"] == pytest.approx(gaps["max_ms"])


def test_one_client_at_a_time(sim: SimFactory, tmp_path: Path) -> None:
    server = sim(SimConfig(style="sonde"))

    async def two() -> None:
        first = asyncio.create_task(capture(server.url, 1.0, tmp_path / "a.blip"))
        await asyncio.sleep(0.3)
        with pytest.raises(ConnectionError, match="can't connect"):
            await capture(server.url, 0.5, tmp_path / "b.blip", connect_timeout_s=0.5)
        await first

    asyncio.run(two())
    assert server.state.rejected >= 1


def test_capture_cli(sim: SimFactory, tmp_path: Path) -> None:
    server = sim(SimConfig(style="outpost"))
    out = tmp_path / "runs" / "cli.blip"
    proc = subprocess.run(
        [
            sys.executable,
            str(BENCH_DIR / "capture.py"),
            server.url,
            "--seconds",
            "1",
            "--out",
            str(out),
            "--label",
            "cli",
            "--meta",
            "channel=6",
        ],
        capture_output=True,
        text=True,
        timeout=30,
        check=False,
    )
    assert proc.returncode == 0, proc.stderr
    assert "imu/acc (record)" in proc.stdout
    data = json.loads(out.with_suffix(".json").read_text())
    assert data["label"] == "cli"
    assert data["meta"] == {"channel": "6"}
    assert data["recording"] == "cli.blip"
    check_recording(data, out)


def test_capture_cli_unreachable(tmp_path: Path) -> None:
    proc = subprocess.run(
        [
            sys.executable,
            str(BENCH_DIR / "capture.py"),
            "ws://127.0.0.1:9/ws/robby",
            "--seconds",
            "1",
            "--out",
            str(tmp_path / "x.blip"),
            "--connect-timeout",
            "0.5",
        ],
        capture_output=True,
        text=True,
        timeout=30,
        check=False,
    )
    assert proc.returncode == 1
    assert "can't connect" in proc.stderr
