"""report.py over captures and a ramp, and decode.py over a recording."""

from __future__ import annotations

import asyncio
import json
from pathlib import Path
from typing import Any

import pytest
from conftest import SimFactory

from robby_bench import decode, report
from robby_bench.ramp import RampConfig, ramp
from robby_bench.sim import SimConfig


@pytest.fixture
def ramp_run(sim: SimFactory, runs_dir: Path) -> dict[str, Any]:
    server = sim(SimConfig(style="outpost", capacity=900))
    cfg = RampConfig(rates=(300, 600, 1200), step_s=0.5, settle_s=0.2)
    return asyncio.run(ramp(server.url, runs_dir / "h1-b.blip", cfg, label="H1-b ramp"))


def test_report_compares_baseline_and_protobuf(
    sonde_run: dict[str, Any], outpost_run: dict[str, Any], ramp_run: dict[str, Any]
) -> None:
    text = report.render([outpost_run, sonde_run, ramp_run], baseline="h0-a", title="T")
    assert text.startswith("# T\n")
    assert "| h0-a | capture | H0-a baseline |" in text
    assert "rssi=-60" in text
    # the baseline comes first, the protobuf run with ratios to it
    assert "| metric | h0-a | h1-a |" in text
    row = next(line for line in text.splitlines() if line.startswith("| `imu/acc` samples/s"))
    assert "(2.0" in row  # 806 vs 393 samples/s
    row = next(line for line in text.splitlines() if line.startswith("| UART kB/s"))
    assert "(0.1" in row  # ~7 kB/s from the link counter vs ~62 kB/s on the baseline WS
    assert "h0-a: WS, h1-a: counter" in text
    assert "## h1-a: traces" in text
    assert "| `outpost/queue_drops` (drop) |" in text
    assert "## h1-b: ramp" in text
    assert "knee between **600** and **1,200** samples/s" in text
    assert "outpost/queue_drops +" in text


def test_report_cli_writes_results(
    sonde_run: dict[str, Any],
    outpost_run: dict[str, Any],
    runs_dir: Path,
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
) -> None:
    (tmp_path / ".git").mkdir()
    (tmp_path / "tools").mkdir()
    monkeypatch.chdir(tmp_path / "tools")
    files = [runs_dir / "h0-a.json", runs_dir / "h1-a.json"]
    assert report.main([*map(str, files), "--write"]) == 0
    written = list((tmp_path / "doc" / "bench").glob("results-*.md"))
    assert len(written) == 1
    assert "## Comparison" in written[0].read_text()
    assert report.main([*map(str, files), "--baseline", "nope"]) == 1
    assert "no capture named" in capsys.readouterr().err


def test_report_skips_other_json(tmp_path: Path) -> None:
    other = tmp_path / "other.json"
    other.write_text(json.dumps({"something": 1}))
    assert report.load_runs([other]) == []


def test_decode_summary(
    outpost_run: dict[str, Any], runs_dir: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    m = decode.recording_metrics(runs_dir / "h1-a.blip")
    assert m["messages"]["count"] == outpost_run["messages"]["count"]
    assert m["paths"]["imu/acc"]["count"] == outpost_run["paths"]["imu/acc"]["count"]
    assert m["streams"]["imu/acc"]["style"] == "record"
    assert m["timestamp_span_s"] == pytest.approx(outpost_run["duration_s"], abs=0.1)
    assert decode.main([str(runs_dir / "h1-a.blip"), "--messages", "2"]) == 0
    out = capsys.readouterr().out
    assert "'format': 'blip recording'" in out
    assert "stream imu/acc (record)" in out
    assert "counter outpost/queue_drops (drop)" in out
    assert out.count("[  ") + out.count("[ ") >= 2
