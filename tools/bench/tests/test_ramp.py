"""ramp against the simulated outpost, which drops above its capacity."""

from __future__ import annotations

import asyncio
import json
from pathlib import Path
from typing import Any

import pytest
from conftest import SimFactory

from robby_bench.ramp import RampConfig, parse_rates, ramp
from robby_bench.recording import read_recording
from robby_bench.sim import SimConfig

FAST = {"step_s": 0.6, "settle_s": 0.3}


def run(url: str, out: Path, **cfg: Any) -> dict[str, Any]:
    return asyncio.run(ramp(url, out, RampConfig(**{**FAST, **cfg})))


def test_parse_rates() -> None:
    assert parse_rates("1k, 2.5k,800") == (1000, 2500, 800)


def test_ramp_stops_at_the_knee(sim: SimFactory, tmp_path: Path) -> None:
    server = sim(SimConfig(style="outpost", capacity=1500))
    # a step longer than the 1 s counter period sees a counter value of its own
    r = run(server.url, tmp_path / "ramp.blip", rates=(200, 500, 1000, 2000, 5000), step_s=1.2)
    assert [s["rate"] for s in r["steps"]] == [200, 500, 1000, 2000]
    assert r["knee"] == {"last_ok": 1000, "first_bad": 2000, "reached_end": False}
    for s in r["steps"][:3]:
        assert s["ok"], s["reason"]
        assert s["ratio"] == pytest.approx(1.0, abs=0.05)
        assert s["drops"] == 0
        assert s["drop_counter_samples"] >= 1
    bad = r["steps"][3]
    assert "outpost/queue_drops" in bad["reason"]
    assert bad["drops"] > 0
    assert bad["delivered_rate"] == pytest.approx(1500, rel=0.05)
    # one connection for the whole ramp; the load is switched off at the end
    assert server.state.connections == 1
    assert server.state.rejected == 0
    assert [i["args"]["rate"] for i in server.state.invokes] == [200, 500, 1000, 2000, 0]
    assert all(i["path"] == "bench/load" for i in server.state.invokes)
    assert server.state.load_rate == 0
    # the whole ramp is in one recording, the step metrics in the json
    assert json.loads((tmp_path / "ramp.json").read_text())["knee"] == r["knee"]
    assert len(read_recording(tmp_path / "ramp.blip").items) > sum(
        s["metrics"]["messages"]["count"] for s in r["steps"]
    )


def test_ramp_detects_a_low_rate_without_counters(sim: SimFactory, tmp_path: Path) -> None:
    server = sim(SimConfig(style="outpost", capacity=800, report_drops=False))
    r = run(server.url, tmp_path / "ramp.blip", rates=(300, 600, 1200, 2400))
    assert r["knee"]["last_ok"] == 600
    assert r["knee"]["first_bad"] == 1200
    assert "delivered" in r["steps"][-1]["reason"]
    assert r["steps"][-1]["drops"] == 0


def test_ramp_without_knee(sim: SimFactory, tmp_path: Path) -> None:
    server = sim(SimConfig(style="outpost"))
    r = run(server.url, tmp_path / "ramp.blip", rates=(200, 400))
    assert r["knee"] == {"last_ok": 400, "first_bad": None, "reached_end": True}


def test_ramp_on_firmware_without_the_action(sim: SimFactory, tmp_path: Path) -> None:
    server = sim(SimConfig(style="sonde"))
    r = run(server.url, tmp_path / "ramp.blip", rates=(200,), reset=False)
    assert r["steps"] == []
    assert "rejected: unknown action" in r["error"]
