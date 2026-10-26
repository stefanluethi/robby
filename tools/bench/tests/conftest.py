"""Fixtures: the simulated bridge (`robby_bench.sim`) and captures from it, once per session."""

from __future__ import annotations

import asyncio
from collections.abc import Callable, Iterator
from pathlib import Path
from typing import Any

import pytest

from robby_bench.capture import capture
from robby_bench.sim import SimConfig, SimServer

CAPTURE_S = 2.0

SimFactory = Callable[[SimConfig], SimServer]


@pytest.fixture
def sim() -> Iterator[SimFactory]:
    """Starts simulated bridges; stops them at the end of the test."""
    servers: list[SimServer] = []

    def start(cfg: SimConfig) -> SimServer:
        server = SimServer(cfg).start()
        servers.append(server)
        return server

    yield start
    for server in servers:
        server.stop()


def _capture(cfg: SimConfig, out: Path, **kwargs: Any) -> tuple[dict[str, Any], SimServer]:
    server = SimServer(cfg).start()
    try:
        result = asyncio.run(capture(server.url, CAPTURE_S, out, **kwargs))
    finally:
        server.stop()
    return result, server


@pytest.fixture(scope="session")
def runs_dir(tmp_path_factory: pytest.TempPathFactory) -> Path:
    return tmp_path_factory.mktemp("runs")


@pytest.fixture(scope="session")
def sonde_run(runs_dir: Path) -> dict[str, Any]:
    """Today's firmware: per-axis imu/acc/x,y,z at ~393/s in ~1 kB frames."""
    result, _ = _capture(
        SimConfig(style="sonde"),
        runs_dir / "h0-a.blip",
        label="H0-a baseline",
        meta={"rssi": "-60"},
    )
    return result


@pytest.fixture(scope="session")
def outpost_run(runs_dir: Path) -> dict[str, Any]:
    """The protobuf outpost: imu/acc {x,y,z} records at 806/s plus link/outpost counters."""
    result, _ = _capture(SimConfig(style="outpost"), runs_dir / "h1-a.blip", label="H1-a protobuf")
    return result
