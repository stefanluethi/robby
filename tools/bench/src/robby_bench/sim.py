"""A simulated robby bridge: serves synthetic blip frames over a WebSocket, for testing the tools.

    uv run simulate.py --style sonde      # today's firmware: imu/acc/x,y,z at ~393/s, ~1 kB frames
    uv run simulate.py --style outpost    # protobuf outpost: imu/acc {x,y,z} at 806/s + counters

Like the bridge it serves one client at a time on `/ws/robby`. The outpost style answers the
`bench/load` action {rate: N}: it adds `bench/synthetic` records {x, y, z} at N samples/s, but
delivers at most `--capacity` samples/s and counts the rest in `outpost/queue_drops`.
"""

from __future__ import annotations

import argparse
import asyncio
import contextlib
import math
import threading
import time
from dataclasses import dataclass, field
from http import HTTPStatus
from typing import Any, Literal

from websockets.asyncio.server import Server, ServerConnection, serve
from websockets.exceptions import ConnectionClosed
from websockets.http11 import Request, Response

from robby_bench import blip

Style = Literal["sonde", "outpost"]

WS_PATH = "/ws/robby"
SONDE_IMU_RATE = 393.0
OUTPOST_IMU_RATE = 806.0
TOF_RATE = 6.0
SLOW_RATE = 1.0
TASKS = ("data_publisher", "imu", "tof", "command_handler")
UART_BYTES_PER_SAMPLE = 9  # rough size of an IMU sample in an ImuBatch, for link/tx_bytes


@dataclass(frozen=True)
class SimConfig:
    style: Style = "sonde"
    imu_rate: float | None = None
    """IMU samples/s; default 393 (sonde) or 806 (outpost)."""
    capacity: float = math.inf
    """Most synthetic samples/s delivered; the excess is dropped."""
    report_drops: bool = True
    """Whether the dropped samples show in `outpost/queue_drops`."""
    imu_gaps: tuple[tuple[float, float], ...] = ()
    """(start s, length s) after connecting without IMU samples, to test gap detection."""
    tick_s: float = 0.005
    frame_limit: int | None = None
    """Largest WS frame; default 1024 B (sonde batch) or 64 KiB (outpost)."""
    action: str = "bench/load"
    load_path: str = "bench/synthetic"

    @property
    def imu_hz(self) -> float:
        if self.imu_rate is not None:
            return self.imu_rate
        return SONDE_IMU_RATE if self.style == "sonde" else OUTPOST_IMU_RATE

    @property
    def frame_bytes(self) -> int:
        if self.frame_limit is not None:
            return self.frame_limit
        return 1024 if self.style == "sonde" else 65536


@dataclass
class SimState:
    """What the server saw, for the tests."""

    connections: int = 0
    rejected: int = 0
    active: int = 0
    invokes: list[dict[str, Any]] = field(default_factory=list)
    load_rate: int = 0


class _Periodic:
    """Counts the events of a fixed rate due since a start time."""

    def __init__(self, rate: float, start: float) -> None:
        self.rate = rate
        self.start = start
        self.done = 0

    def due(self, now: float) -> range:
        target = math.floor((now - self.start) * self.rate) + 1 if self.rate > 0 else 0
        first = self.done
        self.done = max(self.done, target)
        return range(first, self.done)

    def time_of(self, i: int) -> float:
        return self.start + i / self.rate


class _Connection:
    def __init__(self, ws: ServerConnection, cfg: SimConfig, state: SimState) -> None:
        self.ws = ws
        self.cfg = cfg
        self.state = state
        now = time.monotonic()
        self.t0 = now
        self.t0_ns = time.time_ns()
        self.imu = _Periodic(cfg.imu_hz, now)
        self.tof = _Periodic(TOF_RATE, now)
        self.slow = _Periodic(SLOW_RATE, now)
        self.load: _Periodic | None = None
        self.load_requested = 0.0
        self.load_since = now
        self.drops_before = 0.0
        self.imu_samples = 0
        self.load_samples = 0
        self.ws_bytes = 0
        self.ws_frames = 0
        self.pending: list[dict[str, Any]] = []

    def ns(self, t: float) -> int:
        return self.t0_ns + round((t - self.t0) * 1e9)

    def drops(self, now: float) -> float:
        """Synthetic samples dropped so far."""
        if not self.cfg.report_drops:
            return 0.0
        excess = max(0.0, self.load_requested - self.cfg.capacity)
        return self.drops_before + excess * (now - self.load_since)

    def set_load(self, rate: float) -> None:
        now = time.monotonic()
        self.drops_before = self.drops(now)
        self.load_requested = rate
        self.load_since = now
        delivered = min(rate, self.cfg.capacity)
        self.load = _Periodic(delivered, now) if delivered > 0 else None

    def in_gap(self, t: float) -> bool:
        return any(s <= t - self.t0 < s + length for s, length in self.cfg.imu_gaps)

    def messages(self, now: float) -> list[dict[str, Any]]:
        out, self.pending = self.pending, []
        sonde = self.cfg.style == "sonde"
        for i in self.imu.due(now):
            t = self.imu.time_of(i)
            if self.in_gap(t):
                continue
            self.imu_samples += 1
            x, y, z = (blip.float32(0.01 * math.sin(i / 50 + k) + (k == 2)) for k in range(3))
            if sonde:
                out += [
                    blip.value(self.ns(t), "imu/acc/x", x),
                    blip.value(self.ns(t), "imu/acc/y", y),
                    blip.value(self.ns(t), "imu/acc/z", z),
                ]
            else:
                out.append(blip.value(self.ns(t), "imu/acc", {"x": x, "y": y, "z": z}))
        for i in self.tof.due(now):
            image = [[(r * 8 + c + i) % 2000 for c in range(8)] for r in range(8)]
            out.append(blip.value(self.ns(self.tof.time_of(i)), "tof/distance", image))
        if self.load is not None:
            for i in self.load.due(now):
                self.load_samples += 1
                out.append(
                    blip.value(
                        self.ns(self.load.time_of(i)),
                        self.cfg.load_path,
                        {"x": i % 1000, "y": -(i % 777), "z": i % 333},
                    )
                )
        for i in self.slow.due(now):
            out += self.slow_messages(self.ns(self.slow.time_of(i)), now, first=i == 0)
        return out

    def slow_messages(self, t: int, now: float, first: bool) -> list[dict[str, Any]]:
        sonde = self.cfg.style == "sonde"
        out = [
            blip.value(t, "battery/soc", 87),
            blip.value(t, "battery/voltage", blip.float32(7.83)),
            blip.value(t, "battery/current", -412),
        ]
        out += [
            blip.value(t, f"tasks/{name}", blip.float32(1.5 * k)) for k, name in enumerate(TASKS)
        ]
        if sonde or first:
            imu_paths = ["imu/acc/x", "imu/acc/y", "imu/acc/z"] if sonde else ["imu/acc"]
            out += [blip.value_metadata(p, "g") for p in imu_paths]
            out += [
                blip.value_metadata("tof/distance", "mm"),
                blip.value_metadata("battery/voltage", "V"),
            ]
        if not sonde:
            if first:
                out.append(
                    {
                        "Actions": [
                            {
                                "path": self.cfg.action,
                                "description": "synthetic load",
                                "params": [
                                    {
                                        "name": "rate",
                                        "unit": "samples/s",
                                        "kind": {"Int": {"min": 0, "max": 200000}},
                                    }
                                ],
                            }
                        ]
                    }
                )
            uart_bytes = UART_BYTES_PER_SAMPLE * (self.imu_samples + self.load_samples)
            out += [
                blip.value(t, "link/tx_bytes", uart_bytes),
                blip.value(t, "link/tx_frames", self.imu_samples // 8),
                blip.value(t, "link/drops", 0),
                blip.value(t, "outpost/uart_frames", self.imu_samples // 8),
                blip.value(t, "outpost/seq_gaps", 0),
                blip.value(t, "outpost/crc_errors", 0),
                blip.value(t, "outpost/queue_drops", round(self.drops(now))),
                blip.value(
                    t,
                    "outpost/ws",
                    {
                        "bytes": self.ws_bytes,
                        "frames": self.ws_frames,
                        "send_max_us": 850,
                    },
                ),
                blip.value(t, "outpost/rssi", -58),
            ]
        return out

    def handle_request(self, data: str | bytes) -> None:
        if isinstance(data, str):
            return
        items, _ = blip.decode_sequence(data)
        for item in items:
            body = item.obj.get("Invoke") if isinstance(item.obj, dict) else None
            if not isinstance(body, dict):
                continue
            self.state.invokes.append(body)
            request_id = body.get("id", 0)
            args = body.get("args") or {}
            rate = args.get("rate") if isinstance(args, dict) else None
            if self.cfg.style == "sonde" or body.get("path") != self.cfg.action:
                self.pending.append(blip.action_result(request_id, "unknown action"))
            elif not isinstance(rate, int) or rate < 0:
                self.pending.append(blip.action_result(request_id, "rate must be an int >= 0"))
            else:
                self.set_load(rate)
                self.state.load_rate = rate
                self.pending.append(blip.action_result(request_id))

    def frames(self, messages: list[dict[str, Any]]) -> list[bytes]:
        frames: list[bytes] = []
        current = bytearray()
        for m in messages:
            data = blip.encode(m)
            if current and len(current) + len(data) > self.cfg.frame_bytes:
                frames.append(bytes(current))
                current = bytearray()
            current += data
        if current:
            frames.append(bytes(current))
        return frames

    async def run(self) -> None:
        async def receive() -> None:
            async for data in self.ws:
                self.handle_request(data)

        async def send() -> None:
            while True:
                for frame in self.frames(self.messages(time.monotonic())):
                    await self.ws.send(frame)
                    self.ws_bytes += len(frame)
                    self.ws_frames += 1
                await asyncio.sleep(self.cfg.tick_s)

        receiver = asyncio.create_task(receive())
        sender = asyncio.create_task(send())
        try:
            await asyncio.wait({receiver, sender}, return_when=asyncio.FIRST_COMPLETED)
        finally:
            for task in (receiver, sender):
                task.cancel()
                with contextlib.suppress(asyncio.CancelledError, ConnectionClosed):
                    await task


async def serve_sim(
    cfg: SimConfig, state: SimState, host: str = "127.0.0.1", port: int = 0
) -> Server:
    """Starts the server; the caller closes it."""

    def process_request(conn: ServerConnection, request: Request) -> Response | None:
        if request.path.split("?")[0] != WS_PATH:
            return conn.respond(HTTPStatus.NOT_FOUND, "not found\n")
        if state.active > 0:
            state.rejected += 1
            return conn.respond(HTTPStatus.SERVICE_UNAVAILABLE, "one client at a time\n")
        return None

    async def handler(ws: ServerConnection) -> None:
        state.connections += 1
        state.active += 1
        try:
            await _Connection(ws, cfg, state).run()
        finally:
            state.active -= 1

    return await serve(handler, host, port, process_request=process_request, compression=None)


class SimServer:
    """The simulated bridge in a background thread, for tests."""

    def __init__(self, cfg: SimConfig) -> None:
        self.cfg = cfg
        self.state = SimState()
        self._loop = asyncio.new_event_loop()
        self._ready = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._server: Server | None = None
        self.port = 0

    @property
    def url(self) -> str:
        return f"ws://127.0.0.1:{self.port}{WS_PATH}"

    def _run(self) -> None:
        asyncio.set_event_loop(self._loop)
        self._server = self._loop.run_until_complete(serve_sim(self.cfg, self.state))
        self.port = self._server.sockets[0].getsockname()[1]
        self._ready.set()
        self._loop.run_forever()

    def start(self) -> SimServer:
        self._thread.start()
        self._ready.wait(5)
        return self

    def stop(self) -> None:
        async def shutdown() -> None:
            if self._server is not None:
                self._server.close()
                await self._server.wait_closed()

        asyncio.run_coroutine_threadsafe(shutdown(), self._loop).result(5)
        self._loop.call_soon_threadsafe(self._loop.stop)
        self._thread.join(5)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--style", choices=("sonde", "outpost"), default="sonde")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--imu-rate", type=float)
    parser.add_argument(
        "--capacity",
        type=float,
        default=math.inf,
        help="most synthetic samples/s delivered (outpost style)",
    )
    args = parser.parse_args(argv)
    cfg = SimConfig(style=args.style, imu_rate=args.imu_rate, capacity=args.capacity)

    async def run() -> None:
        server = await serve_sim(cfg, SimState(), args.host, args.port)
        print(f"serving {cfg.style} style on ws://{args.host}:{args.port}{WS_PATH}", flush=True)
        await server.serve_forever()

    with contextlib.suppress(KeyboardInterrupt):
        asyncio.run(run())
    return 0
