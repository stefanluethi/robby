"""One WebSocket connection to the blip target, recorded to a `.blip` file.

The robby bridge serves one client at a time and refuses a new connection for ~3 s after the
last one ended, so the tools keep a single connection open for a whole run and retry the
connect for a while.
"""

from __future__ import annotations

import asyncio
import logging
import time
from collections.abc import Callable
from types import TracebackType
from typing import Self

from websockets.asyncio.client import ClientConnection, connect
from websockets.exceptions import ConnectionClosed, InvalidHandshake, InvalidStatus

from robby_bench import blip
from robby_bench.metrics import Metrics
from robby_bench.recording import RecordingWriter

log = logging.getLogger(__name__)

FLUSH_INTERVAL_S = 1.0

MessageHook = Callable[[list[blip.Item]], None]


class ActionError(RuntimeError):
    pass


class Session:
    """Receives frames, writes them all to the recording and feeds the current metrics window."""

    def __init__(self, ws: ClientConnection, recording: RecordingWriter) -> None:
        self.ws = ws
        self.recording = recording
        self.metrics: Metrics | None = None
        self.closed: str | None = None
        self._next_id = 1
        self._last_flush = time.monotonic()
        self._results: dict[int, dict[str, object]] = {}

    @classmethod
    async def open(
        cls, url: str, recording: RecordingWriter, connect_timeout_s: float = 10.0
    ) -> Self:
        deadline = time.monotonic() + connect_timeout_s
        while True:
            try:
                ws = await connect(
                    url,
                    max_size=None,
                    compression=None,
                    ping_interval=None,
                    open_timeout=max(1.0, deadline - time.monotonic()),
                )
            except (OSError, InvalidHandshake, InvalidStatus, TimeoutError) as error:
                if time.monotonic() + 1.0 > deadline:
                    raise ConnectionError(f"can't connect to {url}: {error}") from error
                log.info("connect failed (%s), retrying", error)
                await asyncio.sleep(1.0)
                continue
            return cls(ws, recording)

    async def close(self) -> None:
        await self.ws.close()
        self.recording.flush()

    async def __aenter__(self) -> Self:
        return self

    async def __aexit__(
        self,
        exc_type: type[BaseException] | None,
        exc: BaseException | None,
        tb: TracebackType | None,
    ) -> None:
        await self.close()

    async def pump(self, duration_s: float, until: Callable[[], bool] | None = None) -> float:
        """Receives for `duration_s` or until `until()` is true; returns the time it took."""
        loop = asyncio.get_running_loop()
        start = loop.time()
        end = start + duration_s
        while self.closed is None and not (until and until()):
            remaining = end - loop.time()
            if remaining <= 0:
                break
            try:
                async with asyncio.timeout(remaining):
                    frame = await self.ws.recv()
            except TimeoutError:
                break
            except ConnectionClosed as error:
                self.closed = str(error)
                break
            self._handle(frame)
        return loop.time() - start

    def _handle(self, frame: str | bytes) -> None:
        if isinstance(frame, str):
            if self.metrics:
                self.metrics.add_text_frame()
            return
        self.recording.write(frame)
        if self.metrics:
            items = self.metrics.add_frame(frame)
        else:
            items, _ = blip.decode_sequence(frame)
        for item in items:
            if blip.message_type(item.obj) == "ActionResult":
                body = item.obj["ActionResult"]
                if isinstance(body, dict) and isinstance(body.get("id"), int):
                    self._results[body["id"]] = body
        now = time.monotonic()
        if now - self._last_flush > FLUSH_INTERVAL_S:
            self.recording.flush()
            self._last_flush = now

    async def invoke(self, path: str, args: dict[str, object], timeout_s: float = 5.0) -> None:
        """Invokes an action and waits for its result, recording what arrives meanwhile."""
        request_id = self._next_id
        self._next_id += 1
        await self.ws.send(blip.encode(blip.invoke(request_id, path, args)))
        await self.pump(timeout_s, until=lambda: request_id in self._results)
        body = self._results.pop(request_id, None)
        if body is None:
            raise ActionError(
                f"no ActionResult for {path} {args} within {timeout_s} s"
                + (f" (connection closed: {self.closed})" if self.closed else "")
            )
        result = body.get("result")
        if isinstance(result, dict) and "Err" in result:
            raise ActionError(f"{path} {args} rejected: {result['Err']}")
