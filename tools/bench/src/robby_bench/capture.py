"""Records the blip stream for N seconds to a `.blip` file and writes its metrics as `.json`.

uv run capture.py ws://192.168.1.70/ws/robby --seconds 120 --out runs/h0-a.blip
"""

from __future__ import annotations

import argparse
import asyncio
import contextlib
import sys
from pathlib import Path
from typing import Any

from robby_bench import common
from robby_bench.metrics import COUNTER_PREFIXES, DROP_PATTERN, Metrics
from robby_bench.recording import RecordingWriter
from robby_bench.session import Session


async def capture(
    url: str,
    seconds: float,
    out: Path,
    *,
    label: str | None = None,
    meta: dict[str, str] | None = None,
    counter_prefixes: tuple[str, ...] = COUNTER_PREFIXES,
    drop_pattern: str = DROP_PATTERN,
    connect_timeout_s: float = 10.0,
) -> dict[str, Any]:
    """Captures and writes `out` and its `.json`; returns the metrics."""
    started = common.now_iso()
    with RecordingWriter(out) as recording:
        session = await Session.open(url, recording, connect_timeout_s)
        async with session:
            metrics = session.metrics = Metrics(counter_prefixes, drop_pattern)
            loop = asyncio.get_running_loop()
            start = loop.time()
            with contextlib.suppress(asyncio.CancelledError):  # Ctrl-C: keep the recording
                await session.pump(seconds)
            duration = loop.time() - start
    result: dict[str, Any] = {
        "kind": "capture",
        "tool_version": common.TOOL_VERSION,
        "name": out.stem,
        "label": label,
        "meta": meta or {},
        "url": url,
        "started": started,
        "requested_s": seconds,
        "recording": out.name,
        "closed_early": session.closed,
        **metrics.result(duration),
    }
    common.write_json(common.json_path(out), result)
    return result


def summary(result: dict[str, Any]) -> str:
    ws = result["ws"]
    lines = [
        f"{result['name']}: {result['duration_s']:.1f} s, "
        f"{ws['bytes_per_s'] / 1000:.1f} kB/s, {ws['frames_per_s']:.0f} frames/s, "
        f"{result['messages']['per_s']:.0f} msgs/s, "
        f"{result['messages']['decode_errors']} decode errors, drops {result['drops']['total']:g}",
    ]
    for name, s in result["streams"].items():
        lines.append(
            f"  {name} ({s['style']}): {s['rate_hz']:.1f} samples/s, "
            f"{s['bytes_per_sample']:.1f} B/sample, {s['gaps']} gaps"
        )
    if result["closed_early"]:
        lines.append(f"  connection closed early: {result['closed_early']}")
    return "\n".join(lines)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    common.add_common_args(parser)
    parser.add_argument("--seconds", type=float, default=60.0, help="capture duration")
    args = parser.parse_args(argv)
    common.setup_logging(args.verbose)
    try:
        result = asyncio.run(
            capture(
                args.url,
                args.seconds,
                args.out,
                label=args.label,
                meta=common.parse_meta(args.meta),
                counter_prefixes=common.counter_prefixes(args),
                drop_pattern=args.drop_pattern,
                connect_timeout_s=args.connect_timeout,
            )
        )
    except (ConnectionError, argparse.ArgumentTypeError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 1
    print(summary(result))
    print(f"wrote {args.out} and {common.json_path(args.out)}")
    return 0
