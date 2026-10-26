"""Prints a summary of a `.blip` recording: header, message types, traces, counters.

    uv run decode.py runs/h0-a.blip [--messages 20] [--json]

A recording doesn't keep the frame boundaries or arrival times, so the rates come from the
span of the message timestamps.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

from robby_bench import blip
from robby_bench.metrics import Metrics
from robby_bench.recording import read_recording


def recording_metrics(path: Path) -> dict[str, Any]:
    rec = read_recording(path)
    metrics = Metrics()
    connections: list[str] = []
    t_min: int | None = None
    t_max: int | None = None
    for item in rec.items:
        if isinstance(item.obj, str):  # version 2: name of the connection of what follows
            connections.append(item.obj)
            continue
        metrics.add_message(item.obj, item.size)
        if blip.message_type(item.obj) in ("Value", "Log"):
            t = next(iter(item.obj.values())).get("timestamp")
            if isinstance(t, int):
                t_min = t if t_min is None else min(t_min, t)
                t_max = t if t_max is None else max(t_max, t)
    span = (t_max - t_min) / 1e9 if t_min is not None and t_max is not None else 0.0
    result = metrics.result(span)
    del result["ws"]  # frame boundaries aren't stored
    return {
        "file": str(path),
        "header": rec.header,
        "compressed": rec.compressed,
        "error": rec.error,
        "connections": connections,
        "timestamp_span_s": span,
        **result,
    }


def format_summary(m: dict[str, Any]) -> str:
    msgs = m["messages"]
    lines = [
        f"{m['file']}: {m['header']}, {'gzip' if m['compressed'] else 'uncompressed'}",
        f"  {msgs['count']} messages, {msgs['bytes']} bytes, "
        f"timestamps span {m['timestamp_span_s']:.3f} s",
        "  types: " + ", ".join(f"{k} {v}" for k, v in msgs["by_type"].items()),
    ]
    if m["error"]:
        lines.append(f"  read up to: {m['error']}")
    if m["connections"]:
        lines.append(f"  connections: {', '.join(m['connections'])}")
    lines.append("")
    lines.append(
        f"  {'path':<28} {'count':>7} {'rate/s':>9} {'B/msg':>6} {'share':>6} "
        f"{'kind':<7} {'mean ms':>8} {'std ms':>7} {'max ms':>8} {'gaps':>5}"
    )
    for path, p in m["paths"].items():
        iv = p["interval_ms"] or {}
        rate = p["timestamp_rate_hz"]
        lines.append(
            f"  {path:<28} {p['count']:>7} {rate or 0:>9.1f} {p['bytes_per_msg']:>6.1f} "
            f"{p['bytes_share']:>6.1%} {p['kind'] or '':<7} {iv.get('mean', 0):>8.3f} "
            f"{iv.get('stddev', 0):>7.3f} {iv.get('max', 0):>8.3f} {p['gaps']['count']:>5}"
        )
    if m["streams"]:
        lines.append("")
        for name, s in m["streams"].items():
            lines.append(
                f"  stream {name} ({s['style']}): {s['samples']} samples, "
                f"{s['bytes_per_sample']:.1f} B/sample"
            )
    if m["counters"]:
        lines.append("")
        for name, c in m["counters"].items():
            mark = " (drop)" if c["drop"] else ""
            lines.append(f"  counter {name}{mark}: last {c['last']:g}, delta {c['delta']:g}")
    return "\n".join(lines)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("file", type=Path)
    parser.add_argument("--messages", type=int, default=0, help="also print the first N messages")
    parser.add_argument("--json", action="store_true", help="print the metrics as JSON")
    args = parser.parse_args(argv)
    try:
        m = recording_metrics(args.file)
    except (OSError, ValueError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 1
    if args.json:
        print(json.dumps(m, indent=2, default=str))
    else:
        print(format_summary(m))
    if args.messages:
        print()
        for item in read_recording(args.file).items[: args.messages]:
            print(f"  [{item.size:>4} B] {item.obj!r}")
    return 0
