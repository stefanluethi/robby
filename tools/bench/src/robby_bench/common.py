"""Shared command line options and output files of the tools."""

from __future__ import annotations

import argparse
import datetime as dt
import json
import logging
import math
from pathlib import Path
from typing import Any

from robby_bench.metrics import COUNTER_PREFIXES, DROP_PATTERN

TOOL_VERSION = 1


def add_common_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("url", help="blip WebSocket of the outpost, e.g. ws://127.0.0.1:3000/ws/outpost")
    parser.add_argument(
        "--out",
        type=Path,
        required=True,
        help="recording to write, e.g. runs/h0-a.blip; the metrics go next to it as .json",
    )
    parser.add_argument("--label", help="what was measured, e.g. 'H0-a baseline sonde'")
    parser.add_argument(
        "--meta",
        action="append",
        default=[],
        metavar="KEY=VALUE",
        help="notes stored with the run, e.g. rssi=-58 channel=6 ap_distance=2m (repeatable)",
    )
    parser.add_argument(
        "--counter-prefix",
        action="append",
        dest="counter_prefixes",
        metavar="PREFIX",
        help=f"trace paths read as counters (default: {', '.join(COUNTER_PREFIXES)})",
    )
    parser.add_argument(
        "--drop-pattern",
        default=DROP_PATTERN,
        help=f"regex on a counter name marking it a drop counter (default: {DROP_PATTERN!r})",
    )
    parser.add_argument(
        "--connect-timeout", type=float, default=10.0, help="seconds to keep retrying the connect"
    )
    parser.add_argument("-v", "--verbose", action="store_true")


def setup_logging(verbose: bool) -> None:
    logging.basicConfig(
        level=logging.INFO if verbose else logging.WARNING, format="%(levelname)s %(message)s"
    )


def parse_meta(items: list[str]) -> dict[str, str]:
    meta: dict[str, str] = {}
    for item in items:
        key, sep, val = item.partition("=")
        if not sep or not key:
            raise argparse.ArgumentTypeError(f"--meta expects KEY=VALUE, got {item!r}")
        meta[key] = val
    return meta


def counter_prefixes(args: argparse.Namespace) -> tuple[str, ...]:
    prefixes: list[str] | None = args.counter_prefixes
    return tuple(prefixes) if prefixes else COUNTER_PREFIXES


def json_path(recording: Path) -> Path:
    return recording.with_suffix(".json")


def now_iso() -> str:
    return dt.datetime.now(dt.UTC).astimezone().isoformat(timespec="seconds")


def write_json(path: Path, data: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    # NaN/inf can't go into strict JSON
    path.write_text(json.dumps(_finite(data), indent=2) + "\n")


def _finite(x: Any) -> Any:
    if isinstance(x, float) and not math.isfinite(x):
        return None
    if isinstance(x, dict):
        return {k: _finite(v) for k, v in x.items()}
    if isinstance(x, list | tuple):
        return [_finite(v) for v in x]
    return x
