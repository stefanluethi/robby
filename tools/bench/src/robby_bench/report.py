"""Markdown tables comparing captures (baseline vs protobuf) and ramps (the knee).

uv run report.py runs/*.json                  # to stdout
uv run report.py runs/*.json --write          # to <repo>/doc/bench/results-<date>.md
uv run report.py runs/*.json --out file.md --baseline h0-a
"""

from __future__ import annotations

import argparse
import datetime as dt
import json
import sys
from collections.abc import Callable
from pathlib import Path
from typing import Any

DEFAULT_STREAM = "imu/acc"
DEFAULT_UART_COUNTER = "link/tx_bytes"
TOP_PATHS = 12

Run = dict[str, Any]


def load_runs(paths: list[Path]) -> list[Run]:
    runs = []
    for path in paths:
        data = json.loads(path.read_text())
        if data.get("kind") not in ("capture", "ramp"):
            print(f"skipping {path}: not a capture or ramp result", file=sys.stderr)
            continue
        runs.append(data)
    return runs


def _num(x: float | None, digits: int = 1) -> str:
    if x is None:
        return "–"
    return f"{x:,.{digits}f}"


def _table(header: list[str], rows: list[list[str]], align: str | None = None) -> list[str]:
    align = align or "l" + "r" * (len(header) - 1)
    sep = ["---:" if a == "r" else "---" for a in align]
    return [
        "| " + " | ".join(header) + " |",
        "| " + " | ".join(sep) + " |",
        *("| " + " | ".join(row) + " |" for row in rows),
    ]


def _stream(run: Run, name: str) -> dict[str, Any] | None:
    s: dict[str, Any] | None = run.get("streams", {}).get(name)
    return s


def uart_bytes_per_s(run: Run, counter: str) -> tuple[float | None, str]:
    """UART payload bytes/s from the link counter; for the baseline (no counters) the WebSocket
    payload, as today's bridge forwards each UART frame 1:1."""
    c = run.get("counters", {}).get(counter)
    if c is not None and c["per_s"] is not None:
        return c["per_s"], "counter"
    if not run.get("counters"):
        return run["ws"]["bytes_per_s"], "WS"
    return None, ""


def _comparison(captures: list[Run], stream: str, uart_counter: str) -> list[str]:
    base = captures[0]

    def row(label: str, get: Callable[[Run], float | None], digits: int = 1) -> list[str]:
        cells = [label]
        b = get(base)
        for run in captures:
            x = get(run)
            cell = _num(x, digits)
            if run is not base and x is not None and b and round(b, digits):
                cell += f" ({x / b:.2f}×)"
            cells.append(cell)
        return cells

    def s(key: str) -> Callable[[Run], float | None]:
        def get(r: Run) -> float | None:
            st = _stream(r, stream)
            return st[key] if st else None

        return get

    def interval(key: str) -> Callable[[Run], float | None]:
        def get(r: Run) -> float | None:
            st = _stream(r, stream)
            iv = st["interval_ms"] if st else None
            return iv[key] if iv else None

        return get

    def uart(r: Run) -> float | None:
        x = uart_bytes_per_s(r, uart_counter)[0]
        return x / 1000 if x is not None else None

    def uart_per_sample(r: Run) -> float | None:
        x = uart_bytes_per_s(r, uart_counter)[0]
        st = _stream(r, stream)
        return x / st["rate_hz"] if x is not None and st and st["rate_hz"] else None

    rows = [
        row("WS payload kB/s", lambda r: r["ws"]["bytes_per_s"] / 1000),
        row("WS frames/s", lambda r: r["ws"]["frames_per_s"], 0),
        row("messages/s", lambda r: r["messages"]["per_s"], 0),
        row(f"`{stream}` samples/s", s("rate_hz")),
        row(f"`{stream}` WS bytes/sample", s("bytes_per_sample")),
        row(f"`{stream}` interval mean ms", interval("mean"), 3),
        row(f"`{stream}` interval stddev ms", interval("stddev"), 3),
        row(f"`{stream}` interval max ms", interval("max"), 3),
        row(f"`{stream}` gaps", s("gaps"), 0),
        row("UART kB/s", uart),
        row(f"UART bytes/`{stream}` sample", uart_per_sample),
        row("drops (counters)", lambda r: r["drops"]["total"], 0),
        row("decode errors", lambda r: r["messages"]["decode_errors"], 0),
    ]
    sources = ", ".join(
        f"{r['name']}: {uart_bytes_per_s(r, uart_counter)[1] or 'n/a'}" for r in captures
    )
    return [
        f"Baseline: **{base['name']}**, ratios relative to it. "
        f"`{stream}` styles: "
        + ", ".join(
            f"{r['name']} {st['style']}" if (st := _stream(r, stream)) else f"{r['name']} absent"
            for r in captures
        )
        + f". UART bytes from: {sources} (WS = WebSocket payload, the baseline bridge forwards "
        "UART frames 1:1).",
        "",
        *_table(["metric", *(r["name"] for r in captures)], rows),
    ]


def _paths(run: Run, top: int) -> list[str]:
    paths = sorted(run["paths"].items(), key=lambda kv: kv[1]["bytes"], reverse=True)
    rows = []
    for path, p in paths[:top]:
        iv = p["interval_ms"] or {}
        rows.append(
            [
                f"`{path}`",
                _num(p["rate_hz"]),
                _num(p["bytes_per_msg"]),
                f"{p['bytes_share']:.1%}",
                _num(iv.get("mean"), 3),
                _num(iv.get("stddev"), 3),
                _num(iv.get("max"), 3),
                str(p["gaps"]["count"]),
            ]
        )
    lines = _table(
        ["path", "msgs/s", "B/msg", "bytes", "mean ms", "stddev ms", "max ms", "gaps"], rows
    )
    if len(paths) > top:
        lines.append(f"\n{len(paths) - top} more paths.")
    return lines


def _counters(run: Run) -> list[str]:
    rows = [
        [
            f"`{n}`" + (" (drop)" if c["drop"] else ""),
            f"{c['last']:g}",
            f"{c['delta']:g}",
            _num(c["per_s"]),
        ]
        for n, c in run["counters"].items()
    ]
    return _table(["counter", "last", "delta", "per s"], rows)


def _ramp(run: Run) -> list[str]:
    knee = run["knee"]
    cfg = run["config"]
    if knee["first_bad"] is not None:
        verdict = (
            f"knee between **{_num(knee['last_ok'], 0)}** and **{knee['first_bad']:,}** samples/s"
        )
    elif knee["reached_end"]:
        verdict = f"no drops up to {max(cfg['rates']):,} samples/s"
    else:
        verdict = "ended without a result"
    lines = [
        f"{verdict} (load trace `{cfg['load_path']}`, {cfg['step_s']:g} s per step, "
        f"min ratio {cfg['min_ratio']:.0%}).",
    ]
    if run.get("error"):
        lines.append(f"\nError: {run['error']}")
    rows = [
        [
            f"{s['rate']:,}",
            _num(s["delivered_rate"], 0),
            f"{s['ratio']:.1%}",
            _num(s["ws_bytes_per_s"] / 1000),
            f"{s['drops']:g}",
            "ok" if s["ok"] else s["reason"],
        ]
        for s in run["steps"]
    ]
    lines += [
        "",
        *_table(
            ["requested /s", "delivered /s", "ratio", "WS kB/s", "drops", "result"], rows, "rrrrrl"
        ),
    ]
    return lines


def render(
    runs: list[Run],
    baseline: str | None = None,
    stream: str = DEFAULT_STREAM,
    uart_counter: str = DEFAULT_UART_COUNTER,
    title: str | None = None,
) -> str:
    captures = [r for r in runs if r["kind"] == "capture"]
    ramps = [r for r in runs if r["kind"] == "ramp"]
    if baseline is not None:
        match = [r for r in captures if r["name"] == baseline]
        if not match:
            raise ValueError(f"no capture named {baseline!r}")
        captures = match + [r for r in captures if r is not match[0]]
    out = [f"# {title or 'Bench results ' + dt.date.today().isoformat()}", ""]
    out += ["## Runs", ""]
    out += _table(
        ["run", "kind", "label", "started", "duration s", "notes"],
        [
            [
                r["name"],
                r["kind"],
                r.get("label") or "",
                r.get("started") or "",
                _num(r.get("duration_s")) if r["kind"] == "capture" else "",
                ", ".join(f"{k}={v}" for k, v in r.get("meta", {}).items()),
            ]
            for r in runs
        ],
        "lllll" + "l",
    )
    if captures:
        out += ["", "## Comparison", ""]
        out += _comparison(captures, stream, uart_counter)
    for r in captures:
        out += ["", f"## {r['name']}: traces", ""]
        out += _paths(r, TOP_PATHS)
        if r["counters"]:
            out += ["", *_counters(r)]
    for r in ramps:
        out += ["", f"## {r['name']}: ramp", ""]
        out += _ramp(r)
    return "\n".join(out) + "\n"


def repo_root(start: Path) -> Path:
    for d in (start, *start.parents):
        if (d / ".git").exists():
            return d
    return start


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("runs", nargs="+", type=Path, help="run results (.json)")
    parser.add_argument("--baseline", help="name of the capture the others compare to")
    parser.add_argument("--stream", default=DEFAULT_STREAM, help="sample stream to compare")
    parser.add_argument(
        "--uart-counter", default=DEFAULT_UART_COUNTER, help="counter of the UART bytes sent"
    )
    parser.add_argument("--title")
    group = parser.add_mutually_exclusive_group()
    group.add_argument("--out", type=Path, help="markdown file to write")
    group.add_argument(
        "--write", action="store_true", help="write <repo>/doc/bench/results-<date>.md"
    )
    args = parser.parse_args(argv)
    runs = load_runs(args.runs)
    if not runs:
        print("error: no runs", file=sys.stderr)
        return 1
    try:
        text = render(runs, args.baseline, args.stream, args.uart_counter, args.title)
    except ValueError as error:
        print(f"error: {error}", file=sys.stderr)
        return 1
    out: Path | None = args.out
    if args.write:
        out = repo_root(Path.cwd()) / "doc" / "bench" / f"results-{dt.date.today()}.md"
    if out is None:
        print(text, end="")
    else:
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(text)
        print(f"wrote {out}")
    return 0
