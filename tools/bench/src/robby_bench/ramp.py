"""Ramps the synthetic load with the `bench/load` action until drops start.

    uv run ramp.py ws://127.0.0.1:3000/ws/outpost --out runs/h1-b.blip

For each rate it invokes `bench/load` {rate: N}, waits `--settle` seconds, then measures for
`--step-seconds`. It stops after the first step with drops: a drop counter (`link/*`,
`bridge/*`, `outpost/*` traces whose name matches the drop pattern) went up, or the delivered
rate of the load trace was below `--min-ratio` of the requested rate. All steps go to one
connection and one `.blip` recording; the `.json` holds the metrics of each step and the knee.
"""

from __future__ import annotations

import argparse
import asyncio
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from robby_bench import common
from robby_bench.metrics import COUNTER_PREFIXES, DROP_PATTERN, Metrics
from robby_bench.recording import RecordingWriter
from robby_bench.session import ActionError, Session

DEFAULT_RATES = (1000, 2000, 5000, 10000, 20000, 40000, 80000)


@dataclass(frozen=True)
class RampConfig:
    rates: tuple[int, ...] = DEFAULT_RATES
    step_s: float = 10.0
    settle_s: float = 2.0
    action: str = "bench/load"
    arg: str = "rate"
    load_path: str = "bench/synthetic"
    min_ratio: float = 0.95
    reset: bool = True
    counter_prefixes: tuple[str, ...] = COUNTER_PREFIXES
    drop_pattern: str = DROP_PATTERN


def delivered_rate(metrics: dict[str, Any], load_path: str) -> float:
    """Samples per second of the load trace: a stream (record or per-axis) or a plain path."""
    if load_path in metrics["streams"]:
        return float(metrics["streams"][load_path]["rate_hz"])
    if load_path in metrics["paths"]:
        return float(metrics["paths"][load_path]["rate_hz"])
    return 0.0


def evaluate(
    rate: int, metrics: dict[str, Any], cfg: RampConfig, closed: str | None
) -> dict[str, Any]:
    delivered = delivered_rate(metrics, cfg.load_path)
    ratio = delivered / rate if rate > 0 else 1.0
    reasons = []
    if metrics["drops"]["total"] > 0:
        by = ", ".join(f"{n} +{d:g}" for n, d in metrics["drops"]["by_counter"].items())
        reasons.append(f"drops ({by})")
    if ratio < cfg.min_ratio:
        reasons.append(f"delivered {ratio:.1%} of the requested rate")
    if closed:
        reasons.append(f"connection closed: {closed}")
    return {
        "rate": rate,
        "delivered_rate": delivered,
        "ratio": ratio,
        "ws_bytes_per_s": metrics["ws"]["bytes_per_s"],
        "drops": metrics["drops"]["total"],
        # 0 means no drop counter was published during the step: make the steps longer
        "drop_counter_samples": max(
            (c["samples"] for c in metrics["counters"].values() if c["drop"]), default=0
        ),
        "ok": not reasons,
        "reason": "; ".join(reasons) or None,
        "metrics": metrics,
    }


async def ramp(
    url: str,
    out: Path,
    cfg: RampConfig,
    *,
    label: str | None = None,
    meta: dict[str, str] | None = None,
    connect_timeout_s: float = 10.0,
    progress: bool = False,
) -> dict[str, Any]:
    """Runs the ramp, writes `out` and its `.json`; returns the result."""
    started = common.now_iso()
    steps: list[dict[str, Any]] = []
    error: str | None = None
    with RecordingWriter(out) as recording:
        session = await Session.open(url, recording, connect_timeout_s)
        async with session:
            counters: dict[str, float] = {}
            for rate in cfg.rates:
                try:
                    session.metrics = Metrics(cfg.counter_prefixes, cfg.drop_pattern, counters)
                    await session.invoke(cfg.action, {cfg.arg: rate})
                    await session.pump(cfg.settle_s)
                    counters = session.metrics.counters_last()
                    window = session.metrics = Metrics(
                        cfg.counter_prefixes, cfg.drop_pattern, counters
                    )
                    duration = await session.pump(cfg.step_s)
                except ActionError as e:
                    error = str(e)
                    break
                counters = window.counters_last()
                step = evaluate(rate, window.result(duration), cfg, session.closed)
                steps.append(step)
                if progress:
                    print(step_line(step), flush=True)
                if not step["ok"]:
                    break
            session.metrics = None
            if cfg.reset and session.closed is None:
                try:
                    await session.invoke(cfg.action, {cfg.arg: 0})
                except ActionError as e:
                    error = error or f"reset: {e}"
    ok = [s["rate"] for s in steps if s["ok"]]
    bad = [s["rate"] for s in steps if not s["ok"]]
    result: dict[str, Any] = {
        "kind": "ramp",
        "tool_version": common.TOOL_VERSION,
        "name": out.stem,
        "label": label,
        "meta": meta or {},
        "url": url,
        "started": started,
        "recording": out.name,
        "config": {
            "rates": list(cfg.rates),
            "step_s": cfg.step_s,
            "settle_s": cfg.settle_s,
            "action": cfg.action,
            "arg": cfg.arg,
            "load_path": cfg.load_path,
            "min_ratio": cfg.min_ratio,
            "drop_pattern": cfg.drop_pattern,
        },
        "knee": {
            "last_ok": max(ok) if ok else None,
            "first_bad": bad[0] if bad else None,
            "reached_end": not bad and len(steps) == len(cfg.rates),
        },
        "error": error,
        "steps": steps,
    }
    common.write_json(common.json_path(out), result)
    return result


def step_line(step: dict[str, Any]) -> str:
    verdict = "ok" if step["ok"] else f"STOP: {step['reason']}"
    if step["drop_counter_samples"] == 0:
        verdict += " (no drop counter values in this step)"
    return (
        f"{step['rate']:>7} /s requested  {step['delivered_rate']:>9.0f} /s delivered "
        f"({step['ratio']:6.1%})  {step['ws_bytes_per_s'] / 1000:8.1f} kB/s  {verdict}"
    )


def parse_rates(text: str) -> tuple[int, ...]:
    rates = []
    for part in text.split(","):
        word = part.strip().lower()
        factor = 1000 if word.endswith("k") else 1
        rates.append(int(float(word.removesuffix("k")) * factor))
    return tuple(rates)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    common.add_common_args(parser)
    d = RampConfig()
    parser.add_argument(
        "--rates",
        type=parse_rates,
        default=d.rates,
        help="samples/s per step, e.g. 1k,2k,5k (default: 1k,2k,5k,10k,20k,40k,80k)",
    )
    parser.add_argument("--step-seconds", type=float, default=d.step_s)
    parser.add_argument("--settle", type=float, default=d.settle_s, help="seconds before a step")
    parser.add_argument("--action", default=d.action, help="action that sets the load")
    parser.add_argument("--arg", default=d.arg, help="argument of the action taking the rate")
    parser.add_argument("--load-path", default=d.load_path, help="trace of the synthetic load")
    parser.add_argument("--min-ratio", type=float, default=d.min_ratio)
    parser.add_argument("--no-reset", action="store_true", help="leave the load on at the end")
    args = parser.parse_args(argv)
    common.setup_logging(args.verbose)
    cfg = RampConfig(
        rates=args.rates,
        step_s=args.step_seconds,
        settle_s=args.settle,
        action=args.action,
        arg=args.arg,
        load_path=args.load_path,
        min_ratio=args.min_ratio,
        reset=not args.no_reset,
        counter_prefixes=common.counter_prefixes(args),
        drop_pattern=args.drop_pattern,
    )
    try:
        result = asyncio.run(
            ramp(
                args.url,
                args.out,
                cfg,
                label=args.label,
                meta=common.parse_meta(args.meta),
                connect_timeout_s=args.connect_timeout,
                progress=True,
            )
        )
    except (ConnectionError, argparse.ArgumentTypeError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 1
    knee = result["knee"]
    print(f"knee: last ok {knee['last_ok']} /s, first with drops {knee['first_bad']} /s")
    if result["error"]:
        print(f"error: {result['error']}", file=sys.stderr)
    print(f"wrote {args.out} and {common.json_path(args.out)}")
    return 1 if result["error"] else 0
