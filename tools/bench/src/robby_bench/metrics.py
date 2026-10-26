"""Metrics of a captured blip stream.

Fed frame by frame (or message by message, e.g. from a recording), `Metrics.result()` gives a
JSON-ready dict:

- `ws`: WebSocket payload bytes and frames per second, frame sizes
- `messages`: messages per second, by type, decode errors
- `paths`: per trace path its rate, bytes share and the sample interval statistics taken from
  the message timestamps, with gaps (interval > 3x the median)
- `streams`: samples per second of the record traces (e.g. `imu/acc` {x, y, z}) and of
  per-axis groups (`imu/acc/x`, `/y`, `/z` counted as one `imu/acc` sample), so both
  firmwares compare by sample
- `counters`: first/last/delta of the `link/*` and `outpost/*` counter traces (records are
  flattened as `path.field`), and `drops`, the sum of the deltas of the drop counters
"""

from __future__ import annotations

import itertools
import re
import statistics
from collections import Counter
from dataclasses import dataclass, field
from typing import Any

from robby_bench import blip

COUNTER_PREFIXES = ("link/", "outpost/")
DROP_PATTERN = r"drop|gap|lost|overrun|overflow|error"
GAP_FACTOR = 3.0
MAX_LISTED_GAPS = 20
AXES = ("x", "y", "z")
NS_PER_MS = 1e6


@dataclass
class _PathStats:
    count: int = 0
    bytes: int = 0
    scalars: int = 0
    kinds: Counter[str] = field(default_factory=Counter)
    timestamps: list[int] = field(default_factory=list)


@dataclass
class _CounterStats:
    first: float
    last: float
    t_first: int | None
    t_last: int | None
    delta: float = 0.0
    """Increase since the window start (or the counter's start value), resets included."""
    window_delta: float = 0.0
    """Increase between the first and the last sample in the window, for the rate."""
    samples: int = 1
    resets: int = 0

    def add(self, x: float, t: int | None, in_window: bool) -> None:
        # a decrease means the counter restarted, e.g. after a reboot: it counted x since
        step = x - self.last if x >= self.last else x
        self.resets += x < self.last
        self.delta += step
        if in_window:
            self.window_delta += step
            self.t_last = t
        self.last = x
        self.samples += 1

    def per_s(self) -> float | None:
        if self.t_first is None or self.t_last is None or self.t_last <= self.t_first:
            return None
        return self.window_delta / ((self.t_last - self.t_first) / 1e9)


class Metrics:
    """Accumulates the metrics of a stream of frames.

    `counter_start` holds the counter values seen before this window (e.g. in the previous ramp
    step), so the deltas of a window also count the increments up to its first value.
    """

    def __init__(
        self,
        counter_prefixes: tuple[str, ...] = COUNTER_PREFIXES,
        drop_pattern: str = DROP_PATTERN,
        counter_start: dict[str, float] | None = None,
    ) -> None:
        self.counter_prefixes = counter_prefixes
        self.drop_regex = re.compile(drop_pattern, re.IGNORECASE)
        self.counter_start = dict(counter_start or {})
        self.frames = 0
        self.frame_bytes = 0
        self.frame_bytes_max = 0
        self.text_frames = 0
        self.decode_errors: list[str] = []
        self.messages = 0
        self.message_bytes = 0
        self.types: Counter[str] = Counter()
        self.type_bytes: Counter[str] = Counter()
        self.log_levels: Counter[int] = Counter()
        self.paths: dict[str, _PathStats] = {}
        self.counters: dict[str, _CounterStats] = {}

    # --- input ---

    def add_frame(self, data: bytes) -> list[blip.Item]:
        """Adds a binary WebSocket frame; returns its messages."""
        self.frames += 1
        self.frame_bytes += len(data)
        self.frame_bytes_max = max(self.frame_bytes_max, len(data))
        items, error = blip.decode_sequence(data)
        if error is not None:
            self.decode_errors.append(error)
        for item in items:
            self.add_message(item.obj, item.size)
        return items

    def add_text_frame(self) -> None:
        self.text_frames += 1

    def add_message(self, obj: Any, size: int) -> None:
        kind = blip.message_type(obj)
        self.messages += 1
        self.message_bytes += size
        self.types[kind] += 1
        self.type_bytes[kind] += size
        if kind == "Value":
            self._add_value(obj["Value"], size)
        elif kind == "Log":
            level = obj["Log"].get("level")
            if isinstance(level, int):
                self.log_levels[level] += 1

    def _add_value(self, body: Any, size: int) -> None:
        if not isinstance(body, dict) or not isinstance(body.get("path"), str):
            self.types["invalid Value"] += 1
            return
        path: str = body["path"]
        data = body.get("value")
        stats = self.paths.setdefault(path, _PathStats())
        stats.count += 1
        stats.bytes += size
        stats.scalars += blip.scalar_count(data)
        stats.kinds[blip.value_kind(data)] += 1
        timestamp = body.get("timestamp")
        if isinstance(timestamp, int):
            stats.timestamps.append(timestamp)
        if path.startswith(self.counter_prefixes):
            t = timestamp if isinstance(timestamp, int) else None
            for leaf, x in blip.numeric_leaves(data).items():
                self._add_counter(f"{path}.{leaf}" if leaf else path, x, t)

    def _add_counter(self, name: str, x: float, t: int | None) -> None:
        stats = self.counters.get(name)
        if stats is not None:
            stats.add(x, t, in_window=True)
            return
        start = self.counter_start.get(name)
        stats = self.counters[name] = _CounterStats(
            first=x, last=x if start is None else start, t_first=t, t_last=t
        )
        if start is not None:
            stats.samples = 0
            stats.add(x, t, in_window=False)

    def counters_last(self) -> dict[str, float]:
        """The last value of each counter, as `counter_start` of the next window."""
        last = dict(self.counter_start)
        last.update({name: c.last for name, c in self.counters.items()})
        return last

    def is_drop_counter(self, name: str) -> bool:
        leaf = name.rsplit("/", 1)[-1]
        return self.drop_regex.search(leaf) is not None

    # --- output ---

    def result(self, duration_s: float) -> dict[str, Any]:
        d = duration_s if duration_s > 0 else float("nan")
        paths = {p: _path_result(s, d, self.message_bytes) for p, s in sorted(self.paths.items())}
        counters: dict[str, dict[str, Any]] = {
            name: {
                "first": c.first,
                "last": c.last,
                "delta": c.delta,
                "per_s": c.per_s(),
                "samples": c.samples,
                "resets": c.resets,
                "drop": self.is_drop_counter(name),
            }
            for name, c in sorted(self.counters.items())
        }
        drop_deltas = {n: c["delta"] for n, c in counters.items() if c["drop"] and c["delta"] > 0}
        return {
            "duration_s": duration_s,
            "ws": {
                "frames": self.frames,
                "frames_per_s": self.frames / d,
                "bytes": self.frame_bytes,
                "bytes_per_s": self.frame_bytes / d,
                "frame_bytes_mean": self.frame_bytes / self.frames if self.frames else 0.0,
                "frame_bytes_max": self.frame_bytes_max,
                "text_frames": self.text_frames,
            },
            "messages": {
                "count": self.messages,
                "per_s": self.messages / d,
                "bytes": self.message_bytes,
                "per_frame_mean": self.messages / self.frames if self.frames else 0.0,
                "by_type": dict(sorted(self.types.items())),
                "bytes_by_type": dict(sorted(self.type_bytes.items())),
                "decode_errors": len(self.decode_errors),
                "decode_error_examples": self.decode_errors[:5],
                "log_levels": {str(k): v for k, v in sorted(self.log_levels.items())},
            },
            "paths": paths,
            "streams": _streams(paths, self.paths, d, self.counter_prefixes),
            "counters": counters,
            "drops": {"total": sum(drop_deltas.values()), "by_counter": drop_deltas},
        }


def interval_stats(timestamps: list[int]) -> dict[str, Any]:
    """Statistics of the intervals between consecutive timestamps (ns), in ms.

    Intervals <= 0 (repeated or out-of-order timestamps) are counted, but left out of the
    statistics. A gap is an interval longer than 3x the median.
    """
    diffs = [b - a for a, b in itertools.pairwise(timestamps)]
    positive = [x for x in diffs if x > 0]
    out: dict[str, Any] = {"non_increasing": len(diffs) - len(positive)}
    if not positive:
        out["interval_ms"] = None
        out["gaps"] = {"count": 0, "max_ms": 0.0, "total_ms": 0.0, "list": []}
        return out
    median = statistics.median(positive)
    out["interval_ms"] = {
        "mean": statistics.fmean(positive) / NS_PER_MS,
        "stddev": statistics.pstdev(positive) / NS_PER_MS,
        "median": median / NS_PER_MS,
        "min": min(positive) / NS_PER_MS,
        "max": max(positive) / NS_PER_MS,
    }
    gaps = [
        (timestamps[i], diff)
        for i, diff in enumerate(diffs)
        if diff > 0 and diff > GAP_FACTOR * median
    ]
    out["gaps"] = {
        "count": len(gaps),
        "max_ms": max((g for _, g in gaps), default=0) / NS_PER_MS,
        "total_ms": sum(g for _, g in gaps) / NS_PER_MS,
        "list": [{"after_ns": t, "length_ms": g / NS_PER_MS} for t, g in gaps[:MAX_LISTED_GAPS]],
    }
    return out


def _path_result(s: _PathStats, duration_s: float, total_bytes: int) -> dict[str, Any]:
    ts = s.timestamps
    span_ns = ts[-1] - ts[0] if len(ts) > 1 else 0
    return {
        "count": s.count,
        "rate_hz": s.count / duration_s,
        "timestamp_rate_hz": (len(ts) - 1) / (span_ns / 1e9) if span_ns > 0 else None,
        "bytes": s.bytes,
        "bytes_per_s": s.bytes / duration_s,
        "bytes_per_msg": s.bytes / s.count if s.count else 0.0,
        "bytes_share": s.bytes / total_bytes if total_bytes else 0.0,
        "kind": s.kinds.most_common(1)[0][0] if s.kinds else None,
        "scalars_per_msg": s.scalars / s.count if s.count else 0.0,
        **interval_stats(ts),
    }


def _streams(
    paths: dict[str, dict[str, Any]],
    raw: dict[str, _PathStats],
    duration_s: float,
    counter_prefixes: tuple[str, ...],
) -> dict[str, dict[str, Any]]:
    """Samples of the record traces and of the per-axis groups P/x, P/y, P/z (not counters)."""
    streams: dict[str, dict[str, Any]] = {}
    for path, p in paths.items():
        if p["kind"] == "record" and not path.startswith(counter_prefixes):
            streams[path] = {
                "style": "record",
                "paths": [path],
                "samples": p["count"],
                "rate_hz": p["rate_hz"],
                "bytes_per_s": p["bytes_per_s"],
                "bytes_per_sample": p["bytes_per_msg"],
                "interval_ms": p["interval_ms"],
                "gaps": p["gaps"]["count"],
            }
    parents = {path.rsplit("/", 1)[0] for path in paths if "/" in path}
    for parent in sorted(parents):
        members = [f"{parent}/{axis}" for axis in AXES]
        if parent in streams or not all(m in paths for m in members):
            continue
        samples = min(raw[m].count for m in members)
        total = sum(raw[m].bytes for m in members)
        first = paths[members[0]]
        streams[parent] = {
            "style": "per-axis",
            "paths": members,
            "samples": samples,
            "rate_hz": samples / duration_s,
            "bytes_per_s": total / duration_s,
            "bytes_per_sample": total / samples if samples else 0.0,
            "interval_ms": first["interval_ms"],
            "gaps": max(paths[m]["gaps"]["count"] for m in members),
        }
    return dict(sorted(streams.items()))
