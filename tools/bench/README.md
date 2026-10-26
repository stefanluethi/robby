# robby bench tools

Measure the trace link of robby (STM32 → UART → ESP32 → WiFi → PC) from the PC side, over the
same blip WebSocket that blip uses. See `doc/plan/protobuf-tracing.md` §5 for what is measured
and why, and `.claude/skills/bench-robby/SKILL.md` for the hardware campaign.

The bridge serves **one client at a time**: close blip before running a tool, and wait ~3 s
between two runs (the tools retry the connect for `--connect-timeout` seconds).

```bash
cd tools/bench
uv run capture.py ws://192.168.1.70/ws/robby --seconds 120 --out runs/h0-a.blip --label "H0-a baseline" --meta rssi=-58
uv run ramp.py ws://192.168.1.70/ws/robby --out runs/h1-b.blip --rates 1k,2k,5k,10k,20k,40k,80k
uv run report.py runs/*.json --baseline h0-a --write
uv run decode.py runs/h0-a.blip --messages 10
```

## Tools

| script | does |
|---|---|
| `capture.py URL --seconds N --out runs/NAME.blip` | records every binary frame to a blip recording (opens in blip) and writes `runs/NAME.json` with the metrics |
| `ramp.py URL --out runs/NAME.blip` | invokes `bench/load` {rate: N} step by step on one connection, measures each step, stops after the first step with drops; one recording, `.json` with all steps and the knee |
| `report.py runs/*.json` | markdown: runs, baseline-vs-others comparison, traces and counters per capture, ramp tables; `--write` puts it in `doc/bench/results-<date>.md` |
| `decode.py FILE.blip` | summary of a recording (types, traces, rates from the timestamps, counters); `--messages N`, `--json` |
| `simulate.py --style sonde\|outpost` | a simulated bridge on `ws://127.0.0.1:8765/ws/robby` to try the tools without hardware |

## Metrics (`.json`)

- `ws`: payload bytes/s, frames/s, frame sizes. On today's bridge each WebSocket frame is one
  UART frame, so these are the UART payload bytes too.
- `messages`: messages/s, by type, decode errors.
- `paths`: per trace path its rate, bytes share, kind, and the sample interval
  mean/stddev/median/max from the message timestamps, gaps (interval > 3× median).
- `streams`: samples/s and bytes/sample of record traces (`imu/acc` {x, y, z}) and of
  per-axis groups (`imu/acc/x`, `/y`, `/z` count as one `imu/acc` sample), so the sonde
  baseline and the outpost compare per sample.
- `counters`: first/last/delta/per s of the `link/*` and `outpost/*` traces (record fields
  flattened as `path.field`); `drops`: sum of the deltas of the drop counters.

## Counter convention (for the firmware)

What the tools expect from the `link/*` (STM32) and `outpost/*` (ESP32) traces:

- **Cumulative** counters, published about once per second. A decrease is read as a restart
  (reboot) and counted from 0.
- A counter whose last path segment (or record field) matches `drop|gap|lost|overrun|overflow|error`
  (`--drop-pattern`) is a drop counter, e.g. `outpost/seq_gaps`, `outpost/queue_drops`,
  `outpost/crc_errors`, `link/drops`. Any increase during a ramp step ends the ramp.
- Gauges (e.g. `outpost/rssi`, `outpost/queue_high_water`) may sit beside them; only their last
  value is used. Avoid the drop words in their names.
- `report.py` takes the UART bytes from `link/tx_bytes` (`--uart-counter`), else, for a run
  without counters (the baseline), the WebSocket payload.
- The synthetic load of `bench/load` is expected on `bench/synthetic` (`--load-path`), one
  message per sample (a record, or per-axis `/x`, `/y`, `/z`).

A ramp step must be longer than the counter period (default 10 s steps, 2 s settle); a step
with `drop_counter_samples: 0` saw no counter value.

## Development

```bash
uv run pytest       # against the simulated bridge, ~20 s
uv run ruff check . && uv run ruff format --check . && uv run mypy
```
