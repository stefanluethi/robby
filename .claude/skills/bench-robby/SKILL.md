---
name: bench-robby
description: Runbook of the robby throughput campaign on the hardware (plan doc/plan/protobuf-tracing.md §7) - flash the baseline (sonde) and protobuf firmwares of app_core and app_comm, capture and ramp the blip stream with tools/bench, write doc/bench/results-<date>.md. Use when the user says the hardware is ready, asks to run the campaign / baseline / H0 / H1, or invokes /bench-robby.
---

# robby bench campaign

Measures the trace link STM32 (`app_core`) → USART2 → ESP32-C6 (`app_comm`) → WiFi → PC with
the tools in `tools/bench/` (see its README). You can run all of it yourself; ask the user only
for what needs hands (cables, power, moving the robot to another distance from the AP).

## Hardware and access

- STM32F723: ST-Link V2-1 `0483:374b`, flashed with probe-rs (openocd on this host is broken).
- ESP32-C6 bridge: `/dev/ttyACM0`, flashed with espflash.
- Bridge WebSocket: `ws://192.168.1.70/ws/robby` (DHCP; if it's not reachable, ask the user for
  the address or check the router; don't open the serial console just to look, see below).
  Since the bridge only forwards protobuf, the tools connect to the outpost on the PC instead:
  start it in the background with `cd tools/outpost && uv run outpost.py ws://192.168.1.70/ws/robby`
  (it serves `ws://127.0.0.1:3000/ws/outpost`). The baseline (sonde) firmware still speaks
  blip CBOR itself: point the tools at the bridge directly for H0.
- WiFi credentials: `app_comm/wifi.env` in the main checkout (gitignored, written by the user).
  Source it into the environment of the cargo build only. **Never print, echo, cat or commit
  it**, and don't pass its values on a command line you show.

## Gotchas

- The bridge serves **one client at a time** (the outpost, or a tool on the baseline). The
  outpost serves any number of clients.
- Wait **~3 s** between two WebSocket sessions, otherwise the bridge refuses the connection
  (the tools retry for `--connect-timeout`, 10 s by default).
- **Opening `/dev/ttyACM0` reboots the bridge** (even with DTR/RTS off): never run
  `espflash monitor` or open the port during a capture. Flash with `espflash flash` without
  `--monitor` (the cargo runner adds `--monitor`, so don't use `cargo run`).
- `probe-rs download` leaves the core halted; `probe-rs reset` starts it clean. In a capture
  started right after a reset the first records can still be from before it: wait ~5 s after a
  reset or reflash before capturing.
- The user doesn't want to run cargo themselves: build and flash for them.

## Firmwares

| firmware | source |
|---|---|
| baseline (sonde, pass-through bridge) | git tag `bench/baseline-sonde` (892f3f3), both `app_core` and `app_comm` |
| baseline, all IMU samples (H0-b) | the tag with `ACCELERATION_DECIMATION {1U}` in `app_core/src/service/data_publisher.h`; local change only, never commit it |
| protobuf (H1) | the integrated branch/tag of workstream A6 (`git tag -l 'bench/*'`, else ask) |
| ESP32 WiFi only (H1-w) | the protobuf `app_comm` of tag `bench/*` before the bridge rework, built with `--features bench-wifi` (the forwarding bridge has no `bench-wifi`) |

Build each firmware in its own worktree so the user's checkout stays untouched:

```bash
git worktree add ../robby-bench-baseline bench/baseline-sonde
```

### Build and flash the STM32

```bash
cd <worktree>/app_core
cmake --preset Release && cmake --build --preset Release
probe-rs download --probe 0483:374b --chip STM32F723IEKx build/Release/app_core.elf
probe-rs reset --probe 0483:374b --chip STM32F723IEKx
```

### Build and flash the ESP32

```bash
cd <worktree>/app_comm
set -a; . /home/stefan/embedded/robby/app_comm/wifi.env; set +a   # bash; in fish run it via `bash -c`
cargo build --release [--features bench-wifi]
espflash flash --chip esp32c6 -p /dev/ttyACM0 target/riscv32imac-unknown-none-elf/release/<bin>
```

`<bin>` is `proto-bride` on the baseline, `robby-outpost` on the translating protobuf bridge,
`robby-bridge` on the forwarding one (check `[[bin]]` in `Cargo.toml`). Keep the env vars inside the build command's shell; don't export them
into a shell whose output you print. After flashing, wait ~5 s for the WiFi to come up, then
check the bridge with a short capture (`--seconds 3`) before the real run.

## Run matrix

Run from `tools/bench/`; files go to `tools/bench/runs/` (gitignored). Note RSSI, AP distance
and WiFi channel for every run with `--meta` (RSSI is in the `bridge/rssi` trace, `outpost/rssi`
on the translating protobuf bridge; for the baseline ask the user or read it from the router).

| run | firmware | baud | load | command |
|---|---|---|---|---|
| H0-a | baseline, decimation 2 | 1 M | normal | `uv run capture.py $URL --seconds 120 --out runs/h0-a.blip --label "H0-a baseline sonde" --meta ...` |
| H0-b | baseline, decimation 1 | 1 M | normal | `uv run capture.py $URL --seconds 60 --out runs/h0-b.blip --label "H0-b sonde all samples"` |
| H1-a | protobuf | 1 M | normal, all samples | `uv run capture.py $URL --seconds 120 --out runs/h1-a.blip --label "H1-a protobuf"` |
| H1-b | protobuf | 1 M | ramp | `uv run ramp.py $URL --out runs/h1-b.blip --label "H1-b ramp 1M"` |
| H1-c | protobuf | 2 / 3 / 4 M | ramp | `uv run ramp.py $URL --out runs/h1-c-2m.blip ...` once per baud rate (set it in both firmwares as A4/A5 document) |
| H1-w | `bench-wifi` | – | ramp | `uv run ramp.py $URL --out runs/h1-w-near.blip ...`, once close to the AP and once at the robot's normal distance |

`URL=ws://127.0.0.1:3000/ws/outpost` (the outpost) for the protobuf runs,
`ws://192.168.1.70/ws/robby` for the baseline. H0 needs only the baseline and can run before the protobuf
work is merged. The ramp's default rates are 1k…80k samples/s with 10 s steps; extend with
`--rates` if a ramp ends without drops (`no drops up to …` in the report).

What to check while running:

- H0-a: `ws.bytes_per_s` ≈ 67 kB/s, `imu/acc` (per-axis) ≈ 393 samples/s. Far off → stop and
  look before measuring more (the IMU stall of 2026-10-19 showed as 15 s gaps).
- H0-b: expect gaps or a rate below 806/s: sonde can't carry all samples at 1 Mbaud.
- Ramps: each step must show `drop_counter_samples` ≥ 1; the knee is the first step with a drop
  counter going up or a delivered rate < 95 %. Read the cause from the counters (plan §5
  "Reading the result"): `outpost/seq_gaps` or STM32 link load ≈ 100 % → UART; `bridge/queue_drops` / long
  `bridge/ws_send_max` → WiFi; `bridge/cpu_idle` ≈ 0 → ESP32 CPU.

## Results

```bash
uv run report.py runs/*.json --baseline h0-a --write
```

writes `doc/bench/results-<date>.md` at the repo root. Add a short summary on top (the knee of
each ramp, which hop limits, RSSI/distance per run) and the conclusions for plan §8. Commit the
report (not `runs/`; the recordings are large; keep them locally or attach them when the user
asks). Then update the reference numbers in the agent memory (`hardware-stream-testing`).

To test the tools without hardware: `uv run simulate.py --style outpost --capacity 25000` and
point the commands at `ws://127.0.0.1:8765/ws/robby`.
