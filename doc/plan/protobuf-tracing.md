# Plan: protobuf link and blip outpost on the ESP32

Status: implemented and measured on 2026-10-26, see
[the benchmark report](../bench/results-2026-10-26.md). H1-c (baud sweep) was skipped
because the ESP32 limits before the UART does; H1-w close to the AP is still open.

## 1. Where we are

```
STM32F723 (app_core)                     ESP32-C6 (app_comm)              PC
data_publisher ─ sonde (CBOR) ─ COBS ─ USART2 1 Mbaud ─ UART1 ─ pass-through ─ WS ─ blip
command_handler ◄─ tinycbor ◄── COBS ◄──────────────────────── pass-through ◄─ WS ◄─ blip
```

- The link is USART2 at 1 Mbaud, 8N1: **at most 100 kB/s**, TX by interrupt.
- Measured 2026-10-19 with sonde: **~67 kB/s** on the UART, so the link is ~2/3 full.
  That's with the IMU **decimated by 2** (~393 samples/s per axis of ~806 Hz).
- Why so heavy: each axis value is a full blip `Value` message, i.e. a CBOR map with the
  keys `timestamp`/`path`/`value`, a u64 timestamp and the path string `"imu/acc/x"`.
  That's about **53 bytes per axis value, ~160 bytes per 3-axis sample** (estimate,
  matches the 67 kB/s). Sending every sample would take ~130 kB/s, more than the UART
  can carry.
- sonde stays at its pinned commit (blip `96ff5b2`) as the baseline and is then removed.
  It doesn't get updated.

## 2. Target

```
STM32 (app_core)                              ESP32-C6 "robby outpost" (app_comm)          PC
data_publisher ─ nanopb ─ COBS+CRC ─ USART2 ─ UART1 ─ decode protobuf ─ encode blip CBOR ─ WS ─ blip
command_handler ◄─ nanopb ◄─ COBS+CRC ◄────────────── encode protobuf ◄─ decode Invoke ◄── WS ◄─ blip
```

- The UART carries compact, schema-based protobuf frames: raw sensor counts, no strings,
  one timestamp per batch.
- The ESP32 becomes the outpost. It knows the schema. It turns frames into blip messages
  (paths, units, scaling, timestamps per sample) and batches them into CBOR-sequence
  WebSocket frames. It also turns blip's `Invoke` requests into protobuf for the STM32.
- **blip and its protocol stay unchanged.** It still gets standard blip CBOR over the
  WebSocket; there's no compact message type for blip.

Expected effect (estimates, to be confirmed by the campaign):

| | today (sonde) | protobuf |
|---|---|---|
| bytes per 3-axis IMU sample on the UART | ~160 | ~8–10 (3 zigzag varints + batch header share) |
| IMU rate | 403 Hz (decimated) | 806 Hz (all samples) |
| UART load for the normal stream | ~67 kB/s (67 %) | ~10 kB/s (~10 %) |
| WebSocket bytes for the IMU | ~62 kB/s | ~55 kB/s as one `imu/acc` record {x,y,z} per sample |

Once the UART isn't the limit any more, the bytes move to WiFi: the outpost expands each
sample back into blip CBOR. So we have to measure where the WiFi limit is, see §5.

## 3. Decisions

| Topic | Choice | Why |
|---|---|---|
| STM32 encoder | **nanopb**, generated at build time (FetchContent + `nanopb_generator`), static sizes via `.options` | no heap, small, C, well-proven on Cortex-M |
| ESP32 decoder/encoder | **micropb** (no_std, no alloc, heapless containers) for protobuf; **minicbor** for the blip CBOR | same `.proto` as the STM32; no heap use in the hot path (the radio needs the heap) |
| Schema location | `proto/robby_link.proto` + `proto/robby_link.options` at repo root, shared by both builds | single source of truth |
| Framing | keep COBS; add `seq` (u32) in each frame and a CRC-16/CCITT trailer (STM32 HW CRC unit) | `seq` gaps = drops on the UART, CRC catches bit errors at higher baud rates |
| IMU encoding | `ImuBatch { t_first_ns, period_ns, packed sint32 x/y/z }` in raw LSB, scale applied on the ESP32 | removes per-sample timestamps and floats; the frame-spreading logic of `data_publisher` stays on the STM32 |
| blip paths | outpost emits `imu/acc` as a record `{x, y, z}` (one message per sample) | halves WiFi bytes vs. three per-axis messages; breaking the current blip layouts is accepted |
| Units / metadata | outpost sends `ValueMetadata` itself (to every new client) | STM32 no longer spends bytes on strings |
| Actions | outpost owns the descriptor list (static, derived from the schema); STM32 handles typed `Invoke` messages and answers with `ActionResult` | removes the hand-encoded CBOR in `command_handler` |
| Logs | `LogRecord { t_ns, level, component_id, message }` from the se-oss sink; component names mapped on the ESP32 | |
| sonde / tinycbor | not updated; removed from app_core by A4 | |

## 4. Schema sketch (workstream A3 finalises it)

```proto
syntax = "proto3";
package robby.link;

// STM32 -> ESP32, one per COBS frame
message Uplink {
  uint32 seq = 1;               // +1 per frame; gaps show drops on the UART
  repeated Record records = 2;  // max_count in .options, frame <= 1 KiB
}

message Record {
  oneof kind {
    ImuBatch      imu           = 1;
    DistanceImage distance      = 2;  // 3 sensors x 8x8, sint32 mm, -1 = invalid
    Battery       battery       = 3;
    TaskStats     tasks         = 4;
    LogRecord     log           = 5;
    ActionResult  action_result = 6;
    LinkStats     link_stats    = 7;  // STM32 side counters, see §5
    Synthetic     synthetic     = 15; // load generator payload
  }
}

message ImuBatch {
  uint64 t_first_ns = 1;
  uint32 period_ns  = 2;
  repeated sint32 x = 3 [packed = true];
  repeated sint32 y = 4 [packed = true];
  repeated sint32 z = 5 [packed = true];
}

// ESP32 -> STM32
message Downlink {
  uint32 seq = 1;
  oneof kind {
    Invoke invoke = 2;      // id + typed args per action, e.g. MotorSpeed { int32 rpm }
    BenchLoad bench = 3;    // set the synthetic load, see §5
  }
}
```

## 5. Measuring throughput

The measurements have to answer three questions: how much better the protobuf link is,
how close each hop is to its limit, and at what load WiFi becomes the bottleneck.

### Metrics

| Metric | Where | How |
|---|---|---|
| UART wire bytes/s and load % (bytes/s ÷ baud/10) | STM32 | `LinkStats`: bytes and frames written, write time |
| Encoder CPU | STM32 | `tasks/data_publisher` runtime % (exists) |
| Trace drops on the STM32 | STM32 | `LinkStats`: frames dropped because the queue or buffer was full |
| UART frames, COBS/CRC errors, `seq` gaps | ESP32 | outpost counters |
| Queue drops / high-water of the UART→WS queue | ESP32 | outpost counters |
| WS bytes/s, frames/s, longest send time | ESP32 | outpost counters |
| ESP32 CPU idle % and RSSI | ESP32 | outpost counters |
| Delivered rate per path, interval jitter, gaps | PC | capture tool |
| bytes on the UART per IMU sample | derived | UART bytes/s ÷ delivered IMU rate |

With the new firmware both devices publish their counters as blip traces once per second
(`link/*` from the STM32, `outpost/*` from the ESP32), so they show up in blip and in
every capture.

**Baseline without counters**: today's bridge forwards each COBS frame 1:1 as a
WebSocket frame. So the WebSocket payload bytes the capture tool sees *are* the UART
payload bytes (plus ~0.4 % COBS overhead and 1 delimiter per frame). The baseline runs
on today's firmware unchanged.

### Load generator (new firmware only)

Normal robot traffic doesn't push the protobuf link anywhere near its limit, so we add
synthetic load:

- **STM32 `bench/load` action**: adds `Synthetic` IMU-like batches at N samples/s on top
  of the normal stream. It can be called from blip or from the bench script. A ramp
  (1k, 2k, 5k, 10k, 20k, 40k samples/s, …) finds the knee of the whole chain.
- **ESP32-only synthetic mode** (build feature `bench-wifi`): the outpost generates blip
  CBOR itself without the UART and ramps up until the WebSocket stalls. This gives the
  plain WiFi/TCP limit of the ESP32-C6 with esp-radio + embassy-net.
- **Baud sweep** (cheap): USART2 on APB1 can go up to 6.75 Mbaud with OVER8, the
  ESP32-C6 UART up to 5 Mbaud. Try 2, 3, 4 Mbaud with the same ramp. Risk: at 4 Mbaud
  the 128-byte RX FIFO of the ESP32 fills in ~320 µs, so the WiFi interrupts may cause
  overruns. The CRC and `seq` counters will show this.

### Reading the result

- `seq` gaps at the ESP32, or STM32 link load ≈ 100 % → **UART limit**.
- UART clean, but outpost queue drops / long WS send times → **WiFi/TCP limit**.
- ESP32 CPU idle ≈ 0 with both links below their limit → **outpost translation limit**
  (then optimise the CBOR encoding).
- Compare the knee of the chain with the knee of the `bench-wifi` run: if they match,
  WiFi is the limit.

### Tooling (`tools/bench/`, Python with uv)

- `capture.py ws://192.168.1.70/ws/robby --seconds 120 --out runs/<name>.blip`: records
  the raw frames as a blip recording (gzip CBOR sequence, can be opened in blip) and
  writes `runs/<name>.json` with the metrics above.
- `ramp.py`: calls `bench/load` step by step via `Invoke`, captures each step, stops when
  drops start.
- `report.py runs/*.json`: markdown tables and comparisons, writes `doc/bench/results-<date>.md`.

It works for both firmwares: sonde and the new outpost both deliver blip CBOR, so the
baseline and the protobuf runs use the same tool.

## 6. Workstreams for the agents

Each one runs as its own session in its own worktree and branch, and ends in a branch
that builds and has tests. The workstreams of a wave touch different files, so they can
run in parallel.

```
wave 1 (parallel):  A1 bench tooling (tools/bench)      A3 schema (proto/)
                                                              │
wave 2 (parallel):                     A4 STM32 nanopb ◄──────┤
                                       A5 ESP32 outpost ◄─────┘
wave 3 (here):      A6 integration: merge, cross-check, readiness checklist
later (hardware):   H0 baseline (needs A1)    H1 protobuf campaign (needs A6)
```

The baseline firmware is today's `main`, tagged `bench/baseline-sonde` (892f3f3).

### A1 — Bench tooling · `tools/bench/` · branch `feat/bench-tooling`

- `capture.py`, `ramp.py`, `report.py` as described in §5, uv project, typed and tested.
- Tests against a simulated server: a pytest fixture serves recorded/synthetic blip
  frames over a local WebSocket, so the tool is checked without hardware.
- Project skill `.claude/skills/bench-robby/SKILL.md` with the runbook of §7.
- Done when `uv run pytest` passes and `capture.py` against the simulated server writes
  a correct `.blip` + `.json`.

### A3 — Link schema · `proto/` · branch `feat/link-schema`

- `proto/robby_link.proto` + `.options` (max_count/max_size so that nanopb and micropb
  generate static code), following §4. Typed `Invoke` messages for the current actions
  (see `command_handler.cpp`).
- `proto/README.md`: mapping of each record to blip paths, units and scale factors,
  frame layout (COBS, CRC-16/CCITT, max 1 KiB), versioning rule (fields only added).
- Golden vectors: a handful of `Uplink`/`Downlink` messages as `.bin` + expected decoded
  JSON, generated by a small script, used by A4 and A5.
- Done when `protoc` / `nanopb_generator` / micropb all generate from it without errors.

### A4 — STM32 nanopb encoder · `app_core` · branch `feat/nanopb-core` (after A3)

- nanopb via FetchContent, generate from `proto/` in CMake.
- `Link`: frames with `seq` + CRC, COBS. Consider TX by DMA instead of interrupt.
- `data_publisher`: encode `ImuBatch` (no decimation), distance, battery, tasks
  directly into the frame buffer. Log sink → `LogRecord`.
- `LinkStats` counters (bytes, frames, write time, drops) once per second.
- `command_handler`: decode `Downlink` (typed `Invoke`, `BenchLoad`), answer with
  `ActionResult`. Remove tinycbor, sonde and the hand-encoded actions.
- `bench/load` → `Synthetic` records at N samples/s.
- Host unit tests (new `app_core/test/`, Catch2, host build of the encoder parts), checked
  against the golden vectors.
- Done when the firmware builds, the host tests pass and the flash/RAM delta is noted.

### A5 — ESP32 outpost · `app_comm` · branch `feat/protobuf-outpost` (after A3)

- New no_std crate `app_comm/outpost-core` (host-testable): COBS+CRC framing, micropb
  decoding, translation to blip messages with minicbor, batching into ≤ N KiB CBOR
  sequences, `ValueMetadata` + `Actions` to every new client, `Invoke` → `Downlink`.
- Host tests: golden vectors in; the CBOR output is decoded with blip's own `proto`
  crate (dev-dependency) to prove blip compatibility.
- `app_comm/src/main.rs`: uses `outpost-core`; counters (UART bytes/frames, `seq` gaps,
  COBS/CRC errors, queue drops/high-water, WS bytes/send time, CPU idle, RSSI) published
  as `outpost/*`; `bench-wifi` feature (§5).
- Rename `proto-bride` → `robby-outpost`, update the README.
- Done when `cargo test -p outpost-core` passes on the host and the ESP32 firmware
  builds for both features.

### A6 — Integration (in the planning session)

- Merge A1, A3–A5, run all host tests, cross-check: frames from the A4 host encoder
  through `outpost-core` on the host, decoded by blip's `proto` crate.
- Optional host end-to-end: a host build of `outpost-core` behind a local WebSocket, fed
  by the A4 host encoder, captured with `tools/bench/capture.py`. This checks the
  tooling and gives a first number for the translation cost (not representative of the
  ESP32).
- Readiness checklist for the campaign: all firmware variants build, tags exist,
  `tools/bench` runs.

## 7. Hardware campaign (later)

Triggered by you once the hardware is connected (e.g. "hardware is ready, run the
campaign", or `/bench-robby`). An agent can do all of it itself: flashing works with
probe-rs (ST-Link 0483:374b) and espflash (`/dev/ttyACM0`), the bridge is at
`ws://192.168.1.70/ws/robby`.

| Run | Firmware | Baud | Load | Duration | Answers |
|---|---|---|---|---|---|
| H0-a | `bench/baseline-sonde`, decimation 2 | 1 M | normal | 120 s | baseline, confirms ~67 kB/s |
| H0-b | baseline with `ACCELERATION_DECIMATION = 1` (one-line local change) | 1 M | normal | 60 s | sonde can't carry all samples |
| H1-a | protobuf | 1 M | normal, all samples | 120 s | improvement at normal load |
| H1-b | protobuf | 1 M | ramp | to knee | protobuf knee at 1 Mbaud |
| H1-c | protobuf | 2 / 3 / 4 M | ramp | to knee | does raising the baud rate move the knee to WiFi? |
| H1-w | `bench-wifi` (no UART) | – | ramp | to stall | plain ESP32 WiFi limit |

- H0 can already run when A1 is done, before the protobuf work.
- Note RSSI, the AP distance and the channel for each run; repeat H1-w once close to the
  AP and once at the robot's normal distance.
- Results go to `doc/bench/results-<date>.md` (made by `report.py`) and update the
  reference numbers in the agent memory.

## 8. Risks and open points

- **WiFi bytes grow**: the outpost turns ~10 B/sample back into ~55 B/sample of blip
  CBOR. If H1-w shows WiFi is the limit, the options stay inside this design: larger WS
  frames, TCP buffer tuning, AP placement, a lower trace rate. blip's protocol doesn't
  change.
- **ESP32 UART RX at high baud rates**: may need the UHCI/DMA path of esp-hal instead
  of FIFO interrupts.
- **Heap on the ESP32**: the radio shares the 64 KiB heap; the outpost must use only
  static buffers (bitten before, see f570e27).
- **One client at a time** on the bridge: the capture tool and blip can't both connect.
  The tool records a `.blip` file you can open in blip afterwards.
