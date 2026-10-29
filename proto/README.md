# robby link schema

The protobuf messages between the STM32 (`app_core`, nanopb) and blip, and what blip
makes of them. The ESP32 bridge (`app_comm`) only frames them: it forwards the payloads on
its WebSocket without decoding them. On the PC, `blip-outpost protobuf` (the outpost of
[blip](https://github.com/stefanluethi/blip)) loads this schema at run time and translates
the messages as its annotations say. A change of the schema therefore needs new STM32
firmware only: no new bridge firmware, no generated Python code, no change of the outpost.

```
STM32 ── Uplink frames ──► ESP32 bridge ── Uplink payloads ──► blip-outpost ── blip CBOR ──► blip
      ◄─ Downlink frames ──              ◄─ Downlink payloads ──   protobuf    ◄─ Invoke ──────
            UART 1 Mbaud                  ws://<ip>/ws/robby                 ws://127.0.0.1:3000/ws/outpost
```

| File | What |
|---|---|
| [`robby_link.proto`](robby_link.proto) | the messages (proto3, package `robby.link`) with their blip annotations |
| [`blip/options.proto`](blip/options.proto) | the annotations, a copy of blip's `outpost/src/blip_outpost/proto/blip/options.proto` (the outpost uses its own copy first) |
| [`robby_link.options`](robby_link.options) | nanopb limits: everything static, no callbacks |
| [`vectors/`](vectors) | golden vectors and the script that makes them |

Run the outpost with [`tools/outpost`](../tools/outpost): `uv run outpost.py`.

## Frames on the UART

Both directions use the same framing. One frame carries exactly one `Uplink` (STM32 →
ESP32) or one `Downlink` (ESP32 → STM32):

```
payload  = protobuf bytes of the Uplink / Downlink         ≤ 1022 bytes (may be empty)
decoded  = payload ‖ CRC-16(payload), big-endian            ≤ 1024 bytes
frame    = COBS(decoded) ‖ 0x00                             ≤ 1030 bytes on the wire
```

- **CRC-16/CCITT-FALSE**: polynomial 0x1021, init 0xFFFF, not reflected, no final XOR
  (check value: `"123456789"` → 0x29B1). The STM32 CRC unit computes it with
  `POLYSIZE = 16`, `POL = 0x1021`, `INIT = 0xFFFF`, no input/output reversal. It's
  appended most significant byte first, so the CRC over the whole decoded frame
  (payload and trailer) is 0: a receiver can check that instead of comparing.
- **COBS** with 0x00 as the delimiter after each frame. The `.frame` vectors use
  canonical COBS (no 0x01 after a final 254-byte run); decoders accept both forms.
- **Size**: the decoded frame is at most 1 KiB, so the payload is at most 1022 bytes and
  the frame on the wire at most 1024 + 5 (COBS) + 1 (delimiter) = 1030 bytes.
- A receiver drops a frame that's too long, fails COBS or fails the CRC, counts it
  (`rx_errors` / `bridge/*_errors`) and goes on with the next 0x00. A payload that doesn't
  decode as protobuf is counted by the STM32 (`rx_errors`) or the outpost
  (`outpost/decode_errors`). An `Uplink` with `seq` 0 and no records is a valid, empty payload: the frame is
  `03 FF FF 00` ([`uplink_empty.frame`](vectors/uplink_empty.frame)).

### Packing records (STM32)

An `Uplink` holds as many `Record`s as fit into 1022 bytes (and at most 32). The
encoder writes `seq`, then appends each record as field 2 (`pb_encode_tag` +
`pb_encode_submessage`) until the next record doesn't fit, then sends the frame and
starts the next. It never builds a whole `robby_link_Uplink` struct.

Any single record fits into an empty frame. The largest is a 100-sample `ImuBatch` with
every value at its widest: 929 bytes as a `Record`, 938 bytes as an `Uplink`
([`uplink_imu_worst`](vectors/uplink_imu_worst.json)). nanopb's
`robby_link_Record_size` (1220) and `robby_link_ImuBatch_size` (1217) overstate it, because nanopb counts a tag per element
of a packed array. So the encoder checks the size of each record with
`pb_get_encoded_size` against the room left in the frame, not with the `_size` macros.

The struct sizes on the Cortex-M7: `robby_link_Record` 632 bytes (the `ImuBatch` with
its three `int16_t[100]`), `robby_link_Downlink` 24 bytes, `robby_link_Uplink` 20232
bytes (which is why the encoder streams records instead).

### On the WebSocket of the bridge

The bridge checks COBS and the CRC of each uplink frame and sends its payload, the
protobuf bytes of one `Uplink`, as one binary WebSocket frame. A binary frame from the
outpost holds the protobuf bytes of one `Downlink` (at most 128 bytes, an `Invoke` takes
fewer than 100), which the bridge sends to the STM32 as a frame. Text frames from the bridge hold its own counters as JSON (`bridge/*`, see
below). One client at a time, at `ws://<ip of the bridge>/ws/robby`.

## Sequence numbers

`Uplink.seq` (annotated `ROLE_SEQUENCE`): the STM32 sends 0 in its first frame after a
reset and adds 1 per frame, wrapping at 2³². The outpost keeps the `seq` it expects next:

- `d = (seq − expected) mod 2³²`, read as a signed 32-bit number.
- `d = 0`: in order. `d > 0`: `d` frames were lost (on the UART, or dropped by the bridge),
  add them to `outpost/seq_gaps`. `d < 0` or `seq = 0`: the STM32 restarted (count
  `outpost/resets`, no gap), the time of its records starts again near 0.
- Then `expected = seq + 1`. Frames that fail COBS/CRC don't change `expected`, so they
  show up as gaps too. After the connection to the bridge was lost, the first frame
  counts no gap.

`Downlink.seq`: the same rule the other way, the outpost starts at 0 when it starts; the
STM32 counts gaps in `LinkStats.rx_seq_gaps`.

## Records and their blip messages

The `(blip.message)` and `(blip.field)` annotations in [`robby_link.proto`](robby_link.proto)
decide what each record becomes in blip, see `blip/options.proto` for all of them. In
short: a message has a path, its value fields become traces under it (or one record value
with `record: true`), `unit` and `scale` apply per field, and the field with
`ROLE_TIMESTAMP` times the values. Today's annotations give:

| Record | blip path | blip value | Annotation |
|---|---|---|---|
| `ImuBatch` | `imu/acc` | record `{x, y, z}`, floats in g, one `Value` per sample | `record`, `scale: 0.000061`; sample `i` at `t_first_ns + i × period_ns` (`ROLE_PERIOD`) |
| `DistanceImage` | `tof/distance` | array of 24 arrays of 8: int mm or `null` | `shape: [24, 8]`, `null_below: 0`: `tof/distance[s × 8 + y][x]` = `mm[(s × 8 + y) × 8 + x]` (s 0 left, 1 middle, 2 right; zone `x + 8y`) |
| `Battery` | `battery/soc` | int, % | `soc_pct` |
| | `battery/voltage` | float, V | `voltage_mv`, `scale: 0.001` |
| | `battery/cell1`, `battery/cell2` | float, V | `cell1_mv`, `cell2_mv`, `scale: 0.001` |
| | `battery/current` | int, mA | `current_ma` (+ charging, − discharging) |
| `TaskStats` | `tasks/<name>` | record `{priority, runtime, stack_size, stack_usage}` | path `tasks/{name}`; `runtime = runtime_bp × 0.01` (float %); `stack_size = stack_size_bytes` (int); `stack_usage = stack_usage_bp × 0.01` (float %, computed by the STM32 from the high water mark) |
| `LogRecord` | – | `Log {timestamp, level, component, message}` | `KIND_LOG`; `level` above ERROR (FATAL) is an error; `component`: the `Component` name without its prefix, lower case (`COMPONENT_IMU` → `imu`) |
| `ActionResult` | – | `ActionResult {id, result}` | `KIND_ACTION_RESULT`: answers the invocation of that `id`, `error` empty → `{"Ok": null}`, else `{"Err": error}` |
| `LinkStats` | `link/*` | see below | path `link` |
| `Synthetic` | `bench/synthetic` | record `{x, y, z}`, floats, one `Value` per sample | like `ImuBatch`, so the load costs the same on WiFi as the IMU |

The outpost sends the blip messages in the order of the records and samples; it doesn't
reorder them. The unit of a trace goes out before its first value, and to every blip that
connects later.

### `link/*`: the STM32 side of the link

`LinkStats` arrives once per second. Its counters count up from 0 after a reset and wrap
at 2³²; they are passed on as totals (the bench tooling takes first/last/delta of every
`link/*`, `bridge/*` and `outpost/*` trace). The STM32 computes the rates itself, over the
time since the previous `LinkStats` (since the reset for the first one).

| blip path | Unit | Field |
|---|---|---|
| `link/tx_bytes` | B | `tx_bytes` |
| `link/tx_frames` | – | `tx_frames` |
| `link/tx_frames_dropped` | – | `tx_frames_dropped` |
| `link/records_dropped` | – | `records_dropped` |
| `link/rx_frames` | – | `rx_frames` |
| `link/rx_errors` | – | `rx_errors` |
| `link/rx_seq_gaps` | – | `rx_seq_gaps` |
| `link/baud` | Bd | `baud` |
| `link/tx_write_max` | µs | `tx_write_max_us` (longest write of the last second) |
| `link/tx_rate` | B/s | `tx_bytes_per_s` |
| `link/tx_frame_rate` | 1/s | `tx_frames_per_ks × 0.001` |
| `link/tx_load` | % | `tx_load_bp × 0.01`: tx bytes/s / (baud / 10) |
| `link/tx_write_time` | % | `tx_write_time_bp × 0.01`: share of the time spent writing |

### `bridge/*`: counters of the ESP32

The bridge sends its own counters once per second as JSON in a text frame,
`{"values": {"bridge/uart_rx_bytes": 1234, ...}}`, and their units once per connection.
They have no timestamp: the outpost gives them its estimate of the STM32 time (the newest
STM32 timestamp plus the time since). Totals since boot, wrapping at 2³², unless a unit
says otherwise.

| blip path | Unit | Counts |
|---|---|---|
| `bridge/uart_rx_bytes` | B | bytes read from the UART |
| `bridge/uart_frames` | – | frames that passed COBS and the CRC |
| `bridge/uart_rx_errors` | – | UART RX errors (FIFO overflow, framing), each resyncs to the next frame |
| `bridge/cobs_errors` | – | frames that failed COBS or were longer than 1 KiB |
| `bridge/crc_errors` | – | frames with a CRC mismatch |
| `bridge/queue_drops` | – | uplink frames dropped because the UART→WebSocket queue was full |
| `bridge/queue_high_water` | – | most frames ever in that queue (of 32) |
| `bridge/ws_tx_bytes` | B | WebSocket payload bytes sent |
| `bridge/ws_frames` | – | WebSocket frames sent |
| `bridge/ws_send_max` | µs | longest WebSocket send of the last second (until the bytes are in the 16 KiB TCP buffer) |
| `bridge/downlink_frames` | – | downlink frames written to the UART |
| `bridge/downlink_drops` | – | downlink messages dropped (longer than 128 bytes, or the queue to the UART full) |
| `bridge/cpu_idle` | % | idle share of the CPU in the last second |
| `bridge/rssi` | dBm | WiFi signal, `null` while not connected |

### `outpost/*`: counters of the outpost

Once per second, with the outpost's estimate of the STM32 time.

| blip path | Counts |
|---|---|
| `outpost/frames` | `Uplink` messages received |
| `outpost/decode_errors` | messages that weren't a valid `Uplink` |
| `outpost/seq_gaps` | `Uplink` frames lost, by `seq` |
| `outpost/resets` | restarts of the STM32 seen in `seq` |

## Actions

The outpost derives the actions from the schema: the `Invoke` message (`KIND_INVOKE`) in
`Downlink` has a oneof `args`, each message member is an action and its fields are the
parameters. The annotations of the member give the path and the description, those of
the fields the unit, the range and the default:

```json
{"Actions": [
  {"path": "motor/speed", "description": "Duty cycle of the motor PWM",
   "params": [{"name": "speed", "unit": "‰", "kind": {"Int": {"min": 0, "max": 1000}}}]},
  {"path": "bench/load", "description": "Synthetic IMU-like samples on top of the normal stream",
   "params": [{"name": "rate", "unit": "samples/s", "kind": {"Int": {"min": 0, "max": 100000}}},
              {"name": "batch", "unit": "samples", "kind": {"Int": {"min": 0, "max": 100}},
               "default": 0}]}
]}
```

| blip action | `Invoke.args` | blip args → protobuf |
|---|---|---|
| `motor/speed` | `MotorSpeed` | `speed` → `speed` (required) |
| `bench/load` | `BenchLoad` | `rate` → `rate` (required, 0 stops), `batch` → `batch` (optional, 0 = STM32 default 100) |

Invoking an action from blip:

1. blip_outpost checks the arguments against the parameters (type, range, required) and
   rejects invalid ones itself, e.g. `{"Err": "speed: 1001 is above the maximum 1000"}`.
2. The outpost sends `Downlink {seq, invoke: {id, <args>}}` with an `id` of its own
   (1, 2, ...). The actions are disabled in blip while the bridge isn't connected.
3. The STM32 checks again, carries the action out and answers with an `ActionResult` of
   that `id` (also `Downlink`s with an `Invoke` without args: `"unknown action"`). The
   outpost answers blip with it, or with `{"Err": "no answer from the target"}` if none
   comes within 1 s.

An `Invoke` with a `MotorSpeed` of speed 0 still carries the (empty) `MotorSpeed`: a set
oneof member is encoded even when all its fields are defaults
([`downlink_motor_speed_zero`](vectors/downlink_motor_speed_zero.json)).

A new action is a message for its arguments and a new member of `Invoke.args` with the
next free number, annotated with its path and description, and the STM32 code that
carries it out. The bridge and the outpost need no change.

## Versioning

Fields and messages are only ever **added**. A field number is never reused, renumbered
or given another type, and a field that goes away is marked `reserved`, so old and new
firmware of either side keep decoding each other's frames (unknown fields are skipped,
missing ones read as defaults). Changing what a field means (its unit, its scale) counts
as a new field. The annotations may change at any time: they only change what the outpost
shows, not the bytes on the wire. Numbers 1–15 (one-byte tags) are for fields sent often.

## Generating code

The schema itself: `protoc -I proto --descriptor_set_out=/dev/null proto/robby_link.proto`.

Only the STM32 has generated code. The bridge doesn't decode the messages, and the outpost
builds its message classes from the schema at run time (with protoc).

### nanopb (STM32)

nanopb 0.4.9.1. The generator finds `robby_link.options` next to the `.proto`; `-x`
keeps the annotations out of the generated `#include`s (they generate no code):

```bash
uvx --from nanopb==0.4.9.1 nanopb_generator -I proto -D <out-dir> -x blip/options.proto proto/robby_link.proto
```

This writes `robby_link.pb.h` / `robby_link.pb.c`, which build against the nanopb runtime
of the same version (`pb_encode.c`, `pb_decode.c`, `pb_common.c`; FetchContent of the
`0.4.9.1` tag). All fields are static: no `pb_callback_t`, the `int16_t` arrays take
`AccelerationFrame` data as they are.

## Golden vectors

[`vectors/generate.py`](vectors/generate.py) builds the messages from the `.proto` at
run time (needs `protoc`) and writes for each one:

- `<name>.bin`: the protobuf bytes (the payload). nanopb and the Python protobuf runtime
  of the outpost encode the same message to the same bytes (fields in number order,
  packed arrays), so encoders can compare byte for byte.
- `<name>.frame`: the frame on the UART, CRC and COBS with the 0x00 delimiter.
- `<name>.json`: the message type, the sizes and the decoded content: all fields that
  aren't in a oneof (also defaults), of a oneof only the member that is set, 64-bit
  integers as numbers, enums by name.

```bash
uv run proto/vectors/generate.py          # write them
uv run proto/vectors/generate.py --check  # fail if they're out of date
```

| Vector | Content | Protobuf | Frame |
|---|---|---:|---:|
| `uplink_empty` | `seq` 0, no records | 0 | 4 |
| `uplink_imu_full` | `ImuBatch` of 100 samples at rest, 806 Hz | 728 | 734 |
| `uplink_imu_worst` | the same with every value at its widest (`seq` and times at their maximum, ±32767/−32768) | 938 | 945 |
| `uplink_distance` | `DistanceImage` with invalid zones (top rows and 3 more) | 375 | 380 |
| `uplink_status` | `Battery`, 3 × `TaskStats`, `LinkStats` | 163 | 167 |
| `uplink_log` | `LogRecord`, `ActionResult` ok and with an error | 83 | 87 |
| `uplink_synthetic` | `Synthetic` of 50 samples | 225 | 229 |
| `downlink_motor_speed` | `Invoke` `motor/speed` {speed: 500} | 11 | 15 |
| `downlink_motor_speed_zero` | `Invoke` `motor/speed` {speed: 0} | 8 | 12 |
| `downlink_bench_load` | `Invoke` `bench/load` {rate: 20000, batch: 100} | 14 | 18 |
| `downlink_invoke_no_args` | `Invoke` without args | 6 | 10 |

**Bytes per IMU sample**: 734 / 100 = **7.3 bytes** on the wire for a realistic full
batch (z ≈ 1 g takes 3 bytes, x/y 2 bytes each, the batch header and frame ~0.3 bytes per
sample), at most 9.5 bytes. At 806 Hz that's ~5.9 kB/s, against ~160 bytes per sample
with sonde. With the distance images (~3.8 kB/s at up to 10 Hz) and the status records
(~0.5 kB/s for 16 tasks) the normal stream is ~10 kB/s, ~10 % of the UART at 1 Mbaud.
