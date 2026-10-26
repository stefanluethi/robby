# robby link schema

The protobuf messages on the UART between the STM32 (`app_core`, nanopb) and the ESP32
outpost (`app_comm`, micropb), and how the outpost turns them into blip messages. blip's
own protocol doesn't change: the outpost speaks standard blip CBOR on the WebSocket.

```
STM32 ── Uplink frames ──► ESP32 outpost ── blip Message (CBOR) ──► blip
      ◄─ Downlink frames ──              ◄─ blip Request (Invoke) ──
```

| File | What |
|---|---|
| [`robby_link.proto`](robby_link.proto) | the messages (proto3, package `robby.link`) |
| [`robby_link.options`](robby_link.options) | nanopb limits: everything static, no callbacks |
| [`robby_link.toml`](robby_link.toml) | micropb limits: heapless 0.9 containers, no heap |
| [`vectors/`](vectors) | golden vectors and the script that makes them |

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
- A receiver drops a frame that's too long, fails COBS, fails the CRC or doesn't decode
  as protobuf, counts it (`rx_errors` / `outpost/*_errors`) and goes on with the next
  0x00. An `Uplink` with `seq` 0 and no records is a valid, empty payload: the frame is
  `03 FF FF 00` ([`uplink_empty.frame`](vectors/uplink_empty.frame)).

### Packing records (STM32)

An `Uplink` holds as many `Record`s as fit into 1022 bytes (and at most 32). The
encoder writes `seq`, then appends each record as field 2 (`pb_encode_tag` +
`pb_encode_submessage`) until the next record doesn't fit, then sends the frame and
starts the next. It never builds a whole `robby_link_Uplink` struct.

Any single record fits into an empty frame. The largest is a 100-sample `ImuBatch` with
every value at its widest: 929 bytes as a `Record`, 938 bytes as an `Uplink`
([`uplink_imu_worst`](vectors/uplink_imu_worst.json)). micropb computes this bound exactly
(`Record::MAX_SIZE` = 929). nanopb's `robby_link_Record_size` (1220) and
`robby_link_ImuBatch_size` (1217) overstate it, because nanopb counts a tag per element
of a packed array. So the encoder checks the size of each record with
`pb_get_encoded_size` against the room left in the frame, not with the `_size` macros.

The struct sizes on the Cortex-M7: `robby_link_Record` 632 bytes (the `ImuBatch` with
its three `int16_t[100]`), `robby_link_Downlink` 24 bytes, `robby_link_Uplink` 20232
bytes (which is why the encoder streams records instead).

### Decoding records (ESP32)

A whole `Uplink` struct holds 32 `Record`s, ≈ 20 KiB with micropb too. The outpost decodes
one record at a time instead: read the tags of the `Uplink` with `PbDecoder`, take
field 1 as `seq`, decode each field 2 into one `Record` with
`decode_submessage_delimited`, skip unknown fields with `skip_wire_value`. The whole-struct
decode is still fine for host tests.

## Sequence numbers

`Uplink.seq`: the STM32 sends 0 in its first frame after a reset and adds 1 per frame,
wrapping at 2³². The outpost keeps the `seq` it expects next:

- `d = (seq − expected) mod 2³²`, read as a signed 32-bit number.
- `d = 0`: in order. `d > 0`: `d` frames were lost on the UART, add them to
  `outpost/seq_gaps`. `d < 0` or `seq = 0`: the STM32 restarted (count
  `outpost/stm32_resets`, no gap), the time of its records starts again near 0.
- Then `expected = seq + 1`. Frames that fail COBS/CRC don't change `expected`, so they
  show up as gaps too.

`Downlink.seq`: the same rule the other way, the outpost starts at 0 when it boots; the
STM32 counts gaps in `LinkStats.rx_seq_gaps`.

## Records and their blip messages

All times are the STM32 uptime in ns. The outpost passes them on as the blip
`timestamp`, so the traces of the STM32 keep their time domain.

| Record | blip path | blip value | Conversion |
|---|---|---|---|
| `ImuBatch` | `imu/acc` | record `{x, y, z}`, floats in g, one `Value` per sample | `raw × 0.000061`; sample `i` at `t_first_ns + i × period_ns` |
| `DistanceImage` | `tof/distance` | array of 24 arrays of 8: int mm or `null` | `mm` cut into arrays of 8, in order: `tof/distance[s × 8 + x][y]` = `mm[(s × 8 + x) × 8 + y]` (s 0 left, 1 middle, 2 right; zone `x + 8y`), today's layout; `mm < 0` → `null` |
| `Battery` | `battery/soc` | int, % | `soc_pct` |
| | `battery/voltage` | float, V | `voltage_mv / 1000` |
| | `battery/cell1`, `battery/cell2` | float, V | `cell1_mv / 1000`, `cell2_mv / 1000` |
| | `battery/current` | int, mA | `current_ma` (+ charging, − discharging) |
| `TaskStats` | `tasks/<name>` | record `{priority, runtime, stack_usage, stack_size}` | `priority`; `runtime = runtime_bp / 100` (float %); `stack_usage = 100 × (1 − stack_free_min_bytes / stack_size_bytes)` (float %, 0 if the size is 0); `stack_size = stack_size_bytes` (int) |
| `LogRecord` | – | `Log {timestamp, level, component, message}` | `level`: `min(level, 4)` (blip has no FATAL); `component`: see below |
| `ActionResult` | – | `ActionResult {id, result}` | `error` empty → `{"Ok": null}`, else `{"Err": error}`; to the client that sent the `Invoke` |
| `LinkStats` | `link/*` | see below | |
| `Synthetic` | `bench/synthetic` | record `{x, y, z}`, floats, one `Value` per sample | like `ImuBatch`, so the load costs the same on WiFi as the IMU |

The outpost sends the blip messages in the order of the records and samples; it doesn't
reorder them.

Component names for `LogRecord.component_id` (the log contexts of `app_core`,
`robby::LogContextId`): `COMPONENT_CMD` → `cmd`, `COMPONENT_SPACE` → `space`,
`COMPONENT_IMU` → `imu`. An unknown value becomes its decimal number, e.g. `7`.

### `link/*`: counters of the STM32

`LinkStats` arrives once per second. Its counters count up from 0 after a reset and wrap
at 2³²; the outpost passes them on as totals (the bench tooling takes first/last/delta of
every `link/*` and `outpost/*` trace) and adds rates from the difference to the previous
`LinkStats` (`Δt` from `t_ns`, wrapping subtraction for the counters; no rates for the
first record after a reset).

| blip path | Unit | Value |
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
| `link/tx_rate` | B/s | `Δtx_bytes / Δt` |
| `link/tx_frame_rate` | 1/s | `Δtx_frames / Δt` |
| `link/tx_load` | % | `100 × (Δtx_bytes / Δt) / (baud / 10)` |
| `link/tx_write_time` | % | `100 × Δtx_write_us / Δt` (share of the time spent writing) |

### `outpost/*`: counters of the ESP32

The outpost's own counters, once per second, as totals (same rule as above) unless a
unit says otherwise. Their `timestamp` is the outpost's estimate of the STM32 uptime: the
newest `t_ns` it received plus the ESP32 time since then (its own uptime before the first
record). Workstream A5 may add more under `outpost/`.

| blip path | Unit | Counts |
|---|---|---|
| `outpost/uart_rx_bytes` | B | bytes read from the UART |
| `outpost/uart_frames` | – | frames that decoded |
| `outpost/cobs_errors` | – | frames that failed COBS or were too long |
| `outpost/crc_errors` | – | frames with a CRC mismatch |
| `outpost/decode_errors` | – | frames that weren't a valid `Uplink` |
| `outpost/seq_gaps` | – | Uplink frames lost, by `seq` |
| `outpost/stm32_resets` | – | restarts of the STM32 seen in `seq` |
| `outpost/queue_drops` | – | blip messages dropped because the UART→WS queue was full |
| `outpost/queue_high_water` | – | most entries ever in that queue |
| `outpost/ws_tx_bytes` | B | WebSocket payload bytes sent |
| `outpost/ws_frames` | – | WebSocket frames sent |
| `outpost/ws_send_max` | µs | longest WebSocket send of the last second |
| `outpost/cpu_idle` | % | idle share of the last second |
| `outpost/rssi` | dBm | WiFi signal |

### `ValueMetadata` (units)

The outpost sends these to every client that connects (blip addresses a record field as
`path.field`):

| Path | Unit |
|---|---|
| `imu/acc.x`, `imu/acc.y`, `imu/acc.z` | g |
| `bench/synthetic.x`, `.y`, `.z` | g |
| `tof/distance` | mm |
| `battery/soc` | % |
| `battery/voltage`, `battery/cell1`, `battery/cell2` | V |
| `battery/current` | mA |
| `tasks/<name>.runtime`, `tasks/<name>.stack_usage` (for each task seen) | % |
| `tasks/<name>.stack_size` | B |
| `link/*`, `outpost/*` | the units in the tables above (none for plain counts) |

## Actions

The outpost owns the action list. It sends this `Actions` message to every client that
connects (shown as JSON):

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

Translating an `Invoke {id, path, args}` from blip:

1. Unknown `path`: the outpost answers `{"Err": "unknown action"}` itself.
2. A required argument is missing or isn't an integer, or is outside the bounds of the
   descriptor: the outpost answers `{"Err": "<name> must be an integer from <min> to
   <max>"}` itself.
3. Otherwise it sends `Downlink {seq, invoke: {id, <args>}}` with blip's `id` unchanged.
   The STM32 checks again, carries the action out and answers with an `ActionResult` of
   that `id` (also `Downlink`s with an `Invoke` without args: `"unknown action"`). The
   outpost answers `{"Err": "no answer from the STM32"}` if none comes within 1 s.

An `Invoke` with a `MotorSpeed` of speed 0 still carries the (empty) `MotorSpeed`: a set
oneof member is encoded even when all its fields are defaults
([`downlink_motor_speed_zero`](vectors/downlink_motor_speed_zero.json)).

A new action gets a message for its arguments, a new member of `Invoke.args` with the
next free number, a descriptor here and in the outpost.

## Versioning

Fields and messages are only ever **added**. A field number is never reused, renumbered
or given another type, and a field that goes away is marked `reserved`, so old and new
firmware of either side keep decoding each other's frames (unknown fields are skipped,
missing ones read as defaults). Changing what a field means (its unit, its scale) counts
as a new field. Numbers 1–15 (one-byte tags) are for fields sent often.

## Generating code

The schema itself: `protoc -I proto --descriptor_set_out=/dev/null proto/robby_link.proto`.

### nanopb (STM32)

nanopb 0.4.9.1. The generator finds `robby_link.options` next to the `.proto`:

```bash
uvx --from nanopb==0.4.9.1 nanopb_generator -I proto -D <out-dir> proto/robby_link.proto
```

This writes `robby_link.pb.h` / `robby_link.pb.c`, which build against the nanopb runtime
of the same version (`pb_encode.c`, `pb_decode.c`, `pb_common.c`; FetchContent of the
`0.4.9.1` tag). All fields are static: no `pb_callback_t`, the `int16_t` arrays take
`AccelerationFrame` data as they are.

### micropb (ESP32)

micropb 0.7 with heapless 0.9 (the version `app_comm` already uses):

```toml
[dependencies]
micropb = { version = "0.7", default-features = false, features = ["encode", "decode", "enable-64bit", "container-heapless-0-9"] }
heapless = "0.9"

[build-dependencies]
micropb-gen = "0.7"
```

```rust
// build.rs
use std::path::Path;

fn main() {
    let dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../proto");
    let proto = dir.join("robby_link.proto");
    let config = dir.join("robby_link.toml");
    println!("cargo:rerun-if-changed={}", proto.display());
    println!("cargo:rerun-if-changed={}", config.display());

    let mut generator = micropb_gen::Generator::new();
    generator.add_protoc_arg(format!("-I{}", dir.display()));
    generator.parse_config_file(&config, ".robby.link").unwrap();
    let out = Path::new(&std::env::var("OUT_DIR").unwrap()).join("robby_link.rs");
    generator.compile_protos(&[proto], out).unwrap();
}
```

```rust
mod link {
    #![allow(clippy::all, nonstandard_style, unused, irrefutable_let_patterns)]
    include!(concat!(env!("OUT_DIR"), "/robby_link.rs"));
}
use link::robby_::link_::*; // Uplink, Record, Record_::Kind, Downlink, Invoke_::Args, ...
```

Load the config with `parse_config_file` as above: `compile_protos_with_config_files`
looks for the `.toml` at the path protoc reports *relative to the working directory of the
build script*, so with `-I` it silently finds nothing and the generator then fails with
"vec_type not configured". `protoc` must be on the PATH.

## Golden vectors

[`vectors/generate.py`](vectors/generate.py) builds the messages from the `.proto` at
run time (needs `protoc`) and writes for each one:

- `<name>.bin`: the protobuf bytes (the payload). nanopb and micropb encode the same
  message to the same bytes (fields in number order, packed arrays), so encoders can
  compare byte for byte.
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
| `uplink_status` | `Battery`, 3 × `TaskStats`, `LinkStats` | 156 | 160 |
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
