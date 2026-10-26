# robby outpost

The ESP32-C6 between the STM32 of robby and [blip](https://github.com/stefanluethi/blip).

```
STM32 ── protobuf frames, UART 1 Mbaud ──► ESP32 outpost ── blip CBOR, ws://<ip>/ws/robby ──► blip
      ◄─ protobuf frames ──────────────────              ◄─ Invoke ────────────────────────────
```

- receives `Uplink` frames on the UART (protobuf, CRC-16, COBS, see
  [`proto/README.md`](../proto/README.md)), checks them and turns their records into blip
  messages: paths, units, scaling and a timestamp per sample
- sends them to the blip client on the WebSocket, batched as CBOR sequences of up to 4 KiB per
  frame (or 20 ms after the first message of a batch)
- sends each new client the units (`ValueMetadata`) and the actions (`Actions`)
- turns blip's `Invoke` requests into `Downlink` frames for the STM32, routes its
  `ActionResult`s back and answers itself when the request is invalid or the STM32 doesn't
  answer within 1 s
- publishes its own counters once per second as `outpost/*`

One client at a time. Without a client the frames are still decoded, so the outpost keeps
the task names, the link rates and the time of the STM32 current.

## Layout

| Path | What |
|---|---|
| [`outpost-core/`](outpost-core) | `no_std` library, all of the translation: framing, `seq`, micropb decoding, blip messages with minicbor, batching, actions, counters, the load generator. No heap, no I/O, tested on the host |
| [`src/main.rs`](src/main.rs) | the firmware: WiFi, the UART task (interrupt executor), the WebSocket task, CPU idle and RSSI |

Everything is static: the radio needs the 64 KiB heap for itself. The UART task only
deframes, checks the CRC and the `seq` and queues the payload (32 × 1 KiB); the WebSocket
task decodes and translates, so the translation can't delay reading the UART FIFO.

## Usage

```bash
set -a; source wifi.env; set +a   # WIFI_SSID=..., WIFI_PASSWORD=... (gitignored)
cargo run --release
```

The build generates the protobuf code with micropb, so `protoc` must be on the `PATH`.

- connect to the same WiFi as the ESP
- in blip, add the connection `ws://<ip of the esp>/ws/robby`

### `bench-wifi`

```bash
cargo run --release --features bench-wifi
```

No UART: the outpost generates `bench/synthetic` samples itself (like the IMU at rest, the
same bytes on WiFi) at the rate set by the `bench/load` action (`rate` in samples/s, 0
stops). The ramp is done by the host tool (`tools/bench/ramp.py`). Samples more than 100 ms
behind are skipped and counted in `outpost/bench_drops`. `motor/speed` is refused.

## Tests

`outpost-core` runs on the host. The firmware's `.cargo/config.toml` builds for the ESP32
by default, so name the host target:

```bash
cargo test -p outpost-core --target host-tuple
cargo run -p outpost-core --release --target host-tuple --example bench   # translation speed
```

The tests feed the golden vectors of [`proto/vectors`](../proto/vectors) through the outpost
and read its CBOR with blip's own `proto` crate (a git dev-dependency pinned to a commit of
blip), compare the downlink frames byte for byte and cover the error paths.

## `outpost/*` counters

Totals since boot (wrapping at 2³²) unless a unit says otherwise. Their timestamp is the
outpost's estimate of the STM32 time.

| Path | Unit | What |
|---|---|---|
| `outpost/uart_rx_bytes` | B | bytes read from the UART |
| `outpost/uart_frames` | | frames that passed COBS, CRC and the `seq` check |
| `outpost/uart_rx_errors` | | UART RX errors (FIFO overflow, framing), each resyncs to the next frame |
| `outpost/cobs_errors` | | frames that failed COBS or were longer than 1 KiB |
| `outpost/crc_errors` | | frames with a CRC mismatch |
| `outpost/decode_errors` | | frames that weren't a valid `Uplink` |
| `outpost/seq_gaps` | | `Uplink` frames lost, by `seq` |
| `outpost/stm32_resets` | | restarts of the STM32 seen in `seq` |
| `outpost/queue_drops` | | uplink **frames** dropped because the UART→WebSocket queue was full |
| `outpost/queue_high_water` | | most frames ever in that queue (of 32) |
| `outpost/ws_tx_bytes` | B | WebSocket payload bytes sent |
| `outpost/ws_frames` | | WebSocket frames sent |
| `outpost/ws_messages` | | blip messages sent |
| `outpost/ws_send_max` | µs | longest WebSocket send of the last second (until the bytes are in the 16 KiB TCP buffer) |
| `outpost/bench_drops` | | `bench-wifi`: synthetic samples skipped because the WebSocket fell behind |
| `outpost/cpu_idle` | % | idle share of the CPU in the last second (idle hook of esp-rtos) |
| `outpost/rssi` | dBm | WiFi signal, `null` while not connected |
