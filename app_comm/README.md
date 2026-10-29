# robby bridge

The ESP32-C6 between the STM32 of robby and the outpost on the PC, which serves
[blip](https://github.com/stefanluethi/blip).

```
STM32 ── protobuf frames, UART 1 Mbaud ──► ESP32 bridge ── protobuf, ws://<ip>/ws/robby ──► blip-outpost protobuf ──► blip
      ◄─ protobuf frames ──────────────────              ◄─ protobuf ──────────────────────
```

- receives `Uplink` frames on the UART (protobuf, CRC-16, COBS, see
  [`proto/README.md`](../proto/README.md)), checks COBS and the CRC and sends each payload
  as one binary WebSocket frame, unchanged
- frames the binary WebSocket frames of the outpost (`Downlink` payloads) for the STM32
- sends its own counters once per second as JSON in a text frame (`bridge/*`), and their
  units once per connection

The bridge doesn't decode the protobuf messages, so a change of
[`robby_link.proto`](../proto/robby_link.proto) needs no new bridge firmware: the outpost
loads the schema at run time ([`tools/outpost`](../tools/outpost)).

One client at a time. Without a client the uplink frames are dropped.

## Layout

| Path | What |
|---|---|
| [`bridge-core/`](bridge-core) | `no_std` library: framing (COBS, CRC), the counters and their JSON. No heap, no I/O, tested on the host |
| [`src/main.rs`](src/main.rs) | the firmware: WiFi, the UART task (interrupt executor), the WebSocket task, CPU idle and RSSI |

Everything is static: the radio needs the 64 KiB heap for itself. Downlink frames are
capped at 128 payload bytes: the UART task runs in an interrupt executor, on the stack of
whatever task it interrupts, and 1 KiB frames moved there overflowed the WiFi tasks'
stacks (a load access fault in `esp_radio` at boot). The UART task deframes,
checks the CRC and queues the payload (32 × 1 KiB); the WebSocket task sends it, so a slow
WiFi can't delay reading the UART FIFO.

## Usage

```bash
set -a; source wifi.env; set +a   # WIFI_SSID=..., WIFI_PASSWORD=... (gitignored)
cargo run --release
```

- connect the PC to the same WiFi as the ESP
- start the outpost: `cd tools/outpost && uv run outpost.py ws://<ip of the esp>/ws/robby`
- in blip, add the connection `ws://127.0.0.1:3000/ws/outpost`

## Tests

`bridge-core` runs on the host. The firmware's `.cargo/config.toml` builds for the ESP32
by default, so name the host target:

```bash
cargo test -p bridge-core --target host-tuple
```

The tests deframe the golden vectors of [`proto/vectors`](../proto/vectors), frame the
downlink vectors byte for byte, cover the error paths and check the JSON of the counters.

## `bridge/*` counters

Totals since boot (wrapping at 2³²) unless a unit says otherwise. They have no timestamp of
their own: the outpost gives them its estimate of the STM32 time.

| Path | Unit | What |
|---|---|---|
| `bridge/uart_rx_bytes` | B | bytes read from the UART |
| `bridge/uart_frames` | | frames that passed COBS and the CRC |
| `bridge/uart_rx_errors` | | UART RX errors (FIFO overflow, framing), each resyncs to the next frame |
| `bridge/cobs_errors` | | frames that failed COBS or were longer than 1 KiB |
| `bridge/crc_errors` | | frames with a CRC mismatch |
| `bridge/queue_drops` | | uplink frames dropped because the UART→WebSocket queue was full |
| `bridge/queue_high_water` | | most frames ever in that queue (of 32) |
| `bridge/ws_tx_bytes` | B | WebSocket payload bytes sent |
| `bridge/ws_frames` | | WebSocket frames sent |
| `bridge/ws_send_max` | µs | longest WebSocket send of the last second (until the bytes are in the 16 KiB TCP buffer) |
| `bridge/downlink_frames` | | downlink frames written to the UART |
| `bridge/downlink_drops` | | downlink messages dropped: longer than 128 bytes, or the queue to the UART was full |
| `bridge/cpu_idle` | % | idle share of the CPU in the last second (idle hook of esp-rtos) |
| `bridge/rssi` | dBm | WiFi signal, `null` while not connected |

Frames lost between the STM32 and the outpost show in `outpost/seq_gaps`, decoding errors
in `outpost/decode_errors` (see [`proto/README.md`](../proto/README.md)).
