# robby outpost

Serves blip the traces of robby: `blip-outpost protobuf` with the schema of
[`proto/`](../../proto). It connects to the ESP32 bridge, loads
[`robby_link.proto`](../../proto/robby_link.proto) at run time and translates the protobuf
messages of the STM32 as their annotations say: trace paths, units, scaling, log records
and the actions. No code is generated from the schema, so a change of the `.proto` takes
effect when the outpost starts again.

```bash
cd tools/outpost
uv run outpost.py                                   # the bridge at ws://192.168.1.70/ws/robby
uv run outpost.py ws://10.0.0.5/ws/robby --record run.blip
```

Then add the connection `ws://127.0.0.1:3000/ws/outpost` in blip. Options of
`blip-outpost protobuf` may follow the URL: `--ws-port`, `--ws-host 0.0.0.0` (blip on
another machine), `--record`, `--timeout` (for the answer of the STM32 to an action),
`-v`.

The script only fills in robby's schema; the same command by hand:

```bash
blip-outpost protobuf ws://192.168.1.70/ws/robby \
    --proto proto/robby_link.proto -I proto \
    --uplink robby.link.Uplink --downlink robby.link.Downlink
```

Needs `protoc` on the PATH. `blip-outpost` comes from the checkout of blip next to this
repository (`../blip/outpost`, see `pyproject.toml`).

## What it publishes

The traces of the STM32 as annotated, the `bridge/*` counters of the ESP32, and its own
`outpost/*` counters, see [`proto/README.md`](../../proto/README.md).

## Tests

```bash
uv run pytest
```

The golden vectors of [`proto/vectors`](../../proto/vectors) through the outpost: the blip
messages of `proto/README.md`, the actions and their downlink payloads byte for byte.
