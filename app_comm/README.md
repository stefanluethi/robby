# ESP Tap

Tap into a project using an ESP32-C6 and WiFi.

- receives COBS frames on the UART, each a batch of [blip](https://github.com/stefanluethi/blip)
  messages encoded by sonde on the target
- forwards each frame as one binary message to a [blip](https://github.com/stefanluethi/blip)
  client connected to the websocket
- forwards the messages of blip (e.g. invoked actions) COBS framed to the UART

based on: https://github.com/esp-rs/esp-idf-svc/blob/master/examples/http_ws_server.rs

usage

```
cargo run --release
```

- connect to the same WiFi as the ESP
- in blip, add the connection `ws://<ip of the esp>/ws/robby`
