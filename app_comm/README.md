# ESP Tap

Tap into a project using an ESP32-C6 and WiFi.

- serves web page
- sends data using websocket

based on: https://github.com/esp-rs/esp-idf-svc/blob/master/examples/http_ws_server.rs

usage

```
cargo run --release
```

- connecto to esp-tap
- go to the page at 192.168.71.1
- or connect to websocket directly: websocat ws://192.168.1.70/ws/adc