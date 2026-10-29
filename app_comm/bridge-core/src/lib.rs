//! The core of the robby bridge on the ESP32: unwraps the frames of the STM32 (COBS, CRC)
//! and wraps the frames for it, and keeps the bridge's own counters. It doesn't decode
//! the protobuf messages: `blip-outpost protobuf` does that, from the schema at run time,
//! see `proto/README.md`.
//!
//! No heap and no I/O: the firmware (`app_comm/src/main.rs`) moves the bytes, this crate
//! is tested on the host.
//!
//! ```text
//! UART bytes ─ Deframer ─ payload ─ queue ─ binary WebSocket frame ─► outpost
//! outpost ─► binary WebSocket frame ─ payload ─ frame() ─ UART
//! ```
#![cfg_attr(not(test), no_std)]

pub mod crc;
pub mod framing;
pub mod stats;

pub use framing::{frame, Deframer, DownlinkFrame, FrameError, MAX_DOWNLINK_PAYLOAD, MAX_ENCODED, MAX_PAYLOAD};
pub use stats::{Counter, Counters, Report};
