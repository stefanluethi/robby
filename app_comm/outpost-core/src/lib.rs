//! The core of the robby outpost on the ESP32: turns the protobuf frames of the STM32 into
//! blip messages and blip's requests into protobuf frames, see `proto/README.md`.
//!
//! No heap and no I/O: the firmware (`app_comm/src/main.rs`) moves the bytes, this crate
//! decides what they become, so it is tested on the host.
//!
//! ```text
//! UART bytes ─ Deframer ─ peek_seq/SeqTracker ─ queue ─ UplinkCursor ─ Out ─ Batcher ─ WebSocket
//! WebSocket ─ parse_request ─ Translator::forward ─ Downlink frame ─ UART
//! ```
#![cfg_attr(not(test), no_std)]

pub mod actions;
pub mod batch;
pub mod blip;
pub mod crc;
pub mod framing;
pub mod stats;
pub mod synthetic;
pub mod translate;
pub mod uplink;

/// The protobuf types generated from `proto/robby_link.proto`.
pub mod link {
    #![allow(clippy::all, nonstandard_style, unused, irrefutable_let_patterns, missing_docs)]
    include!(concat!(env!("OUT_DIR"), "/robby_link.rs"));
    pub use robby_::link_::*;
}

pub use actions::{parse_request, Action, Invocation, Rejected};
pub use batch::{Batcher, PushError};
pub use blip::Out;
pub use framing::{Deframer, FrameError};
pub use stats::{Counter, Counters, Report};
pub use synthetic::SyntheticGen;
pub use translate::Translator;
pub use uplink::{peek_seq, DecodeError, SeqEvent, SeqTracker, UplinkCursor};
