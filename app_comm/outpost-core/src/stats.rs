//! The outpost's own counters, published once per second as `outpost/*`.
//!
//! They are atomics, so the UART task (in a higher priority executor) and the WebSocket task
//! can both count. Totals count up from 0 at boot and wrap at 2³², like the counters of the
//! STM32; the bench tooling uses their differences.

use core::sync::atomic::{AtomicU32, Ordering::Relaxed};

use crate::blip::{Out, Path, Val};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(usize)]
pub enum Counter {
    /// Bytes read from the UART
    UartRxBytes,
    /// Frames that passed COBS, CRC and the `seq` check
    UartFrames,
    /// RX errors of the UART (FIFO overflow, framing, parity)
    UartRxErrors,
    /// Frames that failed COBS or were too long
    CobsErrors,
    CrcErrors,
    /// Frames that weren't a valid `Uplink`
    DecodeErrors,
    /// Uplink frames lost, by `seq`
    SeqGaps,
    Stm32Resets,
    /// Uplink frames dropped because the UART→WS queue was full
    QueueDrops,
    /// Most frames ever in the UART→WS queue
    QueueHighWater,
    /// WebSocket payload bytes sent
    WsTxBytes,
    WsFrames,
    /// blip messages sent
    WsMessages,
    /// Longest WebSocket send of the last second in µs
    WsSendMax,
    /// `bench-wifi`: synthetic samples skipped because the WebSocket fell behind
    BenchDrops,
}

/// Path and unit of each [`Counter`], in its order.
pub const COUNTERS: [(&str, &str); 15] = [
    ("outpost/uart_rx_bytes", "B"),
    ("outpost/uart_frames", ""),
    ("outpost/uart_rx_errors", ""),
    ("outpost/cobs_errors", ""),
    ("outpost/crc_errors", ""),
    ("outpost/decode_errors", ""),
    ("outpost/seq_gaps", ""),
    ("outpost/stm32_resets", ""),
    ("outpost/queue_drops", ""),
    ("outpost/queue_high_water", ""),
    ("outpost/ws_tx_bytes", "B"),
    ("outpost/ws_frames", ""),
    ("outpost/ws_messages", ""),
    ("outpost/ws_send_max", "µs"),
    ("outpost/bench_drops", ""),
];

/// The values measured by the firmware, after the counters.
pub const GAUGES: [(&str, &str); 2] = [("outpost/cpu_idle", "%"), ("outpost/rssi", "dBm")];

/// Path and unit of counter or gauge `i`.
pub fn unit(i: usize) -> (&'static str, &'static str) {
    match i.checked_sub(COUNTERS.len()) {
        None => COUNTERS[i],
        Some(g) => GAUGES[g.min(GAUGES.len() - 1)],
    }
}

pub struct Counters([AtomicU32; COUNTERS.len()]);

impl Default for Counters {
    fn default() -> Self {
        Self::new()
    }
}

impl Counters {
    pub const fn new() -> Self {
        Self([const { AtomicU32::new(0) }; COUNTERS.len()])
    }

    #[inline]
    pub fn add(&self, counter: Counter, n: u32) {
        self.0[counter as usize].fetch_add(n, Relaxed);
    }

    #[inline]
    pub fn max(&self, counter: Counter, value: u32) {
        self.0[counter as usize].fetch_max(value, Relaxed);
    }

    pub fn get(&self, counter: Counter) -> u32 {
        self.0[counter as usize].load(Relaxed)
    }

    /// The values for a report; resets the ones that cover only the last second.
    pub fn snapshot(&self) -> [u32; COUNTERS.len()] {
        let mut values = [0; COUNTERS.len()];
        for (value, counter) in values.iter_mut().zip(&self.0) {
            *value = counter.load(Relaxed);
        }
        values[Counter::WsSendMax as usize] = self.0[Counter::WsSendMax as usize].swap(0, Relaxed);
        values
    }
}

/// One report of the counters: a blip `Value` per counter and gauge.
pub struct Report {
    pub timestamp: u64,
    pub values: [u32; COUNTERS.len()],
    /// Idle share of the last second in %, if measured
    pub cpu_idle: Option<f32>,
    /// dBm, if connected
    pub rssi: Option<i32>,
}

impl Report {
    pub const LEN: usize = COUNTERS.len() + GAUGES.len();

    /// Message `i` of [`Self::LEN`].
    pub fn message(&self, i: usize) -> Out<'static> {
        let (path, _) = unit(i);
        let value = match i.checked_sub(COUNTERS.len()) {
            None => Val::Uint(self.values[i].into()),
            Some(0) => self.cpu_idle.map_or(Val::Null, Val::Float),
            Some(_) => self.rssi.map_or(Val::Null, |r| Val::Int(r.into())),
        };
        Out::Value { timestamp: self.timestamp, path: Path::fixed(path), value }
    }
}
