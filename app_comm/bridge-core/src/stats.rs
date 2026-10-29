//! The bridge's own counters, sent to the outpost once per second as JSON in a WebSocket
//! text frame, `{"values": {"bridge/uart_rx_bytes": 1234, ...}}`, and their units once per
//! connection, `{"units": {"bridge/uart_rx_bytes": "B", ...}}`.
//!
//! They are atomics, so the UART task (in a higher priority executor) and the WebSocket task
//! can both count. Totals count up from 0 at boot and wrap at 2³², like the counters of the
//! STM32; the bench tooling uses their differences.

use core::fmt::{self, Write};
use core::sync::atomic::{AtomicU32, Ordering::Relaxed};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(usize)]
pub enum Counter {
    /// Bytes read from the UART
    UartRxBytes,
    /// Frames that passed COBS and the CRC
    UartFrames,
    /// RX errors of the UART (FIFO overflow, framing, parity)
    UartRxErrors,
    /// Frames that failed COBS or were too long
    CobsErrors,
    CrcErrors,
    /// Uplink frames dropped because the UART→WebSocket queue was full
    QueueDrops,
    /// Most frames ever in the UART→WebSocket queue
    QueueHighWater,
    /// WebSocket payload bytes sent
    WsTxBytes,
    WsFrames,
    /// Longest WebSocket send of the last second in µs
    WsSendMax,
    /// Downlink frames written to the UART
    DownlinkFrames,
    /// Downlink messages dropped: too long, or the queue to the UART was full
    DownlinkDrops,
}

/// Path and unit of each [`Counter`], in its order.
pub const COUNTERS: [(&str, &str); 12] = [
    ("bridge/uart_rx_bytes", "B"),
    ("bridge/uart_frames", ""),
    ("bridge/uart_rx_errors", ""),
    ("bridge/cobs_errors", ""),
    ("bridge/crc_errors", ""),
    ("bridge/queue_drops", ""),
    ("bridge/queue_high_water", ""),
    ("bridge/ws_tx_bytes", "B"),
    ("bridge/ws_frames", ""),
    ("bridge/ws_send_max", "µs"),
    ("bridge/downlink_frames", ""),
    ("bridge/downlink_drops", ""),
];

/// The values measured by the firmware, after the counters.
pub const GAUGES: [(&str, &str); 2] = [("bridge/cpu_idle", "%"), ("bridge/rssi", "dBm")];

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

/// Writes the units of all counters and gauges as JSON. Counters without a unit are
/// included with an empty one, so the outpost sees every path at once.
pub fn write_units(out: &mut impl Write) -> fmt::Result {
    out.write_str("{\"units\":{")?;
    for (i, (path, unit)) in COUNTERS.iter().chain(GAUGES.iter()).enumerate() {
        if i > 0 {
            out.write_char(',')?;
        }
        write!(out, "\"{path}\":\"{unit}\"")?;
    }
    out.write_str("}}")
}

/// One report of the counters and gauges.
pub struct Report {
    pub values: [u32; COUNTERS.len()],
    /// Idle share of the last second in %, if measured
    pub cpu_idle: Option<f32>,
    /// dBm, if connected
    pub rssi: Option<i32>,
}

impl Report {
    /// Writes the report as JSON. Without a timestamp: the outpost gives it its estimate of
    /// the STM32 time.
    pub fn write_json(&self, out: &mut impl Write) -> fmt::Result {
        out.write_str("{\"values\":{")?;
        for (i, ((path, _), value)) in COUNTERS.iter().zip(self.values).enumerate() {
            if i > 0 {
                out.write_char(',')?;
            }
            write!(out, "\"{path}\":{value}")?;
        }
        write!(out, ",\"{}\":", GAUGES[0].0)?;
        match self.cpu_idle {
            Some(idle) if idle.is_finite() => write!(out, "{idle:.1}")?,
            _ => out.write_str("null")?,
        }
        write!(out, ",\"{}\":", GAUGES[1].0)?;
        match self.rssi {
            Some(rssi) => write!(out, "{rssi}")?,
            None => out.write_str("null")?,
        }
        out.write_str("}}")
    }
}
