//! The load generator of the `bench-wifi` firmware: `bench/synthetic` samples like those of
//! the IMU at rest, generated on the ESP32 without the UART.

use crate::blip::Out;
use crate::translate::BENCH_SYNTHETIC;

pub struct SyntheticGen {
    /// Samples per second, 0 = off
    rate: u32,
    start_us: u64,
    /// Samples emitted or skipped since `start_us`
    count: u64,
    rng: u32,
}

impl Default for SyntheticGen {
    fn default() -> Self {
        Self::new()
    }
}

impl SyntheticGen {
    pub const fn new() -> Self {
        Self { rate: 0, start_us: 0, count: 0, rng: 0x1234_5678 }
    }

    pub fn rate(&self) -> u32 {
        self.rate
    }

    pub fn set_rate(&mut self, rate: u32, now_us: u64) {
        self.rate = rate;
        self.restart(now_us);
    }

    /// Starts over without a backlog, e.g. when a client connects.
    pub fn restart(&mut self, now_us: u64) {
        self.start_us = now_us;
        self.count = 0;
    }

    /// The number of samples due now. Samples more than `max_backlog_us` behind are skipped,
    /// returned as the second number.
    pub fn due(&mut self, now_us: u64, max_backlog_us: u64) -> (u32, u32) {
        if self.rate == 0 {
            return (0, 0);
        }
        let rate = u64::from(self.rate);
        let target = now_us.saturating_sub(self.start_us) * rate / 1_000_000;
        let due = target.saturating_sub(self.count);
        let max = (rate * max_backlog_us / 1_000_000).max(1);
        let skipped = due.saturating_sub(max);
        self.count += skipped;
        (due.min(max) as u32, skipped.min(u64::from(u32::MAX)) as u32)
    }

    /// When the next sample is due.
    pub fn next_due_us(&self) -> Option<u64> {
        (self.rate > 0).then(|| self.start_us + (self.count + 1) * 1_000_000 / u64::from(self.rate))
    }

    /// The next sample, timestamped with the ESP32 time it is due at.
    pub fn next_sample(&mut self) -> Out<'static> {
        let rate = u64::from(self.rate.max(1));
        let timestamp = self.start_us * 1000 + self.count * 1_000_000_000 / rate;
        self.count += 1;
        let x = self.noise();
        let y = self.noise();
        // ≈ 1 g on z, like the robot at rest
        let z = 16393 + self.noise();
        Out::Value { timestamp, path: BENCH_SYNTHETIC, value: crate::translate::xyz(x, y, z) }
    }

    /// ±64 LSB of noise, xorshift
    fn noise(&mut self) -> i16 {
        self.rng ^= self.rng << 13;
        self.rng ^= self.rng >> 17;
        self.rng ^= self.rng << 5;
        (self.rng % 129) as i16 - 64
    }
}
