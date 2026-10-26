//! Batching blip messages into WebSocket frames: a CBOR sequence of whole messages.

use crate::blip::{encode_into, Out};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PushError {
    /// The batch is full: send it, clear it and push again.
    Full,
    /// The message doesn't fit into an empty batch.
    TooLarge,
}

/// A WebSocket frame being filled with messages. Sent when the next message doesn't fit or
/// `flush_after_us` after its first message, whichever is first.
pub struct Batcher<const N: usize> {
    buf: [u8; N],
    len: usize,
    limit: usize,
    messages: u32,
    opened_us: u64,
    flush_after_us: u64,
}

impl<const N: usize> Batcher<N> {
    /// Frames of at most `limit` bytes (at most `N`).
    pub const fn new(limit: usize, flush_after_us: u64) -> Self {
        Self {
            buf: [0; N],
            len: 0,
            limit: if limit < N { limit } else { N },
            messages: 0,
            opened_us: 0,
            flush_after_us,
        }
    }

    pub fn push(&mut self, out: &Out<'_>, now_us: u64) -> Result<(), PushError> {
        match encode_into(out, &mut self.buf[self.len..self.limit]) {
            Some(n) => {
                if self.messages == 0 {
                    self.opened_us = now_us;
                }
                self.len += n;
                self.messages += 1;
                Ok(())
            }
            None if self.len == 0 => Err(PushError::TooLarge),
            None => Err(PushError::Full),
        }
    }

    /// The bytes of the frame.
    pub fn frame(&self) -> &[u8] {
        &self.buf[..self.len]
    }

    pub fn messages(&self) -> u32 {
        self.messages
    }

    pub fn is_empty(&self) -> bool {
        self.messages == 0
    }

    pub fn clear(&mut self) {
        self.len = 0;
        self.messages = 0;
    }

    /// When the frame is due, `None` while it's empty.
    pub fn deadline(&self) -> Option<u64> {
        (self.messages > 0).then(|| self.opened_us + self.flush_after_us)
    }

    pub fn is_due(&self, now_us: u64) -> bool {
        self.deadline().is_some_and(|d| now_us >= d)
    }
}
