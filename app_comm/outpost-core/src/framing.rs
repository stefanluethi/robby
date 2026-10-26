//! The frames on the UART: `COBS(payload ‖ CRC-16 big-endian) ‖ 0x00`, see `proto/README.md`.

use crate::crc;

/// Largest decoded frame: payload and CRC.
pub const MAX_DECODED: usize = 1024;
/// Largest payload (the protobuf bytes).
pub const MAX_PAYLOAD: usize = MAX_DECODED - 2;
/// Largest frame on the wire: COBS adds 1 byte per 254, plus the delimiter.
pub const MAX_ENCODED: usize = MAX_DECODED + MAX_DECODED.div_ceil(254) + 1;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FrameError {
    /// The frame ended in the middle of a COBS block.
    Cobs,
    /// The decoded frame is longer than [`MAX_DECODED`].
    TooLong,
    /// The CRC doesn't match, or the frame is too short to hold one.
    Crc,
}

/// Decodes the bytes of the UART into payloads, one byte at a time, COBS on the fly.
pub struct Deframer {
    buf: [u8; MAX_DECODED],
    len: usize,
    /// Data bytes left in the current COBS block, 0 when the next byte is a code byte.
    block_left: u8,
    /// The code byte of the current block, 0 before the first one of a frame.
    code: u8,
    state: State,
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum State {
    /// Waiting for a delimiter, the bytes before it are a partial frame.
    Unsynced,
    Receiving,
    /// The frame is too long, waiting for its end.
    Overflow,
}

impl Default for Deframer {
    fn default() -> Self {
        Self::new()
    }
}

impl Deframer {
    /// Ready for a frame: the first byte starts one.
    pub const fn new() -> Self {
        Self { buf: [0; MAX_DECODED], len: 0, block_left: 0, code: 0, state: State::Receiving }
    }

    /// Drops the bytes up to the next delimiter without counting them as an error: at start
    /// up and after an RX error, when the first frame may be partial.
    pub fn resync(&mut self) {
        self.reset();
        self.state = State::Unsynced;
    }

    fn reset(&mut self) {
        self.len = 0;
        self.block_left = 0;
        self.code = 0;
        self.state = State::Receiving;
    }

    /// Feeds one byte. At the end of a frame, returns its payload (CRC checked and removed) or
    /// why it was dropped. Empty frames (two delimiters in a row) are skipped.
    #[inline]
    pub fn push(&mut self, byte: u8) -> Option<Result<&[u8], FrameError>> {
        if byte == 0 {
            return self.end_of_frame();
        }
        match self.state {
            State::Unsynced | State::Overflow => {}
            State::Receiving => {
                if self.block_left == 0 {
                    // a code byte: the block before ends with a zero unless it was a full one
                    if self.code != 0 && self.code != 0xFF && !self.put(0) {
                        return None;
                    }
                    self.code = byte;
                    self.block_left = byte - 1;
                } else {
                    if !self.put(byte) {
                        return None;
                    }
                    self.block_left -= 1;
                }
            }
        }
        None
    }

    #[inline]
    fn put(&mut self, byte: u8) -> bool {
        if self.len == MAX_DECODED {
            self.state = State::Overflow;
            return false;
        }
        self.buf[self.len] = byte;
        self.len += 1;
        true
    }

    fn end_of_frame(&mut self) -> Option<Result<&[u8], FrameError>> {
        let state = self.state;
        let (len, block_left, code) = (self.len, self.block_left, self.code);
        self.reset();
        match state {
            State::Unsynced => None,
            State::Overflow => Some(Err(FrameError::TooLong)),
            State::Receiving if code == 0 => None,
            State::Receiving if block_left != 0 => Some(Err(FrameError::Cobs)),
            State::Receiving => {
                if len < 2 || crc::crc16(&self.buf[..len]) != 0 {
                    Some(Err(FrameError::Crc))
                } else {
                    Some(Ok(&self.buf[..len - 2]))
                }
            }
        }
    }
}

/// Encodes `payload` into `out` as a frame: CRC, COBS and the delimiter. Returns the length
/// of the frame, or `None` if `out` is too short or the payload too long.
pub fn encode_frame(payload: &[u8], out: &mut [u8]) -> Option<usize> {
    if payload.len() > MAX_PAYLOAD {
        return None;
    }
    let crc = crc::crc16(payload).to_be_bytes();
    let mut pos = 1; // where the next data byte goes
    let mut code_pos = 0; // where the code of the current block goes
    let mut code = 1_u8;
    // the block before the current one was full (254 data bytes, no zero after it)
    let mut after_full = false;
    for &byte in payload.iter().chain(crc.iter()) {
        after_full = false;
        if byte == 0 {
            *out.get_mut(code_pos)? = code;
            code_pos = pos;
            pos += 1;
            code = 1;
        } else {
            *out.get_mut(pos)? = byte;
            pos += 1;
            code += 1;
            if code == 0xFF {
                *out.get_mut(code_pos)? = code;
                code_pos = pos;
                pos += 1;
                code = 1;
                after_full = true;
            }
        }
    }
    // canonical COBS: no empty block after a final run of 254 bytes
    if after_full {
        pos -= 1;
    } else {
        *out.get_mut(code_pos)? = code;
    }
    *out.get_mut(pos)? = 0;
    Some(pos + 1)
}
