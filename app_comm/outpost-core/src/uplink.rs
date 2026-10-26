//! Decoding `Uplink` payloads record by record, and the `seq` of each.

use micropb::{MessageDecode, PbDecoder};

use crate::link::{Record, Record_::Kind};
use crate::translate::{Prepared, Translator};
use crate::Out;

/// Why a payload isn't a valid `Uplink`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DecodeError;

impl<E> From<micropb::DecodeError<E>> for DecodeError {
    fn from(_: micropb::DecodeError<E>) -> Self {
        DecodeError
    }
}

const WIRE_VARINT: u8 = 0;
const WIRE_LEN: u8 = 2;

/// The `seq` of an `Uplink` (0 if it isn't set), after checking that the payload is a
/// sequence of well-formed fields. Cheap: the records are skipped, not decoded.
pub fn peek_seq(payload: &[u8]) -> Result<u32, DecodeError> {
    let mut dec = PbDecoder::new(payload);
    let mut seq = 0;
    while dec.bytes_read() < payload.len() {
        let tag = dec.decode_tag()?;
        match (tag.field_num(), tag.wire_type()) {
            (0, _) => return Err(DecodeError),
            (1, WIRE_VARINT) => seq = dec.decode_varint32()?,
            (2, WIRE_LEN) => {
                let len = dec.decode_varint32()? as usize;
                dec.skip_bytes(len)?;
            }
            (1 | 2, _) => return Err(DecodeError),
            (_, wire_type) => dec.skip_wire_value(wire_type)?,
        }
    }
    Ok(seq)
}

/// What the `seq` of a frame says about the frames before it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SeqEvent {
    InOrder,
    /// This many frames were lost.
    Gap(u32),
    /// The STM32 restarted.
    Reset,
}

/// Keeps the `seq` expected next, see "Sequence numbers" in `proto/README.md`.
#[derive(Debug, Default)]
pub struct SeqTracker {
    expected: Option<u32>,
}

impl SeqTracker {
    pub const fn new() -> Self {
        Self { expected: None }
    }

    pub fn check(&mut self, seq: u32) -> SeqEvent {
        let event = match self.expected {
            // the first frame the outpost sees
            None => SeqEvent::InOrder,
            Some(_) if seq == 0 => SeqEvent::Reset,
            Some(expected) => match seq.wrapping_sub(expected) as i32 {
                0 => SeqEvent::InOrder,
                d if d > 0 => SeqEvent::Gap(d as u32),
                _ => SeqEvent::Reset,
            },
        };
        self.expected = Some(seq.wrapping_add(1));
        event
    }
}

/// Walks the records of one `Uplink` payload and hands out their blip messages one at a time,
/// so the caller can send a full WebSocket frame between two messages. One `Record` is
/// decoded at a time (a whole `Uplink` would take ≈ 20 KiB).
pub struct UplinkCursor<'p> {
    payload: &'p [u8],
    dec: PbDecoder<&'p [u8]>,
    record: Record,
    prepared: Prepared,
    /// The next message of `record`
    next: u16,
}

impl<'p> UplinkCursor<'p> {
    pub fn new(payload: &'p [u8]) -> Self {
        Self {
            payload,
            dec: PbDecoder::new(payload),
            record: Record::default(),
            prepared: Prepared::default(),
            next: 0,
        }
    }

    /// The next blip message, `None` at the end of the payload. After an error the messages of
    /// the records before stay sent, the rest of the payload is dropped.
    pub fn next(
        &mut self,
        translator: &mut Translator,
        now_us: u64,
    ) -> Result<Option<Out<'_>>, DecodeError> {
        loop {
            if self.next < self.prepared.count {
                let i = self.next;
                self.next += 1;
                return Ok(Some(crate::translate::message(&self.record, &self.prepared, i)));
            }
            if !self.decode_record()? {
                return Ok(None);
            }
            self.prepared = translator.prepare(&self.record, now_us);
            self.next = 0;
        }
    }

    /// Decodes the next record into `self.record`, false at the end.
    fn decode_record(&mut self) -> Result<bool, DecodeError> {
        while self.dec.bytes_read() < self.payload.len() {
            let tag = self.dec.decode_tag()?;
            match (tag.field_num(), tag.wire_type()) {
                (0, _) => return Err(DecodeError),
                (2, WIRE_LEN) => {
                    // a fresh record: decoding merges into the one that is there
                    self.record.kind = None;
                    self.dec.decode_submessage_delimited(&mut self.record)?;
                    return Ok(true);
                }
                (1, WIRE_VARINT) => {
                    self.dec.decode_varint32()?;
                }
                (1 | 2, _) => return Err(DecodeError),
                (_, wire_type) => self.dec.skip_wire_value(wire_type)?,
            }
        }
        Ok(false)
    }
}

/// Decodes a whole `Uplink` (≈ 20 KiB, for tests and tools only).
pub fn decode_uplink(payload: &[u8]) -> Result<crate::link::Uplink, DecodeError> {
    let mut uplink = crate::link::Uplink::default();
    let mut dec = PbDecoder::new(payload);
    uplink.decode(&mut dec, payload.len())?;
    Ok(uplink)
}

/// The number of samples of an IMU or synthetic batch.
pub(crate) fn samples(kind: &Kind) -> usize {
    match kind {
        Kind::Imu(b) => b.x.len().min(b.y.len()).min(b.z.len()),
        Kind::Synthetic(b) => b.x.len().min(b.y.len()).min(b.z.len()),
        _ => 0,
    }
}
