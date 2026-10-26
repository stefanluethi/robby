//! The blip messages the outpost sends, encoded as CBOR with minicbor.
//!
//! Each [`Out`] is one blip `Message` (see blip's `proto` crate). They borrow from the
//! decoded record, so translating needs no buffer of its own: the bytes go straight into
//! the WebSocket frame.

use minicbor::encode::{write::Cursor, Encoder, Write};

pub type Enc<'b> = Encoder<Cursor<&'b mut [u8]>>;
pub type EncResult = Result<(), minicbor::encode::Error<minicbor::encode::write::EndOfSlice>>;

/// A path made of up to three parts, e.g. `tasks/` + the task name + `.runtime`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Path<'a> {
    pub prefix: &'static str,
    pub name: &'a str,
    pub suffix: &'static str,
}

impl Path<'static> {
    pub const fn fixed(path: &'static str) -> Self {
        Self { prefix: path, name: "", suffix: "" }
    }
}

impl Path<'_> {
    fn len(&self) -> usize {
        self.prefix.len() + self.name.len() + self.suffix.len()
    }

    fn encode(&self, enc: &mut Enc<'_>) -> EncResult {
        enc.str_len(self.len() as u64)?;
        let w = enc.writer_mut();
        for part in [self.prefix, self.name, self.suffix] {
            w.write_all(part.as_bytes()).map_err(minicbor::encode::Error::write)?;
        }
        Ok(())
    }
}

/// The value of a blip `Value` message.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Val<'a> {
    /// No value, e.g. a gauge that wasn't measured
    Null,
    Uint(u64),
    Int(i64),
    Float(f32),
    /// A record `{x, y, z}` of floats, e.g. an acceleration in g.
    Xyz([f32; 3]),
    /// The distance image: arrays of 8, `null` for negative values.
    Distance(&'a [i16]),
    /// `tasks/<name>`
    Task { priority: u32, runtime: f32, stack_usage: f32, stack_size: u32 },
}

/// The result of an action.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ActionOutcome<'a> {
    Ok,
    Err(&'a str),
}

/// One blip `Message`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Out<'a> {
    Value { timestamp: u64, path: Path<'a>, value: Val<'a> },
    Metadata { path: Path<'a>, unit: &'a str },
    Log { timestamp: u64, level: u8, component: Component, message: &'a str },
    ActionResult { id: u32, result: ActionOutcome<'a> },
    /// The descriptors of [`crate::actions`].
    Actions,
}

/// The component of a log record: its name, or the number if it has none.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Component {
    Name(&'static str),
    Number(i32),
}

impl Out<'_> {
    pub fn encode(&self, enc: &mut Enc<'_>) -> EncResult {
        enc.map(1)?;
        match self {
            Out::Value { timestamp, path, value } => {
                enc.str("Value")?.map(3)?;
                enc.str("timestamp")?.u64(*timestamp)?;
                enc.str("path")?;
                path.encode(enc)?;
                enc.str("value")?;
                value.encode(enc)
            }
            Out::Metadata { path, unit } => {
                enc.str("ValueMetadata")?.map(2)?;
                enc.str("path")?;
                path.encode(enc)?;
                enc.str("unit")?.str(unit)?;
                Ok(())
            }
            Out::Log { timestamp, level, component, message } => {
                enc.str("Log")?.map(4)?;
                enc.str("timestamp")?.u64(*timestamp)?;
                enc.str("level")?.u8(*level)?;
                enc.str("component")?;
                match component {
                    Component::Name(name) => enc.str(name)?,
                    Component::Number(n) => {
                        let mut buf = [0; 11];
                        enc.str(format_i32(*n, &mut buf))?
                    }
                };
                enc.str("message")?.str(message)?;
                Ok(())
            }
            Out::ActionResult { id, result } => {
                enc.str("ActionResult")?.map(2)?;
                enc.str("id")?.u32(*id)?;
                enc.str("result")?.map(1)?;
                match result {
                    ActionOutcome::Ok => enc.str("Ok")?.null()?,
                    ActionOutcome::Err(reason) => enc.str("Err")?.str(reason)?,
                };
                Ok(())
            }
            Out::Actions => {
                enc.str("Actions")?;
                crate::actions::encode_descriptors(enc)
            }
        }
    }
}

impl Val<'_> {
    fn encode(&self, enc: &mut Enc<'_>) -> EncResult {
        match self {
            Val::Null => {
                enc.null()?;
            }
            Val::Uint(v) => {
                enc.u64(*v)?;
            }
            Val::Int(v) => {
                enc.i64(*v)?;
            }
            Val::Float(v) => float(enc, *v)?,
            Val::Xyz([x, y, z]) => {
                enc.map(3)?;
                enc.str("x")?;
                float(enc, *x)?;
                enc.str("y")?;
                float(enc, *y)?;
                enc.str("z")?;
                float(enc, *z)?;
            }
            Val::Distance(mm) => {
                enc.array(mm.len().div_ceil(8) as u64)?;
                for row in mm.chunks(8) {
                    enc.array(row.len() as u64)?;
                    for &d in row {
                        if d < 0 {
                            enc.null()?;
                        } else {
                            enc.u16(d as u16)?;
                        }
                    }
                }
            }
            Val::Task { priority, runtime, stack_usage, stack_size } => {
                enc.map(4)?;
                enc.str("priority")?.u32(*priority)?;
                enc.str("runtime")?;
                float(enc, *runtime)?;
                enc.str("stack_usage")?;
                float(enc, *stack_usage)?;
                enc.str("stack_size")?.u32(*stack_size)?;
            }
        }
        Ok(())
    }
}

/// A float at the shortest width that keeps it, like blip's `proto` crate (serde_cbor):
/// half if exact, else single. The outpost computes its values in single precision, more
/// than the resolution of any of the sensors.
pub fn float(enc: &mut Enc<'_>, v: f32) -> EncResult {
    if v.is_nan() {
        // the canonical NaN of serde_cbor
        enc.writer_mut().write_all(&[0xf9, 0x7e, 0x00]).map_err(minicbor::encode::Error::write)?;
    } else if v.is_infinite() || half::f16::from_f32(v).to_f32() == v {
        enc.f16(v)?;
    } else {
        enc.f32(v)?;
    }
    Ok(())
}

fn format_i32(n: i32, buf: &mut [u8; 11]) -> &str {
    let mut pos = buf.len();
    let mut v = n.unsigned_abs();
    loop {
        pos -= 1;
        buf[pos] = b'0' + (v % 10) as u8;
        v /= 10;
        if v == 0 {
            break;
        }
    }
    if n < 0 {
        pos -= 1;
        buf[pos] = b'-';
    }
    // only ASCII digits and '-'
    core::str::from_utf8(&buf[pos..]).unwrap_or("?")
}

/// Encodes `out` into `buf`, returns the number of bytes, `None` if it doesn't fit.
pub fn encode_into(out: &Out<'_>, buf: &mut [u8]) -> Option<usize> {
    let mut enc = Encoder::new(Cursor::new(buf));
    out.encode(&mut enc).ok()?;
    Some(enc.writer().position())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn formats_numbers() {
        let mut buf = [0; 11];
        assert_eq!(format_i32(7, &mut buf), "7");
        assert_eq!(format_i32(-123, &mut buf), "-123");
        assert_eq!(format_i32(i32::MIN, &mut buf), "-2147483648");
    }

    #[test]
    fn floats_take_the_shortest_width() {
        let enc = |v: f32| {
            let mut buf = [0; 8];
            let mut e = Encoder::new(Cursor::new(&mut buf[..]));
            float(&mut e, v).unwrap();
            let n = e.writer().position();
            buf[..n].to_vec()
        };
        assert_eq!(enc(1.5), [0xf9, 0x3e, 0x00]);
        assert_eq!(enc(0.0), [0xf9, 0x00, 0x00]);
        assert_eq!(enc(0.1).len(), 5);
        assert_eq!(enc(f32::INFINITY), [0xf9, 0x7c, 0x00]);
        assert_eq!(enc(f32::NAN), [0xf9, 0x7e, 0x00]);
    }
}
