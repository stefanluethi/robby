//! The actions the outpost announces, and blip's `Invoke` requests as `Downlink` frames, see
//! "Actions" in `proto/README.md`.

use core::fmt::Write as _;

use heapless::{String, Vec};
use micropb::{MessageEncode, PbEncoder};
use minicbor::data::Type;
use minicbor::Decoder;

use crate::blip::{Enc, EncResult};
use crate::link::{BenchLoad, Downlink, Downlink_, Invoke, Invoke_, MotorSpeed};

/// An integer parameter of an action.
pub struct ParamDesc {
    pub name: &'static str,
    pub unit: &'static str,
    pub min: i64,
    pub max: i64,
    /// `Some` for an optional parameter: its value when it's left out.
    pub default: Option<i64>,
}

pub struct ActionDesc {
    pub path: &'static str,
    pub description: &'static str,
    pub params: &'static [ParamDesc],
}

pub const MOTOR_SPEED: &str = "motor/speed";
pub const BENCH_LOAD: &str = "bench/load";

/// The actions of the robot, in the order of the `Actions` message.
pub const ACTIONS: [ActionDesc; 2] = [
    ActionDesc {
        path: MOTOR_SPEED,
        description: "Duty cycle of the motor PWM",
        params: &[ParamDesc { name: "speed", unit: "‰", min: 0, max: 1000, default: None }],
    },
    ActionDesc {
        path: BENCH_LOAD,
        description: "Synthetic IMU-like samples on top of the normal stream",
        params: &[
            ParamDesc { name: "rate", unit: "samples/s", min: 0, max: 100_000, default: None },
            ParamDesc { name: "batch", unit: "samples", min: 0, max: 100, default: Some(0) },
        ],
    },
];

/// The descriptors as the value of blip's `Actions` message.
pub(crate) fn encode_descriptors(enc: &mut Enc<'_>) -> EncResult {
    enc.array(ACTIONS.len() as u64)?;
    for action in &ACTIONS {
        enc.map(3)?;
        enc.str("path")?.str(action.path)?;
        enc.str("description")?.str(action.description)?;
        enc.str("params")?.array(action.params.len() as u64)?;
        for param in action.params {
            enc.map(if param.default.is_some() { 4 } else { 3 })?;
            enc.str("name")?.str(param.name)?;
            enc.str("unit")?.str(param.unit)?;
            enc.str("kind")?.map(1)?.str("Int")?.map(2)?;
            enc.str("min")?.i64(param.min)?;
            enc.str("max")?.i64(param.max)?;
            if let Some(default) = param.default {
                enc.str("default")?.i64(default)?;
            }
        }
    }
    Ok(())
}

/// A checked invocation of one of the [`ACTIONS`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Invocation {
    pub id: u32,
    pub action: Action,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Action {
    MotorSpeed { speed: u32 },
    BenchLoad { rate: u32, batch: u32 },
}

/// Why a request isn't passed on.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Rejected {
    /// Not a blip `Invoke`, nothing to answer.
    Unreadable,
    /// The outpost answers it with `{"Err": reason}`.
    Answer { id: u32, reason: String<64> },
}

/// Reads blip's `{"Invoke": {id, path, args}}` and checks it against the descriptors.
pub fn parse_request(bytes: &[u8]) -> Result<Invocation, Rejected> {
    let raw = read_invoke(bytes).ok_or(Rejected::Unreadable)?;
    let answer = |reason: &str| {
        let mut s = String::new();
        let _ = s.push_str(reason);
        Rejected::Answer { id: raw.id, reason: s }
    };
    let Some(desc) = ACTIONS.iter().find(|a| a.path == raw.path) else {
        return Err(answer("unknown action"));
    };
    let mut values = [0_u32; 2];
    for (param, value) in desc.params.iter().zip(values.iter_mut()) {
        let arg = raw.args.and_then(|pos| find_arg(bytes, pos, param.name));
        let v = match (arg, param.default) {
            (None, Some(default)) => Some(default),
            (Some(Arg::Int(v)), _) if (param.min..=param.max).contains(&v) => Some(v),
            _ => None,
        };
        match v {
            Some(v) => *value = v as u32,
            None => {
                let mut reason = String::new();
                let _ = write!(reason, "{} must be an integer from {} to {}", param.name, param.min, param.max);
                return Err(Rejected::Answer { id: raw.id, reason });
            }
        }
    }
    let action = match desc.path {
        MOTOR_SPEED => Action::MotorSpeed { speed: values[0] },
        _ => Action::BenchLoad { rate: values[0], batch: values[1] },
    };
    Ok(Invocation { id: raw.id, action })
}

struct RawInvoke<'b> {
    id: u32,
    path: &'b str,
    /// Where the args map starts
    args: Option<usize>,
}

fn read_invoke(bytes: &[u8]) -> Option<RawInvoke<'_>> {
    let mut d = Decoder::new(bytes);
    if d.map().ok()? != Some(1) || d.str().ok()? != "Invoke" {
        return None;
    }
    let len = d.map().ok()??;
    let (mut id, mut path, mut args) = (None, None, None);
    for _ in 0..len {
        match d.str().ok()? {
            "id" => id = Some(d.u32().ok()?),
            "path" => path = Some(d.str().ok()?),
            "args" => {
                args = Some(d.position());
                d.skip().ok()?;
            }
            _ => d.skip().ok()?,
        }
    }
    Some(RawInvoke { id: id?, path: path?, args })
}

enum Arg {
    Int(i64),
    Other,
}

fn find_arg(bytes: &[u8], pos: usize, name: &str) -> Option<Arg> {
    let mut d = Decoder::new(bytes);
    d.set_position(pos);
    let len = d.map().ok()??;
    for _ in 0..len {
        let key = d.str().ok()?;
        if key != name {
            d.skip().ok()?;
            continue;
        }
        return Some(match d.datatype().ok()? {
            Type::U8 | Type::U16 | Type::U32 | Type::U64 | Type::I8 | Type::I16 | Type::I32 | Type::I64 => {
                d.i64().map_or(Arg::Other, Arg::Int)
            }
            _ => Arg::Other,
        });
    }
    None
}

/// A `Downlink` frame on the UART: at most 28 bytes of protobuf, CRC, COBS and delimiter.
pub type DownlinkFrame = Vec<u8, 40>;

/// The frame of `Downlink {seq, invoke: {id, <args>}}`.
pub fn downlink_frame(seq: u32, invocation: &Invocation) -> DownlinkFrame {
    let args = match invocation.action {
        Action::MotorSpeed { speed } => Invoke_::Args::MotorSpeed(MotorSpeed { speed }),
        Action::BenchLoad { rate, batch } => Invoke_::Args::BenchLoad(BenchLoad { rate, batch }),
    };
    let downlink = Downlink {
        seq,
        kind: Some(Downlink_::Kind::Invoke(Invoke { id: invocation.id, args: Some(args) })),
    };
    let mut payload = Vec::<u8, 32>::new();
    let mut frame = Vec::new();
    // fits, see the test
    if downlink.encode(&mut PbEncoder::new(&mut payload)).is_ok() {
        let mut buf = [0; 40];
        if let Some(n) = crate::framing::encode_frame(&payload, &mut buf) {
            let _ = frame.extend_from_slice(&buf[..n]);
        }
    }
    frame
}

/// Invocations waiting for the answer of the STM32.
pub struct Pending {
    /// (id, deadline in µs)
    items: Vec<(u32, u64), 8>,
}

impl Pending {
    pub const fn new() -> Self {
        Self { items: Vec::new() }
    }

    pub fn add(&mut self, id: u32, deadline_us: u64) -> bool {
        self.items.push((id, deadline_us)).is_ok()
    }

    /// Whether `id` was waiting; it isn't any more.
    pub fn take(&mut self, id: u32) -> bool {
        match self.items.iter().position(|&(i, _)| i == id) {
            Some(pos) => {
                self.items.remove(pos);
                true
            }
            None => false,
        }
    }

    pub fn expired(&mut self, now_us: u64) -> Option<u32> {
        let pos = self.items.iter().position(|&(_, deadline)| deadline <= now_us)?;
        Some(self.items.remove(pos).0)
    }

    pub fn next_deadline(&self) -> Option<u64> {
        self.items.iter().map(|&(_, d)| d).min()
    }

    pub fn clear(&mut self) {
        self.items.clear();
    }
}

impl Default for Pending {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_downlink_fits_its_buffers() {
        let max = Downlink::MAX_SIZE.unwrap();
        assert!(max <= 32);
        // CRC, one COBS byte (< 254 bytes) and the delimiter
        assert!(max + 4 <= DownlinkFrame::new().capacity());
    }
}
