//! Helpers of the host tests: the golden vectors of `proto/vectors`, running frames through
//! the outpost, and reading its CBOR with blip's own `proto` crate.
#![allow(dead_code)]

use std::path::PathBuf;

use outpost_core::{Batcher, Deframer, Out, PushError, Translator, UplinkCursor};
use proto::{Data, Message};
use serde_json::Value as Json;

pub struct Vector {
    pub bin: Vec<u8>,
    pub frame: Vec<u8>,
    pub json: Json,
}

pub fn vector(name: &str) -> Vector {
    let dir = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../proto/vectors");
    let read = |ext: &str| std::fs::read(dir.join(format!("{name}.{ext}"))).unwrap();
    Vector {
        bin: read("bin"),
        frame: read("frame"),
        json: serde_json::from_slice(&read("json")).unwrap(),
    }
}

pub const UPLINKS: [&str; 7] = [
    "uplink_empty",
    "uplink_imu_full",
    "uplink_imu_worst",
    "uplink_distance",
    "uplink_status",
    "uplink_log",
    "uplink_synthetic",
];

/// The payloads of all frames in `bytes`, or why each was dropped.
pub fn deframe(bytes: &[u8]) -> Vec<Result<Vec<u8>, outpost_core::FrameError>> {
    let mut deframer = Deframer::new();
    let mut frames = Vec::new();
    for &b in bytes {
        if let Some(result) = deframer.push(b) {
            frames.push(result.map(<[u8]>::to_vec));
        }
    }
    frames
}

/// The CBOR of all messages of a payload, as the outpost sends them (one big batch).
pub fn translate(translator: &mut Translator, payload: &[u8], now_us: u64) -> Vec<u8> {
    let mut batcher = Box::new(Batcher::<65536>::new(65536, 1000));
    let mut cursor = UplinkCursor::new(payload);
    while let Some(out) = cursor.next(translator, now_us).unwrap() {
        batcher.push(&out, now_us).unwrap();
    }
    batcher.frame().to_vec()
}

/// Encodes messages one after another.
pub fn encode(outs: &[Out<'_>]) -> Vec<u8> {
    let mut batcher = Box::new(Batcher::<65536>::new(65536, 1000));
    for out in outs {
        assert_eq!(batcher.push(out, 0), Ok::<(), PushError>(()));
    }
    batcher.frame().to_vec()
}

/// Reads a CBOR sequence of blip messages with blip's `proto` crate, all bytes must be read.
pub fn messages(cbor: &[u8]) -> Vec<Message> {
    serde_cbor_2::Deserializer::from_slice(cbor)
        .into_iter::<Message>()
        .collect::<Result<_, _>>()
        .unwrap()
}

pub fn record(fields: Vec<(&str, Data)>) -> Data {
    Data::Record(fields.into_iter().map(|(k, v)| (k.to_owned(), v)).collect())
}

pub fn value(timestamp: u64, path: &str, value: Data) -> Message {
    Message::Value(proto::Value { timestamp, path: path.to_owned(), value })
}

pub fn metadata(path: &str, unit: &str) -> Message {
    Message::ValueMetadata(proto::ValueMetadata { path: path.to_owned(), unit: unit.to_owned() })
}

/// Compares message lists, floats within the precision the outpost uses (f32).
pub fn assert_messages(got: &[Message], expected: &[Message]) {
    assert_eq!(got.len(), expected.len(), "number of messages\n got: {got:?}\nwant: {expected:?}");
    for (i, (g, e)) in got.iter().zip(expected).enumerate() {
        let same = match (g, e) {
            (Message::Value(g), Message::Value(e)) => {
                g.timestamp == e.timestamp && g.path == e.path && data_close(&g.value, &e.value)
            }
            _ => g == e,
        };
        assert!(same, "message {i}\n got: {g:?}\nwant: {e:?}");
    }
}

pub fn data_close(got: &Data, expected: &Data) -> bool {
    match (got, expected) {
        (Data::Float(g), Data::Float(e)) => (g - e).abs() <= 1e-6 * e.abs().max(1e-3),
        (Data::Array(g), Data::Array(e)) => g.len() == e.len() && g.iter().zip(e).all(|(g, e)| data_close(g, e)),
        (Data::Record(g), Data::Record(e)) => {
            g.len() == e.len() && g.iter().zip(e).all(|((gk, gv), (ek, ev))| gk == ek && data_close(gv, ev))
        }
        _ => got == expected,
    }
}

fn u(json: &Json, key: &str) -> u64 {
    json[key].as_u64().unwrap_or_else(|| panic!("{key} in {json}"))
}

fn i(json: &Json, key: &str) -> i64 {
    json[key].as_i64().unwrap()
}

fn uint(n: u64) -> Data {
    Data::Uint(n)
}

/// The blip messages `proto/README.md` asks for, computed here in f64 from the `.json` of a
/// vector, for a translator that hasn't seen its tasks yet.
pub fn expected(json: &Json) -> Vec<Message> {
    let mut out = Vec::new();
    for rec in json["value"]["records"].as_array().unwrap() {
        let (kind, r) = rec.as_object().unwrap().iter().next().unwrap();
        match kind.as_str() {
            "imu" | "synthetic" => {
                let path = if kind == "imu" { "imu/acc" } else { "bench/synthetic" };
                let axis = |a: &str| r[a].as_array().unwrap().iter().map(|v| v.as_i64().unwrap()).collect::<Vec<_>>();
                let (x, y, z) = (axis("x"), axis("y"), axis("z"));
                for n in 0..x.len() {
                    let t = u(r, "t_first_ns").wrapping_add(n as u64 * u(r, "period_ns"));
                    let g = |raw: i64| Data::Float(raw as f64 * 0.000061);
                    out.push(value(t, path, record(vec![("x", g(x[n])), ("y", g(y[n])), ("z", g(z[n]))])));
                }
            }
            "distance" => {
                let mm: Vec<i64> = r["mm"].as_array().unwrap().iter().map(|v| v.as_i64().unwrap()).collect();
                assert_eq!(mm.len(), 192);
                let mut rows = Vec::new();
                for s in 0..3 {
                    for x in 0..8 {
                        let row = (0..8)
                            .map(|y| {
                                let d = mm[(s * 8 + x) * 8 + y];
                                if d < 0 { Data::Null } else { uint(d as u64) }
                            })
                            .collect();
                        rows.push(Data::Array(row));
                    }
                }
                out.push(value(u(r, "t_ns"), "tof/distance", Data::Array(rows)));
            }
            "battery" => {
                let t = u(r, "t_ns");
                out.push(value(t, "battery/soc", uint(u(r, "soc_pct"))));
                out.push(value(t, "battery/voltage", Data::Float(u(r, "voltage_mv") as f64 / 1000.0)));
                out.push(value(t, "battery/cell1", Data::Float(u(r, "cell1_mv") as f64 / 1000.0)));
                out.push(value(t, "battery/cell2", Data::Float(u(r, "cell2_mv") as f64 / 1000.0)));
                let current = i(r, "current_ma");
                out.push(value(t, "battery/current", if current < 0 { Data::Int(current) } else { uint(current as u64) }));
            }
            "task" => {
                let name = r["name"].as_str().unwrap();
                let path = format!("tasks/{name}");
                out.push(metadata(&format!("{path}.runtime"), "%"));
                out.push(metadata(&format!("{path}.stack_usage"), "%"));
                out.push(metadata(&format!("{path}.stack_size"), "B"));
                let size = u(r, "stack_size_bytes") as f64;
                let usage = if size == 0.0 { 0.0 } else { 100.0 * (1.0 - u(r, "stack_free_min_bytes") as f64 / size) };
                out.push(value(
                    u(r, "t_ns"),
                    &path,
                    record(vec![
                        ("priority", uint(u(r, "priority"))),
                        ("runtime", Data::Float(u(r, "runtime_bp") as f64 / 100.0)),
                        ("stack_usage", Data::Float(usage)),
                        ("stack_size", uint(u(r, "stack_size_bytes"))),
                    ]),
                ));
            }
            "log" => {
                let level = match r["level"].as_str().unwrap() {
                    "LOG_LEVEL_TRACE" => 0,
                    "LOG_LEVEL_DEBUG" => 1,
                    "LOG_LEVEL_INFO" => 2,
                    "LOG_LEVEL_WARN" => 3,
                    _ => 4,
                };
                let component = match r["component_id"].as_str().unwrap() {
                    "COMPONENT_CMD" => "cmd",
                    "COMPONENT_SPACE" => "space",
                    "COMPONENT_IMU" => "imu",
                    other => panic!("{other}"),
                };
                out.push(Message::Log(proto::LogRecord {
                    timestamp: u(r, "t_ns"),
                    level,
                    component: component.to_owned(),
                    message: r["message"].as_str().unwrap().to_owned(),
                }));
            }
            "action_result" => {
                let error = r["error"].as_str().unwrap();
                out.push(Message::ActionResult(proto::ActionResult {
                    id: u(r, "id") as u32,
                    result: if error.is_empty() { Ok(()) } else { Err(error.to_owned()) },
                }));
            }
            "link_stats" => {
                let t = u(r, "t_ns");
                for (path, key) in [
                    ("link/tx_bytes", "tx_bytes"),
                    ("link/tx_frames", "tx_frames"),
                    ("link/tx_frames_dropped", "tx_frames_dropped"),
                    ("link/records_dropped", "records_dropped"),
                    ("link/rx_frames", "rx_frames"),
                    ("link/rx_errors", "rx_errors"),
                    ("link/rx_seq_gaps", "rx_seq_gaps"),
                    ("link/baud", "baud"),
                    ("link/tx_write_max", "tx_write_max_us"),
                ] {
                    out.push(value(t, path, uint(u(r, key))));
                }
            }
            other => panic!("unknown record {other}"),
        }
    }
    out
}

/// COBS without the delimiter, the 0x01 after a final run of 254 bytes is kept when asked.
pub fn cobs(decoded: &[u8], trailing_empty_block: bool) -> Vec<u8> {
    let mut out = vec![0];
    let mut code_pos = 0;
    let mut code = 1u8;
    let mut after_full = false;
    for &b in decoded {
        after_full = false;
        if b == 0 {
            out[code_pos] = code;
            code_pos = out.len();
            out.push(0);
            code = 1;
        } else {
            out.push(b);
            code += 1;
            if code == 0xFF {
                out[code_pos] = code;
                code_pos = out.len();
                out.push(0);
                code = 1;
                after_full = true;
            }
        }
    }
    if after_full && !trailing_empty_block {
        out.pop();
    } else {
        out[code_pos] = code;
    }
    out
}
