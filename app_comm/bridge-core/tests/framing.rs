//! The frames of the golden vectors (`proto/vectors`) through the bridge, the error paths of
//! the deframer and the JSON of the counters.

use std::path::PathBuf;

use bridge_core::crc::crc16;
use bridge_core::framing::{encode_frame, MAX_DECODED};
use bridge_core::stats::{write_units, COUNTERS, GAUGES};
use bridge_core::{
    frame, Counter, Counters, Deframer, DownlinkFrame, FrameError, Report, MAX_DOWNLINK_PAYLOAD, MAX_ENCODED, MAX_PAYLOAD,
};
use serde_json::Value as Json;

const UPLINKS: [&str; 10] = [
    "uplink_empty",
    "uplink_imu_full",
    "uplink_imu_worst",
    "uplink_distance",
    "uplink_status",
    "uplink_log",
    "uplink_synthetic",
    "uplink_drive",
    "uplink_pose",
    "uplink_imu_pose",
];

const DOWNLINKS: [&str; 4] =
    ["downlink_motor_drive", "downlink_motor_drive_zero", "downlink_bench_load", "downlink_invoke_no_args"];

/// The payload (`.bin`) and the frame on the UART (`.frame`) of a vector.
fn vector(name: &str) -> (Vec<u8>, Vec<u8>) {
    let dir = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../proto/vectors");
    let read = |ext: &str| std::fs::read(dir.join(format!("{name}.{ext}"))).unwrap();
    (read("bin"), read("frame"))
}

/// The payloads of all frames in `bytes`, or why each was dropped.
fn deframe(bytes: &[u8]) -> Vec<Result<Vec<u8>, FrameError>> {
    let mut deframer = Deframer::new();
    bytes.iter().filter_map(|&b| deframer.push(b).map(|r| r.map(<[u8]>::to_vec))).collect()
}

/// COBS without the delimiter, the 0x01 after a final run of 254 bytes is kept when asked.
fn cobs(decoded: &[u8], trailing_empty_block: bool) -> Vec<u8> {
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

#[test]
fn uplink_frames_decode_to_their_payload() {
    for name in UPLINKS {
        let (bin, frame) = vector(name);
        assert_eq!(deframe(&frame), vec![Ok(bin)], "{name}");
    }
}

#[test]
fn downlink_payloads_are_framed_like_the_vectors() {
    for name in DOWNLINKS.iter().chain(UPLINKS.iter()) {
        let (bin, expected) = vector(name);
        assert_eq!(frame::<MAX_ENCODED>(&bin).unwrap().as_slice(), &expected[..], "{name}");
    }
    for name in DOWNLINKS {
        let (bin, expected) = vector(name);
        let framed: DownlinkFrame = frame(&bin).unwrap();
        assert_eq!(framed.as_slice(), &expected[..], "{name}");
    }
}

#[test]
fn a_payload_longer_than_a_frame_is_refused() {
    assert!(frame::<MAX_ENCODED>(&[1; MAX_PAYLOAD]).is_some());
    assert!(frame::<MAX_ENCODED>(&[1; MAX_PAYLOAD + 1]).is_none());
    // the worst case of COBS: no zeros at all
    let downlink: Option<DownlinkFrame> = frame(&[1; MAX_DOWNLINK_PAYLOAD]);
    assert!(downlink.is_some());
    let downlink: Option<DownlinkFrame> = frame(&[1; MAX_DOWNLINK_PAYLOAD + 1]);
    assert!(downlink.is_none());
}

#[test]
fn a_stream_of_all_vectors_in_any_chunks() {
    let stream: Vec<u8> = UPLINKS.iter().flat_map(|n| vector(n).1).collect();
    let expected: Vec<Vec<u8>> = UPLINKS.iter().map(|n| vector(n).0).collect();
    for chunk in [1, 2, 7, 64, 4096] {
        let mut deframer = Deframer::new();
        let mut got = Vec::new();
        for part in stream.chunks(chunk) {
            for &b in part {
                if let Some(r) = deframer.push(b) {
                    got.push(r.unwrap().to_vec());
                }
            }
        }
        assert_eq!(got, expected, "chunks of {chunk}");
    }
}

#[test]
fn bad_frames_are_reported_and_the_next_one_decodes() {
    let (bin, good) = vector("uplink_status");
    let mut wrong_crc = bin.clone();
    wrong_crc.extend_from_slice(&[0x12, 0x34]);
    let mut bit_flip = good.clone();
    bit_flip[40] ^= 0x01; // a data byte (the first COBS block is longer)
    assert_ne!(bit_flip[40], 0);

    let mut stream = Vec::new();
    stream.extend(cobs(&wrong_crc, false));
    stream.push(0);
    stream.extend(&bit_flip);
    stream.extend([0x05, 0x11, 0x22, 0x00]); // a block of 4 bytes cut after 2
    stream.extend(std::iter::repeat_n(0x42, MAX_DECODED + 10)); // too long
    stream.push(0);
    stream.extend([0x01, 0x00]); // decodes to nothing: too short for a CRC
    stream.extend([0x00, 0x00]); // empty frames are skipped
    stream.extend(&good);

    assert_eq!(
        deframe(&stream),
        vec![
            Err(FrameError::Crc),
            Err(FrameError::Crc),
            Err(FrameError::Cobs),
            Err(FrameError::TooLong),
            Err(FrameError::Crc),
            Ok(bin),
        ]
    );
}

#[test]
fn resync_drops_a_partial_frame_silently() {
    let (bin, frame) = vector("uplink_distance");
    let mut stream = frame[100..].to_vec(); // the end of a frame
    stream.extend(&frame);
    let mut deframer = Deframer::new();
    deframer.resync();
    let got: Vec<_> = stream.iter().filter_map(|&b| deframer.push(b).map(|r| r.map(<[u8]>::to_vec))).collect();
    assert_eq!(got, vec![Ok(bin)]);
}

#[test]
fn both_cobs_forms_after_a_run_of_254_bytes_decode() {
    // 252 payload bytes + the CRC = one run of 254 non-zero bytes
    let payload = (0..=255u8)
        .map(|seed| (0..252).map(|i| (i as u8).wrapping_mul(7).wrapping_add(seed) | 1).collect::<Vec<u8>>())
        .find(|p| crc16(p).to_be_bytes().iter().all(|&b| b != 0))
        .unwrap();
    let mut decoded = payload.clone();
    decoded.extend(crc16(&payload).to_be_bytes());
    for trailing in [false, true] {
        let mut frame = cobs(&decoded, trailing);
        frame.push(0);
        assert_eq!(deframe(&frame), vec![Ok(payload.clone())], "trailing 0x01: {trailing}");
    }
    // the bridge writes the canonical form
    let mut canonical = cobs(&decoded, false);
    canonical.push(0);
    let mut buf = vec![0; 1100];
    let n = encode_frame(&payload, &mut buf).unwrap();
    assert_eq!(&buf[..n], &canonical[..]);
}

#[test]
fn the_report_is_json_with_every_counter_and_gauge() {
    let counters = Counters::new();
    counters.add(Counter::UartRxBytes, 1234);
    counters.add(Counter::QueueDrops, 2);
    counters.max(Counter::WsSendMax, 900);
    let report = Report { values: counters.snapshot(), cpu_idle: Some(82.46), rssi: Some(-61) };
    let mut text = String::new();
    report.write_json(&mut text).unwrap();

    let json: Json = serde_json::from_str(&text).unwrap();
    let values = json["values"].as_object().unwrap();
    assert_eq!(values.len(), COUNTERS.len() + GAUGES.len());
    assert_eq!(values["bridge/uart_rx_bytes"], 1234);
    assert_eq!(values["bridge/queue_drops"], 2);
    assert_eq!(values["bridge/ws_send_max"], 900);
    assert_eq!(values["bridge/cpu_idle"], 82.5);
    assert_eq!(values["bridge/rssi"], -61);

    // the longest send covers one second only
    assert_eq!(counters.get(Counter::WsSendMax), 0);
    let mut text = String::new();
    Report { values: counters.snapshot(), cpu_idle: None, rssi: None }.write_json(&mut text).unwrap();
    let json: Json = serde_json::from_str(&text).unwrap();
    assert!(json["values"]["bridge/cpu_idle"].is_null());
    assert!(json["values"]["bridge/rssi"].is_null());
}

#[test]
fn the_units_name_every_counter_and_gauge() {
    let mut text = String::new();
    write_units(&mut text).unwrap();
    let json: Json = serde_json::from_str(&text).unwrap();
    let units = json["units"].as_object().unwrap();
    assert_eq!(units.len(), COUNTERS.len() + GAUGES.len());
    assert_eq!(units["bridge/ws_send_max"], "µs");
    assert_eq!(units["bridge/uart_frames"], "");
}

#[test]
fn the_report_fits_a_websocket_frame_of_the_bridge() {
    let report = Report { values: [u32::MAX; COUNTERS.len()], cpu_idle: Some(100.0), rssi: Some(-100) };
    let mut text = String::new();
    report.write_json(&mut text).unwrap();
    let mut units = String::new();
    write_units(&mut units).unwrap();
    // the firmware writes them into a heapless::String of 1 KiB
    assert!(text.len() < 1024 && units.len() < 1024, "{} {}", text.len(), units.len());
}
