//! Translation speed on the host: frames of a golden vector through the whole outpost path
//! (deframe, seq, decode, CBOR, batching into 4 KiB frames).
//!
//! `cargo run --release --target host-tuple --example bench [vector]`

use std::time::Instant;

use outpost_core::{peek_seq, Batcher, Deframer, PushError, SeqTracker, Translator, UplinkCursor};

fn main() {
    let name = std::env::args().nth(1).unwrap_or_else(|| "uplink_imu_full".to_owned());
    let dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../proto/vectors");
    let frame = std::fs::read(dir.join(format!("{name}.frame"))).unwrap();

    let mut deframer = Deframer::new();
    let mut seq = SeqTracker::new();
    let mut translator = Translator::new();
    let mut batcher = Box::new(Batcher::<4096>::new(4096, 20_000));
    let (mut frames, mut messages, mut ws_bytes, mut ws_frames) = (0u64, 0u64, 0u64, 0u64);
    let start = Instant::now();
    while start.elapsed().as_secs_f64() < 2.0 {
        for _ in 0..1000 {
            for &b in &frame {
                let Some(Ok(payload)) = deframer.push(b) else { continue };
                seq.check(peek_seq(payload).unwrap());
                frames += 1;
                let mut cursor = UplinkCursor::new(payload);
                while let Some(out) = cursor.next(&mut translator, 0).unwrap() {
                    messages += 1;
                    if batcher.push(&out, 0) == Err(PushError::Full) {
                        ws_bytes += batcher.frame().len() as u64;
                        ws_frames += 1;
                        batcher.clear();
                        batcher.push(&out, 0).unwrap();
                    }
                }
            }
        }
    }
    let s = start.elapsed().as_secs_f64();
    println!("{name}: {} B frame on the UART", frame.len());
    println!("  {:.0} frames/s, {:.2} M messages/s", frames as f64 / s, messages as f64 / s / 1e6);
    println!("  {:.0} UART bytes/s in, {:.1} MB/s of CBOR out", (frames * frame.len() as u64) as f64 / s, ws_bytes as f64 / s / 1e6);
    println!("  {:.1} CBOR bytes per message, {:.0} messages per 4 KiB frame", ws_bytes as f64 / messages as f64, messages as f64 / ws_frames as f64);
}
