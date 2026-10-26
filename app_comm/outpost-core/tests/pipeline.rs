//! Error paths, sequence numbers, batching, counters and the load generator.

mod common;

use common::*;
use micropb::{MessageEncode, PbEncoder};
use outpost_core::blip::{Path, Val};
use outpost_core::framing::{encode_frame, FrameError, MAX_DECODED};
use outpost_core::link::{self, Record, Record_::Kind};
use outpost_core::stats::{Counter, Counters, Report, COUNTERS};
use outpost_core::{
    Batcher, Deframer, Out, PushError, SeqEvent, SeqTracker, SyntheticGen, Translator, UplinkCursor,
};
use proto::{Data, Message};

fn frame_of(payload: &[u8]) -> Vec<u8> {
    let mut buf = vec![0; 1100];
    let n = encode_frame(payload, &mut buf).unwrap();
    buf.truncate(n);
    buf
}

fn uplink(seq: u32, records: Vec<Kind>) -> Vec<u8> {
    let mut msg = link::Uplink { seq, ..Default::default() };
    for kind in records {
        msg.records.push(Record { kind: Some(kind) }).unwrap();
    }
    let mut out = heapless::Vec::<u8, 4096>::new();
    msg.encode(&mut PbEncoder::new(&mut out)).unwrap();
    out.to_vec()
}

#[test]
fn a_stream_of_all_vectors_in_any_chunks() {
    let stream: Vec<u8> = UPLINKS.iter().flat_map(|n| vector(n).frame).collect();
    let expected: Vec<Vec<u8>> = UPLINKS.iter().map(|n| vector(n).bin).collect();
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
fn bad_frames_are_counted_and_the_next_one_decodes() {
    let good = vector("uplink_status");
    let mut wrong_crc = good.bin.clone();
    wrong_crc.extend_from_slice(&[0x12, 0x34]);
    let mut bit_flip = good.frame.clone();
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
    stream.extend(&good.frame);

    assert_eq!(
        deframe(&stream),
        vec![
            Err(FrameError::Crc),
            Err(FrameError::Crc),
            Err(FrameError::Cobs),
            Err(FrameError::TooLong),
            Err(FrameError::Crc),
            Ok(good.bin.clone()),
        ]
    );
}

#[test]
fn resync_drops_a_partial_frame_silently() {
    let v = vector("uplink_distance");
    let mut stream = v.frame[100..].to_vec(); // the end of a frame
    stream.extend(&v.frame);
    let mut deframer = Deframer::new();
    deframer.resync();
    let got: Vec<_> = stream.iter().filter_map(|&b| deframer.push(b).map(|r| r.map(<[u8]>::to_vec))).collect();
    assert_eq!(got, vec![Ok(v.bin)]);
}

#[test]
fn both_cobs_forms_after_a_run_of_254_bytes_decode() {
    // 252 payload bytes + the CRC = one run of 254 non-zero bytes
    let payload = (0..=255u8)
        .map(|seed| (0..252).map(|i| (i as u8).wrapping_mul(7).wrapping_add(seed) | 1).collect::<Vec<u8>>())
        .find(|p| outpost_core::crc::crc16(p).to_be_bytes().iter().all(|&b| b != 0))
        .unwrap();
    let mut decoded = payload.clone();
    decoded.extend(outpost_core::crc::crc16(&payload).to_be_bytes());
    for trailing in [false, true] {
        let mut frame = cobs(&decoded, trailing);
        frame.push(0);
        assert_eq!(deframe(&frame), vec![Ok(payload.clone())], "trailing 0x01: {trailing}");
    }
    // the outpost writes the canonical form
    let mut canonical = cobs(&decoded, false);
    canonical.push(0);
    assert_eq!(frame_of(&payload), canonical);
}

#[test]
fn payloads_that_are_no_uplink_are_decode_errors() {
    // field 2 with 80 bytes, but there are none
    assert!(outpost_core::peek_seq(&[0x12, 0x50]).is_err());
    // field 1 as a length
    assert!(outpost_core::peek_seq(&[0x0a, 0x00]).is_err());
    // field 0
    assert!(outpost_core::peek_seq(&[0x00]).is_err());
    // unknown fields are skipped (a newer STM32)
    assert_eq!(outpost_core::peek_seq(&[0x08, 0x05, 0x18, 0x01]), Ok(5));

    // well-formed on the outside, but the ImuBatch inside is longer than its record
    let bad = [0x08, 0x01, 0x12, 0x02, 0x0a, 0x05];
    assert_eq!(outpost_core::peek_seq(&bad), Ok(1));
    let mut translator = Translator::new();
    let mut cursor = UplinkCursor::new(&bad);
    assert!(cursor.next(&mut translator, 0).is_err());
}

#[test]
fn records_before_a_bad_one_are_kept() {
    let mut payload = uplink(1, vec![Kind::Battery(link::Battery { t_ns: 5, soc_pct: 50, ..Default::default() })]);
    payload.extend([0x12, 0x02, 0x0a, 0x05]);
    let mut translator = Translator::new();
    let mut cursor = UplinkCursor::new(&payload);
    let mut n = 0;
    let err = loop {
        match cursor.next(&mut translator, 0) {
            Ok(Some(_)) => n += 1,
            Ok(None) => break false,
            Err(_) => break true,
        }
    };
    assert!(err);
    assert_eq!(n, 5);
}

#[test]
fn unknown_record_kinds_are_skipped() {
    // a Record with field 14 (not in the schema) between two known ones
    let battery = uplink(0, vec![Kind::Battery(link::Battery { t_ns: 5, ..Default::default() })]);
    let mut payload = battery.clone();
    payload.extend([0x12, 0x02, 0x70, 0x01]);
    payload.extend(&battery);
    let got = messages(&translate(&mut Translator::new(), &payload, 0));
    assert_eq!(got.len(), 10);
}

#[test]
fn seq_gaps_and_resets() {
    let mut t = SeqTracker::new();
    let events: Vec<_> = [7, 8, 11, 12, 0, 1, 1, 2, u32::MAX - 1].iter().map(|&s| t.check(s)).collect();
    assert_eq!(
        events,
        [
            SeqEvent::InOrder, // the first frame the outpost sees
            SeqEvent::InOrder,
            SeqEvent::Gap(2),
            SeqEvent::InOrder,
            SeqEvent::Reset,
            SeqEvent::InOrder,
            SeqEvent::Reset, // went back
            SeqEvent::InOrder,
            SeqEvent::Reset, // more than 2^31 ahead reads as back
        ]
    );
    let mut t = SeqTracker::new();
    assert_eq!(t.check(u32::MAX - 2), SeqEvent::InOrder);
    assert_eq!(t.check(u32::MAX), SeqEvent::Gap(1));
    // a wrap to 0 can't be told from a restart, the README counts it as one
    assert_eq!(t.check(0), SeqEvent::Reset);
    assert_eq!(t.check(5), SeqEvent::Gap(4));
}

#[test]
fn batches_hold_whole_messages_up_to_the_limit() {
    let v = vector("uplink_imu_full");
    let mut translator = Translator::new();
    let mut batcher = Batcher::<4096>::new(4096, 20_000);
    let mut frames: Vec<Vec<u8>> = Vec::new();
    for _ in 0..5 {
        let mut cursor = UplinkCursor::new(&v.bin);
        while let Some(out) = cursor.next(&mut translator, 0).unwrap() {
            match batcher.push(&out, 0) {
                Ok(()) => {}
                Err(PushError::Full) => {
                    frames.push(batcher.frame().to_vec());
                    batcher.clear();
                    batcher.push(&out, 0).unwrap();
                }
                Err(PushError::TooLarge) => panic!(),
            }
        }
    }
    frames.push(batcher.frame().to_vec());
    let total: usize = frames.iter().map(|f| messages(f).len()).sum();
    assert_eq!(total, 500);
    for f in &frames[..frames.len() - 1] {
        assert!(f.len() <= 4096 && f.len() > 4096 - 68, "{}", f.len());
    }
}

#[test]
fn batches_are_due_after_their_deadline() {
    let mut batcher = Batcher::<256>::new(1024, 20_000);
    assert_eq!(batcher.deadline(), None);
    let out = Out::Value { timestamp: 1, path: Path::fixed("a"), value: Val::Uint(1) };
    batcher.push(&out, 1_000).unwrap();
    batcher.push(&out, 15_000).unwrap();
    assert_eq!(batcher.deadline(), Some(21_000));
    assert!(!batcher.is_due(20_999));
    assert!(batcher.is_due(21_000));
    assert_eq!(batcher.messages(), 2);
    batcher.clear();
    assert!(batcher.is_empty() && !batcher.is_due(u64::MAX));
}

#[test]
fn a_message_larger_than_a_batch_is_refused() {
    let mut batcher = Batcher::<64>::new(64, 1);
    assert_eq!(batcher.push(&Out::Actions, 0), Err(PushError::TooLarge));
    assert!(batcher.is_empty());
}

#[test]
fn link_stats_get_rates_from_the_previous_record() {
    let stats = |t_ns: u64, tx_bytes: u32, tx_frames: u32, tx_write_us: u32| {
        Kind::LinkStats(link::LinkStats { t_ns, tx_bytes, tx_frames, tx_write_us, baud: 1_000_000, ..Default::default() })
    };
    let mut translator = Translator::new();
    // the counters wrap in between
    let first = uplink(0, vec![stats(1_000_000_000, u32::MAX - 999, 10, 100)]);
    let second = uplink(1, vec![stats(1_500_000_000, 49_000, 60, 50_100)]);
    assert_eq!(messages(&translate(&mut translator, &first, 0)).len(), 9);
    let got = messages(&translate(&mut translator, &second, 0));
    assert_messages(
        &got[9..],
        &[
            value(1_500_000_000, "link/tx_rate", Data::Float(100_000.0)),
            value(1_500_000_000, "link/tx_frame_rate", Data::Float(100.0)),
            value(1_500_000_000, "link/tx_load", Data::Float(100.0)),
            value(1_500_000_000, "link/tx_write_time", Data::Float(10.0)),
        ],
    );
    // after a restart of the STM32 (time went back): no rates
    let restarted = uplink(0, vec![stats(1_000_000, 10, 1, 1)]);
    assert_eq!(messages(&translate(&mut translator, &restarted, 0)).len(), 9);
}

#[test]
fn unanswered_invocations_time_out_after_a_second() {
    let mut translator = Translator::new();
    let invocation = outpost_core::Invocation { id: 42, action: outpost_core::Action::MotorSpeed { speed: 0 } };
    translator.forward(&invocation, 1_000).unwrap();
    assert_eq!(translator.next_timeout(), Some(1_001_000));
    assert_eq!(translator.expired(1_000_999), None);
    let out = translator.expired(1_001_000).unwrap();
    assert_eq!(
        messages(&encode(&[out])),
        [Message::ActionResult(proto::ActionResult { id: 42, result: Err("no answer from the STM32".to_owned()) })]
    );
    assert_eq!(translator.next_timeout(), None);
    // a late answer isn't passed on
    let late = uplink(0, vec![Kind::ActionResult(link::ActionResult { id: 42, ..Default::default() })]);
    assert!(translate(&mut translator, &late, 0).is_empty());
}

#[test]
fn a_new_client_drops_the_invocations_of_the_one_before() {
    let mut translator = Translator::new();
    let invocation = outpost_core::Invocation { id: 1, action: outpost_core::Action::MotorSpeed { speed: 0 } };
    translator.forward(&invocation, 0).unwrap();
    translator.on_connect();
    assert_eq!(translator.next_timeout(), None);
}

#[test]
fn counters_are_reported_as_outpost_values() {
    let counters = Counters::new();
    counters.add(Counter::UartRxBytes, 1000);
    counters.add(Counter::SeqGaps, 2);
    counters.max(Counter::WsSendMax, 300);
    counters.max(Counter::WsSendMax, 120);
    counters.max(Counter::QueueHighWater, 7);
    let report = Report { timestamp: 99, values: counters.snapshot(), cpu_idle: Some(87.5), rssi: Some(-61) };
    let outs: Vec<_> = (0..Report::LEN).map(|i| report.message(i)).collect();
    let got = messages(&encode(&outs));
    assert_eq!(got.len(), COUNTERS.len() + 2);
    assert!(got.contains(&value(99, "outpost/uart_rx_bytes", Data::Uint(1000))));
    assert!(got.contains(&value(99, "outpost/seq_gaps", Data::Uint(2))));
    assert!(got.contains(&value(99, "outpost/ws_send_max", Data::Uint(300))));
    assert!(got.contains(&value(99, "outpost/queue_high_water", Data::Uint(7))));
    assert!(got.contains(&value(99, "outpost/cpu_idle", Data::Float(87.5))));
    assert!(got.contains(&value(99, "outpost/rssi", Data::Int(-61))));
    // the longest send covers one second, the high water mark all time
    let values = counters.snapshot();
    assert_eq!(values[Counter::WsSendMax as usize], 0);
    assert_eq!(values[Counter::QueueHighWater as usize], 7);
    // gauges without a measurement are null
    let report = Report { cpu_idle: None, rssi: None, ..report };
    let got = messages(&encode(&[report.message(Report::LEN - 1)]));
    assert_eq!(got, [value(99, "outpost/rssi", Data::Null)]);
}

#[test]
fn outpost_time_follows_the_stm32() {
    let mut translator = Translator::new();
    assert_eq!(translator.stm32_time_ns(5_000), 5_000_000);
    let battery = uplink(0, vec![Kind::Battery(link::Battery { t_ns: 7_000_000_000, ..Default::default() })]);
    translate(&mut translator, &battery, 10_000);
    assert_eq!(translator.stm32_time_ns(10_500), 7_000_500_000);
}

#[test]
fn the_load_generator_keeps_its_rate_and_skips_what_falls_behind() {
    let mut synthetic = SyntheticGen::new();
    assert_eq!(synthetic.due(1_000_000, 100_000), (0, 0));
    synthetic.set_rate(1000, 1_000_000);
    assert_eq!(synthetic.due(1_010_000, 100_000), (10, 0));
    let outs: Vec<_> = (0..10).map(|_| synthetic.next_sample()).collect();
    let got = messages(&encode(&outs));
    for (n, m) in got.iter().enumerate() {
        let Message::Value(v) = m else { panic!() };
        assert_eq!(v.path, "bench/synthetic");
        assert_eq!(v.timestamp, 1_000_000_000 + n as u64 * 1_000_000);
        let Data::Record(fields) = &v.value else { panic!() };
        assert_eq!(fields.iter().map(|(k, _)| k.as_str()).collect::<Vec<_>>(), ["x", "y", "z"]);
    }
    // about the size of an IMU sample on WiFi: the path is 8 bytes longer, these timestamps
    // 4 bytes shorter, and a value of 0 takes a half float
    let per_sample = encode(&outs).len() / 10;
    assert!((66..=72).contains(&per_sample), "{per_sample}");
    // 1 s later nothing was sent: all but 100 ms are skipped
    assert_eq!(synthetic.due(2_010_000, 100_000), (100, 900));
    // 10 sent, 900 skipped: the next one is overdue
    assert_eq!(synthetic.next_due_us(), Some(1_911_000));
}
