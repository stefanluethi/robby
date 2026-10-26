//! The golden vectors of `proto/vectors` through the outpost, checked with blip's `proto` crate.

mod common;

use common::*;
use micropb::{MessageEncode, PbEncoder};
use outpost_core::actions::{downlink_frame, Action, Invocation};
use outpost_core::framing::encode_frame;
use outpost_core::link::{self, Downlink, Downlink_, Invoke};
use outpost_core::{parse_request, peek_seq, Translator};
use proto::{Data, Message, Request};

#[test]
fn uplink_frames_decode_to_their_payload_and_seq() {
    for name in UPLINKS {
        let v = vector(name);
        let frames = deframe(&v.frame);
        assert_eq!(frames, vec![Ok(v.bin.clone())], "{name}");
        assert_eq!(u64::from(peek_seq(&v.bin).unwrap()), v.json["value"]["seq"].as_u64().unwrap(), "{name}");
    }
}

#[test]
fn the_outpost_frames_like_the_vectors() {
    for name in UPLINKS {
        let v = vector(name);
        let mut buf = [0; outpost_core::framing::MAX_ENCODED];
        let n = encode_frame(&v.bin, &mut buf).unwrap();
        assert_eq!(&buf[..n], &v.frame[..], "{name}");
    }
}

#[test]
fn uplink_records_become_the_blip_messages_of_the_readme() {
    for name in UPLINKS {
        let v = vector(name);
        let mut translator = Translator::new();
        if name == "uplink_log" {
            // the action results answer invocations of this client
            for id in [1, 2] {
                let invocation = Invocation { id, action: Action::MotorSpeed { speed: 1 } };
                translator.forward(&invocation, 0).unwrap();
            }
        }
        let cbor = translate(&mut translator, &v.bin, 0);
        assert_messages(&messages(&cbor), &expected(&v.json));
    }
}

#[test]
fn an_imu_sample_takes_68_bytes_of_cbor() {
    let v = vector("uplink_imu_full");
    let cbor = translate(&mut Translator::new(), &v.bin, 0);
    // {"Value": {"timestamp": u64, "path": "imu/acc", "value": {"x": f32, "y": f32, "z": f32}}},
    // 2 bytes less for each value that is exact as a half float, e.g. 0
    assert!(cbor.len() <= 100 * 68 && cbor.len() >= 100 * 66, "{}", cbor.len());
}

#[test]
fn tasks_get_their_units_once() {
    let v = vector("uplink_status");
    let mut translator = Translator::new();
    translate(&mut translator, &v.bin, 0);
    let again = messages(&translate(&mut translator, &v.bin, 0));
    assert!(!again.iter().any(|m| matches!(m, Message::ValueMetadata(_))));
    // 5 battery, 3 tasks, 9 link totals; no link rates, the time didn't move on
    assert_eq!(again.len(), 5 + 3 + 9);
}

#[test]
fn action_results_go_only_to_the_invocations_waiting_for_them() {
    let v = vector("uplink_log");
    let mut translator = Translator::new();
    let invocation = Invocation { id: 2, action: Action::MotorSpeed { speed: 1 } };
    translator.forward(&invocation, 0).unwrap();
    let got = messages(&translate(&mut translator, &v.bin, 0));
    let results: Vec<_> = got.iter().filter(|m| matches!(m, Message::ActionResult(_))).collect();
    assert_eq!(
        results,
        [&Message::ActionResult(proto::ActionResult { id: 2, result: Err("speed out of range".to_owned()) })]
    );
    // answered once
    let got = messages(&translate(&mut translator, &v.bin, 0));
    assert!(!got.iter().any(|m| matches!(m, Message::ActionResult(_))));
}

fn invoke(id: u32, path: &str, args: Vec<(&str, Data)>) -> Vec<u8> {
    serde_cbor_2::to_vec(&Request::Invoke(proto::Invocation { id, path: path.to_owned(), args: record(args) }))
        .unwrap()
}

#[test]
fn invocations_become_the_downlink_vectors() {
    for (name, request) in [
        ("downlink_motor_speed", invoke(1, "motor/speed", vec![("speed", Data::Uint(500))])),
        ("downlink_motor_speed_zero", invoke(2, "motor/speed", vec![("speed", Data::Uint(0))])),
        (
            "downlink_bench_load",
            invoke(3, "bench/load", vec![("rate", Data::Uint(20000)), ("batch", Data::Uint(100))]),
        ),
    ] {
        let v = vector(name);
        let seq = v.json["value"]["seq"].as_u64().unwrap() as u32;
        let invocation = parse_request(&request).unwrap();
        let frame = downlink_frame(seq, &invocation);
        assert_eq!(&frame[..], &v.frame[..], "{name}");
        assert_eq!(deframe(&frame), vec![Ok(v.bin.clone())], "{name}");
    }
}

#[test]
fn an_invoke_without_args_encodes_like_the_vector() {
    // the outpost never sends one (it answers unknown actions itself), but encodes it the same
    let v = vector("downlink_invoke_no_args");
    let downlink = Downlink { seq: 10, kind: Some(Downlink_::Kind::Invoke(Invoke { id: 4, args: None })) };
    let mut payload = heapless::Vec::<u8, 32>::new();
    downlink.encode(&mut PbEncoder::new(&mut payload)).unwrap();
    assert_eq!(&payload[..], &v.bin[..]);
    let mut buf = [0; 40];
    let n = encode_frame(&payload, &mut buf).unwrap();
    assert_eq!(&buf[..n], &v.frame[..]);
}

#[test]
fn downlink_frames_count_up_from_zero() {
    let mut translator = Translator::new();
    let invocation = parse_request(&invoke(5, "motor/speed", vec![("speed", Data::Uint(500))])).unwrap();
    for seq in 0..3 {
        let frame = translator.forward(&Invocation { id: seq + 10, ..invocation }, 0).unwrap();
        let payload = deframe(&frame).pop().unwrap().unwrap();
        let mut decoded = link::Downlink::default();
        micropb::MessageDecode::decode(&mut decoded, &mut micropb::PbDecoder::new(&payload[..]), payload.len()).unwrap();
        assert_eq!(decoded.seq, seq);
    }
}

#[test]
fn bad_invocations_are_answered_by_the_outpost() {
    use outpost_core::Rejected;
    let answer = |request: Vec<u8>| match parse_request(&request) {
        Err(Rejected::Answer { id, reason }) => (id, reason.as_str().to_owned()),
        other => panic!("{other:?}"),
    };
    assert_eq!(answer(invoke(1, "motor/reverse", vec![])), (1, "unknown action".to_owned()));
    let speed = "speed must be an integer from 0 to 1000".to_owned();
    assert_eq!(answer(invoke(2, "motor/speed", vec![])), (2, speed.clone()));
    assert_eq!(answer(invoke(3, "motor/speed", vec![("speed", Data::Uint(1001))])), (3, speed.clone()));
    assert_eq!(answer(invoke(4, "motor/speed", vec![("speed", Data::Int(-1))])), (4, speed.clone()));
    assert_eq!(answer(invoke(5, "motor/speed", vec![("speed", Data::Float(5.0))])), (5, speed.clone()));
    assert_eq!(answer(invoke(6, "motor/speed", vec![("speed", Data::from("5"))])), (6, speed));
    assert_eq!(
        answer(invoke(7, "bench/load", vec![("rate", Data::Uint(1)), ("batch", Data::Uint(101))])),
        (7, "batch must be an integer from 0 to 100".to_owned())
    );
    // not an Invoke at all: nothing to answer
    assert_eq!(parse_request(&[0xa0]), Err(Rejected::Unreadable));
    assert_eq!(parse_request(b"junk"), Err(Rejected::Unreadable));
}

#[test]
fn optional_args_take_their_default_and_unknown_args_are_ignored() {
    let request = invoke(9, "bench/load", vec![("extra", Data::Bool(true)), ("rate", Data::Uint(5000))]);
    assert_eq!(
        parse_request(&request),
        Ok(Invocation { id: 9, action: Action::BenchLoad { rate: 5000, batch: 0 } })
    );
}

#[test]
fn hello_announces_units_and_actions() {
    let mut translator = Translator::new();
    translate(&mut translator, &vector("uplink_status").bin, 0);
    let outs: Vec<_> = (0..translator.hello_len()).map(|i| translator.hello(i)).collect();
    let got = messages(&encode(&outs));
    for (path, unit) in [
        ("imu/acc.x", "g"),
        ("bench/synthetic.z", "g"),
        ("tof/distance", "mm"),
        ("battery/voltage", "V"),
        ("battery/current", "mA"),
        ("link/tx_load", "%"),
        ("link/tx_write_max", "µs"),
        ("outpost/ws_send_max", "µs"),
        ("outpost/rssi", "dBm"),
        ("outpost/seq_gaps", ""),
        ("tasks/data_publisher.stack_size", "B"),
        ("tasks/IDLE.runtime", "%"),
    ] {
        assert!(got.contains(&metadata(path, unit)), "{path} {unit}");
    }
    let int = |min, max| proto::ParamKind::Int { min: Some(min), max: Some(max) };
    let param = |name: &str, unit: &str, kind, default| proto::Param {
        name: name.to_owned(),
        kind,
        unit: unit.to_owned(),
        default,
    };
    let actions = Message::Actions(vec![
        proto::ActionDescriptor {
            path: "motor/speed".to_owned(),
            description: "Duty cycle of the motor PWM".to_owned(),
            params: vec![param("speed", "‰", int(0, 1000), None)],
            readback: None,
            confirm: false,
            enabled: true,
        },
        proto::ActionDescriptor {
            path: "bench/load".to_owned(),
            description: "Synthetic IMU-like samples on top of the normal stream".to_owned(),
            params: vec![
                param("rate", "samples/s", int(0, 100_000), None),
                param("batch", "samples", int(0, 100), Some(Data::Uint(0))),
            ],
            readback: None,
            confirm: false,
            enabled: true,
        },
    ]);
    assert_eq!(got.last(), Some(&actions));
}
