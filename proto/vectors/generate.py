#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.11"
# dependencies = ["protobuf>=5.28"]
# ///
"""Golden vectors of the robby link: Uplink/Downlink messages as raw protobuf (.bin), as
UART frames (.frame: CRC-16 trailer, COBS, 0x00 delimiter) and as their expected decoded
content (.json).

    uv run proto/vectors/generate.py          # writes the vectors next to this script
    uv run proto/vectors/generate.py --check  # fails if the files differ from a new run

Needs `protoc` on the PATH. The message classes are built from the .proto at run time, so
there is no generated Python code to keep in sync.

The .json files show the content as plain JSON: every field that is not in a oneof, also
with its default value; of a oneof only the member that is set; 64-bit integers as
numbers; enums by name.
"""

from __future__ import annotations

import argparse
import json
import random
import subprocess
import sys
import tempfile
from pathlib import Path
from typing import Any

from google.protobuf import descriptor, descriptor_pb2, descriptor_pool, message, message_factory

HERE = Path(__file__).resolve().parent
PROTO_DIR = HERE.parent
PACKAGE = "robby.link"

# Frame limits, see README.md
MAX_PAYLOAD = 1022  # protobuf bytes
MAX_DECODED = MAX_PAYLOAD + 2  # with the CRC: 1 KiB
MAX_WIRE = MAX_DECODED + (MAX_DECODED + 253) // 254 + 1  # COBS overhead + delimiter


def crc16_ccitt_false(data: bytes) -> int:
    """CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, not reflected, no final XOR."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if crc & 0x8000 else crc << 1
            crc &= 0xFFFF
    return crc


def cobs_encode(data: bytes) -> bytes:
    """Canonical COBS, without the 0x00 delimiter: no 0x01 after a final 254-byte run."""
    out = bytearray()
    block = bytearray()
    after_full_run = False
    for byte in data:
        if byte == 0:
            out.append(len(block) + 1)
            out += block
            block.clear()
            after_full_run = False
        else:
            block.append(byte)
            if len(block) == 254:
                out.append(255)
                out += block
                block.clear()
                after_full_run = True
    if block or not after_full_run:
        out.append(len(block) + 1)
        out += block
    return bytes(out)


def cobs_decode(data: bytes) -> bytes:
    """Also accepts a 0x01 after a final 254-byte run."""
    out = bytearray()
    i = 0
    while i < len(data):
        code = data[i]
        if code == 0 or i + code > len(data):
            raise ValueError(f"bad COBS code {code} at {i}")
        out += data[i + 1 : i + code]
        i += code
        if code != 255 and i < len(data):
            out.append(0)
    return bytes(out)


def frame(payload: bytes) -> bytes:
    """Protobuf bytes -> UART frame: CRC-16 big-endian trailer, COBS, 0x00 delimiter."""
    decoded = payload + crc16_ccitt_false(payload).to_bytes(2, "big")
    return cobs_encode(decoded) + b"\x00"


def unframe(wire: bytes) -> bytes:
    if not wire.endswith(b"\x00") or b"\x00" in wire[:-1]:
        raise ValueError("a frame has exactly one 0x00, at its end")
    decoded = cobs_decode(wire[:-1])
    if len(decoded) < 2 or crc16_ccitt_false(decoded) != 0:
        raise ValueError("CRC mismatch")
    return decoded[:-2]


def load_classes() -> dict[str, type[message.Message]]:
    with tempfile.TemporaryDirectory() as tmp:
        out = Path(tmp) / "robby_link.pb"
        subprocess.run(
            # with blip/options.proto and descriptor.proto, which robby_link.proto imports
            ["protoc", f"-I{PROTO_DIR}", "--include_imports", f"--descriptor_set_out={out}", "robby_link.proto"],
            check=True,
        )
        fds = descriptor_pb2.FileDescriptorSet.FromString(out.read_bytes())
    pool = descriptor_pool.DescriptorPool()
    for file in fds.file:
        pool.Add(file)
    link = next(f for f in fds.file if f.name == "robby_link.proto")
    names = [m.name for m in link.message_type]
    return {
        name: message_factory.GetMessageClass(pool.FindMessageTypeByName(f"{PACKAGE}.{name}"))
        for name in names
    }


def to_plain(msg: message.Message) -> dict[str, Any]:
    FD = descriptor.FieldDescriptor
    out: dict[str, Any] = {}
    for field in msg.DESCRIPTOR.fields:
        oneof = field.containing_oneof
        if oneof is not None and msg.WhichOneof(oneof.name) != field.name:
            continue

        def plain(value: Any) -> Any:
            if field.type == FD.TYPE_MESSAGE:
                return to_plain(value)
            if field.type == FD.TYPE_ENUM:
                return field.enum_type.values_by_number[value].name
            return value

        value = getattr(msg, field.name)
        if field.is_repeated:
            out[field.name] = [plain(v) for v in value]
        else:
            out[field.name] = plain(value)
    return out


def vectors(c: dict[str, type[message.Message]]) -> list[tuple[str, str, message.Message]]:
    """(name, description, message) of each vector."""
    U, R, D, I = c["Uplink"], c["Record"], c["Downlink"], c["Invoke"]
    rng = random.Random(806)

    # 100 samples at rest, slightly tilted: z about 1 g (16393 LSB), x/y small, with noise
    imu = c["ImuBatch"](t_first_ns=3_600_123_456_789, period_ns=1_240_694)
    for _ in range(100):
        imu.x.append(-150 + rng.randint(-40, 40))
        imu.y.append(230 + rng.randint(-40, 40))
        imu.z.append(16393 + rng.randint(-60, 60))

    # The largest possible Uplink with one full batch: every field at its widest value
    extremes = c["ImuBatch"](t_first_ns=2**64 - 1, period_ns=2**32 - 1)
    for i in range(100):
        extremes.x.append(-32768 if i % 2 else 32767)
        extremes.y.append(-32768)
        extremes.z.append(-32768 if i % 2 == 0 else 32767)

    # 3 sensors x 8 x 8, a wall ahead of the middle sensor, no target in the top row and
    # at a few scattered zones
    distance = c["DistanceImage"](t_ns=3_600_200_000_000)
    for sensor in range(3):
        for row in range(8):
            for column in range(8):
                if row == 0 or (sensor, row, column) in {(0, 3, 7), (2, 5, 0), (2, 6, 1)}:
                    distance.mm.append(-1)
                else:
                    base = (850, 420, 1200)[sensor]
                    distance.mm.append(base + 15 * row - 7 * column + rng.randint(-5, 5))

    synthetic = c["Synthetic"](t_first_ns=3_600_300_000_000, period_ns=50_000)
    for i in range(50):
        synthetic.x.append(i)
        synthetic.y.append(-i)
        synthetic.z.append(1000 + 10 * i)

    # name, priority, runtime_bp, stack size, stack usage in 0.01 %
    tasks = [
        ("data_publisher", 24, 1234, 4096, 2919),
        ("imu", 40, 860, 2048, 4687),
        ("IDLE", 0, 9655, 512, 5390),
    ]
    status = U(seq=1234)
    status.records.add(
        battery=c["Battery"](
            t_ns=3_601_000_000_000, soc_pct=87, voltage_mv=8012, cell1_mv=4003, cell2_mv=4009,
            current_ma=-350,
        )
    )
    for name, priority, runtime_bp, stack_size, stack_usage in tasks:
        status.records.add(
            task=c["TaskStats"](
                t_ns=3_601_000_000_000, name=name, priority=priority, runtime_bp=runtime_bp,
                stack_size_bytes=stack_size, stack_usage_bp=stack_usage,
            )
        )
    status.records.add(
        link_stats=c["LinkStats"](
            t_ns=3_601_000_000_000, tx_bytes=36_012_345, tx_frames=46_210,
            tx_write_max_us=812, tx_frames_dropped=3, records_dropped=0, rx_frames=12,
            rx_errors=1, rx_seq_gaps=0, baud=1_000_000, tx_bytes_per_s=10_012,
            tx_frames_per_ks=12_833, tx_load_bp=10_012, tx_write_time_bp=37,
        )
    )

    log = U(seq=1235)
    log.records.add(
        log=c["LogRecord"](
            t_ns=3_601_002_000_000, level=c["LogRecord"].DESCRIPTOR.fields_by_name["level"]
            .enum_type.values_by_name["LOG_LEVEL_WARN"].number,
            component_id=2, message="IMU FIFO stalled, reconfiguring",
        )
    )
    log.records.add(action_result=c["ActionResult"](id=1))
    log.records.add(action_result=c["ActionResult"](id=2, error="speed out of range"))

    # driving forward and turning left, the wheels still catching up
    drive = U(seq=4)
    drive.records.add(
        drive=c["Drive"](
            t_ns=3_602_000_000_000, left_target_mm_s=150, right_target_mm_s=250,
            left_speed_mm_s=142, right_speed_mm_s=257, left_duty=478, right_duty=731,
            left_counts=123_456, right_counts=-7,
        )
    )

    # 1.2 m ahead, 0.35 m to the right, facing back and a bit to the left
    # driving along an arc, after a first rest
    imu_pose = U(seq=6)
    imu_pose.records.add(
        imu_pose=c["ImuPose"](
            t_ns=3_602_040_000_000, x_mm=1_190, y_mm=-402, heading_cdeg=-17_640, speed_mm_s=-215,
            stationary=False, ready=True,
        )
    )

    pose = U(seq=5)
    pose.records.add(pose=c["Pose"](t_ns=3_602_020_000_000, x_mm=1_234, y_mm=-350, heading_cdeg=-17_950))

    full_imu = U(seq=1)
    full_imu.records.add(imu=imu)
    worst = U(seq=2**32 - 1)
    worst.records.add(imu=extremes)
    dist = U(seq=2)
    dist.records.add(distance=distance)
    synth = U(seq=3)
    synth.records.add(synthetic=synthetic)

    return [
        ("uplink_empty", "seq 0 (first frame after a reset) without records: 0 protobuf bytes", U()),
        ("uplink_imu_full", "one ImuBatch of 100 samples at rest, at 806 Hz", full_imu),
        ("uplink_imu_worst", "the largest Uplink with one 100-sample batch: every value at its widest", worst),
        ("uplink_distance", "a distance image with invalid zones (-1)", dist),
        ("uplink_status", "the once-per-second records: battery, 3 tasks, link stats", status),
        ("uplink_log", "a log record, an ActionResult ok and one with an error", log),
        ("uplink_synthetic", "a Synthetic batch of 50 samples", synth),
        ("uplink_drive", "a Drive record: the speed loop of both wheels", drive),
        ("uplink_pose", "a Pose record: the position from the wheel encoders", pose),
        ("uplink_imu_pose", "an ImuPose record: the position from the IMU alone", imu_pose),
        ("downlink_motor_drive", "Invoke motor/drive {speed: 200, turn: -50}",
         D(seq=7, invoke=I(id=1, motor_drive=c["MotorDrive"](speed=200, turn=-50)))),
        ("downlink_motor_drive_zero", "Invoke motor/drive {speed: 0, turn: 0}: an empty MotorDrive that is set",
         D(seq=8, invoke=I(id=2, motor_drive=c["MotorDrive"](speed=0, turn=0)))),
        ("downlink_bench_load", "Invoke bench/load {rate: 20000, batch: 100}",
         D(seq=9, invoke=I(id=3, bench_load=c["BenchLoad"](rate=20_000, batch=100)))),
        ("downlink_invoke_no_args", "an Invoke without args: the STM32 answers \"unknown action\"",
         D(seq=10, invoke=I(id=4))),
    ]


def render(classes: dict[str, type[message.Message]]) -> dict[str, bytes]:
    files: dict[str, bytes] = {}
    for name, description, msg in vectors(classes):
        payload = msg.SerializeToString(deterministic=True)
        wire = frame(payload)
        assert len(payload) <= MAX_PAYLOAD, f"{name}: {len(payload)} bytes"
        assert len(wire) <= MAX_WIRE, f"{name}: {len(wire)} bytes on the wire"
        # round trip: frame -> protobuf -> same message
        back = type(msg).FromString(unframe(wire))
        assert back == msg, name
        content = {
            "description": description,
            "type": msg.DESCRIPTOR.full_name,
            "payload_bytes": len(payload),
            "frame_bytes": len(wire),
            "value": to_plain(msg),
        }
        files[f"{name}.bin"] = payload
        files[f"{name}.frame"] = wire
        files[f"{name}.json"] = (json.dumps(content, indent=2) + "\n").encode()
    return files


def self_test() -> None:
    assert crc16_ccitt_false(b"123456789") == 0x29B1
    run = bytes(range(1, 255))  # 254 non-zero bytes
    assert cobs_encode(b"") == b"\x01"
    assert cobs_encode(run) == b"\xff" + run
    assert cobs_encode(b"\x00" + run) == b"\x01\xff" + run
    assert cobs_encode(run + b"\x00") == b"\xff" + run + b"\x01\x01"
    assert cobs_decode(b"\xff" + run + b"\x01") == run
    for data in (b"", b"\x00", b"\x11\x00\x00\x22", run, run + b"\x2a", bytes(range(256)) * 5):
        encoded = cobs_encode(data)
        assert b"\x00" not in encoded and cobs_decode(encoded) == data, data


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--check", action="store_true", help="compare with the files instead of writing them")
    args = parser.parse_args()

    self_test()
    files = render(load_classes())

    if args.check:
        stale = [n for n, data in files.items() if not (HERE / n).is_file() or (HERE / n).read_bytes() != data]
        stale += [p.name for p in HERE.glob("*.*") if p.suffix in {".bin", ".frame", ".json"} and p.name not in files]
        for name in stale:
            print(f"out of date: {name}", file=sys.stderr)
        return 1 if stale else 0

    for name, data in files.items():
        (HERE / name).write_bytes(data)
    for name in files:
        if name.endswith(".json"):
            info = json.loads(files[name])
            print(f"{name[:-5]:28} {info['payload_bytes']:5} B protobuf {info['frame_bytes']:5} B frame")
    return 0


if __name__ == "__main__":
    sys.exit(main())
