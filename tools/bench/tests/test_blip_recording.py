from __future__ import annotations

import gzip
from pathlib import Path

import cbor2
import pytest

from robby_bench import blip
from robby_bench.recording import RecordingWriter, header_bytes, read_recording

T = 1_793_045_673_056_748_667


def test_sonde_value_is_53_bytes() -> None:
    # the size the plan estimates for one axis value of today's firmware
    msg = blip.value(T, "imu/acc/x", blip.float32(0.123))
    assert len(blip.encode(msg)) == 53


def test_decode_sequence_sizes_and_partial_frame() -> None:
    a = blip.encode(blip.value(T, "a", 1))
    b = blip.encode(blip.value_metadata("a", "V"))
    items, error = blip.decode_sequence(a + b)
    assert error is None
    assert [i.size for i in items] == [len(a), len(b)]
    assert blip.message_type(items[1].obj) == "ValueMetadata"

    items, error = blip.decode_sequence(a + b[:-3])
    assert [i.size for i in items] == [len(a)]
    assert error is not None


@pytest.mark.parametrize(
    ("value", "kind", "scalars"),
    [
        (1, "int", 1),
        (1.5, "float", 1),
        (True, "bool", 1),
        (None, "null", 0),
        ({"x": 1.0, "y": 2.0, "z": 3.0}, "record", 3),
        ([[1, 2], [3, 4]], "array", 4),
        ("text", "string", 1),
    ],
)
def test_value_kind_and_scalars(value: object, kind: str, scalars: int) -> None:
    assert blip.value_kind(value) == kind
    assert blip.scalar_count(value) == scalars


def test_numeric_leaves_flattens_records() -> None:
    assert blip.numeric_leaves({"a": 1, "b": {"c": 2.5}, "s": "x"}) == {"a": 1, "b.c": 2.5}
    assert blip.numeric_leaves(7) == {"": 7}


def frames() -> list[bytes]:
    return [
        blip.encode(blip.value(T, "imu/acc/x", 0.5)) + blip.encode(blip.value(T, "imu/acc/y", 1.0)),
        blip.encode(blip.log(T, 3, "hello", "main")),
        blip.encode({"Actions": []}),
    ]


def test_recording_round_trip(tmp_path: Path) -> None:
    path = tmp_path / "r.blip"
    with RecordingWriter(path) as rec:
        for f in frames():
            rec.write(f)
    assert path.read_bytes()[:2] == b"\x1f\x8b"
    r = read_recording(path)
    assert r.header == {"format": "blip recording", "version": 1}
    assert r.error is None
    assert b"".join(blip.encode(i.obj) for i in r.items) == b"".join(frames())
    assert [blip.message_type(i.obj) for i in r.items] == ["Value", "Value", "Log", "Actions"]


def test_recording_several_gzip_members(tmp_path: Path) -> None:
    path = tmp_path / "r.blip"
    a, b, c = frames()
    path.write_bytes(gzip.compress(header_bytes() + a) + gzip.compress(b + c))
    assert len(read_recording(path).items) == 4


def test_recording_uncompressed_and_v2_names(tmp_path: Path) -> None:
    path = tmp_path / "r.blip"
    header = cbor2.dumps({"format": "blip recording", "version": 2})
    path.write_bytes(header + cbor2.dumps("robby") + b"".join(frames()))
    r = read_recording(path)
    assert not r.compressed
    assert r.items[0].obj == "robby"
    assert len(r.items) == 5


def test_recording_cut_off_reads_whole_messages(tmp_path: Path) -> None:
    path = tmp_path / "r.blip"
    data = gzip.compress(header_bytes() + b"".join(frames()))
    path.write_bytes(data[: len(data) - 12])  # loses the gzip trailer and some data
    r = read_recording(path)
    assert r.error is not None
    assert 1 <= len(r.items) < 4


def test_not_a_recording(tmp_path: Path) -> None:
    path = tmp_path / "x.blip"
    path.write_bytes(cbor2.dumps({"format": "other"}))
    with pytest.raises(ValueError, match="not a blip recording"):
        read_recording(path)
