"""The golden vectors of proto/vectors through `blip-outpost protobuf` with the annotations of
robby_link.proto: the blip messages of proto/README.md, and the actions and their downlink
frames."""

import json
from pathlib import Path
from typing import Any

import cbor2
import pytest
from blip_outpost import Outpost
from blip_outpost.protobuf import Invoker, Translator
from blip_outpost.schema import Schema

import outpost

PROTO = Path(__file__).resolve().parents[3] / "proto"
VECTORS = PROTO / "vectors"
ACC = 0.000061


@pytest.fixture(scope="module")
def schema() -> Schema:
    return Schema.load([PROTO / "robby_link.proto"], [PROTO])


def vector(name: str) -> tuple[bytes, dict]:
    return (VECTORS / f"{name}.bin").read_bytes(), json.loads(
        (VECTORS / f"{name}.json").read_text()
    )


def translate(schema: Schema, name: str) -> tuple[list[dict], list[tuple[int, str]], dict]:
    """The blip messages of a vector, the action results and its decoded content."""
    sent: list[dict] = []
    results: list[tuple[int, str]] = []
    out = Outpost()
    out._send_frame = lambda frame: sent.append(cbor2.loads(frame))  # type: ignore[method-assign]
    translator = Translator(
        schema, "robby.link.Uplink", out, on_action_result=lambda i, e: results.append((i, e))
    )
    payload, content = vector(name)
    translator.feed(payload)
    assert translator.counters["decode_errors"] == 0
    return sent, results, content["value"]


def values(sent: list[dict]) -> list[tuple[str, Any, int]]:
    return [
        (m["Value"]["path"], m["Value"]["value"], m["Value"]["timestamp"])
        for m in sent
        if "Value" in m
    ]


def units(sent: list[dict]) -> dict[str, str]:
    return {
        m["ValueMetadata"]["path"]: m["ValueMetadata"]["unit"] for m in sent if "ValueMetadata" in m
    }


def xyz(record: dict, i: int) -> dict:
    return {axis: pytest.approx(record[axis][i] * ACC) for axis in "xyz"}


def test_an_empty_uplink_has_no_messages(schema):
    sent, _, _ = translate(schema, "uplink_empty")
    assert sent == []


@pytest.mark.parametrize(
    ("name", "kind", "path"),
    [
        ("uplink_imu_full", "imu", "imu/acc"),
        ("uplink_imu_worst", "imu", "imu/acc"),
        ("uplink_synthetic", "synthetic", "bench/synthetic"),
    ],
)
def test_batches_are_one_record_per_sample_in_g(schema, name, kind, path):
    sent, _, content = translate(schema, name)
    batch = content["records"][0][kind]
    n = len(batch["x"])
    expected = [
        (path, xyz(batch, i), batch["t_first_ns"] + i * batch["period_ns"]) for i in range(n)
    ]
    assert values(sent) == expected
    assert units(sent) == {f"{path}.x": "g", f"{path}.y": "g", f"{path}.z": "g"}


def test_the_distance_image_is_24_rows_of_8_with_null_for_no_target(schema):
    sent, _, content = translate(schema, "uplink_distance")
    image = content["records"][0]["distance"]
    mm = image["mm"]
    rows = [[mm[r * 8 + x] if mm[r * 8 + x] >= 0 else None for x in range(8)] for r in range(24)]
    assert values(sent) == [("tof/distance", rows, image["t_ns"])]
    assert units(sent) == {"tof/distance": "mm"}


def test_the_status_records_are_scaled_with_their_units(schema):
    sent, _, content = translate(schema, "uplink_status")
    records = content["records"]
    b = records[0]["battery"]
    t = b["t_ns"]
    got = {path: (value, ts) for path, value, ts in values(sent)}

    assert got["battery/soc"] == (b["soc_pct"], t)
    assert got["battery/voltage"] == (pytest.approx(b["voltage_mv"] / 1000), t)
    assert got["battery/cell1"] == (pytest.approx(b["cell1_mv"] / 1000), t)
    assert got["battery/cell2"] == (pytest.approx(b["cell2_mv"] / 1000), t)
    assert got["battery/current"] == (b["current_ma"], t)

    for record in records[1:4]:
        task = record["task"]
        assert got[f"tasks/{task['name']}"] == (
            {
                "priority": task["priority"],
                "runtime": pytest.approx(task["runtime_bp"] / 100),
                "stack_size": task["stack_size_bytes"],
                "stack_usage": pytest.approx(task["stack_usage_bp"] / 100),
            },
            task["t_ns"],
        )

    s = records[4]["link_stats"]
    for field in [
        "tx_bytes",
        "tx_frames",
        "tx_frames_dropped",
        "records_dropped",
        "rx_frames",
        "rx_errors",
        "rx_seq_gaps",
        "baud",
    ]:
        assert got[f"link/{field}"] == (s[field], s["t_ns"])
    assert got["link/tx_write_max"] == (s["tx_write_max_us"], s["t_ns"])
    assert got["link/tx_rate"] == (s["tx_bytes_per_s"], s["t_ns"])
    assert got["link/tx_frame_rate"] == (pytest.approx(s["tx_frames_per_ks"] / 1000), s["t_ns"])
    assert got["link/tx_load"] == (pytest.approx(s["tx_load_bp"] / 100), s["t_ns"])
    assert got["link/tx_write_time"] == (pytest.approx(s["tx_write_time_bp"] / 100), s["t_ns"])

    assert units(sent) == {
        "battery/soc": "%",
        "battery/voltage": "V",
        "battery/cell1": "V",
        "battery/cell2": "V",
        "battery/current": "mA",
        **{
            f"tasks/{n}.{f}": u
            for n in ["data_publisher", "imu", "IDLE"]
            for f, u in [("runtime", "%"), ("stack_size", "B"), ("stack_usage", "%")]
        },
        "link/tx_bytes": "B",
        "link/tx_write_max": "µs",
        "link/baud": "Bd",
        "link/tx_rate": "B/s",
        "link/tx_frame_rate": "1/s",
        "link/tx_load": "%",
        "link/tx_write_time": "%",
    }


def test_log_records_and_action_results(schema):
    sent, results, content = translate(schema, "uplink_log")
    log = content["records"][0]["log"]
    assert [m["Log"] for m in sent if "Log" in m] == [
        {
            "timestamp": log["t_ns"],
            "level": 3,
            "component": "imu",
            "message": log["message"],
        }
    ]
    assert results == [(1, ""), (2, "speed out of range")]


def test_the_actions_of_the_readme(schema):
    out = Outpost()
    Invoker(schema, "robby.link.Downlink", send=None).install(out.actions)  # type: ignore[arg-type]
    actions = [
        {k: a[k] for k in ("path", "description", "params")} for a in out.actions.descriptors()
    ]
    assert actions == [
        {
            "path": "motor/speed",
            "description": "Duty cycle of the motor PWM",
            "params": [
                {
                    "name": "speed",
                    "kind": {"Int": {"min": 0, "max": 1000}},
                    "unit": "‰",
                    "default": None,
                },
            ],
        },
        {
            "path": "bench/load",
            "description": "Synthetic IMU-like samples on top of the normal stream",
            "params": [
                {
                    "name": "rate",
                    "kind": {"Int": {"min": 0, "max": 100000}},
                    "unit": "samples/s",
                    "default": None,
                },
                {
                    "name": "batch",
                    "kind": {"Int": {"min": 0, "max": 100}},
                    "unit": "samples",
                    "default": 0,
                },
            ],
        },
    ]


@pytest.mark.parametrize(
    ("name", "path", "args"),
    [
        ("downlink_motor_speed", "motor/speed", {"speed": 500}),
        ("downlink_motor_speed_zero", "motor/speed", {"speed": 0}),
        ("downlink_bench_load", "bench/load", {"rate": 20_000, "batch": 100}),
    ],
)
def test_invocations_encode_like_the_vectors(schema, name, path, args):
    payload, content = vector(name)
    invoker = Invoker(schema, "robby.link.Downlink", send=None)  # type: ignore[arg-type]
    # the vector's seq and id
    invoker._seq = content["value"]["seq"]
    invoker._next_id = content["value"]["invoke"]["id"]
    action = next(a for a in invoker.actions if a.path == path)
    assert invoker.encode(action, args) == (content["value"]["invoke"]["id"], payload)


def test_the_launcher_passes_robbys_schema():
    assert outpost.arguments([]) == [
        "protobuf",
        "ws://192.168.1.70/ws/robby",
        "--proto",
        str(PROTO / "robby_link.proto"),
        "-I",
        str(PROTO),
        "--uplink",
        "robby.link.Uplink",
        "--downlink",
        "robby.link.Downlink",
    ]
    assert outpost.arguments(["ws://x/ws/robby", "--record", "r.blip"])[1] == "ws://x/ws/robby"
    assert outpost.arguments(["-v"])[1:2] == ["ws://192.168.1.70/ws/robby"]
    assert outpost.arguments(["-v"])[-1] == "-v"
