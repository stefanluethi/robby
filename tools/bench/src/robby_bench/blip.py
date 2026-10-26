"""The parts of the blip wire protocol the bench tools need.

A WebSocket frame to the PC is a CBOR sequence of blip `Message`s, each a map with one key
naming the variant (`Value`, `ValueMetadata`, `Log`, `Actions`, `ActionResult`). A frame to
the target carries one `Request` (`Invoke`). See blip's `doc/source/architecture/protocol.rst`.
"""

from __future__ import annotations

import io
import math
import struct
from dataclasses import dataclass
from typing import Any

import cbor2

MESSAGE_TYPES = ("Value", "ValueMetadata", "Log", "Actions", "ActionResult")


@dataclass(frozen=True)
class Item:
    """One CBOR item of a sequence and the number of bytes it took."""

    obj: Any
    size: int


def decode_sequence(data: bytes) -> tuple[list[Item], str | None]:
    """Decodes a CBOR sequence; returns the whole items and an error if it can't be read to its end.

    As in blip, the items before an error are kept.
    """
    fp = io.BytesIO(data)
    decoder = cbor2.CBORDecoder(fp)
    items: list[Item] = []
    while (start := fp.tell()) < len(data):
        try:
            obj = decoder.decode()
        except (cbor2.CBORDecodeError, EOFError, ValueError) as error:
            return items, f"{type(error).__name__} at byte {start}: {error}"
        items.append(Item(obj, fp.tell() - start))
    return items, None


def message_type(obj: Any) -> str:
    """The variant of a blip message, or "other" for anything else (e.g. a connection name)."""
    if isinstance(obj, dict) and len(obj) == 1:
        (key,) = obj
        if key in MESSAGE_TYPES:
            return str(key)
    return "other"


def value_kind(value: Any) -> str:
    """The kind of a blip `Data` value."""
    if value is None:
        return "null"
    if isinstance(value, bool):
        return "bool"
    if isinstance(value, int):
        return "int"
    if isinstance(value, float):
        return "float"
    if isinstance(value, str):
        return "string"
    if isinstance(value, list):
        return "array"
    if isinstance(value, dict):
        return "record"
    return "invalid"


def scalar_count(value: Any) -> int:
    """Number of scalar leaves in a value, e.g. 3 for a record {x, y, z}; 0 for null."""
    if value is None:
        return 0
    if isinstance(value, list):
        return sum(scalar_count(v) for v in value)
    if isinstance(value, dict):
        return sum(scalar_count(v) for v in value.values())
    return 1


def numeric_leaves(value: Any, prefix: str = "") -> dict[str, float]:
    """The numeric leaves of a value by name, records flattened with dots ("" for a scalar)."""
    if isinstance(value, bool):
        return {prefix: float(value)}
    if isinstance(value, int | float):
        return {prefix: value}
    if isinstance(value, dict):
        out: dict[str, float] = {}
        for key, inner in value.items():
            name = f"{prefix}.{key}" if prefix else str(key)
            out.update(numeric_leaves(inner, name))
        return out
    return {}


def float32(x: float) -> float:
    """Rounds to the nearest float32, which CBOR encodes in 4 bytes like the targets do."""
    if not math.isfinite(x):
        return x
    result: float = struct.unpack("<f", struct.pack("<f", x))[0]
    return result


def encode(obj: Any) -> bytes:
    """CBOR with the shortest float width that keeps the value, as blip's proto crate does."""
    data: bytes = cbor2.dumps(obj, canonical=True)
    return data


def value(timestamp_ns: int, path: str, data: Any) -> dict[str, Any]:
    return {"Value": {"timestamp": timestamp_ns, "path": path, "value": data}}


def value_metadata(path: str, unit: str) -> dict[str, Any]:
    return {"ValueMetadata": {"path": path, "unit": unit}}


def log(
    timestamp_ns: int, level: int, message: str, component: str | None = None
) -> dict[str, Any]:
    body: dict[str, Any] = {"timestamp": timestamp_ns, "level": level, "message": message}
    if component is not None:
        body["component"] = component
    return {"Log": body}


def invoke(request_id: int, path: str, args: dict[str, Any]) -> dict[str, Any]:
    return {"Invoke": {"id": request_id, "path": path, "args": args}}


def action_result(request_id: int, error: str | None = None) -> dict[str, Any]:
    result: dict[str, Any] = {"Ok": None} if error is None else {"Err": error}
    return {"ActionResult": {"id": request_id, "result": result}}
