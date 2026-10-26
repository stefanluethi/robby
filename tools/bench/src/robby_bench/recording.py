"""blip recording files: a gzip-compressed CBOR sequence of a header and the received frames.

The header is `{"format": "blip recording", "version": 1}`, then the bytes of each binary frame
as received. Readers accept several gzip members, uncompressed files and files cut off in the
middle of a message (read up to the last whole message), as blip does.
"""

from __future__ import annotations

import gzip
import zlib
from dataclasses import dataclass, field
from pathlib import Path
from types import TracebackType
from typing import Any, Self

import cbor2

from robby_bench.blip import Item, decode_sequence

FORMAT = "blip recording"
VERSION = 1
GZIP_MAGIC = b"\x1f\x8b"


def header_bytes() -> bytes:
    data: bytes = cbor2.dumps({"format": FORMAT, "version": VERSION})
    return data


class RecordingWriter:
    """Appends frames to a recording file as they arrive."""

    def __init__(self, path: Path) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        self.path = path
        self._file = gzip.open(path, "wb")
        self._file.write(header_bytes())
        self.frames = 0
        self.bytes = 0

    def write(self, frame: bytes) -> None:
        self._file.write(frame)
        self.frames += 1
        self.bytes += len(frame)

    def flush(self) -> None:
        """Makes the frames so far readable, e.g. if the tool is killed later."""
        self._file.flush(zlib.Z_SYNC_FLUSH)

    def close(self) -> None:
        self._file.close()

    def __enter__(self) -> Self:
        return self

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc: BaseException | None,
        tb: TracebackType | None,
    ) -> None:
        self.close()


@dataclass
class Recording:
    header: dict[str, Any]
    items: list[Item] = field(default_factory=list)
    """The items after the header: messages, and connection names (text) of version 2."""
    error: str | None = None
    """Why the file couldn't be read to its end, e.g. it was cut off."""
    compressed: bool = True


def _decompress(data: bytes) -> tuple[bytes, str | None]:
    """Decompresses all gzip members, keeping what was read of a truncated one."""
    out = bytearray()
    rest = data
    while rest:
        decompressor = zlib.decompressobj(wbits=31)
        try:
            out += decompressor.decompress(rest)
        except zlib.error as error:
            return bytes(out), f"gzip: {error}"
        if not decompressor.eof:
            return bytes(out), "gzip stream cut off"
        rest = decompressor.unused_data
        # padding of zeros after the last member is allowed
        if rest.strip(b"\0") == b"":
            break
    return bytes(out), None


def read_recording(path: Path) -> Recording:
    raw = path.read_bytes()
    compressed = raw.startswith(GZIP_MAGIC)
    gzip_error: str | None = None
    if compressed:
        raw, gzip_error = _decompress(raw)
    items, error = decode_sequence(raw)
    if not items or not isinstance(items[0].obj, dict) or items[0].obj.get("format") != FORMAT:
        raise ValueError(f"{path} is not a blip recording")
    header = items[0].obj
    return Recording(
        header=header, items=items[1:], error=error or gzip_error, compressed=compressed
    )
