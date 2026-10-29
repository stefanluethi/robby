"""Serves blip the traces of robby: connects to the ESP32 bridge and translates the
protobuf messages of the STM32 with the schema in proto/, see tools/outpost/README.md.

    uv run outpost.py                               # the bridge at ws://192.168.1.70/ws/robby
    uv run outpost.py ws://10.0.0.5/ws/robby --record run.blip

Any option of `blip-outpost protobuf` may follow, e.g. --ws-port or -v.
"""

import sys
from pathlib import Path

from blip_outpost.main import main as blip_outpost

PROTO = Path(__file__).resolve().parents[2] / "proto"
DEFAULT_URL = "ws://192.168.1.70/ws/robby"


def arguments(argv: list[str]) -> list[str]:
    """The command line of `blip-outpost protobuf` for robby."""
    url = DEFAULT_URL
    if argv and not argv[0].startswith("-"):
        url, argv = argv[0], argv[1:]
    return [
        "protobuf",
        url,
        "--proto",
        str(PROTO / "robby_link.proto"),
        "-I",
        str(PROTO),
        "--uplink",
        "robby.link.Uplink",
        "--downlink",
        "robby.link.Downlink",
        *argv,
    ]


def main() -> None:
    blip_outpost(arguments(sys.argv[1:]), prog_name="outpost.py")


if __name__ == "__main__":
    main()
