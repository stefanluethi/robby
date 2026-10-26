"""Runs the simulated bridge `robby_bench.sim`, see `uv run simulate.py --help`."""

import sys

from robby_bench.sim import main

if __name__ == "__main__":
    sys.exit(main())
