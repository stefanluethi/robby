import sys
from pathlib import Path

# outpost.py is a script next to the tests, not a package
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
