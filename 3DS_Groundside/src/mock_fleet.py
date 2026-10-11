"""Shortcut so the fleet still runs as `python -m src.mock_fleet`.

The code lives in src/mock_drone_fleet/mock_fleet.py.
"""

import sys

from src.mock_drone_fleet.mock_fleet import main

if __name__ == "__main__":
    sys.exit(main())
