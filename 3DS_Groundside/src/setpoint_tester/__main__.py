"""Entry point: uv run python -m src.setpoint_tester --host <drone-ip>"""

import sys

from src.setpoint_tester.cli import main

sys.exit(main())
