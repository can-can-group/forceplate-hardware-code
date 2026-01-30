#!/usr/bin/env python3
"""
Entry point for the Counter Movement Jump (CMJ) test client.
Connects to the force plate via BLE, records data with live visualization,
and runs CMJ analysis on the recorded buffer.
"""

import sys
from pathlib import Path

# Add jumptest_client root so that "import config", "from force_plate import ...", "from ui import ..." work
_root = Path(__file__).resolve().parent
if str(_root) not in sys.path:
    sys.path.insert(0, str(_root))

from ui import CMJRecorderApp


def main() -> None:
    app = CMJRecorderApp()
    app.run()


if __name__ == "__main__":
    main()
