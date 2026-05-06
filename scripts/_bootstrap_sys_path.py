"""Put workspace root on sys.path so `import ml.*` works when running scripts from repo root."""

from __future__ import annotations

import sys
from pathlib import Path

_ROOT = Path(__file__).resolve().parent.parent
_rp = str(_ROOT)
if _rp not in sys.path:
    sys.path.insert(0, _rp)
