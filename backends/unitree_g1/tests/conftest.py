"""Make `robolang_g1` importable from a checkout (no install needed) and skip what needs the fetched assets."""
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
SITE = Path(__file__).resolve().parents[1] / "robolang_g1" / "sites" / "phase1.json"


def _assets_missing() -> bool:
    from robolang_g1 import assets

    try:
        assets.asset(*assets.POLICY_DIR)
        assets.asset(*assets.HAND_MODEL)
    except FileNotFoundError:
        return True
    return False


needs_assets = pytest.mark.skipif(_assets_missing(), reason="run backends/unitree_g1/scripts/fetch_assets.sh first")
