# Measurement only (g1-phase4-acceptance.md, F2). Needs the backend installed; writes under /tmp/f2tools.
"""robolang_g1.cli with the sidesteps switched off (and, with NO_FIX=1, the walk back to the approach point too)."""

import os
import sys
from robolang_g1 import ops, cli

ops.SIDESTEPS = 0
if os.environ.get("NO_FIX"):
    ops.APPROACH_AGAIN_M = 99.0
sys.exit(cli.main(sys.argv[1:]))
