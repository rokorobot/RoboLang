"""
Entry point for the read-only RoboLang commands.

    python -m robolang validate FILE.rob      (with src/ on the import path)
    python src/robolang validate FILE.rob     (straight from a checkout, nothing installed)
"""

import sys

if __package__ in (None, ""):
    # Started as `python src/robolang`: the package directory itself is on
    # sys.path. Replace it with its parent so `robolang` imports as a package.
    from pathlib import Path

    package_dir = Path(__file__).resolve().parent
    sys.path[:] = [p for p in sys.path if p and Path(p).resolve() != package_dir]
    sys.path.insert(0, str(package_dir.parent))

from robolang.cli import run

if __name__ == "__main__":
    run()
