"""Where the fetched assets are (scripts/fetch_assets.sh) and which versions they are."""
from __future__ import annotations

import json
import os
import platform
from importlib import metadata
from pathlib import Path


def assets_dir() -> Path:
    return Path(os.environ.get("G1_ASSETS") or Path.home() / ".cache" / "robolang-g1")


def asset(*parts: str) -> Path:
    """A path below the assets directory; raises a clear error when the fetch script has not run."""
    path = assets_dir().joinpath(*parts)
    if not path.exists():
        raise FileNotFoundError(f"{path} is missing: run backends/unitree_g1/scripts/fetch_assets.sh")
    return path


POLICY_DIR = ("unitree_rl_lab", "deploy", "robots", "g1_29dof", "config", "policy", "velocity", "v0")
HAND_MODEL = ("unitree_rl_gym", "resources", "robots", "g1_description", "g1_29dof_with_hand.xml")


def versions() -> dict:
    """The pinned asset commits (written by the fetch script) plus the versions of what is running now."""
    pins = {}
    try:
        pins = json.loads((assets_dir() / "versions.json").read_text())
    except (OSError, ValueError):
        pass

    def ver(pkg):
        try:
            return metadata.version(pkg)
        except metadata.PackageNotFoundError:
            return None

    return {
        "arch": platform.machine(),
        "python": platform.python_version(),
        "assets": pins.get("assets", {}),
        "packages": {p: ver(p) for p in ("mujoco", "onnxruntime", "numpy", "opencv-python-headless", "cyclonedds",
                                         "unitree_sdk2py")},
    }
