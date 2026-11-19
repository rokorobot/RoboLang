#!/usr/bin/env bash
# Fetch the G1 robot models, the walking policy and the Unitree SDK at pinned commits, build the CycloneDDS C library,
# and install cyclonedds and unitree_sdk2py against it (decision 0046, U10). Idempotent: a step whose result is in
# place is skipped. Prints a versions JSON on stdout (also written to $G1_ASSETS/versions.json); progress goes to stderr.
#
#   G1_ASSETS   where everything goes (default: $HOME/.cache/robolang-g1)
#   PYTHON      the interpreter to install into (default: python3; use a venv)
#   JOBS        parallel build jobs (default: nproc)
#
# Needs: git, cmake, a C compiler, python3-dev. Licences stay with the fetched files (each checkout keeps its
# LICENSE file; the script checks they are there).
set -euo pipefail

G1_ASSETS="${G1_ASSETS:-$HOME/.cache/robolang-g1}"
PYTHON="${PYTHON:-python3}"
JOBS="${JOBS:-$(nproc 2>/dev/null || echo 2)}"
mkdir -p "$G1_ASSETS"
G1_ASSETS="$(cd "$G1_ASSETS" && pwd)"
log() { echo "[fetch_assets] $*" >&2; }

# name | url | full commit SHA | sparse paths (cone mode; root files such as LICENSE always come along)
UNITREE_MUJOCO_SHA=1eb6642e3f3fdfb7fb13a9794fd6a2dd93ea0e7d       # 2026-09-07  BSD-3-Clause  G1 29-DoF MJCF
UNITREE_RL_LAB_SHA=4960b84732b0c2ec593dccbfe963fda1bcd7b1e3        # 2025-11-19  Apache-2.0    whole-body policy
MUJOCO_MENAGERIE_SHA=370c43965323c0944b433b202aac5058b5f907a9      # unitree_g1 only (licence in the folder)
UNITREE_RL_GYM_SHA=276801e46c5d433564f24658bac64f254b7d2d4b        # 2025-07-25  BSD-3-Clause  hand model (U1)
UNITREE_SDK2_PYTHON_SHA=814556d15970dd2ecf1c9984e845ca02ab07e206   # 2026-09-21  BSD-3-Clause  installs as 1.0.1
CYCLONEDDS_SHA=abb064e35b6fbc79554c255ff8b371333e87b029            # eclipse-cyclonedds releases/0.10.x (0.10.5)
CYCLONEDDS_PY=0.10.2                                               # the version unitree_sdk2py requires

fetch() { # name url sha [sparse paths...]
  local name=$1 url=$2 sha=$3 dir="$G1_ASSETS/$1"
  shift 3
  if [ "$(cat "$dir/.fetched_sha" 2>/dev/null)" = "$sha" ]; then log "$name: already at ${sha:0:7}"; return; fi
  log "$name: fetching ${sha:0:7}"
  rm -rf "$dir" && mkdir -p "$dir"
  git -C "$dir" init -q
  git -C "$dir" remote add origin "$url"
  if [ $# -gt 0 ]; then
    git -C "$dir" sparse-checkout init --cone
    git -C "$dir" sparse-checkout set "$@"
    git -C "$dir" fetch -q --depth 1 --filter=blob:none origin "$sha"
  else
    git -C "$dir" fetch -q --depth 1 origin "$sha"
  fi
  git -C "$dir" -c advice.detachedHead=false checkout -q FETCH_HEAD
  [ "$(git -C "$dir" rev-parse HEAD)" = "$sha" ] || { log "$name: checked out the wrong commit"; exit 1; }
  echo "$sha" >"$dir/.fetched_sha"
}

fetch unitree_mujoco https://github.com/unitreerobotics/unitree_mujoco.git "$UNITREE_MUJOCO_SHA" unitree_robots/g1
fetch unitree_rl_lab https://github.com/unitreerobotics/unitree_rl_lab.git "$UNITREE_RL_LAB_SHA" \
  deploy/robots/g1_29dof/config/policy/velocity/v0
fetch mujoco_menagerie https://github.com/google-deepmind/mujoco_menagerie.git "$MUJOCO_MENAGERIE_SHA" unitree_g1
fetch unitree_rl_gym https://github.com/unitreerobotics/unitree_rl_gym.git "$UNITREE_RL_GYM_SHA" resources/robots/g1_description
fetch unitree_sdk2_python https://github.com/unitreerobotics/unitree_sdk2_python.git "$UNITREE_SDK2_PYTHON_SHA"

for f in unitree_mujoco/LICENSE unitree_rl_lab/LICENCE mujoco_menagerie/unitree_g1/LICENSE unitree_rl_gym/LICENSE \
  unitree_sdk2_python/LICENSE; do
  [ -f "$G1_ASSETS/$f" ] || { log "missing licence file $f"; exit 1; }
done

# CycloneDDS C library (the Python binding builds against it; no wheel exists for every Python/arch).
CDDS="$G1_ASSETS/cyclonedds"
if [ "$(cat "$CDDS/.built_sha" 2>/dev/null)" != "$CYCLONEDDS_SHA" ]; then
  log "cyclonedds: building ${CYCLONEDDS_SHA:0:7} (about 2 minutes)"
  src="$G1_ASSETS/cyclonedds-src"
  rm -rf "$src" "$CDDS" && mkdir -p "$src"
  git -C "$src" init -q
  git -C "$src" remote add origin https://github.com/eclipse-cyclonedds/cyclonedds.git
  git -C "$src" fetch -q --depth 1 origin "$CYCLONEDDS_SHA"
  git -C "$src" -c advice.detachedHead=false checkout -q FETCH_HEAD
  cmake -S "$src" -B "$src/build" -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$CDDS" \
    -DBUILD_TESTING=OFF -DBUILD_DOCS=OFF -DENABLE_SECURITY=NO -DENABLE_SSL=NO >&2
  cmake --build "$src/build" --target install -j "$JOBS" >&2
  cp "$src/LICENSE" "$CDDS/LICENSE-cyclonedds" 2>/dev/null || cp "$src"/LICENSE* "$CDDS/" || true
  echo "$CYCLONEDDS_SHA" >"$CDDS/.built_sha"
  rm -rf "$src"
else
  log "cyclonedds: already built (${CYCLONEDDS_SHA:0:7})"
fi
export CYCLONEDDS_HOME="$CDDS"

# Python bindings: cyclonedds from source against that library (--no-binary so a bundled wheel library is never used),
# then the SDK from the pinned checkout. The SDK's setup.py asks for opencv-python (the GUI build); this backend uses the
# headless build, so the requirement is changed to match before installing (the only edit to a fetched file), and the
# SDK is installed with --no-deps.
if ! "$PYTHON" -c "import cyclonedds, unitree_sdk2py" 2>/dev/null; then
  log "installing cyclonedds==$CYCLONEDDS_PY and unitree_sdk2py"
  "$PYTHON" -m pip install -q numpy opencv-python-headless
  "$PYTHON" -m pip install -q --no-binary cyclonedds "cyclonedds==$CYCLONEDDS_PY" >&2
  sed -i 's/"opencv-python"/"opencv-python-headless"/' "$G1_ASSETS/unitree_sdk2_python/setup.py"
  "$PYTHON" -m pip install -q --no-deps "$G1_ASSETS/unitree_sdk2_python" >&2
fi

G1_ASSETS="$G1_ASSETS" PYTHON_BIN="$PYTHON" "$PYTHON" -I - <<'PY' | tee "$G1_ASSETS/versions.json"
import json, os, platform
from importlib import metadata

a = os.environ["G1_ASSETS"]
def sha(name):
    return open(os.path.join(a, name, ".fetched_sha")).read().strip()

def ver(pkg):
    try:
        return metadata.version(pkg)
    except metadata.PackageNotFoundError:
        return None

print(json.dumps({
    "arch": platform.machine(),
    "python": platform.python_version(),
    "assets": {
        "unitree_mujoco": sha("unitree_mujoco"),
        "unitree_rl_lab": sha("unitree_rl_lab"),
        "mujoco_menagerie": sha("mujoco_menagerie"),
        "unitree_rl_gym": sha("unitree_rl_gym"),
        "unitree_sdk2_python": sha("unitree_sdk2_python"),
        "cyclonedds_c": open(os.path.join(a, "cyclonedds", ".built_sha")).read().strip(),
    },
    "python_packages": {p: ver(p) for p in ("cyclonedds", "unitree_sdk2py", "mujoco", "onnxruntime", "numpy")},
}, indent=2))
PY
