"""
Packaging smoke test for RoboLang.

Builds the wheel and the source distribution, installs each one into its own
fresh virtual environment with plain `pip install <file>` (dependencies are
resolved normally), and runs the installed `robolang` command from a directory
outside the repository.

    python tools/smoke_install.py

Needs the `build` package and network access (pip fetches the build backend).
Exits non-zero on the first failed check.
"""

from __future__ import annotations
import json
import os
import shutil
import subprocess
import sys
import tempfile
import tomllib
import venv
import zipfile
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
EXAMPLE = REPO / "examples" / "pick_and_place.rob"
INVALID = REPO / "tests" / "fixtures" / "invalid" / "e303_undeclared_name.rob"
SNAPSHOT = REPO / "tests" / "snapshots" / "ir" / "multiple_tasks.json"
MULTI = REPO / "tests" / "fixtures" / "valid" / "multiple_tasks.rob"
FORBIDDEN = ("rclpy", "control_msgs", "trajectory_msgs", "std_msgs")
# The v1 prototype: repository source only, never part of an installation.
LEGACY = ("robolang_runtime", "robolang_adapter", "robolang_adapter_ros2", "robolang_ros2_main")
ROS_STUBS = ("rclpy", "rclpy.node", "rclpy.action", "trajectory_msgs.msg", "control_msgs.action", "std_msgs.msg")
VERSION = tomllib.loads((REPO / "pyproject.toml").read_text(encoding="utf-8"))["project"]["version"]


def run(command, cwd, check=True, env=None) -> subprocess.CompletedProcess:
    result = subprocess.run([str(c) for c in command], cwd=cwd, capture_output=True, env=env)
    if check and result.returncode != 0:
        sys.stdout.write(result.stdout.decode(errors="replace"))
        sys.stderr.write(result.stderr.decode(errors="replace"))
        raise SystemExit(f"FAILED ({result.returncode}): {' '.join(str(c) for c in command)}")
    return result


def expect(condition: bool, what: str):
    print(f"    {'ok  ' if condition else 'FAIL'}  {what}")
    if not condition:
        raise SystemExit(f"FAILED: {what}")


def clean_env() -> dict:
    """Environment in which nothing points back at the checkout."""
    env = {k: v for k, v in os.environ.items() if k not in ("PYTHONPATH", "PYTHONHOME", "VIRTUAL_ENV")}
    env["PYTHONNOUSERSITE"] = "1"
    return env


def build(dist: Path) -> tuple[Path, Path]:
    print("build: wheel and source distribution")
    run([sys.executable, "-m", "build", "--outdir", dist, REPO], cwd=REPO)
    (wheel,) = dist.glob("*.whl")
    (sdist,) = dist.glob("*.tar.gz")
    print(f"    {wheel.name}\n    {sdist.name}")

    with zipfile.ZipFile(wheel) as archive:
        names = archive.namelist()
        metadata = archive.read(next(n for n in names if n.endswith(".dist-info/METADATA"))).decode()
        entry_points = archive.read(next(n for n in names if n.endswith(".dist-info/entry_points.txt"))).decode()
    headers = metadata.split("\n\n", 1)[0].splitlines()  # the README follows the header block
    requires = [line for line in headers if line.startswith("Requires-Dist:")]
    runtime = [line for line in requires if "extra ==" not in line]
    expect(runtime == [], "wheel declares no runtime dependencies")
    expect(
        not any(name in line for line in requires for name in FORBIDDEN),
        "no dependency, required or optional, names a ROS 2 package",
    )
    expect(f"Version: {VERSION}" in headers, f"wheel version is {VERSION}")
    expect("Requires-Python: >=3.11" in headers, "wheel requires Python 3.11 or newer")
    expect("robolang/cli.py" in names and "robolang/ir.py" in names, "wheel contains the robolang package")
    expect(not any(f"{m}.py" in names for m in LEGACY), "wheel contains none of the four legacy modules")
    code = sorted(n for n in names if ".dist-info/" not in n)
    expect(all(n.startswith("robolang/") for n in code), f"wheel code is only the robolang package ({len(code)} files)")
    section = entry_points.split("[console_scripts]", 1)[1].split("[", 1)[0]
    scripts = [line.strip() for line in section.splitlines() if line.strip()]
    expect(scripts == ["robolang = robolang.cli:run"], f"the only console script is robolang ({scripts})")
    expect("ros2_runtime" not in entry_points, "wheel defines no ros2_runtime entry point")
    return wheel, sdist


def check_install(artifact: Path, work: Path):
    print(f"install: {artifact.name}")
    env_dir = work / "venv"
    venv.EnvBuilder(with_pip=True, clear=True).create(env_dir)
    bin_dir = env_dir / ("Scripts" if os.name == "nt" else "bin")
    python = bin_dir / ("python.exe" if os.name == "nt" else "python")
    robolang = bin_dir / ("robolang.exe" if os.name == "nt" else "robolang")
    outside = work / "elsewhere"
    outside.mkdir()
    env = clean_env()

    # Plain install: no --no-deps, so an undeclared or unavailable dependency would fail here.
    run([python, "-m", "pip", "install", "--disable-pip-version-check", artifact], cwd=outside, env=env)
    run([python, "-m", "pip", "check"], cwd=outside, env=env)

    installed = run([python, "-m", "pip", "list", "--format=json"], cwd=outside, env=env).stdout
    names = {package["name"].lower() for package in json.loads(installed)}
    expect("robolang" in names, "robolang is installed")
    expect(not names & set(FORBIDDEN), "no ROS 2 package was installed")
    expect(names <= {"robolang", "pip", "setuptools", "wheel"}, f"nothing else was installed ({sorted(names)})")

    location = run(
        [python, "-c", "import robolang, robolang.cli; print(robolang.__file__)"], cwd=outside, env=env
    ).stdout.decode().strip()
    expect(env_dir.resolve() in Path(location).resolve().parents, "the package is imported from the environment")
    expect(REPO.resolve() not in Path(location).resolve().parents, "the package is not imported from the checkout")
    expect(robolang.exists(), "the robolang command exists")
    commands = sorted(p.stem for p in bin_dir.iterdir() if p.stem.lower().startswith(("robolang", "ros2")))
    expect(commands == ["robolang"], f"robolang is the only RoboLang command installed ({commands})")
    expect(not list(bin_dir.glob("ros2_runtime*")), "no ros2_runtime command is installed")
    help_text = run([robolang, "--help"], cwd=outside, env=env).stdout.decode()
    expect("validate" in help_text and "inspect" in help_text, "the command offers validate and inspect")
    for word in ("run", "execute", "ros2_runtime"):
        attempt = run([robolang, word, EXAMPLE], cwd=outside, env=env, check=False)
        expect(attempt.returncode == 2, f"'robolang {word}' is not a command (exit 2)")

    version = run([robolang, "--version"], cwd=outside, env=env).stdout.decode().strip()
    expect(version == f"robolang {VERSION} (language 1.1, IR schema 1.0)", f"--version: {version}")

    valid = run([robolang, "validate", EXAMPLE], cwd=outside, env=env)
    expect(b"valid RoboLang 1.1 (1 task, 1 warning)" in valid.stdout, "validate accepts the example")
    expect(b"warning[W401]" in valid.stderr, "validate reports the deprecation warning")

    invalid = run([robolang, "validate", INVALID], cwd=outside, env=env, check=False)
    expect(invalid.returncode == 1 and b"error[E303]" in invalid.stderr, "validate rejects an invalid file with exit 1")

    missing = run([robolang, "validate", "missing.rob"], cwd=outside, env=env, check=False)
    expect(missing.returncode == 2, "a missing file gives exit 2")

    text = run([robolang, "inspect", EXAMPLE], cwd=outside, env=env).stdout
    expect(b"task demo_pick_and_place" in text and b"Nothing was executed." in text, "inspect prints the summary")

    document = run([robolang, "inspect", MULTI, "--json"], cwd=outside, env=env).stdout
    expect(document == SNAPSHOT.read_text(encoding="ascii").encode("ascii"), "inspect --json matches the IR snapshot")

    world = REPO / "tests" / "fixtures" / "simulation" / "worlds" / "base.json"
    simulated = run([robolang, "simulate", EXAMPLE, "--world", world], cwd=outside, env=env)
    expect(
        b'"mode": "simulation"' in simulated.stdout and b'"state": "SUCCEEDED"' in simulated.stdout,
        "simulate runs the example in simulation and it SUCCEEDED (exit 0)",
    )

    module = run([python, "-m", "robolang", "--version"], cwd=outside, env=env).stdout.decode().strip()
    expect(module == version, "python -m robolang works")

    check_no_legacy_code(python, outside, env, work)


PROBE = """
import importlib, importlib.metadata, importlib.util, json, sys
legacy = sys.argv[1].split(",")
found = {m: importlib.util.find_spec(m) is not None for m in legacy}
errors = {}
for m in legacy:
    try:
        importlib.import_module(m)
        errors[m] = "imported"
    except ModuleNotFoundError as error:
        errors[m] = error.name
files = sorted({str(f).replace(chr(92), "/").split("/")[0] for f in importlib.metadata.files("robolang")})
print(json.dumps({"found": found, "errors": errors, "top": files}))
"""


def check_no_legacy_code(python: Path, outside: Path, env: dict, work: Path):
    """The installation holds only the read-only package, also when ROS 2 modules are importable."""
    result = json.loads(run([python, "-c", PROBE, ",".join(LEGACY)], cwd=outside, env=env).stdout)
    expect(not any(result["found"].values()), "none of the four legacy modules can be found in the environment")
    expect(
        all(result["errors"][m] == m for m in LEGACY),
        "importing each legacy module fails because the module itself is absent",
    )
    top = [name for name in result["top"] if name not in ("..", "") and not name.endswith(".dist-info")]
    expect(top == ["robolang"], f"the distribution installs only the robolang package ({top})")

    # Same check with stand-ins for the ROS 2 Python modules importable, as on a ROS 2 machine:
    # the legacy code must be absent, not merely blocked by a missing rclpy.
    stubs = work / "ros_stubs"
    for module in ROS_STUBS:
        package = stubs.joinpath(*module.split("."))
        package.mkdir(parents=True, exist_ok=True)
        for directory in (package, *package.relative_to(stubs).parents):
            (stubs / directory / "__init__.py").touch()
    with_ros = dict(env, PYTHONPATH=str(stubs))
    probe = "import rclpy.node, control_msgs.action, std_msgs.msg, trajectory_msgs.msg"
    expect(run([python, "-c", probe], cwd=outside, env=with_ros, check=False).returncode == 0,
           "stand-in ROS 2 modules are importable in this second check")
    result = json.loads(run([python, "-c", PROBE, ",".join(LEGACY)], cwd=outside, env=with_ros).stdout)
    expect(
        not any(result["found"].values()) and all(result["errors"][m] == m for m in LEGACY),
        "with ROS 2 modules importable, the four legacy modules are still absent",
    )


def main():
    if shutil.which("git") is None:
        print("note: git not found; continuing")
    with tempfile.TemporaryDirectory(prefix="robolang-smoke-") as tmp:
        tmp = Path(tmp)
        wheel, sdist = build(tmp / "dist")
        for artifact in (wheel, sdist):
            work = tmp / artifact.name.replace(".", "_")
            work.mkdir()
            check_install(artifact, work)
    print("all packaging checks passed")


if __name__ == "__main__":
    main()
