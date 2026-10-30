"""
The gate 2 specification bundle (spec/interop-gate-2-v0.1.md 4.1; INTEROP-2g).

Builds the allowlisted files, an example amr-07 view (6.4) and a SHA-256 manifest into an empty
directory, and verifies a built bundle.

    python tools/interop_bundle.py build OUT_DIR [--rev REV]
    python tools/interop_bundle.py verify BUNDLE_DIR

With `--rev`, every file is read from that git revision, so a bundle built from the same commit
is byte-identical on every platform. The example view is made by the view builder
(`robolang.interop.build_view`) of the code that runs, so a pinned build fails closed unless the
checkout is that commit and its code (`src/`, `tools/`) has no local change. Without `--rev`, the
working tree is read with line endings normalized to LF, as git stores them: a candidate, never a
freeze.

`verify` needs nothing from this repository: it checks that the directory holds exactly the files
the manifest lists, with those hashes. `sha256sum -c MANIFEST.sha256` checks the hashes too.
"""

from __future__ import annotations
import argparse
import hashlib
import json
import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "src"))

from robolang.arch02 import load_scenario  # noqa: E402
from robolang.interop import AMR, build_view  # noqa: E402

DOCUMENTS = (  # gate 2 4.1, in its order
    "spec/architecture/rfc-001-ai-native-architecture.md",
    "spec/core-v0.1.md",
    "spec/reference-simulation-v0.1.md",
    "spec/decisions/0007-rl-arch-02-core-and-reference-simulation.md",
    "spec/decisions/0008-core-sequencing-per-recipient.md",
    "spec/interop-gate-2-v0.1.md",
)
NOT_GATE_2 = ("spec/core/message-0.2.schema.json",)  # Core 0.2 is not in the gate 2 bundle, which is Core 0.1
DIRECTORIES = ("spec/core", "tests/fixtures/core")  # the schemas; the conformance kit (G-T0)
VIEW = "view/amr-07-base-scenario.json"
README = "BUNDLE-README.md"
MANIFEST = "MANIFEST.sha256"
SCENARIO = "tests/fixtures/arch02/scenario.json"  # read to build the view; never copied


CODE = ("src", "tools")  # what builds the bundle: it must be the pinned commit's


class Source:
    """Files at a git revision, or in the working tree with LF line endings."""

    def __init__(self, rev: str | None):
        self.rev = rev and self._git("rev-parse", "--verify", f"{rev}^{{commit}}").decode().strip()
        if self.rev:
            self.check_pinned()

    @staticmethod
    def _git(*args: str) -> bytes:
        return subprocess.run(["git", *args], cwd=REPO, capture_output=True, check=True).stdout

    @classmethod
    def _head(cls) -> str:
        return cls._git("rev-parse", "HEAD").decode().strip()

    @classmethod
    def _changed_code(cls) -> list[str]:
        """Files of the building code that differ from HEAD, or are untracked."""
        changed = cls._git("status", "--porcelain", "--untracked-files=all", "--", *CODE).decode()
        return [line[3:] for line in changed.splitlines()]

    def check_pinned(self) -> None:
        """Fail closed: the code that builds a pinned bundle must be the pinned commit's (F-1)."""
        head = self._head()
        if head != self.rev:
            raise SystemExit(f"checkout is {head}, not {self.rev}: check out the pinned commit first")
        changed = self._changed_code()
        if changed:
            raise SystemExit(f"local changes to the building code: {', '.join(changed)}")

    def files(self, directory: str) -> list[str]:
        if self.rev:
            listed = self._git("ls-tree", "-r", "--name-only", self.rev, "--", directory).decode()
            found = listed.splitlines()
        else:
            found = [p.relative_to(REPO).as_posix() for p in (REPO / directory).rglob("*") if p.is_file()]
        return sorted(f for f in found if f not in NOT_GATE_2)

    def read(self, path: str) -> bytes:
        if self.rev:
            return self._git("show", f"{self.rev}:{path}")
        return (REPO / path).read_bytes().replace(b"\r\n", b"\n")


def readme(commit: str | None) -> bytes:
    source = f"commit `{commit}`" if commit else "a working tree (not a frozen bundle)"
    return f"""# RoboLang gate 2 specification bundle

Delivery metadata, not normative. Built from the RoboLang repository at {source}.

This bundle is everything the independent author of `rl:robot:amr-07` receives for gate 2
(`spec/interop-gate-2-v0.1.md`, section 4.1). The normative documents are the specifications
under `spec/` and the schemas under `spec/core/`. Files keep their repository paths, so the
relative links between them work.

## Contents

| Path | What |
|---|---|
| `spec/architecture/rfc-001-ai-native-architecture.md` | RFC-001: semantics and authority rules |
| `spec/core-v0.1.md`, `spec/core/*.schema.json` | Core 0.1 and its two JSON Schemas |
| `spec/reference-simulation-v0.1.md` | amr-07's behavior, the scenario and the faults |
| `spec/decisions/0007-…`, `spec/decisions/0008-…` | Choices and their reasons |
| `spec/interop-gate-2-v0.1.md` | The test binding `robolang-sim-stdio` 0.1 and the tests |
| `tests/fixtures/core/` | The conformance kit (G-T0): fixtures, `context.json`, `cases.json`, README |
| `{VIEW}` | An example `start` view for amr-07 in the base scenario (gate 2 6.4) |
| `{MANIFEST}` | SHA-256 of every other file |

## Checking the bundle

    sha256sum -c {MANIFEST}

On Windows, `Get-FileHash -Algorithm SHA256 <file>` gives the same values. The directory must
hold exactly the files the manifest lists, plus the manifest itself.

## Starting

1. Core 0.1 (`spec/core-v0.1.md`) and its schemas: the contract on the wire.
2. The conformance kit (`tests/fixtures/core/README.md` and gate 2 section 7): a `check` mode
   that gives each of the 47 fixtures its stated receipt is the first thing you can test alone.
3. The reference simulation (`spec/reference-simulation-v0.1.md`), sections 4 to 8: what amr-07
   does, with section 6.4 for amr-07 in detail.
4. Gate 2 (`spec/interop-gate-2-v0.1.md`), section 6: the binding, and the view this bundle's
   example shows.

Some documents link to files that are not in the bundle: other decision records, the
development plan, evidence, and the reference's fixtures and code. Those links are informative
citations. Every rule they record is stated in the documents above; a rule you cannot find here
is a finding to raise through the question log (gate 2 4.3), not something to look up elsewhere.
""".encode("utf-8")


def manifest_text(directory: Path) -> bytes:
    lines = []
    for path in sorted(p for p in directory.rglob("*") if p.is_file() and p.name != MANIFEST):
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
        lines.append(f"{digest}  {path.relative_to(directory).as_posix()}\n")
    return "".join(lines).encode("ascii")


def contents(source: Source) -> dict[str, bytes]:
    """Bundle path -> bytes, for every file except the manifest."""
    files = {path: source.read(path) for path in DOCUMENTS}
    for directory in DIRECTORIES:
        files.update({path: source.read(path) for path in source.files(directory)})
    view = build_view(load_scenario(source.read(SCENARIO)), AMR)
    files[VIEW] = (json.dumps(view, indent=2, ensure_ascii=False) + "\n").encode("utf-8")
    files[README] = readme(source.rev)
    return files


def build(out: Path, rev: str | None = None) -> str:
    """Write the bundle into `out` (which must be absent or empty); return its manifest's SHA-256."""
    if out.exists() and any(out.iterdir()):
        raise SystemExit(f"{out} is not empty")
    for path, data in contents(Source(rev)).items():
        target = out / path
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(data)
    manifest = manifest_text(out)
    (out / MANIFEST).write_bytes(manifest)
    return hashlib.sha256(manifest).hexdigest()


def verify(directory: Path) -> str:
    """Raise if the files differ from the manifest; return the manifest's SHA-256 (the bundle ID)."""
    manifest = (directory / MANIFEST).read_bytes()
    if manifest_text(directory) != manifest:
        raise SystemExit("the bundle does not match its manifest")
    return hashlib.sha256(manifest).hexdigest()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    commands = parser.add_subparsers(dest="command", required=True)
    b = commands.add_parser("build")
    b.add_argument("out", type=Path)
    b.add_argument("--rev")
    v = commands.add_parser("verify")
    v.add_argument("bundle", type=Path)
    args = parser.parse_args()
    if args.command == "build":
        print(f"bundle {build(args.out, args.rev)}")
    else:
        print(f"verified {verify(args.bundle)}")


if __name__ == "__main__":
    main()
