"""Create/check the BENCH-2 input manifest before paid runs; never includes credentials or results."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parent.parent
MANIFEST = ROOT / 'docs/evidence/bench2-freeze.json'
PATTERNS = (
    'bench/tasks*/*.json', 'bench/reference*/*.json', 'bench/reference*/*.rob', 'bench/worlds/*.json',
    'bench/*.py', 'bench/prompt*.md', 'bench/gate.json', 'bench/requirements.txt',
    'src/robolang/**/*.py', 'src/robolang/**/*.json', 'src/robolang/reference/*.md',
    'docs/language-reference.md', 'spec/grammar-v1.3.ebnf', 'tests/test_bench*.py', 'tools/bench_freeze.py',
)


def fingerprint() -> dict:
    paths = sorted({p for pattern in PATTERNS for p in ROOT.glob(pattern) if p.is_file()})
    return {'schema': 'robolang.bench.freeze.v1', 'patterns': list(PATTERNS),
            'files': {p.relative_to(ROOT).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}}


def digest(payload: dict) -> str:
    return hashlib.sha256((json.dumps(payload, sort_keys=True, separators=(',', ':')) + '\n').encode()).hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check', action='store_true', help='verify the recorded manifest without modifying it')
    args = parser.parse_args()
    payload = fingerprint()
    current = {'sha256': digest(payload), **payload}
    if args.check:
        if not MANIFEST.exists() or json.loads(MANIFEST.read_text()) != current:
            print('BENCH-2 freeze mismatch: input files changed; record reviewed errata before another paid run.',
                  file=sys.stderr)
            return 1
    else:
        if MANIFEST.exists():
            print('Manifest already exists. Preserve it and record an explicit erratum before replacing it.',
                  file=sys.stderr)
            return 1
        MANIFEST.write_text(json.dumps(current, indent=2) + '\n', encoding='utf-8')
    print(f"{current['sha256']} ({len(payload['files'])} files)")
    return 0


if __name__ == '__main__':
    sys.exit(main())
