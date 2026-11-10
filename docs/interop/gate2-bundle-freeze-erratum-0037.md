# Gate 2 bundle freeze: erratum 0037 (F1, F2) — 2026-11-10

Supersedes the Core 0.2 freeze ([gate2-bundle-freeze-core-0-2.md](gate2-bundle-freeze-core-0-2.md), bundle `f3164c5a…`)
for INTEROP-4 run 2. Decision record [0037](../../spec/decisions/0037-gate-2-errata-f1-f2.md), approved by the owner on
2026-11-10, from [INTEROP-4 run 1](interop-4-run-1.md).

| Item | Value |
|---|---|
| Source commit | `05859a61b2a78e53520cf5c12bab0148082bcf3f` (decision 0037 on branch `room/erratum-0037`) |
| Bundle ID (SHA-256 of `MANIFEST.sha256`) | `3a33dce09fd3be38deef191fe4c2c662c1f4094848c7f38aa5c53ecc6cc112a7` |
| Files | 122 |
| Built by | `tools/interop_bundle.py build <dir> --rev 05859a61b2a78e53520cf5c12bab0148082bcf3f`, twice, each in a clean detached worktree at that commit |
| Archive | `robolang-gate2-bundle-3a33dce0.zip`, SHA-256 `16a97331526f5f011c89976be5c5c4031b1397779186510f3a69a0dac4d07194`, 548283 bytes (`tools/interop_bundle.py archive`) |

Checks: the two builds are byte-identical (`diff -r`); `interop_bundle.py verify` passes; `tests/test_interop_bundle.py`
and the other interop tests pass on the source commit (116 tests), including all 47 Core 0.1 fixtures and all 62
Core 0.2 entries giving their stated receipts from the bundle with the reference receiver.

**Changes from bundle `f3164c5a…`, and nothing else:** `spec/interop-gate-2-v0.1.md` (status line, 6.3 `sensed` row:
F1), `spec/core-v0.1.md` 5.9 and `spec/core-v0.2.md` 3 (F2), `BUNDLE-README.md` (the build commit), and
`MANIFEST.sha256`. The reference already behaves as the errata state: no code changed.

**Delivery:** the archive is attached to the draft release `interop-3-author-kit` and delivered to the author session
as an erratum (protocol 2.4). The author updates its implementation from the bundle alone.
