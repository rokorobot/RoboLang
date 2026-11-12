# Physical-run authorization 0.1

**Status:** decision [0038](decisions/0038-physical-mode-authorization-stop-approval.md) step 2. Reference:
`robolang.authorization` (needs `pip install 'robolang[ed25519]'`). **Physical mode is not enabled:** a chain that
passes every check below still ends in `A731`, and nothing in this release executes on hardware.

## 1. The chain

A physical run needs three signed documents. Each one names the one before it, so none can be reused for another
robot, cell, task or run.

| Document | `kind` | Signed by | Says |
|---|---|---|---|
| Cell configuration | `robolang.cell_configuration` | The owner (0038 P1) | This robot in this cell may run in physical mode, under this safety case and commissioning record, until `valid_until`; these people review and grant; this is the safe state |
| Task approval | `robolang.task_approval` | A reviewer listed in the cell (P4) | I read this exact Task IR and approve it for this robot and cell |
| Run grant | `robolang.run_grant` | An authority listed in the cell (P2, P3) | Run this approval once, now, for at most the cell's maximum lease |

Every document has `"version": "0.1"`, exactly the fields below (any other field is `A702`), and an `ed25519`
`signature` `{alg, key, value}` over the document without its `signature`, in RFC 8785 canonical form, as in Core
section 7. Times are Core times (`2026-11-12T10:00:00Z`). No JSON number may have a fraction or exponent.

The owner's public keys are the trust root. They are installed on the robot out of band and are never read from
any of these documents.

## 2. Fields

**Cell configuration:** `id`, `robot`, `cell` (Core identifiers); `safety_case` `{version, sha256}` and
`commissioning` `{record, sha256}` (the signed safety case and commissioning record it rests on); `keys`: key
reference → `{alg: "ed25519", public_key}` with the 32-byte key as 43 unpadded base64url characters, the participant
being the part before `#`; `authorities` and `reviewers`: non-empty lists of identifiers; `max_grant_lease_s`: an
integer from 1 to 86400; `safe_state`: in 0.1 exactly `{"position": "hold", "gripper": "unchanged"}` (P8);
`valid_from`, `valid_until`.

**Task approval:** `id`; `ir_sha256`: SHA-256, lowercase hex, of the Task IR in canonical form, so the same IR from
a `.rob` file or an IR JSON file gives the same hash; `task`, `language_version`; `robot`, `cell`; `author` (who
wrote the task: a person or a model, as text); `reviewer`; `approved_at`.

**Run grant:** `id`; `approval_sha256`: SHA-256 of the whole approval, signature included, in canonical form;
`robot`, `cell`; `operator`; `mode`: `"physical"`; `granted_at`, `expires_at`.

## 3. Checks, in order

The first failure is reported, with the document and a JSON Pointer into it.

| Code | Check |
|---|---|
| `A701` | The document is not a JSON object of the expected `kind` and `version` (also: invalid JSON, a repeated key, a fractional number) |
| `A702` | A field is missing, unknown or malformed |
| `A711` | The signature is not `ed25519`, or `ed25519` is not installed |
| `A712` | The key is not a listed key of the expected signer: the owner's installed keys for the cell configuration, the cell's `keys` for the reviewer and the operator |
| `A713` | The signature does not verify |
| `A721` | The cell configuration is not current |
| `A722` | The approval does not match the task: IR hash, task name, language version, robot or cell |
| `A723` | The reviewer is not a reviewer of the cell, or is the task's author |
| `A724` | The grant does not name this approval, robot or cell |
| `A725` | The operator is not an authority of the cell |
| `A726` | The grant's lease is empty or longer than `max_grant_lease_s`, or the grant is not current |
| `A731` | Every check passed. Physical mode is not enabled in this release |

Order: the cell configuration (`A701`, `A702`, signature, `A721`), then the approval (form, signature, `A722`,
`A723`), then the grant (form, signature, `A724`–`A726`). "Current" allows for the receiver's clock tolerance on the
safe side: the period counts only if it holds whether the clock is that much fast or slow.

## 4. Not in 0.1

Revocation lists, key rotation, two-person review (allowed by 0038 P5, not required), and the run record fields of
0038 P9: they arrive with the physical run itself (0038 step 4).
