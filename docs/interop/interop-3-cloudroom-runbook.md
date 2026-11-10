# INTEROP-3 runbook: clean-room author in a Cloudroom Codex thread

**For the owner (operator and intermediary) only. Never put this file, or any other RoboLang repository file, into
the author's project.** It applies the [INTEROP-3 protocol](interop-3-protocol.md) to a Codex thread in Cloudroom
(owner's choice, 2026-11-10). Prepared by a RoboLang session, which therefore takes no role in the run: it does not
start, message or answer the author.

## 0. Why the network check is not optional

The RoboLang repository is public and `pip install robolang` ships the reference implementation. An author who can
reach GitHub or PyPI could read the answer. The run counts as clean-room (gate 2 spec 2, AI form) only if the author's
shell cannot reach the network. Step 3 checks this; if it fails, stop.

## 1. Get the author kit (4 files)

Download every asset of the draft GitHub release **`interop-3-author-kit`** (Releases page of the repository, visible
to collaborators only). Check the SHA-256 of each:

| File | Bytes | SHA-256 |
|---|---:|---|
| `robolang-gate2-bundle-f3164c5a.zip` | 547,496 | `ba87425152ebf007fb7d740de90ec130b4151b2bb78f7bb5a95e83e93476da70` |
| `typescript-5.9.3.tgz` | 4,377,468 | `10e108c9cf7d5f2879053dff18515fb405abf2ccef63eaaf017d9c571687a1d3` |
| `types-node-24.19.2.tgz` | 474,850 | `e01cc90bdafb14295f15a8dfbcddd37834250e9dfab4763ff686fb751915d9ce` |
| `undici-types-7.24.6.tgz` | 28,510 | `2abf595a86b87dd22951218848ec4ad89c018db0e656716dc04b8b2f9290a0f4` |

The archive is the frozen gate 2 bundle (rebuilt and re-verified 2026-11-10: bundle `f3164c5a…7e22`, 122 files,
manifest check passes). The three npm packages are the TypeScript compiler and Node type definitions, unchanged from
the npm registry, so the author can compile with no network (protocol 2.1, 2.3).

## 2. Create the author's project

1. Make a new, empty folder, for example `C:\gate2-author\`, and put **only those 4 files** in it. No Git repository,
   no other files.
2. In Cloudroom, add that folder as a **new project**.
3. Start a **Cloud** thread in it (not Local): provider **Codex**, the GPT-6-family model if it is offered (write
   down the exact model name Cloudroom shows). Use no custom instructions that mention RoboLang.

## 3. First message: environment check (paste verbatim)

```text
Before any task, check this environment and report the raw output. Do not do anything else yet.

for u in https://github.com https://pypi.org/simple/ https://registry.npmjs.org/ https://www.google.com; do
  curl -sS -m 10 -o /dev/null "$u" && echo "OPEN $u" || echo "BLOCKED $u"
done
sha256sum *.zip *.tgz
mkdir -p bundle impl && cd bundle && unzip -q ../*.zip && cd ..
echo "files: $(find bundle -type f | wc -l)"
sha256sum "$(find bundle -name MANIFEST.sha256)"
(cd "$(dirname "$(find bundle -name MANIFEST.sha256)")" && sha256sum -c --quiet MANIFEST.sha256 && echo "MANIFEST OK")
chmod -R a-w bundle
(cd impl && npm init -y >/dev/null && npm install --offline --no-audit --no-fund ../undici-types-7.24.6.tgz ../types-node-24.19.2.tgz ../typescript-5.9.3.tgz && npx --offline tsc --version)
node --version; uname -m
Also say whether you have any web search or browsing tool, yes or no.
```

**Stop unless all of these hold:** every URL says `BLOCKED`; no web search or browsing tool; the four hashes match
section 1; `files: 122`; the manifest hash is `f3164c5afc54893be232433c785f11f765dae780483a3e8d7ba28178247c7e22`;
`MANIFEST OK`; `tsc` prints `Version 5.9.3`. If any URL is `OPEN`, this environment is not clean-room: archive the
thread and tell the RoboLang thread; the fallback is a container or VM you provision (protocol, option A).

## 4. Second message: the task (paste verbatim, then the environment facts)

Paste the prompt of [protocol section 4](interop-3-protocol.md#4-initial-prompt-verbatim-core-02-wording-approved-by-the-owner-2026-11-02)
exactly, from "You are an independent software engineer" to "whether Gate 2 passes.", then this paragraph (environment
facts only, which the protocol allows; it is recorded with the prompt):

```text
Environment facts: the frozen bundle is unpacked, read-only, in ./bundle. Write your implementation in ./impl, where
TypeScript 5.9.3 and @types/node are already installed (use `npx tsc`; there is no network). Node.js and a Linux shell
are available. To ask a question, write a message that starts with "QUESTION:" and stop; the intermediary will answer
in this thread.
```

## 5. While it works: questions (you are the intermediary)

- Answer **only from the bundle**: quote the file and section. If the bundle does not answer it, say exactly "The
  bundle does not specify this." Never add a rule, a hint or an expectation.
- Do not ask RoboLang threads for answers. Keep every question and answer in the thread (it is the Q&A log).
- An answer that would add or change a rule is an erratum, not an answer: stop and record it (protocol 2.4).

## 6. When it declares ready

Ask it, in the thread: "Create impl.tar.gz of ./impl without node_modules, print its SHA-256 and your build, test and
run commands, and your G-T0 results." Keep the thread (its transcript is the audit trail; do not archive it). Then
tell the RoboLang thread the thread name, model, the tarball hash, and how to retrieve the tarball. INTEROP-4 (gate 2
run against the reference) is a separate step you authorize.

## 7. Attestation (protocol section 5): fill in before step 4

| Item | Value |
|---|---|
| Operator, date | |
| Provider; exact model ID shown by Cloudroom; model ID the thread reports | Codex via Cloudroom; |
| Account/project: new, no memory or stored conversations, no custom instructions | new Cloudroom project; Codex defaults |
| Agent runtime and configuration | Cloudroom Cloud thread, Codex; tools as offered (record) |
| VM or container image and digest | Cloudroom cloud sandbox; `uname -m` from step 3 |
| Node.js and TypeScript versions | from step 3; TypeScript 5.9.3 |
| Pre-installed libraries: name, version, integrity hash, source | typescript 5.9.3, @types/node 24.19.2, undici-types 7.24.6; hashes in section 1; npm registry |
| Network egress policy | step 3 output: all `BLOCKED` |
| Filesystem layout | `./bundle` read-only; `./impl` empty except node_modules; nothing else |
| Archive SHA-256 in the environment (must be `ba874251…da70`) | |
| Files after extraction (122); MANIFEST.sha256 SHA-256 (`f3164c5a…7e22`); `sha256sum -c` | |
| Transcript capture | the Cloudroom thread itself, kept, not archived |
| Q&A channel and intermediary | this thread; the owner |
| Nothing from the RoboLang repository in the environment | project contains only the 4 kit files |
| Operator signature | |

**Deviations from the protocol, for the owner to accept or reject:** the network is closed by the agent's sandbox, not
by a firewall rule you wrote (step 3 shows the effect, not the rule set); the model may not be GPT-6 family. Launch
also needs your authorization, written in the thread or here.

## 8. Owner's choice on 2026-11-10: option A (Cloudroom Codex, search audited)

The first check in a Cloudroom Codex Cloud thread showed every URL `OPEN`, no `iptables`, and a web search tool;
Cloudroom's Codex settings have no switch to turn web search off. The owner chose option A over a self-provisioned
container (B):

- **Codex memory off** (Settings → Codex) before the author thread starts; the old test thread is archived.
- **Shell network closed by a firewall** in the author's sandbox: the first message installs `iptables`, runs the
  section 3 checks, then sets `OUTPUT DROP` (loopback and established connections allowed) for IPv4 and IPv6, and
  re-tests the four URLs, which must all be `BLOCKED`.
- **External search is not technically blocked** (deviation from protocol 2.3). The environment paragraph says that
  web search, browsing and any network access are not permitted and that any use voids the work.
- **Audit** after the author declares ready: the full transcript is checked for any web search or browsing call and
  any command that changes the firewall or reaches the network (`iptables`, `curl`, `wget`, `git`, `pip`, registry
  `npm install`). Any hit voids the run.
- **Label** in the gate 2 record: "clean-room AI (Codex via Cloudroom); shell network blocked by firewall; external
  search not technically blocked, verified by transcript audit". A protocol-compliant run (B, or a human team) is
  stronger evidence and may follow.
