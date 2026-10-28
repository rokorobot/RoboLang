# Gate 2 specification bundle — freeze record

**Status:** **FROZEN — approved by the owner on 2026-10-28.** The bundle below is the bundle of
the whole gate 2 run (`spec/interop-gate-2-v0.1.md` 4.2): any change to a bundled file is an
erratum with a new bundle and a new record. The frozen source commit is fixed at
`28c69feece0de14b1537a4d29e0cd4b700cff920`; the commit that merges this record does not change it,
and the bundle is never rebuilt from a later commit.

This record lives outside the bundle on purpose: writing the bundle ID into a bundled file would
change the bundle (audit report `docs/interop/interop-2g-bundle-audit.md`, section 8).

## Identity

| | |
|---|---|
| Source commit | `28c69feece0de14b1537a4d29e0cd4b700cff920` (squash merge of PR #46, INTEROP-2g, on `main`) |
| CI on the source commit | Run 40707558908: Ubuntu and Windows, Python 3.11 and 3.13, all passed |
| Bundle ID (SHA-256 of `MANIFEST.sha256`) | `e6c9bb751503d52a16aa7f6aa0c42c9fe54726027ae5ff6ef229e566df22d104` |
| Files | 60, plus the manifest |
| Built by | `tools/interop_bundle.py build <dir> --rev 28c69feece0de14b1537a4d29e0cd4b700cff920`, in a clean detached worktree at that commit (the tool refuses any other checkout) |

The bundle's own `spec/interop-gate-2-v0.1.md` section 13 says "INTEROP-2g authorized; the freeze
itself needs owner approval": that was the state at the source commit, by construction.

## Verification performed (2026-10-28)

1. Built twice from the clean worktree into empty directories: byte-identical.
2. `python tools/interop_bundle.py verify`: the files are exactly those of the manifest.
3. `sha256sum -c MANIFEST.sha256`: every file OK.
4. `BUNDLE-README.md` names the source commit above.

Anyone can repeat it: clone the repository, `git checkout --detach 28c69feece0de14b1537a4d29e0cd4b700cff920`,
build into an empty directory, and compare the manifest with the one below. Its SHA-256 must be
the bundle ID.

## Manifest

```
43bb6ba0c2f3bc6cccb1480f8848a6592ce4bd2cd9faf68a4b4e2d4b14e0ca37  BUNDLE-README.md
30c6cb9062e7a34a984315760b21c42e182a8f49a4a696b0a81a50048a1eda13  spec/architecture/rfc-001-ai-native-architecture.md
3f180e87bb68d94a9edfd92eab4234a24cca8abcc4dee59544754889a73b698a  spec/core/capability-descriptor-0.1.schema.json
a7eaa9d35e5399b088bb7cf4ca77dd86de23bca998abeb0fbf5bdb40ce881f94  spec/core/message-0.1.schema.json
fac387097f01f64d5d35ed72767ed990dec6085357c45118e9bd428d31251db8  spec/core-v0.1.md
736719c64fa0ac9a32e1b96aa505893f2ae471ab7d55e062ae83e3119404fbdf  spec/decisions/0007-rl-arch-02-core-and-reference-simulation.md
f0e6f58124151d7c9f7f0879afe1eccecdd3640fdc80a27f8a5d3547dcca3e71  spec/decisions/0008-core-sequencing-per-recipient.md
bf1f2b132b9bf4254b5cb68f2fc489bcce0123ceb99015a2c8fc53d0e2292e65  spec/interop-gate-2-v0.1.md
c0bf8cb67ce6df896546bf87aa736d8eb030de642ab543aeb58849d38e59729f  spec/reference-simulation-v0.1.md
67103648ad735684de65db85c340382d721ef010507b6658a51dc4e8313c99c3  tests/fixtures/core/cases.json
53ff08bb009acb689b14931c1e853ca00f5f26f9175069d930b4819190cc1893  tests/fixtures/core/context.json
f764efcb88aeed324a1a4eb487b22464f7c8b549208388482fd751b5afb46085  tests/fixtures/core/invalid/c101_byte_order_mark.json
e6b11433e05d399add7d0c6398c380bec50eed6e1948ac8a77063d33e7f9cb0d  tests/fixtures/core/invalid/c101_duplicate_key.json
6f975ff195b823559fae2515c150215edc9350308c89eb50114e9bef187034ae  tests/fixtures/core/invalid/c101_non_integer_number.json
50f3090449faf175bc751a3f0e9d4f81afe0e44f40ba64ddcda0a3752ae89e14  tests/fixtures/core/invalid/c102_before_c106.json
b1ba5551daa3ce5b79220ce2d444a5aa6d77ffe369d687d8aea53127cf6c6bab  tests/fixtures/core/invalid/c102_before_c107.json
042f0e7007d9208d9aeb23f963e0a17a45810759b3897e002136273ba3e547bc  tests/fixtures/core/invalid/c102_before_c201.json
da1970f197f2a3538a043923acd7db5c1d6cf1f2d6bb54517f2796b4ff849fa1  tests/fixtures/core/invalid/c102_before_c402.json
e10157a70e7ae70dd4c862f6f8e47b465e316ff12f99c2f6ccb4de06bbdc5b5d  tests/fixtures/core/invalid/c102_unsupported_version.json
26d8497b9a843a6d6c0c757398b57ae122f295e756e49d2047c34323f92d1a0b  tests/fixtures/core/invalid/c103_before_c104.json
1188ed96c4ef77c576935e5d8b808915b58974e46c5e8457d3c55182544c7f82  tests/fixtures/core/invalid/c103_unknown_field.json
3fd3c2218047e7b94497c598d3374f1063d2fa293bd12e290327dd2812735001  tests/fixtures/core/invalid/c104_goal_without_criteria.json
ffaa0abba3e42022f803980f28733cabf59d14e904db4d9ed4e4692444699d42  tests/fixtures/core/invalid/c104_missing_field.json
6aab4ae0b8d8fe573d5254412aadc4b500cc413a92e5ba75c2b0e999338d9138  tests/fixtures/core/invalid/c105_before_c201.json
59a3522e21b4ae8194c479343c164c47b2721e170b832aab538f5e947b134543  tests/fixtures/core/invalid/c105_unknown_type.json
fcbf1e90d295356883403f69aed830d53f73f4eacf54a104bba016b9faefe4c8  tests/fixtures/core/invalid/c106_before_c201.json
7f9ebf30171c4f4c5e5d5c7f3fde049d8e55c6ea80a79f5b9d1e73ee0a36d572  tests/fixtures/core/invalid/c106_reserved_type.json
26251b878dd8a12e5deb53e5c163bd3e0207e345c8097ca6e09197774778f924  tests/fixtures/core/invalid/c107_profile_mismatch.json
eaee2c547040946fa14fcf12d726b628bbe4c5f632387835b2ed2407d5e015f1  tests/fixtures/core/invalid/c201_before_c107.json
33f23803cdc4344384bad63a5264a15c0cf843499ab1e9541767325a8e7c1d63  tests/fixtures/core/invalid/c201_before_c402.json
149736e5b9efa864c1ad1e6a9917b198e20bd60935d62b28f8f6c163e5b84163  tests/fixtures/core/invalid/c201_unsigned.json
c4fba5d5a6bb5bad3772aa4e518951164e56f4671fcdbbfc1bc75ee9228fdb8f  tests/fixtures/core/invalid/c202_message_before_c203_observation.json
c810f3c892a04107e2ee2ac8475f17151068c8fee10e8cfb1a03a37f8b32b887  tests/fixtures/core/invalid/c202_wrong_signature.json
f35dc429565981fbc91690ca4248ae1a8d7b115938347c74945445ad9407c2d4  tests/fixtures/core/invalid/c203_before_c202.json
a9d61511765dca0f238b0d07f2dec68366b2aa6c229d1d68b60e03562fa23ddc  tests/fixtures/core/invalid/c203_key_not_the_senders.json
4359e4d60f1fe50d943d8be07a73c8d881692a0879b295cb608d4c8f211bb483  tests/fixtures/core/invalid/c203_message_before_c204_observation.json
ead5cbec1b7b6ed49c2bd629eab13a383af9fd50d9861c4bdabd2016d0f76a93  tests/fixtures/core/invalid/c204_before_c203_before_c202.json
0eb636d6c2755b4cca76909ce14ad06fcbfb0003bfa62492f33a2edabdcb82b5  tests/fixtures/core/invalid/c204_simulation_signature_in_physical_mode.json
44935aa083b5c7a1d18b8624e0ea5e9e4bc823938ac07e1fa4ead6f766d78a57  tests/fixtures/core/invalid/c301_before_c307.json
93cc264f895aa1b826e32f9791b5a5b785bd24bd1e03d5064cfadd2152da83dd  tests/fixtures/core/invalid/c301_expired.json
9001b2ac20c66e9f3f06898931af4ce1c89b1f9fe4dccd4f4d8588b7356735fd  tests/fixtures/core/invalid/c302_before_c301.json
223814d7ae5d5a5c7be3b47a43833b8049dcb4bba8f3cc733b4873e4bd2dafbc  tests/fixtures/core/invalid/c302_before_c303.json
f14b1eb2aaeb1344341a0ff928e668d3887b4ba96ee34d233413c5d535f80d07  tests/fixtures/core/invalid/c302_lifetime_too_long.json
5b5764b779f85d6bef43cecef8940743b2ae054ad3bcd210b41a07b981fd3a28  tests/fixtures/core/invalid/c303_before_c307.json
805a69a6382b65c1b23416a6b1064ec716924a0effda8a409248d41350e73908  tests/fixtures/core/invalid/c303_sent_in_the_future.json
0eb636d6c2755b4cca76909ce14ad06fcbfb0003bfa62492f33a2edabdcb82b5  tests/fixtures/core/invalid/c307_wrong_recipient.json
dc73ef769237885ecd98de96985b5ee66f3ef756ec24c68caf2570c2cc3ef863  tests/fixtures/core/invalid/c401_requester_authorizes_its_own_goal.json
c524ee711d98ca036cef870fa57f97b5d2abb68a6a881b54a365d4ae33dbcda2  tests/fixtures/core/invalid/c402_succeeded_with_unmet_criterion.json
a93e623574747a0b9ba55b50a7e136f767409577a8b9b9b9103e5679ec21647d  tests/fixtures/core/invalid/c402_succeeded_without_verification.json
9c9d9fa34adbd2c06163c6f69c5771587c48016eb32af3df8c2f7a861dd3781f  tests/fixtures/core/invalid/c404_physical_mode.json
6aef3cf1ba2af43dd7631e6161c38ef12fb52bae8e31cf1a91b31676aa8735a4  tests/fixtures/core/README.md
0eb636d6c2755b4cca76909ce14ad06fcbfb0003bfa62492f33a2edabdcb82b5  tests/fixtures/core/valid/assessment.json
73a94926601552bab6a8a9e31b4a34ebe54edea2942c284054969e26f234fa94  tests/fixtures/core/valid/authorization_decision.json
c3843f73449a0f99710b658d6c63315cc2fb4b9128c07c9f861e322668c1a887  tests/fixtures/core/valid/commit.json
8529f8803eb7b491253e34b2a58663849d67be1e5b85bd47562fd65fd87517e5  tests/fixtures/core/valid/custody_confirmed.json
46683daf402c2630fc78031fef7e163029475d29eb8ffeb9a2e0e3040480bd28  tests/fixtures/core/valid/descriptor_amr_07.json
8bc187a2ac482faa801e06801d8e32a42754ac5c10cc3923da1a53fdcca15315  tests/fixtures/core/valid/descriptor_arm_02.json
dc85157eb8d7d557d557d7eb6f1c910c40be8a988bbe0318e39b05538a1dd18d  tests/fixtures/core/valid/outcome_succeeded.json
939be05544a144cea736e710ae1c0876060001ab304661d9fbff9c61375bf511  tests/fixtures/core/valid/propose_goal.json
598e0c9b78b2fe29b86704d7e375af9bdba7c4e4234eed89305e7c14b6c86f66  view/amr-07-base-scenario.json
```

## Not covered by this record

- Publishing the bundle to the independent author (INTEROP-3, separately authorized, with the
  author named). **Delivery requirement (owner, 2026-10-28):** before INTEROP-3 begins, the frozen
  bundle is packed into one ZIP or TAR archive containing exactly the 61 files above; the archive
  is preserved, its own SHA-256 is recorded separately from the bundle ID, and only that archive
  is delivered. The independent author never clones this repository to rebuild the bundle.
- Harness limitation H4 (discrepancy register), to be corrected before INTEROP-4. The harness is
  not part of the bundle, so its correction does not change this bundle.
