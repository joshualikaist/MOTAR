# G4-R handoff — completed CPU/design scope, awaiting user review

Base/current HEAD `cdfaffd28534cd37693a6cd678de9766bef4bd25`, branch `codex/mt-p1`.
**No new Git commit, staging, push or merge.** The current user explicitly forbids
local commit until the diff/results are reported. Earlier standing commit rules
therefore do not authorize it. Do not auto-commit this package in a resumed session.

Session started2026-10-20 03:05:48 UTC (12:05:48 KST). Actual five-hour quota deadline
is UNKNOWN; conservative wrap-up point16:05 KST. This package is finished before
that point. No owned evaluator/test process remains running. GPU/physics/learning
were not started. Disk free about9.65 GB; stop threshold3 GB. No data deletion.

## Completed, with precise scope

- New explicit CPU-only capsule/next-cycle APIs in types/geometry/safety; no loop hook.
- Exact horizon-pass/next-cycle-fail counterexample rejects in the new model.
- Final full BOIA189 PASS, including29 new tests; all old160 and ten test files preserved.
- PUBLIC_CPU2927/0 assertion failures/4 existing errors/10 skipped/5 excluded,
 156.488 s, overall FAIL. Its source is BEFORE the final time-grid consistency
 guard; final source is verified by the complete189 BOIA suite, not mislabelled
 as a final-source PUBLIC_CPU pass.
- First-prefix kinematic consistency guards malformed/mismatched external rollouts;
 assumed.1s grid consistency is not measured timestamp or plant qualification.
- All-body capsule math, float32 near-contact/floor and infinite backup-tail cases.
- New design-only evidence/paired-benefit/timing specification,15-row prior-art
 comparison and F0–F5 plus factorial mechanism draft. No new arbitration implemented.
- Independent review identified/fixed the nominal timing-contract hole. No concrete
 false-safe was found in the reviewed backup/capsule model; physical limits remain.
- Main HEAD/WIP, stash, Policy F hash, protected symlinks, old evidence manifest and
 production/source/test preservation checked. Full Raw rehash NOT_RUN; no external backup.

See [IMPLEMENTATION_REPORT](IMPLEMENTATION_REPORT.md), [validation](validation.json),
[integrity](integrity.json) and [manifest](EVIDENCE_MANIFEST.json).

## Remaining research gates (tracked in Beads)

- `motar-3p3.13`: B04 target/foreground association remains BLOCKED. Preserve hits.
- `motar-3p3.15`: B03 physical free-volume and full physical C02 qualification remain
 OPEN. This task's nominal C02 does not close them or activate production.
- `motar-3p3.17`: new measured-evidence contract and paired-benefit/timing CPU oracle
 package; **needs user approval**. Prospective scope is in the algorithm proposal.
- `motar-6nn.5`: pre-existing DetectReduction whole-suite errors, unchanged.
- G5 HOLD: real3-D sensing, plant/latency/rotor bounds, calibrated likelihoods and
 runtime remain unqualified. CPU N128K20 standalone verifier took3.863113 s even
 with no obstacle points. No real-time/GPU claim.

Next session first inspect HEAD/status and compare validation source/test hashes.
Review the current uncommitted patch with the user. Any implementation of new
arbitration/evidence inputs, GPU/Isaac run, Raw calibration or Git commit/push must
follow new explicit authority. Do not silently run the example receipt scripts
into existing outputs: they deliberately use exclusive output creation; use a new
scratch path and preserve existing receipts.

## Exact uncommitted package files

- `docs/evidence/boia_g4r_2026-10-20/C02_SOURCE_DIFF.patch`
- `WORKLOG.md`
- `docs/evidence/boia_g4r_2026-10-20/CPU_VALIDATION_PROTOCOL.md`
- `docs/evidence/boia_g4r_2026-10-20/EVIDENCE_MANIFEST.json`
- `docs/evidence/boia_g4r_2026-10-20/HANDOFF.md`
- `docs/evidence/boia_g4r_2026-10-20/IMPLEMENTATION_REPORT.md`
- `docs/evidence/boia_g4r_2026-10-20/adversarial_geometry.json`
- `docs/evidence/boia_g4r_2026-10-20/adversarial_geometry_check.py`
- `docs/evidence/boia_g4r_2026-10-20/boia_before_timing_guard.txt`
- `docs/evidence/boia_g4r_2026-10-20/boia_final_tests.txt`
- `docs/evidence/boia_g4r_2026-10-20/collect_model_receipt.py`
- `docs/evidence/boia_g4r_2026-10-20/integrity.json`
- `docs/evidence/boia_g4r_2026-10-20/model_receipt.json`
- `docs/evidence/boia_g4r_2026-10-20/public_cpu_before_timing_guard.txt`
- `docs/evidence/boia_g4r_2026-10-20/public_cpu_build.json`
- `docs/evidence/boia_g4r_2026-10-20/validation.json`
- `docs/research/occlusion_algorithm_v1/g4r_2026-10-20/ALGORITHM_PROPOSAL.md`
- `docs/research/occlusion_algorithm_v1/g4r_2026-10-20/EXPERIMENT_DRAFT.md`
- `docs/research/occlusion_algorithm_v1/g4r_2026-10-20/NOVELTY_REVIEW.md`
- `motar_eval/boia/geometry.py`
- `motar_eval/boia/safety.py`
- `motar_eval/boia/types.py`
- `tests/test_boia_c02_geometry.py`
- `tests/test_boia_c02_safety.py`

The pre-existing untracked symlinks `results/mt_p1_precision_v1` and
`results/occlusion_pilot_v1` are protected and are not package additions. No Raw
will become tracked merely through these links or a merge. No manifest/checkpoint
payload is staged. No main changes or stash operations are needed for review.

## Git Traceability

- Branch: `codex/mt-p1`
- Base SHA / unchanged HEAD: `cdfaffd28534cd37693a6cd678de9766bef4bd25`
- New Commit SHA: NONE — current instruction forbids local commit.
- Changed Files: exact list above.
- Tests: final BOIA189 PASS; broad pre-guard PUBLIC_CPU FAIL with4 known errors.
- Working Tree: approved unstaged/untracked package; protected symlinks preserved.
- Remote Push: NO
- Main Merge: NO
- Raw/Checkpoint Integrity: checkpoint hash and scope/path preservation PASS;
 Raw full payload rehash NOT_RUN, external backup NOT_CREATED.
