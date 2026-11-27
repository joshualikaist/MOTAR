# Source review r2 — repaired preparation bytes

**PASS_WITH_LIMITS for offline source preparation.** Open findings:0 BLOCKER,0 MAJOR,0 MINOR within the four-file scope. Initial r1 M1 and m1 are repaired and independently rechecked. This review grants no formal GO, measurement approval, flight readiness, contact-response qualification or S3a equivalence.

Reviewer: independent fresh-context Codex source-review agent, continuing the same review after repairs. Model: GPT-6 family; exact backend variant **NOT_RECORDED**, because it is not exposed. Worktree `/home/fair/workspaces/aerial_gym_ws/src/MOTAR-camonly`, branch `camonly/dev`, base/HEAD `f14bd059aab4c107cd547a221c1b10726ea57ca2`. Inherited continuation clock was checked and retained without reset/close. No simulator, policy inference, checkpoint deserialize or GPU process was launched.

## Exact reviewed identities

| File | SHA-256 |
| --- | --- |
| tools/sitl_training_geometry.py | cadd5167e874c2101118d3d24d9e6c517604571a64ed50a34391aeb3b7b0df35 |
| tests/test_sitl_training_geometry.py | 9f0712df400d9afffffd34699aad780203f6a6c4642fd6698b1b043d1dbd706c |
| tools/sitl_formal_readiness.py | c6ff36dc954e40dc1b83182810c2d2c2704723f13e38c8ca89d2469846022c6d |
| tests/test_sitl_formal_readiness.py | ecda56ed96a424b938c85aefb7880199e7464f91b2e53fc03c9225125fb71d09 |

Copies are under `reviewed_source/`. `bindings_before.json` equals `bindings_after.json` across44 files, including all four reviewed files, Frozen F, historical and installed vehicle/airframe/jinja/world/launcher assets, existing bridge/runner/owned-session source, and the inventory's raw full/zero receipt/episode/frame/layout/summary files. Existing tracked launchers, runtime and training URDF also have no diff from the base commit. Other author work, including a new follow-up plan, is outside this review.

## Findings resolved

**r1 M1:** `validate_trace_contract()` now checks the exact v1 wire dtype, contextual cell/seed/density/checkpoint/arm/behavior, target-model identity, scene bar pool/slot range, receipt identity, source-commit agreement and clean-source flags. Actual frame and layout files are independently checked against inventory byte/hash bindings; the summary's internal byte/hash records must agree; the declared frame count must agree with binary byte length. The synthetic positive contains all20 selected episode0 windows with601 finite states. Independent full-audit mutants rebind the edited summary in the inventory, so rejection cannot be explained by an outer summary-file hash mismatch. All15 single-field mutants are rejected: context seed/bars/cell/checkpoint, behavior identity, source commit/dirty flag, bar pool/slots, wire dtype, internal frame hash/bytes, internal layout hash/bytes, and frame count. Actual layout-byte tampering is separately rejected. See `identity_mutation_results.json`, `synthetic_positive.json`, and `probe.py`.

**r1 m1:** config bytes now come from one canonical `MODEL_CONFIG` constant. The adapter compares both actual bytes and the receipt hash with that constant. Independent probes reject altered config with both the original hash and a self-updated hash. The new unit test also changes the SDF filename inside config, updates the receipt hash, and verifies that planning is never reached.

## Evidence

- **17 focused tests pass:**6 geometry +11 readiness;0 failures/errors/skips (`tests.log`). The positive controls are nonempty. Negative decoder, identity, geometry, package and S3a cases accompany valid controls. Only focused and independent review probes are claimed here, not a full PUBLIC_CPU run.
- **Native OFF is exact original bytes:** installed and historical dev07 SDFs match SHA `f9188e906a23dfa498328beef0eb69b57b50e4d2c8acc06c8ba7d3f022fcd8b2`. Training URDF matches SHA `ad9a3110037cb01f94caa11054e367e557e9089b4edd520d0aa59b78578988be`. The optional transformation deletes exactly the four rotor collision elements; every other SDF byte is retained. Its output SHA remains `1b658c0bcf318df1cbe038776b92142310914cc3396da46573680aeface95932`. The origin-centered0.28×0.28×0.08m base box, dynamics/inertia/motor/plugin/joint/visual/include content and model resource paths survive unchanged. The correction is opt-in and defaults native.
- **Actual owned-session plan controls pass for both modes:** whole-plan equality permits only the substituted SDF path, its single spawn-file argument and added geometry receipt. All other arguments, environment, cwd, world path, rootfs and PX4 plan values stay identical. This is a pure plan comparison and starts no process.
- **601-state endpoint remains mandatory:** selected episode0/env completeness is exactly frame indices0..600; duplicates, gaps, truncation, trailing bytes and nonfinite selected records refuse or remain incomplete as appropriate. An independent infinity placement in each of20 float fields is rejected. Other episodes cannot fill the endpoint. No padding, extrapolation or scene substitution occurs in the tool.
- **Actual historical readiness is still BLOCKED:**20/20 input scenes incomplete;19 have600 states and70/env5 has208. Both historical S3a cells remain blocked (`actual_readiness.json`). Fixed observation/action tolerances remain1e-3/1e-4. Every original first8 env must have positive step count and finite in-tolerance values for all observation slices, bridge action, same-observation action and governor errors. An `EQUIVALENT` reading alone does not pass.
- **Even the full positive synthetic audit grants no authority:** all20 synthetic601-state scenes return `STRUCTURAL_CHECKS_PASS_REVIEW_REQUIRED`, while `formal_gate_authorized=False` and `formal_gate_verdict=NOT_TESTED`. Source-bound S3a controls, the1° yaw sensitivity control and independent review remain explicitly required separately. Synthetic tests are not rollout results.
- **Host/resources:**8GiB AS cap; timeout180s/kill10s; cores6,7,12,13; nice10; CUDA hidden; PYTHONNOUSERSITE1; OMP/MKL/OpenBLAS1. Recorded review probe elapsed0.253s. Kernel faults0→0, with before/after captures. Frozen F SHA remains `f702213936601860995cf61dcc570247e72543b1976e3716055cd8ec5593ad40`; it was hash-read only.

## Limits that remain

This is a structural and source review. A layout blob is bound by bytes/hash but not decoded for NPZ geometry semantics here. Matching metadata/source fields and a self-supplied inventory do not independently prove a newly generated trajectory's provenance or its semantic match to the frozen scenes. That source-bound input/scene qualification remains a separate requirement before formal use. Historical missing states remain missing, and historical S3a remains NOT_EQUIVALENT. No live native-control equivalence, flight/contact behavior, sensor realism, timing, boot repeatability, gain effect or formal gate was measured. The later diagnostic adapter/plan is outside the listed-file scope.

The failed supplemental r1 attempt is preserved in `../source_review_r1/`; it ran after author edits and yielded no success evidence. The r1 negative findings are retained rather than replaced by this repaired result.

## Git Traceability

Branch: `camonly/dev`. Base SHA/HEAD: `f14bd059aab4c107cd547a221c1b10726ea57ca2`. New Commit SHA: none (read-only reviewer). Changed Files: external review evidence only in `source_review_r1/` and `source_review_r2/`; no repository edit by reviewer. Tests:17/17 focused; actual owned-session plan checks in both modes;15 identity/schema/binding mutants; all20 nonfinite-float negatives; canonical config and layout-byte tamper controls. Working Tree: author-owned four untracked preparation files plus `docs/plans/sitl_agq_followup_v1_2026-11-06.md` at final check. Remote Push: NO. Main Merge: NO. Raw/Checkpoint Integrity:44 before/after bindings identical, canonical F matches, no deserialize or raw mutation.
