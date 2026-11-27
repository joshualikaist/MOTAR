# Independent dev08 preparation source review r1 — 2026-11-05

Verdict: **PASS_WITH_LIMITS for static configuration and planning preparation only.** No open scoped source defect remains after the author's two narrow repairs described below. PUBLIC_CPU is **FAIL**, runtime is **NOT_IMPLEMENTED**, measurement is **NOT_RUN**, and the fixed twenty-episode input cohort is **BLOCKED_INCOMPLETE_TARGET_TRACE**. This review supplies no experiment, gate, transfer or flight acceptance.

Reviewer: independent delegated Codex agent `/root/dev08_source_review`, read-only to the worktree. The active role identifies Codex as based on GPT-6; the exact serving model ID is not exposed to this reviewer. No Opus identity or Opus execution is claimed. The parent supplied the code and criteria; this review inspected the actual source, reports and input payloads rather than adopting an author conclusion.

Worktree: `/home/fair/workspaces/aerial_gym_ws/src/MOTAR-camonly`, branch `camonly/dev`, HEAD/base `1fe9afa398fcbbe7e009f193c93d74735e03a5c0`. The shared root clock was read, not reset or closed: deadline 2026-11-05T11:47:14Z, wrap 10:47:14Z, finish 11:37:14Z. Latest handoff, worktree AGENTS.md, latest WORKLOG and Beads state were read. Beads unit `motar-mx0` is owned by the parent.

## Findings by severity

- **P0/P1: none found in this preparation scope.**
- **P2 F1, repaired and rechecked:** the initial protocol's future-runtime requirement said “Keep the native parameter path default OFF.” This contradicted the native default in both source and intended contract. Final line78 says “Keep candidate overrides OFF by default,” preserving the native path. No measurement or historical record was changed.
- **P3 F2, repaired and rechecked:** initial JSON truthfully emitted `execution_supported:false` and `measurement_status:NOT_RUN`, but lacked the explicitly requested machine-readable runtime status. Final source additionally emits `runtime_status:NOT_IMPLEMENTED`; the short-target test asserts it.
- **Open validation limitation, not a dev08 source defect:** the shared PUBLIC_CPU run is FAIL, as detailed below. It must not be described as a green full suite or a completed global quality gate.

The input shortfall and unimplemented runtime are intentionally retained blockers, not repaired or relabeled as experimental success.

## Source and protocol assessment

The only production implementation is additive `tools/sitl_s3b_dev08.py`. Its imports are standard library modules only. It reads four pinned files, hashes them before interpreting them, parses the runner using `ast.parse`/`ast.literal_eval`, and prints JSON. There is no runner/bridge/Isaac/pymavlink/policy import, launch operation, world write, checkpoint access, or runtime option. `--execute` is rejected. Normal JSON exit0 means configuration preparation completed; the document and output explicitly limit that scope.

Native is the default in `variant_params`, `prepare` and the CLI. Candidate updates a new dictionary. The code asserts exactly the registered three changed keys: `MPC_XY_VEL_P_ACC=2.0`, `MC_YAWRATE_P=0.2`, `MC_YAWRATE_I=0.1`. All historical entries remain intact; yaw's absent native override is labeled `NOT_EXPLICITLY_OVERRIDDEN`, without inventing effective PX4 values. The literal contract fixes DT0.1, ODE100, ALT1.0, MAX_STEPS600, CAPTURE_M0.5, FOOT_HALF0.14 and OOB_MARGIN1.0. Both pinned dev07 reports agree with the native map, ODE iterations, ordered env0–9 selection, zero-action target source and truth command yaw.

Pinned original runner/bridge bytes match actual files and inventory. Their git diffs, together with `motar_eval/occlusion_trace.py`, are empty against the base. The tool accurately leaves `trace_payload_verified:false`; it does not pretend that report provenance verifies live raw inputs or runtime behavior. Missing files or any byte drift in each pinned file refuse preparation. No episode/output identity is inferred from parameter equality.

The final protocol fixes seed4501, densities70/205, env0–9/episode0, twenty proposed candidate episodes, exact historical full/zero paths and dev07 comparisons. It retains the historical205/env8 crash. It fixes fresh per-attempt output naming, bounded process-tree resources, idle host, telemetry/clock/start/trace/host-fault aborts, readback/isolation requirements, owned-only cleanup, generated-SDF/template limitations, source freeze and preregistration/approval boundary. These are explicitly future-runtime requirements, not implemented features. Analysis is descriptive with fixed paired outcome/step summaries; no posthoc tolerance, gain ranking, noninferiority, S3b PASS or flight claim is introduced. S3a remains NOT_EQUIVALENT. The amended experiment-family index uses the same narrow statuses. Evidence/WORKLOG/local-commit integration remains the parent's responsibility. Before finalizing this review, the newly added evidence README and two JSON records were inspected: `input_inventory.json` is byte-identical to the independently verified inventory (same SHA-256), and `candidate_configuration.json` parses exactly equal to final `prepare(arm="candidate")` output (SHA-256 `fb643423a3f9f74093eda38f84adf8ace8e83e1eb7e2f322ecc1c63c91ca9e1f`). Its runtime/measurement/input statuses are correct. Open Beads follow-ups `motar-q8j` and `motar-lp5.6` explicitly retain the runtime/input and historical-availability blockers. The README truthfully retains PUBLIC_CPU FAIL and the final-metadata timing limitation; its pending validation/review artifacts remain for the parent to copy before commitment.

## Independent actual-input audit

The inventory file `/home/fair/motar_tmp/dev08_preparation_20261105/input_audit/input_inventory_final.json` was itself checked against SHA-256 `379c02fac58b62fdd160ab7c6c6f99065d8c4939044cc97931e5fcac73fcd12d`.

The independent stdlib-only script `verify_actual_inputs.py` in this audit directory hashed and checked actual sizes for all34 listed files: twenty raw receipt/ledger/summary/frame/layout payloads and fourteen source/report/vehicle/launch assets. Every entry matched. It independently decoded the94-byte binary wire records without importing MOTAR or the trace-reader module, checked each selected frame sequence and finite floating state, and compared full/zero target positions on every selected common frame. It also checked selected initial pursuer pose/quaternion equality, each historical report's target and Isaac frame counts, and actual Isaac ledger outcomes.

All selected sequences are contiguous from frame0. Zero-action70bars/env5/episode0 has208 frames; every other one of the twenty proposed zero-action targets has600. Its actual zero-action ledger says capture at completion_step208. The historical dev07 report says target_frames208 and capture at step35, so that episode did not exhaust its target. This supports the stated future coverage blocker without invalidating or rewriting dev07. No padding, dropping, replacement, target regeneration or measurement was attempted. Full/zero common-frame equality does not overturn G8's TARGET_DEPENDS_ON_PURSUER reading.

The audit took0.152s, 2026-11-05T06:56:54.964073Z–06:56:55.115995Z, on core6/nice19, CUDA hidden, PYTHONNOUSERSITE=1, RLIMIT_AS512MiB, CPU limit20s, outer timeout30s. The parent authorized this brief low-CPU read while the shared suite ran. Checkpoint payloads were neither read nor loaded.

Machine record: `actual_input_checks.json`, SHA-256 `3ffe3bb5c763b65a0112202d55259b6a980a5bd5129eca360e27c72b433ef93e`.

## CPU verification and its limits

After the shared suite ended and both repairs were present, this reviewer independently ran:

```bash
env CUDA_VISIBLE_DEVICES= PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
  nice -n 10 taskset -c 6 timeout -k 2 30 \
  bash -c 'ulimit -v 1048576; exec python3 -B tests/test_sitl_dev08_preparation.py -v'
```

**7/7 PASS in0.096s.** The controls accept real intact inputs and native/candidate configurations; reject each of four independently drifted pinned files with positive controls before/after; reject altered DT/ODE/duplicate contracts and invalid arms/CLI; verify the exact nonzero gain delta, native copy/environment/path preservation, explicit runtime/measurement statuses and exact coverage shortfall. The literal-reader test includes source that would raise if executed. The result is configuration testing, not runtime identity. `focused_tests.log` SHA-256: `3d43f2d32577f0f3c50682a347139ff012c0f9e9477c19f757d760da38ee8827`. `git diff --check` passed.

The actual parent-run `/home/fair/motar_tmp/dev08_preparation_20261105/public_cpu.log` was inspected: **3285 tests in219.869s,2 failures,4 errors,17 skipped,5 excluded, status FAIL**. Its tracebacks show two missing `/tmp/codex_occ` temporary-parent errors; missing `results/navrl_v2_ep25000_stopcap_seed49_screen/off/205bars.log`; missing `results/navrl_physical_target_braking_lower1p25_headingrest_seed827/summary.json` and its result-directory manifest check; and an absent historical P1c ep900 checkpoint in the ref5in launcher contract. The affected test files and stopcap summarizer have empty diffs against the base. The actual targeted recheck log shows the two temporary-parent tests pass after creation of the empty parent, while the four historical-availability failures remain. Parent before/after journal records both show fault_count0. This reviewer did not rerun the entire suite or read the missing checkpoint. A full clean baseline run was not performed, and global validation remains FAIL.

## Reviewed final file hashes

| File | SHA-256 |
| --- | --- |
| tools/sitl_s3b_dev08.py | e63049cc92bae56afad212219e42af9f91def1f2fac88f688e69a0065a8fa3fb |
| tests/test_sitl_dev08_preparation.py | 957b317a6fa2a4dab446f318127d0adfb3e74654b21832c9bbae1f05f42d169d |
| docs/plans/prereg_sitl_dev08_candidate_gains_v1.md | f9ac386e88eaef3f2e022124b5e8e227ffa567dbf9b05458f97e5df37b50d483 |
| docs/EXPERIMENTS.md | 2eba99d9756b2cc35a0fe66e09fd85ca7e872f870a2134eb67bf6a72c87bc484 |

Initial reviewed tool/test/plan hashes, before the narrow repairs: `85f6321692e645d308d4efb8df248980a365157eda935f19d7c54f87602e5f3e`, `3f37fa969a32d59f5d673ed58a23a580acf0c1892c366ffc3ca473cca4d2ca24`, `af62ec1ea2c43b77b43cc0dcca106109a16162d7f30538dac3af00fc450293f2`.

## Scoped readiness and remaining boundary

Static configuration/protocol preparation is ready to record with these explicit limits. Runtime remains NOT_IMPLEMENTED; no SITL/GPU work is authorized or tested. The fixed candidate cohort cannot launch until a separate input decision/amendment resolves the208/600 coverage shortfall and an additive runtime meets the listed CPU/readback/ownership/provenance/resource requirements. Formal S3b acceptance requires its own protocol and authorization. PUBLIC_CPU remains FAIL and must travel with the preparation record.

## Git Traceability

Branch: `camonly/dev`. Base SHA: `1fe9afa398fcbbe7e009f193c93d74735e03a5c0`. New Commit SHA: NONE by reviewer. Changed Files: reviewer wrote only this report, its script and JSON/log evidence under `/home/fair/motar_tmp/dev08_preparation_20261105/source_review_r1/`; no tracked file mutation. Tests: independent7/7 focused PASS, actual raw-input audit PASS within stated scope, full PUBLIC_CPU FAIL as above. Working Tree: parent-owned modified `docs/EXPERIMENTS.md`; parent-owned untracked plan, test and additive tool at review snapshot; parent-owned dev08 evidence directory subsequently added; final validation/review copies, WORKLOG and commit integration pending. Remote Push: NO. Main Merge: NO. Raw/Checkpoint Integrity: all34 inventoried file hashes/sizes match; original runner/bridge/trace reader unchanged against base; no reviewer raw writes or checkpoint payload reads/loads. No claim that uninspected checkpoint contents were independently rehashed.
