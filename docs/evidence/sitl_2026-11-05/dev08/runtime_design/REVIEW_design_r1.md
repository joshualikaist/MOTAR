# Independent source/design review r1 — dev08 observed prefix

**Verdict: PASS_WITH_LIMITS for the repaired offline preparation and additive design only.**
No open source blocker was found in the final snapshot. Runtime remains NOT_IMPLEMENTED,
measurement NOT_RUN, launch_ready=false and execution_supported=false. The historical
strict600 protocol remains BLOCKED_INCOMPLETE_TARGET_TRACE. This is no SITL/GPU execution
approval, formal S3b verdict, full-horizon outcome claim or physical/real-flight qualification.

Reviewer: independent fresh-context Codex subagent `/root/dev08_prefix_review`. The environment
identifies Codex as based on GPT-6; exact backend model is NOT_RECORDED. No Opus review is claimed.
Root worktree `/home/fair/workspaces/aerial_gym_ws/src/MOTAR-camonly`, branch `camonly/dev`,
review base `5f444abcfe94fc8ae59cbb7c5946e71ac32106e8`. Root clock was read and retained:
start06:47:14Z, wrap10:47:14Z, finish11:37:14Z, deadline11:47:14Z on2026-11-05.

## Findings by severity and repairs

- **MAJOR-1, repaired:** Initial `summarize_complete_pairs` accepted any self-consistent
  `frame_window`, so replacing every pinned208/600-position window with one position and
  censoring all40 records at frame0 produced a complete-pair summary. This could hide harness
  failure as administrative censoring. The final implementation checks exact20 scene-window
  identities and canonical208 for70/env5 and600 for the other nineteen. The independently
  reproduced initial counterexample now raises ValueError. Required repair is present.
- **MAJOR-2, repaired:** Initial ledger validation used bars/env/arm only and ignored seed,
  episode, protocol and operational validity. All40 rows with seed9999, episode17, a foreign
  protocol and ERROR operational status were accepted as20 successful pairs. The final version
  requires exact integer seed4501/episode0, observed-prefix-v1, VALID status, finite nonnegative
  nondecreasing actual simulator endpoints, exact nominal scheduled_elapsed_s=k×DT and the
  preexisting command/check counts. Independent wrong-identity/status counterexamples now
  refuse. The amendment explicitly states that scalar endpoints/status cannot prove live
  per-step timing or parameter checks; those remain future runtime boundaries.

Initial source snapshots, hashes and accepted counterexamples are retained externally in
`initial_snapshot/`, `initial_manifest.json`, `initial_checks.json` and
`initial_accepted_counterexamples.json`. Final snapshots and checks are separate; no initial
failure evidence was overwritten.

## Source, actual inputs and design checked

- Read the workspace and actual-worktree AGENTS, shared latest handoff, current WORKLOG,
  motar-q8j, project/evidence/experiment/decision context, the preserved parent protocol,
  proposed amendment, both new source/test files and the external resolution probe. Reviewed
  actual historical recorder and SITL terminal-loop code rather than relying on the probe's
  conclusions. Confirmed the installed launcher has global `pkill -x gazebo` and a shared
  rootfs, and actual source/build rcS import working-directory parameter files.
- Independently rehashed all34 inventoried raw/source/report/PX4 assets against their pinned
  byte counts and SHA-256 values. Independently decoded the four actual selected binary trace
  cells with stdlib struct, checking exact env0..9/episode0 membership, contiguous indices,
  finite selected states, common full/zero target positions and initial pursuer pose. Full
  provenance hashes and exact per-env counts are in `initial_checks.json`.
- The actual zero70/env5 prefix has208 positions0..207; the other nineteen selected zero paths
  have600 positions0..599. Recorder consume/after_physics ordering supports the documented
  state-after-k-actions convention. The legacy loop logs the current check and commands before
  checking terminal state: its steps35 is final index34, not a measured35-transition endpoint.
  Historical physical checks are inside the observed support. The amendment separates actual
  timestamps from nominal20.7/59.9s and never infers frame208/600, a600-step timeout or padding.
- Physical capture-before-crash precedence and terminal-before-censoring are explicit. The
  planned runtime emits commands only on CONTINUE, so the last nonterminal target checkpoint
  administratively censors without an unscored next command. Cleanup is explicitly outside
  scoring. Fresh native and candidate arms share this new lifecycle; historical dev07 is a
  descriptive comparator, and full legacy process-lifecycle identity is not claimed.
- The fixed future proposal is two native smoke episodes plus20 main scene pairs/40 main
  records, total42 episodes, with fixed density/env/arm order and smoke excluded from analysis.
  Exact20 windows, seed4501/episode0 and40 unique main records preserve all denominators,
  including205/env8. Censored nonterminal paths remain in20-per-arm/10-per-density denominators.
  Logical capture/crash bounds are correctly separate from confidence/safety/causal claims.
- CPU fixtures exercise positive and rejection cases, rather than passing by refusing every
  input. The final13 tests passed independently. Additional independent mixed arithmetic
  accepted native18capture/1crash/1censor and candidate17capture/2crash/1censor, with20 paired
  transitions,2 pairs containing censoring and16 dual-capture checkpoint differences. The
  bounds and per-density counts matched hand counts. Details are in
  `final_independent_mixed_summary.json` and `final_checks.json`.
- Final real-input preflight ran with both defaults and explicit prefix selection:
  strict=BLOCKED_STRICT600; prefix=PREFIX_INPUTS_VERIFIED; verified files34;
  execution_supported=false and launch_ready=false. It imported no torch/numpy/pymavlink,
  Isaac or aerial_gym, did not mutate environment/sys.path, did not load a checkpoint and
  did not launch PX4/Gazebo. Pinned configuration code is stdlib-only; historical runner and
  bridge are parsed/hashed rather than imported.

## Remaining limits and future implementation boundaries

The review certifies offline input/record contracts and a coherent proposed design. It does
not certify physical geometry, semantic NPZ/world fidelity, live command emission, coherent
packet freshness, 10Hz pacing, exact native policy command-prefix identity, parameter echoes,
fresh rootfs isolation, exclusive UDP14540/process ownership, cleanup/reaping or runtime
resource guards. These are clearly marked mandatory and unimplemented. Scalar record fields
are declarations to be bound to live guard evidence, not a substitute for it.

The future design specifies an idle host, CUDA hidden/PYTHONNOUSERSITE, cores6/7/12/13/nice10,
bounded threads,12GiB aggregate sampled RSS watchdog at0.25s,240s per episode,2400s per main
density block,5400s overall and kill grace20s. It accurately calls this sampled RSS rather
than a kernel hardcap, warns caps may abort a slow42-episode unit, requires fresh no-overwrite
paths, source/protocol/runtime-review commitment, source freeze, host-fault preservation and
new explicit scoped approval. Exact direct-launch argv, isolated rootfs operation, thread env
values and timing/freshness rules still need concrete implementation/review before readiness.
Installed/source rcS and airframe, actual binaries, original SDF and plugin/environment paths
must be bound then; the mismatched jinja must not regenerate the preserved vehicle.

The independent review did not run full PUBLIC_CPU. The parent started that suite separately;
its result must be recorded without converting prior missing-asset failures into PASS. Evidence,
EXPERIMENTS/WORKLOG integration and local commit were parent-owned and pending at this review.

## Verification windows and resource use

All reviewer compute was brief CPU-only stdlib work on core6, CUDA hidden and
PYTHONNOUSERSITE=1. Initial decode/hash/test checks ran
2026-11-05T07:28:56.470841+00:00 to 2026-11-05T07:28:56.703600+00:00 under nice10,
1GiB address-space cap and timeout30s. Final13 tests/mixed fixtures/preflight ran
2026-11-05T07:30:14.040281+00:00 to 2026-11-05T07:30:14.353575+00:00 with the same bounds.
That final window may overlap the parent PUBLIC_CPU start; there is no timing/performance
claim. The parent's nice19 instruction arrived after this final verification completed;
subsequent report-writing used nice19/256MiB/timeout15s. No GPU/evaluation/SITL job ran here.

## Reviewed final source hashes

| File | SHA-256 |
| --- | --- |
| `tools/sitl_dev08_prefix_contract.py` | `f309dd5d2fdbfc8fb21c19fd5361a903b8fadfe525f8d246132c4ed75afc8c70` |
| `tests/test_sitl_dev08_prefix_contract.py` | `325e90730f5ccb02f68d8644eb8702fddb47d9f53a479e480e021f70ab0abc2e` |
| `docs/plans/prereg_sitl_dev08_candidate_gains_v1_amendment1_2026-11-05.md` | `159fea947a4db752afa746946dd7506ae8f24ee8a62f4c3b6d08f2ac7802c014` |

The following historical inputs are byte-equal to the review base:

| File | SHA-256 |
| --- | --- |
| `docs/plans/prereg_sitl_dev08_candidate_gains_v1.md` | `f9ac386e88eaef3f2e022124b5e8e227ffa567dbf9b05458f97e5df37b50d483` |
| `tools/sitl_s3b_closed_loop.py` | `d170b0271f73822280d8c92af68ab2bb56e350bbdf598702c353d8b79f0284b4` |
| `tools/sitl_bridge.py` | `f0031f114f4428ddd5f3b126067c80b1808071e1864402fb119b342259083db7` |
| `motar_eval/occlusion_trace.py` | `ec4aa27c994afb233aedcc48901db29d5c40e9ef33373580f99c03cb68a750e1` |
| `aerial_gym/task/navrl_task/kroc_eval_adapter.py` | `1712a7ef38e1dfc3542888eae4c556138695e96d38ee158811955a81c13819af` |

## Git Traceability

Branch `camonly/dev`; base `5f444abcfe94fc8ae59cbb7c5946e71ac32106e8`; New Commit SHA NONE
(reviewer read-only); Changed Files only this external audit directory; Tests final13/13 plus
independent mixed-ledger/counterexample and real-input stdlib checks PASS within scope;
Working Tree parent-owned staged/preparation changes, no reviewer tracked mutation;
Remote Push NO; Main Merge NO; Raw/Checkpoint Integrity: no raw/source/PX4 mutation, all34
inventoried assets match pinned hashes, historical source/parent prereg match base, no checkpoint
payload read/write or independently recomputed checkpoint hash.
