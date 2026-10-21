# G4-R7 implementation report — Fable G4-R6 fixes and Q01-V2

2026-10-21 KST, branch `codex/mt-p1`, base `6bd986eb585600ece1f8c59a7348ebfd8093fd42`.
Source commits `1efbcf8`, `e6db0f1`, `a0840c2` (final). CPU implementation and verification only.
**GPU NOT_RUN. G5 HOLD.**
Test results and bindings: [validation.json](validation.json). Commands:
[GPU runbook](GPU_SHADOW_RUNBOOK.md). Plan: [Q01-V2 preregistration](../../plans/boia_shadow_q01_v2.md),
[v1 amendment](../../plans/boia_shadow_q01_v1_amendment_2026-10-21.md).

## Findings and changes

| ID | Severity | Status | Change |
|---|---|---|---|
| F1 | P1 | FIXED (CPU) | `S1_NATIVE` stage; shared player loop `drive` and read-only `HostTrace` in every stage; exact NATIVE/OFF comparison in `verify`; cx1 regression tests |
| F2 | P2 | FIXED (claims) | Diagnostic latency labelled `DIAGNOSTIC_ONLY_NOT_INFORMATIVE_FOR_FULL_PLANNER`; `qualification_scope` in config and every receipt; no algorithm added |
| F3 | P2 | FIXED (CPU) | `runtime_attestation` records numerics per stage (never changes them); S0/S1_ON replay control; unattributed `REPLAY_INSTABILITY_OR_PROFILE_MISMATCH` |
| F4 | NOTE | FIXED | Resource poll 2 s, source/config guard 10 s, deadline-aware sleep; every guard kept |
| F5 | NOTE | FIXED | S0 wall 120 → 180 s |
| F6 | NOTE | FIXED | `sys.path` bootstrap in `tests/test_boia_research.py` and `tests/test_boia_research_safety.py`; assertions unchanged |
| F7 | NOTE | ANNOTATED | Inactive dropout/noise variables kept (runtime env identical to v1), listed in `inactive_runtime_env` and validated to stay gated by `NAVRL_PERCEPTION_PERTURB=0` |
| F8 | NOTE | FIXED | Worker SIGTERM → `failure.json`; 3 s grace inside a 4 s shutdown margin; supervisor receipt embeds the worker's first cause and return code |

## How the native baseline works

`tools/boia_shadow.py worker` builds Frozen F exactly as before (`eq1d_fixture_launcher.build_player`,
deterministic `get_action`, `env_step`). For `S1_NATIVE` it constructs no `EventLogger`,
`Recorder` or `EvaluatorTap`, so no method is wrapped, no `ContactChain` is installed and
`diagnose` never runs. Each wrapper EvaluatorTap installs now carries a marker, and
`hook_audit` reads the eight hooked attributes and the physics callback before, during and
after the loop: NATIVE must show none live, observer stages all of them. Production code,
controller, governor, checkpoint and RMS are untouched.

All four stages run the same `motar_eval.boia_shadow.drive` loop, which writes `host.jsonl`:
actor-input hash (taken before the policy call, because the returned observation aliases the
task buffer and is overwritten in place), F action bytes and values, RNG fingerprints before
and after the policy and after the step, returned/reset observation hash, reward, done, and
19 state hashes kept evaluator-side (own pose/velocity, crash/truncation buffers, target and
obstacle state, tracker, histories, previous action, `sim_steps`). In observer stages `drive` also requires
the host row to match the tap's own actor, action, post-step observation and terminal record.
These host-side reads exist in every stage, NATIVE included; no fully uninstrumented run is
compared.

## Comparisons (`verify`, CPU, exact)

| Comparison | Files | Mode | Supports |
|---|---|---|---|
| S1_NATIVE vs S1_OFF | `host.jsonl` | full | observer + ContactChain + logger noninterference on the host-visible F path |
| S1_OFF vs S1_ON | `parity.jsonl`, `host.jsonl` | full | diagnostic-computation noninterference over an installed observer |
| S0 vs S1_ON | `parity.jsonl`, `host.jsonl` | S0 prefix | fresh-process replay of the ON configuration, ≤ 20 actions |
| runtime profile | receipts | equality | same numerics attestation in every stage; hook state as planned |

No tolerance, exclusion or salvage. Missing or empty traces and length mismatches never pass.
A mismatch records the first row, differing field paths and both rows; its cause stays
`NOT_VERIFIED`.

## Fable cx1, as a regression test

`PlantNudgeTap` moves the plant 6 mm per action identically in OFF and ON. Result in the
fixture: OFF/ON `parity.jsonl` and `host.jsonl` PASS (the v1 gate is blind), NATIVE/OFF host
trace MISMATCH at row 1 on `state.own_position`. Through the real worker and `verify` the
campaign is `REPLAY_INSTABILITY_OR_PROFILE_MISMATCH` with observer noninterference
`NOT_SUPPORTED`. Related tests: a post-reset-only perturbation is caught on the terminal row;
an observer RNG draw is caught on `rng_post_step`; native RNG draws match across fresh stages.

## Fable 5.1 re-review and follow-up

An independent read-only Fable 5.1 review of source commit `1efbcf8` returned
**PASS_WITH_LIMITATIONS with no P0/P1**. It reran the focused tests (48 and 23 OK), both
research tests standalone (30 and 20 OK) and the CPU preflight (PASS), confirmed from the real
call path that S1_NATIVE installs no hook and that cx1 fails NATIVE/OFF while passing OFF/ON, and
confirmed the research worktree at 0/0/0 with both symlinks preserved. Its findings and the
follow-up commit:

| ID | Severity | Finding | Follow-up |
|---|---|---|---|
| R1 | P2 | "seed 4901" said not to be the executed seed | Checked: partly incorrect. rl_games seeds the yaml's 42 at runner load, then `BaseTask.seed(task_config.seed)` reseeds from `NAVRL_SEED=4901` (`navrl_task_config.py:108`, missed by the review). 4901 is effective. Added the runtime check `task_seed` and the two-step explanation to the prereg. |
| R2 | NOTE | replay control covers only ON, ≤ 20 actions | Prereg now states the mismatch cases it cannot separate |
| R3 | NOTE | terminal/reset compared only if an episode ends within 200 actions | `claims.terminal_reset_compared` per comparison |
| R4 | NOTE | some counters are not hashed | `sim_steps` (time-limit truncation) added to the host trace |
| R5 | NOTE | receipts not cross-checked with trace row counts | `verify` requires receipt counts to equal the file rows |
| R6 | NOTE | any "MISMATCH" substring counted as parity evidence | explicit online-mismatch tags; schema failures stay INCOMPLETE |
| R7 | NOTE | SIGTERM path not exercised by a real signal | CPU test sends a real SIGTERM inside the worker |

Fable then reviewed the follow-up delta `1efbcf8..e6db0f1`: **PASS**, no P0/P1/P2. It confirmed the
seed chain independently (`BasePlayer` pops and discards the yaml seed; `make_task` passes no seed;
nothing reseeds after `BaseTask.seed`) and left four NOTEs (an unclosed file, an untested claim
branch, "200-action cap" wording for S0, an undocumented allow-list), fixed in `a0840c2`. A final
Fable micro-review of `e6db0f1..a0840c2` returned **PASS** with no findings. Final source: `a0840c2`.

## Limits that remain

- GPU runtime parity, device build, compile time, VRAM and timing: `GPU_PENDING`.
- Exact equality on the GPU may fail from numerics alone (`cudnn.benchmark`, TF32 are on for
  Frozen F). The replay control covers only the ON configuration over ≤ 20 actions; a NATIVE
  replay control does not exist in this design.
- `diagnose()` reads no sensor values: Q01-V2 qualifies acquisition mechanics, not BOIA.
- B03/B04, F continuation, physical C02/backup/plant/deadline/endgame unresolved. G5 HOLD.
