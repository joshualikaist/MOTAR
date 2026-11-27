# Amendment to BOIA-SHADOW-Q01 v1 — 2026-10-21

**Status of v1: `WITHDRAWN_BEFORE_EXECUTION_NOT_RUN`.** Nothing was measured under v1. Its
output root `/home/fair/workspaces/aerial_gym_ws/boia_shadow_runs/BOIA-SHADOW-Q01` was never
created, and the preflight now refuses to proceed if it exists. The v1 files are unchanged:

- [config](boia_shadow_q01_v1.json), SHA-256 `759fef83ebbc4365c9080cb8dc15ebbb2209cc4d097f7dc5a235f1a41bc54df8`
- [preregistration](boia_shadow_q01_v1.md), SHA-256 `cf5da98367770a1f6ebaf83461a533ed74e40c3469402b3e2f269e8a179f36b0`
- source commit `6bd986eb585600ece1f8c59a7348ebfd8093fd42`

It is replaced by [BOIA-SHADOW-Q01-V2](boia_shadow_q01_v2.md) with a new RunID and output
root. The supervisor at the new source accepts only the v2 configuration.

## Why

A Fable 5.1 read-only audit of `6bd986e` (G4-R6, verdict `PASS_WITH_EXPLICIT_LIMITATIONS`)
found that v1's S1_OFF and S1_ON both install EvaluatorTap and ContactChain and differ only in
the diagnostic call. An exact OFF/ON match therefore cannot show that the observer leaves
Frozen F unchanged. A CPU counterexample (an observer moving the plant 6 mm per action) passed
the v1 gate 6/6 rows. v1's claim of "exact OFF/ON noninterference" was broader than its design.

## What changed, by finding

| Finding | v2 change |
|---|---|
| F1 (P1) OFF/ON gate blind to observer-vs-native | New `S1_NATIVE` stage with no EvaluatorTap/ContactChain/diagnostic. All stages share one player loop that writes a read-only host trace; S1_NATIVE vs S1_OFF is compared exactly. The cx1 counterexample is a regression test. |
| F2 (P2) `diagnose()` ignores sensor values | Claims narrowed: WAIT/INTERVENE computation NOT_QUALIFIED; diagnostic latency labelled NOT_INFORMATIVE_FOR_FULL_PLANNER; full-planner latency NOT_MEASURED. No algorithm added. |
| F3 (P2) no determinism control under `cudnn.benchmark`/TF32 | Numerics recorded per stage and required equal; S0 vs S1_ON replay control over the S0 prefix. Mismatches are `REPLAY_INSTABILITY_OR_PROFILE_MISMATCH`, cause NOT_VERIFIED. Settings are not changed. |
| F4 (NOTE) supervisor polling load | Resource poll 0.5 s → 2 s; source/config guard every 10 s; sleep never passes the watchdog deadline. No guard removed. |
| F5 (NOTE) S0 cold start | S0 wall 120 s → 180 s. |
| F7 (NOTE) inactive noise variables | Kept unchanged so the runtime environment equals v1; annotated as inactive and validated to stay gated by `NAVRL_PERCEPTION_PERTURB=0`. |
| F8 (NOTE) SIGTERM leaves no worker receipt | Worker converts SIGTERM into `failure.json` within a 3 s grace; shutdown starts 4 s (was 2 s) before the cap; the supervisor receipt embeds the worker's first cause and return code. |
| F6 (NOTE) research tests fail to import standalone | Test-only `sys.path` bootstrap; no assertion changed. |

Budget: the campaign cap rises from 720 s to **1080 s** — 300 s for the new S1_NATIVE stage
and 60 s for S0. Per-stage action caps (20/200/200/200), disk, output, snapshot and VRAM
limits are unchanged. Logger output is capped per stage at 256 MB / 4.

GPU execution remains unauthorized until the user separately approves the exact v2 commit.
G5 HOLD.
