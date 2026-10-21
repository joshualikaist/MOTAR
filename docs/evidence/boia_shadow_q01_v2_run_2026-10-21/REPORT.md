# BOIA-SHADOW-Q01-V2 — result

2026-10-21 16:58–16:59 KST. One user-approved bounded GPU shadow run at commit
`f6a0ce6b17f46c28a4477df2b914cfac1eb5d2d7` (source `453bd12`), RTX 3070, following the
[preregistration](../../plans/boia_shadow_q01_v2.md) and
[runbook](../boia_g4r7_shadow_v2_preflight_2026-10-21/GPU_SHADOW_RUNBOOK.md). Machine record:
[result.json](result.json); compact receipts in [receipts/](receipts/). Raw output stays at
`/home/fair/workspaces/aerial_gym_ws/boia_shadow_runs/BOIA-SHADOW-Q01-V2/` (48 files, 1.0 MB),
bound by SHA-256 in `result.json` and the per-stage RAW_MANIFESTs.

**Verdict: `PASS_EXACT_NATIVE_OFF_ON_PARITY_WITH_REPLAY_CONTROL`.**

| Stage | Supervisor | Wall (s) | Actions | Episode end | Observer P50/P95/P99 (ms) | Full step P50/P95/P99 (ms) |
|---|---|---|---|---|---|---|
| S0 (ON) | COMPLETED | 8.2 | 20 | censored at 20-action cap | 3.82 / 7.07 / 8.31 | 45.8 / 86.1 / 89.0 |
| S1_NATIVE | COMPLETED | 12.3 | 30 | ended at action 30 | — (no observer) | 40.6 / 72.4 / 271.8 |
| S1_OFF | COMPLETED | 8.2 | 30 | capture | 3.61 / 5.18 / 5.51 | 43.5 / 57.5 / 269.0 |
| S1_ON | COMPLETED | 8.2 | 30 | capture | 3.73 / 5.27 / 5.78 | 44.1 / 48.9 / 269.7 |

Campaign wall 37.0 s of the 1,080 s cap. Device memory, sampled by the supervisor every 2 s
(not a true peak), reached 5,329–5,332 MiB per stage against the 6,656 MiB ceiling, with 661 MiB
in use before launch; torch-allocated peak 15 MiB. Diagnostic-only time (S0 n=4, S1_ON n=6)
stayed under 0.07 ms and is not informative for a full planner.

| Comparison | Rows compared | Result |
|---|---|---|
| S1_NATIVE vs S1_OFF, host trace | 31 / 31 | PASS, exact, terminal/reset row included |
| S1_OFF vs S1_ON, parity + host | 30 / 30, 31 / 31 | PASS, exact, terminal/reset row included |
| S0 vs S1_ON prefix, parity + host | 20, 21 (S0 censored at its cap) | PASS, exact |
| Runtime profile | 4 stages | PASS: same numerics (cudnn.benchmark and TF32 on, as Frozen F sets them), hooks absent in NATIVE and present in observer stages |

All worker and supervisor receipts are COMPLETED with return code 0. Source, config
(`be1c22e4…`), checkpoint (`f7022139…`) and RMS (`642fc9f7…`) bindings match, and the checkpoint
is unchanged after every stage. Imports resolved inside this worktree.

Outcomes: in S1_OFF and S1_ON the single episode ended in capture at action 30 with zero
thresholded contact substeps and no action-boundary height/OOB violation. The collision-free
capture **proxy** is therefore True. It does not certify the absence of sub-threshold contact or
of between-boundary excursions. S1_NATIVE records no outcome labels (no evaluator); its host
trace equals S1_OFF row for row, including the terminal row. No crash, no timeout; S0 was
censored by its process cap.

What this supports, for this exact N1 profile and one episode per stage: the evaluator and
ContactChain did not change the host-visible Frozen F path; the stride-5 diagnostic did not
change anything over an installed observer; the ON configuration replayed exactly over 20
actions. What it does not show: any BOIA decision computation (diagnose ABSTAIN), mission
benefit, physical safety, real-time behaviour, other seeds/profiles or longer horizons.
G5 HOLD.
