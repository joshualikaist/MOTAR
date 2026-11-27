# E-L0 — evaluation-only lineage: implementation and GPU non-interference validation (run EL0-R1)

Status: **completed.** E-L0 adds independent evaluation records to the Frozen Policy F stack. It
does not change any observation, action, normalisation, perception, tracker, controller, physics,
termination or source file. The environment's qualification stays LIMITED_VALIDITY. E-L0 measures
the registered problems; it does not fix them.

## Identity

| Item | Value |
| --- | --- |
| Approval | user, 2026-10-18 ("MOTAR — EQ-1D 결과 검토 및 E-L0 평가 계보 개발 승인") |
| Preregistration | `docs/plans/e_l0_evaluation_lineage_v1.{json,md}` `37e6b19`; amendment 1 (attachment point) `79d65b6` |
| Approved source | `79d65b6fa51b4de1d09ece7cf60b6c1941f02005` (HEAD, tracked tree clean and frozen during the run) |
| Run | `EL0-R1`, 2026-10-18 02:24:03–02:32:21 KST, `../aerial_gym_simulator/results/e_l0_evaluation_v1/EL0-R1` |
| Raw manifest | 99 members, `RAW_MANIFEST.json` `bbc7365e2bf6b638c71e6aaa26518143a7bac22849b99cd3bfa86461ed45f723`; all re-verified |
| Policy F / RMS | `f7022139…`; digest `642fc9f7…` at start and end of every arm |
| Configurations | OCC-P2 receipts (static E0 `85c13b08…`, moving E2 `b29a14f7…`), 160 bars, seeds 4501/4502/4503 |
| Analysis | `tools/e_l0_analysis.py` → [`e_l0_analysis.json`](e_l0_analysis.json); compact run records in [`run_records/`](run_records/) |

**Resources (measured).**
- GPU time: 497.9 s of exclusive wall time (0.138 GPU-h) across 11 processes.
- GPU memory: peak 4,076–5,440 MiB (N = 128) against the launch-time cap of 7,011 MiB, measured from
  free memory with rustdesk and the display resident.
- Disk: 35.05 MB of raw output; free disk 19.23 → 19.20 GB.
- Development smoke in scratch: about 0.13 GPU-h, not evidence.

## Implementation (no frozen file changed)

`motar_eval/e_l0.py` holds the recorder and pure label functions; `motar_eval/e_l0_kernels.py` holds the
Warp centre-ray query. `motar_eval.e_l0.attach(adapter)` wraps the hook methods of the running KROC2027
adapter instance when `NAVRL_KROC2027_E_L0=1`, and each wrapper still calls the original with the same
arguments.

Amendment 1 explains why: hooks placed inside `kroc_eval_adapter.py` broke the frozen OCC-P2 contract
test. Those edits were reverted before any commit, and the adapter is byte-identical to `5ff285a`.

The recorder keeps three read-only attachments:
- a callback chained onto the env manager's physics step, which records contacts at 100 Hz;
- a call-through wrapper on `perception._associate_lidar_target`, which captures the association;
- direct reads of the detector's own ray table, pose and target mask, plus the LiDAR semantic buffer.

## Gates

| Gate | Result | Evidence |
| --- | --- | --- |
| E0-N1 actor observation | **PASS** | 898-D bytes identical at every step, every env, for P2 vs P0, P2 vs P1 and P1 vs P0: static N = 8 (600 steps), moving N = 8 (600), static N = 128 (320). 0 differing steps |
| E0-N2 deterministic actions | **PASS** | identical, same scope |
| E0-N3 trajectories | **PASS** | pursuer root state and target position/velocity identical |
| E0-N4 outcome and reset sequence | **PASS** | done, capture and crash flags, and reset env ids identical. The task's own bulk `result.json` (N = 128) is byte-identical across all three arms; the OCC trace frames are byte-identical P1 vs P2; the KROC recorder episode records differ only in the arm label |
| E0-N5 RMS | **PASS** | frozen digest at start and end in all 9 arms; model in eval mode |
| E0-N6 sensors and tracker | **PASS** | camera RGB/depth, LiDAR range and semantic buffers, tracker state/cov/age/active identical per step |
| E0-L1 silhouette classes | **PASS** | 27/27 target scenes: E-L0 class, alone/visible pixel counts, centre-ray blockage and frustum flag equal the independent pixel-centre oracle. S08 is now RANGE_EXIT, S12 PARTIAL and S14 CENTRE_ONLY, so EQ-1D's three disagreements are resolved by the new labels |
| E0-L2 LiDAR provenance | **PASS** | 9 accepted associations, each equal to the analytic first hit on the beam used: bar-only N1, N2, N3, N5, N6, N7, N8 → FALSE_ASSOCIATION; A1 → CORRECT; A2 (stale track, target farther on the same beam) → FALSE_ASSOCIATION; A3 / A4 not accepted (NONE) |
| E0-L3 camera provenance | **PASS** | TRUE_TARGET exactly where the detector reports the target; no FALSE_DETECTION; NONE in every bar-only scene |
| E0-C1 contact chronology | **PASS** | E-L0's per-action contact counts and first substeps equal the independent substep recorder for all 24 fixtures. First contact lies inside the SAT window (fixtures 20 / 21 / 23); non-contact fixtures have zero contacts |
| E0-C2 episode columns | **PASS** | every E-L0 outcome equals the launcher's per-step flags. `collision_free_capture` ⇔ capture with zero contact substeps; every crash has an evidence-based cause; `safe_abort` = NOT_IMPLEMENTED in all episodes |
| E0-C3 centre ray versus offline LOS | **PASS** | 36,189 in-frustum frames, 100.0 % agreement with the offline OCC line of sight from the recorded layouts |
| E0-T1 timing | **PASS** | each observation is staged and consumed at the same step counter; E-L0 actions are byte-identical to the launcher's action log |
| E0-T2 tracker age/expiry | **PASS** | E-L0 track age equals the launcher's tracker age exactly; the single expiry (moving) is an active → inactive transition without a reset |

## What E-L0 recorded in the validation runs (counts, not population estimates)

These come from three small validation runs. They are not population rates, and the EQ-1D 7/7
bar-only false accepts remain a known failure of the fixture set, not a rate.

| Run | Episodes | Capture (all collision-free) | Capture with contact | Crash (causes) | LiDAR accepts correct / false | Camera false detections |
| --- | ---: | ---: | ---: | --- | --- | ---: |
| static N = 8, 600 steps | 124 | 123 | 0 | 1 (contact) | 450 / 1 | 0 |
| moving N = 8, 600 steps | 102 | 93 | 0 | 9 (8 contact, 1 below height) | 287 / **73** | 0 |
| static N = 128, 320 steps | 1,038 | 1,011 | 0 | 27 (contact) | 3,987 / 17 | 0 |

Silhouette classes per frame:

| Run | VISIBLE | FOV exit | PARTIAL | FULL | CENTRE_ONLY |
| --- | ---: | ---: | ---: | ---: | ---: |
| static N = 8 | 3,836 | 666 | 128 | 108 | 62 |
| moving N = 8 | 3,083 | 819 | 176 | 616 | 106 |

The moving run shows false LiDAR associations at a scale worth measuring on a full population:
73 of 360 accepts are bars. That is a lead for S-L1 identity work, not a measured rate.

## Limits

- Pixel-centre silhouette at the renderer's resolution, not sub-pixel areas. The renderer's mask
  had at most one pixel outside the analytic alone mask (float32 limb pixels).
- Safe abort is not implemented in Policy F; no abort event is synthesised.
- E-L0 is attached by an evaluation launcher. The standard OCC eval scripts would need a separately
  approved entry point.
- The validation runs are small (N = 8 / 128, 320–600 steps); they qualify the instrument, not the
  environment.
