# History

Milestones only. The day-by-day record, including every run, correction and dead end, is
[`WORKLOG.md`](../WORKLOG.md) (newest entry at the bottom; some entries are appended out of date order,
so trust the heading dates). This page was compiled on 2026-09-24 from WORKLOG headings, the `main`
git history and the result folders; each row names one evidence path.

Dates are result dates. Where a result was committed later, the commit is the recording commit.

## Timeline

### July 2026 — building the task

| Date | Milestone | Evidence |
|---|---|---|
| 07-09 | NavRL specification re-implemented on Aerial Gym: yaw-only 36 × 4 LiDAR, static-goal task | `WORKLOG.md` §2026-07-09 |
| 07-14→15 | Cross-field spawns and learned yaw control solve the static task (capture 0.954, crash 0.046) | `WORKLOG.md` §2026-07-15 |
| 07-16→17 | Ground-truth-LiDAR density sweep: flat to 110 bars, cliff at 150 | `WORKLOG.md` §2026-07-16 |
| 07-17→18 | Pivot to sensor-only pursuit: no ground-truth target in the actor observation | `WORKLOG.md` §2026-07-17 |
| 07-22 | Learned RGB-D + LiDAR perception with a 17-token Transformer policy | `RESEARCH_PLAN.md` §4 |
| 07-27 | 898-D observation (8 obstacle tokens, 72-beam LiDAR) becomes the policy contract | `results/general_repr_fov240_speed_axis.csv` |
| 07-29 | LiDAR bearing-mirror (chirality) bug fixed; on-bar match 13.9 % → 94.8 % | `results/corrected_chirality_density_curve.csv` |
| 07-30 | First density × target-speed map: density, not target speed, dominates difficulty | `results/density_speed_map_cluster_sector.csv` |
| 07-31 | Task v2: 40 × 40 m arena, 6–28 m goals, 60 s episodes | `WORKLOG.md` §2026-07-31 |

### August 2026 — frozen policy F and its limits

| Date | Milestone | Evidence |
|---|---|---|
| 08-01 | PPO actor-collapse root cause fixed (atomic rollback) | `WORKLOG.md` §2026-08-01 |
| 08-02 | 205-bar curriculum plateau; checkpoint ep24000 frozen | `results/navrl_v2_ep24000_heldout/` |
| 08-05 | ep25000 + riskcap speed filter frozen as the navigation candidate (SHA `f7022139…`) | `results/navrl_v2_riskcap_postadapt/` |
| 08-06 | Latency: most of the loss was uncompensated ego-motion; ego-motion correction adopted | `results/navrl_v2_latency_ego_motion/` |
| 08-11→12 | Learned detector navigation non-inferiority PASS, replicated on fresh seeds | `results/navrl_v2_detector_navigation_ab_replication_seed97_101_schema2/` |
| 08-13 | ref5in reference platform (1.2 kg) implemented; its adaptation gate FAILs | `results/navrl_ref5in_d1_eval_seed331/` |
| 08-14→20 | Most away-start timeouts never see the target — first acquisition identified as a failure mode | `results/navrl_ref5in_camera_range_control_seed367/` |
| 08-22 | Research line merged into `main` | `WORKLOG.md` §2026-08-22 |
| 08-25→26 | Physical-target routing gates FAIL; research authority frozen | `docs/research_authority_2026-08-26.json` |
| 08-27 | Lineage break: non-overlapping bars, densities 70–205 | `results/navrl_v2_density_geometry_audit_2026-08-27/` |

### September 2026 — measurement, audit and consolidation

| Date | Milestone | Evidence |
|---|---|---|
| 09-01→02 | Braking-v3 FAIL closes the physical-target GPU authority | `results/navrl_corrected_nonoverlap_physical_off_heldout_seed313/` |
| 09-05 | Safety-filter attribution series (A3–A8); first paper draft | `docs/paper_draft_2026-09-05.md` |
| 09-07 | **Safety geometry:** independent re-computation confirms arc-clearance − riskcap crash −1.4903 pp | `results/independent_verification_2026-09-07/` |
| 09-08 | Perception P4–P7: joint detector, temporal association, streaming pipeline | `results/perception_temporal_p6_p7_2026-09-08/` |
| 09-09 | P8–P10: measured perception error costs frozen policy F −4.57 pp | `results/perception_p10_2026-09-09/` |
| 09-10 | ETH ds5 E3-S: 6.2 % median range-proxy error on real footage; E3-P blocked | `results/eth_ds5_e3s_2026-09-10/` |
| 09-11 | **Perception readaptation:** P10 seed replication INCONCLUSIVE (+0.73 pp) | `results/perception_p10_seed_replication_2026-09-10/` |
| 09-11 | Independent renderer prototype R1–R5 | `docs/renderer_r3_r5_results_2026-09-11.md` |
| 09-12→15 | Public release layer: licences, citation, CPU quick start, public snapshot | `docs/public_release_status_2026-09-14.md` |
| 09-13 | **Renderer sensitivity:** D8b MATERIAL_LOSS −48.967 pp, causality NOT_TESTED | `results/dynamic_mesh_policy_sensitivity_d8b_2026-09-13/` |
| 09-14 | Renderer track frozen at Renderer Contract v1 | `docs/renderer_track_v1_freeze_2026-09-14.md` |
| 09-16 | Target-behaviour ladder TM-E0…TM-E4 defined | `docs/target_behavior_ladder_2026-09-16.md` |
| 09-17→18 | Browser preview frozen: GT_BROWSER_V1, then GT_BROWSER_EPISODE_V1 | `docs/status/gt_browser_episode_v1_freeze_2026-09-18.md` |
| 09-18→19 | **Target motion:** frozen-policy H/E0/E1/E2 evaluation, 98,319 episodes | `docs/results/target_motion_generalization_2026-09-19.md` |
| 09-19 | **Quantitative positioning:** 13 published works, Class A matched comparisons = 0; paper spine | `docs/literature_quantitative_positioning_2026-09-18.md` |
| 09-20→21 | **Elastic Tracker B0:** upstream reproduction FAIL; the remaining blocker is `sudo` for ROS | `docs/external_baselines/elastic_tracker_b0_upstream_reproduction_2026-09-20.md` |
| 09-24 | Repository and site reorganised into a canonical documentation surface; P0 claim audit corrected wording (no measured value changed) | `docs/cleanup/cleanup_record_2026-09-24.md` |
| 09-25→26 | **Target-motion replication:** 7 fresh seeds, bias-corrected quota, 229,376 primary episodes; all three contrasts replicated (E0 −5.37, E1 −3.45, E2 +1.35 pp) | `results/target_motion_replication_7seed/` |

## Negative, withdrawn and blocked results

Kept on purpose; each still has its folder or record. A selection of the most consequential:

| Date | Result | Status |
|---|---|---|
| 07-13 | First capture run learned to loiter | FAIL, reward redesigned |
| 07-20 | "Density curriculum fails" | WITHDRAWN (wrong checkpoint selected) |
| 08-05 | Governor runs with an actor semantic leak | VOID, re-run from scratch |
| 08-12 | Detector robustness did not become system robustness | NEGATIVE |
| 08-13 | ref5in P2 held-out / D1 adaptation gates | FAIL; P3 full budget BLOCKED |
| 08-22→23 | Detection-range stage 1 | RANGE_INCONCLUSIVE_AT_THIS_BUDGET |
| 08-25→09-01 | Routed physical-target gates (four rounds) | FAIL_ROUTE_MECHANISM |
| 09-02 | Detector colour shortcut with distractors | COLOR_SHORTCUT_CONFIRMED |
| 09-05→06 | Five safety-filter runs with source or HEAD drift | VOID ×5 |
| 09-09 | C3 "co-adaptation decides the law" explanation | WITHDRAWN (0/2 training seeds) |
| 09-10 | E3-P attitude decomposition | ATTITUDE_NOT_RELIABLE → BLOCKED |
| 09-11 | P10 readaptation | INCONCLUSIVE |
| 09-11 | Renderer R4 / R4b criterion C | FAIL |
| 09-13 | D6 standalone mesh cost | INCONCLUSIVE |
| 09-13 | D8b frozen-policy sensitivity | MATERIAL_LOSS, causality NOT_TESTED |
| 09-19 | `NO_RETRAIN_NEEDED` verdict and its "preregistered 5 pp" threshold | WITHDRAWN; threshold NOT_PREREGISTERED |
| 09-19 | Preregistered primary `min_relative_distance_m` | NOT_RECORDED |
| 09-20→21 | Elastic Tracker B0 | UPSTREAM_REPRODUCTION_FAIL |

`results/VOID_*` and `results/*_VOID_*` folders hold the voided runs with their original receipts.

## Lineage breaks

Results are comparable only within a lineage.

| Date | Break | Consequence |
|---|---|---|
| 07-10 | Success radius 1.0 → 0.5 m | Earlier reach metrics are not comparable |
| 07-15 → 07-27 | Actor observation 152 → 156 → 305 → 1265/574 → **898** | Checkpoints of different widths never mix |
| 07-29 | LiDAR chirality fix | Older checkpoints cannot be warm-started |
| 07-31 | Task v1 → v2 (24 → 40 m arena, 300 → 600-step episodes) | v1 numbers are historical only |
| 08-13 | ref5in platform added beside the legacy quadrotor | D8b uses a ref5in checkpoint, not ep25000 |
| 08-26→27 | Corrected v2: non-overlapping bars, 70–205 density lineage | Pre-correction results are historical only |
| 09-17 | Frozen ep25000 training contract differs from HEAD defaults in 14 settings | Reproduce it only through its recorded launcher environment |

Target-motion lineages (legacy → arm H, bounded → arm E2, physical, browser preview) are explained in
[`target_motion_algorithm_2026-09-17.md`](target_motion_algorithm_2026-09-17.md).

## Where older material lives

- Superseded plans, handoffs and notes: [`docs/archive/`](archive/README.md).
- Frozen crash-tuning notes (July): [`CRASH_TUNING_LOG.md`](../CRASH_TUNING_LOG.md), kept in place because source files cite it.
- The verification ledger with every gate and its authority: [`VERIFICATION.md`](../VERIFICATION.md) (Korean).

The target-motion chronology remains separate: original n=3 (18 September, legacy window);
bias discovery and canary (instrumentation validation, not scientific evidence); independent n=7
replication (25–26 September, per-environment quota, CURRENT). The two campaigns are not pooled.
