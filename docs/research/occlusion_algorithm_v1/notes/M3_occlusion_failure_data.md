# M3: What MOTAR already measures about obstacle density, target motion, occlusion and failure (Policy F)

Audit date 2026-10-19. Repository `MOTAR-mt-p1`, branch `codex/mt-p1`, HEAD `1d4696c`. The audit was read-only: no simulator or GPU ran, and no repository or raw file changed.
Raw root: `/home/fair/workspaces/aerial_gym_ws/src/aerial_gym_simulator/results/` (abbreviated `RAW/` below).
Repository paths are relative to `MOTAR-mt-p1/`. Derived numbers come from CPU-only scripts in
`occ_research/motar/scripts/`, with outputs beside this file (`*.out`, `*.json`). Every
relation below is an association in the frozen closed loop, not a causal effect.

**Common contract of every source used.** Every source uses frozen Policy F (`f7022139…3ad40`), the 898-D actor, RGB-D+LiDAR, riskcap, the 40 m arena, a 600-action timeout and 0.1 s actions.
- **Placement lineage.** Every cited cell uses the corrected 2026-08-27 lineage, where bars do not overlap: `NAVRL_PLACEMENT_MODE=navrl_band`, gap 1.6 m, touch 0.4 m, `bars_h3`. I checked this in receipts for the 3-seed and 7-seed target-motion campaigns, KROC-A and OCC-P2.
- **No overlapping-placement source.** No occlusion or target-motion number here comes from the earlier overlapping-placement lineage, which `docs/HISTORY.md` marks "historical only".
- **Touching bars are intended.** In-cell touching bar pairs are deliberate compound walls (EQ0-08), not overlap defects.

## 0. Lineage key (needed for pooling)

| Tag | Target model / backend | Start condition | Seeds | Episode estimand | TF32 status (`docs/audits/moving_target_tf32_readonly_2026-10-16.md`) |
|---|---|---|---|---|---|
| **TM3** | H / E0 / E1 / E2 (`bounded_planar_drone_v1_rollout`, historical cdist) | random view, goal 6–28 m | 4101–4103 | legacy pooled window | H/E1/E2 `TF32_EXPOSED_VALIDITY_UNVERIFIED`; E0 bypasses the moving executor, but E0−H inherits H |
| **TM7** | same as TM3 | same | 4104–4110 | quota 16 per env (primary); legacy window as diagnostic | same as TM3 |
| **KA** | A1 E0 static visible; A2 E0 hidden; A3/A4 `E2_direct_l2_v1` visible/hidden | fixed 10 m; initial view visible or hidden | 4201–4207 | quota 16 | A1/A2 not exposed (static); A3/A4 `DIRECT_LINEAGE_VALIDATED` (0 violations) |
| **KB** | B0 = A2 contract; B1 = B0 plus external yaw-scan supervisor | 10 m hidden | 4301–4307 | quota 16 | not exposed (static) |
| **OCC-P1** | A1 / A3 with occlusion telemetry | 10 m visible | 4201–4207 | quota 4 | as KA; **every episode is the same episode as in KA** (see §1.4) |
| **OCC-P2** | A1 / A3 with occlusion telemetry | 10 m visible | 4301–4314 | quota 4 | static not exposed; moving `direct_l2_v1` with stop-on-violation. All 112 cells were accepted (zero-violation inferred, not separately attested) |
| **E-L0** | E0 / E2_direct with silhouette classes | 10 m visible, 160 bars | 4501–4503 | validation runs (N = 8 / 128) | as OCC |
| **H-R3** | E2_direct, matched branch arms B0–B5 | OCC-P2 moving receipt, 160 bars | P01 4502; P02 4601–4602; P03-Q 4801 | first post-acquisition loss per env | as OCC |

## 1. Table of every existing measurement linking density or motion to occlusion and outcomes

### 1.1 Occlusion and visibility-loss frequency, and duration, by density × motion

**OCC-P2 confirmatory.** Sources: `docs/evidence/occ_p2_confirmatory_2026-10-17/REPORT.md`, `summary.csv`, `analysis.json`.
- **Cell.** Group × density, with 14 seeds × 128 envs × quota 4 = **7,168 episodes per cell**.
- **Primary indicator.** `center_occlusion_in_view`: the offline 3-D centre ray to the target is blocked while the target is inside the camera frustum and range.
- **Rate.** Intervals per minute of post-first-fused-acquisition observation, as the equal-weight mean of seed rates.
- **CI.** Seed-block percentile bootstrap, 95 %.

| Group | Bars | Rate / min [95 % CI] | Strict clear→blocked onsets / min | Intervals | Median / P95 observed duration (s) | Geometry restored / all | Fused resumption after restoration | 2 s CIF: crash / FOV-exit (all intervals) | 2 s crash CIF, incident-only (clear→blocked) |
|---|---:|---|---:|---:|---|---|---|---|---|
| static | 70 | 1.041 [0.962, 1.121] | 0.876 | 436 | 0.8 / 1.7 | 350/436 | 350/350 | 1.61 % / 18.12 % | 1.09 % |
| static | 115 | 1.933 [1.762, 2.122] | 1.550 | 823 | 0.7 / 1.6 | 636/823 | 636/636 | 2.31 / 20.41 | 2.42 |
| static | 160 | 3.116 [2.947, 3.274] | 2.408 | 1,351 | 0.7 / 1.6 | 1025/1351 | 1025/1025 | 4.15 / 19.99 | 4.50 |
| static | 205 | 4.167 [3.975, 4.361] | 3.061 | 1,848 | 0.7 / 1.5 | 1315/1848 | 1315/1315 | 4.92 / 23.92 | 5.53 |
| moving E2 | 70 | 3.539 [3.348, 3.749] | 3.007 | 1,663 | 0.9 / 2.0 | 1423/1663 | 1423/1423 | 1.56 / 12.21 | 1.56 |
| moving E2 | 115 | 5.886 [5.673, 6.110] | 4.732 | 2,965 | 0.9 / 2.0 | 2446/2965 | 2446/2446 | 1.82 / 15.11 | 2.01 |
| moving E2 | 160 | 7.826 [7.666, 7.978] | 5.877 | 4,084 | 0.9 / 2.1 | 3197/4084 | 3197/3197 | 2.94 / 18.12 | 3.23 |
| moving E2 | 205 | 9.616 [9.445, 9.828] | 6.683 | 5,315 | 0.9 / 2.1 | 3865/5315 | 3865/3865 | 3.48 / 23.14 | 3.82 |

**Confirmatory contrasts.** One-sided paired t tests (df 13) in one Holm family of six; each is 14/14 seeds positive.

| Contrast | Difference / min | 95 % CI | Holm p |
|---|---:|---|---:|
| static 205−70 | +3.126 | [2.893, 3.359] | 5.0e-13 |
| moving 205−70 | +6.078 | [5.790, 6.366] | 3.0e-15 |
| moving−static, 70 bars | +2.498 | [2.273, 2.723] | — |
| moving−static, 115 bars | +3.953 | [3.630, 4.276] | — |
| moving−static, 160 bars | +4.711 | [4.467, 4.955] | — |
| moving−static, 205 bars | +5.449 | [5.078, 5.821] | 2.1e-13 |

**Decisions.** RQ1 and RQ3 are SUPPORTED. RQ2 (duration) is `INCONCLUSIVE_EQUIVALENCE_NOT_TESTED`. The seed-median duration change from 205 to 70 bars is −0.125 s [−0.239, −0.011] for static and −0.021 s [−0.079, +0.036] for moving.

**Tails and terminal outcomes.**
- No interval reached 5 s, and the 5 s tail is `NOT_ESTIMABLE` (risk set 0).
- No capture or timeout ever ended an active centre-occlusion interval.

**Restricted windows (seed-equal, intervals per minute).** In the first 1 s / 2 s / 5 s after acquisition, the moving-205 rates are 20.2 / 16.7 / 10.5 against 9.6 overall. Occlusions are therefore concentrated just after acquisition, which here is at spawn.

**OCC-P1 exploratory (n = 7 seeds, 3,584 episodes per cell).** Source: `docs/evidence/occ_p2_pre_execution_2026-10-17/OCC-P1_rerun1/analysis.json` (`groups`).
- **Rates.** Per minute of post-acquisition exposure.
- **Interval.** Episode-cluster bootstrap, 1,000 resamples. This is not a seed-level interval.
- **Indicators are never merged.**

| Group | Bars | centre_occ / min [boot] | geometric_full / min | out_of_view / min | fused_loss / min | Share of episodes with a centre interval | Median centre / OOV / fused duration (s) |
|---|---:|---|---:|---:|---:|---:|---|
| static | 70 | 1.05 [0.91, 1.21] | 0.66 | 33.16 | 9.47 | 0.053 | 0.7 / 0.3 / 0.1 |
| static | 115 | 2.04 [1.84, 2.25] | 1.30 | 38.31 | 15.39 | 0.108 | 0.7 / 0.3 / 0.1 |
| static | 160 | 2.80 [2.55, 3.03] | 1.87 | 41.31 | 21.16 | 0.142 | 0.7 / 0.3 / 0.1 |
| static | 205 | 4.02 [3.73, 4.31] | 2.56 | 42.75 | 25.70 | 0.199 | 0.7 / 0.3 / 0.1 |
| moving | 70 | 3.77 [3.52, 4.01] | 3.31 | 27.53 | 11.22 | 0.207 | 0.9 / 0.3 / 0.2 |
| moving | 115 | 6.00 [5.70, 6.30] | 5.31 | 29.98 | 17.81 | 0.320 | 0.9 / 0.3 / 0.2 |
| moving | 160 | 7.93 [7.63, 8.24] | 6.91 | 31.27 | 23.25 | 0.427 | 0.9 / 0.3 / 0.2 |
| moving | 205 | 9.56 [9.24, 9.91] | 8.19 | 31.61 | 26.76 | 0.517 | 0.9 / 0.3 / 0.2 |

**Endings of out-of-view intervals.** Many end in capture, about 2,100–2,200 per static group and 1,280–1,500 per moving group. The target leaves the FOV during the terminal approach, so `out_of_view` mixes loss with the endgame.

**OCC-P1 geometry-only kinematic sweep.** `RAW/occlusion_pilot_v1/OCC-P1_rerun1/geometry_sweep.json` uses seed-4201 layouts only, ignores collisions and is labelled `GEOMETRY_ONLY_KINEMATIC_BASELINE`. Rates are centre-occlusion intervals per minute.

| Scripted family | 70 bars | 115 | 160 | 205 | Duration |
|---|---:|---:|---:|---:|---|
| Static pursuit | 0 | 0 | 0 | 0 | — (no re-occlusion is possible geometrically) |
| Static crossing | 19.9 | 24.2 | 25.3 | 24.2 | P95 3.9 s at 70 rising to 6.5 s at 205 |
| Moving pursuit at 1.5 m/s | 6.6 | 10.1 | 13.0 | 15.3 | median 0.9 s |

Policy F's closed-loop rates are below the kinematic moving-pursuit baseline. Longer spells (P95 above 5 s) exist only in the scripted crossing family.

**Observation time shares, derived.** Script `obs_time_shares.py`, output `obs_time_shares.out`. These are per-episode counters over acquired episodes in OCC-P2 (similar in KA A1/A3).

| Group | Fused-invalid share after acquisition, 70→205 | Out of FOV/range | Centre-occluded in view |
|---|---|---|---|
| static | 3.1 % → 10.4 % | 18.9 % → 33.3 % | 1.4 % → 4.8 % |
| moving E2 | 6.7 % → 20.8 % | 17.7 % → 28.8 % | 5.5 % → 14.8 % |

**TM3 / TM7 random-start campaigns.**
- **Recorded.** Only pooled step-weighted visibility and visible→hidden transition counts. Loss duration, maximum loss duration and track age are `NOT_RECORDED` (`docs/results/target_motion_visibility_reacquisition_audit_2026-09-19.md`).
- **TM3 visibility, overall.** E0 0.2202, E1 0.2502, H 0.2889, E2 0.3215.
- **TM3 visibility in timeout episodes.** E0 0.0016, E1 0.0059, H 0.0079, E2 0.0254 (step-weighted, length-biased).
- **TM7 visible→hidden transitions per capture episode** (legacy window, derived in `tm7_density.out`). These rise with density in every arm:

| Arm | 70 bars | 205 bars |
|---|---:|---:|
| E0 | 1.32 | 2.20 |
| E1 | 1.49 | 2.43 |
| E2 | 1.49 | 2.38 |
| H | 1.48 | 2.47 |

### 1.2 Occlusion-type mix (E-L0 silhouette classes)

Classes are FULL, CENTRE_ONLY, PARTIAL, FOV_EXIT and RANGE_EXIT (E-L0), plus SCALE_LOSS and NONE (H-R3 only).

**E-L0 frame counts at 160 bars.** Source: `docs/evidence/e_l0_evaluation_2026-10-18/e_l0_analysis.json` (`V3.*.class_frames`). These are validation counts, not rates. RANGE_EXIT never occurs in a run frame.

| Run | VISIBLE | FOV_EXIT | PARTIAL | FULL | CENTRE_ONLY |
|---|---:|---:|---:|---:|---:|
| static, N = 8, seed 4501, 600 steps | 3,836 | 666 | 128 | 108 | 62 |
| moving E2, N = 8, seed 4502 | 3,083 | 819 | 176 | 616 | 106 |
| static, N = 128, seed 4503, 320 steps (40,960 frames; omitted from the REPORT table) | 32,251 | 5,899 | 1,266 | 1,095 | 449 |

**H-R3 first-loss trigger populations, moving E2, 160 bars only.**

| Run | Triggers | FULL | CENTRE_ONLY | PARTIAL | FOV_EXIT | VISIBLE miss | RANGE_EXIT / SCALE_LOSS / NONE | Source |
|---|---:|---:|---:|---:|---:|---:|---|---|
| P01, seed 4502 (107 of 125 matched) | 107 matched | 18 | 4 | 0 | 83 | 2 | 0 | `docs/evidence/h_r3_pilot_2026-10-18/REPORT.md` |
| P02, seeds 4601/4602 (integrity FAIL) | 246 reference | 44 | 12 | 0 | 181 | 9 | 0 | `docs/evidence/h_r3_p02_2026-10-18/decision.json`, `analysis.json:counts` |
| P03-Q01, seed 4801 | 121 (all matched) | 21 | 5 | 0 | 95 | — | 0 | `docs/evidence/h_r3_p03_preparation_2026-10-19/REPORT.md` |

- **FOV_EXIT share of first losses:** 74–79 %.
- **Geometric share (FULL + CENTRE_ONLY):** 21–23 %.
- **PARTIAL:** frames exist (E-L0), but a PARTIAL frame never triggered a camera loss.

### 1.3 Episode outcomes by density × motion × start condition (capture = 0.5 m swept closest approach, KROC `capture_0p5m`)

**KA, KB and OCC-P2 per-episode outcomes.** Derived from raw `kroc2027_episodes.jsonl` by script `kroc_episode_failure.py` / `kroc_b_failure.py`; outputs `kroc_episode_failure.json` and `kroc_b_failure.json`.
- **Cross-check.** Arm totals reproduce `docs/evidence/mt_precision_review_2026-10-16/numerical_audit.json` exactly: A1 56,288, A2 54,659, A3 54,697 and A4 53,218 captures.
- **Cell size.** n = 14,336 per KA/KB cell (7 seeds × 2,048) and 7,168 per OCC-P2 cell.
- **Never-acquired share** means the share of failures (crash + timeout) in which the target was never fused-acquired.
- Each density cell reads capture / crash / timeout %, then that never-acquired share in brackets.

| Arm | 70 bars | 115 bars | 160 bars | 205 bars |
|---|---|---|---|---|
| KA-A1 static, visible | 99.39 / 0.61 / 0 [0 %] | 98.97 / 1.03 / 0 [0 %] | 97.90 / 2.10 / 0 [0 %] | 96.37 / 3.63 / 0 [0 %] |
| KA-A2 static, hidden | 97.83 / 2.16 / 0.01 [57.6 %] | 96.78 / 3.15 / 0.07 [59.9 %] | 94.80 / 4.74 / 0.46 [57.9 %] | 91.85 / 7.38 / 0.77 [60.5 %] |
| KA-A3 E2 direct, visible | 98.62 / 1.37 / 0.01 [0 %] | 97.16 / 2.84 / 0 [0 %] | 94.75 / 5.24 / 0.01 [0 %] | 91.01 / 8.96 / 0.03 [0 %] |
| KA-A4 E2 direct, hidden | 96.82 / 3.18 / 0 [42.5 %] | 95.21 / 4.74 / 0.05 [41.2 %] | 91.94 / 7.94 / 0.12 [32.2 %] | 87.25 / 12.61 / 0.14 [36.1 %] |
| KB-B0 static, hidden | 97.82 / 2.15 / 0.03 [59.0 %] | 96.72 / 3.12 / 0.15 [60.9 %] | 95.55 / 4.14 / 0.31 [60.0 %] | 92.24 / 6.97 / 0.79 [62.2 %] |
| KB-B1 with yaw-scan | 98.10 / 1.90 / 0 [69.1 %] | 97.13 / 2.82 / 0.05 [68.1 %] | 95.42 / 4.30 / 0.29 [60.6 %] | 92.52 / 6.85 / 0.63 [60.2 %] |
| OCC-P2 static, visible (quota 4) | 99.65 / 0.35 / 0 | 98.94 / 1.06 / 0 | 97.67 / 2.33 / 0 | 95.73 / 4.24 / 0.03 |
| OCC-P2 moving E2, visible (quota 4) | 98.55 / 1.45 / 0 | 97.47 / 2.51 / 0.01 | 94.50 / 5.48 / 0.01 | 90.93 / 9.05 / 0.01 |

**First-acquisition latency.**

| Arm | Median latency, 70→205 bars | P90 latency, 70→205 bars |
|---|---|---|
| Visible start | 0 s | — |
| A2 / B0 hidden | 1.2 → 1.5 s | 1.7 → 8.0 s |
| A4 hidden | 1.2 → 1.7 s | 1.9 → 8.3 s |
| B1 yaw-scan | 2.1 → 8.3 s | 9.4 → 16.3 s |

**Committed contrasts.**

| Contrast | Value | Status | Source |
|---|---|---|---|
| Delta_A (KA) | +2.71 pp [2.45, 2.97] | SUPPORTED, below the 3 pp practical threshold | `mt_precision_review_2026-10-16/README.md` |
| I_A (KA) | −0.26 pp [−0.80, 0.28] | INCONCLUSIVE | same |
| B1−B0 capture | +0.21 pp [−0.09, +0.50] | INCONCLUSIVE, B1 not adopted | `RAW/kroc2027_B_v1/analysis.json` |

**TM7, quota-16 primary outcomes.** From the committed `results/target_motion_replication_7seed/cells.csv`; legacy-window never-acquired shares are from raw `result.json` (output `tm7_density.out`).
- Each density cell reads capture / crash / timeout %, then the share of failures never acquired in the legacy window, in brackets.
- n = 14,336 per cell (7 seeds).

| Arm | 70 bars | 115 bars | 160 bars | 205 bars |
|---|---|---|---|---|
| E0 | 84.55 / 6.31 / 9.14 [90.5 %] | 84.11 / 6.24 / 9.65 [88.6 %] | 82.37 / 8.20 / 9.43 [85.1 %] | 78.54 / 13.99 / 7.48 [80.2 %] |
| E1 | 87.21 / 6.79 / 5.99 [79.1 %] | 86.90 / 7.47 / 5.63 [72.1 %] | 84.05 / 10.02 / 5.94 [66.8 %] | 79.10 / 16.58 / 4.32 [61.5 %] |
| E2 (bounded, TF32-exposed) | 93.53 / 5.40 / 1.07 [65.4 %] | 92.11 / 6.65 / 1.23 [56.0 %] | 87.67 / 9.87 / 2.46 [58.9 %] | 83.15 / 15.55 / 1.30 [50.2 %] |
| H (TF32-exposed) | 91.59 / 5.32 / 3.09 [77.7 %] | 90.16 / 6.69 / 3.15 [69.5 %] | 86.98 / 9.28 / 3.73 [63.3 %] | 82.31 / 14.48 / 3.21 [61.6 %] |

**TM7 confirmatory contrasts.** Seed-paired BCa, n = 7.

| Contrast | Mean | BCa 95 % | Holm p |
|---|---:|---|---:|
| E0−H | −5.37 pp | [−5.78, −4.85] | 0.047 (resolution floor) |
| E1−H | −3.45 pp | [−3.79, −3.11] | 0.047 |
| E2−H | +1.35 pp | [+1.07, +1.87] | 0.047 |

E2−H keeps its sign in only 5/7 seeds at 160 bars and 4/7 at 205 (exploratory).

**TM3, original campaign.** Legacy window, n = 3.
- **Contrasts.** E0−H −3.58 pp, E1−H −2.82 pp, E2−H +1.33 pp.
- **Never-acquired share in timeouts.** E0 95.42 %, E1 83.01 %, H 84.11 %, E2 84.38 %.
- **Never-acquired share in crashes.** E0 78.05 %, E1 60.69 %, H 61.74 %, E2 54.38 %.
- **Sources.** `docs/results/target_motion_generalization_2026-09-19.md`, `…visibility_reacquisition_audit_2026-09-19.md`.

### 1.4 Pooling flags

**Can be pooled (descriptively):**
- **OCC-P1 with OCC-P2.** Same contract and quota 4, disjoint seeds, giving 21 seeds. P1 is exploratory and P2 confirmatory, so a pool is descriptive only.
- **KA-A2 with KB-B0.** Same hidden-static contract on disjoint seeds 4201–4207 and 4301–4307, at quota 16.

**Cannot be pooled:**
- **OCC-P1 with KA-A1/A3.** OCC-P1 is not independent of KA-A1/A3: all 14,336 OCC-P1 static scene hashes equal KA-A1's first-four episodes (`scene_overlap.out`), and `pilot_report.json` shows ledger MATCH in 56/56 cells. They are the same episodes.
- **OCC-P2 with KB.** They partly share layouts: 3,584 of 14,336 OCC-P2 static scene hashes also occur in KB-B0 at the same seed and density, consistent with one layout per env per cell. The OCC-P2 duplicate screen checked only against OCC-P1. Treat B and P2 as one cluster if they are ever combined.
- **OCC-P2 static with moving.** The groups share 7,189 scene hashes within seed by design; these are matched clusters.
- **TM3/TM7 with KA/KB/OCC.** Start geometry (random view, 6–28 m goal versus fixed 10 m), target backend (TF32-exposed bounded versus `direct_l2_v1`) and estimator (legacy window versus quota) all differ. "E2" names two different lineages.
- **TM3 with TM7.** Different estimators and seeds; the replication README forbids a combined p.
- **H-R3 P01, P02 and P03 with each other or with confirmation.** P01 is exploratory, P02 is integrity FAIL and P03 is qualification only. Their metrics also differ: P01 hands control back early on detection, while P02/P03 use a fixed 20-step exposure and the spatial identity metric. All three are excluded from confirmation.
- **E-L0 counts.** These are instrument-validation counts, not rates. Seed 4502 is reused by E-L0 (N = 8) and H-R3 P01 (N = 128); scene overlap was not checked (`NOT_VERIFIED`).
- **KROC original A3/A4 canary.** The TF32 cells at 160 bars, seed 4190, are `OBSERVED_INVALID` and must not be used.

## 2. Share of capture failures associated with occlusion versus other causes

**Partly measured.** No record attributes a failure to occlusion causally (`NOT_TESTED`). What exists:

1. **Visible start (KA-A1/A3, OCC-P2).**
   - **Failures are crashes.** Timeouts are 0–5 per 14,336 episodes, and never-acquired failures are 0, apart from one static-205 episode in OCC-P2.
   - **Tracker expiry plays no part.** It is open at a failure in 0–5 of up to 1,289 acquired failures per cell (`acq_fail_track_lost_at_end`).
   - **Fused loss at the end.** The fused measurement was invalid at the terminal observation in 30–43 % of failures, against a 3–21 % time share of fused-invalid observation after acquisition.
2. **Crash during an active interval (OCC-P2, derived).** These are interval end causes from `analysis.json` (script `p2_crash_assoc.py`, `p2_seed_level.py`). The centre-occluded and out-of-view states are mutually exclusive, so their shares add.

   | Group | Crashes | Share with a centre-occlusion interval open at the crash (time share of that state) | Share out of view or range | Share not geometrically visible (sum) |
   |---|---:|---|---:|---:|
   | static 70 | 25 | 28.0 % (1.4 %) | 32.0 % | 60.0 % |
   | static 115 | 76 | 25.0 % (2.3 %) | 18.4 % | 43.4 % |
   | static 160 | 167 | 33.5 % (3.9 %) | 15.0 % | 48.5 % |
   | static 205 | 304 | 29.9 % (4.8 %) | 16.8 % | 46.7 % |
   | moving 70 | 104 | 26.9 % (5.5 %) | 40.4 % | 67.3 % |
   | moving 115 | 180 | 31.1 % (9.1 %) | 26.1 % | 57.2 % |
   | moving 160 | 393 | 31.8 % (12.2 %) | 28.5 % | 60.3 % |
   | moving 205 | 649 | 29.6 % (14.8 %) | 37.3 % | 66.9 % |

   - **Seed spread.** At moving 205 the share of crashes during an open centre interval is 0.294 ± 0.089 (SD, n = 14 seeds), and crash-ended intervals per interval are 0.036 ± 0.012.
   - **Enrichment.** Crashes occur during in-view centre occlusion about 2–20× more often than its time share predicts.
   - **Why this is not causal.** A pursuer at or next to a bar has that bar between camera and target, so contact geometry itself produces occlusion.
   - **Not recorded.** Whether an occlusion preceded the crash within k seconds (look-back) is `NOT_RECORDED` in the compact records; it is derivable only from the raw traces.
3. **Hidden start (KA-A2/A4, KB).**
   - 58–62 % (static) and 32–43 % (E2) of failures are never-acquired. Most of these are crashes before first acquisition.
   - Also from the gate: `never_acquired_failure_share` = 0.593 (`RAW/kroc2027_A_v1/analysis.json`).
   - This is a first-acquisition failure, not post-acquisition occlusion.
4. **Random start (TM3/TM7).** 50–91 % of failures were never acquired (legacy window). In TM3, crash episodes were never acquired in 54–78 % of cases.

The fraction of failures caused by post-acquisition occlusion is therefore `NOT_RECORDED`. The associations above are what exists.

## 3. FOV_EXIT versus geometric occlusion, and the yaw versus translation finding

- **Frequency.**
  - **OCC-P1.** Out-of-view intervals occur at 27–43 per minute against 1–9.6 per minute for centre occlusion. They are short (median 0.3 s), and many end in capture.
  - **H-R3.** Among first post-acquisition camera losses at moving 160 bars, 74–79 % are FOV_EXIT and 21–23 % geometric.
  - **OCC-P2.** FOV or range exit ends 12–24 % of centre-occlusion intervals within 2 s; it competes with restoration rather than restoring the view.
- **H-R3 arms.** B0 is Policy F; B1 is scripted prediction pursuit at 1.0 m/s; B2 adds yaw; B3 adds a LOS-chosen lateral translation; B4 uses a sham side; B5 matches nominal speed.

**FOV_EXIT events: yaw is what recovers them.**

| Run | Events | B0 | B1 | B2 | B3 | Yaw effect (B2−B1) | Translation effect (B3−B2) |
|---|---:|---:|---:|---:|---:|---|---|
| P01 (corrected) | 83 | 82 | 32 | 82 | 81 | +60.2 pp (50/0 discordant) | −1.2 pp |
| P02 (nominal, invalid) | 181 | 179 | 74 | 178 | 178 | — | — |
| P03-Q01 | 95 | 91 | 34 | 91 | 89 | — | — |

Policy F already reobserves 96–99 % of FOV_EXIT events.

**Geometric events (FULL + CENTRE_ONLY): translation, not yaw, makes the difference.**

| Run | Events | B0 | B1 | B2 | B3 | B4 | B5 | Contrasts |
|---|---:|---:|---:|---:|---:|---:|---:|---|
| P01 | 22 | 18 | 7 | 9 | 17 | — | — | B2−B1 +9.1 pp; B3−B2 +36.4 pp [+13.6, +59.1], McNemar p = 0.021 uncorrected; B3−B0 −4.5 pp |
| P02 (integrity FAIL, invalid) | 56 | 46 | 21 | 34 | 37 | 43 | 32 | none valid |
| P03-Q01 (descriptive) | 26 | 21 | 13 | 14 | 22 | 18 | 13 | B3−B2 +30.8, B3−B4 +15.4, B3−B5 +34.6 pp; no population inference |

- **Attribution.** "Pure viewpoint" attribution is INCONCLUSIVE. Realized path and speed are imbalanced by 20–30 % (P03 balance diagnostic), and B2 stalls behind bars under the shared safety filter.
- **Policy F on geometric events.** It reobserves 77–82 % of them within 2 s.
- **Capture and safety cost of the scripted arms.**
  - Capture by 5 s favours Policy F: P01 B0 11/22 against B3 6/22; P03 B0 13/26 against 3–4/26.
  - Only B0 crashes within the 2 s windows: P03 2/26 against 0 for every scripted arm.
  - B0 has the closest LiDAR minimum: 0.68–0.72 m against 0.87–1.09 m for the scripted arms.
- **Mechanics.** ISO-Q02 and P03-Q01 qualified selected-only reset isolation only for their registered profiles (seed 4602 at 40 steps; seed 4801 at 600 steps).

## 4. Gaps

**Cells never measured with occlusion telemetry.** OCC indicators, durations and end causes are not recorded for:
- E1 CV (the direct lineage has only the seed-4190 canary);
- H historical;
- hidden-start A2/A4/B0/B1;
- the random-start TM3/TM7 campaigns;
- E2 at target speeds other than the KROC setting;
- densities outside 70–205, including zero bars;
- TM-E3/E4, which are PLANNED.

**Gaps in the E-L0 class mix.**
- It exists only at 160 bars, as validation counts.
- It has never been measured at 70, 115 or 205 bars, or for static targets in H-R3.
- PARTIAL, RANGE_EXIT and SCALE_LOSS loss events have zero counts in every run, so they are `NOT_TESTED`.
- Geometric events have been studied only in early first-loss windows; P01, for example, is limited to t0 ≤ 10.

**Metrics a new algorithm study must add or carry.**

| Metric | Status today |
|---|---|
| Time to first acquisition | Recorded only in KROC hidden-start and TM legacy data |
| Occlusion onset to crash look-back and lead time | `NOT_RECORDED` |
| Episode-level competing-risk analysis | Capture, crash and timeout cumulative incidence over time since acquisition, stratified by occlusion exposure: `NOT_RECORDED`. OCC has only per-interval Aalen–Johansen |
| Latent physical occlusion duration, beyond censor-selected elapsed duration | `NOT_RECORDED`; ≥ 5 s is `NOT_ESTIMABLE` |
| Recovery after tracker expiry (> 5 s) | `NOT_ESTIMABLE`; essentially never occurs |
| Persistent identity | Tracker "fresh" is not identity |
| LiDAR false-association rate | 73/360 is a validation count only |
| Obstacle-only clearance | `NOT_RECORDED` (P03) |
| Actuator energy | `NOT_RECORDED` |
| Safe abort | `NOT_IMPLEMENTED` / `NOT_RECORDED` |
| Realized-motion matching for any viewpoint claim | Not achieved (P02/P03) |

## 5. Recommended primary failure target

**Primary target.** The cumulative incidence of crash within 2 s of a post-acquisition geometric occlusion onset, in moving E2_direct_l2_v1 at 160 and 205 bars.
- **Population.** Incident clear→blocked `center_occlusion_in_view` intervals.
- **Estimator.** Aalen–Johansen, with geometric restoration, FOV/range exit, capture and timeout as competing events.
- **Inference.** Equal-weight seed means with seed-block inference.
- **Policy F baseline (OCC-P2).**
  - Incident-only: 3.23 % at 160 bars and 3.82 % at 205.
  - All intervals: 2.94 % and 3.48 %.
  - About 290–380 intervals per seed.
  - Crash-ended intervals per interval at 205: 0.036 ± 0.012 SD across seeds.

**Required co-endpoints (guards).**
- Episode capture and crash at every density. Moving 205 capture is 90.93 % and crash 9.05 %.
- 2 s FOV-exit CIF (23 %).
- Fused resumption after restoration (100 % today).
- Capture by 5 s after onset. H-R3 shows that reobservation-seeking arms can lower it.

**Secondary endpoint.** H-R3-style spatially identity-correct reobservation within 2 s after FULL/CENTRE_ONLY first loss. Policy F scores 77–82 %. This needs the qualified isolation runner and a fresh preregistration.

**Reasons.**
1. Visible-start failures are almost entirely crashes. About 30 % of crashes happen during in-view centre occlusion, which occupies only 1.4–15 % of acquired time; 43–67 % happen while the target is not geometrically visible.
2. Several conventional "occlusion recovery" targets have no measured headroom:
   - fused resumption after restoration is 100 %;
   - Policy F already reobserves 96–99 % of FOV exits;
   - tracker expiry is about 0;
   - no natural 5 s occlusion was observed.
3. The endpoint occurs naturally and in volume (4,084 and 5,315 intervals in 14 seeds), so it needs no matched-branch machinery.
4. Its definition and analysis tool are already frozen (`tools/occlusion_confirmatory_analysis.py`, preregistration v1).
5. H-R3 windows show Policy F with the nearest LiDAR minima and the only in-window crashes.

**Risks.** The crash–occlusion association can be reverse or common-cause (bar proximity), so a gain must also appear in total episode crash, not only in the conditional incidence. Planning power from P2 alone is an assumption: with an SD of 0.012 per seed and no pairing, a 1 pp absolute reduction needs roughly 30 seeds per arm.

**The hidden-start never-acquired gap is the larger failure share.** It accounts for 32–62 % of failures, but it is a first-acquisition problem; B1 yaw-scan was INCONCLUSIVE at +0.21 pp. Treat it as a separate target.

**Matched reuse for a fair comparison.** Use the OCC-P2 contract:
- Policy F `f7022139…`, A1 E0_static and A3 `E2_direct_l2_v1` (full_v2 packets, stop on violation);
- visible start at 10 m, 600-action timeout, 128 envs × quota 4;
- `navrl_band` / `bars_h3` placement at 70/115/160/205 bars;
- OCC trace and recovery events, with E-L0 attached for class labels;
- RTX 3070, never the 1650 Ti;
- the ISO/P03 selected-only reset isolation if any branching is used.

Re-run the Policy F baseline on the same fresh seeds; avoid every seed number already used (4101–4110, 4190, 4201–4207, 4301–4314, 4501–4503, 4601–4602, 4801–4803).

## 6. Inconsistencies between documents

1. **Class taxonomy.**
   - OCC-G0's "partial" scene (centre blocked, rim not all blocked) is E-L0's CENTRE_ONLY.
   - E-L0's PARTIAL (centre clear, pixels hidden) is never an OCC indicator.
   - OCC `out_of_view` merges FOV_EXIT and RANGE_EXIT.
   - SCALE_LOSS and NONE exist only in H-R3.
2. **"E2_obstacle_aware" names two lineages.** In TM3/TM7 it is the bounded, TF32-exposed backend; in KA/OCC/H-R3 it is `direct_l2_v1`. OCC cell IDs keep the old name.
3. **Missing TF32 caveat.** The `docs/EVIDENCE.md` target-motion rows do not carry it, although H/E1/E2 cells, and therefore E0−H, are TF32-exposed. This is already logged as `motar-lp5.3` in `docs/PROJECT_STATE.md`.
4. **OCC-P2 independence statement.** "No overlap with the frozen OCC-P1 scene-hash set" is true. However, 3,584 OCC-P2 layouts coincide with KB-B0 (seeds 4301–4307). The seed reuse was disclosed; the scene-level overlap was not.
5. **H-R3 P02 class table.** FULL 44, CENTRE_ONLY 12, FOV 181 and VIS 9 sum to the 246 reference triggers, not to the 203 matched events.
6. **E-L0 REPORT.** Its class table omits the N = 128 static run, whose counts exist in `e_l0_analysis.json`.
7. **Timeout figures look contradictory but are not.** `docs/EVIDENCE.md` gives the E0 timeout as 7.11 % (TM3 legacy window), while TM7 gives 8.93 % on the quota estimator.
8. **Superseded text kept in place.** These are not errors, but readers must use the addenda:
   - H-R3 P01 body text ("B1 33/83", "0.39 m more") is superseded by its addendum.
   - The P01 plan of "8 seeds / 141 events" is superseded by P02's assumption grid.
   - The ladder doc body still says TM-E2 is "NOT TESTED".
