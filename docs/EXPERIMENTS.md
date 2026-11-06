# Experiments

Which experiments belong together, which file to read first for each, and which can be compared.

**Last verified:** 2026-10-31, on `codex/mt-p1` (base `c40b924`).

A **family** is one research question followed through its preregistrations, runs, reviews and
decisions. [`tests/test_experiment_index.py`](../tests/test_experiment_index.py) checks that every
record in its scope belongs to exactly one family. The scope is:
- each folder under `results/`, `docs/evidence/` and `docs/research/`, and each loose file there
  (except `results/MANIFEST.json`);
- each file under `docs/plans/`, `docs/specs/`, `docs/audits/`, `docs/results/` and
  `docs/external_baselines/`;
- each top-level `docs/*.md` whose name marks a preregistration, a result or a table.

Audits, protocols, freeze records and plans elsewhere in `docs/` are outside that scope. Examples
are `d8c_perception_path_audit_2026-09-24.md`, `renderer_track_v1_freeze_2026-09-14.md`,
`status/gt_browser_*_freeze_*.md` and `release_validation_2026-09-14/`. A family names them in
*Read first* or *Lineage* where they matter.

A record first committed on or after 2026-10-31 must be named literally in its family's *Members*;
a wildcard does not cover it. When a record joins a family, update that family's *Lineage* and *Status* as well.

This page is not a source of results. Its few result numbers are copied from
[`EVIDENCE.md`](EVIDENCE.md) and [`HISTORY.md`](HISTORY.md), which are tested against the records;
run conditions (seeds, densities, episode counts) are copied from the family's records. Milestones are in
`HISTORY.md` and current work in [`PROJECT_STATE.md`](PROJECT_STATE.md). A status below is copied
from the family's records. Where it and the newest record of the family disagree, that record is
right; report the conflict.

## How to use this page

- **To understand a family:** open its *Read first* file. The *Lineage* line gives the order of the
  other records, and *Policy* says which checkpoint they ran on.
- **Before comparing two numbers:**
  - Check that both come from families listed together in [Comparable groups](#comparable-groups),
    and read that group's conditions.
  - Check the checkpoint SHA-256 in both receipts. A number is comparable only within a lineage
    ([`HISTORY.md`](HISTORY.md), *Lineage breaks*).
- **When starting a new experiment:** add it to an existing family, or add a family, in the same
  commit as its preregistration ([`AGENTS.md`](../AGENTS.md), *Recording work*).

## Comparable groups

### Policy F on the corrected geometry

Families: `safety-filter-attribution` (its Policy F grids only), `perception-pipeline` (P10),
`target-motion`, `kroc2027`, `occlusion-occ`, `environment-qualification`,
`sensor-fusion-corrections`, `h-r3`, `boia`, `b1r-classical-baseline`.

- **Shared:** Policy F (ep25000 + riskcap, SHA-256 `f7022139…`) on the corrected non-overlapping bar
  geometry of 2026-08-27 (`corrected-geometry`).
- **Check before comparing:**
  - **Checkpoint.** Not every record in these families is Policy F; each family's *Policy* line
    says which records are.
    - In `safety-filter-attribution`, only the 2026-09-02 stopcap screen, the D1p, L1, L1b, R1 and
      R3 grids and the 205-bar arc attribution (seed 49) record `f7022139…`.
    - In `perception-pipeline`, only the source arms of P10 and of its seed replication are Policy F
      (the −4.57 pp cost). Their adapted arms are descendants of F retrained under the error, so
      readaptation is not a frozen-policy result ([`EVIDENCE.md`](EVIDENCE.md)).
  - **Earlier Policy F records.** `policy-f` (from 2026-08-05), `latency-and-pose` and most of
    `learned-detector` also record Policy F, but they predate the 2026-08-27 geometry correction and
    are historical only ([`HISTORY.md`](HISTORY.md), *Lineage breaks*).
  - **Unit of inference.** P10 pools two evaluation seeds, safety geometry has three, the
    target-motion replication and B1-R have seven ([`EVIDENCE.md`](EVIDENCE.md)).
  - **Stopping window.** The original target-motion campaign (n = 3) used the legacy pooled window.
    The n = 7 replication used a bias-corrected per-environment quota, and the two are not pooled.
  - **TF32.** Historical moving-target cells are TF32-exposed (`kroc2027`; `motar-lp5.3`).
  - **Task path.** B1-R runs on the OCC-P2 task path, and its F shadow reproduces OCC-P2 exactly on
    28/28 cells (WORKLOG 2026-10-24, Stage A).
  - **Training contract.** The frozen training contract differs from current defaults in 14
    settings. Reproduce Policy F only through its recorded launcher environment
    ([contract audit](audits/frozen_policy_training_contract_2026-09-17.md)).

### Policy R and its descendants are not Policy F

Families: `policy-r-ref5in`, `dynamic-mesh`, `task-diagnostics`. Also the Policy R records of three
other families:
- `safety-filter-attribution`;
- `perception-pipeline` (the S1 shadow and the depth-noise order);
- `learned-detector` (the distractor envelope).

- Among the main results in [`EVIDENCE.md`](EVIDENCE.md), Policy R (ref5in D1 ep1900, `197ea269…`)
  carries only the observation contract (D8b). The records listed here ran on it as diagnostics.
- Much of the A-series safety diagnosis ran on Policy R or its retrained descendants: A3–A8
  (except the 205-bar A7 cell, seed 49), the contact-geometry and arc-attribution seeds 491–509,
  the D1, D4 and R0 grids, and the R-C replication. So the C3 withdrawal is a Policy R result.
- D8b in `dynamic-mesh` uses a ref5in checkpoint, not ep25000 ([`HISTORY.md`](HISTORY.md)).

### Speed filter: riskcap → arc clearance → classical baselines

Families: `policy-f` → `safety-filter-attribution` → `b1r-classical-baseline`.

- riskcap became part of Policy F on 2026-08-05.
- The 2026-09-02 stopcap screen and the A-series compare other speed laws with riskcap. Much of the
  A-series ran on Policy R (see above).
- The headline safety-geometry result, arc clearance − riskcap, is a contrast on Policy F
  ([`EVIDENCE.md`](EVIDENCE.md)).
- B1-R's baselines share the riskcap governor contract with F ([`EVIDENCE.md`](EVIDENCE.md), B1-R
  row).
- Every `policy-f` run predates the 2026-08-27 geometry correction, so its riskcap numbers do not
  compare with the later families' ([`HISTORY.md`](HISTORY.md), *Lineage breaks*).

### Perception error: injected versus measured

Families: `learned-detector`, `perception-pipeline`, `eth-ds5`.

- P10 injects the measured detector error into Policy F.
- ETH ds5 E3-S measures an image-size range proxy on one real flight.
- The ETH range proxy is not the injected error ([`HISTORY.md`](HISTORY.md), 2026-09-26), so the
  two numbers do not chain into one estimate.
- `learned-detector` evaluates earlier detectors (offline gates v2 to v7), mostly before the
  2026-08-27 geometry correction.

### Occlusion, re-observation and reacquisition

Families, in date order: `occlusion-occ` → `h-r3` → `boia` → `q02-reacquisition`, then the
C-Planner P0 package ([Not yet in the repository](#not-yet-in-the-repository)).

- These are studies of the same occlusion problem.
- None of them changes Policy F; each later study adds an opt-in layer or an offline module.
- None of them has a qualified online system yet:
  - BOIA is on G5 HOLD;
  - H-R3 is PROVISIONAL;
  - Q02-B/Q03 is on HOLD;
  - OCC is an observation study.
- B1-R uses the OCC-P2 task path (see the first group).

### Sensor-fusion correction lineages

Family: `sensor-fusion-corrections` (S-L1A, S-L1B, S-L1C with C-v2, S-L1D).

- Each lineage is an opt-in switch, default OFF.
- Each lineage has its own preregistration, float64 oracle and GPU qualification.
  - S-L1B, S-L1C, S-L1D (C01 and C02) and C-v2 also have an independent review record.
  - S-L1A has none in the repository: it is `S-L1A_QUALIFIED` and was accepted by the user
    (`aed6aff`).
- Compare them through their own qualification records, not through one another's numbers.

### Physical target and the geometry correction

Families: `physical-target-routing`, `corrected-geometry`.

- The routing gates span the 2026-08-27 geometry correction.
- Pre-correction results are historical only ([`HISTORY.md`](HISTORY.md), *Lineage breaks*).

## Families

Newest last. *Lineage* lists the main records in order; *Members* lists every record of the
family that is in the test's scope.

### `policy-f` — Policy F: training, speed screens and limits

- **Period:** 2026-07 – 2026-08; contract audit 2026-09-17.
- **Question:** Can a learned policy reach a moving target in a dense bar field from vision and
  LiDAR? Which speed filter makes it safe enough to freeze?
- **Policy:**
  - ep24000 (`82f7978b…`), earlier checkpoints and two ep25000 TTC-branch checkpoints
    (`169ddcdd…`, `14e4c72a…`), then Policy F (`f7022139…`) from 2026-08-05.
  - The verification-5A smoke (2026-08-12) trains a fresh legacy-vehicle policy (`f53489aa…`). It
    is not part of the ref5in lineage (`WORKLOG.md` 2026-08-13; [`HISTORY.md`](HISTORY.md), 08-13).
- **Status:**
  - ep25000 + riskcap was frozen on 2026-08-05 as Policy F.
  - It has been the only learned policy since D-2026-10-14-0.
- **Read first:** [`results/navrl_v2_riskcap_postadapt/summary.md`](../results/navrl_v2_riskcap_postadapt/summary.md)
- **Lineage:**
  - `results/navrl_v2_ep24000_heldout/`
  - → `results/navrl_v2_ep24000_causal_1to3/`
  - → speed screens (`results/navrl_v2_ep24000_speed_governor_screen/`)
  - → `results/navrl_v2_riskcap_postadapt/`
  - → `docs/audits/frozen_policy_training_contract_2026-09-17.md` (reproduction audit)
- **Compare with:** `safety-filter-attribution`, `latency-and-pose`, `corrected-geometry`
- **Members:** `results/altitude_pi_speed_axis.csv`, `results/baseline_speed_axis_peak986.csv`, `results/corrected_chirality_density_curve.csv`, `results/density_speed_map_cluster_sector.csv`, `results/general_*.csv`, `results/vision_density_curve.csv`, `results/v2_ttc_ab_1650ti.csv`, `results/navrl_v2_geometry_*.csv`, `results/navrl_v2_ep24000_*`, `results/navrl_v2_ep19100_vs_ep24000_forgetting/`, `results/navrl_v2_riskcap_postadapt/`, `results/navrl_v2_density_curve_riskcap/`, `results/navrl_v2_density_speed_map/`, `results/navrl_v2_bar_ceiling*`, `results/navrl_v2_speed_*`, `results/navrl_v2_joint_speed_allocation_seed379/`, `results/void_navrl_v2_joint_speed_allocation_*`, `results/navrl_v2_governor_adaptation_abc_seed53_schema2/`, `results/navrl_v2_v5a_semantics_smoke_seed197/`, `docs/navrl_joint_speed_preregistration_2026-08-21.md`, `docs/audits/frozen_policy_training_contract_2026-09-17.md`, `docs/audits/current_runtime_contract_2026-09-17.json`, `docs/audits/training_contract_vs_current_runtime_2026-09-17.json`

### `latency-and-pose` — Latency, clock and pose premises

- **Period:** 2026-08-06 – 2026-08-22.
- **Question:** How much capture is lost to latency and to imperfect clock or pose, and what
  corrects it?
- **Policy:** Policy F.
- **Status:**
  - Most of the latency loss was uncompensated ego-motion, and ego-motion correction was adopted
    ([`HISTORY.md`](HISTORY.md), 08-06).
  - Latency compensation: GO in the registered contract; real-world latency is not solved
    ([`EVIDENCE.md`](EVIDENCE.md)).
- **Read first:** [`results/navrl_v2_latency_ego_motion/summary.md`](../results/navrl_v2_latency_ego_motion/summary.md)
- **Lineage:**
  - `results/navrl_v2_detector_robustness/` (R3 screen, in `learned-detector`)
  - → `results/navrl_v2_latency_compensate/` → `results/navrl_v2_latency_obstacle_fix/`
  - → `results/navrl_v2_latency_ego_motion/`
  - → `results/navrl_v2_latency_budget/` (measured with the ego-motion fix on)
  - → `results/navrl_v2_pose_premise_seed163/`
- **Compare with:** `policy-f`, `learned-detector`
- **Members:** `results/navrl_v2_latency_*`, `results/navrl_v2_pose_*`

### `learned-detector` — Learned target detector in the navigation loop

- **Period:** 2026-08-05 – 2026-09-02.
- **Question:**
  - Does a learned detector keep navigation performance?
  - Does detector robustness carry into system robustness?
  - Through which channel does detection dropout cost capture?
- **Policy:** Policy F, except the distractor envelope (Policy R).
- **Status:**
  - Navigation non-inferiority PASS, replicated on fresh seeds (08-11 → 12).
  - Detector robustness did not become system robustness: NEGATIVE.
  - Colour shortcut with distractors: COLOR_SHORTCUT_CONFIRMED (09-02).
- **Read first:** [`results/navrl_v2_detector_navigation_ab_replication_seed97_101_schema2/summary.md`](../results/navrl_v2_detector_navigation_ab_replication_seed97_101_schema2/summary.md)
- **Lineage:**
  - `results/navrl_v2_detector_robustness/`
  - → `results/navrl_detector_offline_gate_v2/`
  - → `results/navrl_v2_detector_navigation_ab_seed83_89_schema2/`
  - → `results/navrl_v2_detector_navigation_ab_replication_seed97_101_schema2/`
  - → `results/navrl_detector_offline_gate_v7_confirmatory/`
  - → `results/navrl_v2_appearance_navigation_ab_seed151_157/`
  - → `results/navrl_detector_distractor_envelope_seed479/`
- **Compare with:** `perception-pipeline`, `latency-and-pose`, `renderer`
- **Members:** `results/navrl_v2_detector_*`, `results/navrl_detector_*`, `results/navrl_v2_lidar_*`, `results/navrl_v2_target_mask_backfill*`, `results/navrl_v2_appearance_navigation_ab_seed151_157/`, `docs/prereg_2026-08-13_detector_coupling.md`, `docs/prereg_2026-09-01_distractor_envelope.md`

### `policy-r-ref5in` — ref5in reference platform and Policy R

- **Period:** 2026-08-13 – 2026-09-03.
- **Question:** Does the policy adapt to the 1.2 kg ref5in platform? What limits first acquisition
  and sensing on it?
- **Policy:**
  - Policy R (ref5in D1 ep1900, `197ea269…`).
  - The P2 and outcome-diagnostic runs, detection range stage 1, the geofence and the ref5in smoke
    use other ref5in training checkpoints.
- **Status:**
  - ref5in P2 held-out and D1 adaptation gates FAIL; P3 full budget BLOCKED.
  - First acquisition was identified as a failure mode (08-14 → 20).
  - Detection-range stage 1: RANGE_INCONCLUSIVE_AT_THIS_BUDGET.
- **Read first:** [`results/navrl_ref5in_d1_eval_seed331/summary.md`](../results/navrl_ref5in_d1_eval_seed331/summary.md)
- **Lineage:**
  - `results/navrl_ref_platform_verification/`
  - → `results/navrl_ref5in_p2_seed313/`
  - → `results/navrl_ref5in_d1_eval_seed331/`
  - → `results/navrl_ref5in_camera_range_control_seed367/`
  - → `results/navrl_ref5in_detection_range_stage1_s457/`
  - The S1 blind-search preregistration (`docs/preregistration_s1_blind_search_state_2026-09-03.md`)
    takes its brief from `docs/plans/target_search_and_adversarial_evader.md` in `target-motion`.
- **Compare with:** `dynamic-mesh`, `task-diagnostics`, `target-motion`
- **Members:** `results/navrl_ref5in_*`, `results/navrl_ref_platform_verification*`, `results/navrl_sim2real_software_preflight_2026-08-24/`, `results/VOID_20260823_*`, `docs/prereg_2026-08-21_n1_real_frame_reflection_audit.md`, `docs/prereg_2026-08-22_*`, `docs/prereg_2026-08-24_distance_fidelity.md`, `docs/preregistration_active_search_geofence_2026-08-21.md`, `docs/preregistration_s1_blind_search_state_2026-09-03.md`

### `physical-target-routing` — Physical target, routing and braking

- **Period:** 2026-08-22 – 2026-09-02.
- **Question:** Can a physically simulated target follow a route through the bar field without
  invalid motion?
- **Policy:**
  - No Policy F or R checkpoint is recorded.
  - The physical-off runs use their own trained checkpoints (`541b36bd…`, `b2b8ece3…`).
- **Status:**
  - The routed physical-target gates FAIL over four rounds: FAIL_ROUTE_MECHANISM.
  - Braking-v3: FAIL_BLOCKS_CONFIRMATORY. It closes the physical-target GPU authority
    (09-01 → 02).
- **Read first:** [`docs/braking_aware_route_v3_lower1p25_matched_spawn_result_2026-09-01.md`](braking_aware_route_v3_lower1p25_matched_spawn_result_2026-09-01.md)
- **Lineage:**
  - `docs/preregistration_navrl_physical_target_braking_2026-08-25.md`
  - → `results/navrl_physical_target_routed_gate_seed827/`
  - → `docs/physical_target_recovery_v2_lower1p25_result_2026-08-26.md`
  - → `docs/research_authority_2026-08-26.json` (the research authority freeze)
  - → `docs/corrected_nonoverlap_route_gate_r2_result_2026-08-31.md`
  - → `docs/braking_aware_route_v3_lower1p25_matched_spawn_result_2026-09-01.md`
  - → `results/navrl_corrected_nonoverlap_physical_off_heldout_seed313/` (route off)
- **Compare with:** `corrected-geometry`
- **Members:** `results/navrl_physical_target_*`, `results/navrl_braking_route_v3_*`, `results/navrl_corrected_nonoverlap_*`, `results/navrl_target_route_cpu_benchmark_seed825/`, `docs/preregistration_physical_target_*`, `docs/preregistration_navrl_physical_target_*`, `docs/preregistration_braking_aware_*`, `docs/preregistration_corrected_nonoverlap_*`, `docs/prereg_2026-08-24_physical_target_speed_envelope.md`, `docs/physical_target_*_result_*.md`, `docs/braking_aware_*_result_*.md`, `docs/corrected_nonoverlap_*_result_*.md`

### `corrected-geometry` — Corrected bar geometry (lineage break)

- **Period:** 2026-08-27.
- **Question:** Do the bars overlap, and which densities does a non-overlapping layout support?
- **Policy:** none (a geometry audit).
- **Status:**
  - Lineage break: non-overlapping bars, densities 70–205.
  - Pre-correction results are historical only.
- **Read first:** [`results/navrl_v2_density_geometry_audit_2026-08-27/summary.md`](../results/navrl_v2_density_geometry_audit_2026-08-27/summary.md)
- **Lineage:**
  - `docs/preregistration_navrl_v2_corrected_density_geometry_2026-08-27.md`
  - → `results/navrl_v2_density_geometry_audit_2026-08-27/`
- **Compare with:** `policy-f`, `physical-target-routing`
- **Members:** `results/navrl_v2_density_geometry_audit_2026-08-27/`, `docs/preregistration_navrl_v2_corrected_density_geometry_2026-08-27.md`

### `safety-filter-attribution` — Safety-filter attribution (stopcap, A1–A8, grid, R-C)

- **Period:** 2026-09-02 – 2026-09-09.
- **Question:** Where do the remaining crashes come from? Does measuring clearance along the
  turning arc lower them relative to riskcap?
- **Policy:**
  - Mixed. The D1p, L1, L1b, R1 and R3 grids, the 205-bar arc attribution (seed 49) and the
    stopcap screen are Policy F.
  - A3–A8 (except the 205-bar A7 cell, seed 49), the contact geometry, arc-attribution seeds 491
    and 509, the D1, D4 and R0 grids and R-C are Policy R or its retrained descendants.
- **Status:**
  - Arc clearance − riskcap crash −1.4903 pp, independently re-computed (09-07).
  - The C3 "co-adaptation decides the law" explanation was WITHDRAWN (R-C: 0/2 training seeds of
    Policy R descendants, 09-09).
  - Five runs are VOID: four for source, source-bundle or HEAD drift, and one for a search-state
    restore defect.
- **Read first:** [`results/independent_verification_2026-09-07/README.md`](../results/independent_verification_2026-09-07/README.md)
- **Lineage:**
  - `docs/prereg_2026-09-02_speed_governor_stopcap_screen.md`
  - → `docs/plans/diagnostic_paper_plan_2026-09-05.md`
  - → `docs/prereg_2026-09-05_a7_arc_attribution.md`
  - → `docs/a7_arc_attribution_table.md`
  - → `docs/plans/confirmation_phase_plan_2026-09-06.md`
  - → `results/independent_verification_2026-09-07/`
  - → `results/navrl_grid_r2_d4_trainseed_rep/` (R-C, planned in
    `docs/plans/perception_s4_rc_execution_2026-09-09.md`)
- **Compare with:** `policy-f`, `policy-r-ref5in`, `b1r-classical-baseline`
- **Members:** `results/navrl_v2_ep25000_stopcap_seed49_screen/`, `results/navrl_contact_geometry_*`, `results/navrl_arc_attribution_*`, `results/navrl_a8_readaptation_*`, `results/VOID_20260905_*`, `results/VOID_20260906_*`, `results/navrl_grid_d*`, `results/navrl_grid_l1*`, `results/navrl_grid_r*`, `results/rc_replication_2026-09-09/`, `results/independent_verification_*`, `results/verification_export_*`, `docs/prereg_2026-09-02_speed_governor_stopcap_screen.md`, `docs/prereg_2026-09-04_contact_corridor_forensics.md`, `docs/prereg_2026-09-05_*`, `docs/a*_table.md`, `docs/plans/diagnostic_paper_plan_2026-09-05.md`, `docs/plans/lateral_contact_density_plan_2026-09-05.md`, `docs/plans/confirmation_phase_plan_2026-09-06.md`, `docs/specs/grid_d*`, `docs/specs/grid_l1*`, `docs/specs/grid_r*`

### `perception-pipeline` — Perception pipeline and its cost to Policy F (S0–S4, P3–P10)

- **Period:** 2026-09-03 – 2026-09-11.
- **Question:** What does the measured perception error cost Policy F? Does retraining under that
  error recover it?
- **Policy:**
  - P10 and its empirical grids evaluate Policy F (source arms) and one descendant of F retrained
    for 1,000 epochs under the P9 error (`db6d9f05…`, adapted arms).
  - The P10 seed replication trains three more descendants of F.
  - The S1 shadow and the depth-noise order runs are Policy R.
  - The other runs record detector checkpoints only.
- **Status:**
  - P10: −4.57 pp capture.
  - P10 seed replication (readaptation): +0.73 pp, INCONCLUSIVE ([`EVIDENCE.md`](EVIDENCE.md)).
  - S1 shadow: RECOGNITION_DOMINANT. Depth-noise order: VARIANCE_INSENSITIVE.
- **Read first:** [`results/perception_p10_seed_replication_2026-09-10/README.md`](../results/perception_p10_seed_replication_2026-09-10/README.md)
- **Lineage:**
  - `docs/plans/perception_shape_temporal_redesign_2026-09-03.md`
  - → `results/navrl_camera_cost/` (S0)
  - → `results/navrl_s1_shadow_seed481/` (S1)
  - → `results/navrl_depth_noise_order_seed487/`
  - → `results/air2air_zeroshot/`
  - → `docs/plans/perception_final_implementation_plan_2026-09-07.md`
  - → `results/perception_joint_detector_p4_p5_2026-09-08/`
  - → `results/perception_temporal_p6_p7_2026-09-08/`
  - → `results/perception_p10_2026-09-09/`
  - → `results/perception_p10_seed_replication_2026-09-10/`
- **Compare with:** `learned-detector`, `eth-ds5`, `policy-r-ref5in`
- **Members:** `results/perception_*`, `results/navrl_camera_cost/`, `results/navrl_s1_shadow_seed481/`, `results/navrl_depth_noise_order_seed487/`, `results/air2air_zeroshot/`, `results/navrl_grid_p10_empirical_*`, `docs/specs/grid_p10_empirical_*`, `docs/specs/perception_*`, `docs/specs/motar_perception_candidates_v1.schema.json`, `docs/plans/perception_*`, `docs/plans/master_plan_2026-09-07.md` (programme plan across tracks), `docs/plans/prereg_2026-09-09_range_error_proxy.md`, `docs/prereg_2026-09-03_s1_structure_fix_shadow.md`, `docs/prereg_2026-09-04_depth_noise_model_order.md`, `docs/preregistration_sam_instance_adapter_offline_2026-09-03.md`, `docs/preregistration_p10_seed_replication_2026-09-10.md`

### `eth-ds5` — Range proxy on real footage (ETH ds5)

- **Period:** 2026-09-10.
- **Question:** Is an image-size range proxy usable on real UAV footage?
- **Policy:** none (offline analysis of recorded footage).
- **Status:**
  - E3-S: 6.2 % median range-proxy error, SIZE_RANGE_USABLE.
  - E3-P attitude decomposition: ATTITUDE_NOT_RELIABLE → BLOCKED.
- **Read first:** [`results/eth_ds5_e3s_2026-09-10/README.md`](../results/eth_ds5_e3s_2026-09-10/README.md)
- **Lineage:**
  - `docs/plans/eth_ds5_e3_2026-09-10.md`
  - → `results/eth_ds5_intake_2026-09-10/`
  - → `results/eth_ds5_e3s_2026-09-10/`
  - → `results/eth_ds5_e3p_gate_2026-09-10/`
- **Compare with:** `perception-pipeline`
- **Members:** `results/eth_ds5_*`, `docs/plans/eth_ds5_*`

### `renderer` — Independent renderer track (R1–R5, follow-up)

- **Period:** 2026-09-11 – 2026-09-14.
- **Question:** Can an independent renderer reproduce the simulator's camera images within stated
  tolerances?
- **Policy:** none recorded (renderer measurements).
- **Status:**
  - `RENDERER_CONTRACT_V1_COMPLETE`: the preregistered v1 questions were answered. It is not a
    blanket PASS.
  - Area control NOT ADOPTED.
  - Normal representation N0 ONLY (the N1 and N2 arms are UNSUPPORTED by the asset).
  - R4 and R4b criterion C FAIL.
- **Read first:** [`docs/renderer_track_v1_freeze_2026-09-14.md`](renderer_track_v1_freeze_2026-09-14.md)
- **Lineage:**
  - `docs/preregistration_renderer_r3_2026-09-11.md`
  - → `docs/renderer_r3_r5_results_2026-09-11.md`
  - → `results/renderer_characterization_2026-09-13/`
  - → `docs/preregistration_renderer_followup_2026-09-14.md`
  - → `docs/renderer_track_v1_freeze_2026-09-14.md`
- **Compare with:** `dynamic-mesh`, `learned-detector`
- **Members:** `results/renderer_*`, `results/v1_shared_airframe_2026-09-11/`, `results/target_appearance_in_sim_2026-09-12/`, `docs/preregistration_renderer_*`, `docs/renderer_r3_r5_results_2026-09-11.md`

### `dynamic-mesh` — Dynamic target mesh and policy sensitivity (D6–D8)

- **Period:** 2026-09-12 – 2026-09-24.
- **Question:** Does a moving target mesh change the detector, and does that cost Policy R, the
  ref5in D1 ep1900 checkpoint that D8b measured?
- **Policy:** D8b is Policy R.
- **Status:**
  - D6 standalone mesh cost: INCONCLUSIVE.
  - D7 integrated cost: GO.
  - D8b: MATERIAL_LOSS −48.967 pp, causality NOT_TESTED.
  - D8c audit (2026-09-24) records the user's decision `NO_RETRAINING_REQUIRED_FOR_CURRENT_CLAIM`.
- **Read first:** [`results/dynamic_mesh_policy_sensitivity_d8b_2026-09-13/README.md`](../results/dynamic_mesh_policy_sensitivity_d8b_2026-09-13/README.md)
- **Lineage:**
  - `results/dynamic_mesh_raycast_feasibility_2026-09-12/`
  - → `results/dynamic_mesh_integrated_cost_2026-09-12/`
  - → `docs/preregistration_dynamic_mesh_detector_d8_2026-09-13.md`
  - → `results/dynamic_mesh_policy_sensitivity_d8b_2026-09-13/`
  - → `docs/d8c_perception_path_audit_2026-09-24.md`
- **Compare with:** `renderer`, `policy-r-ref5in`
- **Members:** `results/dynamic_mesh_*`, `docs/preregistration_dynamic_mesh_*`

### `public-release-records` — Public release and record envelope

- **Period:** 2026-09-12 – 2026-09-14.
- **Question:** Can the public snapshot and its records be checked by someone outside the project?
- **Policy:** none.
- **Status:**
  - Release layer in place: licences, citation, CPU quick start, public snapshot (09-12 → 15).
  - Dataset reachability in Git history: OPEN. Several provenance items are NEEDS_CONFIRMATION.
  - Record envelope v2: `LIVE_GENERIC_VALIDATION_NOT_APPLICABLE`. The historical artifact keeps
    `MATCH` (bytes) and `INVALID_FOR_DECLARED_CONTRACT` (metadata).
- **Read first:** [`docs/public_release_status_2026-09-14.md`](public_release_status_2026-09-14.md)
- **Lineage:**
  - `results/public_release_2026-09-12/`
  - → `results/record_envelope_v2_live_2026-09-14/`
  - → `results/record_integrity_review_2026-09-14/`
- **Compare with:** none
- **Members:** `results/public_release_2026-09-12/`, `results/public_site_*`, `results/record_envelope_v2_live_2026-09-14/`, `results/record_integrity_review_2026-09-14/`

### `task-diagnostics` — Task diagnostics instrumentation (T1, T2)

- **Period:** 2026-09-14.
- **Question:** Does the task-diagnostics instrumentation leave episode behaviour unchanged?
- **Policy:** Policy R (ep1900).
- **Status:**
  - Instrumentation invariance: `TRAJECTORY_INVARIANT` (70 bars, seed 593, 128 episodes per arm,
    Policy R ep1900).
  - The preregistered TD-T1 and TD-T2 results have not been run.
- **Read first:** [`results/task_diagnostics_invariance_2026-09-14/README.md`](../results/task_diagnostics_invariance_2026-09-14/README.md)
- **Lineage:**
  - `docs/preregistration_task_diagnostics_t1_2026-09-14.md`
  - → `docs/preregistration_task_diagnostics_t2_2026-09-14.md`
  - → `results/task_diagnostics_invariance_2026-09-14/`
- **Compare with:** `policy-r-ref5in`
- **Members:** `results/task_diagnostics_invariance_2026-09-14/`, `docs/preregistration_task_diagnostics_*`

### `browser-preview` — Browser ground-truth preview

- **Period:** 2026-09-17 – 2026-09-18.
- **Question:** Does the browser preview reproduce the ground-truth tracking and interception
  episodes?
- **Policy:** none recorded.
- **Status:** Frozen as GT_BROWSER_V1, then GT_BROWSER_EPISODE_V1.
- **Read first:** [`docs/status/gt_browser_episode_v1_freeze_2026-09-18.md`](status/gt_browser_episode_v1_freeze_2026-09-18.md)
- **Lineage:**
  - `docs/status/gt_browser_v1_freeze_2026-09-17.md`
  - → `results/browser_gt_tracking_validation_2026-09-17/`
  - → `results/browser_interception_v1_2026-09-18/`
  - → `docs/status/gt_browser_episode_v1_freeze_2026-09-18.md`
- **Compare with:** none
- **Members:** `results/browser_*`

### `target-motion` — Target-motion generalisation and its replication

- **Period:** 2026-09-03 (plans) – 2026-09-26.
- **Question:** How does Policy F's capture change when the target moves differently from training
  (E0 static, E1, E2 obstacle-aware)?
- **Policy:** Policy F.
- **Status:**
  - The n = 7 replication is current: E0 −5.37, E1 −3.45, E2 +1.35 pp versus H, all three
    replicated.
  - The verdict `NO_RETRAIN_NEEDED` was WITHDRAWN.
  - The TF32 caveat is not yet in the public claims (`motar-lp5.3`).
- **Read first:** [`results/target_motion_replication_7seed/README.md`](../results/target_motion_replication_7seed/README.md)
- **Lineage:**
  - `docs/plans/target_search_and_adversarial_evader.md` (source brief, also of the S1 blind-search
    preregistration in `policy-r-ref5in`)
  - → `docs/audits/retraining_readiness_audit_2026-09-17.md`
  - → `docs/prereg_2026-09-17_target_motion_complexity_e0_e1_e2.md`
  - → `results/target_motion_e0_e2_evaluation_2026-09-18/`
  - → `docs/results/target_motion_generalization_2026-09-19.md`
  - → `docs/audits/target_motion_protocol_deviations_2026-09-19.md`
  - → `docs/prereg_2026-09-24_target_motion_replication_7seed.md`
  - → `results/target_motion_replication_7seed/`
- **Compare with:** `kroc2027`, `policy-f`, `policy-r-ref5in`
- **Members:** `results/target_motion_*`, `docs/results/target_motion_*`, `docs/prereg_2026-09-17_target_motion_*`, `docs/prereg_2026-09-24_target_motion_*`, `docs/audits/target_motion_*`, `docs/audits/retraining_readiness_*`, `docs/plans/target_search_and_adversarial_evader.md`, `docs/plans/moving_target_rendezvous_master_plan_2026-09-12.md` (programme roadmap)

### `external-baseline` — External matched baseline (Elastic Tracker)

- **Period:** 2026-09-18 – 2026-09-21.
- **Question:** Can a published tracker be reproduced and run on the MOTAR task for a matched
  comparison?
- **Policy:** none (an external tracker).
- **Status:**
  - Elastic Tracker B0: UPSTREAM_REPRODUCTION_FAIL.
  - Class A matched comparisons = 0.
- **Read first:** [`docs/external_baselines/elastic_tracker_b0_upstream_reproduction_2026-09-20.md`](external_baselines/elastic_tracker_b0_upstream_reproduction_2026-09-20.md)
- **Lineage:**
  - `docs/external_baseline_selection_2026-09-19.md`
  - → `docs/preregistration_external_matched_baseline_draft_2026-09-19.md`
  - → `docs/external_baselines/elastic_tracker_b0_upstream_reproduction_2026-09-20.md`
- **Compare with:** `b1r-classical-baseline`
- **Members:** `docs/external_baselines/*`, `docs/preregistration_external_matched_baseline_draft_2026-09-19.md`

### `kroc2027` — KROC2027 initial-view study and MT precision

- **Period:** 2026-10-14 – 2026-10-16.
- **Question:**
  - Does the pursuer's initial view of the target change Policy F's capture, and does that effect
    differ between a static and a moving target (Experiment A)?
  - Does a conditional search supervisor around the unchanged Policy F help on out-of-view starts
    (Experiment B)?
  - Are the historical moving-target counterexamples real?
- **Policy:** Policy F.
- **Status:**
  - H-A (Delta_A) SUPPORTED; H-A2 (I_A) INCONCLUSIVE. Neither contrast clears the preregistered
    3 pp practical threshold.
  - H-B INCONCLUSIVE, so B1 was not adopted.
  - The E1/E2 160-bar collision counterexamples came from a TF32 `torch.cdist` (H-MT1 rejected).
- **Read first:** [`docs/evidence/mt_precision_review_2026-10-16/README.md`](evidence/mt_precision_review_2026-10-16/README.md)
- **Lineage:**
  - `docs/plans/kroc2027_protocol_v1.json`
  - → `docs/plans/prereg_kroc2027_A_v1.md`
  - → `results/kroc2027_A_v1/`
  - → `docs/plans/prereg_kroc2027_B_v1.md`
  - → `results/kroc2027_B_v1/`
  - → `docs/audits/moving_target_tf32_readonly_2026-10-16.md`
  - → `docs/plans/mt_p1_precision_v1.md`
  - → `docs/evidence/mt_precision_review_2026-10-16/`
- **Compare with:** `target-motion`, `occlusion-occ`
- **Members:** `results/kroc2027_*`, `docs/plans/kroc2027_*`, `docs/plans/prereg_kroc2027_*`, `docs/plans/mt_p1_*`, `docs/evidence/mt_precision_review_2026-10-16/`, `docs/audits/moving_target_tf32_readonly_2026-10-16.*`, `docs/audits/mt_precision_governance_addendum_2026-10-16.md`

### `occlusion-occ` — Natural occlusion and recovery observation (R-P1a, OCC)

- **Period:** 2026-10-16 – 2026-10-17.
- **Question:** How often, and for how long, is the target occluded? How does that change with
  density and with target motion?
- **Policy:** Policy F.
- **Status:**
  - OCC-P2 independent confirmation complete: RQ1 and RQ3 SUPPORTED.
  - RQ2: INCONCLUSIVE_EQUIVALENCE_NOT_TESTED.
  - These are closed-loop whole-system effects, not isolated causal effects of target speed.
- **Read first:** [`docs/evidence/occ_p2_confirmatory_2026-10-17/REPORT.md`](evidence/occ_p2_confirmatory_2026-10-17/REPORT.md)
- **Lineage:**
  - `docs/plans/r_p1a_observability_v1.md`
  - → `docs/evidence/r_p1a_v2_cpu_2026-10-16/`
  - → `docs/plans/occlusion_pilot_v2.md`
  - → `docs/plans/prereg_occlusion_confirmatory_v1.md`
  - → `docs/evidence/occ_p2_confirmatory_2026-10-17/`
- **Compare with:** `h-r3`, `boia`, `b1r-classical-baseline`
- **Members:** `docs/evidence/r_p1a_*`, `docs/evidence/occ_p2_*`, `docs/plans/occlusion_pilot_*`, `docs/plans/prereg_occlusion_confirmatory_v1.*`, `docs/plans/r_p1a_observability_v1*`, `docs/audits/r_p1a_implementation_2026-10-16.md`

### `environment-qualification` — Environment qualification (EQ-0, EQ-1, EQ-1D, E-L0)

- **Period:** 2026-10-17 – 2026-10-18.
- **Question:** Is the simulation environment fit for the claims made on it?
- **Policy:** Policy F.
- **Status:**
  - EQ-0 → EQ-1D → E-L0 complete.
  - The environment verdict is LIMITED_VALIDITY.
  - E-L0: all 14 gates PASS, environment unchanged.
- **Read first:** [`docs/evidence/eq1d_qualification_2026-10-18/REPORT.md`](evidence/eq1d_qualification_2026-10-18/REPORT.md)
- **Lineage:**
  - `docs/evidence/eq0_environment_audit_2026-10-17/`
  - → `docs/evidence/eq1_validation_2026-10-18/`
  - → `docs/plans/eq1d_qualification_v1.md`
  - → `docs/evidence/eq1d_qualification_2026-10-18/`
  - → `docs/plans/e_l0_evaluation_lineage_v1.md`
  - → `docs/evidence/e_l0_evaluation_2026-10-18/`
- **Compare with:** `sensor-fusion-corrections`
- **Members:** `docs/evidence/eq0_*`, `docs/evidence/eq1_*`, `docs/evidence/eq1d_*`, `docs/evidence/e_l0_*`, `docs/plans/eq1d_*`, `docs/plans/e_l0_*`

### `sensor-fusion-corrections` — Sensor-fusion correction lineages (S-L1A–D)

- **Period:** 2026-10-18.
- **Question:** Which camera–LiDAR fusion defects can be corrected behind opt-in switches without
  changing Policy F?
- **Policy:** Policy F, with each correction OFF by default.
- **Status:**
  - S-L1A QUALIFIED and accepted; S-L1B, S-L1C and S-L1D v1 GO (S-L1D via SL1D-C02).
  - C-v2 GO in a limited scope.
- **Read first:** [`docs/SENSOR_CONTRACT.md`](SENSOR_CONTRACT.md)
- **Lineage:**
  - `docs/plans/sensor_contract_amendment_2026-10-18.md`
  - → `docs/plans/s_l1a_v1.md` → `docs/evidence/s_l1a_2026-10-18/`
  - → `docs/plans/s_l1b_v1.md` → `docs/evidence/s_l1b_2026-10-18/`
  - → `docs/plans/s_l1c_v1.md` → `docs/evidence/s_l1c_2026-10-18/`
  - → `docs/plans/s_l1d_v1.md` → `docs/evidence/s_l1d_2026-10-18/`
  - → `docs/plans/s_l1c_v2.md` → `docs/evidence/s_l1c_v2_2026-10-18/`
  - The original approval boundary is `docs/plans/correction_lineages_plan_v2.md`.
- **Compare with:** `environment-qualification`
- **Members:** `docs/evidence/s_l1*`, `docs/plans/s_l1*`, `docs/plans/correction_lineages_plan_*`, `docs/plans/post_s_l1c_v2_validation_design.md`, `docs/plans/sensor_contract_amendment_2026-10-18.*`

### `h-r3` — H-R3 re-observation by translation

- **Period:** 2026-10-18 – 2026-10-19.
- **Question:** Does translating, rather than only yawing, help the pursuer re-observe a target it
  has lost?
- **Policy:** Policy F.
- **Status:**
  - H-R3 remains PROVISIONAL / CONFIRMATION HOLD until the estimand is chosen (`motar-6nn.1`).
  - The P01 pilot is exploratory. P02: INTEGRITY FAIL, confirmation NO-GO.
  - Environment isolation ISO-Q02: mechanics GO.
  - P03-Q01: GO for qualification only, a one-seed engineering qualification.
  - Viewpoint attribution INCONCLUSIVE.
- **Read first:** [`docs/evidence/h_r3_p03_preparation_2026-10-19/REPORT.md`](evidence/h_r3_p03_preparation_2026-10-19/REPORT.md)
- **Lineage:**
  - `docs/plans/h_r3_pilot_v1.md` → `docs/evidence/h_r3_pilot_2026-10-18/`
  - → `docs/plans/h_r3_p02.md` → `docs/evidence/h_r3_p02_2026-10-18/`
  - → `docs/plans/h_r3_env_isolation_v1.md` → `docs/evidence/h_r3_env_isolation_2026-10-18/`
  - → `docs/plans/h_r3_p03_qualification_v1.md` → `docs/evidence/h_r3_p03_preparation_2026-10-19/`
- **Compare with:** `occlusion-occ`, `boia`
- **Members:** `docs/evidence/h_r3_*`, `docs/plans/h_r3_*`

### `boia` — Occlusion-handling algorithm (BOIA)

- **Period:** 2026-10-19 – 2026-10-21.
- **Question:** Can an occlusion-aware layer around Policy F be designed and qualified, starting
  from a passive observer?
- **Policy:** Policy F.
- **Status:**
  - The BOIA-SHADOW-Q01-V2 GPU run of 2026-10-21 gave
    `PASS_EXACT_NATIVE_OFF_ON_PARITY_WITH_REPLAY_CONTROL`. It is a passive observer only.
  - No online integration; G5 HOLD.
  - The Master Plan and `PROJECT_STATE.md` still say GPU NOT_RUN; the run report is newer
    (`motar-lp5.4`).
- **Read first:** [`docs/research/occlusion_algorithm_v1/BOIA_RESEARCH_MASTER_PLAN.md`](research/occlusion_algorithm_v1/BOIA_RESEARCH_MASTER_PLAN.md),
  then the run report [`docs/evidence/boia_shadow_q01_v2_run_2026-10-21/REPORT.md`](evidence/boia_shadow_q01_v2_run_2026-10-21/REPORT.md)
- **Lineage:**
  - `docs/research/occlusion_algorithm_v1/`
  - → `docs/evidence/boia_g4_p0_2026-10-20/`
  - → `docs/evidence/boia_g4r_2026-10-20/`
  - → `docs/evidence/boia_g4r5_2026-10-21/`
  - → `docs/evidence/boia_g4r6_shadow_preflight_2026-10-21/`
  - → `docs/evidence/boia_g4r7_shadow_v2_preflight_2026-10-21/`
  - → `docs/plans/boia_shadow_q01_v2.md`
  - → `docs/evidence/boia_shadow_q01_v2_run_2026-10-21/`
- **Compare with:** `occlusion-occ`, `h-r3`, `q02-reacquisition`
- **Members:** `docs/research/occlusion_algorithm_v1/`, `docs/evidence/boia_*`, `docs/plans/boia_shadow_*`

### `q02-reacquisition` — Evidence-aware reacquisition (Q02-A, Q02-B, Q03)

- **Period:** 2026-10-21 – 2026-10-22.
- **Question:** Can the target be re-identified from sensor evidence after occlusion? Can a local
  planner choose where to look?
- **Policy:** none run (CPU modules).
- **Status:**
  - Q02-A v2.6: PASS_WITH_EXPLICIT_IDENTITY_LIMITATIONS, after seven repair and review rounds.
  - The Q02-B/Q03 package review: FAIL (two P1), so the package is on HOLD.
- **Read first:** [`docs/evidence/q02b_q03_opus_review_2026-10-22/REVIEW.md`](evidence/q02b_q03_opus_review_2026-10-22/REVIEW.md)
- **Lineage:**
  - `docs/evidence/q02a_cpu_2026-10-21/`
  - → `docs/evidence/q02a_fable_review_2026-10-21/`
  - → repair and review rounds v2 to v2.6 (`docs/evidence/q02a_v2_repair_2026-10-21/` …
    `docs/evidence/q02a_v26_opus_review_2026-10-22/`)
  - → `docs/evidence/q02b_q03_cpu_package_2026-10-22/`
  - → `docs/evidence/q02b_q03_opus_review_2026-10-22/`
- **Compare with:** `boia`
- **Members:** `docs/evidence/q02*`, `docs/research/q02_evidence_aware_reacquisition/`

### `b1r-classical-baseline` — Classical sensor-based baselines (B1-R)

- **Period:** 2026-10-23 – 2026-10-24.
- **Question:** On the same task path, does Policy F beat classical controllers that steer on the
  same tracker belief?
- **Policy:** Policy F against two classical controllers.
- **Status:**
  - COMPLETE: C1 and C2 SUPERIOR.
  - F is +8.87 pp over pure pursuit and +30.38 pp over proportional navigation
    ([`EVIDENCE.md`](EVIDENCE.md)).
- **Read first:** [`docs/evidence/b1r_v1_audit/AUDIT_REPORT.md`](evidence/b1r_v1_audit/AUDIT_REPORT.md)
- **Lineage:**
  - `docs/plans/b1r_classical_baseline_prereg_DRAFT_2026-10-23.md`
  - → `docs/plans/prereg_b1r_classical_baseline_v1.md`
  - → `docs/evidence/b1r_v1/`
  - → `docs/evidence/b1r_v1_audit/`
- **Compare with:** `occlusion-occ`, `safety-filter-attribution`, `external-baseline`
- **Members:** `docs/evidence/b1r_*`, `docs/plans/*b1r*`, `docs/plans/b1r_classical_baseline_prereg_DRAFT_2026-10-23.md`

### `gain-randomization` — Policy F under randomized controller gains (GR)

- **Period:** 2026-11-04 –.
- **Question:** Policy F was trained and evaluated on one airframe with fixed Lee-controller gains. How much do its
  capture and crash rates change when the gains vary per episode? Does that motivate training with gain
  randomization?
- **Policy:** Frozen Policy F, unchanged; gain randomization is an opt-in switch with default OFF. Any newly trained
  checkpoint (GR-1, not yet preregistered) would be a new policy compared with F (D-2026-11-04-1).
- **Status:** identity_01 IDENTITY_FAIL (runner placement, no comparison made); amendment 1 (A3 placement);
  identity_02 IDENTITY_PASS. GR-0 COMPLETE and independently reviewed (PASS): S20 GAIN_ROBUST, S40
  GAIN_SENSITIVE (crash +3.69 pp). GR-1: both arms TRAINED on `8f64631`; amendment 1 (bindings, claim, controls)
  after the design review AMEND_FIRST; evaluation not run. Compare with `policy-r-ref5in`
  (the failed 2026-08 airframe adaptation) and `b1r-classical-baseline` (the F condition GR-0's S00 mirrors).
- **Read first:** [`docs/evidence/gain_randomization_2026-11-04/README.md`](evidence/gain_randomization_2026-11-04/README.md)
- **Lineage:**
  - D-2026-11-04-1 (new PPO allowed again)
  - → `docs/plans/prereg_gain_randomization_gr0_v1.md`
  - → `docs/plans/prereg_gain_randomization_gr0_v1_amendment1_2026-11-04.md` (A3 placement after identity_01)
  - → `docs/evidence/gain_randomization_2026-11-04/`
  - → `docs/plans/prereg_gain_randomization_gr1_v1.md` (GR-1 training, after GR-0 GAIN_SENSITIVE)
  - → `docs/plans/prereg_gain_randomization_gr1_v1_amendment1_2026-11-04.md` (after both trainings, before evaluation)
- **Compare with:** `policy-r-ref5in`, `b1r-classical-baseline`
- **Members:** `docs/plans/prereg_gain_randomization_gr0_v1.md`,
  `docs/plans/prereg_gain_randomization_gr0_v1_amendment1_2026-11-04.md`, `docs/evidence/gain_randomization_2026-11-04/`,
  `docs/plans/prereg_gain_randomization_gr1_v1.md`, `docs/plans/prereg_gain_randomization_gr1_v1_amendment1_2026-11-04.md`

### `camera-only` — Camera-only vs camera + LiDAR: what the LiDAR contributes

- **Period:** 2026-11-04 –.
- **Question:** What difference appears when the LiDAR is not used? How much of the LiDAR's value comes from
  coverage outside the camera's field of view, and how much from correcting the camera inside it?
- **Policy:** Frozen Policy F, unchanged. The LiDAR is removed only in opt-in conditions selected through the
  study's own entry point (D-2026-11-04-3); the official RGB-D + LiDAR contract is unchanged. A later camera-only
  continuation (C3) would be a new policy compared with F.
- **Status:** plan v1.
  - C0 gate_01 C0_CONDITIONAL_PASS. Independent review r1 FAIL (B1: a vacuous association-leak check,
    test-only).
  - The repairs: association scenes, the governor elevation fix, the condition `no_lidar_in_fov_obs_only` and
    PATH. Then gate_02 C0_CONDITIONAL_PASS, and independent review r2 (Opus 5.5) **PASS**, so **C0 is PASS**.
  - M-r2-1 and M-r2-2 answered (tests and plan). C1/C2 preregistered; GPU approved by the user.
  - C1 on `1908fd8`: C1_IDENTITY_PASS. Design review AMEND_FIRST, answered by amendment 1, so C1 is rerun on the
    amendment commit.
  - C1 on `5522be0`: C1_IDENTITY_PASS. C2 ran there to COMPLETE (60 cells, host faults 0), reading REPORTED.
    Independent results review r1 (Opus 5.5): **PASS_WITH_LIMITS**.
    - MAJOR M-1: the author's CPU sensor check ran alongside two C2 cells, disclosed as a deviation.
    - Six MINOR: wording and record.
  - C2 (zero-shot, Frozen Policy F, this grid), crash:
    - removing all LiDAR: +39.95 pp (OOD, includes K2);
    - removing only the out-of-FOV LiDAR: +39.25 pp (OOD, includes K2);
    - removing only the in-FOV LiDAR: −0.69 pp, a detected change below the 2 pp band.
  - Read these with the limits and the claim scope in `docs/evidence/camonly_2026-11-04/README.md`.
  - Found afterwards: 8 of the 12 GR-0 S00 cells used as condition A also ran alongside the author's CPU jobs
    (`gainrand/dev` `352b919`, `load_overlap/`). An idle-host byte-compare rerun awaits the user.
  - Sensor-level CPU check (exploratory, not a gate): `docs/evidence/camonly_2026-11-04/sensor/`.
  - Crash mechanism (CPU, exploratory, post hoc; user request 2026-11-05): `docs/evidence/camonly_2026-11-04/mechanism/`.
    - With the full LiDAR, F flies diagonally forward-left, outside the camera FOV in 51.8 % of moving frames.
      52.0 % of its imminent bars lie outside the FOV.
    - Without the out-of-FOV LiDAR, its lateral reversals fall from 2.07 % to 0.15 % of steps.
    - 38.5 % of the paired first episodes that A captured crash in C.
    - Consistent with blind-side contacts; the contact bearing in C is NOT_MEASURED.
  - G3 (`prereg_camonly_c2trace_v1.md`; run 2026-11-05 on `c4bc416`, 12 traced C2 cells, host faults 0):
    NONINTERFERENCE_PASS. Reading **BLIND_SIDE_SUPPORTED**.
    - out_last: the contact bar is outside the FOV in A 48.8 %, B_out 72.1 % and C 72.9 % of crashes.
    - C − A is +24.0 pp, CI [19.3, 28.7]. 93 % of C's contact bars are on the left.
    - Independent review r1 (Opus 5.5): PASS_WITH_LIMITS. Two MINOR findings: per-density counts added from the
      review; bearings from the vehicle centre, not the camera eye, where C − A is +21.3 pp.
    - Consistent with blind-side contacts; not a causal test. `docs/evidence/camonly_2026-11-04/c2trace/`.
- **Read first:** [`docs/plans/camonly_study_plan_v1.md`](plans/camonly_study_plan_v1.md)
- **Lineage:**
  - D-2026-11-04-3 (camera-only comparison allowed in opt-in conditions)
  - → `docs/plans/camonly_study_plan_v1.md`
  - → `docs/evidence/camonly_2026-11-04/` (C0 gate and reviews)
  - → `docs/plans/prereg_camonly_c1c2_v1.md` (GPU: C1 identity, C2 five zero-shot conditions; user approval 2026-11-04)
  - → `docs/plans/prereg_camonly_c1c2_v1_amendment1_2026-11-04.md` (after the design review AMEND_FIRST; before C2)
  - → `docs/evidence/camonly_2026-11-04/c1c2/` (C1 rerun, C2 results, results review r1)
  - → `docs/evidence/camonly_2026-11-04/mechanism/` (CPU, exploratory: why C crashes)
  - → `docs/plans/prereg_camonly_c2trace_v1.md` (G3: which bar camera-only F hits; GPU session approval 2026-11-05)
  - → `docs/evidence/camonly_2026-11-04/c2trace/` (G3 results, review r1)
- **Compare with:** `gain-randomization`
- **Members:** `docs/plans/camonly_study_plan_v1.md`, `docs/plans/prereg_camonly_c1c2_v1.md`,
  `docs/plans/prereg_camonly_c1c2_v1_amendment1_2026-11-04.md`, `docs/evidence/camonly_2026-11-04/`,
  `docs/plans/prereg_camonly_c2trace_v1.md`

### `load-invariance` — Does concurrent CPU work change a GPU evaluation? (T-LOAD)

- **Period:** 2026-11-05 –.
- **Question:** Can CPU or disk load from another process change the per-episode outcome files of an evaluation
  cell? Its reading decides whether CPU work may run during evaluations (D-2026-11-05-1).
- **Policy:** Frozen Policy F, unchanged, through the camera-only cell path (`full` and `no_lidar` conditions).
- **Status:** ran 2026-11-05 on `812f6b1`, 15 of 15 runs OK, host faults 0. **LOAD_INVARIANT**: every load arm is
  byte-identical to idle in all three cells, with L2 lengthening the cells by 57-72 %. Independent review r1 PASS_WITH_LIMITS. MAJOR-1 narrowed the rule:
  timing results, real-time SITL and untested CPU-side paths keep an idle host.
  - The loaded GR-0 cells of 2026-11-04 (205/4501 and 70/4501) equal the idle runs.
  - CPU compute is allowed during Isaac Gym evaluation cells (D-2026-11-05-1, narrowed).
- **Read first:** [`docs/plans/prereg_tload_v1.md`](plans/prereg_tload_v1.md)
- **Lineage:**
  - 2026-09-05 A8 `S_off` difference under load (WORKLOG)
  - → the 2026-11-04 overlaps and twins (recorded on branch gainrand/dev at 352b919, folder load_overlap)
  - → D-2026-11-05-1 (definition of concurrent work)
  - → `docs/plans/prereg_tload_v1.md`
  - → `docs/evidence/tload_2026-11-05/` (results)
- **Compare with:** `camera-only`, `gain-randomization`
- **Members:** `docs/plans/prereg_tload_v1.md`, `docs/evidence/tload_2026-11-05/`

### `sitl-transfer` — Gazebo/PX4 SITL before real flight: can Policy F fly a PX4 vehicle?

- **Period:** 2026-11-05 –.
- **Question:** Before real-vehicle flight (the user's goal), can MOTAR's Policy F close the loop through a real
  autopilot stack (PX4 v1.14.3 SITL + Gazebo Classic 11): its estimator, controllers, offboard setpoints and
  latency? And what dynamics and sensor gaps must be closed first?
- **Policy:** Frozen Policy F, unchanged. The vehicle is MOTAR's training-spec navrl_quad as a PX4 airframe, with
  PX4's default iris as the comparison.
- **Status:** plan v1. System, build and boot checks done. S1 (iris) and S2 (navrl_quad) offboard tests are
  exploratory.
  - Found and fixed: ODE per-step damping, the thrust model, PX4's norm speed limit.
  - The velocity loop is still to be matched to G9.
  - G8 r2 (2026-11-05) read TARGET_DEPENDS_ON_PURSUER, consistent with global-RNG coupling across envs.
  - G9 attempt 1 lost its report to a locale bug; attempt 2 was contaminated by random placement and an
    unverified raw asset count. The additive v2 one-bar correction produced four contact-free responses in
    attempt 3, but its systemd memory cap was unenforced. Amendment 4 fixes the RSS watchdog.
    G9 attempt4 is recorded with independent review LIMITED_VALIDITY for descriptive response use (REPORTED ONLY);
    no transfer or real-flight acceptance follows.
  - G7 completed7/7 cells at06383eb with byte identity. Opus5.5 independently reproduces k2d0 INCONCLUSIVE,
    k4d0 HARMFUL and k1d1 TOLERANT under the fixed pooled crash rules. One seed/two densities; TOLERANT permits
    bounded harm and is not a PX4/real-flight latency guarantee. No chunked training or later gate was started.
  - The bridge is built.
    - S3a reads NOT_EQUIVALENT. The bridge had dropped F's lateral mu scale (fixed; action error 0.388 → 1.7e-3),
      and a target-slice error is still open.
    - S3b dev01-dev06 are confounded: the mu-scale bug, PX4's EKF diverging under ODE iters 10 (fixed with 100) and
      the GPS height drift (fixed with a rangefinder).
    - **dev07** (development, not acceptance; first 10 envs of seed 4501 at 70 and 205 bars): the SITL outcome
      equals Isaac's in 20 of 20 episodes (19 captures, 1 crash in both). Capture takes a median of 2 steps longer.
      The PX4 velocity loop is not yet matched to MOTAR's.
    - **dev08 preparation:** additive static native/candidate configuration and protocol only;
      runtime NOT_IMPLEMENTED, measurement NOT_RUN. Fixed first-ten cohort execution is blocked:
      zero-action70bars/env5 has208 target frames, not600. Historical dev07 captured at35, before
      exhaustion; its result is preserved. No SITL/GPU run, later gate or flight acceptance follows.
    - **dev08 amendment1 design:** a separate observed-prefix diagnostic preserves the20 scenes;
      records end at checkpoint207/599 with explicit censoring when no physical terminal occurs.
      Offline input/checkpoint/ledger contracts are implemented; live runtime remains NOT_IMPLEMENTED.
      The strict600 parent remains BLOCKED; no prefix measurement or formal stage advancement.
    - Local historical-asset availability is repaired with hash-matched copies of absent files;
      shared PUBLIC_CPU is PASS. This is local validation, not a SITL/flight result.
    - **dev08 runtime stack (motar-5md, CPU preparation record):** the owned session, parameter/telemetry guards,
      prefix step, attempt recorder and guarded runner were implemented with CPU fixtures. No simulator ran
      during that preparation unit; its live status was NOT_VERIFIED and measurement NOT_RUN. Later execution follows below.
      - Native prefix identity with the historical loop holds on the CPU with the real bridge and Policy F.
      - Amendment2, corrected by amendments3/4, proposes the bounded 42-member execution. The user fixed
        `skip_max_s = 0.15` s in [D-2026-11-05-2](DECISIONS.md#d-2026-11-05-2--dev08-skiplate-tolerance-fixed-at-015-s).
        This is the tolerance decision only. Independent r7 review of `821706d` and the small earlier owned/link
        repairs is PASS_WITH_LIMITS; `motar-5md` CPU/runtime acceptance is complete. Permanent-test and inherited
        limits continue in `motar-bmg`; guarded CLI and execution are `motar-1ab`. No formal stage advancement follows.
      - The user subsequently authorized completion through the two smoke and40main members in a new five-hour
        session ([D-2026-11-05-3](DECISIONS.md#d-2026-11-05-3--complete-dev08-through-main-measurement-in-a-new-five-hour-session)).
        Amendment5 adds the guarded CLI, numeric ledger handling and pre-main smoke timing checks;
        the original42-member schedule is retained. Source and launcher reviews are PASS_WITH_LIMITS; the
        reviewed source was committed as `d8f9bc6` before launch.
      - **Dev08 execution COMPLETE:** all42 members (two excluded smoke +40main) recorded, no unrun, host faults0→0.
        Native18capture/2crash versus candidate19capture/1crash, no censoring. At70bars both10/0; at205bars native8/2,
        candidate9/1. Paired transitions18capture→capture,1crash→crash,1crash→capture; jointly captured18pairs have
        candidate-minus-native nominal checkpoint-time median0.0s (min−0.9,max+1.3).
      - Independent result review is **LIMITED_VALIDITY**, acceptable for the preregistered observed-prefix
        descriptive record; integrity/recomputation checks PASS. Exact packet/stream phase/send-receive evidence
        is NOT_RECORDED. No gain choice, complete60s claim or formal stage advancement. Read
        `docs/evidence/sitl_2026-11-05/dev08/execution/analysis.json` and the result review there.
        `motar-1ab` is complete; `motar-agq` follows the shared205/env2 bar failure and differing205/env5 outcomes
        using existing traces/ULogs first. Minor coverage limits remain `motar-bmg`.
      - **AGQ follow-up:** existing four ULogs distinguish pre-contact-proxy tracking differences from later
        angular disturbances, altitude loss and growing position discrepancy. Actual contacts are NOT_RECORDED;
        the diagnosis does not establish causality or select gains. An additive opt-in SDF correction removes
        four extra rotor colliders while preserving the training base box and every dynamics/sensor byte.
        Source review r1 FAIL is retained; repaired source review r2 PASS_WITH_LIMITS covers offline preparation.
        New model live response remains NOT_TESTED. S3a and complete601-state inputs still block formal validation;
        the new structural readiness audit cannot authorize a gate. Read `dev08/agq/README.md` in the evidence folder.
      - **AGQ geometry-v1 (preregistered, NOT_RUN):** four fixed development members (205/env5 native gains
        native→training-box collision; 205/env2 candidate gains training-box→native) with evaluation-only Gazebo
        contact, MAVLink send/arrival and snapshot recording, a pre-takeoff floor-contact positive control and a
        fixed pair reading. CPU implementation and reviews only; execution needs a new scoped approval.
      - **S3a-P1 policy precision (preregistered; GPU NOT_RUN):** Frozen F on the stored G8 r2 observations, CPU
        versus GPU with TF32 ON and OFF set after player construction, one 128-environment batch per call as recorded.
        The CPU preflight reproduces the known action residual (the CPU-versus-recorded maximum is in
        `dev08/agq/s3a_p1/`). The GPU conditions need a scoped approval; no S3a verdict changes.
- **Read first:** [`docs/plans/sitl_validation_plan_v1.md`](plans/sitl_validation_plan_v1.md)
- **Lineage:**
  - user direction 2026-11-05 (real flight, SITL check first)
  - → `docs/plans/sitl_validation_plan_v1.md`
  - → `docs/evidence/sitl_2026-11-05/` (S1/S2 offboard tests, vehicle files)
  - → `docs/plans/prereg_sitl_reference_g8g9_v1.md` (G8 reference dump, G9 Isaac velocity response)
  - → `docs/plans/prereg_sitl_reference_g8g9_v1_amendment1_2026-11-05.md` (G8 adds a state recorder; before any run)
  - → `docs/plans/prereg_sitl_reference_g8g9_v1_amendment2_2026-11-05.md` (G8 adds zero-action cells: full target trajectories)
  - → `docs/plans/prereg_sitl_reference_g8g9_v1_amendment3_2026-11-05.md` (G9 additive one-bar correction before attempt 3)
  - → `docs/plans/prereg_sitl_reference_g8g9_v1_amendment4_2026-11-05.md` (retain attempt 3's resource deviation; enforce RSS watchdog for confirmation/G7)
  - → `docs/plans/prereg_actionhold_g7_v1.md` (G7: F under held / delayed actions; chunking premise and latency budget)
  - → `docs/evidence/sitl_2026-11-05/g9/attempt4_report.json` (four-env one-bar response; descriptive LIMITED_VALIDITY)
  - → `docs/evidence/sitl_2026-11-05/g7/analysis.json` (fixed G7 readings, actual Opus5.5 independent verification)
  - → `docs/plans/prereg_sitl_dev08_candidate_gains_v1.md` (configuration preparation, incomplete target input blocker)
  - → `docs/evidence/sitl_2026-11-05/dev08/` (source/input audit, CPU preparation checks; measurement NOT_RUN)
  - → `docs/plans/prereg_sitl_dev08_candidate_gains_v1_amendment1_2026-11-05.md` (observed-prefix input/runtime design)
  - → `docs/evidence/sitl_2026-11-05/dev08/local_availability/` (canonical local asset copies and full CPU validation)
  - → `docs/evidence/sitl_2026-11-05/dev08/runtime_stack/` (runtime stack CPU implementation, reviews, identity evidence)
  - → `docs/plans/prereg_sitl_dev08_candidate_gains_v1_amendment2_2026-11-05.md` (runtime facts, bounded execution proposal)
  - → `docs/plans/prereg_sitl_dev08_candidate_gains_v1_amendment3_2026-11-05.md` (review r5 corrections: held signals, outer timeout, stop list)
  - → `docs/plans/prereg_sitl_dev08_candidate_gains_v1_amendment4_2026-11-05.md` (one compute thread; timing basis for skip_max_s)
  - → `docs/plans/prereg_sitl_dev08_candidate_gains_v1_amendment5_2026-11-05.md` (authorized CLI, numeric records, pre-main smoke timing)
  - → `docs/evidence/sitl_2026-11-05/dev08/execution/` (authorized execution preparation, source/result reviews and records)
  - → `docs/evidence/sitl_2026-11-05/dev08/agq/` (existing-log mechanism diagnosis and additive CPU preparation)
  - → `docs/plans/sitl_agq_followup_v1_2026-11-06.md` (geometry-only intervention, S3a precision controls and new input lineage proposal)
  - → `docs/plans/prereg_sitl_agq_geometry_v1_2026-11-06.md` (four-member geometry diagnostic: recording, stop rules, fixed reading; NOT_RUN)
  - → `docs/plans/prereg_sitl_s3a_policy_precision_v1_2026-11-06.md` (S3a-P1: stored-observation CPU/GPU TF32 control; GPU NOT_RUN)
- **Compare with:** `camera-only`, `gain-randomization`
- **Members:** `docs/plans/sitl_validation_plan_v1.md`, `docs/evidence/sitl_2026-11-05/`,
  `docs/plans/prereg_sitl_reference_g8g9_v1.md`, `docs/plans/prereg_sitl_reference_g8g9_v1_amendment1_2026-11-05.md`,
  `docs/plans/prereg_sitl_reference_g8g9_v1_amendment2_2026-11-05.md`,
  `docs/plans/prereg_sitl_reference_g8g9_v1_amendment3_2026-11-05.md`,
  `docs/plans/prereg_sitl_reference_g8g9_v1_amendment4_2026-11-05.md`,
  `docs/plans/prereg_actionhold_g7_v1.md`, `docs/plans/prereg_sitl_dev08_candidate_gains_v1.md`,
  `docs/plans/prereg_sitl_dev08_candidate_gains_v1_amendment1_2026-11-05.md`,
  `docs/plans/prereg_sitl_dev08_candidate_gains_v1_amendment2_2026-11-05.md`,
  `docs/plans/prereg_sitl_dev08_candidate_gains_v1_amendment3_2026-11-05.md`,
  `docs/plans/prereg_sitl_dev08_candidate_gains_v1_amendment4_2026-11-05.md`,
  `docs/plans/prereg_sitl_dev08_candidate_gains_v1_amendment5_2026-11-05.md`,
  `docs/plans/sitl_agq_followup_v1_2026-11-06.md`, `docs/plans/prereg_sitl_agq_geometry_v1_2026-11-06.md`,
  `docs/plans/prereg_sitl_s3a_policy_precision_v1_2026-11-06.md`

## Not yet in the repository

- **C-Planner P0** (2026-10-29 →): its task did not allow commits, so the repository holds no record
  of it yet, not even a WORKLOG entry.
  - Its records are outside the repository, in `~/motar_tmp/` ([`AGENTS.md`](../AGENTS.md),
    *Work and evidence outside the repository*): `p0_v6_20261029/`, `p0_v7_20261029/`,
    `p0r_20261029/` and their `*_independent_*` reviews.
  - State: v7 FAIL. In the P0-R redesign, R2 attempt 1 was aborted by host hardware faults.
  - It joins the occlusion group when its first record is committed.
- **Merging `origin/main`:** its three publication commits add
  `docs/evidence/website_boia_progress_2026-10-22/`, `docs/evidence/website_figure_rebuild_2026-10-22/`,
  `docs/research/boia-progress.md` and `docs/research/figure-redesign-2026-10-22.md`. The merge
  commit must add them to a family here, or the test fails (`motar-lp5.2`).

## What this page does not cover

Contracts, paper drafts, public-release audits and other reference documents at the top of `docs/`
are outside the test's scope (see the top of this page). Some of them are research records, such as
the D8c audit and the renderer freeze; the family they belong to names them. Find the rest through
[`README.md`](README.md).
