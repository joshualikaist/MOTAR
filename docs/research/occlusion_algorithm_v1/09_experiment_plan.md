# 09 — Experiment design: density × target motion × occlusion

**Status: DESIGN.** GPU stages G6 and G7 each need a committed preregistration — RunID, seeds, source
SHA, scale, wall/VRAM/disk caps and stop rules — and separate user approval. Nothing here is run.

## 1. Arms

All arms share Policy F (`f7022139…`), the unchanged governor and controller, and the online
whitelist. Arms F1–F5 also share BOIA's two-tier safety screen and SAFE_HOLD, so the safety layer is
isolated in F1 and not confounded with visibility.

| Arm | Definition | Purpose |
| --- | --- | --- |
| F0 | Policy F unchanged | Reference |
| F1 | F + safety-only arbitration: BOIA with w_l = w_u = 0 (capture + clearance + stopping feasibility) | Visibility-blind planner; isolates the safety contribution |
| F2 | F + reactive recovery (candidate A, H-R3 B3 geometry), with the F1 screen | Prior-art reactive baseline |
| F3 | F + predictive visibility: BOIA with P_loss only (w_u = 0), oracle p_det | Visibility forecast without belief/covariance |
| F4 | F + belief-aware visibility: BOIA with P_loss + U, no capture term (w_c = 0) | Belief term without mission coupling |
| F5 | Full BOIA | Proposed |

**Ablations** (exploratory only, inside G7; not part of a confirmatory family):
- calibration off (τ_on = 0, always arbitrate);
- reactive-only (PRE_OCCLUSION disabled);
- BPOD analytic in place of Monte Carlo;
- `unknown_blocks` = 0.

A run with the safety layer removed is **not** an arm. Unsafe execution is never compared; safety
alternatives are compared only through F1.

## 2. Factors

| Factor | Levels | Notes |
| --- | --- | --- |
| Obstacle density | 160 and 205 bars (primary); 70 and 115 (secondary) | Matched `navrl_band` lineage only. Bar count is not physical density, and this lineage is never pooled with the non-overlapping lineage |
| Target motion | E2_direct_l2_v1 (primary); E0 static (secondary) | The TF32-exposed bounded "E2" lineage is never used |
| Start | Visible at 10 m (primary); hidden (secondary, first-acquisition descriptive) | The hidden start is a separate problem (04 §3) |
| Detector realism | Oracle (as built); stressed with p_miss 0.1 and 0.3, implemented as an **opt-in evaluation wrapper** that drops camera detections before the tracker | The stressed setting makes the frozen Policy F out of distribution. It is labelled OOD and analysed separately |
| Seeds | New, never used before (exclude 4101–4110, 4190, 4201–4207, 4301–4314, 4501–4503, 4601–4602, 4801–4803) | Independent seed clusters are the unit of inference |

## 3. Endpoints

**Primary.** The cumulative incidence of crash within 2 s after an incident clear→blocked
`center_occlusion_in_view` onset, after first acquisition.
- **Estimator:** Aalen–Johansen.
- **Competing events:** geometric restoration, FOV/range exit, capture and timeout.
- **Inference:** equal-weight seed means, with seed-block bootstrap.
- **Tool:** the existing frozen tool `tools/occlusion_confirmatory_analysis.py` (OCC-P2) is reused
  unchanged.
- **Policy F baseline (OCC-P2):** 3.23 % at 160 bars and 3.82 % at 205.

**Co-primary guards (non-inferiority, harm limits preregistered).**
- Episode capture: F5 − F0 ≥ −1.0 pp.
- Episode crash: F5 − F0 ≤ 0.

**Secondary.**
- 2 s FOV-exit CIF;
- fused resumption after restoration;
- capture by 5 s after onset;
- spatially identity-correct re-observation within 2 s after FULL/CENTRE_ONLY first loss
  (H-R3 P03 metric);
- occlusion duration distribution;
- time to capture (competing risks);
- intervention rate and duration;
- hand-back Policy F action change;
- predicted vs realised P_det reliability (Brier, ECE, against E-L0 labels offline);
- hard-screen false-alarm rate;
- realised path length and closing speed;
- controller latency P95/P99;
- first acquisition time (hidden start, descriptive).

**Never:**
- selecting episodes by realised post-treatment motion;
- matching realised paths;
- pooling across lineages;
- comparing with numbers published in other simulators.

## 4. Pairing and isolation

- **Layouts.** Arms run the same seeds and layouts (the same scene hashes).
- **Branching.** If within-episode branching from a common prefix is used, it must use the qualified
  P03 isolation runner, and the qualification must be re-run for every new seed and density profile.
- **Otherwise unpaired.** Arms are analysed as unpaired streams with layout-level matching of
  episode starts only. They are expected to diverge after the first intervention; that divergence is
  the treatment, not a confound.

## 5. Sample size (planning assumption, not a power analysis)

**From M3 §5.**
- The seed SD of crash-ended intervals per interval at 205 bars is 0.012.
- A 1 pp absolute reduction from 3.8 % needs about 30 seeds per arm, unpaired.

**Plan.**
- A paired-layout design is simulated in the preregistration from OCC-P2 seed-level data before the
  number of seeds is fixed.
- The exploratory G7 pilot is capped at 4 seeds × 2 densities × 6 arms, moving E2, visible start.
- Wall time is NOT_RECORDED for the BOIA overhead. It is measured in G6 (one seed, one density,
  F0 + F5, short profile) before G7 is sized.

## 6. Gates (V0–V8) with stop conditions

| Gate | What | Pass condition | Stop / return |
| --- | --- | --- | --- |
| V0 Provenance | Paper manifest, source/Policy F/RMS hashes, protected WIP untouched | All hashes match | Mismatch → stop |
| V1 Simulator | Reset isolation qualification for the chosen profile; terminal-first trace check | ISO/P03-style gates PASS | Cross-env drift → stop |
| V2 Sensor/geometry | 2D line of sight against E-L0 3D silhouette (offline); known-free against layout (offline evaluator) | Disagreement ≤ 5 % of frames | Exceeded → redesign §4/§5 |
| V3 Tracker | NEES/NIS of the shadow belief at 0.5/1.0 s; expected-covariance check | NEES within the χ² 95 % band in ≥ 80 % of cases | Fails → inflate Q_b, else H = 0.5 s, else return to G3 |
| V4 Dynamics | Rollout error on recorded pre-contact traces | 95th-percentile position error at 1 s ≤ 0.15 m; speed error ≤ 0.2 m/s | Fails → recalibrate, else return to G3 |
| V5 Planner CPU | U7 replay: P_det reliability, hard-screen false alarms ≤ 2 %, hysteresis, timing ≤ 5 ms per decision (N = 128) | All pass | Forecast adds nothing over the camera flag → drop BOIA (candidate redesign) |
| V6 GPU qualification | One seed, one density, F0 + F5, short profile; same SHA; no retry | Determinism, no OOM, deadline met | Stop and report |
| V7 Exploratory | Preregistered pilot (§5) | Report effect sizes with seed-cluster CIs; no claim | — |
| V8 Confirmatory | New seeds, preregistered family (primary + guards), Holm across density cells | — | Separate approval |
