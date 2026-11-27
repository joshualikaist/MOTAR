# Simulation experiment map

One page that says, for every simulation experiment family of MOTAR, what was asked, on which policy, what the
record says (its label, verbatim), the headline numbers with the file they come from, and on which branch and
commit the record lives. Written for the paper draft, because the records are spread over five local branches
and none of the work after 2026-10-23 is on GitHub `main`.

**Written:** 2026-11-23, from the worktrees at `camonly/dev` `5d9610e`, `camonly/t1` `4da43ab`, `gainrand/dev`
`575b026`, `cplanner/dev` `2837b18` and `codex/mt-p1` `42415ff`.
**Method:** every label and number below is copied from the file named next to it (by four read-only extraction
passes over `docs/EXPERIMENTS.md`, `docs/EVIDENCE.md`, `docs/HISTORY.md` and each family's own records). Nothing
is re-derived or rounded. Where this page and a record disagree, the record is right. The tested pages stay the
source of truth: [`EVIDENCE.md`](EVIDENCE.md) (numbers), [`EXPERIMENTS.md`](EXPERIMENTS.md) (families and
members), [`HISTORY.md`](HISTORY.md) (milestones).
**Scope:** simulation only (Isaac Gym, and the Gazebo/PX4 SITL transfer track in section 5). No real-flight
result exists anywhere in the repository.

## 1. Where the work is

### 1.1 Branches

```
main (local 198a200; GitHub main a32932f, 2026-10-23)
 └─ codex/mt-p1 (42415ff) ── main research line to 2026-11-03 (KROC27, OCC, EQ, S-L1, H-R3, BOIA, Q02, B1-R)
      ├─ gainrand/dev (575b026) ── gain randomization GR-0 / GR-1 / GR-1b
      │    └─ camonly/dev (5d9610e) ── camera-only study, T-LOAD, G7, then the SITL transfer track (newest)
      │         └─ camonly/t1 (4da43ab) ── CAM-T1 (the last Isaac Gym experiment, 2026-11-12)
      └─ cplanner/dev (2837b18) ── C-Planner M1–M3, direction-bound repair, D1-F
```

| Branch | Worktree | Holds | Not in it |
| --- | --- | --- | --- |
| `codex/mt-p1` | `src/MOTAR-mt-p1` | every family of section 3 (through B1-R, 2026-10-24) | everything from 2026-11-04 on |
| `camonly/dev` | `src/MOTAR-camonly` | all of `codex/mt-p1` (except one AGENTS.md text commit), GR-0, GR-1 training records, camera-only C0–CAM-K2-E3, T-LOAD, G7, SITL | GR-1 results/GR-1b, C-Planner, CAM-T1 |
| `camonly/t1` | `src/MOTAR-camonly-t1` | `camonly/dev` as of `eefb832` (2026-11-10) + CAM-T1 (2 commits) | `camonly/dev` after 2026-11-12 (transfer-track work only) |
| `gainrand/dev` | `src/MOTAR-gainrand` | GR-1 results, load-overlap deviation, GR-1b (7 commits beyond `camonly/dev`) | camera-only |
| `cplanner/dev` | `src/MOTAR-cplanner` | C-Planner (18 commits beyond `camonly/dev`) | camera-only, GR |

Counts on 2026-11-23: `camonly/dev` is 154 commits ahead of GitHub's `codex/mt-p1` (`13d5f19`); `camonly/t1`,
`gainrand/dev` and `cplanner/dev` add 2, 7 and 18 commits respectively. None has been merged into `main`.

### 1.2 Raw data

Raw results (`results/**`) are never committed. On the maintainer's workstation they live in the main checkout
`src/aerial_gym_simulator/results/` (worktrees link to them), or in `~/motar_tmp/<unit>_<date>/` for the
2026-11 units. Each committed record binds its raw folder by path and SHA-256 manifest. The committed records
(`docs/evidence/`, `docs/plans/`, `docs/results/`, `results/*/README.md`) are what the branches carry.

### 1.3 Policies that appear in this map

| Name | What it is | SHA-256 prefix |
| --- | --- | --- |
| **Policy F** | ep25000 + riskcap, frozen 2026-08-05; the reference policy of every family from section 3 on | `f7022139…` |
| Policy R | ref5in D1 ep1900; observation-contract diagnostics only, never compared with F | `197ea269…` |
| GR-1 `rand40` / `ctrl` | F continued 1,000 epochs with / without gain randomization (section 4.1) | `3f122b47…` / `0f6ca010…` |
| CAM-K1 s1 / s2 | F continued 3,000 epochs camera-only at 66° × 54° (section 4.2) | `d7bf51a8…` / `5128bf2e…` |
| CAM-K2 v2 s1 / s2 | CAM-K1 continued 1,000 epochs at T/W 4.0 (INFERRED), motor randomization, OAK-D depth model | `769a26f4…` / `e76eb048…` |
| CAM-K2 v3 s1 / s2 | CAM-K2 v2 recipe + depth-validity rule; the policy bound into the ROS 2 runtime | `888e8056…` / `203deea4…` |

Every continuation is a new policy compared with F; none replaces F (D-2026-11-04-1).

## 2. Timeline overview

One row per family, in date order. "Label" is the newest recorded verdict, verbatim. Section numbers point to
the detail below.

| Period | Family | Question in one line | Policy | Newest label | Branch | § |
| --- | --- | --- | --- | --- | --- | --- |
| 2026-07 – 08 | `policy-f` | Can a learned policy reach a moving target in a bar field; which speed filter to freeze? | → F | F frozen 2026-08-05; riskcap "capture supported; crash not confirmed" | mt-p1 | 3.1 |
| 08-06 – 08-22 | `latency-and-pose` | How much capture does latency cost, and what corrects it? | F | GO in the registered contract; real-world latency not solved | mt-p1 | 3.2 |
| 08-05 – 09-02 | `learned-detector` | Does a learned detector keep navigation performance? | F (R for distractors) | Non-inferiority PASS; COLOR_SHORTCUT_CONFIRMED | mt-p1 | 3.3 |
| 08-13 – 09-03 | `policy-r-ref5in` | Does the policy adapt to the 1.2 kg ref5in platform? | R | D1 gates FAIL; RANGE_INCONCLUSIVE_AT_THIS_BUDGET | mt-p1 | 3.4 |
| 08-22 – 09-02 | `physical-target-routing` | Can a physically simulated target follow a route? | own | FAIL_ROUTE_MECHANISM; FAIL_BLOCKS_CONFIRMATORY | mt-p1 | 3.5 |
| 08-27 | `corrected-geometry` | Do the bars overlap; which densities are supported? | — | lineage break; 70–205 bars | mt-p1 | 3.6 |
| 09-02 – 09-09 | `safety-filter-attribution` | Does arc clearance lower crashes vs riskcap? | F (R for A-series) | −1.4903 pp crash; C3 WITHDRAWN | mt-p1 | 3.7 |
| 09-03 – 09-11 | `perception-pipeline` | What does measured perception error cost F; does retraining recover it? | F + descendants | −4.57 pp; readaptation INCONCLUSIVE | mt-p1 | 3.8 |
| 09-10 | `eth-ds5` | Is an image-size range proxy usable on real footage? | — | SIZE_RANGE_USABLE; E3-P BLOCKED | mt-p1 | 3.9 |
| 09-11 – 09-14 | `renderer` | Can an independent renderer reproduce the camera images? | — | RENDERER_CONTRACT_V1_COMPLETE; area control NOT ADOPTED | mt-p1 | 3.10 |
| 09-12 – 09-24 | `dynamic-mesh` | Does a moving target mesh cost Policy R? | R | D8b MATERIAL_LOSS; causality NOT_TESTED | mt-p1 | 3.11 |
| 09-12 – 09-14 | `public-release-records` | Can the public snapshot be checked from outside? | — | release layer in place; provenance NEEDS_CONFIRMATION | mt-p1 | 3.12 |
| 09-14 | `task-diagnostics` | Does the diagnostics instrumentation change behaviour? | R | TRAJECTORY_INVARIANT; T1/T2 not run | mt-p1 | 3.13 |
| 09-17 – 09-18 | `browser-preview` | Does the browser preview reproduce GT episodes? | — | FROZEN (engineering validation, not research) | mt-p1 | 3.14 |
| 09-03 – 09-26 | `target-motion` | How does capture change with target motion (E0/E1/E2 vs H)? | F | REPLICATED (n = 7) | mt-p1 | 3.15 |
| 09-18 – 09-21 | `external-baseline` | Can a published tracker be reproduced for a matched comparison? | — | UPSTREAM_REPRODUCTION_FAIL; Class A = 0 | mt-p1 | 3.16 |
| 10-14 – 10-16 | `kroc2027` | Does the initial view change capture (A); does a search supervisor help (B)? | F | H-A SUPPORTED; H-A2, H-B INCONCLUSIVE | mt-p1 | 3.17 |
| 10-16 – 10-17 | `occlusion-occ` | How often and how long is the target occluded? | F | RQ1, RQ3 SUPPORTED; RQ2 INCONCLUSIVE_EQUIVALENCE_NOT_TESTED | mt-p1 | 3.18 |
| 10-17 – 10-18 | `environment-qualification` | Is the environment fit for the claims? | F | LIMITED_VALIDITY; E-L0 14/14 PASS | mt-p1 | 3.19 |
| 10-18 | `sensor-fusion-corrections` | Which fusion defects can be corrected opt-in? | F | S-L1A–D QUALIFIED | mt-p1 | 3.20 |
| 10-18 – 10-19 | `h-r3` | Does translating help re-observe a lost target? | F | PROVISIONAL / CONFIRMATION HOLD | mt-p1 | 3.21 |
| 10-19 – 10-21 | `boia` | Can an occlusion-aware layer be qualified (passive first)? | F | exact OFF/ON parity PASS; G5 HOLD | mt-p1 | 3.22 |
| 10-21 – 10-22 | `q02-reacquisition` | Can the target be re-identified from sensor evidence? | — | Q02-A PASS_WITH_EXPLICIT_IDENTITY_LIMITATIONS; Q02-B/Q03 HOLD | mt-p1 | 3.23 |
| 10-23 – 10-24 | `b1r-classical-baseline` | Does F beat classical controllers on the same task path? | F vs B-PP, B-PN | C1, C2 SUPERIOR | mt-p1 | 3.24 |
| 11-04 – 11-05 | `gain-randomization` | Is F sensitive to controller gains; does training with randomized gains help? | F, GR-1 arms | S40 GAIN_SENSITIVE; GR-1 SUPPORTED; GR-1b NOT_CONFIRMED | gainrand | 4.1 |
| 11-04 – 11-12 | `camera-only` | What does the LiDAR contribute; can a camera-only continuation fly? | F, CAM-K1/K2 | C2 +39.95 pp crash; K1-E MATERIAL_RECOVERY + MATERIAL_LOSS; E3 NO_DETECTED_CHANGE; T1 DEFINITION_SENSITIVE | camonly (+t1) | 4.2 |
| 11-05 | `load-invariance` | Does concurrent CPU work change a GPU evaluation? | F | LOAD_INVARIANT | camonly | 4.3 |
| 11-05 | G7 (in `sitl-transfer`) | Can F fly with held or delayed actions? | F | k2d0 INCONCLUSIVE, k4d0 HARMFUL, k1d1 TOLERANT | camonly | 4.4 |
| 11-04 | `c-planner` | What does a sound static-bar certificate cost, and does it hold? | none (CPU planner) | M1 PASS; M2 FAIL → repair PASS; M3 part 1 PASS; D1-F PASS; capture NOT_RECORDED | cplanner | 4.5 |
| 11-05 – 11-21 | `sitl-transfer` | Can F fly a PX4 vehicle in Gazebo; real-flight preparation | F, CAM-K2 v3 | S3b FEASIBLE_IN_SITL; no outdoor GO | camonly | 5 |

## 3. Families on `codex/mt-p1` (also in `camonly/dev`)

Read-first paths are relative to the repository root. `EVIDENCE.md` numbers are tested against their records
by `tests/test_canonical_docs.py`.

### 3.1 `policy-f` — Policy F: training, speed screens and limits
- **Question:** can a learned policy reach a moving target in a dense bar field from vision and LiDAR, and
  which speed filter makes it safe enough to freeze?
- **Policy:** ep24000 (`82f7978b…`) → ep25000 TTC branch → **Policy F = ep25000 + riskcap** (`f7022139…`),
  frozen 2026-08-05.
- **Status:** F frozen; riskcap adaptation "capture supported; crash not confirmed (its interval contains
  zero)" (`docs/EVIDENCE.md`). Training-contract audit 2026-09-17.
- **Numbers:** riskcap adaptation capture +3.75 pp, 95 % CI [+1.30, +6.19] (`docs/EVIDENCE.md`); crash
  −2.14 pp, 95 % CI [−4.42, +0.15]; uniform_trained_riskcap capture 81.94 %, crash 15.67 % vs uniform_off
  70.03 % / 27.87 % (`results/navrl_v2_riskcap_postadapt/summary.md`).
- **Read first:** `results/navrl_v2_riskcap_postadapt/summary.md`; contract audit
  `docs/audits/frozen_policy_training_contract_2026-09-17.md`.
- **Limits:** every run predates the 2026-08-27 geometry correction, so its numbers do not compare with
  later families. The frozen training contract differs from HEAD defaults in 14 settings.

### 3.2 `latency-and-pose` — Latency, clock and pose premises
- **Question:** how much capture is lost to latency and imperfect clock/pose, and what corrects it?
- **Policy:** F. **Status:** "GO in the registered contract; real-world latency is not solved"
  (`docs/EVIDENCE.md`).
- **Numbers:** capture 37.82 % → 78.04 % (+40.21 pp) with ego-motion correction; latency_0p1s_raw crash
  58.22 % vs latency_0p1s_p3 19.67 %; seed 47, 205 bars, 2,049 episodes per cell
  (`results/navrl_v2_latency_ego_motion/summary.md`).
- **Read first:** `results/navrl_v2_latency_ego_motion/summary.md`. **Limits:** pre-correction geometry
  (historical only).

### 3.3 `learned-detector` — Learned target detector in the navigation loop
- **Question:** does a learned detector keep navigation performance, and through which channel does dropout
  cost capture?
- **Policy:** F (distractor envelope on R). **Status:** navigation non-inferiority PASS (replicated);
  detector robustness NEGATIVE; COLOR_SHORTCUT_CONFIRMED (2026-09-02).
- **Numbers:** in-loop detector −0.015 pp, 95 % CI [−1.752, +1.723], "Non-inferiority PASS at a −2 pp
  margin; not superiority" (`docs/EVIDENCE.md`); distractor-lock share `default` 88.53 % / `v7` 90.27 % at
  5 distractors, threshold ≥ 50.00 % (`results/navrl_detector_distractor_envelope_seed479/summary.md`).
- **Read first:** `results/navrl_v2_detector_navigation_ab_replication_seed97_101_schema2/summary.md`.
- **Limits:** mostly pre-correction; F's detector is in effect a colour rule.

### 3.4 `policy-r-ref5in` — ref5in reference platform and Policy R
- **Question:** does the policy adapt to the 1.2 kg ref5in platform; what limits first acquisition?
- **Policy:** R (`197ea269…`). **Status:** P2 held-out and D1 adaptation gates FAIL; P3 BLOCKED; detection
  range RANGE_INCONCLUSIVE_AT_THIS_BUDGET.
- **Numbers:** D1 held-out global capture +8.80 pp [+7.44, +10.16], crash −7.36 pp [−8.63, −6.09] vs its
  reference, labelled FAIL against its gates (`results/navrl_ref5in_d1_eval_seed331/summary.md`).
- **Read first:** `results/navrl_ref5in_d1_eval_seed331/summary.md`. **Limits:** Policy R records are
  diagnostics; never compared with F.

### 3.5 `physical-target-routing` — Physical target, routing and braking
- **Question:** can a physically simulated target follow a route through the bar field?
- **Policy:** none of F/R. **Status:** FAIL_ROUTE_MECHANISM (four rounds); braking-v3 FAIL_BLOCKS_CONFIRMATORY,
  "closes the physical-target GPU authority" (2026-09-01/02).
- **Numbers:** v3 mean-speed ratio 0.7872 / 0.7681 / 0.5984 / 0.6297 at 0.60/0.90/1.20/1.25 m/s against a
  0.80 minimum, all FAIL; soft-envelope exits 6,825 vs allowed 0
  (`docs/braking_aware_route_v3_lower1p25_matched_spawn_result_2026-09-01.md`).
- **Read first:** that file. **Limits:** spans the 08-27 correction; "the preregistered failure prediction was
  confirmed".

### 3.6 `corrected-geometry` — Corrected bar geometry (lineage break)
- **Question:** do the bars overlap, and which densities does a non-overlapping layout support?
- **Status:** lineage break 2026-08-27; densities 70–205.
- **Numbers:** 205 bars connectivity 99.167 %, no-route 0.833 % PASS; 300 bars 94.661 % / 5.339 % FAIL;
  highest passing density 250; gates ≥ 95 % / ≤ 5 %, 60 layouts × 128 pairs
  (`results/navrl_v2_density_geometry_audit_2026-08-27/summary.md`).
- **Read first:** that file. **Limits:** spawn clearance 0.65 m; `start_in_inflated_obstacle_frac` 27.45 % at
  205 bars before the 0.6 m snap.

### 3.7 `safety-filter-attribution` — Safety-filter attribution (stopcap, A1–A8, grid, R-C)
- **Question:** where do the remaining crashes come from, and does clearance along the turning arc lower
  them relative to riskcap?
- **Policy:** F for the arc contrast and the F grids; R or descendants for A3–A8, R-C.
- **Status:** arc − riskcap −1.4903 pp independently recomputed (2026-09-07); C3 "co-adaptation decides the
  law" WITHDRAWN (R-C 0/2); five runs VOID.
- **Numbers:** −1.4903 pp crash, 95 % CI [−1.8981, −1.0826]; seed-level 95 % CI [−2.42, −0.57]; lower in
  15/15 seed × density cells (`docs/EVIDENCE.md`; `results/independent_verification_2026-09-07/README.md`);
  stopcap screen "off−riskcap crash +9.23 %", M1 IMPLEMENTATION_VOID
  (`results/navrl_v2_ep25000_stopcap_seed49_screen/summary.md`).
- **Read first:** `results/independent_verification_2026-09-07/README.md`.
- **Limits:** "implemented arc-clearance speed filter", not a DWA planner, not a collision guarantee; the 15
  cells share three seeds and one policy.

### 3.8 `perception-pipeline` — Perception error and its cost to Policy F (S0–S4, P3–P10)
- **Question:** what does the measured perception error cost F, and does retraining under it recover it?
- **Policy:** F source arms; one F descendant (`db6d9f05…`) and three more for the seed replication.
- **Status:** P10 −4.57 pp; readaptation INCONCLUSIVE (2026-09-11); S1 shadow RECOGNITION_DOMINANT;
  depth-noise order VARIANCE_INSENSITIVE.
- **Numbers:** −4.57 pp capture, 95 % CI [−6.32, −2.82], 3,161/4,099 vs 3,350/4,101 episodes over two
  evaluation seeds; readaptation +0.73 pp, 95 % CI [−1.04, +2.50], training seeds +1.41 / +0.78 / −0.01 pp,
  residual cost −1.90 / −2.71 / −3.84 pp (`docs/EVIDENCE.md`;
  `results/perception_p10_seed_replication_2026-09-10/README.md`); temporal selector +0.00697 validation
  utility, S4 test 0.4701 (`docs/EVIDENCE.md`).
- **Read first:** `results/perception_p10_seed_replication_2026-09-10/README.md`.
- **Limits:** one density, one injector; "a determinism check, not three replications"; retraining costs
  0.74–1.25 pp of clean capture.

### 3.9 `eth-ds5` — Range proxy on real footage
- **Question:** is an image-size range proxy usable on real UAV footage? **Policy:** none.
- **Status:** E3-S SIZE_RANGE_USABLE; E3-P ATTITUDE_NOT_RELIABLE → BLOCKED.
- **Numbers:** 6.2 % median absolute relative range error (median of block medians), 3,107 frames, 9 blocks of
  15 s from one flight (`docs/EVIDENCE.md`); G2 central 0.062, G3 stability 0.093
  (`results/eth_ds5_e3s_2026-09-10/README.md`).
- **Read first:** `results/eth_ds5_e3s_2026-09-10/README.md`. **Limits:** one flight; proxy, not ground-truth
  boxes; frames not independent; does not chain with P10. Licence CC BY-NC-SA 4.0.

### 3.10 `renderer` — Independent renderer track (R1–R5)
- **Question:** can an independent renderer reproduce the simulator's camera images within tolerance?
- **Status:** RENDERER_CONTRACT_V1_COMPLETE (not a blanket PASS); area control NOT ADOPTED; normal
  representation N0 ONLY; R4/R4b criterion C FAIL (freeze 2026-09-14).
- **Numbers:** R2b AREA_CONTROL_FAILED, held-out median 8.132 % vs a 5 % gate
  (`docs/renderer_track_v1_freeze_2026-09-14.md`).
- **Read first:** that file. **Limits:** "a failed renderer gate is a preserved scientific result".

### 3.11 `dynamic-mesh` — Dynamic target mesh and policy sensitivity (D6–D8)
- **Question:** does a moving target mesh change the detector, and does it cost Policy R?
- **Policy:** R. **Status:** D6 INCONCLUSIVE; D7 GO; D8b MATERIAL_LOSS, causality NOT_TESTED; D8c user
  decision NO_RETRAINING_REQUIRED_FOR_CURRENT_CLAIM (2026-09-24).
- **Numbers:** −48.967 pp capture, 95 % CI [−50.113, −47.821], paired seed n = 3 (t, df = 2)
  (`docs/EVIDENCE.md`; `results/dynamic_mesh_policy_sensitivity_d8b_2026-09-13/README.md`); D6 cost ratio
  1.5338 in the 1.5–3.0× INCONCLUSIVE band (`results/dynamic_mesh_raycast_feasibility_2026-09-12/README.md`).
- **Read first:** the D8b README. **Limits:** on R at 70 bars, not F; no renderer property isolated as cause.

### 3.12 `public-release-records` — Public release and record envelope
- **Status:** release layer in place; dataset reachability OPEN; several provenance items NEEDS_CONFIRMATION
  (`docs/public_release_status_2026-09-14.md`). No research numbers.

### 3.13 `task-diagnostics` — Task diagnostics instrumentation (T1, T2)
- **Policy:** R. **Status:** TRAJECTORY_INVARIANT (70 bars, seed 593, 128 episodes per arm); "the
  preregistered TD-T1 and TD-T2 results have not been run"
  (`results/task_diagnostics_invariance_2026-09-14/README.md`).

### 3.14 `browser-preview` — Browser ground-truth preview
- **Status:** FROZEN as GT_BROWSER_EPISODE_V1 (2026-09-18). "Not the PPO policy, not PhysX, and not research
  performance evidence" (`docs/status/gt_browser_episode_v1_freeze_2026-09-18.md`). Not for the paper.

### 3.15 `target-motion` — Target-motion generalisation and its replication
- **Question:** how does F's capture change when the target moves differently from training (E0 static, E1,
  E2 obstacle-aware) vs the training target H?
- **Policy:** F. **Status:** REPLICATED, n = 7 (2026-09-26); NO_RETRAIN_NEEDED WITHDRAWN (09-19).
- **Numbers:** E0 −5.37 pp [−5.78, −4.85], E1 −3.45 pp [−3.79, −3.11], E2 +1.35 pp [+1.07, +1.87]; n = 7
  fresh seeds, exact sign-flip p = 0.015625, Holm p = 0.046875; 229,376 primary episodes; original campaign
  n = 3: E0 −3.58, E1 −2.82, E2 +1.33 pp (`docs/EVIDENCE.md`;
  `results/target_motion_replication_7seed/README.md`).
- **Read first:** `results/target_motion_replication_7seed/README.md`.
- **Limits:** E2 − H not uniform at high density (5/7 at 160, 4/7 at 205, exploratory); mechanism "consistent
  with" visibility, association only; historical moving-target cells TF32-exposed; n = 3 and n = 7 not pooled.

### 3.16 `external-baseline` — External matched baseline (Elastic Tracker)
- **Status:** B0 UPSTREAM_REPRODUCTION_FAIL (environment-blocked: ROS needs `sudo`); **Class A matched
  comparisons = 0** (`docs/EVIDENCE.md`;
  `docs/external_baselines/elastic_tracker_b0_upstream_reproduction_2026-09-20.md`).

### 3.17 `kroc2027` — KROC2027 initial-view study and MT precision
- **Question:** does the pursuer's initial view of the target change F's capture, differently for static vs
  moving targets (Exp A); does a conditional search supervisor help on out-of-view starts (Exp B); are the
  historical moving-target counterexamples real (MT)?
- **Policy:** F. **Status:** H-A (Delta_A) SUPPORTED; H-A2 (I_A) INCONCLUSIVE; H-B INCONCLUSIVE, B1 not
  adopted; H-MT1 rejected (TF32 `torch.cdist`).
- **Numbers:** Delta_A +2.709961 pp, 95 % t CI [+2.452724, +2.967198], Holm p 4.49e-07; I_A −0.261579 pp
  [−0.801079, +0.277920], Holm p 0.2803; neither clears the preregistered 3 pp practical threshold
  (`docs/evidence/mt_precision_review_2026-10-16/README.md`); Exp B primary paired-t mean 0.0020751953125,
  p 0.1336, INCONCLUSIVE (`results/kroc2027_B_v1/analysis.json`); B gate visible − hidden capture
  0.028407505580357144 (n = 7), never_acquired failure share 0.5932960893854748
  (`results/kroc2027_B_v1/b_gate.json`).
- **Read first:** `docs/evidence/mt_precision_review_2026-10-16/README.md`.
- **Limits:** "Delta_A is not a fully blinded independent confirmation"; "lack of an interaction finding is not
  evidence of equivalence"; E1/E2 160-bar counterexamples came from TF32
  (`docs/audits/moving_target_tf32_readonly_2026-10-16.md`).

### 3.18 `occlusion-occ` — Natural occlusion and recovery observation (R-P1a, OCC-P1/P2)
- **Question:** how often and for how long is the target occluded, by density and target motion?
- **Policy:** F. **Status:** OCC-P2 complete; RQ1 SUPPORTED, RQ3 SUPPORTED, RQ2
  INCONCLUSIVE_EQUIVALENCE_NOT_TESTED (2026-10-17).
- **Numbers:** 112 cells / 57,344 primary quota episodes, 14 seeds; occlusions per observation minute:
  static 205 − 70 bars 3.126 [2.893, 3.359]; moving 205 − 70 6.078 [5.790, 6.366]; moving − static at 205
  5.449 [5.078, 5.821] (`docs/evidence/occ_p2_confirmatory_2026-10-17/REPORT.md`).
- **Read first:** that REPORT. **Limits:** centre-ray geometric occlusion, whole-system effects; no interval
  lasted 5 s (tail NOT_ESTIMABLE).

### 3.19 `environment-qualification` — EQ-0, EQ-1, EQ-1D, E-L0
- **Question:** is the simulation environment fit for the claims made on it? **Policy:** F.
- **Status:** environment verdict LIMITED_VALIDITY (dynamics, sensors, information flow, reproducibility);
  E-L0 all 14 gates PASS, environment unchanged (2026-10-18).
- **Numbers:** D2-R3 delivered 160 × 90 depth FAIL: 92.7 % of robust pixels within 1e-4 m, 10,673 (2.3 %)
  off by > 0.05 m, max 7.3 m; D2-N2 LiDAR negatives FAIL 7/7; D1-H1 hover thrust 2.452504 N vs 2.4525 N PASS
  (`docs/evidence/eq1d_qualification_2026-10-18/REPORT.md`); E-L0 898-D actor bytes identical, 0 differing
  steps (`docs/evidence/e_l0_evaluation_2026-10-18/REPORT.md`).
- **Read first:** the EQ-1D REPORT. **Limits:** "a PASS here does not certify hardware physics or real-camera
  redetection"; EQ-1A/B/C FAILs not fixed.

### 3.20 `sensor-fusion-corrections` — S-L1A–D
- **Question:** which camera–LiDAR fusion defects can be corrected behind opt-in switches without changing F?
- **Status:** S-L1A_QUALIFIED 6/6, S-L1B 10/10, S-L1C 9/9, S-L1D 8/8 gates (each
  `docs/evidence/s_l1*/REPORT.md`); C-v2 record reads GPU_QUALIFIED_PENDING_REVIEW while `EXPERIMENTS.md`
  says "GO in a limited scope" — report this conflict when citing.
- **Numbers:** C-v2 GPU positive decisions 212 unchanged + 124 geometry recoveries + 96 abstentions; v1
  rejects 220 (`docs/evidence/s_l1c_v2_2026-10-18/RESULTS.md`).
- **Read first:** `docs/SENSOR_CONTRACT.md`. **Limits:** S-L1A has no in-repository independent review; no
  hardware realism or recovery performance established.

### 3.21 `h-r3` — Re-observation by translation
- **Question:** does translating, not only yawing, help re-observe a lost target? **Policy:** F.
- **Status:** PROVISIONAL / CONFIRMATION HOLD; P01 exploratory; P02 INTEGRITY FAIL, NO-GO; P03-Q01
  GO_QUALIFICATION_ONLY.
- **Numbers:** P01 B3 − B2 +36.4 pp [+13.6, +59.1], 17/22 vs 9/22, discordant 9/1, p 0.021
  (`docs/evidence/h_r3_pilot_2026-10-18/REPORT.md`); P03-Q01 +30.77 pp, one seed, population inference
  NOT_ESTIMATED (`docs/evidence/h_r3_p03_preparation_2026-10-19/REPORT.md`).
- **Read first:** the P03 REPORT. **Limits:** one seed; all comparisons fail the realized-balance diagnostic.

### 3.22 `boia` — Occlusion-handling algorithm (passive observer)
- **Status:** PASS_EXACT_NATIVE_OFF_ON_PARITY_WITH_REPLAY_CONTROL (BOIA-SHADOW-Q01-V2, 2026-10-21): S1_OFF vs
  S1_ON parity 30/30, 31/31; passive observer only; G5 HOLD
  (`docs/evidence/boia_shadow_q01_v2_run_2026-10-21/REPORT.md`).
- **Read first:** `docs/research/occlusion_algorithm_v1/BOIA_RESEARCH_MASTER_PLAN.md`. **Limits:** no benefit,
  safety or real-time result; no online integration.

### 3.23 `q02-reacquisition` — Evidence-aware reacquisition (CPU only)
- **Status:** Q02-A v2.6 PASS_WITH_EXPLICIT_IDENTITY_LIMITATIONS (3.5–13 % fail-safe loss, "mostly no support
  beyond ~13 m"); Q02-B/Q03 package review FAIL as a stage gate, two P1 → HOLD
  (`docs/evidence/q02b_q03_opus_review_2026-10-22/REVIEW.md`). Nothing ran in Isaac Gym.

### 3.24 `b1r-classical-baseline` — Classical sensor-based baselines
- **Question:** on the identical OCC-P2 task path, does F beat classical controllers steering on the same
  tracker belief? **Policy:** F vs pure pursuit (B-PP) and proportional navigation (B-PN).
- **Status:** COMPLETE; C1 (F − B-PN) and C2 (F − B-PP) SUPERIOR (2026-10-24).
- **Numbers:** F − B-PP +8.87 pp, BCa 95 % CI [+8.54, +9.23]; F − B-PN +30.38 pp [+29.95, +30.84];
  seed-level mean capture F 95.33 %, B-PP 86.46 %, B-PN 64.96 %; n = 7, 7/7 same sign, sign-flip
  p = 0.015625, Holm p = 0.03125; exploratory: 205 bars crash F 9.54 % vs B-PP 27.68 %, RMTC60 70 → 205
  bars F 4.84 → 10.05 s, B-PP 8.33 → 24.01 s (`docs/EVIDENCE.md`).
- **Read first:** `docs/evidence/b1r_v1_audit/AUDIT_REPORT.md`.
- **Limits:** internal matched baselines, not Class A; reactive controllers without a map; B-PN's gap partly
  arena exits (34–84 % of its crashes); tuned on one seed; capture is a swept < 0.5 m close approach.

## 4. Families of November 2026 (branch-specific)

### 4.1 `gain-randomization` — Policy F under randomized controller gains (`gainrand/dev`)

| Unit | Commits (prereg → results) | Design | Labels | Numbers (source) |
| --- | --- | --- | --- | --- |
| GR-0 | `3c31a14` → `2e45ca0`, review `8f64631` | F frozen; 4 densities × seeds 4501-4503 × S00/S20/S40 = 36 cells, 2,048 quota episodes each | identity_02 IDENTITY_PASS; COMPLETE; **S20 GAIN_ROBUST; S40 GAIN_SENSITIVE**; review PASS (Opus 5.5) | S00 crash 4.46 %, S20 5.07 %, S40 8.15 %; Δ S40 +3.69 pp [+3.03, +4.37]; Δ S20 +0.61 pp [+0.25, +0.98] (`docs/evidence/gain_randomization_2026-11-04/README.md`) |
| GR-1 | `847e777` → `6b3ac27` | F continued 1,000 epochs, `rand40` vs `ctrl`, one training seed per arm; 72 cells | TRAINED; GR1_REPRO_PASS; COMPLETE; **GR1_SUPPORTED** "reduced gain sensitivity, single training seed"; review PASS (Opus 5.5) | P1 crash at ±40 % rand40 − ctrl −2.21 pp [−2.68, −1.76]; P2 nominal capture +1.20 pp [+0.75, +1.70]; DiD crash gap −1.02 pp [−1.37, −0.71]; ctrl − F drift +0.54 pp (`…/gr1/gr1_analysis.json` via README) |
| GR-1b | `31d5761` → `e507ff3` | same with training seed 2 | TRAINED; GR1_REPRO_PASS; COMPLETE; pair GR1_NOT_SUPPORTED; **NOT_CONFIRMED**; review PASS (Opus 5.5) | P1 seed 2 +0.444 pp [+0.000, +0.875]; P2 −0.448 pp [−0.850, −0.094]; DiD +0.004 pp [−0.623, +0.618] (`…/gr1b/gr1b_analysis.json`, `confirmation.json`) |

- **Reading:** F's crash rate is sensitive to ±40 % gain variation; the GR-1 improvement was not reproduced
  with a second training seed, so no gain-randomization variant is adopted and F stays frozen.
- **Deviations recorded:** author CPU jobs overlapped 23/36 GR-0 cells, both GR-1 trainings and 25/72 GR-1
  evaluation cells (`…/load_overlap/`, `352b919`); readings stand as recorded; T-LOAD (4.3) later read
  LOAD_INVARIANT. GR-1b: a tracked file was edited during training and reverted at once.
- **Raw:** `aerial_gym_simulator/results/gain_randomization_gr{0,1,1b}_v1/` (main checkout).
- **Limits:** simulation only, fixed 205-bar training, nothing about PX4 or real flight; about half the S40
  crash increase is early floor strikes at 41–64° tilt.

### 4.2 `camera-only` — What the LiDAR contributes; camera-only continuations (`camonly/dev`, `camonly/t1`)

Evidence folder: `docs/evidence/camonly_2026-11-04/`. Grid "GR-0 S00": F reference cells, E2 target,
70/115/160/205 bars × seeds 4501-4503, 2,048 quota episodes per cell. Decision D-2026-11-04-3 (opt-in LiDAR
ablation; the official RGB-D + LiDAR contract is unchanged) and D-2026-11-06-1 (the real vehicle has no LiDAR;
OAK-D Pro RGB 66° × 54°).

| Unit | Date / commits | Policy | Labels | Numbers (source) |
| --- | --- | --- | --- | --- |
| C0 switch | 11-04, `4d03f35` → `86202c5` | F (CPU gate) | review r1 FAIL repaired; gate_02 C0_CONDITIONAL_PASS; review r2 PASS → **C0 PASS** | 14 tests, 7 mutants caught (`…/README.md`) |
| C1 identity | 11-04, `1908fd8`, `5522be0` | F | C1_IDENTITY_PASS (twice) | 14 of 14 checks |
| **C2 zero-shot LiDAR removal** | 11-04, run on `5522be0`, results `f821058` | F, 60 cells + 12 reference | COMPLETE; REPORTED; review **PASS_WITH_LIMITS** (M-1 deviation) | crash `full` 4.46 %, `no_lidar` 44.40 %, `no_lidar_out_fov` 43.71 %, `no_lidar_in_fov` 3.76 %; total +39.95 pp [+33.95, +45.52]; out_fov +39.25 pp [+33.65, +44.71]; in_fov −0.69 pp [−1.07, −0.33]; gov_all +5.05 pp [+4.15, +5.83] (`…/c1c2/c2_analysis.json` via README) |
| Sensor-level CPU check | 11-04, `a7d368b` | none (traces) | exploratory, no label | 0–2 m phantom 0.2 % / miss 9.1 %; 8–10 m 17.8 % / 42.1 % (`…/sensor/sensor_check_01.json`) |
| Crash mechanism | 11-05, `de1a892` | F telemetry | exploratory, "consistent with blind-side contacts" | travel outside FOV 51.8 % of moving frames; lateral reversals 2.07 % → 0.15 % of steps; 38.5 % of A-captured first episodes crash in C (`…/mechanism/`) |
| G3 traced cells | 11-05, `90c00b4` → `05a630e` | F, 12 traced reruns | NONINTERFERENCE_PASS; **BLIND_SIDE_SUPPORTED**; review PASS_WITH_LIMITS | out_last A 48.8 %, C 72.9 %; C − A +24.0 pp [19.3, 28.7] (`…/c2trace/c2trace_analysis.json`) |
| CAM-K1 training | 11-07, `5e58984` → `814c28a` | F → CAM-K1 s1/s2, 3,000 epochs, 66° × 54°, no LiDAR | SMOKE_OK; both seeds **TRAINED**; review PASS_WITH_LIMITS | rollout capture s1 8.45 → 26.29 %, s2 7.79 → 25.55 % (`…/camk1/training_curve.json`) |
| **CAM-K1-E** | 11-08, `971ae67` → `ab2eeb5` | CAM-K1 vs F66 (F camera-only at 66°) and A (F + LiDAR) | IDENTITY_PASS; COMPLETE; gate R **MATERIAL_RECOVERY** (both seeds); gate P **MATERIAL_LOSS** (both); review PASS_WITH_LIMITS | pooled crash A 4.46, C87 44.40, F66 53.89, K1s1 16.22, K1s2 17.15 %; R −37.67 [−38.45, −36.78] / −36.74 pp; P +11.77 [+11.26, +12.30] / +12.69 pp; F66 − C87 +9.49 pp (`…/camk1e/analysis.json`) |
| CAM-K2 v1 | 11-08, `62e2699` → `aa0e691` | — | **WITHDRAWN_BEFORE_RUN** (real T/W about 4.0, not 2.25) | none |
| CAM-K2 v2 training | 11-08, `aa0e691` → `04f341c` | CAM-K1 → v2, 1,000 epochs, T/W 4.0 INFERRED, motor randomization, OAK-D depth model | SMOKE_OK; both **TRAINED**; design review r2 PASS_WITH_LIMITS | commands above 2 N 5.95 / 5.25 % (`…/camk2/README.md`) |
| **CAM-K2-E** | 11-08/09, `1237f9f` → `d2a018b` | v2 vs CAM-K1 under the treatments | gate A **SMALL_ADAPTATION**; gate N **NO_MATERIAL_LOSS**; gate I SEEDS_DISAGREE; `adaptation_supported` false; review PASS | crash K1 treated 18.64 / 19.58 %, K2 treated 14.84 / 14.84 %; A −3.80 [−4.63, −2.93] / −4.74 [−5.26, −4.24] pp; N −2.99 / −4.18 pp (`…/camk2e/analysis.json`) |
| CAM-K2 v3 training | 11-09, `d85589d` → `326e307` | v2 recipe + depth-validity rule `blob_near` | both **TRAINED**; **DIAG_OK** | hit_token 0.6958 / 0.6898 vs v2 0.1668 / 0.1916 (`…/camk2/README.md`) |
| **CAM-K2-E3** | 11-10, `aaca40e` → `504eb4e` | v3 and v2 + rule vs v2 | gate D **NO_DETECTED_CHANGE** (both seeds); R, T SEEDS_DISAGREE; review PASS (Codex) | crash V2 14.8397 %, V3 13.3382 / 13.6922 %; D −1.5015 [−2.0264, −0.9766] / −1.1475 [−1.7619, −0.5493] pp (`…/camk2e/analysis_v3e.json`) |
| **CAM-T1** (`camonly/t1`) | 11-12, `b3b9f0b` → `4da43ab` | CAM-K2 v3; 24 traced + 36 envelope cells | NONINTERFERENCE_PASS; Stage A **DEFINITION_SENSITIVE**; Stage B d1 **WORSE** (both seeds), cap25/cap20 SEEDS_DISAGREE; review ACCEPT_WITH_LIMITS | footprint profile IN_VIEW 67.7 % vs nearest-point BLIND 42.5 %; d1 +4.95 [4.30, 5.60] / +3.67 [3.12, 4.17] pp; native crash 15.97 / 17.37 % (`…/camt1/RESULTS_2026-11-12.md`, `camonly/t1`) |

- **What the study established (labels only):** removing all LiDAR from F raises crash by +39.95 pp; nearly
  all of that comes from the out-of-FOV LiDAR (+39.25 pp), the in-FOV part reads −0.69 pp; the loss is
  "consistent with" blind-side contacts (G3 BLIND_SIDE_SUPPORTED). A camera-only continuation (CAM-K1)
  recovers most of it (gate R MATERIAL_RECOVERY) but stays +11.77 / +12.69 pp above F with LiDAR (gate P
  MATERIAL_LOSS). Vehicle-matched retraining (CAM-K2 v2) lowers crash further by −3.80 / −4.74 pp under the
  treatments but adaptation is not supported (gate I SEEDS_DISAGREE); the depth-validity rule (v3) reads
  NO_DETECTED_CHANGE in evaluation; a one-step delay makes v3 WORSE (+4.95 / +3.67 pp).
- **Raw:** `results/camonly_c1c2_v1/`, `results/camonly_c2trace_v1/` (main checkout);
  `~/motar_tmp/camk1e_20261108/raw/`, `camk2e_20261108/raw/`, `camk2v3e_20261110/`, `camt1_20261112/raw/`;
  training runs under `aerial_gym/rl_training/rl_games/runs/` and `~/motar_tmp/cam*`.
- **Limits:** one grid, Frozen F or its continuations, OOD conditions; T/W 4.0 of the real 1.1 kg vehicle is
  INFERRED; the near-bar depth gap (MinZ 0.4 m) effect is NOT_TESTED; nothing about a real OAK-D is recorded.

### 4.3 `load-invariance` — T-LOAD (`camonly/dev`)
- **Question:** can CPU or disk load from another process change a GPU evaluation's per-episode outcome files?
- **Status:** **LOAD_INVARIANT** (15 of 15 runs byte-equal, host faults 0 → 0); review PASS_WITH_LIMITS; rule
  narrowed to Isaac Gym outcome-file cells (D-2026-11-05-1). Wall time rose up to +72 % under load
  (`docs/evidence/tload_2026-11-05/README.md`; prereg `812f6b1`, results `26a4a57`, review `a278c2b`).
- **Limits:** timing results, real-time SITL and CPU-side decision paths still need an idle host.

### 4.4 G7 — Policy F under held or delayed actions (`camonly/dev`, recorded in `sitl-transfer`)
- **Question:** can unchanged F fly with open-loop action chunks (k = 2, 4 steps) or a one-step (100 ms) delay?
- **Status:** identity PASS; COMPLETE 7/7; **k2d0 INCONCLUSIVE, k4d0 HARMFUL, k1d1 TOLERANT**; results review
  r1 (Opus 5.5) all checks PASS. Prereg `f3930c6`, run on `06383eb`, results `b58b389`.
- **Numbers:** pooled crash Δ k2d0 +2.001953125 pp [0.977, 3.027]; k4d0 +10.4736328125 pp [9.229, 11.743];
  k1d1 +1.513671875 pp [0.537, 2.515]; k4d0 at 205 bars crash 26.26953125 % vs F 8.837890625 %
  (`docs/evidence/sitl_2026-11-05/g7/analysis.json`, CIs in `REVIEW_results_r1.md`).
- **Limits:** one seed, two densities; TOLERANT "is not a PX4/real-flight latency guarantee".

### 4.5 `c-planner` — Certified predictive planner (`cplanner/dev`, CPU only)

Charter `docs/plans/cplanner_charter_v1.md`; evidence `docs/evidence/cplanner_2026-11-04/`. No learned policy;
F unchanged. P0 history (v1–v7, P0-R, r1; 2026-10-28 → 11-03) is outside the repository in `~/motar_tmp/`.

| Unit | Commits | Labels | Numbers (source, under `docs/evidence/cplanner_2026-11-04/`) |
| --- | --- | --- | --- |
| M1 (R2' + review R3') | `b407039` → `954311f` | CONDITIONAL → review PASS (Opus 5.5) → **R2' PASS**, M1 complete | attacks 106/106; scenarios FALSE_CERT 0; map invariants 0 violations in 115,831,788 cell checks; verbatim certify p95 612 ms, reported only (`README.md`, `r2prime/`) |
| M2 (V1 + G-alpha) | `bf1ebc7`, `a5b7022` → `3a58f5a`, review `2e5ffd7` | **FAIL** (V1 one violation in 691,200 beams); G-alpha FILTER_ARM_RETAINED 0.816, 0 FALSE_CERT, from the FAIL run; review FAIL confirming | range error max 2.48e-4 m; certifiable 4,969 / 6,089 = 0.816 (`m2/v1.json`, `m2/galpha.json`) |
| M3 part 1 (E1, E2, G-beta) | `265b244` → `2db38a1`, review `1aee5ee` | E1 PASS (0 mismatches), E2 PASS, **G-beta PASS_K16**; review PASS (Opus 5.5) | step + certify K = 16 p95 20.7 ms, limit 50 ms (fastest P-core, idle host); native vs verbatim 0.26 vs 33.6 ms per segment (`m3/gbeta.json`, `m3/e1.json`) |
| Direction-bound repair + amendment 1 | `8e2b77d` → `f5b4802` (FAIL) → `ed4265a`, review `45e7999` | first run FAIL (D2); D2-A1 PASS; D3 FILTER_ARM_RETAINED; review PASS (Opus 5.5) → **repair PASS** for `MOTAR-72x4-RUNTIME-DIR`; V1 stays FAIL for `MOTAR-72x4-RUNTIME` | D3 certifiable 0.8162, 0 FALSE_CERT; reviewer probe 0 FALSE_CERT repaired vs 1,577 unrepaired (`dirbound/`) |
| D1-F adversary | `37762ab` → `e10f37c`, review `42285d1` | PASS twice; review PASS (Opus 5.5), closes repair review I1 | repaired 0 FALSE_CERT / 4,435 certified vs eps = 0 2,880 FALSE_CERT / 8,917 (`dirbound/d1f/d1f.json`) |
| Hardening | `5944d43` | not a gate; 8/8 binding checks; planner default = repaired configuration | (`hardening/`) |

- **Reading:** the certificate is sound on the CPU checks after the repair, and its compute fits the 50 ms
  budget at K = 16 on an idle P-core; **C-Planner capture is NOT_RECORDED** (no closed-loop result, no
  comparison with F or B-PP yet).
- **Limits:** charter §4 assumptions only; not a safety, real-flight, dynamic-object or real-time claim; M3
  review I2 (E1 mutant blind spots) open; G-beta not re-measured on the repaired native configuration.

## 5. Transfer track: `sitl-transfer` (Gazebo/PX4 SITL and real-flight preparation, `camonly/dev`)

Not an Isaac Gym study. Read first `docs/plans/sitl_validation_plan_v1.md`; evidence
`docs/evidence/sitl_2026-11-05/` (47 commits, `8f58402` → `5d9610e`). Policy F unchanged; the vehicle is
MOTAR's training-spec navrl_quad as a PX4 airframe, PX4's iris as the comparison.

| Unit | Date | Label (verbatim) | Key numbers (source under `docs/evidence/sitl_2026-11-05/`) |
| --- | --- | --- | --- |
| S1 / S2 offboard (iris, navrl_quad) | 11-05 | exploratory | S2 rise 0.88 → 0.38 s after MPC_XY_VEL P 2.5 (`README.md`) |
| G9 attempt 4 (Isaac velocity response) | 11-05 | LIMITED_VALIDITY (REPORTED ONLY) | four contact-free responses (`g9/attempt4_report.json`) |
| S3a bridge equivalence | 11-05 | NOT_EQUIVALENT | action error 0.388 → 1.7e-3 after the mu-scale fix; target-slice error open (`s3a/`) |
| S3b dev07 | 11-05 | development, not acceptance | 20/20 episodes same outcome as Isaac; EKF error median ≤ 0.16 m (`README.md`) |
| dev08 execution | 11-05 | COMPLETE; review LIMITED_VALIDITY | native 18 capture / 2 crash vs candidate 19 / 1 of 20 (`dev08/execution/analysis.json`) |
| S3a-P1 | 11-06 | TF32_EXPLAINS_POLICY_RESIDUAL | no S3a verdict change (`dev08/agq/s3a_p1/`) |
| **S3b formal v1** | 11-06 | **FEASIBLE_IN_SITL**; review PASS_WITH_LIMITS | 70 bars SITL 122/122 captured vs Isaac 121; 205 bars 27 captures / 3 crashes vs 26 / 4; 162 VALID (`s3b_formal/`) |
| S3c (iris, delay 1) | 11-07 | reported only; review PASS_WITH_LIMITS | iris 40/40 captured; delay1 37 captures / 3 crashes (+7.5 pp, 95 % CI 1.6–20.4 %) (`s3c_s4/`) |
| S4 (Gazebo's own sensors) | 11-07 | reported only; review PASS_WITH_LIMITS | ground arm 20 captures / 17 crashes / 2 timeouts of 39; camera recall 0.634 and 0.479 (`s3c_s4/`) |
| S4F-1 (2 × 2 appearance × ground) | 11-08 | REPORTED, neither contrast reached after Holm | capture 55 / 60 / 67.5 / 75 % (n = 40 each); ground +13.75 pp (Holm p 0.069) (`s4f1/`) |
| S5 safety v2 | 11-10 | COMPLETE / PASS_SITL; live discrimination NOT_TESTED | 10/10 failsafe scenarios (`s5_safety_v2/attempt01_reading.json`) |
| S4R-1 repeatability | 11-10 | COMPLETE / REPORTED_DESCRIPTIVE | 12 fresh boots valid, max step 0.128 s (`s4r1/attempt02_reading.json`) |
| Data utility audit r1 | 11-10 | descriptive; review PASS_WITH_LIMITS | 462 ULogs, 460 complete; four manifests 0 missing / 0 mismatch (`data_utility/`) |
| Camera-only ROS 2 runtime (CAM-K2 v3) | 11-11 – 11-13 | PASS_WITH_LIMITS; ROS graph / DDS / flight NOT_TESTED | CPU source only (`camera_runtime/`) |
| Outdoor profile unit A | 11-13 – 11-15 | reviews r1–r3 FAIL on criterion 2; **no outdoor GO** | synthetic only (`camera_runtime/OUTDOOR_*`) |
| Unit B (PX4 vertical fields, refusal hold) | 11-18 | PASS_WITH_LIMITS; P2-1 open | PROVISIONAL 3.0 s hold bound (`camera_runtime/OUTDOOR_UNIT_B_*`) |
| Unit C (OAK-D field recorder) + preflight r4 | 11-18, 11-21 | r3, r4, r4b, r4c PASS_WITH_LIMITS; device, Orin, firmware NOT_TESTED | Sync 10 → 60 ms; IMX378 ISP 4/13 (`camera_runtime/OAKD_RECORDER_*`) |

Recorded as open for real flight: S3a NOT_EQUIVALENT; "no real-flight or 205-bar claim follows" (S3b); PX4
velocity loop not matched to MOTAR's; S4 capture loss with Gazebo's own sensors not explained; outdoor profile
criterion 2 FAIL until real OAK-D recordings exist; recorder never run on the device.

## 6. Comparable groups (from `EXPERIMENTS.md`, condensed)

1. **Policy F on the corrected geometry:** safety-filter-attribution (F grids), perception-pipeline (P10 source
   arms), target-motion, kroc2027, occlusion-occ, environment-qualification, sensor-fusion-corrections, h-r3,
   boia, b1r-classical-baseline, gain-randomization (S00 cells), camera-only (`full`), load-invariance, G7.
   Check the checkpoint SHA, seed count (2 / 3 / 7), stopping window (n = 3 legacy vs n = 7 quota are not
   pooled), TF32 exposure and task path before comparing two numbers.
2. **Policy R is not Policy F:** policy-r-ref5in, dynamic-mesh, task-diagnostics, the A-series and R-C,
   S1 shadow, depth-noise order, the distractor envelope.
3. **Speed filter:** policy-f (riskcap) → safety-filter-attribution (arc clearance) → b1r-classical-baseline
   (shared governor contract). Pre-correction riskcap numbers do not compare with later families.
4. **Perception error:** learned-detector, perception-pipeline (injected), eth-ds5 (measured); the two do not
   chain into one estimate.
5. **Occlusion, re-observation, reacquisition:** occlusion-occ → h-r3 → boia → q02-reacquisition → c-planner.
   None changes F; none has a qualified online system.
6. **Sensor-fusion corrections:** compare each S-L1 lineage only through its own qualification record.
7. **Physical target and geometry:** physical-target-routing, corrected-geometry.
8. **Camera-only chain:** C2 (`full` = GR-0 S00) → CAM-K1-E (reference A = GR-0 S00, C87 = C2 `no_lidar`) →
   CAM-K2-E (reference = CAM-K1-E cells) → CAM-K2-E3 (reference = CAM-K2-E cells) → CAM-T1 (reference = E3 V3
   cells). Each step reuses the previous step's cells as its reference, so the chain is one grid and one seed
   set (4501-4503); the SITL S3b/S3c/S4 runs use Policy F, the ROS 2 runtime uses CAM-K2 v3.

## 7. For the paper: what is established, and the wording limits

- **Frozen-policy results (Policy F):** target-motion REPLICATED (n = 7); B1-R SUPERIOR vs B-PP and B-PN;
  arc clearance −1.4903 pp crash; perception error −4.57 pp capture; KROC27 Delta_A +2.71 pp SUPPORTED but
  below the 3 pp threshold; OCC-P2 occlusion rates SUPPORTED; GR-0 S40 GAIN_SENSITIVE; C2 LiDAR removal
  +39.95 pp crash; G7 k4d0 HARMFUL, k1d1 TOLERANT; S3b FEASIBLE_IN_SITL.
- **New-policy results (not F):** readaptation INCONCLUSIVE; GR-1 SUPPORTED then GR-1b NOT_CONFIRMED;
  CAM-K1 MATERIAL_RECOVERY + MATERIAL_LOSS; CAM-K2 v2 SMALL_ADAPTATION, adaptation not supported; CAM-K2 v3
  NO_DETECTED_CHANGE; CAM-T1 DEFINITION_SENSITIVE, d1 WORSE. Each is "a new policy compared with F".
- **Not established (write as such):** no real-flight claim; no state-of-the-art claim; Class A matched
  external comparisons = 0; the arc filter is the "implemented arc-clearance speed filter", not a DWA planner
  or a safety guarantee; the target-motion mechanism is "consistent with" visibility, not causal; D8b
  causality NOT_TESTED; C-Planner capture NOT_RECORDED; BOIA, H-R3 and Q02 have no online or confirmed
  result; the environment verdict is LIMITED_VALIDITY; the real vehicle's T/W is INFERRED.
- **Known conflicts to report when citing:** S-L1C v2 reads GPU_QUALIFIED_PENDING_REVIEW in its record and
  "GO in a limited scope" in `EXPERIMENTS.md`; `docs/PROJECT_STATE.md` is last verified 2026-10-21 and does
  not describe the November branches; `docs/paper_draft_2026-09-05.md` predates everything from section 3.7 on.

## 8. Not in the repository

- C-Planner P0 v1–v7, P0-R and the r1 repair (2026-10-28 → 11-03): records only in `~/motar_tmp/`.
- Raw results of every unit (section 1.2).
- GitHub `main` (`a32932f`) and `codex/mt-p1` (`13d5f19`) stop at 2026-10-23 and 2026-10-19; the three
  publication commits of `origin/main` (`website_boia_progress_2026-10-22/`,
  `website_figure_rebuild_2026-10-22/`, two `docs/research/*.md`) are not in any local research branch and must
  join a family when merged (`motar-lp5.2`).
