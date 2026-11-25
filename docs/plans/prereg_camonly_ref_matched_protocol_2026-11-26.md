# Preregistration: CAM-REF — F, CAM-K1 and CAM-S2b-FT on one protocol ladder, 205 bars

- **Status:** preregistration, committed before any GPU cell (2026-11-26). Preparation and the stage-1 GPU run approved by the user ("1,2번 하고"; "4번까지 구현 승인.", 2026-11-26); stage 2 needs a separate approval. Package report `~/motar_tmp/camref_20261126/PACKAGE_REPORT.md`; independent review r1 (Claude Opus 5.5) GO_WITH_FIXES, `~/motar_tmp/camref_20261126/review_r1/REVIEW.md` (P1-1 commit hygiene and P1-2 complete-block analysis fixed before this commit).
- **Why:** the three policies were measured under different protocols, so their crash rates cannot be compared
  (claims audit 2026-11-26, `~/motar_tmp/review_20261126/audit_opus/AUDIT.md`):
  - Policy F (frozen ep25000 riskcap, camera 87° + 360° LiDAR): 8.8 % crash at 205 bars, GR-0 S00 cells
    (`results/gain_randomization_gr0_v1/gr0_main/S00/`), protocol P_K;
  - CAM-K1 (camera-only, 66°): 26.5 / 27.7 % at 205 bars, CAM-K1-E, protocol P_K;
  - CAM-S2b-FT (camera-only, 66.7°, 60°/s): 28.5 % crash / 70.4 % capture, CAM-DX1 V312, protocol P_S.
  The next training design depends on two unknowns: (a) how much of S2b-FT's 28.5 % is protocol rather than policy or
  sensing; (b) whether a policy with 360° ranging reaches crash ≤ 20 % (the user's bar) under the CAM-S2 task rules.
  Design source: `~/motar_tmp/camref_20261126/FEASIBILITY.md` (key-by-key receipt diff in `work/env_diff.txt`).
- **Evaluation only.** No training, no checkpoint change. Each policy keeps its own perception (sensor, FOV,
  resolution, detector, P9 / depth model) and its own trained control stack (S2b-FT: yaw scale 1.0472, governor FREE
  2.5, camsafe).

## Policies and checkpoints (SHA-256 pinned in the runner)
- **F:** `last_gen_ppo_ep_25000_rew_39.742134.pth`, `f702213936601860995cf61dcc570247e72543b1976e3716055cd8ec5593ad40`, read-only
  copy `MOTAR-camonly/aerial_gym/rl_training/rl_games/runs/ppo_260805_0413_navrl_v2-speedgov-ep24000-205bars-main-riskcap-s1/nn/`.
- **CAM-K1 seed 1:** `last_gen_ppo_ep_28000_rew_-17.796482.pth`, `d7bf51a89b0e8124687e618cf8d95735713d36af2eb34fc5b8819efb14715beb`,
  copy `~/motar_tmp/camk1_20261106/train_s1/checkpoint/` (its TRAINED train report). "K1" below is this checkpoint.
- **CAM-S2b-FT:** `last_gen_ppo_ep_15000_rew_51.018074.pth`, `f2c80fcc2b2eaeb24e0b05d2284e74594a7ed8e7953f6b17b7dce49b09f2bbab`,
  run `MOTAR-camonly/.../ppo_261125_1344_navrl_v2-cams2b-ft-yaw60-camonly-s1/nn/`, training receipt
  `~/motar_tmp/cams2b_20261124/train_ft/receipt.json` (`3411d59c…`).

## Protocol points
- **P_K (K1-E):** KROC2027 A3 start (10 m, boresight, clear line of sight), E2 bounded waypoint target, 0.5 m swept
  capture, no delay, nominal vehicle and gains, altitude 1.0 m, camera mount 0.10 m, analytic target.
- **P_S (CAM-S2, DX1 V312):** visible start (6-12 m, bearing within ±28.35°), legacy mixed target motion, capture
  = ≤ 1 m AND closing ≤ 1 m/s AND detected, 0.40 m swept target contact = crash, delay U{1,2} steps, vehicle (T/W 3.5
  ±15 %, imbalance ±5 %, τ 0.02-0.04 s) and gain (±20 %) randomisation, yaw 60°/s, governor FREE 2.5, altitude 1.5 m,
  mount 0.08 m, D8 quadrotor target (mesh_flat).
- **P_T (matched middle):** P_S task rules and D8 target, with P_K vehicle and environment.
- **Legacy motion** (F, K1 from F3 on) is S2b-FT's form: `NAVRL_TARGET_DYNAMICS=legacy`, `NAVRL_TARGET_PATTERN=mixed`,
  default behaviour level and the `legacy_cdist` distance backend. 205 bars is outside the known TF32 cdist pattern (not
  divisible by 4). The KROC2027 recorder runs only in the P_K identity cells F0ID and K1ID (as GR-0 and CAM-K1-E ran).
- **Module seeds:** F8 and K1S use S2b-FT's per-cell module seeds (delay 10 × seed + 4, vehicle + 1, gain + 5), so F8,
  K1S and S0 share those draws per seed block.

## Stage 1 (this approval request): minimal session, 205 bars, seeds 4501-4503, 1,024 episodes per cell (128 × 8)

| Cell | Policy | Protocol | Mechanism |
| --- | --- | --- | --- |
| F0-ID | F | P_K, seed 4501, idle host | identity vs GR-0 S00 205/4501 (first 8 episodes per env) |
| K1-ID | K1 | P_K (its native), seed 4501, through the new entry path | identity vs CAM-K1-E K1s1 205/4501 (first 8 episodes per env) |
| S0-ID | S2b-FT | P_S, seed 4501 | identity vs DX1 V312 205/4501 |
| F3 | F | P_K vehicle/env + V312 start + legacy motion + CAM-S2 capture/contact | new entry + new rule module |
| F4 | F | F3 + D8 target = **F at P_T** | env |
| F8 | F | F4 + delay U{1,2} + vehicle/gain randomisation + yaw clip 1.047 rad/s (scale kept 3.0) + FREE 2.5 + altitude 1.5 / mount 0.08 = **F at P_S** (no camsafe) | rule module + navrl_vehicle_rand + navrl_yawclip |
| S3 | S2b-FT | P_S minus altitude/mount, randomisation, delay = **S2b-FT at P_T** | cams2 parameters, env |
| S6 | S2b-FT | S3 minus CAM-S2 capture, E2 target, A3 start = **S2b-FT at P_K** | env |
| K1-T | K1 | F4 settings, K1 perception | new entry |
| K1-S | K1 | F8 settings, K1 perception | new entry |

Identity cells are single seeds; the other seven are 3 seeds each (21 cells + 3 identities + 3 smoke cells). Estimated
GPU time 1.4-1.8 h; disk ≤ 5 MB per cell. Stage 2 (intermediate rungs F1, F2, F5-F7, S1, S2, S4, S5) needs its own
approval after stage 1.

**Smoke (never counted):** F8, K1-S and S6 at 205 / 4501, quota 1 (128 episodes each); S0-ID exercises S2b-FT's native
path. SMOKE_OK needs every cell OK with ≥ 1 finished episode and: in F8 and K1-S, delay draws of both 1 and 2 steps,
≥ 1 capture or target contact, a vehicle-randomisation init line, yaw-clip commands > 0, and every rendered step-0
bearing within ±28.35°; in S6, every rendered step-0 bearing within ±2° (A3 boresight), no cams2 telemetry, and a
CAM-REF recorder line.

**S6 details:** the goal-distance maximum returns to 28 m (S2b-FT's trained value; A3 fixes the 10 m start, as in GR-0 and
CAM-K1-E); turning cams2 off also removes its contact code and `navrl_cams2diag` (S6's crash classes come from the
CAM-REF recorder); the start recorder (`navrl_inview_start`'s) stays on as the A3-start control; with cams2 off the
altitude / mount / delay are the task defaults 1.0 m / 0.10 m / 0, equal to S3's.

## Fixed choices
- **Detection signal for F's capture rule:** the fused camera-or-LiDAR detection (F's own `_visible_now`; K1's is camera
  only). Captures end episodes, so this cannot be rescored later.
- **Episodes counted:** the first 8 per env everywhere (as DX1); F0-ID and K1-ID compare against the GR-0 S00 and
  CAM-K1-E K1s1 ledgers' first 8 per env.
- **Crash split:** bar contact, target contact (0.40 m swept rule), out of bounds (below / above as "other"); reported
  for every cell that has a per-episode recorder. Target contacts get crash-cause code 0 in the task (as
  `navrl_cams2_contactcode`; the task's strata validator accepts codes 0-3 only and requires the per-stratum cause sum to
  equal the crash count, so a fifth code would abort the result export); the rule module's recorder writes
  `crash_class = target_contact_swept` per episode from its own contact mask, and the crash split reads the recorders
  (`camref_episodes.jsonl`, or `cams2diag_episodes.jsonl` where cams2 runs). Bar contact = cause 0 not converted by the
  swept rule; it includes any physical contact with the target body. GR-0 S00 and CAM-K1-E K1s1 have no per-episode
  crash cause (their `result.json` `crash_causes` count the legacy window, not the first 8 per env): their bar /
  target / out-of-bounds rates read NOT_RECORDED; F0-ID's and K1-ID's splits are reported for seed 4501.
- **New code (new files, plus one test allowlist line):** an entry `tools/camref_entry.py` (+ `tools/camref_python`) and
  a rule module `aerial_gym/task/navrl_task/navrl_camref_rules.py` (each rule switchable: capture/contact, delay,
  altitude / mount; the start is `navrl_inview_start`; own target-contact class; detection signal chosen by setting), a
  runner `tools/camref_eval_run.py`, an analyser `tools/camref_eval_analyse.py`, a pinned launcher copy
  `aerial_gym/rl_training/rl_games/eval_navrl_camref_arms.sh`, tests `tests/test_camref.py`. A CPU test shows the new
  rule module equals `navrl_cams2` + `navrl_cams2_contactcode` (capture mask, contact conversion, delay) on synthetic
  tensors. CAM-K1 and S2b-FT run through `tools/realveh_entry.py` unchanged; `camref_entry` installs
  `navrl_vehicle_rand` for Policy F only (realveh_entry refuses the LiDAR), so the isolation test's allowlist names it:
  `tests/test_realveh.py` `test_only_the_entry_imports_the_new_modules` reads `["tools/camref_entry.py",
  "tools/realveh_entry.py"]` (amended in the package commit; the depth-model and ablation importer checks unchanged).
- **Guards:** fail-closed preflight (approval naming this unit and HEAD, clean tree, branch camonly/s2b, launcher
  pin, host-fault count / boot id / online CPUs unchanged per stage, idle host); identity first, stop on
  IDENTITY_FAIL. **Wall cap 2.5 h** shared by smoke (sub-cap 0.5 h), identity (0.75 h) and eval: each stage gets
  min(sub-cap, 2.5 h − the earlier stages' wall time). A cell whose projected wall time (the longest OK cell of the same
  policy and quota in the stage, else 1.5 × the longest in an earlier stage; unknown = run, its timeout bounded by the
  remaining cap) does not fit the remaining cap is skipped, not started (BUDGET_SKIPPED); a stage with a skipped cell
  reads SMOKE_INCOMPLETE / IDENTITY_INCOMPLETE / INCOMPLETE, and the analysis reads complete seed blocks only.
- **Idle host:** < 15 % of the online CPUs busy over a 2 s `/proc/stat` sample at each GPU stage's preflight; the 1-min
  load average is recorded per cell.
- **Approval file:** `~/motar_tmp/camref_20261126/execution_approval.json` with `approved_by`, `approved_at`, `scope`
  (= `tools/camref_eval_run.py scope`), `scope_sha256` and `commit` (HEAD). This file:
  `docs/plans/prereg_camonly_ref_matched_protocol_2026-11-26.md` (the runner's `PREREG`).
- **Branch side effect (accepted):** committing new files on camonly/s2b closes CAM-DX1's runner allowlist (DX1 is
  complete; a rerun would need the commit 09937a8 re-checked out). No new branch (user, 2026-11-26).

## Identity criteria
- S0-ID: per-episode outcomes identical to DX1 V312 205/4501 cell (same seed, same 1,024 episodes); else
  IDENTITY_FAIL.
- F0-ID: per-episode outcomes identical to GR-0 S00 205/4501's first 8 episodes per env; else IDENTITY_FAIL. (That GR-0
  cell overlapped CPU jobs, but it is byte-identical to its idle twins C1 / C1_r2 `full`, EXPERIMENTS T-LOAD, so a
  difference is not load.)
- K1-ID: per-episode outcomes identical to CAM-K1-E K1s1 205/4501's first 8 episodes per env; else IDENTITY_FAIL.
- Eval needs IDENTITY_PASS (all three cells).

## Readings (descriptive; fixed now)
For each cell: capture, crash (all / bar / target / OOB), timeout, with seed-block intervals (the t-interval over the
seed blocks, t 0.975 with 2 df = 4.303, for per-cell rates and for paired block differences) and an episode bootstrap
(10,000 resamples, seed 4611). Labels use the bootstrap interval (as DX1).
- **R1 (protocol share of S2b-FT):** S0 − S6 crash and capture (S0 = DX1 V312, whose seed-4501 cell S0-ID reproduces),
  split into P_S → P_T (S0 − S3) and P_T → P_K (S3 − S6). Labels: MATERIAL (|difference| ≥ 5 pp and the
  episode-bootstrap 95 % interval excludes 0), SMALL (interval excludes 0, |difference| < 5 pp), NO_DETECTED_CHANGE
  (otherwise); the sign is reported; labels on capture and all crashes; the crash classes are reported.
- **R2 (policy gap at matched points):** F − S2b-FT and K1 − S2b-FT at P_K (F0 = GR-0 S00 205 / 4501-4503, K1 = CAM-K1-E
  K1s1 205 / 4501-4503, S6), P_T (F4, K1-T, S3) and P_S (F8, K1-S, S0). Reported.
- **R3 (is 20 % reachable with 360° ranging under CAM-S2 task rules):** F8 and F4 one-sided 95 % Wilson upper bounds of
  (i) all crashes and (ii) bar-contact crashes, against 20 %, the three seeds pooled as n = 3,072 independent episodes
  (seed clustering ignored). Labels per metric: BELOW_20 (UCB ≤ 20 %), ABOVE_20 (lower bound > 20 %), UNRESOLVED. The
  same bounds are reported for S0 and S3. F was never trained for the slow-approach rule: F8 is an upper bound on what a
  360° policy trained under P_S would crash (INFERRED), not its value; bar-contact crash is co-primary for this reason.
- **Use of the result (fixed now):**
  - R3(ii) BELOW_20 at F8 and S2b-FT at P_S (S0 = DX1 V312) bar-contact crash ABOVE_20 by the same Wilson rule → the gap is
    sensing/representation; the next design gives the camera policy a 360° range representation (map / memory,
    NavRL-style) and/or a privileged teacher.
  - R3(ii) ABOVE_20 at F8 → 205 bars under CAM-S2 rules is not reached even with 360° ranging zero-shot; the next design
    revisits the task rules, the obstacle field (bar width vs real forests) or trains a 360° reference under P_S first.
  - R1 MATERIAL → the protocol factors that carry it are reported for the design; no flight claim.

## Controls (on the analysis code, before it reads the eval)
Synthetic ledgers: identical cells → NO_DETECTED_CHANGE; +8 pp → MATERIAL; 18 % crash with n = 3,072 → BELOW_20;
23 % → ABOVE_20. Recorded DX1 R312 − V312 as a pair → crash MATERIAL (matches DX1's START reading).

## Limits
Zero-shot evaluation of F and K1 under rules they never trained on; the ladder is cumulative (interactions assigned to
later steps); residual differences that cannot be bridged (S2b-FT's yaw scale and camsafe, each policy's perception);
one density; no flight claim. Records under `~/motar_tmp/camref_20261126/`.
- Review r1 P2-1 (recorded, not fixed): eval cells carry no runtime check that the CAM-REF rules took effect beyond smoke (F8/K1S) and the environment checks; the analysis reports each cell's recorder counters and capture-row distance/closing maxima descriptively.
- Review r1 P2-2: S6 keeps the D8 target while F0 and K1 at P_K use the analytic target, so R2 at P_K compares across target appearance.
