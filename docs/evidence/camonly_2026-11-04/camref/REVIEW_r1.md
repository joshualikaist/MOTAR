# CAM-REF stage 1 — independent pre-GPU review r1 (2026-11-26)

- **Reviewer:** Claude Opus 5.5 (Claude Code subagent, fresh context, read-only on the repository).
- **Tree:** `/home/fair/workspaces/aerial_gym_ws/src/MOTAR-camonly-s2b`, branch `camonly/s2b`, HEAD `e410e5e` (unchanged);
  working tree: `tests/test_realveh.py` modified, the 7 package files and the prereg untracked. Nothing edited, staged,
  stashed or committed by the reviewer. No GPU (CUDA_VISIBLE_DEVICES=""), no `*.pth` loaded, no `check` stage run
  (it queries nvidia-smi). CPU jobs: `nice -n 10 taskset -c 6,7,12,13`, PYTHONNOUSERSITE=1, aerialgym python,
  PYTHONPATH = worktree.
- **Files reviewed (SHA-256):** prereg `5dfa6c01bbaa…` (150 lines), `navrl_camref_rules.py` `9c36d868cdef…`,
  `camref_entry.py` `9d11d0b995cc…`, `camref_python` `d3083a087909…` (executable), launcher
  `eval_navrl_camref_arms.sh` `51f949d08ec7…` (= the runner's `LAUNCHER_SHA`), `camref_eval_run.py` `5c3254fa9c03…`,
  `camref_eval_analyse.py` `846e303e8022…`, `tests/test_camref.py` `8c968325dec7…`, `git diff tests/test_realveh.py`
  (+2/−1). These match PACKAGE_REPORT round 2.
- **Verdict: GO_WITH_FIXES** — no P0; two P1 (commit hygiene of the prereg, analysis fallback that the run commit
  freezes). Fix them, then the user's GPU approval is the remaining gate.

## What I ran (CPU)

| Check | Result |
| --- | --- |
| `python -m unittest test_camref test_realveh` (from `tests/`, OMP/MKL=1) | **86 tests OK** (37 + 49), 0 skipped |
| `camref_eval_analyse.py controls` | `controls_ok: true`; identical → NO_DETECTED_CHANGE ×2; ±8 pp → MATERIAL ×2; 18 % / n 3,072 → BELOW_20; 23 % → ABOVE_20; recorded DX1 R312 − V312 crash +13.12 pp [10.74, 15.49] MATERIAL (pins OK) |
| `camref_eval_run.py scope` | scope_sha256 `fb5eb257971127701b4fb8b28970bbcf400b62878d3206ee322f6a747a79ff90` (= package report) |
| Identity cell env vs recorded receipt `env_snapshot_navrl` (runner's own `cell_navrl_env` + `launcher_values`, key by key; script in my scratch) | see §1b — only label / receipt-path / quota keys differ |
| Mismatch allowlist on the three reference `cell.log`s | F0ID 6 lines, K1ID 6, S0ID 3 — 0 problems |
| Full PUBLIC_CPU suite | **NOT_RUN** by me (package report: 4,551 run, failure list identical to baseline) |

## 1. Protocol per cell

### 1a. Ladder construction (code vs prereg table, lines 44-55)
Verified by dumping every cell's environment and diffing rungs (`F3→F4`, `F4→F8`, `F8 vs K1S`, `F4 vs S3`, `F8 vs S0ID`,
`F0ID vs S6`):
- **F3** = F_BASE_ENV − E2 keys + legacy mixed (`DYNAMICS=legacy`, `PATTERN=mixed`, BEHAVIOR_LEVEL / DISTANCE_BACKEND
  removed → defaults `historical` / `legacy_cdist`, navrl_task_config.py:605,614) + visible start (INVIEW 28.35°, seed =
  cell seed, goal max 12) + CAM-S2 capture (1.0 m, ≤ 1.0 m/s, contact 0.40, detect `visible_now`) + recorders
  (camref_eval_run.py:186-191, 202-229). Matches.
- **F4** = F3 + `NAVRL_DYNAMIC_MESH_TREATMENT=mesh_flat` (only difference). Matches.
- **F8** = F4 + camref delay U{1,2} (seed 10·s+4 = 45014 at 4501, = DX1), VEH_RAND (raise, T/W 3.5, ±15 %, ±5 %, τ
  0.02-0.04, seed +1 = 45011), GAIN_RAND 0.2 (seed +5 = 45015), yaw clip 1.047 with `NAVRL_YAW_RATE_MAX` kept 3.0, FREE 2.5,
  geometry 1.5 m / 0.08 m (:194-199). Matches P_S; seeds equal DX1 V312's receipt values.
- **K1T / K1S** = F4 / F8 + `no_lidar` + 66.0/54.0/0.6/0.03 (only differences vs F4/F8). Matches.
- **S3** = DX1 V312 − veh/gain rand, cams2 delay 0/0 (`ACTION_DELAY_STEPS_MAX=0`, required equal by navrl_cams2.py:65),
  cams2 altitude 1.0 / mount 0.10 (:232-239). Matches P_T for S2b-FT.
- **S6** = S3 − cams2 / contact code / cams2diag, + E2 keys, + A3 (10 m), − visible start, goal max 28 (trained), +
  CAM-REF recorder, inview diag kept (:248-258). With cams2 off: success_radius 0.5, altitude 1.0, mount 0.10 are the task
  defaults (navrl_task_config.py:208,492,727). D8 target kept (see P2-2).
- **F4 vs S3** environment differences are all policy stack / perception (resolution, FOV, depth model, P9, camsafe,
  ablation, yaw scale, FREE, ACTION_* and training-only keys) plus diagnostics (`BAR_PROBE`, `CRASH_DIAG`,
  `SPEED_GOVERNOR_DIAG`). `DETECTOR_MIN_PIXELS=2` and `PERCEPTION_PERTURB=0` equal the defaults F runs with
  (navrl_task_config.py:273,277); `EVAL_CV_INITIAL_HEADING=random` equals the default (navrl_task.py:2040). No hidden
  environment difference found.
- `RUNG_PROTOCOL` (:405-424) is an independent restatement and `protocol_problems` derives the fields from each cell env;
  every planned cell passes (test `test_every_planned_cell_matches_its_protocol_and_env_problems_bite`).

### 1b. Identity cells against the recorded receipts (not prose)
Runner-built env + launcher keys vs each reference's `receipt.json` `env_snapshot_navrl`:
- **F0ID vs GR-0 S00 205/4501:** differs only in `NAVRL_CAMREF_POLICY/RECEIPT/RECORD/RECORD_JSONL` (new, passive),
  `NAVRL_KROC2027_RUN_ID` (label only, kroc_eval_adapter.py:785), `NAVRL_EVAL_RUN_NONCE`, and
  **`NAVRL_EPISODE_QUOTA_PER_ENV` 8 vs 16**. Quota does not change the simulation: every env keeps stepping until all
  reach quota (navrl_episode_ledger.py:11-14) and the run ends only at `ledger.complete` (navrl_task.py:5155-5162), so the
  first 8 episodes per env are comparable. The comparison (`identity_compare`, :888-910) uses `env_episode_index < 8` on both
  sides and requires 128 × 8 complete keys.
- **K1ID vs CAM-K1-E K1s1 205/4501:** as F0ID plus `NAVRL_CAMTRAIN_RECEIPT` (ref) vs `NAVRL_REALVEH_RECEIPT` (new).
  realveh_entry with veh-rand / depth / camsafe / cams2 off installs exactly what camtrain_entry did: the `no_lidar`
  ablation + `camtrain.apply_overrides` (realveh_entry.py:286-290 vs camtrain_entry.py:85-87). Quota 8 vs 16 as above.
- **S0ID vs CAM-DX1 V312 205/4501:** differs only in `NAVRL_CAMREF_POLICY/RECEIPT` and the nonce; quota 8 both.
- Code between the reference commits and HEAD: `navrl_task.py` differs from GR-0's `c56be07` only by the removal of
  `_gain_rand_condition` (recording, not dynamics); 971ae67 → HEAD and 335a1fa → HEAD change only opt-in module files.
- The camref recorder is ON in F0ID / K1ID (OFF in GR-0 / K1-E); it draws no randomness and writes no task buffer, so
  the identity cell is also the recorder-on reproduction control. Any difference stops the unit (IDENTITY_FAIL,
  :913-918, :1280-1281) — appropriate.

### 1c. Per-point checks
- **Capture/contact, detection:** equal to cams2 + contact code (see §2). F's `visible_now` = camera | LiDAR
  (navrl_perception.py:2286, 2394; task copies it at navrl_task.py:6482); K1's LiDAR bins are all masked to max range by
  the ablation (navrl_sensor_ablation.py:92-103), so K1's flag is camera-only in effect, as the prereg states.
- **Yaw clip vs scale:** F8/K1S clip the command (navrl_yawclip.py:90-96) with scale 3.0 kept; S2b-FT keeps 1.0472
  scale; entry refuses a yaw clip for S2B (camref_entry.py:184-185).
- **Governor:** FREE 2.5 only in F8/K1S (and S2b-FT's own); riskcap everywhere; `condition_problems` checks FREE per arm
  (:635).
- **Altitude/mount:** F8/K1S via camref geometry; S0 via cams2; S3/S6 1.0/0.10. (Runtime check gap: P2-1.)
- **Vehicle/gain:** F8 veh-rand installed by camref_entry (own generator; wraps the robot class, so install order vs the
  rules is irrelevant, navrl_vehicle_rand.py:300-330); K1S/S0 by realveh. Gain rand via controller config (own
  generator, base_lee_controller.py:115-133). (Runtime signal gap: P3-3.)

## 2. Rule module equivalence, inertness, wrapper order
- `navrl_camref_rules.install_rules_task` (:198-304) is line-for-line navrl_cams2.install_task (:103-174) + navrl_cams2_
  contactcode (:67-84): same contact mask (task's swept test at 0.40), cause code 0, diag tallies, penalty, flag
  conversion, `alive = terminations <= 0`, capture mask, bonus, termination; delay history / roll / generator seeding /
  initial draw / per-reset draw identical (the extra `_count(draw)` only reads). Test `test_step_by_step_equal` compares
  80 steps × 48 envs on commands, rewards, terminations, flags, cause codes, diag, contact mask and delay draws with
  non-zero contacts, captures, resets and both delays drawn. PASS.
- **Inert when off:** `settings_from_env` returns None with every switch off (:133-134); the entry imports the module
  only if a rule or the recorder is on (camref_entry.py:238); child-process test shows the stub class unchanged.
- **Silent non-application:** unknown `NAVRL_CAMREF_*` keys, parameters set while a switch is off, malformed /
  out-of-range values, contact ≥ capture radius, unknown detection source, existing recorder path are all refused
  (:92-136); the runner compares every cell env key-by-key with its construction (:495-527) and the receipts / launcher
  snapshots after the run (:561-570, :746-820); smoke checks rule telemetry for F8/K1S. Remaining gap: P2-1.
- **Wrapper order:** F: veh-rand (robot) → rules → recorder → yawclip (command only) → inview start / diag (outermost).
  K1: rules → recorder → realveh (veh-rand robot, ablation = perception + governor, camtrain config, yawclip, inview).
  S6: recorder → realveh (ablation, camsafe, inview diag). The recorder is not outermost in K1 / S6, but none of the
  later wrappers changes `captured_now / crashed_now / terminations / _crash_cause_code` (camsafe's reward wrapper changes
  rewards only, navrl_camsafe.py:178-189; ablation wraps perception / governor; yawclip the command). Recorder rows are
  also checked against the ledger per (env, episode) (:676-691). OK (P3-1 note).
- Ledger ordering: the ledger records after `compute_state_reward_and_terminations` in `step()` (navrl_task.py:5063,
  5148-5162), so it sees the rule-modified flags, as for cams2 in DX1.

## 3. Observation contract
- No ground-truth target quantity reaches the actor: the capture rule reads `target_position`, `target_vel_w` for
  termination / reward only; the recorder and inview diag are passive; the inview start sets the initial yaw (a reset
  condition, DX1 precedent).
- F's actor input is unchanged: `NAVRL_YAW_RATE_MAX` 3.0 and `NAVRL_MAX_VELOCITY` 2.5 kept (the yaw clip acts on the
  command after the transform); `prev_action` is the raw action (governed xy as always), not the delayed one — the same
  semantics cams2 has for S2b-FT (navrl_task.py:5049-5055). Altitude 1.5 keeps the z feature at 1/3 (normalised by
  flight_altitude per FEASIBILITY, CODE). PASS.

## 4. Runner guards
- Approval: file keys + scope string + `scope_sha256` + `commit` ∈ {HEAD, HEAD[:7]} (camk1e_run.py:78-89; runner
  :1130-1140); `--approved-commit` must prefix HEAD and be ≥ 7 chars; clean tree; branch `camonly/s2b`; committed files;
  launcher SHA pin; play-script pin; code since 335a1fa inside the allowlist; checkpoints and reference cells at pins;
  base provenance (F_BASE_ENV = GR-0 S00 ×3 seeds, + K1 perception = K1-E ×3, DX1 construction = V312 ×3); one RTX 3070,
  no other GPU compute; idle host (< 15 % busy over 2 s); import origins; free disk; new output folder. PASS.
- Host invariance after every cell and at stage end (fault count, boot id, online CPUs) → VOID_HOST_FAULT. PASS.
- Identity before eval: eval needs exactly one `identity_report.json` = IDENTITY_PASS on the same commit in the same
  parent, plus SMOKE_OK; identity needs SMOKE_OK; runner env allowlist must equal the identity stage's. PASS.
- **Output folders:** each stage needs its own new `--out` (existing folder → PREFLIGHT_FAIL, :1165-1166); stages link
  through `out.parent/*/…_report.json` (exactly one). The earlier failure mode (same `--out` for every stage) is refused
  before any GPU use. PASS.
- Wall cap: each stage gets `min(sub-cap, 2.5 h − earlier stages' wall_s)`; skip-before-start by projection; cell timeout
  bounded by the remaining cap (killpg SIGTERM → SIGKILL). Matches prereg lines 97-102.
- Quota: 8 × 128, ledger status / partition / per-env counts checked (kroc2027_eval.py:261-279); analysis re-checks 8 per
  env. PASS.
- OOM: no fallback; an OOM is a FAIL that stops the stage (P3-4).
- Eval stops at the first FAIL (not in prereg, conservative) → INCOMPLETE; interacts with P1-2.

## 5. Analysis
- R1 labels (MATERIAL / SMALL / NO_DETECTED_CHANGE on the bootstrap interval, |Δ| ≥ 5 pp) :145-149; R2 unlabelled; R3
  one-sided Wilson z = 1.6449, BELOW_20 iff upper ≤ 0.20, ABOVE_20 iff lower > 0.20, else UNRESOLVED (:186-201) — as
  written in the prereg. Bootstrap 10,000, `default_rng(4611)`, multinomial per cell × block in fixed order. t(0.975, 2)
  = 4.3027 seed-block interval. PASS.
- NOT_RECORDED: GR-0 / K1-E references have no class rows → bar / target / OOB rates NOT_RECORDED in cells and effects
  (:108-111, :173-175, :226-228). PASS.
- Controls are non-vacuous (paired: identical → NO_DETECTED_CHANGE, +8 pp → MATERIAL; 18 % → BELOW_20, 23 % → ABOVE_20;
  recorded R312 − V312 → MATERIAL with SHA pins); unit tests add 20 % → UNRESOLVED. No SMALL control (P3-6).
- S0 bar-contact (cams2diag `contact` class, first 8/env): 214/1,024, 235/1,024, 237/1,024 at 4501-4503 (MEASURED from
  the pinned ledgers) — the use-of-result rule's S0 leg is well defined.
- **Fallback issue: P1-2.**

## 6. Tests
- 86/86 OK. Negative checks are paired with positive controls and non-zero counts (e.g. `assertGreater(hits, 0)`,
  contacts / captures / resets > 0, both delays drawn, camera-only source a strict non-empty subset of fused, refusals
  followed by a successful install, all planned cells accepted). Reference-file tests skip (not pass) when files are
  absent; here none skipped.
- `tests/test_realveh.py`: the exact-list assertion stays (positive control included); the allowlist names
  `tools/camref_entry.py` beside `tools/realveh_entry.py`; the depth-model and ablation importer checks are unchanged. The
  guard's intent (every importer of the opt-in module is an explicit, reviewed entry) is kept. P3-7 on an evidence
  manifest that binds the old bytes.

## 7. Prereg vs code; interpretability; GPU time
- Prereg ↔ code agree on cells, seeds, quota, cap / sub-caps, skip rule, approval path / keys, identity criteria,
  detection choice, crash split, readings, controls (checked line by line). Mismatches: P1-1 (status line), P1-2
  (complete blocks), P2-2 (S6 target vs P_K definition), P3-2 (bearing tolerance), P3-5 (draw sharing wording).
- **GPU time (from recorded cells):** GR-0 S00 205 quota 16: 170 / 175 / 159 s; CAM-K1-E K1s1: 181 / 165 / 164 s;
  DX1 V312 quota 8: 331 / 303 / 276 s; R312: 372-419 s; ≈ 7-9 control steps/s at 128 envs. Estimate: smoke ≈ 6-8 min,
  identity ≈ 9-10 min (F0ID, K1ID ≈ 2 min each, S0ID ≈ 5.5 min), eval 21 cells ≈ 60-90 min if F/K1 cells take
  100-250 s (S3 ≈ 5 min, S6 ≈ 5-7 min) → **≈ 1.3-1.8 h total**, inside the 2.5 h cap. INFERRED risk: F / K1 have never run
  under the slow-capture rule; if their episodes run to time-outs, a cell can reach 8 × 600 steps ≈ 10-11 min, and the
  cap would then skip late cells of seed 4503 (the analysis keeps complete blocks — after P1-2).

## Findings

### P0 (blocks the GPU run)
None.

### P1 (must fix before the run)
- **P1-1 Commit hygiene of the prereg (path-contract file, frozen once committed).** (a) `docs/EXPERIMENTS.md` has no
  CAM-REF entry; AGENTS "Connect every new record" requires it in the same commit and `tests/test_experiment_index.py`
  will fail for the new `docs/plans/prereg_*` file. (b) The prereg's line 3 says "**Status:** DRAFT (not committed)" and
  "Every GPU cell needs the user's separate approval after this file is committed and reviewed"; committed as is, the
  frozen record says DRAFT forever. Rewrite the status line (and add the review reference) before the commit; add the
  WORKLOG entry in the same commit.
- **P1-2 Analysis reads incomplete blocks when none is complete.** `camref_eval_analyse.py:400`
  `readings(cells, blocks=binding["complete_blocks"] or list(SEEDS))`: with zero complete seed blocks (likely after any
  eval FAIL in seed 4501, since the runner stops at the first FAIL, camref_eval_run.py:1292-1294) it falls back to all
  seeds, and `effect()` then labels R1/R3 from whatever per-comparison blocks exist, contrary to prereg line 102 ("the
  analysis reads complete seed blocks only"). Because `load_eval` requires the analysis script to equal the eval commit's
  copy (:363), this cannot be fixed after the run without an amendment. Fix: no complete block → R1/R2/R3 = NOT_RUN (no
  labels); add a test. (Optionally restrict `cell_summary` to the complete blocks too, P3.)

### P2 (fix or record)
- **P2-1 No per-cell runtime proof that the CAM-REF geometry and capture rules acted in eval cells.** For cams2 cells
  the runner checks `config_after` (alt / mount / success radius, :785-790); for F8/K1S the camref receipt's
  `camref_rules.config_after` is written but never checked, and F3/F4/K1T/K1S eval cells are not checked for rule
  activity (only smoke F8/K1S). Cheap non-vacuous checks: camref receipt `config_after` = (0.40, 1.5, 0.08) for F8/K1S and
  (0.40, 1.0, 0.10) for F3/F4/K1T; last `[camref]` line `rules.steps > 0`; every capture row has `capture_dist_m ≤ 1.0`
  and `capture_closing_mps ≤ 1.0` (+tol). If not added, record the limit.
- **P2-2 S6 is "P_K with the D8 target".** P_K is defined with an analytic target (prereg line 29-30) but S6 keeps
  mesh_flat (table line 53; code RUNG_PROTOCOL S6 `target="D8"`, :424). R2 at P_K therefore compares F0 / K1 (analytic)
  with S6 (D8), and R1's P_T → P_K step excludes target appearance. Defensible (S2b-FT's detector was trained on D8;
  FEASIBILITY row 15), but say it in the prereg's Limits / P_K definition before commit.

### P3 (notes)
- **P3-1** The recorder is not the outermost wrapper in K1 and S6 cells; safe because no later wrapper changes the
  outcome flags (verified), but nothing in code enforces it; the ledger alignment check would catch a violation.
- **P3-2** Smoke / per-cell bearing limit is 28.35° + 0.01° (`D.BEARING_TOL_DEG`, float round trip) vs the prereg's
  "within ±28.35°"; record it.
- **P3-3** Gain randomisation in F8 / K1S has no runtime positive signal: HEAD's `navrl_task.py` no longer records
  `cfg_gain_rand*` (removed vs c56be07) and no telemetry line exists; application is INFERRED from env + controller code
  (same as DX1's S0).
- **P3-4** No OOM fallback (DX1 had one); an OOM is a FAIL that stops the stage. The recorded resolutions ran at 128 envs.
  No per-process memory cap (wall timeout with killpg only).
- **P3-5** Prereg "F8, K1S and S0 share those draws per seed block": only the generator seeds (and the first draw) are
  shared; later draws depend on policy-dependent reset timing. Reword.
- **P3-6** No SMALL-label analysis control.
- **P3-7** `docs/evidence/sitl_2026-11-05/camera_runtime/SOURCE_SHA256_v2.json` binds `tests/test_realveh.py` at
  `9e8b1732…` (the HEAD bytes); after the edit the working file no longer matches. No code reads that manifest; the
  bound bytes stay in git history. Mention it in the commit's WORKLOG entry.
- **P3-8** Smoke controls read the last running-total `[camref]` line (calls 1, 32, 320, then every 320); a smoke cell
  shorter than 320 steps would be judged on step-32 totals. Low risk (DX1 V312 first-episode runs reached ≥ 472 steps).
- **P3-9** Docstrings of `navrl_camref_rules.py`, `camref_entry.py`, the launcher, the runner and the tests cite the
  draft path `~/motar_tmp/camref_20261126/draft/prereg_…`; update to the committed path if convenient (the launcher pin
  must then be refreshed).
- **P3-10** Legacy motion uses `legacy_cdist` under TF32 at 205 bars (not in the known divisible-by-4 pattern, but not a
  whitelist); shared with S2b-FT's training/eval, so comparisons are on the same footing.
- **P3-11** The full PUBLIC_CPU suite was not re-run by this reviewer.

## Verdict
**GO_WITH_FIXES.** The ladder, identity construction, rule equivalence, observation contract and runner guards are
correct as audited. Before the commit and the GPU request: fix P1-1 (EXPERIMENTS.md entry, prereg status line,
WORKLOG) and P1-2 (no-complete-block fallback), and either fix or record P2-1 and P2-2. If P1-2 or P2-1 change runner /
analysis code, re-run `tests/test_camref.py`, `tests/test_realveh.py`, the analysis controls and the full suite; a
changed launcher needs its pin refreshed.

Reviewer: Claude Opus 5.5 (fresh-context subagent), 2026-11-26. USAGE_UNKNOWN.
