# CAM-T1 v1: where camera-only CAM-K2 v3 crashes, and its speed-cap and delay envelope (preregistration, 2026-11-12)

**State:** **Reviewed r1 + r2 (both PASS_WITH_FIXES, repaired); ready to commit. GPU run pending the user's
approval.** NOT_RUN. Once committed, this file is never edited; amendments go
in a new dated file. The independent review is recorded under *Review before the commit* at the end.

- Runner: `tools/camt1_run.py` (stages `identity`, `trace`, `control`, `envelope`; `plan` is a dry run).
- Analysis: `tools/camt1_analyse.py` (`stage-a`, `stage-b`, `g3-control`, `controls`).
- Launcher copy: `aerial_gym/rl_training/rl_games/eval_navrl_camt1_arms.sh` (SHA-256 pinned in the runner).
- Tests: `tests/test_camt1.py`, run as `PYTHONNOUSERSITE=1 CUDA_VISIBLE_DEVICES= python -B -m unittest discover -s tests
  -p test_camt1.py` (`tests/` is not a package, so `-m unittest tests.test_camt1` does not import it; review r2 N5).
- Chain: `~/motar_tmp/camt1_20261112/run_camt1.sh`. Unit folder: `~/motar_tmp/camt1_20261112/`. Raw output:
  `~/motar_tmp/camt1_20261112/raw/` (outside the repository).
- Branch `camonly/t1` (worktree `src/MOTAR-camonly-t1`), base `eefb832`.

## Why

- **CAM-K2 v3** (both seeds TRAINED at `326e307`) is the camera-only policy bound into the ROS2 runtime. CAM-K2-E3
  (`aaca40e`, results review r1 PASS) measured its crash rate in the V3 arm (no_lidar, 66° × 54°, CAM-K2 v2's
  treatments, the `blob_near` validity rule) on GR-0's S00 grid; it did not measure **where** the crashes are.
- **The 2026-11-12 CPU re-analysis** (exploratory, not reviewed; `~/motar_tmp/policy_filter_review_20261112/`)
  found that the camera-only policies lost F's diagonal flight, that crash rises with density and target speed,
  and listed the contact position relative to the camera's FOV as NOT_MEASURED.
- **G3** (`docs/evidence/camonly_2026-11-04/c2trace/`) measured contact bearings for zero-shot camera-only Policy F
  at 87°, from the vehicle centre. CAM-K2 v3 is a different policy with a 66° camera.
- **The user** (2026-11-12): run the trace evaluation to locate where the trained camera-only policy crashes, then
  the speed-cap and delay evaluation; "신경 써서 집중해서 한번 오차를 줄이고 충돌 위치를 판단해줘" (be careful, reduce
  error, determine the crash location). This is approval in principle for these two GPU evaluations only; it must be
  re-confirmed before the first launch, and the approval file must name SCOPE and HEAD. Retraining is not approved.

## Questions

1. **Stage A (descriptive, no hypothesis):** for CAM-K2 v3's crashes, was the contacted bar inside the camera's
   field of view at contact, earlier in the last second, or never in that second? On which side, at what speed,
   how soon after the start, and with the target in view or not?
2. **Stage B (gated):** does a lower free-speed cap (2.5 or 2.0 m/s instead of 3.54) or a one-step (100 ms) action
   delay change the crash rate, against the native cells of CAM-K2-E3, by at least the 2 pp floor?

## Design

### Policy and condition (all cells)

- CAM-K2 v3 seeds 1 and 2 at their pins (`888e8056…`, `203deea4…`), from their TRAINED train reports and sidecar
  manifests (`tools/camk2v3e_run.checkpoints`).
- Exactly CAM-K2-E3's V3 arm: `no_lidar`, the 66° × 54° camera through `tools/realveh_python`, CAM-K2 v2's
  treatments ON with the per-cell module seeds (10 × seed + 1, + 2), the validity rule `blob_near` ON, GR-0's S00
  grid condition (E2 target, A3 placement, quota 16 per env × 128 envs, recorder, `direct_l2_v1`).
- **The launcher copy** `eval_navrl_camt1_arms.sh` is CAM-K2-E's launcher with one change, marked `[camt1]`: the
  governor's free-speed cap `NAVRL_SPEED_GOVERNOR_FREE_MPS` comes from `CAMT1_GOVERNOR_FREE_MPS`, which is required
  and must be exactly `3.53553390593` (native), `2.5` or `2.0`; anything else exits 2.
  - `NAVRL_MAX_VELOCITY` (2.5) is never changed: it is the action scale and an observation normaliser.
  - The governor is `riskcap` (`speed_governor.py`): the per-axis action scale is 2.5 m/s, so the largest horizontal
    command is 3.54 m/s. The cap is FIXED (2.0 m/s) at a corridor clearance ≤ 3 m and rises linearly to FREE at
    ≥ 5 m; commands outside the camera FOV see clearance 0 and are held at FIXED. So **cap25** is FREE 2.5 with
    FIXED 2.0, and **cap20** is FREE = FIXED = 2.0, a constant 2.0 m/s cap.
  - In CAM-K2-E3's native V3 cells at 115 and 205 bars, 96–98 % of episodes command more than 2.5 m/s at some step
    (maximum 2.95 m/s), so both caps bind.
- **The delay arm d1** uses G7's opt-in hook (`tools/actionhold_hook.py`, loaded by
  `tools/actionhold_site/sitecustomize.py` when `PYTHONPATH` = tree : `tools/actionhold_site` and
  `MOTAR_ACTION_HOLD_K=1`, `MOTAR_ACTION_DELAY_D=1`): the action computed one step earlier is executed, the zero
  action on an episode's first step. Downstream (governor, controller, recorders) is unchanged.

### Stages, cells and order

| Stage | Cells | Arms | Densities × seeds | Order | Wall cap |
| --- | --- | --- | --- | --- | --- |
| `identity` | 1 | native_identity (V3s1, native cap, no hook, no trace) | 205 × 4501 | first | 0.25 h |
| A `trace` | 24 | V3s1, V3s2 (native cap, `NAVRL_KROC2027_OCCLUSION_TRACE=1`) | 70/115/160/205 × 4501–4503 | density, seed, arm | 2.0 h |
| `control` | 3 | k1d0_identity, d1_smoke, cap20_smoke (all V3s1) | 205 × 4501 | as listed | 0.5 h |
| B `envelope` | 36 | cap25s1, cap25s2, cap20s1, cap20s2, d1s1, d1s2 | 115/205 × 4501–4503 | density, seed, arm | 2.5 h |

- **Native reference** = CAM-K2-E3's `eval_01/V3s1` and `V3s2` cells (`~/motar_tmp/camk2v3e_20261110/raw`), bound
  by `docs/evidence/camonly_2026-11-04/camk2e/SHA256_MANIFEST_v3e.json`; they are not re-run. Every reference file
  read (both ledgers, result, receipt) must match the manifest, and E3's eval report must be COMPLETE at `aaca40e`.
- **Stage A noninterference gate, per cell:** `episodes.jsonl` and `kroc2027_episodes.jsonl` byte-identical to the
  E3 cell of the same arm and block, and `result.json` outcome counts equal. Reading **NONINTERFERENCE_PASS** (all
  24) or **TRACE_INTERFERES** (the traces then describe these runs only, not E3's). Each trace must read back
  through `motar_eval/occlusion_trace.read_trace` (both SHA-256 values), closed complete, with the 66° × 54° camera
  at (0.10, 0, 0.03) m.
- **Identity rule** (review r1 F3, as G7): a cell equals the E3 cell when both ledgers are byte-identical, the
  outcome counts are equal **and** `result.json` is equal after removing the per-run nonce and the absolute
  checkpoint path (`gr_run.identity_compare`'s `result_equal`).
- **Identity stage, first** (review r1 F4): `native_identity` (this launcher at the native cap, no hook, no trace)
  must equal E3's V3s1 205/4501 cell under the identity rule. Reading **IDENTITY_PASS**, else **NOT_REPRODUCED**:
  the chain stops after one cell (about 3 min), and no trace cell runs, so a commit, launcher, driver or host
  difference is never labelled TRACE_INTERFERES. It shows that the launcher copy and this commit reproduce E3.
- **Controls (after the trace stage):**
  - `k1d0_identity`: the hook loaded with K = 1, D = 0 must equal the same E3 cell under the identity rule.
  - `d1_smoke` (D = 1) and `cap20_smoke` (cap 2.0) must be valid cells whose `episodes.jsonl` differs from it (the
    arms act).
  - Reading **CONTROL_PASS** only if all three hold; otherwise CONTROL_FAIL and Stage B does not start.
  - Determinism (an analysis binding): Stage B's `d1s1` and `cap20s1` at 205/4501 must equal `d1_smoke` and
    `cap20_smoke` byte for byte.
- **Order of stages:** identity, then trace (needs one IDENTITY_PASS on this commit), then control (needs one
  finished trace report on this commit), then envelope (needs one CONTROL_PASS on this commit, the same allowlisted
  environment and E3 folder, and the projection gate).
- **If Stage A reads TRACE_INTERFERES while the identity passed:** the trace changed the run; the traces describe
  these runs only, and Stage B proceeds, because its native comparison rests on `native_identity` (V3s1).
  V3s2's native reproduction is then not tested directly (a disclosed limit).

### Cell checks (a FAIL stops the stage)

`gr_run.check_cell` (host fault count, boot id and online CPUs unchanged; receipt on this commit with 0 dirty
files; ledger complete; recorder clean; import origins inside the tree); `camk2e_run.receipt_problems` with a
treated arm and `camk2v3e_run.validity_problems` with the rule ON (both unchanged); and this unit's
`camt1_problems`, which extends the receipt checks explicitly for the new keys without weakening them:

- the NAVRL snapshot differs from the E3 reference cell's only in the per-run keys (`NAVRL_BULK_EVAL_JSON`,
  `NAVRL_EVAL_RUN_NONCE`, `NAVRL_KROC2027_RUN_ID`, `NAVRL_REALVEH_RECEIPT`) and the arm's own key:
  `NAVRL_KROC2027_OCCLUSION_TRACE = 1` for traced cells (absent otherwise), `NAVRL_SPEED_GOVERNOR_FREE_MPS` = the cap
  for cap arms (the native string otherwise); `NAVRL_SPEED_GOVERNOR_FIXED_MPS` = 2.0, `NAVRL_MAX_VELOCITY` = 2.5 and
  `NAVRL_SPEED_GOVERNOR` = riskcap always;
- `result.json`'s condition equals the reference's except `speed_governor_free_mps` for cap arms, which must equal
  the cap;
- the hook sidecar `actionhold_hook.json` is present exactly for hook arms, with its config, patch applied,
  calls > 0 and, for D = 1, delayed steps > 0;
- trace files exist exactly for traced cells;
- no episode's commanded horizontal speed (`pursuer_commanded_speed_max_mps`) exceeds the arm's free cap + 0.001 m/s.

### Preflight (fail closed) and VOID rules

- An approval file `~/motar_tmp/camt1_20261112/execution_approval.json` naming this SCOPE (the runner's `SCOPE`
  string) and HEAD; clean tree; branch `camonly/t1`; this preregistration, the launcher copy, the runner, the
  analysis, the tests, the hook files, the trace modules and CAM-K2-E3's committed files committed.
- The launcher at its pinned SHA-256; the KROC2027 protocol's launcher and `play_navrl.sh` at their pins (Policy F's
  checkpoint is not required: CAM-T1 never loads F, and this worktree holds no copy of it).
- Exactly one RTX 3070 and no other GPU compute process, at preflight and again before every cell (review r1 F10;
  a busy GPU, or a GPU query that fails or times out after 30 s (review r2 N2), stops the stage, which then reads
  INCOMPLETE). There is no grace period, and only rustdesk is excluded: a transient CUDA client of another program
  between two cells stops the stage (review r2 N4), which is then rerun in a new parent folder.
- **Import origins:** the runner's `PYTHONPATH` must be exactly this worktree; `aerial_gym` and `motar_eval` must
  resolve inside it for plain and hook cells, and `sitecustomize` must resolve to `tools/actionhold_site` for hook
  cells only (the editable install otherwise imports the main checkout).
- No inherited `MOTAR_ACTION_*` or `CAMT1_*` variable.
- Free disk ≥ 1.5 GB (expected use about 0.9 GB: Stage A ≈ 0.6 GB of traces, control and Stage B ≈ 0.3 GB); a cell
  is not started below 1.0 GB.
- A new output folder per stage. **Reruns** (review r2 N7): the runner refuses an existing output folder and a second
  report of a stage in one parent folder, the analyses never overwrite their outputs, and the chain uses fixed names,
  so a rerun after VOID, INCOMPLETE or a stop needs a new parent folder and new analysis output names.
- **VOID_HOST_FAULT:** the kernel's machine-check / segfault / general-protection count, the boot id or the online
  CPU list changed during a stage (checked after every cell and at the end). The stage's results are not evidence.

### GPU budget

- CAM-K2-E3 measured 164 s per V3 cell (167 s at 115/205 bars).
- Expected: identity about 0.05 h, Stage A 1.1–1.4 h (G3's traced 205-bar cell took 173 s against 170–171 s
  untraced, so the overhead is small), control about 0.15 h, Stage B 1.7–2.0 h; about 3.0–3.6 h in all.
- Caps: 0.25 / 2.0 / 0.5 / 2.5 h per stage (5.25 h at most). Before every cell, a rolling rate gate stops the stage when
  elapsed time + mean cell time × remaining cells × 1.10 exceeds the cap. The envelope also refuses to start unless
  36 × E3's mean V3 cell at 115/205 × (control cell mean / E3's V3s1 205/4501 cell) × 1.10 ≤ 2.5 h.
- A stopped stage reads INCOMPLETE (Stage B's analysis then reads EVAL_BINDING_FAIL); the chain does not continue.
- Idle host during the GPU stages; the CPU analyses run after the last GPU stage ends **and** after `nvidia-smi`
  lists no compute process (the chain waits up to 15 min, then stops without analysing; review r1 F5).
- **Chain bounds** (review r1 F5): each stage runs under `timeout -k` and a resident-memory watchdog (20 GB) over
  the runner's process tree. The runner starts every launcher in a new session, so the chain records the session
  of every descendant while the stage runs; a memory-cap kill signals every descendant and every recorded session
  (TERM, then KILL), and after every stage any process left in a recorded session is killed and logged.

## Analysis (fixed before the run)

### Stage A: crash location (descriptive)

**Episodes.** Primary scope: the quota rows of `episodes.jsonl` with outcome crash, in the 24 traced cells, joined to
the trace by (env, episode). Crash rows without trace frames are counted (coverage) and excluded.

**Geometry.**
- Frame k of the trace is the state after k actions. The last recorded frame L of a crash episode is the state from
  which the terminal action was taken: one control step (dt = the trace's `dt_s`, 0.1 s) before the contact was
  registered. The contact time is (L + 1) × dt.
- **Contacted bar:** the bar with the smallest horizontal surface distance from the vehicle centre at L, among bars
  whose vertical extent contains the vehicle height (G3's rule; the bars are axis-aligned boxes, which the analysis
  checks). **AMBIGUOUS:** the second-nearest bar within 0.1 m of the nearest. **VALID:** distance ≤ 0.6 m. Recorded
  as checks: whether the bar nearest to the one-step extrapolation p_L + v_L·dt is the same bar, and whether the
  kroc2027 action count equals L + 1.
- **Camera:** yaw-level (the simulator renders it with `robot_vehicle_orientation`). Heading = yaw of
  `p_vehicle_quat`; eye = centre + R(yaw) × the trace's camera translation (0.10, 0, 0.03) m.
- **In FOV (primary):** from the eye, some part of the bar's rectangular footprint lies within the horizontal FOV
  (|bearing| ≤ half-FOV, **33°**, a parameter); the bar's vertical extent meets the vertical FOV (|elevation| ≤
  **27°**) at the horizontal distance r of its nearest footprint point; and r ≤ **10 m**, the camera's obstacle range
  (`camera_obstacle_max_range`). An eye inside the footprint counts as in view. This is the geometric FOV: occlusion
  by other bars is NOT_MEASURED.
- **Windows:** the frames with index ≥ L − round(T / dt) for T = 0.5, 1 and 2 s (6, 11, 21 frames when complete).

**Categories** (mutually exclusive, primary geometry):

| Category | Definition |
| --- | --- |
| IN_VIEW | the contacted bar is in FOV at L |
| LEFT_FOV | not in FOV at L, but in FOV at some earlier frame of the 1 s window |
| BLIND | in FOV at no frame of the 1 s window |

- **EARLY** (a flag, not a category): contact within 1 s of the episode start, (L + 1) × dt ≤ 1.0 s; its 1 s
  window is truncated. EARLY crashes stay in their category and are also counted separately.

**Also per episode:** contact bearing (the nearest footprint point from the eye, relative to the heading) and side
(LEFT if > 0); whether the bar was in FOV at any frame of the last 0.5, 1 and 2 s; velocity bearing at L and of the
mean vehicle-frame velocity over the 1 s window (when the speed > 0.3 m/s), and whether it is outside the FOV;
horizontal and 3-D speed at L; roll and pitch at L (`p_quat`); time since start; the share of the 1 s window with a
valid camera detection of the target (`CAMERA_VALID`) and whether the target was geometrically visible
(`GEO_VISIBLE`); eye distance to the bar below the depth model's MinZ (0.4 m) at L.

**Shares over VALID crash episodes** (review r1 F2). The category shares, the other per-episode shares and every
interval are computed over the VALID crash episodes, where the contacted bar is identified. Non-VALID crash episodes
(contact distance > 0.6 m at L, for example out-of-bounds crashes: 3 of 161 in E3's V3s1 70/4501 cell) are not
assigned to a bar; they are reported separately per seed and density, with their count, their nominal category split
and their distance quantiles. The all-crash shares are kept as a sensitivity (`sensitivity_all_crashes`).

**Statistics.** Per policy seed (V3s1, V3s2) and pooled; point estimates are episode-weighted pooled ratios.
- **Primary interval: a cluster bootstrap with the cell as the cluster** (one density × evaluation-seed run of one
  policy), cells resampled with replacement within density, 10,000 resamples, seed 20,261,112, percentile 95 %.
  - Why the cell: the episodes of one cell share its process, scene stream and synchronized resets, and the cell is
    the unit that was replicated independently. An episode-level bootstrap (G3's) treats correlated crashes as
    independent and is too narrow. The densities are fixed by design, so cells are resampled within density, as in
    CAM-K1-E, CAM-K2-E and CAM-K2-E3.
  - With 3 cells per density per seed (6 pooled) the percentile interval is coarse and somewhat narrow.
  - Reported beside it: the env-within-cell cluster bootstrap (128 clusters per cell: within-cell uncertainty only)
    and the episode bootstrap (comparability with G3).
- Per-density category counts; quantiles of contact bearing, speed, roll, pitch, time and distance.

**Sensitivity profiles** (reported, not read): `eye_nearest_point` (primary, with the bearing of the nearest
footprint point only instead of the footprint), `g3_geometry_33` (G3's geometry: from the centre, nearest point,
no vertical or range test, half-FOV 33°) and `all_episodes` (primary geometry over every traced episode, as G3).

**Reading:**
- **TRACE_BINDING_FAIL:** the trace run is not a finished 24-cell run in plan order; a cell fails the runner's
  checks again; a reference differs from E3's manifest; the analysis script differs from the run commit's; the
  identity stage beside the trace folder is not IDENTITY_PASS on the same commit (review r1 F4); or the chain's
  G3 positive control, run by this script's bytes, is not POSITIVE_CONTROL_PASS (review r1 F7). Stage A is then
  not reported.
- **MEASUREMENT_INVALID**, with its reasons written in the output, when for either policy seed:
  - it has no crash episode, or fewer than 90 % of its traced quota crash episodes are VALID;
  - (review r1 F1) fewer than 99 % of its quota crash rows have trace frames (coverage), fewer than 99 % of its
    traced crash episodes have a kroc2027 action count equal to L + 1 (`actions_match`; a missing count counts as a
    mismatch), or fewer than 99 % have contiguous frames from 0 (`frames_contiguous`). These guard the timing on
    which every category rests (L = one step before contact); G3's traces read 100 % on all three;
  - or the analysis cannot handle the geometry (for example rotated obstacles), in which case the error is written
    as the reason instead of the analysis exiting (review r1 F10).
- **DESCRIBED** otherwise, carrying the trace run's NONINTERFERENCE_PASS or TRACE_INTERFERES.

**Positive control of the analysis (run before this preregistration; repeated by the chain at the run commit, where
it binds Stage A as stated above):**
the same code with G3's profile (origin centre, nearest point, no vertical or range test, every traced episode,
half-FOV 43.5°) on G3's raw traces (`aerial_gym_simulator/results/camonly_c2trace_v1/`) must reproduce
`docs/evidence/camonly_2026-11-04/c2trace/c2trace_analysis.json` exactly (|difference| < 1e-9).
Build run, 2026-11-12, on the uncommitted build tree:

| Condition | Crash episodes | out_last | never_in_1s | valid | left |
| --- | --- | --- | --- | --- | --- |
| A (full) | 475 | 48.8421 % | 33.6842 % | 97.2632 % | 63.5789 % |
| B_out | 5,138 | 72.0708 % | 56.4617 % | 99.8248 % | 93.3243 % |
| C (no_lidar) | 5,109 | 72.8518 % | 56.7430 % | 99.7455 % | 93.3842 % |

All equal the record (POSITIVE_CONTROL_PASS). Negative control: at half-FOV 33° the same profile gives out_last
56.0 / 80.0 / 80.2 %, which differs from the record, so the parameter is live. (out_last = 100 − IN_VIEW;
never_in_1s = BLIND.) Scope (review r1 F7): this control proves the shared plumbing exactly (trace reading, the
join, the layout lookup, the contacted bar, the windows, the categories and the percentages). The primary profile's
own geometry (eye origin, footprint interval, vertical and range tests) is validated only by the synthetic unit tests
with hand-computed answers in `tests/test_camt1.py`.

**Disclosed before the run: the in-FOV definition matters.** On G3's traces (a different policy and camera; build
smoke, not evidence), the eye-origin footprint test and the eye-origin nearest-point test differ by tens of points
in IN_VIEW (camera-only F at 43.5°: 50 % against 22 %). Wide bars beside the camera can reach into the FOV while
their nearest point lies outside it. Both are reported; the guidance below requires them to agree.

### Stage B: speed cap and delay (gated)

- **Contrasts:** each arm against native, paired by cell: per policy seed s, arm_s − native_s over the 6 blocks
  (115 / 205 bars × 3 seeds); pooled, weights ½ (arm_s1 + arm_s2 − native_s1 − native_s2).
- **Metrics:** quota crash, capture and timeout rates (pp); median capture time of the quota capture episodes
  (kroc2027 actions × `rl_dt_s`, seconds).
- **Intervals** (as CAM-K2-E3): the stratified percentile bootstrap (seeds resampled within density, 10,000
  resamples, seed 20,261,108) and the stratified t interval (df 4).
- **Gate, per arm and seed:** CAM-K2-E3's `gate_d` on the crash contrast, unchanged:

| Label | Rule (both intervals must agree) |
| --- | --- |
| UNRESOLVED | the two intervals disagree on whether 0 is excluded, or on the side |
| MATERIAL_REDUCTION | both upper ends < −5 pp |
| SMALL_REDUCTION | both upper ends < 0 and the mean ≤ −2 pp |
| WORSE | both lower ends > 0 and the mean ≥ +2 pp |
| NEGLIGIBLE | both intervals inside (−2, +2) pp |
| NO_DETECTED_CHANGE | otherwise |
| CRASH_TO_TIMEOUT | a REDUCTION whose timeout contrast mean is ≥ +2 pp |

  The **material threshold is the 2 pp floor**: a reduction needs a mean of at least −2 pp (SMALL_) or an interval
  wholly below −5 pp (MATERIAL_). The arm's reading is the label both seeds share, else SEEDS_DISAGREE; the pooled
  label is reported, not read.
- **Controls on CAM-K2-E's treated cells at the same 6 blocks** (each must read as required, else CONTROL_FAIL and
  no label is reported: the output then carries `labels: null` and no per-arm label anywhere, only the contrasts'
  numbers; review r1 F8). Values on the existing data, computed before this preregistration
  (`tools/camt1_analyse.py controls`):

| Contrast | Required | Label | Mean crash |
| --- | --- | --- | --- |
| K1s1 − K2s1 (known bad) | WORSE | WORSE | +3.54 pp |
| K2s1 − K1s1 | SMALL_ or MATERIAL_REDUCTION | SMALL_REDUCTION | −3.54 pp |
| K1s2 − K2s2 (known bad) | WORSE | WORSE | +4.93 pp |
| K2s2 − K1s2 | SMALL_ or MATERIAL_REDUCTION | SMALL_REDUCTION | −4.93 pp |
| K2s1 − K2s2 (null) | not MATERIAL_REDUCTION, not WORSE | NEGLIGIBLE | +0.42 pp |

  The resolution is about ±2 pp (the null's bootstrap interval is [−0.07, 0.90] pp, its t interval
  [−0.44, 1.27] pp).
- **Bindings** (EVAL_BINDING_FAIL otherwise): identity IDENTITY_PASS, control CONTROL_PASS and envelope COMPLETE on
  one commit; 36 cells in
  plan order; every envelope cell passes the runner's checks again; the determinism equalities above; the native
  cells at E3's manifest; the analysis script equal to the run commit's.
- **Reported, not gated:** capture, timeout and median capture time contrasts with both intervals; per-arm commanded
  speed maxima, the share of episodes commanding above 2.5 / 2.0 m/s, and the mean actual speed (the caps'
  manipulation check).

## Guidance for a later retraining (not a gate)

Retraining is not approved; this mapping only fixes in advance how the results would be read.

- **Stage A, pooled:** the dominant category is the one with the largest share, and is CLEAR when its primary
  interval's lower end exceeds both other categories' upper ends. A direction is named only when the primary
  (footprint) and the `eye_nearest_point` profiles give the same CLEAR dominant category; otherwise
  DEFINITION_SENSITIVE (or MIXED when the primary has none).
  - BLIND → out-of-FOV obstacle memory, or a wider depth FOV (80°).
  - LEFT_FOV → short-term memory of obstacles that were seen and left the view.
  - IN_VIEW → depth fidelity near the bar (MinZ, the validity rule) or speed (Stage B).
  - EARLY ≥ 20 % → a note that initial placement, not only the policy, contributes.
- **Stage B:** a cap reading SMALL_ or MATERIAL_REDUCTION → speed is a lever (a retraining or deployment cap);
  NEGLIGIBLE → speed is not the bottleneck at these caps. d1 reading WORSE → the policy is latency-sensitive at
  100 ms (deployment latency must be bounded or trained for); NEGLIGIBLE → not at one step.

## What may be said, and what may not

- Stage A is descriptive: "consistent with", never causal. It describes CAM-K2 v3 in simulation under this
  condition, with geometric FOV (no occlusion) and the eye at the trace's mount.
- If Stage A reads TRACE_INTERFERES, its numbers describe the traced runs only, not E3's cells.
- Stage B labels are per arm and lineage (both seeds); the pooled label is descriptive.
- No real-flight claim. The OAK-D model, the vehicle randomization and the validity rule are simulation models.

## Disclosed limits

- One condition (E3's V3 arm), the E2 target, 3 evaluation seeds; Stage B at 115 and 205 bars only.
- The contacted bar is inferred at the last recorded frame, one step before the contact. Ambiguous episodes stay
  in the shares and are reported; non-VALID episodes are reported separately (review r1 F2).
- Occlusion by other bars is not modelled; "in FOV" is geometric.
- Geometric approximations (review r1 F9), none expected to bind here:
  - the vertical test measures elevation over the horizontal distance r of the nearest footprint point, whereas a
    pinhole camera's bound is z / (r·cos(bearing)), so it is slightly lenient off-axis; at 1 m altitude with 0–3 m
    bars it never binds;
  - the policy's 40-column obstacle depth image has pixel centres only out to about ±32.3° (about 1.65° apart), so
    the geometric ±33° is an upper bound on the half-FOV the policy samples;
  - with a median contact distance of about 0.23 m at L (G3), the eye is usually closer than the depth model's
    MinZ (0.4 m). IN_VIEW therefore means "inside the camera's FOV", not "visible in depth"; `below_minz` is
    reported.
- The Stage B "reverse" controls (K2s − K1s) mirror the known-bad contrasts on the same blocks. They exercise the
  reduction branch of `gate_d` but are not independent controls (review r1 F10).
- The runner compares its allowlisted environment stage to stage, not with CAM-K2-E3's report; `native_identity`
  covers this empirically (an inherited `PYTHON` that changed the run would fail it) (review r1 F10).
- If Stage A reads TRACE_INTERFERES while the identity passed, V3s2's native reproduction is not tested directly.
- With 3 cells per density per seed, the cell-cluster percentile interval is coarse.
- The delay arm uses G7's hook, which zeroes the first action of an episode (hover).
- The tree contains no copy of Policy F's checkpoint; the preflight checks the protocol's launcher and player pins
  only (a deliberate difference from `kroc2027_eval.static_problems`).
- Memory bound: the chain caps the resident memory of each runner's process tree, launcher sessions included, by a
  watchdog (20 GB, polled every 10 s, so a faster spike is not caught); a virtual-memory ulimit is not used for the
  GPU process, and this host's user scopes do not enforce a memory cgroup.
- Orphan gap (review r2 N3): the chain records launcher sessions every 10 s. A launcher started less than one poll
  before the chain's own `timeout` fires is not recorded and is not reaped; it would run its cell to the end
  (about 3 min), and the final GPU-idle wait would wait for it. This is unlikely: the runner's own budget ends each
  stage 600–900 s before the chain's timeout.

## CPU gate before the GPU run: baseline equivalence (review r1 F6)

PUBLIC_CPU does not read 0 failures / 0 errors in this worktree, and is never reported as PASS here.
- 15 tests fail identically on a clean detached worktree at the base `eefb832`. A new worktree lacks Policy F's
  ignored checkpoint under `runs/` and the untracked raw `results/` folders, and some records name the
  `MOTAR-camonly` path. None of the 15 imports CAM-T1 code. Their names, sorted with `LC_ALL=C`, are in
  `~/motar_tmp/camt1_20261112/cpu_gate_baseline_failures.txt`.
- **The gate**, after the local commit and before the GPU approval file is written:
  1. `tests/test_experiment_index.py` passes;
  2. a complete full PUBLIC_CPU run (`OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 PYTHONNOUSERSITE=1 CUDA_VISIBLE_DEVICES=
     nice -n 10 timeout 3600 python -B tools/run_tests.py`; **≥ 3,977 tests, 18 skipped**, with run_tests.py's
     PUBLIC_CPU report block) fails exactly those 15 names. `~/motar_tmp/camt1_20261112/cpu_gate_diff.sh <log>`
     checks this mechanically (review r2 N1): BASELINE_EQUIVALENT only when the report block is present and agrees
     with the "Ran N tests" line, N ≥ 3,977, skipped = 18, failures + errors = 15, the sorted FAIL/ERROR set equals
     the 15 names, and the log is newer than HEAD's commit; otherwise BLOCK. It writes
     `~/motar_tmp/camt1_20261112/cpu_gate_<HEAD>.json` (HEAD, the log path and SHA-256, the counts, the verdict) and
     never overwrites one;
  3. the result is recorded as "PUBLIC_CPU not 0/0 (baseline-equivalent): 15 pre-existing environment failures,
     identical to base `eefb832` in a fresh worktree".
- **Any 16th failure, or a missing one, blocks the GPU run.** The chain `run_camt1.sh` refuses to start unless
  `cpu_gate_<HEAD>.json` exists for the HEAD it launches and reads BASELINE_EQUIVALENT.

## Review before the commit

### Review r1 (pre-gate)

- **Reviewer:** Claude Code subagent, Opus 5.5 (`claude-opus-5-5`), fresh context, read-only on the repository, writing
  only to `~/motar_tmp/camt1_20261112/review_pre_r1/` (`REVIEW.md`).
- **Verdict: PASS_WITH_FIXES.** No P0 or P1 finding; six P2 and four P3.
- **Dispositions** (the repair map with file locations and tests is `~/motar_tmp/camt1_20261112/REPAIR_r1.md`):

| ID | Severity | Finding | Disposition |
| --- | --- | --- | --- |
| F1 | P2 | Stage A did not gate trace coverage, `actions_match` or `frames_contiguous` | FIXED: each below 0.99 in a seed reads MEASUREMENT_INVALID with the reason |
| F2 | P2 | Non-VALID crashes were assigned a category relative to a bar they did not hit | FIXED: shares over VALID episodes; non-VALID reported separately; all-crash shares a sensitivity |
| F3 | P2 | Identity controls did not require `result_equal` | FIXED: the identity rule requires it (G7) |
| F4 | P2 | A reproduction failure would surface only after the trace stage, as TRACE_INTERFERES | FIXED: `native_identity` runs first as its own stage; NOT_REPRODUCED stops the chain |
| F5 | P2 | The chain's watchdog and timeout could not stop a launcher in its own session | FIXED: sessions recorded and killed; the GPU must be idle before any CPU analysis |
| F6 | P2 | CPU gate wording | FIXED: baseline-equivalence gate above |
| F7 | P3 | Positive control scope; run-commit control not bound | FIXED: scope stated; Stage A requires the chain's POSITIVE_CONTROL_PASS at its own bytes |
| F8 | P3 | Labels written when the Stage B controls fail | FIXED: no label written on CONTROL_FAIL |
| F9 | P3 | Vertical, pixel-coverage and MinZ approximations | DISCLOSED in *Disclosed limits* |
| F10 | P3 | Mirror controls; analysis exit on rotated obstacles; environment vs E3; GPU idleness per cell; narrow intervals | Exit FIXED (a written MEASUREMENT_INVALID); GPU idle check before every cell ADDED; the others DISCLOSED |

### Review r2 (pre-commit re-review)

- **Reviewer:** Claude Code subagent, Opus 5.5 (`claude-opus-5-5`), fresh context, read-only on the repository, writing
  only to `~/motar_tmp/camt1_20261112/review_pre_r2/` (`REVIEW.md`).
- **Verdict: PASS_WITH_FIXES.** No P0 or P1. F1–F5 and F7–F10 RESOLVED or DISCLOSED as cited; F6 PARTIAL (→ N1).
  It reran the tests (42 OK), the positive control (byte-identical), the gate script, and the chain's kill logic with
  bystander processes (untouched).
- **Dispositions** (map: `~/motar_tmp/camt1_20261112/REPAIR_r2.md`):

| ID | Severity | Finding | Disposition |
| --- | --- | --- | --- |
| N1 | P2 | The gate script accepted a partial-suite log; nothing tied a gate result to the launch | FIXED: report block, ≥ 3,977 tests, 18 skipped, log newer than HEAD; `cpu_gate_<HEAD>.json`; the chain refuses without it |
| N2 | P3 | `nvidia-smi` unbounded in the chain; an uncaught GPU-query error in the runner | FIXED: `timeout 30` and a failed query is "not idle"; the runner records `gpu_check_error` and reads INCOMPLETE |
| N3 | P3 | A launcher orphaned within one poll by the chain timeout is not reaped | DISCLOSED in *Disclosed limits* |
| N4 | P3 | No grace period for the per-cell GPU check | DISCLOSED in the preflight list (rustdesk only excluded; rerun in a new parent folder) |
| N5 | P3 | The module form of the test command does not import | FIXED in the text: the discover form is named |
| N6 | P3 | Stage A's written-reading path missed `TypeError` | FIXED: caught; MEASUREMENT_INVALID with the reason |
| N7 | P3 | Status lines, r2 record, rerun sentence, nonce test case | FIXED |

- **Status:** reviewed r1 + r2, ready to commit. The GPU run needs the post-commit CPU gate and the user's approval.
