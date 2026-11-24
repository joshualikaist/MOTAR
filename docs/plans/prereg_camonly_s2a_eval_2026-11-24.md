# CAM-S2a-E v1: evaluation of the CAM-S2a camera-only from-scratch policy in its training environment (preregistration, 2026-11-24)

**State:** **PREREGISTERED, NOT_RUN** (draft r3 committed unchanged in substance). Written after CAM-S2a run 1 ended (2026-11-24 11:20 KST, `COMPLETED_PROCESS`, faults 1 → 1)
and before any evaluation cell; revised after the independent pre-run design reviews r1 (Claude Opus 5.5, GO_WITH_FIXES,
`~/motar_tmp/cams2a_eval_20261124/review_r1/REVIEW_r1.md`; 0 P0, 5 P1, 10 P2, 5 P3) and r2 (delta, GO_WITH_FIXES,
`.../review_r2/REVIEW_r2.md`; three P0 introduced by r2's code, one P1 wording, two P2, one P3), all addressed below.
To become `docs/plans/prereg_camonly_s2a_eval_2026-11-24.md`; never edited after its commit, amendments in a new dated file.
Branch `camonly/dev`, base `fef6800` (the commit that recorded CAM-S2a run 1's result; the training ran at `22ed16f`, which
is the diff base the runner binds). Family `camera-only`.

- Runner: `tools/cams2a_eval_run.py`. Analysis: `tools/cams2a_eval_analyse.py`. Launcher copy:
  `aerial_gym/rl_training/rl_games/eval_navrl_cams2a_arms.sh` (SHA-pinned in the runner at the commit).
- New runtime wrappers (opt-in, default OFF, installed by `tools/realveh_entry.py` after `navrl_cams2`, in this order):
  `aerial_gym/task/navrl_task/navrl_yawclip.py` (the yaw-rate condition) and `aerial_gym/task/navrl_task/navrl_cams2diag.py`
  (per-episode recorder, outermost). Tests `tests/test_yawclip.py`, `tests/test_cams2diag.py`, `tests/test_cams2a_eval.py`.
- Unit folder: `~/motar_tmp/cams2a_eval_20261124/` (raw cells under `raw/identity_01/`, `raw/eval_01/`).

## Authorization

- The user's approval words (2026-11-24, Korean, verbatim): **"평가 fable 5.2로 진행해줘"**. Note: Fable 5.2 is not available in
  this session; the evaluation work was delegated to Fable 5.1 (Claude Fable 5.1), which drafted this package. The GPU run
  itself still needs the approval file `~/motar_tmp/cams2a_eval_20261124/execution_approval.json` with `approved_by`,
  `approved_at`, `commit` (HEAD), `scope` and `scope_sha256`; the two scope fields are generated, not typed, by
  `python tools/cams2a_eval_run.py scope`, and the preflight refuses any mismatch (review Q6).
- Design reviews r1 and r2 are done (below). Review r2 asked for no further full review: its three P0 and the other
  findings were fixed in r3 and are covered by regression tests and the full PUBLIC_CPU suite on the eval commit.

## Why

- **CAM-S2a** (`docs/plans/prereg_camonly_s2a_2026-11-23.md` + amendment 1; D-2026-11-23-3) trained a camera-only policy from
  random initialisation against the quadrotor-mesh target with the measured vehicle values, a 1–2 step action delay and the
  capture rule "centre distance ≤ 1.0 m AND closing speed ≤ 1.0 m/s AND detected" (contact < 0.40 m = collision). Run 1
  (seed 1, 12,000 epochs, 13.1 h, `~/motar_tmp/cams2a_20261123/train_s1/`) ended at **175 bars**, not the preregistered 205:
  the inherited 16,384-episode promotion window made each stage last 1,585–1,828 epochs (training review F3). Training-rollout
  rates are not evaluation results (last 100 epochs at 175 bars: capture 0.615, crash 0.263, timeout 0.123).
- The training preregistration says evaluation needs its own preregistration. This is it. No number of this unit compares
  with earlier lineages: the capture rule, the target and the environment are new.
- The committed training review (`docs/evidence/camonly_2026-11-04/cams2a/REVIEW_r1.md`, Claude Opus 5.5, PASS_WITH_LIMITS)
  sets five requirements for any evaluation, all met below: (F2) every cell must install `navrl_cams2` with the receipt's
  settings, else a 0.40 m contact scores as a capture; (F4) the cams2 contact/capture counters were never written out, so the
  evaluation must separate target contact from obstacle collision per episode; (F6) the task's crash-cause code and recorders
  see a wrapper-converted contact as a capture, so crash-cause diagnostics must be corrected or labelled; (F5) the capture
  rule uses the previous observation's detection flag: evaluate as trained and say so; (F7) report the relative speed at
  capture, lateral part included, so fly-by captures are visible.

## Questions

1. **Does the final checkpoint outperform the earliest kept checkpoint in the final training environment?** Gate L:
   ep12000 against ep1000 on the same blocks. This is a pipeline-sanity gate, not a discovery: ep1000 trained only at 70 bars
   with the FOV bearing curriculum a third of the way through and the goal-distance window still widening (review P2-4), so
   LEARNING_SUPPORTED is the expected outcome; LEARNING_NOT_DETECTED or LEARNING_REFUTED would question the pipeline.
2. **How does the final checkpoint behave at the trained density (175), one density below (115) and the preregistered but
   never trained density (205)?** Reported per density; not gated (densities are strata, not a contrast).
3. **Does the real vehicle's 60 deg/s yaw-rate limit change the outcome?** Gate Y: ep12000 with the yaw command clipped at
   1.047 rad/s against ep12000 unclipped, action scale unchanged, with a preregistered minimum binding share.
4. **Diagnostics (not gated):** the intermediate checkpoints at 175 bars; crash split into target contact vs obstacle
   collision; crash per 1,000 steps; capture kinematics; what the policy commands while the target is not detected;
   camsafe, yaw-clip and governor telemetry.

## Design

**The environment is the training environment, as trained.** Every cell exports the NAVRL_* variables of the training
receipt (`~/motar_tmp/cams2a_20261123/train_s1/receipt.json`, key `env_navrl`, **161 keys**; SHA-256 recorded in every cell
receipt and run report) into the launcher copy, with exactly these replacements (`cams2a_eval_run.REMOVED_FROM_RECEIPT`,
`EVAL_OVERRIDES`, `MODULE_SEED_KEYS`, and the launcher's per-cell keys):

| Replaced | Training value | Evaluation value | Why |
| --- | --- | --- | --- |
| `NAVRL_DENSITY_CURRICULUM` | 1 | 0 | fixed density |
| `NAVRL_DENSITY_START/FINAL`, `NAVRL_NUM_BARS` | 70 / 205 (curriculum) | the cell's density | `NAVRL_NUM_BARS` wins over the checkpoint's density (navrl_task.py:2137-2140, 4603-4607) |
| `NAVRL_SEED` | 1 | 4501, 4502, 4503 | the family's grid seeds; distinct from training |
| `NAVRL_VEH_RAND_SEED`, `NAVRL_DEPTH_SEED`, `NAVRL_P9_SEED`, `NAVRL_CAMS2_DELAY_SEED`, `NAVRL_GAIN_RAND_SEED` | 4932, 4942, 4952, 4961, 4971 | 10·seed + 1 … + 5 (45011–45035) | the same distributions, new draws; matched across the arms of a block (CAM-K2-E's rule) |
| `NAVRL_REALVEH_RECEIPT` | the training receipt | the cell's receipt | the entry refuses an existing receipt |
| added | — | `NAVRL_BULK_EVAL=1`, `NAVRL_EVAL_CHECKPOINT` (= the arm's pin), `NAVRL_EVAL_RUN_NONCE`, `NAVRL_BULK_EVAL_JSON`, `NAVRL_EPISODE_LEDGER=1`, `NAVRL_EPISODE_QUOTA_PER_ENV=8`, `NAVRL_PLAYER_GAMES_CAP=10⁹`, `NAVRL_CAMS2DIAG=1`, `NAVRL_CAMS2DIAG_YAW_REF_RADPS=1.047`, `NAVRL_CAMS2DIAG_JSONL` | evaluation mode, per-episode files, the recorder (OFF in the identity control `S12k_nodiag`) |
| added (YC arms only) | — | `NAVRL_YAWCLIP=1`, `NAVRL_YAWCLIP_RADPS=1.047` | the yaw-rate condition |

Everything else is the receipt's: `no_lidar`; camera and detection 312 × 234 at 66.7 × 52.6°, mount 0.08 m forward / 0.03 m up;
the D8 quadrotor mesh target (`mesh_flat`); legacy `mixed` target motion, 0.3–1.5 m/s; `navrl_band` placement with the
general goal distance 6–28 m and **no** KROC2027 initial-view rule; riskcap governor with FREE 2.5 m/s; camsafe (blind cap
1.0 m/s, penalty 0.5, collision −40, r_ss clip 4 m); cams2 (delay k ∈ {1, 2} per episode, capture 1.0 m / 1.0 m/s / detected,
contact 0.40 m, altitude 1.5 m); vehicle randomisation at T/W 3.5 (±15 %, imbalance ±5 %, τ 0.02–0.04 s); OAK-D depth model;
gain randomisation ±20 %; P9 injector; `NAVRL_YAW_RATE_MAX=3.0`, `NAVRL_MAX_VELOCITY=2.5`. The training-only PPO and curriculum
keys stay set and are inert or saturated in play mode: `NAVRL_BULK_EVAL` selects the full 6–28 m goal distribution whatever the
K window (navrl_task.py:1626-1632, 2068-2072, 2426-2440, 2695; review Q1 resolved), the FOV bearing curriculum and the
target-speed ramp read the task-step clock and are saturated (:266-276). **Machine-checked per cell** (review P2-3): the
result's `condition` must show `goal_dist_min_m 6 / goal_dist_max_m 28`, `full_goal_distribution true`,
`fov_curriculum_saturated true`, riskcap with FREE 2.5, P9 ON at the training receipt's SHA, 600 steps, 128 envs,
`d8_dynamic_mesh_treatment true` (a bool, as the task writes it; review r2 P0-1) and `d8_dynamic_mesh_contract.mode = mesh_flat`;
the test of this check runs against a real CAM-K2-E3 `result.json`, not a fixture built from the expectation.
Non-NAVRL runtime: `AERIAL_GYM_SIM_NAME=base_sim`, the aerialgym interpreter and `GPU4GB` unset are **set by the runner, never
inherited from a shell**, recorded in every cell receipt and checked (review P2-2). The cell's launcher-shell NAVRL snapshot
**and** the entry's own process `env_navrl` must both equal the training `env_navrl` except the keys of the table, each of
which must carry the cell's value (`SNAPSHOT_EXPECTED`; review P2-1), or the cell FAILs.

**Evaluated as trained (training review F5, F1, F8, F9).** The capture test uses the detection flag of the observation the
policy acted on (one control step older than the position test); the previous-action channel is the training hybrid (delayed
governed xy, raw z and yaw); the progress-term and priority details of the wrapper are unchanged. The player acts
deterministically (the yaml's `deterministic: True`, the mean action; `action_selection: configured` in the condition). Nothing
in the trained rules is altered; the two new modules only clip the yaw command (YC arms) and record.

**Source binding (review P1-3).** The realveh receipt hashes 13 files plus cams2 and camsafe; the rest of what a cell
executes (`runner.py`, `target_motion.py`, the network yaml, the ledger, configs, the D8 asset) is bound by the preflight rule
`git diff --name-only 22ed16f HEAD ⊆ {docs/**, WORKLOG.md, tests/**, tools/realveh_entry.py, tools/cams2a_eval_run.py,
tools/cams2a_eval_analyse.py, the launcher copy, the two new modules}`; the changed-path list is recorded in the run report
and re-checked by the analysis. **A refusal caused by unrelated commits** landing on `camonly/dev` between `22ed16f` and
the eval commit (review r2 P2-11) is resolved only by a dated amendment that lists the changed paths and states why each
leaves the cells' execution unchanged; the allowlist in the runner is never widened silently. Alternatively the run is made
from a dedicated worktree at the eval commit.

**Checkpoint-contract lines (review P1-4).** The task compares the checkpoint's saved contract with the running environment
and logs `… MISMATCH |` warnings (navrl_task.py:3771-3920, 4276, 4499-4591). Every such line in `cell.log` is recorded, and a
cell FAILs on any line outside the preregistered allowlist: `TRAINING SEED MISMATCH` (seed 1 vs 4501-4503) and `CURRICULUM
CONFIG MISMATCH | NAVRL_DENSITY_*` / density / n_bars (curriculum 0 and START = FINAL = N against the checkpoint's 70 / 205).
The training log has zero such lines. The identity stage confirms the list: an unexpected line there is IDENTITY_FAIL.

**Arms** (checkpoints are `last_gen_ppo_ep_*.pth` of `runs/ppo_261123_2213_navrl_v2-cams2a-scratch-camonly-s1/nn/`, pinned
by SHA-256 in the runner from `~/motar_tmp/cams2a_20261123/train_s1/checkpoint_sha256.txt`; `gen_ppo.pth` is refused):

| Arm | Checkpoint | Trained density at that epoch | Yaw clip | Stage |
| --- | --- | --- | --- | --- |
| S1k | ep1000 (`1bf00899…`) | 70 (FOV curriculum at 1/3, K window still widening) | off | A (gated) |
| S12k | ep12000 (`9fc9dfb7…`) | 175 | off | A (gated) |
| YC12k | ep12000 | 175 | **on, 1.047 rad/s at the command stage** | A (gated) |
| S3k | ep3000 (`22a20ddd…`) | 85 | off | B (175 bars), C (115, 205; optional) |
| S6k | ep6000 (`bb956b55…`) | 100 (115 from epoch 5108) | off | B (175 bars), C (115, 205; optional) |

**Grid and stages (review P1-5; THE plan, not a fallback):** densities **115, 175, 205** bars × seeds **4501–4503**; quota
**8 episodes per environment** (128 environments; 1,024 in-quota episodes per cell); blocks are (density, seed).
- **Stage A** (27 cells): S1k, S12k, YC12k on all 9 blocks, **block-major** (for each density, for each seed: S1k, S12k,
  YC12k), so that every block is complete before the next starts. The gates use stage A only.
- **Stage B** (6 cells): S3k, S6k at 175 bars × 3 seeds. Required for COMPLETE.
- **Stage C** (12 cells, optional): S3k, S6k at 115 and 205 bars × 3 seeds. A C cell starts only when the remaining eval budget
  is ≥ 1.10 × the ep12000 identity mean cell time (review r2 P0-3); otherwise C stops (`stage_c_stopped_by_budget`). A
  timed-out or FAILed C cell is recorded and stops C but **cannot** turn a complete A+B into INCOMPLETE or EVAL_BINDING_FAIL:
  the verdict and every binding are judged on the stage A and B records only (a VOID_HOST_FAULT anywhere still voids the run).
Why these densities: 175 is the density the final checkpoint trained at; 205 is the preregistered final density it never
reached; 115 is a passed stage and a family grid density; none is divisible by 4 (TF32 note). Why quota 8: the 6 h window (time
estimate below) with the controls at this scale (below).

**The yaw-rate condition (review P1-2).** `navrl_yawclip` wraps `transform_action_to_command` after `navrl_cams2` and clamps
`command[:, 3]` to ±1.047 rad/s. The action scale (`NAVRL_YAW_RATE_MAX=3.0`), the observation normaliser
(`robot_body_angvel / yaw_rate_max`, navrl_task.py:6403, 6473), the previous-action channel (the raw action, :5053) and the
controller clamp (`lee_controller_config_navrl.max_yaw_rate` = 3.0) are untouched. Telemetry `[yawclip]` lines are running
totals written every 320 calls; the last line is the whole-cell clipped share to within 320 calls. The **binding share** of the
clip, the share of control steps whose requested yaw rate (raw action × 3.0) exceeds 1.047 rad/s, is recorded exactly per
episode by the recorder in **both** S12k and YC12k cells (`yaw_request_over_ref_steps`), together with the executed-yaw-over-limit
count, which must be 0 in every clipped cell: checked per cell by the runner (so `YC12k_negative` surfaces a defect in the
identity stage) and again as an analysis binding. The identity stage requires a whole-cell clipped share > 0 and at least
one requested step over the limit; gate Y requires a pooled binding share ≥ **1 %** (`MIN_BINDING_SHARE`) for any tolerance
label.

**Per-episode recorder (training review F4, F6, F7; design review P1-1).** `navrl_cams2diag` wraps
`transform_action_to_command` and `compute_state_reward_and_terminations` as the outermost layer and writes
`cams2diag_episodes.jsonl`, one row per finished episode aligned with the task's ledger (`episodes.jsonl`) on
(env_index, env_episode_index): outcome; `crash_class` (`target_contact_swept` = the cams2 conversion, recognised by the
task's `_crash_cause_code` staying −1, navrl_task.py:6137; `contact` = the task's cause 0, bar contact or a PhysX contact
with the target mesh; `below`, `above`, `out_of_bounds`); the episode's delay k; steps; steps with no detection; the raw action
(as vehicle-frame velocity and yaw rate) and the executed command averaged over hidden and over visible steps; hidden steps
executed above the 1.0 m/s blind cap; requested / executed yaw over 1.047 rad/s (compared with a 1e-5 tolerance: a float32
clip at 1.047 reads 1.0470000505, review r2 P0-2; a float32 integration test cams2 → yawclip → cams2diag covers it); and at a
capture the centre distance, closing
speed, relative speed and its lateral part. Its `[cams2diag]` summary line (every 320 steps) is labelled a running snapshot
and carries `navrl_cams2`'s own counters. `result.json`'s `crash_causes` are reported with the label "excludes target
contacts". The recorder reads only; its passivity is shown by the identity cell `S12k_nodiag` (recorder OFF, everything else
equal, must equal `S12k_identity` on `result.json` and every ledger row) and by the CPU test
`test_cams2diag.Install.test_recorder_is_passive` (identical rewards, terminations, flags and commands with and without it).

**TF32 `cdist` note.** rl_games enables TF32. The evaluated densities avoid multiples of 4. Distance checks that decide
outcomes do not use `cdist`: the task's capture/contact swept test is a difference-norm segment test (navrl_task.py:6118-6124),
the cams2 capture rule is `rel.norm` (navrl_cams2.py:98-100), obstacle contact is the task's collision path, and the recorder
uses difference norms. What remains `cdist`-based, as in training, is the legacy target's own bar screening
(`target_motion.py:763`; `docs/audits/moving_target_tf32_readonly_2026-10-16.md`: H is `TF32_EXPOSED_VALIDITY_UNVERIFIED`)
and spawn/goal placement queries (navrl_task.py:133, 2745, 4813, 4827). The `direct_l2_v1` backend of the earlier evaluations
is not available for legacy dynamics (target_motion.py:54-56); the training environment is kept as trained and this exposure is
disclosed as a limit, not corrected.

**Identity stage** (5 cells, 205 bars, seed 4501, quota 8; cap 1.0 h; IDENTITY_FAIL stops the unit):
- `S12k_identity`: ep12000, the reference cell;
- `S12k_repeat`: must equal it exactly (`result_equal` and `episodes_equal`): determinism of the whole path;
- `S12k_nodiag`: recorder OFF, must equal it exactly: the recorder is passive (review P1-1); its wall time against
  `S12k_identity` measures the recorder's cost;
- `S1k_smoke`: ep1000 must be valid and differ from it: the checkpoint path acts;
- `YC12k_negative`: ep12000 with the clip must differ from it and show a whole-cell `[yawclip]` clipped share > 0: the clip acts.
`identity_verdict` is a pure function with tests (review P2-6). There is no earlier CAM-S2a cell to reproduce.

**Eval stage:** one invocation; the plan above (A, B, C). **Rate gate** (recorded in the run report; review P1-5): the
projected seconds of stages A + B, computed per arm from the identity cells (the 9 S1k cells at `S1k_smoke`'s wall time, the
other 24 cells at the mean of the four ep12000 identity cells) × 1.10, must not exceed 5.0 h, else the eval refuses
(PREFLIGHT_FAIL) and an amendment decides. The identity cells run at 205 bars, which is **not** claimed to be the slowest
density (lower density means fewer early crashes and longer episodes); the per-arm projection and the 1.10 margin are the
protection, and stage C absorbs the remainder under its own start rule. Caps: identity 1.0 h, eval 5.0 h, enforced in both
loops; every cell's timeout is min(2,400 s, the remaining budget) (review P2-7). A FAIL cell in A or B stops the stage
(INCOMPLETE); in C it stops C only. A rise in the host
fault count, a boot-id or online-CPU change voids the run (VOID_HOST_FAULT). Idle host.

**Cell checks** (every cell; `cams2a_eval_run.check_cell` + `receipt_problems`): launcher receipt OK on this commit with 0
dirty files, the checkpoint SHA, the training-receipt SHA, `base_sim`, the interpreter, `GPU4GB` unset; ledger complete (quota 8
in 128 envs) and clean; result condition as above; realveh receipt: `cams2`, `vehicle_rand`, `depth_model` settings equal to
training's with the cell's seeds, `camsafe` and `camtrain` equal, `cams2.config_after` equal (altitude 1.5, camera x 0.08,
success_radius 0.40), `cams2_sha256` and `camsafe_sha256` equal to training's, every hashed module byte-identical to training's
except `realveh_entry` (which must equal HEAD's); `yawclip` record present with limit 1.047 and scale 3.0 iff a YC arm;
`cams2diag` record present with the yaw reference iff not `S12k_nodiag` (and absent there); both snapshots as above;
`[camsafe]` present; `[yawclip]` present iff YC; `[cams2diag]` present iff the recorder is on; recorder rows aligned with the
ledger (count and outcome per episode); in YC cells zero executed-yaw-over-limit steps and at least one requested; no
MISMATCH line outside the allowlist.
**Training review F2 is enforced twice:** the runner refuses to build a cell environment whose training receipt lacks
`NAVRL_CAMS2=1`, any `NAVRL_CAMS2_*` parameter or `NAVRL_CAMSAFE=1` (ValueError before launch; positive control in
`tests/test_cams2a_eval.py`), and a cell whose realveh receipt lacks the `cams2` install record FAILs.

**Preflight** (fail closed): approval naming `SCOPE`, its SHA-256 and HEAD; clean tree; branch `camonly/dev`; the
preregistration, launcher copy, entry, wrapper, both modules, runner and analysis committed; launcher at its pinned SHA (a
placeholder refuses); the diff allowlist above; training run `COMPLETED_PROCESS` at `22ed16f` with faults unchanged; the four
checkpoints at their pins and listed in the training unit's hash file; one RTX 3070; no other GPU compute; import origins
inside the tree; the interpreter present; free disk ≥ 1 GB; new output folder; for the eval, exactly one IDENTITY_PASS report
on this commit, the same runner environment and training receipt, and the rate gate.

**Reference cell with Frozen Policy F: omitted (NOT_FEASIBLE_WITHOUT_NEW_CODE, and not meaningful).** The CAM-S2a entry
chain requires the LiDAR ablation (`tools/camtrain_entry.py:37` refuses any condition but `no_lidar`; `navrl_camsafe.py:133`
refuses to install without the ablation; `navrl_cams2.py:108` refuses without camsafe), so F's 898-D camera + LiDAR actor
cannot run with LiDAR on inside the CAM-S2a rules without new code. Without the cams2 rules F would be scored on a different
task (0.5 m success radius, FREE 3.5355 m/s, 87 × 58° camera, red-sphere contract), which is not a reference for this unit.

## Estimated GPU time (review P1-5)

- A cell runs until the **slowest** of 128 environments has finished 8 episodes. From the training log near 175 bars the mean
  episode is 224–232 steps with 12 % timeouts at 600 steps; an 8-episode sum has a mean of about 1,840 steps and (INFERRED,
  non-timeout SD ≈ 120 steps) an SD of 450–550, so the maximum over 128 environments is plausibly 2,900–3,400 steps. Training
  rollout at 175 bars ran 3.3–3.7 s per 32-step epoch (105–115 ms per step). With 60–90 s of task build: **5.5–7.5 min per
  ep12000-like cell**; the S1k cells (early crashes) are shorter.
- Identity 5 cells ≈ 0.5–0.6 h (cap 1.0 h). Stage A + B = 33 cells ≈ 3.0–4.1 h; the rate gate refuses above
  (9 × t_S1k + 24 × t_12k) × 1.10 = 5.0 h, i.e. with t_S1k = 300 s roughly t_12k > 637 s (10.6 min). Stage C (12 cells,
  1.1–1.5 h) runs only inside the cap. **Total ≈ 4–6 h**, hard cap 6.0 h.

## Analysis (fixed before the run)

Blocks are (density, seed) pairs; rates are per-block quota rates (`gr_analyse.rates`). Intervals as CAM-K2-E with this
grid's degrees of freedom: a stratified percentile bootstrap (10,000 resamples, seed 20,261,124, seeds resampled within
density) and a stratified t-interval with df = Σ(n_d − 1) = 6 (t₀.₉₇₅ = 2.4469). Contrasts in pp; a label needs both
intervals to agree. With 3 seeds per stratum the bootstrap has at most 10 distinct resample means per stratum and is narrow;
the t-interval is the operative guard (review P3-3).

**Crash labels** (CAM-K2-E's gate-A rule, CAM-K2-E3's names): UNRESOLVED (intervals disagree on the side);
MATERIAL_REDUCTION (both upper ends < −5 pp); SMALL_REDUCTION (both upper ends < 0, mean ≤ −2); WORSE (both lower ends > 0,
mean ≥ +2); NEGLIGIBLE (both inside (−2, +2)); NO_DETECTED_CHANGE otherwise; a reduction with timeouts up ≥ 2 pp reads
CRASH_TO_TIMEOUT. **Capture and timeout labels** (C2's scheme, bands 3 pp and 2 pp): LOWER / SMALL_LOWER / HIGHER /
SMALL_HIGHER / NEGLIGIBLE / NO_DETECTED_CHANGE / UNRESOLVED.

| Gate | Contrast (stage A) | Verdict rule |
| --- | --- | --- |
| **L** | S12k − S1k | capture HIGHER and crash ∉ {WORSE, UNRESOLVED} → **LEARNING_SUPPORTED** ("the final checkpoint outperforms the earliest kept checkpoint in the final training environment"); capture LOWER or crash WORSE → **LEARNING_REFUTED**; otherwise **LEARNING_NOT_DETECTED** |
| **Y** | YC12k − S12k | `[yawclip]` telemetry missing in any YC12k cell → **YAWCLIP_INACTIVE**; pooled binding share < 1 % → **YAWCLIP_WEAKLY_BINDING** (no tolerance label); crash WORSE or capture LOWER → **YAWCLIP_SENSITIVE**; crash ∈ {NEGLIGIBLE, SMALL_REDUCTION, MATERIAL_REDUCTION} and capture ∈ {NEGLIGIBLE, SMALL_HIGHER, HIGHER, SMALL_LOWER} → **YAWCLIP_TOLERATED**; otherwise **YAWCLIP_UNRESOLVED**. **TOLERATED admits a capture loss whose intervals exclude 0 but whose mean is under 3 pp (SMALL_LOWER).** The binding share is reported next to the label, for YC12k and S12k. |

Reported with the same labels, not gated: S3k − S1k, S6k − S3k, S12k − S6k on the 175-bar blocks (3 blocks: per-block
differences, no interval) and on all 9 blocks with intervals if stage C completed; pooled and per-density rates per arm (the
205-bar rates of S12k are the policy's behaviour at a density it never trained at; the per-density difference 205 − 175 is
descriptive); crash per 1,000 steps (crashes / in-quota steps; steps from ledger `completion_step` differences per env);
crash split per arm (recorder classes; shares of crashes); capture kinematics (relative speed, closing, lateral: mean, p50,
p90, share > 1.0 m/s); blind behaviour (share of steps with no detection; hidden vs visible means of the raw and executed
speed and |yaw|; share of hidden steps executed above the blind cap); `[camsafe]` / `[yawclip]` last lines (labelled running
totals); governor telemetry; the delay distribution; every MISMATCH line.

**Controls** (computed 2026-11-24 on CPU on CAM-K1-E's cells at 115/160/205 bars × 4501–4503, manifest-bound, with the final
analysis script, SHA-256 `96f42e083cc8f9f3e4664fc67c59971ef8a0eed43d3310b06be71a46d7719357`, recorded in
`~/motar_tmp/cams2a_eval_20261124/controls_camk1e_9blocks_r3.json` (review r2 P2-12: the committed script must have this SHA);
any required miss reads CONTROL_FAIL and no gate label is given; review P2-5 added the rows at this unit's scale):

| Control | Episodes/cell | Required | Computed |
| --- | --- | --- | --- |
| crash rule on F66 − K1s1 (known bad) | 2,048 | WORSE | WORSE, +41.90 pp, bootstrap [+40.84, +42.82], t [+40.35, +43.45] |
| crash rule on K1s1 − K1s2 (null) | 2,048 | not MATERIAL_REDUCTION, not WORSE | NEGLIGIBLE, −1.07 pp; half-widths 0.60 / 0.93 pp |
| crash rule on K1s1 − F66 | 2,048 | MATERIAL_REDUCTION | MATERIAL_REDUCTION, −41.90 pp |
| capture scheme on K1s1 − F66 | 2,048 | HIGHER | HIGHER, +41.91 pp |
| crash rule on K1s1 − K1s2 (null at this unit's scale: first 8 episodes per env) | 1,024 | not MATERIAL_REDUCTION, not WORSE | UNRESOLVED, −1.20 pp, bootstrap [−2.03, −0.40], t [−2.47, +0.06]; **half-widths 0.81 / 1.26 pp = this unit's resolution** |
| crash rule on (K1s1 + 5 pp) − K1s2 at 1,024 | 1,024 | WORSE | WORSE, +3.80 pp, bootstrap [+2.98, +4.60], t [+2.53, +5.06] |
| capture scheme on (K1s1 − 5 pp) − K1s2 at 1,024 | 1,024 | LOWER | LOWER, −3.80 pp |
| crash **contrast** of +3.00 pp at 1,024 (K1s1 shifted by 3 pp − the null's mean −1.20 = +4.20 pp; reported, not required) | 1,024 | — | WORSE, +3.00 pp, bootstrap [+2.19, +3.80], t [+1.74, +4.26] |
| crash contrast of +2.00 pp at 1,024 (the floor case; shift +3.20 pp; reported) | 1,024 | — | WORSE at the floor (mean exactly 2.00; bootstrap [+1.15, +2.79], t [+0.74, +3.26]) |

Reading (review r2 P1-6): the quota-8 null's t half-width is 1.26 pp at df 6 (standard error ≈ 0.52 pp). **At 1,024 episodes
per cell a crash increase of about 3 pp reads WORSE** (≈ 0.97 under the measured null variance, INFERRED normal
approximation); **contrasts between 2 and 3 pp are not reliably labelled** (the +2.00 pp row sits on the floor); **under 2 pp
WORSE cannot be read**, by the label's definition. In the other direction NEGLIGIBLE needs both intervals inside ±2 pp, so the
observed mean must lie within about ±0.73 pp: **a true null yaw-clip effect reads NEGLIGIBLE with ≈ 0.84 probability and
UNRESOLVED or NO_DETECTED_CHANGE otherwise (about one chance in six), which gate Y then labels YAWCLIP_UNRESOLVED instead of
TOLERATED.** This is accepted rather than bought back with quota 16 (which would double the time and not shrink the seed-level
block variance). Limit: the controls are on another task and policy family; they test the rule and the interval machinery at 9
blocks and this episode count, not this unit's effect sizes.

**Bindings** (EVAL_BINDING_FAIL otherwise, checked before any number; `report_bindings` is a pure function with tests):
identity IDENTITY_PASS on the eval commit and the report the eval named; eval COMPLETE with stages A and B (33 cells) in the
planned order and every stage A and B record OK (stage C records are never judged, review r2 P0-3); the same runner environment and training receipt (at its SHA); the changed-path list inside
the diff allowlist; every cell passes the runner's checks again with its checkpoint SHA; complete 9-block grids for S1k, S12k,
YC12k and complete 175-bar blocks for S3k, S6k; in every block S12k ≠ S1k and YC12k ≠ S12k; `S12k_identity` = eval S12k,
`S1k_smoke` = eval S1k (review P3-4) and `YC12k_negative` = eval YC12k at 205 / 4501; recorder episode lengths equal the
ledger's; executed yaw never over the limit in any YC12k cell; the checkpoints at their pins; the analysis script's bytes
equal the eval commit's (and therefore the controls' recorded SHA).

**Stop rules and labels.** PREFLIGHT_FAIL (nothing runs), IDENTITY_FAIL (no eval), VOID_HOST_FAULT, INCOMPLETE (a FAIL cell
in A or B, or the cap before stage B is done), COMPLETE (stage C partial, stopped by budget or by a cut cell, allowed and
recorded); analysis EVAL_BINDING_FAIL, CONTROL_FAIL
or REPORTED with the gate labels above. No label is upgraded afterwards; a rerun writes to a new folder and needs an amendment.

## What may be said, and what may not

- **May be said** (if bound and the controls pass): "In simulation, in CAM-S2a's own training environment at fixed densities
  115 / 175 / 205 bars, the ep12000 checkpoint captures X % / crashes Y % / times out Z % (per density, 1,024 episodes per
  cell); against ep1000, which trained only at 70 bars, its capture rate differs by +A pp [CI; t CI] (gate L: label); with the
  yaw command clipped at 1.047 rad/s, binding on B % of control steps, the crash rate changes by C pp [CI; t CI] (gate Y:
  label); D % of crashes are target contacts; the mean relative speed at capture is E m/s (F % above 1.0 m/s); while the target
  is not detected the policy issues a mean speed of G m/s and |yaw| of H rad/s."
- **May not be said:** anything about real flight, the real OAK-D stream or the real detector (the detector here is the
  rendered mask plus the P9 placeholder); a comparison with Frozen Policy F, CAM-K1, CAM-K2 or CAM-S1 (different task and
  rules); that gate L measures "learning" in general (ep1000 is off its own training distribution: FOV curriculum, K window,
  70 bars; the contrast is the final checkpoint against an early one in the final environment); a causal reading of the yaw
  clip (one evaluation condition on one checkpoint); that YAWCLIP_TOLERATED means "no capture loss" (it admits a loss under
  3 pp); a crash increase under about 3 pp as detected; a NEGLIGIBLE label as excluding changes under 2 pp beyond the interval
  it reports; generalisation beyond these densities, this target motion
  and this seed; that the 205-bar numbers are a trained competence (the policy never trained above 175); any rate from
  training rollouts as an evaluation result; that the clip condition says anything about the flight runtime, which has no yaw
  clip or blind cap yet.

## Disclosed limits

- One training seed, one checkpoint per epoch; intervals are over evaluation blocks, not over training runs.
- 1,024 episodes per cell; resolution as measured above (null half-widths 0.81 / 1.26 pp; a +3 pp contrast reads WORSE, 2–3 pp
  not reliably, under 2 pp never; a true null clip effect reads UNRESOLVED about one time in six).
- S1k is evaluated outside its own training distribution (review P2-4); gate L is a sanity gate.
- The legacy target's bar screening and the placement queries remain TF32-exposed `cdist` paths, as in training.
- The capture rule's one-step-stale detection, radial-only closing bound, hybrid previous action and the contact-over-capture
  priority are evaluated as trained (training review F5, F7, F1, F9); the recorder reports the consequences, it does not change
  them.
- `crash_class = contact` cannot separate a bar contact from a PhysX contact with the target mesh (the task's cause 0 is one
  bucket); `target_contact_swept` is the wrapper's 0.40 m conversion and **also absorbs a same-step bar contact** (the task lets
  the swept capture beat the crash before cams2 converts it; review P3-2).
- The hidden/visible split uses the perception flag of the observation the policy acted on; a detection the P9 injector
  dropped counts as hidden, which is the policy's view.
- The `[yawclip]` and `[cams2diag]` lines are running totals through their call count (every 320 calls), not final lines;
  the per-episode rows are the exact record (review P2-10).
- The KROC2027 motion recorder is off: its contracts were built for the bounded target executor and a legacy-target violation
  would stop cells; nothing in the metrics needs it. Camera FOV and import origins come from the realveh receipt instead.
- Module seeds change the vehicle, depth, P9, delay and gain draws; the distributions are training's, the draws are not.
- Stage C is optional; without it the S3k / S6k trajectory is three 175-bar blocks without an interval.

## Review

- **Design review r1 of the draft** (Claude Opus 5.5, fresh context, read-only): **GO_WITH_FIXES**,
  `~/motar_tmp/cams2a_eval_20261124/review_r1/REVIEW_r1.md`. Fixed in this r2: P1-1 recorder-OFF identity cell and passivity
  test; P1-2 whole-cell clipped share, per-episode binding counts in both arms, 1 % minimum binding share, TOLERATED wording;
  P1-3 diff allowlist in the preflight; P1-4 MISMATCH scan with an allowlist; P1-5 the staged plan above, per-arm rate
  projection, realistic timing; P2-1 entry env_navrl comparison and value checks on every allow-listed key; P2-2 pinned
  simulator/interpreter, GPU4GB dropped; P2-3 machine-checked condition; P2-4 gate L wording and S1k disclosure; P2-5 the
  1,024-episode null and the shifted controls; P2-6 `identity_verdict`, `eval_verdict`, `report_bindings` factored and tested;
  P2-7 budget in both loops, per-cell timeout = remaining budget; P2-8 161 keys, base `fef6800`; P2-9 import guards (skips
  with reasons; never a pass); P2-10 telemetry every 320 calls, labelled snapshots; P3-1 exact over-cap counts; P3-2 same-step
  contact disclosure; P3-3 bootstrap granularity; P3-4 `S1k_smoke` bound to eval S1k; P3-5 the committed training review cited,
  deterministic player stated. Not changed: the KROC2027 recorder stays off and quota stays 8 (the review's recommendations).
- **Design review r2** (delta, Claude Opus 5.5): **GO_WITH_FIXES**, `~/motar_tmp/cams2a_eval_20261124/review_r2/REVIEW_r2.md`.
  Fixed in this r3: P0-1 `d8_dynamic_mesh_treatment` expected as the bool the task writes plus the contract's `mode`, tested
  against a real CAM-K2-E3 `result.json`; P0-2 executed-yaw count with a 1e-5 tolerance, a per-cell runner check for clipped
  cells (zero executed over the limit, at least one requested), and a float32 integration test cams2 → yawclip → cams2diag;
  P0-3 stage C starts only with ≥ 1.10 × the ep12000 identity mean remaining, and the verdict and every binding are judged on
  stage A and B records only (tests: a cut C cell leaves COMPLETE and the bindings true); P1-6 the resolution reading above
  (+3 pp / +2 pp contrasts instead of shifts, the one-in-six UNRESOLVED disclosure); P2-11 the amendment rule for an
  allowlist refusal; P2-12 the controls regenerated with the final script and its SHA recorded here and in the JSON; P2-13
  as P0-2; P3-6 `mock.patch.dict` for the one test that sets environment variables. P3-7: the MISMATCH allowlist stands.
- After the files are placed in the tree: the full PUBLIC_CPU suite, then the approval request. No further full review is
  needed (review r2).
- **CPU tests of the drafts** (isolation, cores 6,7,12,13, no CUDA, 2026-11-24, `~/motar_tmp/cams2a_eval_20261124/draft/tests_draft_r3.log`):
  `test_yawclip` 11, `test_cams2diag` 11, `test_cams2a_eval` 27, all 49 OK, none skipped on this host.
