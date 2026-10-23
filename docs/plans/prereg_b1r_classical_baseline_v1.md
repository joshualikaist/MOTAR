# B1-R — Classical sensor-based baselines vs frozen Policy F (preregistration v1)

**Status: preregistration.** This file binds once it is committed at
`docs/plans/prereg_b1r_classical_baseline_v1.md`, and it is immutable after that. Any change goes in a
new dated amendment file.
- Every GPU stage still needs the user's separate approval for that run.
- G5 HOLD is unaffected.
- Branch `codex/mt-p1`, base HEAD `fde21cd`, 2026-10-23.
- It supersedes the untracked draft `docs/plans/b1r_classical_baseline_prereg_DRAFT_2026-10-23.md`
  (SHA-256 `3886be89…f50d`), which bound nothing.

The machine-readable copy of the frozen values is the JSON block in §12, and the runner reads it from
this file. If the prose and the block disagree, the block wins, and the disagreement is recorded as a
defect.

## 1. Purpose and claim boundary

MOTAR has no matched classical baseline that controls whole episodes under Policy F's contract
(`docs/EVIDENCE.md`: Class A = 0). The H-R3 scripted arms cannot fill that gap: they controlled only
2-s windows after the first loss, at 160 bars and one seed. This study adds two sensor-based
classical controllers and evaluates them under the same task path as Policy F.

- **Baseline value.** It tells us whether F's learned behaviour beats simple classical guidance with
  local LiDAR avoidance, under an identical simulator, sensor and governor contract.
- **No novelty claim.**
  - Comparisons of learned policies against proportional navigation or pure pursuit are an
    established type of experiment.
  - It is also known that PN alone does not avoid obstacles.
  - The draft cited specific papers. Those citations are not verified and are dropped here. The
    literature positioning is checked separately before any publication.
- **No other claim.** Nothing here is a real-flight claim, a physical-interception claim, a safety
  guarantee or a state-of-the-art claim. "Capture" means the simulator's swept closest approach
  below 0.5 m (§4).

## 2. Arms (3)

| Arm | Executed action | Online inputs |
| --- | --- | --- |
| **F** (`F_SHADOW`) | Frozen Policy F, deterministic, SHA-256 `f7022139…3ad40` | F's 898-D sensor-only actor observation |
| **B-PN** | Velocity-direction proportional navigation plus the shared LiDAR filter | §2.1 |
| **B-PP** | Pure pursuit (course = line of sight) plus the same LiDAR filter | §2.1 |

**How every arm runs.**
- The player computes F's deterministic action first. The hook then computes the baseline action from
  the online inputs.
- The arm decides only which of the two actions reaches the task:
  - **B arms** execute the baseline action.
  - **F** executes its own action unchanged. Its baseline is computed in shadow and discarded, using
    the fixed shadow configuration B-PN with N = 5 and v_nom = 2.5.
- F's forward pass consumes random numbers even in deterministic mode (`navrl_players.py:212-215`).
  Running it in every arm therefore keeps compute and random-number use the same *per step*.
- Across steps the arms are not paired. Once the actions differ, episode lengths and reset times
  differ, and the random streams diverge after the first arm-dependent reset (the H-R3
  replay-coupling finding).
- The F cells also test that the hook does not interfere with the task (gate G0, §6).

### 2.1 Online inputs (the complete list)

| Input | Source in the running task |
| --- | --- |
| Own position `p`, velocity `v` (world) and yaw-only vehicle quaternion | `obs_dict["robot_position"]`, `["robot_linvel"]`, `["robot_vehicle_orientation"]`. This is the simulator's exact robot state, the same source F's proprioceptive observation uses; there is no estimator noise in either arm. |
| Tracker mean `[p̂_T, v̂_T]` (world) and active flag | `perception.tracker.state` and `perception.tracker.active`. The tracker is the constant-velocity Kalman tracker, which expires after `tracker_memory_s` = 5.0 s. |
| Raw LiDAR ranges, 4 × 72 at 12 m | `task._lidar_distance_m()`. Bearings come from `perception._lidar_angles`; elevations are (20, 10, 0, −10)°. |
| Sensor-associated target-return mask, 4 × 72 | `perception.last_target_like` (`navrl_perception.py:1889`). This is the camera detection–LiDAR association mask that the riskcap governor uses for target exclusion. |

All of these are read between steps, after `perception.observe`. That is the same post-step state from
which F's observation for that step was built.

**Never an input to B-PN or B-PP:**
- target ground truth (`target_position`, `target_vel_w`);
- renderer masks and semantic IDs;
- the obstacle layout;
- the critic `states`;
- E-L0, occlusion-trace or recorder fields.

**How this is enforced.** CPU tests (§9) check three things:
- `motar_eval/classical_baseline.py` imports only `math` and `torch`;
- the hook reads only the attributes in the table;
- neither file names a ground-truth field.

### 2.2 Guidance laws (planar; the pursuer is held at the 1.0 m flight altitude)

All quantities are in the world XY plane. Let `r = p̂_T − p` and `ṙ = v̂_T − v`.

- **LOS angle:** `λ = atan2(r_y, r_x)`.
- **LOS rate:** `λ̇ = (r_x ṙ_y − r_y ṙ_x) / max(|r|², 1e−6)`.
- **B-PN:**
  - If `|v_xy| < 0.2 m/s`, the course is `ψ_c = λ`.
  - Otherwise `ψ_c = ψ_v + clamp(N · λ̇, −3.0, 3.0) · Δt`, with `ψ_v = atan2(v_y, v_x)` and Δt = 0.1 s.
  - The ±3.0 rad/s saturation of the commanded turn rate equals the yaw-rate limit. It stops
    tracker-velocity noise from flipping the course near capture.
  - PN integrates from the *measured* velocity course. The velocity loop lags, which lowers the
    effective gain; this is why the grid spans N ∈ {3, 5, 8}.
- **B-PP:** `ψ_c = λ`.
- **Nominal velocity:** `v_nom · [cos ψ_c, sin ψ_c]`, rotated into the yaw-only vehicle frame.

### 2.3 Shared components (identical in B-PN and B-PP)

**Target-return exclusion** (new relative to H-R3).
- The LiDAR ray-casts the target's 0.20 m proxy sphere (`_sync_target_to_sensor`;
  `aerial_gym/config/sensor_config/lidar_config/navrl_lidar_config.py:84`).
- Without exclusion, the filter below removes the approach component close to the target. A CPU
  prototype showed that a 2.0 m/s approach to a target return 1.0 m ahead falls to 0.125 m/s. That
  would prevent capture by construction.
- Before filtering, two kinds of return are therefore set to "far":
  - (a) bins flagged by `last_target_like`, which is the governor's own exclusion;
  - (b) while the tracker is active, returns whose horizontal point lies within 0.5 m of the belief
    position. 0.5 m is H-R3's `belief_exclusion_m`.

**LiDAR approach filter.**
- This is H-R3's sequential safety filter (`motar_eval/h_r3_arms.py:85-93`). The norm clamp at lines
  94-95 is not used.
- It acts on the remaining horizontal points. For each of the 5 nearest points, it removes the
  approach component with weight `clamp((r_s − r)/(r_s − 0.5), 0, 1)`.
- **Speed-scaled onset radius.** H-R3 designed this filter for 1.0 m/s, with r_s = 1.5 m. Here
  `r_s(v_nom) = max(1.5, 0.5 + 0.1·v_nom + v_nom²/(2·2.9608856678))`: the full-removal distance
  0.5 m, plus the stopping distance under the governor's own brake and reaction parameters.
  - This gives 1.5 m at v_nom 1.5 and 2.0, 1.805 m at 2.5, and 2.320 m at 3.0.
  - At v_nom ≤ 2.0 the filter is exactly H-R3's.
- **Disclosed property.** All four elevation rows hit a full-height bar at almost the same
  horizontal point. The "5 nearest points" are therefore often one bar seen four or five times, and
  the attenuation compounds. For example, at r = r_s − 0.25·(r_s − 0.5), four rows leave about 32 %
  of the approach component. This property is part of the H-R3 reference and is kept unchanged.

**Speed bound.**
- The bound preserves direction: `s = max(1, max(|v_x|, |v_y|)/2.5)`, then `v ← v/s` and
  `action_xy = v/2.5 ∈ [−1, 1]`.
- This is the same per-axis action bound F has: 2.5 m/s per axis, and a horizontal request norm up
  to 3.5355 m/s.
- **Disclosed asymmetry.** The yaw law points body x at the belief, so flight along the line of
  sight is bounded at 2.5 m/s. F instead requests a mean of 3.216–3.246 m/s across the 56 OCC-P2
  moving cells, by flying with a lateral component (`result.json` `speed_governor`,
  `LEGACY_STEP_WINDOW`).
  - `v_nom` = 3.0 exceeds 2.5 only off the body x axis.
  - A speed-matched "crabbing" classical variant is out of scope.

**Other components.**
- **z action:** 0. The altitude PI overwrites it, but it is persisted in `prev_action`.
- **Yaw law:** yaw rate = `clamp(2.0 · e, −3.0, 3.0)`, where `e` is the belief bearing in the vehicle
  frame; `action[3]` = yaw rate / 3.0. This is H-R3's law, and it keeps the camera on the target.
- **No belief (tracker inactive):** the action is `[0, 0, 0, 0.2]`, a 0.6 rad/s counter-clockwise
  scan with no translation. 0.6 rad/s is the KROC B1 search-supervisor rate.
- **Stale belief** (camera lost, tracker still active): the tracker's own prediction is used until it
  expires after 5.0 s.

The controllers are pure, deterministic functions of the current inputs:
- no random numbers;
- no internal state;
- no matrix multiplication or `cdist`. Clearances are explicit difference norms, because of the TF32
  hazard.

## 3. Execution path and injection point (resolves O2)

All three arms run the OCC-P2 moving-A3 task path unchanged, in this order:
1. `tools/kroc2027_eval.cell_env(..., quota, view="visible", recorder=True, arm="A3")`, with the OCC-P2
   moving `extra_env` and `telemetry_env` added.
2. The byte-identical `eval_navrl_target_behavior_arms.sh`, then `play_navrl.sh`, then
   `runner.py --play`.
3. The rl_games `BasePlayer.run` loop.
4. `task.step`. This holds the riskcap governor, the altitude PI, the Lee velocity controller,
   `prev_action` feedback, the KROC recorder and the episode ledger.

Riskcap executes `min(requested, cap)`, with cap ∈ [2.0, 3.5355] m/s (`speed_governor.py:465-475,
497`). A request below 2.0 m/s is never changed. A B arm with `v_nom ≤ 2.0` is therefore governed only
by its own filter.

**Injection point (O2 decision).** The action is replaced at the player boundary.
- The runner adds `tools/b1r_site` to `PYTHONPATH` for B1-R cells only, after `base_env` has set
  `PYTHONPATH` to the worktree. That directory holds a `sitecustomize.py`.
- Without `MOTAR_B1R_ARM`, that file does nothing.
- With `MOTAR_B1R_ARM` set:
  - It loads `tools/b1r_hook.py` by file path. `tools/` is never put on `sys.path`.
  - It registers a post-import patch for exactly one module,
    `aerial_gym.rl_training.rl_games.navrl_players`.
- At start-up it imports only standard-library modules, never torch or `aerial_gym`. So
  `runner.py`'s rule of importing Isaac Gym before torch still holds.
  - This includes the module level of `tools/b1r_hook.py`, which every process loads.
  - `torch` and `motar_eval.classical_baseline` are imported lazily, inside the patched
    `get_action`, after `runner.py` has imported Isaac Gym.
- Python 3.8's `site.py` swallows exceptions raised in `sitecustomize`. To fail closed instead, an
  install error ends the process with `os._exit(3)`.
- **What the patch does.** It wraps `NavRLPpoPlayerContinuous.get_action`. The original method runs
  first; the hook then returns the baseline action or F's action (§2).
- rl_games applies no post-processing between `get_action` and `env_step`. The clamp and rescale
  inside `get_action` are the identity for the Box[−1, 1] action space.
- The `aerialgym` environment has no `sitecustomize` module of its own, so nothing is shadowed.
- No change is made to `runner.py`, the launchers or the task.

**Why the quota ledger and recorder stay intact.** All of the following live inside `task.step`:
- the episode ledger (`navrl_episode_ledger.py`) and its `SystemExit(0)` at quota completion
  (`navrl_task.py:5148-5159`);
- the KROC recorder hooks (`before_action`, `after_physics`, `on_episode_end`, `finalize`);
- the governor.

The hook changes only the tensor that is passed into `task.step`. The recorder's requested-speed
telemetry therefore measures the executed arm's own request (`kroc_eval_adapter.py:1181`).

**Rejected alternatives:**
- An H-R3/E-L0-style custom player loop. It bypasses `BasePlayer.run`, so it would not reproduce the
  OCC-P2 contract by construction.
- The task-side `_kroc_search` slot. Using it needs a production edit.
- Any edit to `kroc_eval_adapter.py`, which is byte-pinned, `navrl_task.py` or `speed_governor.py`.
  All are forbidden.

**Process and output identity:**
- The launcher also starts short-lived Python helpers: the nonce, timestamp and receipt writers, and
  the import probe. They load `sitecustomize` too, but they never import `navrl_players`.
- Only the process that patches and calls `get_action` writes the sidecar `b1r_hook.json`, next to
  `result.json`, at exit. The sidecar records:
  - its pid and argv;
  - the `sitecustomize` origin;
  - the import origins;
  - the SHA-256 of the hook files.
- Every cell gets a fresh directory `<stage>/<arm-or-configuration>/<cell_id>`, and the runner refuses
  an existing one. The launcher's own SKIP path (`eval_navrl_target_behavior_arms.sh:159`) is
  therefore never reached.

**Environment identity:**
- The hook variables are deliberately not prefixed `NAVRL_`.
- A cell's `env_snapshot_navrl` must equal its OCC-P2 template except for these keys:
  - `NAVRL_BULK_EVAL_JSON` (per-cell path);
  - `NAVRL_EVAL_RUN_NONCE`;
  - `NAVRL_KROC2027_RUN_ID`, which must be `B1R-V1`. It is a context label only
    (`kroc_eval_adapter.py:785`).
- For tuning cells only, two more keys may differ:
  - `NAVRL_SEED`, which must be `4390`;
  - `NAVRL_EPISODE_QUOTA_PER_ENV`, which must be `2`, as written by
    `cell_env` (`tools/kroc2027_eval.py:126-127`).
- The template is:
  - for evaluation cells, the OCC-P2 cell of the same density and seed;
  - for tuning cells, the OCC-P2 cell of the same density at seed 4301.
- Any other difference, including any new `NAVRL_*` key, is a G2 failure.
- `NAVRL_KROC2027_ARM` stays `A3` in every arm, so raw recorder records in the B arms also say `A3`.
  The B1-R arm is recorded in the sidecar, in the directory name and in the runner's receipts.

## 4. Evaluation contract (copied from OCC-P2 moving A3)

The contract source is the OCC-P2 A3 receipt; for example,
`results/occlusion_confirmatory_v1/OCC-P2/moving/A3/E2_obstacle_aware__160bars__seed4301/receipt.json`.
The following must be identical across arms.

**Episodes.**
- `NAVRL_EPISODE_LEN_STEPS=600` and `NAVRL_EPISODE_QUOTA_PER_ENV=4`, with 128 environments: 512
  quota episodes per cell.
- The estimand is the first 4 completed episodes of every environment. Later episodes form the
  non-primary tail and are never used.

**Initial view and placement.** Visible, at 10.0 m, `PLACEMENT_MODE navrl_band`.

**Target.** `e2_obstacle_aware`, waypoint pattern, bounded dynamics, `direct_l2_v1` backend, speed
U[0.3, 1.5] m/s.
- `ON_VIOLATION=stop` is kept.
- B-arm cells may run up to about 5× more target-motion steps than F cells. A violation stop is a G2
  failure; OCC-P2 recorded 0 violations in 112 cells.

**Limits.** `MAX_VELOCITY 2.5`, `YAW_RATE_MAX 3.0`, `ALT_HOLD_VMAX 2.5`, `MAX_TILT_DEG 45`.

**Sensing.** LiDAR 72×4 at 12 m; perception and vision on.

**Governor.** Riskcap, with FIXED 2.0 / FREE 3.53553390593 / SLOW 3.0 / RELEASE 5.0 /
HALF_WIDTH 0.45 / MARGIN 0.45.

**Termination** (`navrl_task.py:6100-6124`).
- **Capture:** the swept closest approach is under 0.5 m.
- **Crash:** contact force above 0.05 N, a height breach or an arena exit.
- **Timeout:** 600 actions.
- **Precedence.** Capture is tested after crash on the same step and wins: `crashed_out & ~captured`.
  A captured episode therefore had no crash on any *earlier* step. A contact on the capture step
  itself is not excluded, and it is not recorded per episode. "Collision-free capture" is therefore
  `NOT_RECORDED`, and the endpoints below say "capture".

**Densities.** 70 / 115 / 160 / 205 bars.

**Evaluation seeds.** 4301–4307, the first seven OCC-P2 seeds.
- F's values on these seeds are known in advance (unblinded). The seed-level primary mean is 0.9533,
  from the OCC-P2 `episode_ledger_summary.json` quota counts.
- The B arms are evaluated blind: their parameters are frozen before any evaluation cell.

**Cells.** 3 arms × 4 densities × 7 seeds = **84**.

## 5. Baseline parameter selection (before evaluation, separate seed)

**Grids.**
- **B-PN:** `N ∈ {3, 5, 8}` × `v_nom ∈ {1.5, 2.0, 2.5, 3.0}` m/s, giving 12 configurations.
- **B-PP:** `v_nom ∈ {1.5, 2.0, 2.5, 3.0}`, giving 4 configurations.
- `r_s` follows `v_nom` (§2.3), so it adds no axis to the grid.
- Changes from the draft:
  - The draft used N ∈ {3, 4, 5}. The new range compensates for velocity-loop lag (§2.2).
  - The draft had no 1.5 m/s. A slower setting is the obvious way for a reactive controller to trade
    speed for safety.

**Tuning seed: 4390.** O1 was resolved on 2026-10-23.
- A structured census parsed 14,626 JSON/JSONL files across `results/`, `.codex_worktrees/`, `docs/`
  and `rl_games/runs/`. It found no integer 4390 under any seed-named key, and no file or directory
  name containing it.
- A text grep of the raw store and `git log --all -G` were also empty.
- The runner repeats the census in the Stage B preflight.

**Tuning cells.**
- Densities 115 and 205, quota 2 (256 episodes per cell).
- 16 configurations × 2 densities = 32 tuning cells.

**Scene disjointness.**
- After Stage B, the scene hashes in the tuning cells' `occlusion_layouts.npz` must not overlap those
  of OCC-P2 seeds 4301–4307.
- Any overlap means `STOP_TUNING_CONTAMINATED`: record it, start no Stage C, and handle it only by a
  dated amendment.

**Selection rule (per controller).**
- **Metric:** the largest total quota capture count over the two tuning cells, which is equivalent to
  the mean capture rate.
- **Ties** (equal integer counts) are broken in this order:
  1. lower total crash count;
  2. lower `v_nom`;
  3. lower `N`.
- `tools/b1r_run.py select` writes `selection.json`.
- Before any Stage C cell starts, the SHA-256 of `selection.json` and the G1 verdict are committed in
  a docs-only evidence record.
- There is no re-tuning after evaluation.

## 6. Integrity and stop gates

### G0 — reproduction and non-interference

**Rule.** Applied to all 28 F (`F_SHADOW`) cells:
- `kroc2027_eval.historical_match(new, OCC-P2 same cell, quota=4, fields)` returns `MATCH`.
- The fields are those of `docs/plans/occlusion_pilot_v2.json` `g0_noninterference.ledger_fields`:
  episode_id, outcome, capture, crash, timeout, completion_step, completion_rank and
  min_relative_distance_m.
- Every recorder field present in both cells must match exactly, and neither cell may have recorder
  fields the other lacks (`keys_only_new` and `keys_only_old` both empty). `kroc_eval_adapter.py`
  is byte-pinned, so any new or missing field is a contract change.
- The quota counts must be equal.
- The precedent is OCC-G0_rerun1, which matched exactly across commits.

**On failure.** The verdict is `VOID_CONTRACT_MISMATCH`. Stop; no tuning or B cell starts.
- One attribution run is declared in advance: the first failing cell, with plain F and no hook, on
  the same HEAD. It is diagnostic only and needs its own approval.
- Continuing needs a dated amendment.

### G1 — baseline defect signatures (tuning only)

**Scope.** Each controller's selected configuration. D1–D3 are tested *per tuning cell*, and a stop
follows if either cell (115 or 205 bars) exceeds a limit. D0 pools the two cells.

**Signatures.** Any one of these means `STOP_BASELINE_DEFECT`:
- **D0:** zero captures in both cells.
- **D1:** perception sanity. More than 5 % of quota episodes have `ever_track_active` = false.
  - With the visible start, the tracker activates on the first observation, before any B action.
    F shows 0 such episodes out of 28,672.
  - D1 is therefore essentially controller-independent. D3 is the controller's real no-acquisition
    signature.
- **D2:** in-view stand-off. More than 10 % of quota episodes meet all three of these conditions:
  - the episode is a timeout;
  - `swept_min_distance_m` < 1.5 m, where `None` does not count;
  - `obs_camera_valid / observations` ≥ 0.5.

  The camera condition separates treating the target as an obstacle, which happens in view, from
  being blocked by bars near the target, which mostly happens under occlusion. F has 3 timeouts
  within 1.5 m out of 28,672 quota episodes, with or without the camera condition.
- **D3:** belief starvation. No-belief steps exceed 50 % of the sidecar's env-steps
  (`PROCESS_WINDOW`).

**If no signature is present**, Stage C proceeds whatever the capture rate. A weak baseline is
reported as weak.

**On failure.** Stage C does not start, and the tuning results are recorded as they are. A change
needs a dated amendment with a diagnosis, followed by re-tuning on a fresh census-clean seed.

**Why there is no performance floor.** The draft's floor of 0.80 would, at any level, push the
baseline to be re-engineered until it came close to F. Defect signatures detect a broken controller
without conditioning on how good it is.

### G2 — runtime (every cell)

**Rule.** The cell passes only if all of the following hold:
- The launcher exits 0.
- `kroc2027_eval.check_cell(..., expect=cell_expect)` reports no problem.
- `occlusion_trace_problems` and `mt_identity_problems` are empty.
- The environment identity of §3 holds.
- The sidecar shows all of the following:
  - it exists and was written by the patched process;
  - `patch_applied` is true;
  - its arm and parameters equal the cell's;
  - more than 0 `get_action` calls;
  - 0 non-finite actions, 0 actions outside [−1, 1] before the clamp, and 0 non-zero z values;
  - `sitecustomize` and the import origins lie inside the worktree.
- Source HEAD, the tracked tree and the checkpoint SHA are unchanged.

**On failure.** Stop at the first failing cell, record it, keep its partial files, and do not retry or
salvage.

### G3 — resources

**Rule.**
- **Preflight free disk:** at least 8 GB plus 1.5 × the stage's projected worst-case output.
  - The projection uses 48 kB per task step, the OCC-P2 maximum: A 1.0 GB, B 1.9 GB, C-PN 3.3 GB and
    C-PP 3.3 GB.
  - The ×1.5 margin is there because `recovery_events.jsonl` (about 45 % of an F cell) grows with
    loss events, which may be more frequent in the B arms.
  - The requirement is therefore 9.5 / 10.9 / 13.0 / 13.0 GB.
- **During a stage:** free disk ≥ 8 GB.
- **Wall caps:** 900 s per cell. Per stage: A 2 h, B 3.5 h, C-PN 5.5 h, C-PP 5.5 h.
- **Measured free space on 2026-10-23:** 4.4 GB. Every stage is refused until space is freed.
- **Before each cell:** free disk ≥ 8 GB + 1.5 × 2400 × 48 kB, about 8.2 GB.

**On failure.** Stop.

### Stopped stages

- A stopped stage is never analysed, in part or in whole.
- With the user's approval it can be rerun from scratch into a new attempt directory. The stopped
  attempt is kept and reported, and no cell is ever resumed.

### Stage order

Each stage needs its own approval. Each preflight requires that the previous gate passed.

| Stage | Content |
| --- | --- |
| **A** | G0: F, 28 cells |
| **B** | Tuning: 32 cells, then `select`, G1 and scene disjointness, then the docs-only selection commit |
| **C-PN** | B-PN evaluation, 28 cells |
| **C-PP** | B-PP evaluation, 28 cells |

The runtime and gate paths must be unchanged between stages: `aerial_gym`, `motar_eval`,
`resources/robots`, `tools/b1r_hook.py`, `tools/b1r_site`, `tools/b1r_run.py`,
`tools/b1r_analysis.py` and `tools/kroc2027_eval.py`.

## 7. Endpoints (O3 resolved: every field below is recorded per cell today)

**Data rules.**
- Rates use in-quota rows only: `episodes.jsonl` with `in_quota = true`.
- Per-episode fields come from `kroc2027_episodes.jsonl`, joined on (`env_index`,
  `env_episode_index`).
- Aggregates that are not quota-based carry a window label.

**Primary**
- Capture rate per arm. The unit is the evaluation seed:
  `P(arm, seed) = (1/4) Σ_density capture_count / 512`.

**Secondary** (descriptive, per arm and density, with seed-level summaries)

| Endpoint | Definition and source |
| --- | --- |
| Crash and timeout rates; per-density capture and crash | Ledger quota counts. |
| Time to capture | `actions` × 0.1 s on captured quota episodes. Also RMTC60, the restricted mean time to capture over all quota episodes, where a non-captured episode counts as 60 s. This is not a censoring RMST. |
| Requested, executed and actual speed | Per-episode means `pursuer_requested_speed_mean_mps` (clamped action × 2.5), `pursuer_commanded_speed_mean_mps` (the governed command, i.e. executed) and `pursuer_actual_speed_mean_mps`; also `steps_commanded_over_2p5`. |
| Governor intervention | `result.json` `speed_governor.intervention_rate` and `mean_scale`. Label `LEGACY_STEP_WINDOW`: every env-step of all 128 environments up to `cutoff_step`, the step at which pooled completions first reached 512, including episodes still in progress. |
| Stopping margin at contact | `result.json` `speed_governor.contact.mean_stopping_margin_{requested,executed}_m`. `LEGACY_STEP_WINDOW`, contact steps only. There is no per-episode record. |
| Camera detection fraction | Σ `obs_camera_valid` / Σ `observations` over quota episodes. Also `obs_track_active` and `track_losses`. |
| Hook telemetry | Sidecar counts, all `PROCESS_WINDOW` (the whole process, non-quota tail included): no-belief steps, excluded returns (a) and (b), filter-active steps, PN-saturated steps, slow-course steps. Also a course-rate diagnostic: mean realised \|Δψ_v\|/Δt against mean commanded \|clamp(N·λ̇)\| over consecutive active, non-slow B-PN steps (approximate across resets). |

**Exploratory**
- Crash hazard by tracking state (lost, after reacquisition, steady), using the R01 method on the
  occlusion frame trace.

**Anything not listed here is `NOT_RECORDED`.** No proxy is substituted for it.

## 8. Analysis

**When the analysis runs.** It runs once, after A, B, C-PN and C-PP have all completed with G0, G1 and
G2 passed. The C1–C2 family needs both B arms. No contrast is computed from a single arm's stage.

**Contrasts on the primary endpoint**
- C1: F − B-PN
- C2: F − B-PP
- C3: B-PN − B-PP

**Inference**
- The analysis is seed-paired, with n = 7.
- **Exact two-sided sign-flip test** over all 2⁷ = 128 sign vectors. The statistic is `|mean|`, and a
  vector counts when `|mean| ≥ observed − 1e−12`. The minimum attainable p is 2/128 = 0.015625.
- **BCa 95 % CI** over seeds:
  - 20,000 resamples, `numpy.random.default_rng(20261023)`;
  - z0 uses the share of resamples below the estimate plus half the share equal to it;
  - jackknife acceleration;
  - a bootstrap with zero spread gives `NOT_ESTIMABLE`;
  - BCa under-covers at n = 7. It is used only as a conjunctive condition, so it adds no type-I risk.
- **Holm** over C1 and C2 only, at α = 0.05.
  - The first step needs p ≤ 0.025, which is attainable only as 2/128, with all seven seed
    differences on one side.
  - The second step admits 2/128, 4/128 or 6/128.
- **C3** is outside the family. It is reported with its unadjusted p and is descriptive only.
- Per-density contrasts are exploratory and uncorrected.

**Pre-declared reading**

Each of C1 and C2 gets exactly one of three readings, and together they cover every outcome:

| Reading | Rule |
| --- | --- |
| `SUPERIOR` | mean > 0, Holm p < 0.05 and BCa lower bound > 0 |
| `INFERIOR` | mean < 0, Holm p < 0.05 and BCa upper bound < 0 |
| `NOT_SHOWN` | every other case, including `NOT_ESTIMABLE` |

The readings translate into statements as follows:
- **"F outperforms both classical sensor-based baselines under this contract"** requires C1 =
  `SUPERIOR` and C2 = `SUPERIOR`.
- **C1 = `NOT_SHOWN`:** "it is not shown that F beats B-PN". This is reported as a primary limitation
  of the system paper.
- **C1 = `INFERIOR`:** "B-PN outperforms F under this contract". This is a primary limitation, reported
  as such.
- **C2** is read the same way for B-PP.
- **C3** is descriptive only.

No result is described as a new method, a safety guarantee or a physical-interception capability.

## 9. Implementation scope (after this file is committed)

**New files only**
- `motar_eval/classical_baseline.py`: pure functions for guidance, exclusion, filter, bound and yaw.
  No simulator imports.
- `tools/b1r_hook.py`: the post-import patch, the online-input reader and the sidecar.
- `tools/b1r_site/sitecustomize.py`: the opt-in loader. Standard library only; fails closed.
- `tools/b1r_run.py`: the census, the preflights, stages A, B, C-PN and C-PP, `select`, the gates and
  the manifests.
- `tools/b1r_analysis.py`: the analysis of §7 and §8.
- `tests/test_b1r_classical_baseline.py`.

**CPU tests**
- **Actions:** action bounds, z = 0, and direction preservation.
- **Input firewall:** import AST, the attribute allow-list, and banned ground-truth names.
- **Guidance:**
  - LOS-rate sign and magnitude;
  - B-PP equals B-PN when `λ̇` = 0 and the course is on the LOS;
  - PN saturation;
  - the slow-course fallback.
- **Belief handling:** the no-belief scan and determinism.
- **Filter:**
  - at r_s = 1.5 it equals the `h_r3_arms` reference;
  - the `r_s(v_nom)` values;
  - exclusions (a) and (b), and that (b) needs an active track.
- **Hook**, in a child process only:
  - it is inert without the variable;
  - `F_SHADOW` returns F's action bit for bit;
  - B arms return the baseline;
  - the sidecar fields;
  - no torch, `aerial_gym` or `classical_baseline` import at start-up;
  - fail-closed exit.
- **Runner and analysis:**
  - the §12 block parses;
  - the stage matrices;
  - the environment-identity check, including the tuning template. The tuning environment is built
    through `cell_env` and compared against the real seed-4301 template receipt;
  - the G0 comparator on real OCC-P2 cells;
  - selection and its tie-breaks;
  - the G1 signatures;
  - the census;
  - exact sign-flip p-values, Holm, BCa and the readings.
- **The full PUBLIC_CPU suite** stays at 0 failures and 0 errors.

**Unchanged:** all production and task files, the governor, checkpoints, `results/` and existing
preregistrations.

## 10. Cost estimate

**Time model.**
- OCC-P2 moving cells fit cell time ≈ 10.6 s + 0.258 s × task step (56 cells; maximum residual 13 s).
- A cell lasts until its slowest environment completes its quota.
- With per-episode B timeout rate p, the chance that some environment times out all four episodes is
  `1 − (1 − p⁴)^128`. That is 0.012 at p = 0.1 and 0.65 at p = 0.3.
- Worst case: 2400 steps at quota 4 (about 630 s) and 1200 steps at quota 2 (about 320 s).

**Disk model.** About 48 kB per task step.

| Stage | Cells | Expected | Worst case | Projected output (worst) |
| --- | --- | --- | --- | --- |
| A: G0 F | 28 | about 1.0 h, as OCC-P2 | 1.6 h | 1.0 GB |
| B: tuning | 32 at quota 2 | 1.5–2.5 h | 2.8 h | 1.9 GB |
| C-PN | 28 | 2.2 h (p ≤ 0.1) to 4.6 h (p = 0.3) | 4.9 h | 3.3 GB |
| C-PP | 28 | 2.2 h to 4.6 h | 4.9 h | 3.3 GB |
| **Total** | 116 | **about 7–13 GPU-h** | 14.2 h | 9.5 GB |

Engineering time (implementation, CPU tests, review) is separate: about 1–2 working days.

## 11. Open items

| Item | Status |
| --- | --- |
| O1 | Resolved: seed 4390 is unused (§5). |
| O2 | Resolved: the player-boundary hook (§3). |
| O3 | Resolved: the field list in §7, with window labels. |
| O4 | Resolved: the Opus 5.5 independent review (§13). |
| O5 | Whether the result must land before the 2026-10-28 KROC Simulation Freeze. This is the user's decision and does not change any frozen value. |
| Disk | 4.4 GB free on 2026-10-23, against a requirement of ≥ 9.5 GB for Stage A and ≥ 13.0 GB for each C stage. Space must be freed (user action) before any GPU stage. |

## 12. Machine-readable frozen values

```json
{
  "schema": "motar.b1r-prereg.v1",
  "run_id": "B1R-V1",
  "output_root": "/home/fair/workspaces/aerial_gym_ws/src/aerial_gym_simulator/results/b1r_classical_baseline_v1",
  "contract_source": {
    "plan": "docs/plans/prereg_occlusion_confirmatory_v1.json",
    "group": "moving",
    "kroc_arm_label": "A3",
    "occ_p2_root": "/home/fair/workspaces/aerial_gym_ws/src/aerial_gym_simulator/results/occlusion_confirmatory_v1/OCC-P2/moving/A3",
    "env_keys_allowed_to_differ": {"NAVRL_BULK_EVAL_JSON": "per-cell path", "NAVRL_EVAL_RUN_NONCE": "any",
                                   "NAVRL_KROC2027_RUN_ID": "B1R-V1"},
    "tuning_env_extra": {"NAVRL_SEED": "4390", "NAVRL_EPISODE_QUOTA_PER_ENV": "2"},
    "tuning_template_seed": 4301
  },
  "policy_f_sha256": "f702213936601860995cf61dcc570247e72543b1976e3716055cd8ec5593ad40",
  "num_envs": 128,
  "evaluation": {"seeds": [4301, 4302, 4303, 4304, 4305, 4306, 4307], "densities": [70, 115, 160, 205], "quota": 4,
                 "arms": ["F_SHADOW", "B-PN", "B-PP"], "cells": 84},
  "tuning": {"seed": 4390, "densities": [115, 205], "quota": 2,
             "grid": {"B-PN": {"N": [3, 5, 8], "v_nom": [1.5, 2.0, 2.5, 3.0]}, "B-PP": {"v_nom": [1.5, 2.0, 2.5, 3.0]}},
             "cells": 32, "selection": ["max_total_capture", "min_total_crash", "min_v_nom", "min_N"]},
  "shadow_config": {"arm": "B-PN", "N": 5, "v_nom": 2.5},
  "controller": {"dt_s": 0.1, "max_velocity": 2.5, "yaw_rate_max": 3.0, "lidar_far_m": 12.0,
                 "lidar_elevations_deg": [20.0, 10.0, 0.0, -10.0], "lidar_hbeams": 72,
                 "slow_course_speed_mps": 0.2, "pn_turn_rate_max_radps": 3.0, "los_rate_eps_m2": 1e-6,
                 "yaw_gain_per_s": 2.0, "scan_yaw_rate_radps": 0.6, "no_belief_action": [0.0, 0.0, 0.0, 0.2],
                 "tracker_memory_s": 5.0,
                 "safety_radius_m": "max(1.5, 0.5 + reaction_s*v_nom + v_nom^2/(2*brake_mps2))",
                 "safety_radius_floor_m": 1.5, "reaction_s": 0.1, "brake_mps2": 2.9608856678,
                 "safety_full_m": 0.5, "safety_points": 5,
                 "exclusion": {"a_target_like_mask": true, "b_belief_radius_m": 0.5, "b_requires_active_track": true},
                 "speed_bound": "direction-preserving: v /= max(1, max(|vx|,|vy|)/max_velocity)",
                 "z_action": 0.0},
  "gates": {"G0": {"comparator": "kroc2027_eval.historical_match", "quota": 4,
                   "ledger_fields_source": "docs/plans/occlusion_pilot_v2.json#g0_noninterference.ledger_fields",
                   "require_empty": ["keys_only_new", "keys_only_old"],
                   "also": "equal quota counts; environment identity"},
            "G1": {"scope": "D1-D3 per tuning cell, either cell exceeding stops; D0 pooled over both cells",
                   "D0_zero_capture_both_cells": true, "D1_max_share_never_track_active": 0.05,
                   "D2_max_share_timeout_with_swept_min_below_m": [0.10, 1.5], "D2_min_camera_valid_share": 0.5,
                   "D3_max_share_no_belief_steps": 0.50},
            "min_free_disk_gb": 8.0, "bytes_per_task_step": 48000, "projection_margin": 1.5,
            "projected_output_gb": {"A": 1.0, "B": 1.9, "C-PN": 3.3, "C-PP": 3.3},
            "preflight_free_disk_gb": {"A": 9.5, "B": 10.9, "C-PN": 13.0, "C-PP": 13.0},
            "cell_cap_s": 900, "stage_caps_h": {"A": 2.0, "B": 3.5, "C-PN": 5.5, "C-PP": 5.5}},
  "stages": ["A", "B", "C-PN", "C-PP"],
  "runtime_paths": ["aerial_gym", "motar_eval", "resources/robots", "tools/b1r_hook.py", "tools/b1r_site",
                    "tools/b1r_run.py", "tools/b1r_analysis.py", "tools/kroc2027_eval.py"],
  "analysis": {"primary": "per seed: mean over densities of quota capture_count/512",
               "runs_after": ["A", "B", "C-PN", "C-PP"],
               "family": [["F_SHADOW", "B-PN"], ["F_SHADOW", "B-PP"]], "descriptive": [["B-PN", "B-PP"]],
               "alpha": 0.05, "multiplicity": "holm", "test": "exact_sign_flip_two_sided_abs_mean",
               "tie_tolerance": 1e-12, "ci": "bca_95", "bootstrap_resamples": 20000, "bootstrap_seed": 20261023,
               "readings": {"SUPERIOR": "mean>0 & p_holm<alpha & bca_low>0",
                            "INFERIOR": "mean<0 & p_holm<alpha & bca_high<0", "NOT_SHOWN": "otherwise"},
               "rmtc_horizon_s": 60.0}
}
```

## 13. Independent review (O4)

**Reviewer.** Opus 5.5, read-only, 2026-10-23, on the pre-review candidate.

**Verdict.** `PASS_WITH_CHANGES`: no P0, four P1 findings.

**What the reviewer confirmed:**
- the injection point;
- that the post-step inputs are the same state F's observation was built from;
- that rl_games does not post-process the action;
- the riskcap law;
- F's seed mean of 0.9533 and the 6987/7168 count;
- the time fit;
- every §7 field;
- that the runtime diff since `5ff285a` is opt-in only.

**Disposition**

| # | Finding | Disposition in this version |
| --- | --- | --- |
| P1-1 | Allowed environment differences were incomplete (run id; tuning template) | Adopted (§3, §12) |
| P1-2 | "Capture implies collision-free capture" was false (`navrl_task.py:6124`) | Adopted: the claim is removed, and collision-free is `NOT_RECORDED` (§4) |
| P1-3 | The readings were not exhaustive and overlapped | Adopted: a three-way rule per contrast (§8) |
| P1-4 | Free disk is 4.4 GB, and output was not projected | Adopted: projected outputs, a preflight rule and the open item (§6 G3, §11) |
| P2-5 | The G0 comparator was weaker than the OCC-G0 precedent | Adopted: `historical_match` plus all recorder fields, and a pre-declared attribution run |
| P2-6 | Baseline fairness: the filter was sized for 1 m/s, PN lag, the speed asymmetry | Adopted: speed-scaled r_s, N ∈ {3, 5, 8}, the course-rate diagnostic, asymmetry disclosed |
| P2-7 | Telemetry windows were mislabelled | Adopted: `LEGACY_STEP_WINDOW` and `PROCESS_WINDOW` |
| P2-8 | The Stage C time was underestimated | Adopted: a p-conditional estimate, C split by arm, stopped stages never analysed |
| P2-9 | The G1 floor was arbitrary | Adopted: defect signatures D0–D3 instead of a floor |
| P2-10 | The sitecustomize mechanism needed explicit constraints | Adopted (§3, §9 tests) |
| P3-11 | Statistics details | Adopted: Holm over C1 and C2 only, tie rules stated |
| P3-12 | The JSON block lacked values; line references were wrong | Adopted |
| P3-13 | Miscellaneous (RMST name, own state, disjointness consequence, violation exposure, A3 label, citations) | Adopted. The citations are dropped (§1). |

**Delta review.** Opus 5.5 reviewed the revised candidate on 2026-10-23. Verdict: `PASS_WITH_CHANGES`.

**What it confirmed:**
- the P1-1 to P1-4 fixes;
- the r_s values, and that the taper stays under the braking envelope;
- the G0 field list;
- the meaning of the D1 and D2 fields;
- the disk and time arithmetic.

**Disposition**

| # | Finding | Disposition |
| --- | --- | --- |
| P1 | The tuning identity rejected every tuning cell, because quota 2 ≠ 4 | Adopted: `NAVRL_EPISODE_QUOTA_PER_ENV` = 2 allowed for tuning (§3, §12), plus a CPU test against the real template |
| P2 | The G1 scope was ambiguous | Adopted: per cell, either cell stops; D0 pooled |
| P2 | D2 could stop on legitimate clutter blocking | Adopted: the camera-valid share ≥ 0.5 condition |
| P2 | The gate logic was not frozen between stages | Adopted: `tools/b1r_run.py` and `tools/b1r_analysis.py` added to the frozen paths |
| P3 | D1 is controller-independent | Adopted: relabelled a perception sanity check |
| P3 | G0 MATCH ignored missing recorder keys | Adopted: both key lists must be empty |
| P3 | The hook's module-level imports | Adopted (§3) and tested |
| P3 | The disk projection used F's rate | Adopted: ×1.5 margin and a per-cell check |
| P3 | When the analysis runs | Adopted (§8) |
