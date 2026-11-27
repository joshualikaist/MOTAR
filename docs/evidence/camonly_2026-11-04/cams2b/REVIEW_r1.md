# CAM-S2b training package — independent review r1

Reviewer: Claude Opus 5.5 (fresh context, read-only). Date: 2026-11-24.
Worktree `/home/fair/workspaces/aerial_gym_ws/src/MOTAR-camonly-s2b`, branch `camonly/s2b`, base 3a453e6. The files are uncommitted.
Reviewed: `docs/plans/prereg_camonly_s2b_2026-11-24.md`, `tools/cams2b_train.py`,
`aerial_gym/rl_training/rl_games/train_navrl_cams2b.sh`, `tools/cams2b_python`, `tests/test_cams2b_train.py`.
No repository file was edited. No GPU was used, and no simulator or training was started.

## Verdict: PASS_WITH_LIMITS

No P0 or P1 finding. Two things I checked independently, beyond reading the code:

- **End-to-end environment check, with the real launchers.** I copied `train_navrl_cams2b.sh`, `train_navrl_v2_search.sh`, `train_navrl.sh` and `tools/cams2b_python` unchanged into a scratch tree. Only `tools/realveh_python` was replaced, by a stub that saves the environment and arguments. The chain ran on CPU cores 6, 7, 12 and 13.
  - Calibration: the CAM-S2a launcher, run the same way, reproduced CAM-S2a's `receipt.json` `env_navrl` exactly. The only differences were the two path keys (`NAVRL_P9_ERROR_MODEL` and `NAVRL_REALVEH_RECEIPT`).
  - S2b-SCR differs from CAM-S2a only in `NAVRL_YAW_RATE_MAX` (1.0472), `NAVRL_DENSITY_CHECK_EPS` (16) and the two path keys.
  - S2b-FT additionally differs in `NAVRL_V2_ALLOW_RESUME=1`, `NAVRL_LEARNING_RATE=7.5e-06`, `NAVRL_RESET_DENSITY_WINDOW=1` and `NAVRL_DENSITY_RESUME_WARMUP=1000`.
  - `receipt_problems()` returned `[]` for both runs, so the receipt check will not stop a correct run.
  - Runner arguments for FT: `... --train --seed 1 --max_epochs 15000 --disable_collapse_early_stop --checkpoint <ckpt> --branch_run`.
  - The wrapper's line `[cams2b-python] NAVRL_YAW_RATE_MAX 3.0 -> 1.0472 rad/s ...` reaches the tee'd log.
- **Checkpoint metadata**, read on CPU with scalars only:
  - epoch 12000; num_task_steps 384000; n_bars_active 175; density_level_start_steps 378130.
  - density_succ_agg/fin_agg 2010/3256; k_min/k_max 20.0/28.0.
  - cfg_yaw_rate_max 3.0; cfg_density_check_eps 16384; cfg_ppo_horizon 32.
  - current_action_learning_rate 7.5e-06; rollback total 2, streak 0.
  - cfg_speed_governor_yaw_cap_radps 0.0; cfg_training_seed 1.
  - Critic keys carry `_orig_mod.`, so `runner.py` would write `_rlnorm.pth` next to the checkpoint it loads. The copy into `~/motar_tmp/cams2b_20261124/source/...` is justified.
- The CPU tests: `tests/test_cams2b_train.py` ran 28 tests, all OK.

## Findings

| ID | Sev | Finding (evidence) | Suggested fix |
| --- | --- | --- | --- |
| F1 | P2 | **The FT wall cap of 4 h is thin.** The preregistration's "about 3.3 h at 3.93 s/epoch" uses CAM-S2a's whole-run average, which includes the faster low-density stages. From CAM-S2a's logger timestamps (`launcher.log`): 160 bars ran at 4.01 s/epoch (7,324 s / 1,828 epochs), 145 bars at 4.10, and 175 bars at 4.05 (epochs 11,816→11,990: 704 s / 174 epochs). Time per epoch rose with density from 85 to 145 bars (3.69→4.10). FT runs 3,000 epochs at 175, 190 and 205 bars: about 3.4–3.6 h plus startup and compile, against a 4.0 h cap (`cams2b_train.py:85`). That leaves about 10–15 % headroom, and a timeout labels a correct run WALL_CAP. SCR: about 13.5–14 h against 17 h, which is adequate. | Before the run, raise the FT cap to 4.5 h in the preregistration and driver. Alternatively, record the measured s/epoch from the FT smoke and state the margin. Correct the 3.3 h estimate. |
| F2 | P2 | **The watcher thread has no exception guard** (`cams2b_train.py:438-475`). Any exception kills it silently, for example an OSError writing `progress.json` on a nearly full disk, or a JSON or regex error. The stop rule and the run-time receipt/start checks are then never applied while the run continues. After the run, the receipt and start checks are recomputed (`:484-490`), but the **stop rule is not**. A run that the preregistered rule should have stopped at epoch 13,000 (or 3,000) could then be labelled TRAINED. | Wrap the loop body in try/except, record the error in the report, and keep polling. After the run, if the run is not a smoke, `state["checked"]` is False and blocks reached `stop_check_epoch`, evaluate `stop_rule` on the blocks up to that epoch. Record the result, and make the verdict at least TRAINING_FAIL or a flagged STOPPED_NO_LEARNING. |
| F3 | P2 | **Comparison 1 in the preregistration is confounded** (`prereg:181-182`). "S2b-FT final vs CAM-S2a ep12000 + 1.047 clip" tests whether adapting to the ceiling beats clipping. But FT also gets 3,000 more PPO epochs and the only exposure to 190/205 bars. The preregistration notes the density range (`:187`) but does not limit the claim. | In "Not claimed" (or the evaluation preregistration), state that a FT advantage cannot be attributed to the yaw adaptation alone. Alternatively, propose a control: CAM-S2a continued for 3,000 epochs at 3.0 rad/s with the same schedule, which needs user approval. Use the identical 1.0472 for the clip, not 1.047, for exact parity. |
| F4 | P3 | **The run-folder lookup trusts `AERIAL_RUN_NAME`.** The dashboard `run :` line comes from `os.environ["AERIAL_RUN_NAME"]` (`pretty_train_stats.py:86`), which `runner.py:1027` sets with `setdefault`. The driver does not strip `AERIAL_RUN_NAME`, `AERIAL_RUN_TITLE` or `AERIAL_TASK_CONFIG_MODULE` (`STRIP_NAMES`, `cams2b_train.py:102`). A stale value in the launching shell would make `RUN_NAME` point to the wrong folder. No final checkpoint would be found, and a correct run would be labelled TRAINING_FAIL. The predecessor driver has the same gap. | Add the `AERIAL_RUN_NAME`, `AERIAL_RUN_TITLE`, `AERIAL_TASK_CONFIG_MODULE` and `AERIAL_REWARD_DESIGN_MAX` names (or the `AERIAL_` prefix minus nothing needed) to the strip list. |
| F5 | P3 | **The stop window lags slightly.** The rule is applied at the first 30 s poll with epoch ≥ `stop_check_epoch`, over the last 200 blocks *at that moment* (`:469-474`). That is about 7 epochs per poll, so the window can be epochs ~12,808–13,007 rather than 12,801–13,000. For FT it can then include a few 190-bar epochs (promotion is at ~13,001). CAM-S2a had the same behaviour (check at 3,001). | Slice the blocks to `epoch <= stop_check_epoch` before `window_summary`. |
| F6 | P3 | **Wording in the preregistration's arithmetic** (`:117-121`). "At least 4096/600 = 6.83 episodes finish per epoch … 16 episodes take at most ceil(16/6.83) = 3 epochs" is a long-run average, not a strict bound. Synchronised resets can delay completions. The empirical evidence is sound: I recomputed episodes per epoch over 1,001–12,000 from CAM-S2a's log as min 6, median 20, mean 19.66. 16 episodes always accumulated within ≤ 2 epochs, and no epoch after 1,000 had zero captures. The driver comment "17.8" (`:72`) is 17.7 by my count. | Say "on average", and cite the empirical maximum (2 epochs) as the evidence. |
| F7 | P3 | **The consumer list is incomplete** (`prereg:47-49`). The list of outside consumers that must use 1.0472 for a CAM-S2b policy omits `tools/b1r_hook.py:116`, `tools/h_r3_p03_launcher.py:97` and `tools/sitl_g9_lee_steps.py:29`. The last one does `setdefault("NAVRL_YAW_RATE_MAX", "3.0")`, so it would silently run a S2b policy at 3.0. Training is not affected. | Add these to the list, or say "for example". |
| F8 | P3 | **"PX4 `MPC_YAWRAUTO_MAX`"** (`prereg:20`, launcher and driver comments) is the author's attribution. It is not verified against the vehicle's parameter file, and offboard yaw-rate limiting in PX4 1.14 may use a different parameter. The 60 °/s value itself is the user's instruction. | Cite 60 °/s as the user's instruction, and mark the parameter name as NOT_VERIFIED unless a parameter dump confirms it. |
| F9 | P3 | **No code-identity check.** The receipt check compares only `env_navrl`. The receipt also carries SHA-256 hashes of the training modules (navrl_task, lee_controller_config_navrl, perception, robot config/URDF, …). I verified by `git diff 22ed16f 3a453e6` that the training path is unchanged since CAM-S2a, except the opt-in additions to `tools/realveh_entry.py`, which are inert when off. A run-time check would make "silently differs from CAM-S2a" impossible. | Optionally compare `receipt["sha256"]` with CAM-S2a's, excepting `realveh_entry`. |
| F10 | P3 | **The driver process is not pinned to cores.** Only the child is pinned with `taskset` (`:420`). The driver's watcher re-parses the full log every 30 s, and SCR's log grows to about 18–20 MB with the 16-episode checks. It can run on cores 4–5 (host MCE hazard). The predecessor driver has the same gap. | Call `os.sched_setaffinity(0, {6,7,12,13})` (or launch it under taskset) and say so in the operations note. |
| F11 | P3 | **Tests.** `Preflight._patched` patches `D.shutil.disk_usage`, which is the global `shutil` module while the test runs. It is restored in `finally`, so there is no leak, but it is shared state. The start-check fixtures are synthetic. The real log format, with ANSI colour codes, was checked by me against `camk1_20261106/train_s1/launcher.log:98-100`: the substrings match. | Optionally, add a fixture with real coloured lines. Patch a driver-local helper instead of `shutil`. |

## Requirement status

- **R1 — PASS.**
  - Simulation: the environment is CAM-S2a's plus exactly the listed differences. The run-time receipt check enforces this.
  - Every `NAVRL_YAW_RATE_MAX` site in the preregistration is correct at 3a453e6:
    - launcher pin `v2_search.sh:380`
    - config `navrl_task_config.py:483`
    - action scale `navrl_task.py:5006-5008`
    - controller clamp `lee_controller_config_navrl.py:7,51` and `base_lee_controller.py:153-155`
    - observation `:6472-6473` (perception) and `:6403` (the `_process_obs_vision` path, unused)
    - previous action `:5053`/`:5055-5057`
    - rewards `:6058-6060`
    - governor `speed_governor.py:71,84,330-336`, `navrl_task.py:5350-5354` (yaw cap 0.0 in the checkpoint)
    - arc clearance `:5317-5321`
    - provenance `:3097`, `:4285`
    - runtime consumers `camera_policy_runtime.py:1292,1319` (at HEAD)
  - The FT differences are needed or justified. `RESET_DENSITY_WINDOW` is redundant with the mismatch-triggered reset but harmless.
  - Code: the training path is unchanged since CAM-S2a (only opt-in eval modules were added).
  - Limit: the outside-consumer list is incomplete (F7).
- **R2 — PASS.**
  - `set_env_state` raises only on robot lineage (`:3753-3774`) and on routed contracts, which are gated on route enabled (`:3934-4104`). The yaw cap goes through the warn-only float loop (`:4282-4502`).
  - All four quoted messages match the format strings and the checkpoint values exactly:
    - `running with 1.047`
    - `16384.000 … 16.000`
    - `discarded=2010/3256 … resume_warmup=1000 … gate_not_before_step=416000` (384000 + 1000×32)
    - `task_steps=384000 bars=70->175 k_min=20.0 k_max=28.0 density_window=0/16 eps`
  - The start-check substrings match the real log line format (CAM-K1 warm start).
- **R3 — PASS_WITH_LIMITS** (F1, F2).
  - The mechanism is `--checkpoint/--branch_run` with `NAVRL_V2_ALLOW_RESUME=1` and an absolute 15,000. The search launcher accepts exactly that tuple. The epoch is restored, so the first block is 12001 (CAM-K1 precedent: 25001).
  - Curricula keyed on the restored num_task_steps are saturated (FOV 3,000, target speed 300, k_min 2,000+3,000, k window restored 20..28).
  - Density: `warmup_steps = max(1000×32, 416000)`. The first check after step 416,000 promotes 175→190, because the dwell is (416000−378130)/32 = 1,183 ≥ 1,000. 190 dwells 1,000 epochs, then 205. Stages are about 1,000 / 1,000 / 1,000.
  - `CHECK_EPS=16` does not bind: 16 episodes accumulated within ≤ 2 epochs in CAM-S2a, measured.
  - LR 7.5e-6 is justified. It is what `resolve_action_learning_rate` would choose on resume without an override, and the search launcher always passes 3e-5 (`:421`). KL stop and rollback are as CAM-S2a.
  - The stop rule (13,000, last 200, < 0.30) and the labels are implemented. The guard collapse exits nonzero, so TRAINING_FAIL.
- **R4 — PASS.**
  - From scratch, seed 1, 12,000 epochs, yaw 1.0472, `CHECK_EPS=16`.
  - The expected promotions are 9 rows ending at 205: 70→85 at ~1,001, then +15 per ~1,000-epoch dwell, so 205 at ~9,001.
  - Every 1,000th checkpoint and the last are hashed separately.
  - The stop rule is 3,000 / 200 / 0.05, as CAM-S2a.
- **R5 — PASS_WITH_LIMITS** (F2, F4, F10).
  - `--run ft|scr` and `--smoke`; unit folder as specified.
  - Preflight: branch, preregistration at HEAD, guarded paths, CUDA with RustDesk on an exact-path allow-list, disk (estimate + 1 GB), receipt SHA, FT source SHA (checked three times: source, copy, launcher), no existing run folder.
  - Host-fault count before and after; cores 0-3,8-11,14-23; caps 4 h and 17 h.
  - `run_report` has the verdict.
  - The evaluation-only wrappers are kept OFF by three layers: driver strip, launcher and wrapper refusal, and the receipt install-record check.
- **R6 — PASS.** 28/28 tests pass. Negative tests have positive controls. os.environ is never written; subprocesses get explicit dicts. The launcher test stubs the search launcher; my scratch simulation covered the full chain (F11 minor).
- **R7 — PASS_WITH_LIMITS** (F3, F8). The comparisons are stated: FT vs CAM-S2a with a 60 °/s clip; 45 °/s first as a 0.785 rad/s clip of FT, which `navrl_yawclip` allows since 0.785 < 1.0472; SCR vs FT. The deviation table matches the simulated differences exactly, and the authorization is quoted. The confound in comparison 1 is not stated (F3).

## Operational notes (not findings)

- Before launch, preflight will refuse until `tools/` has no uncommitted or untracked files. Those include the eval files of the other work stream (`tools/cams2b_eval_*.py`, modified `tools/camera_policy_*.py`) if they reach `camonly/dev` uncommitted. `MOTAR-camonly` was clean at 3a453e6 when reviewed.
- The run must be launched from `MOTAR-camonly` (`camonly/dev`). The P9 path key will then equal CAM-S2a's exactly.
- Disk: 2.4 GB free now. Projected use is about 0.15 GB for FT and about 0.5 GB for SCR (CAM-S2a's run folder was 459 MB), so the estimates hold.
