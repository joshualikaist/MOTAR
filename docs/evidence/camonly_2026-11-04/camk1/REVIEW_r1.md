# Review r1: S3c/S4 runner repair and CAM-K1 v1 (fresh context, read-only)

Reviewer: Claude Opus 5.5. Worktree MOTAR-camonly, camonly/dev, HEAD 0d44972, uncommitted diff. Probes are in `probes/`. CPU only.

## Part 1: S3c/S4 runner repair. Verdict: PASS

What I checked:
- (a) `remove_dataman` runs in the outer `finally` of `run_episode` (s3b_formal:306, s4:132), after `with OwnedSession` has exited. By then `session.json` has been written (`__exit__`:838).
- It unlinks only `ep_dir/rootfs/dataman`. `is_file()` plus `unlink()` would remove a symlink, never its target. The test confirms that `parameters.bson` survives.
- The result is recorded per episode as `dataman_removed_bytes`.
- Nothing hashes or cites dataman:
  - `session.json` lists launches, cleanup and the watchdog;
  - `runtime_binding` hashes build, rc, plugin and model files only;
  - no `rglob` walks `ep_dir`;
  - no prereg mentions it.
- S3b's `ep127/rootfs/dataman` is 7,866,640 bytes. PX4 keeps its mission and fence storage there, not flight data. Deleting it is acceptable as evidence handling.
- (b) The S4 check now allows only lines ending in `/usr/share/rustdesk/rustdesk`. That is stricter than `gr_run.ALLOWED_GPU_APPS` (substring `rustdesk`). The test is non-vacuous: none gives [], rustdesk alone gives [], rustdesk plus a job gives exactly one problem that names the job.
- Tests: `tests/test_sitl_s3c_s4.py` ran 19 tests in 145 s, OK. pytest is not installed, so I ran the file directly.

Findings:
- **MINOR P1-1.** The committed S3c/S4 preregs (0d44972) say "fresh-rootfs session" but do not say that rootfs/dataman is deleted afterwards. Fix: one line in each unit's execution record or README.
- **LOW P1-2.** If `__exit__` raises CleanupError (PX4 leaked), the removal runs while PX4 may still hold the file open. That is harmless, but the space is freed only when PX4 exits, and "after the session ended" holds only on a clean exit.
- **LOW P1-3.** Already in the code before this repair: when nvidia-smi fails, prints nothing to stdout and returns nonzero, the GPU check passes. Fix: check `returncode`.

## Part 2: CAM-K1 v1. Verdict: PASS_WITH_LIMITS for the code and the prereg; the chain script is FAIL as written (K-1)

Verified:
1. **The override reaches the task.**
   - The registry stores the `task_config` class by reference (`task/__init__`:122-131), and `BaseTask` keeps that reference.
   - `self.vis_cfg = task_config.vision` (1413).
   - `self.rw` is copied from `reward_parameters` inside `__init__` (1273-1274), after the entry has applied the override.
   - The detector (321), perception (754), the FOV curriculum (2457, 4799) and the kroc adapter (796) all read the FOV at construction or at run time.
   - No copy is made at import. `OBSTACLE_EFFECTIVE_FOV_DEG` is the separate `NAVRL_OBSTACLE_FOV_DEG` token selector.
   - The obstacle-depth ray table uses the same FOV (detector:391).
   - The FOV curriculum is saturated, because `num_task_steps` is restored from the checkpoint (4597).
   - Limit: my CPU identity probe (`probes/identity_probe.py`) could not finish, because `position_setpoint_task_sim2real_end_to_end_config` allocates a CUDA tensor at import. These conclusions rest on reading the code, not on a run.
2. **The ablation behaves in training as it did in the C-study evaluation.**
   - v2_search exports `NAVRL_PERCEPTION_PERTURB=0`, so `observe(training=False)` takes the evaluation path.
   - Resets re-observe through the wrapped `observe`.
   - The governor wrapper is installed on the class, so it applies during rollouts.
   - The critic states are the ablated observation plus GT extras.
   - `r_ss` reads the simulator's LiDAR and segmentation, in the reward only.
   - The raw-LiDAR actor paths (6365, 6407) belong to non-perception modes; in those, the governor wrapper raises (fail-closed).
   - `_visible_now` is fused visibility, which under `no_lidar` is camera-only.
3. **The launcher keeps GR-1 ctrl's contract.**
   - I diffed the export/unset lines in both directions. Only the declared lines differ: PATH, SEED, MAX_EPOCHS, the ablation, the four CAMTRAIN values, the receipt, REAL_PYTHON, PYTHON, the tag and the session log.
   - PYTHON is called once, for `runner.py` (train_navrl.sh:85).
   - `tools/` contains no executable that shadows a command.
   - `pipefail` is set in train_navrl.sh, so runner.py's exit code survives `tee`.
   - Preflight reruns: seed 1 and `CAMK1_SMOKE=1` both print PREFLIGHT PASS; the smoke shows 25001..25002.
   - `runs/`, `train_session_logs/` and `__pycache__` are gitignored. The log globs for the smoke and the seeds cannot match each other's files.
   - Expected wall time: GR-1 took 65-70 min per 1,000 epochs, so a seed should take about 3.3-3.5 h, inside the 5 h cap.
4. **Tests:** `tests/test_camk1_train.py` ran 13 tests, OK.

Findings:
- **MAJOR K-1. The chain never runs the smoke.**
  - `camk1_train.py 1` refuses with PREFLIGHT_FAIL unless `train_smoke/train_report.json` reads SMOKE_OK at the same HEAD. The chain then logs "chain stops" and the night is lost.
  - The smoke cannot run while SITL runs: the preflight refuses on gzserver or px4.
  - Fix: add `camk1_train.py 1 --smoke` to the chain after the settle sleep, and stop if its rc is not 0. Or run the smoke before starting `run_s3c_s4.sh`, and have the chain check `smoke_ok` at start and exit at once.
- **MINOR K-2. One commit has to carry everything, and nothing may change afterwards.**
  - The preflight refuses on any untracked file: `tree_problems` runs `git status --porcelain` without `--untracked-files=no`.
  - The approval commit and the smoke commit must both equal HEAD.
  - The tree now has 7 untracked and 5 modified paths. They include another session's S3b evidence and `EXPERIMENTS.md`.
  - So: commit everything as one HEAD before the SITL chain starts, have the S3c, S4 and CAM-K1 approvals all name that HEAD, and commit or create nothing until seed 2 has finished.
- **MINOR K-3. The receipt only checks itself.** `overrides_after` reads back the attributes that the entry has just set. Add one end-to-end check: require "hfov=66deg" in the session log (from the warning at navrl_task:1502), or `cfg_reward_parameters.yaw_align_weight == 0.6` in the checkpoint's environment state.
- **MINOR K-4. The smoke does not require `last_gen_ppo_ep_25002_*.pth`.** rl_games' final save at `max_epochs` is exactly the path the 3.5 h seeds depend on. Require it.
- **MINOR K-5. The loiter rationale needs a different argument.** "600 × 0.03 = 18 < 30" is undiscounted. The real argument: the bonus stays below the per-step time cost `alive_weight = −0.05`. Loitering in view therefore still nets −0.02 per step (F: −0.03), and at γ = 0.99 the bonus stream is worth at most 3. The conclusion holds; rewrite the reasoning before the commit.
- **MINOR K-6. The prereg is missing disclosures:**
  - (a) Under `no_lidar` the visibility bonus becomes camera-only. F's 0.02 was paid on 360° camera-or-LiDAR visibility, so this is a change in meaning, not only "1.5×".
  - (b) Out-of-FOV commands get clearance 0, which caps them at the 2.0 m/s floor. Narrowing 87° to 66° widens the capped sector.
  - (c) F's critic starts on states that have lost the LiDAR.
  - (d) 160×90 pixels over 66°×54° gives fx ≈ 123 px/rad (it was 84.3) and fy ≈ 88: non-square pixels and finer horizontal resolution. The 0.30 m target spans about 1.85 px at 20 m, against 1.27 px before, so detection range improves. This works against calling the run "harder sensing".
  - (e) Wording: "Doubling it pushes the motion into the camera's view" should read "is intended to push". "Which is why there are two seeds" implies that two seeds establish an effect; they do not.
- **MINOR K-7. Records.**
  - The prereg is not yet in any `docs/EXPERIMENTS.md` family, so `test_experiment_index` will fail at commit.
  - The user's decisions of 2026-11-06 (no LiDAR on the real vehicle, the 66×54 FOV, a larger in-view reward) are not in `DECISIONS.md`. D-2026-11-04-3 covers C3 training, but not these choices.
- **MINOR K-8. The chain trains after a host fault.** It also proceeds when the SITL chain ends with "HOST FAULT … chain stops" (exit 7). Fix: stop on that line, or record the fault count when the chain starts and require it unchanged before each seed.
- **LOW K-9. The deadline bounds only the wait.**
  - Seed 2 starts whatever the hour. If SITL ends around 01:30, it finishes around 08:30.
  - `date -d "tomorrow 07:00"` is evaluated when the chain starts, so a start after midnight means 07:00 a day later.
- **LOW K-10. The wrapper passes the caller's whole environment.** A stray `NAVRL_*` variable (for example `NAVRL_PREFLIGHT_ONLY`, `NAVRL_CAMERA_WIDTH` or `NAVRL_DETECTOR_MAX_RANGE`) would change the run silently. Strip `NAVRL_*` the way `gr_run.cell_env` does.
- **LOW K-11. Disk.** 4.8 GB is free (96 % used). Each seed's run folder will be about 0.55 GB (GR-1: 182 MB per 1,000 epochs). That fits, but nothing checks free space.
- **LOW K-12. The contract test runs in one direction only.** An undeclared extra export would pass. The two-way diff I ran is clean.

Fix before arming the chain: K-1 (and K-2 operationally). Fix before committing the prereg: K-5, K-6, K-7. K-3, K-4 and K-8 are cheap and protect the overnight run.
