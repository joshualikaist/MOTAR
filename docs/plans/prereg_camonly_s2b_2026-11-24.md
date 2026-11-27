# Preregistration: CAM-S2b — CAM-S2a's camera-only training with the real vehicle's 60 deg/s yaw-rate ceiling (training only)

Written 2026-11-24 ~19:00 KST, before any CAM-S2b run. Built on branch `camonly/s2b` (base 3a453e6), to be
fast-forwarded onto `camonly/dev` before launch. Family `camera-only`. Predecessor: CAM-S2a
(`docs/plans/prereg_camonly_s2a_2026-11-23.md` + `_amendment1.md`; review
`docs/evidence/camonly_2026-11-04/cams2a/REVIEW_r1.md`, PASS_WITH_LIMITS).

## Authorization (the user's words, 2026-11-24)

- "CAM-S2b 회전 상한은 60 °/s로 해 주세요. 가능하면 처음 비행용으로 45 °/s 버전도 검토해 주세요."
- Sequence approved: "위 순서대로 진행하고, 평가 나오면 그때 판정하는거로 하자." The sequence is: after the running
  CAM-S2a evaluation ends, (1) S2b-FT (warm start, about 3 h), then a short evaluation of it (its own preregistration),
  then (2) S2b-SCR (from scratch, 12,000 epochs) overnight.
- Each run below is approved as one development smoke (repeatable until one completes; development only, not a result)
  followed by ONE run. Nothing else is approved by this document: the evaluations, a 45 deg/s training run, a second
  seed, push and merge each need the user.

## Question and scope

The real vehicle limits its yaw rate to 60 deg/s (PX4 `MPC_YAWRAUTO_MAX`, as stated in the unit brief; the
parameter name and value were not re-read from the vehicle here); CAM-S2a trained with
`NAVRL_YAW_RATE_MAX=3.0` rad/s (172 deg/s). Can the CAM-S2a policy be adapted to a 1.0472 rad/s ceiling by
continued training (S2b-FT), and how does that compare with a policy trained from scratch under the ceiling
(S2b-SCR)? Training only; every comparison is made later by preregistered evaluations.

## Where `NAVRL_YAW_RATE_MAX` acts (code at 3a453e6)

| Place | file:line | Effect of 3.0 -> 1.0472 |
| --- | --- | --- |
| Launcher pin | `train_navrl_v2_search.sh:380` `export NAVRL_YAW_RATE_MAX=3.0` | overrides any child setting; CAM-S2b sets the value afterwards in `tools/cams2b_python`, the PYTHON wrapper `train_navrl.sh` calls once for `runner.py` (before `aerial_gym` is imported) |
| Task config | `navrl_task_config.py:483` `yaw_rate_max = _env_float("NAVRL_YAW_RATE_MAX", 2.5)` | read at import |
| Action scale | `navrl_task.py:5006-5008` `command[:,3] = clamp(a_yaw,-1,1) * yaw_rate_max * _governor_yaw_scale` | action 1.0 commands 1.0472 rad/s instead of 3.0 |
| Controller clamp | `lee_controller_config_navrl.py:7,51` `max_yaw_rate` (same variable); applied at `base_lee_controller.py:153-155` | the yaw-rate setpoint is clamped to +-1.0472; equal to the action scale, so no dead band |
| Observation normaliser | `navrl_task.py:6472-6473` (perception path: `yaw_rate = body_angvel_z / yaw_rate_max`), `navrl_task.py:6403` (non-perception ego path, unused here) | the yaw-rate channel reads physical rate / 1.0472 |
| Previous-action channel | `navrl_task.py:5053` `prev_action = clamp(actions)` (yaw channel = raw normalised a_t; xy overwritten by the governed command at :5055-5057) | definition unchanged in normalised units; physically 1.0 now means 1.0472 rad/s |
| Reward | `navrl_task.py:6060` `- yaw_rate_smooth_weight (0.02, navrl_task_config.py:787) * _yaw_cmd^2` | same normalised penalty; per physical rad/s it is (3.0/1.0472)^2 = 8.2 x larger |
| Reward (not scaled) | `navrl_task.py:6058-6059` yaw-align (heading vs velocity) | no `yaw_rate_max`; harder to satisfy with slower turning |
| Governor yaw cap | `speed_governor.py:330-336`, used at `navrl_task.py:5350-5354` only if `yaw_cap_radps > 0` (`speed_governor.py:71,84`) | inactive: CAM-S2a checkpoint `cfg_speed_governor_yaw_cap_radps = 0.0` |
| Governor arc clearance | `navrl_task.py:5317-5321` (physical yaw rate) | inactive: mode `riskcap`, not `dwa_arc`/`riskcap_arc` |
| Provenance | `navrl_task.py:3097` `cfg_yaw_rate_max` in every checkpoint; warm-start check `navrl_task.py:4285` | records 1.0472 |
| Not used | `navrl_camsafe.py`, `navrl_cams2.py`, `navrl_perception.py` | no reference |

Consequence for the warm start: in normalised units the loop is unchanged (action a -> commanded rate a x yaw_rate_max
-> observed channel rate / yaw_rate_max, about a at steady state). What changes is physical: the heading turns 2.865 x
more slowly per action unit, so the camera view, target bearing and obstacle geometry evolve differently, and the
yaw-smoothness penalty per physical rate rises 8.2 x.

Consumers outside training that must use the same value for a CAM-S2b policy (not changed here):
`tools/camera_policy_runtime.py:1292,1319` and `tools/sitl_bridge.py:135,164` read `task_config.yaw_rate_max`;
`navrl_yawclip.py:75-77` refuses a clip that is not below it; `navrl_cams2diag.py:145`; `kroc_eval_adapter.py:698`;
`tools/b1r_hook.py`, `tools/h_r3_p03_launcher.py:97` and `tools/sitl_g9_lee_steps.py:29` (which defaults the variable to 3.0).

## Checkpoint contract checks on the warm start (S2b-FT)

`set_env_state` (`navrl_task.py:3741-4672`) refuses (RuntimeError) only a robot-lineage mismatch
(`navrl_task.py:3753-3774`: robot name, robot config SHA-256, URDF SHA-256; unchanged files) and routed-target
contracts (route mode off; not entered). Everything else warns and, where noted, resets the density evidence. With the
S2b-FT environment it will print exactly these (values from the checkpoint's env_state, read on CPU 2026-11-24):

1. `NavRL CONFIG MISMATCH | NAVRL_YAW_RATE_MAX: checkpoint trained with 3.000, running with 1.047. This changes policy
   inputs -- results are NOT comparable unless intentional.` (`navrl_task.py:4494-4502`; sets density_evidence_changed)
2. `NavRL CURRICULUM CONFIG MISMATCH | NAVRL_DENSITY_CHECK_EPS: checkpoint used 16384.000, running with 16.000. The
   promotion schedule is changing intentionally or the runs are not directly comparable.` (`navrl_task.py:4587-4596`)
3. `NavRL density evidence RESET | discarded=2010/3256 eps resume_warmup=1000 epochs gate_not_before_step=416000`
   (`navrl_task.py:4626-4644`; also forced by `NAVRL_RESET_DENSITY_WINDOW=1`)
4. `NavRL checkpoint state restored | task_steps=384000 bars=<build>->175 k_min=20.0 k_max=28.0 density_window=0/16 eps`
   (`navrl_task.py:4659-4672`)

No seed, action-policy, action-std, arena, perception, target-motion or task-contract mismatch is expected (same
values). The driver stops the run (TRAINING_FAIL) unless lines 1, 3 and 4 appear before the first epoch block and that
block is epoch 12,001.

## S2b-FT — warm start, 60 deg/s

- Source: CAM-S2a `last_gen_ppo_ep_12000_rew_104.0523.pth`, SHA-256
  `9fc9dfb7edfc90db0b9a86432eb8cfb172a6e708a6d58eb1d40f15b6ed548996` (run
  `ppo_261123_2213_navrl_v2-cams2a-scratch-camonly-s1`, MOTAR-camonly worktree, read-only). Its env_state: epoch 12000,
  `num_task_steps` 384000, 175 bars (stage start step 378130 = epoch 11816.6), k window 20..28, `cfg_yaw_rate_max` 3.0,
  `current_action_learning_rate` 7.5e-6 (3e-5 halved twice by the rollback), rollback total 2. The driver copies it to
  `~/motar_tmp/cams2b_20261124/source/runs/<run>/nn/` and verifies the SHA-256, because `runner.py` writes a
  `*_rlnorm.pth` next to the checkpoint it loads (`runner.py:700-722`; the checkpoint's critic keys carry `_orig_mod.`).
- Mechanism: the repository's continuation path, as CAM-K1 (`train_navrl_camk1.sh`): `NAVRL_V2_ALLOW_RESUME=1`,
  `train_navrl_v2_search.sh --checkpoint CKPT --branch_run` (new run folder), absolute `MAX_EPOCHS` 15,000. rl_games
  restores the epoch counter (`set_full_state_weights(set_epoch=True)`), so the run trains epochs 12,001..15,000
  (3,000 epochs, about 3.3 h at CAM-S2a's 3.93 s/epoch wall average).
- Curricula on the warm start (all keyed on the restored `num_task_steps`, `navrl_task.py:4596-4597`):
  FOV curriculum (3,000 epochs, `navrl_task.py:266-277`) saturated; target-speed ramp (300 epochs,
  `navrl_task.py:7273-7294`) saturated; k_min ramp (start 2,000 + 3,000 epochs) saturated; capture-gated distance
  window restored at k 20..28 = final (`navrl_task.py:4598-4601`). Density: restored at 175 bars; dwell clock restored
  (`restore_density_level_start_steps`). `NAVRL_DENSITY_RESUME_WARMUP=1000` sets `gate_not_before_step` = 384000 +
  1000 x 32 = 416000 (epoch 13,000); no evidence accumulates before it (`navrl_task.py:8866-8873`). At epoch 13,000
  the 175-bar dwell is 1,183 >= 1,000, so the first check promotes 175 -> 190; 190 dwells 1,000 epochs and promotes to
  205 at about 14,000; 205 runs to 15,000. Expected: 175 for epochs 12,001-13,000 (1,000), 190 for ~13,001-14,000,
  205 for ~14,001-15,000.
- Optimizer: LR 7.5e-6 = the checkpoint's current LR, set explicitly because `train_navrl_v2_search.sh` always passes
  `NAVRL_LEARNING_RATE` (default 3e-5), and the restore applies the configured LR (`early_stop_a2c_agent.py:1034-1055`).
  Justification: the repository rule "never undo a safety backoff" (`ppo_update_safety.py:resolve_action_learning_rate`,
  `train_navrl_v2_recover_safe.sh`); restarting at 3e-5 would undo two KL rollbacks. Adam moments kept
  (`NAVRL_RESET_ACTOR_OPTIMIZER=0`), KL stop 0.04, epoch rollback (factor 0.5, floor 1e-6, patience 5), entropy 0,
  latent margin as CAM-S2a.
- Seed 1 (= the checkpoint's training seed). Checkpoints every 250 epochs. Wall cap 4.5 h (`timeout -k 120`; raised
  from a 4 h draft before commit after review F1: CAM-S2a's wall time was 3.93 s/epoch over the whole run but about
  4.0-4.1 s/epoch at 145-175 bars, which projects FT's 3,000 epochs to about 3.4-3.6 h, and 190-205 bars may be slower).
  A correct run that hits the cap is labelled WALL_CAP, as preregistered.
- Order: S2b-SCR does not wait for the S2b-FT evaluation verdict, only for the GPU (user's sequence: FT, its short
  evaluation, then SCR overnight).
- Stop rule: at epoch **13,000** (FT +1,000), if the episode-weighted training-rollout capture of the last 200 epochs
  (12,801-13,000; epochs logged after 13,000 are excluded) is below **0.30**, the run stops: STOPPED_NO_LEARNING. (CAM-S2a's rollout capture at 175 bars was 0.617 over epochs
  11,817-12,000; 0.30 is about half.)

## S2b-SCR — from scratch, 60 deg/s

CAM-S2a's from-scratch contract (random initialisation, seed 1, 12,000 epochs, LR 3e-5, warm-up 1,000 epochs, dwell
1,000 epochs, gates 0.001) with the yaw ceiling 1.0472 rad/s and the density-window fix below. Wall cap 17 h. Stop
rule as CAM-S2a: at epoch 3,000, last-200-epoch capture below **0.05** -> STOPPED_NO_LEARNING. Expected curriculum:
70 bars for epochs 1-1,000, then +15 bars after each 1,000-1,010-epoch dwell: 85 (~1,001), 100, 115, 130, 145, 160,
175, 190, 205 (~9,001-9,030), then 205 to 12,000 (about 3,000 epochs at 205).

## Curriculum fix (CAM-S2a review r1 F3), both runs

The density gate is evaluated only when `NAVRL_DENSITY_CHECK_EPS` finished episodes have accumulated, and a check that
fails the dwell discards the window (`navrl_task.py:8880-8979`). At 16384 episodes and ~20 episodes per epoch each
window took ~800-900 epochs, so CAM-S2a's stages lasted 1,585-1,828 epochs and the run ended at 175 bars.
CAM-S2b sets `NAVRL_DENSITY_CHECK_EPS` = **16**. Arithmetic: an epoch is 128 envs x 32 steps = 4,096 env-steps and an
episode lasts at most 600 steps, so at least 4096/600 = 6.83 episodes finish per epoch on average and 16 episodes take
on average at most ceil(16/6.83) = 3 epochs (an average, not a hard bound). In CAM-S2a's log (epochs 1,001-12,000) the episodes per epoch were min 6, median 20,
mean 19.7, and 16 episodes always accumulated within 2 epochs. So a promotion follows the 1,000-epoch dwell by at most
~3 epochs. The gate itself (capture >= 0.001 over 16 episodes) fails only if all 16 episodes miss; CAM-S2a had no epoch
without a capture after epoch 1,000. Cost: ~1.2 checks per epoch, two WARNING log lines each (~7 MB per 12,000 epochs).

## Settings: differences from the CAM-S2a training receipt

The reference is CAM-S2a's receipt `~/motar_tmp/cams2a_20261123/train_s1/receipt.json` (SHA-256
`e3e9a995c5f00f206a9dcc5fa859b3bd25d2fdb38b8052d7390c041c782a076d`), whose `env_navrl` is the exact NAVRL_* environment
of that run. The driver compares the new receipt's `env_navrl` with it and stops the run on any other difference.

| Variable | CAM-S2a | S2b-FT | S2b-SCR | Why |
| --- | --- | --- | --- | --- |
| `NAVRL_YAW_RATE_MAX` | 3.0 | **1.0472** | **1.0472** | the user's 60 deg/s |
| `NAVRL_DENSITY_CHECK_EPS` | 16384 | **16** | **16** | review F3 |
| `NAVRL_V2_ALLOW_RESUME` | 0 | 1 | 0 | warm start |
| `NAVRL_LEARNING_RATE` | 3e-5 | 7.5e-6 | 3e-5 | keep the checkpoint's rollback LR |
| `NAVRL_RESET_DENSITY_WINDOW` | unset | 1 | unset | warm start: discard stale density evidence |
| `NAVRL_DENSITY_RESUME_WARMUP` | unset (250) | 1000 | unset | 1,000 epochs at 175 bars under the new ceiling |
| `NAVRL_REALVEH_RECEIPT` | its run folder | its run folder | its run folder | bookkeeping |
| `NAVRL_P9_ERROR_MODEL` | path in MOTAR-camonly | same file in the running tree | same | path only; SHA-256 pinned by the launcher |

Everything else is CAM-S2a's: camera only (no_lidar), D8 quadrotor target `mesh_flat`, camera/detection 312 x 234,
66.7 x 52.6 deg, mount 0.08/0.03 m, OAK-D depth model, P9 injector, vehicle T/W 3.5 +-15 %, imbalance +-5 %, tau
0.02-0.04 s, gain randomisation +-20 %, delay k ~ U{1,2}, capture 1.0 m / 1.0 m/s / detected, contact 0.40 m =
collision, altitude 1.5 m, camsafe (-40, 4 m, 0.5, 1.0 m/s), governor riskcap with FREE 2.5 m/s, inherited curricula
(review F12), the inherited same-density capture guard (window 50, drop 0.25, patience 25; review F11), and CAM-S2a's
known limits (previous-action hybrid F1, one-step-stale detection F5, radial-only closing bound F7, progress term at
capture F8; none is changed here, so FT stays a continuation of the same task).

The evaluation-only wrappers that `tools/realveh_entry.py` installs whenever their switch is set (`NAVRL_YAWCLIP*`,
`NAVRL_CAMS2DIAG*`, `NAVRL_CAMS2_CONTACTCODE*`; the entry neither refuses nor ignores them) stay OFF: the driver
strips every inherited NAVRL_* variable, the launcher and `tools/cams2b_python` refuse them, and the receipt check
refuses an install record for any of them.

## Driver, preflight and run-time checks

`tools/cams2b_train.py --run ft|scr [--smoke]`; launcher `aerial_gym/rl_training/rl_games/train_navrl_cams2b.sh`;
wrapper `tools/cams2b_python`. Unit folder `~/motar_tmp/cams2b_20261124/{train_ft, train_scr, train_<run>_smoke_HHMMSS}`.
Smoke: 25 epochs (ft: 12,001-12,025), development only. Runner cores 0-3,8-11,14-23 (cores 4-5 excluded).
Preflight refusals: branch other than `camonly/dev`; this file not committed at HEAD; changes under `aerial_gym/`,
`tools/`, `resources/`, `docs/plans/`; any CUDA compute process except `/usr/share/rustdesk/rustdesk` (exact path; the
RustDesk display server lists itself as a compute process); free disk below 1.0 GB + the run's estimate (FT 0.2 GB:
12 checkpoints x 8.9 MB + source copy and its `_rlnorm` 18 MB + logs/TensorBoard; SCR 0.55 GB: 48 x 8.9 MB + logs;
2.2 GB free on 2026-11-24, about 1.5 GB after both runs); the reference receipt or (ft) the source checkpoint not
matching its SHA-256; an existing run folder. Run-time: receipt check and start check as above (failure ->
TRAINING_FAIL); stop rule once at its epoch; host-fault count before and after.

## Labels and what is recorded

Labels: **TRAINED** (exit 0, exit reason `max_epochs`, last epoch = MAX_EPOCHS, final checkpoint present),
**STOPPED_NO_LEARNING**, **WALL_CAP** (timeout 124/137), **TRAINING_FAIL** (any other end, including a failed
receipt/start check and the inherited same-density guard's `early_stop_density_capture_collapse`),
**VOID_HOST_FAULT** (kernel machine-check/segfault count rose; overrides every other label). A smoke reads
SMOKE_COMPLETED / SMOKE_FAIL / VOID_HOST_FAULT. ABORTED_BY_USER if the user stops a run.
If the driver's watcher thread fails, the error is recorded and the stop rule is applied after the run; a rule that
would have fired but was not applied makes the run TRAINING_FAIL. `run_report.json` records: preflight, values, SHA-256 of driver/launcher/wrapper, receipt and start checks, stop check,
exit reason, last epoch, per-density-stage epochs/episodes/capture/crash/timeout, every promotion with its epoch and
dwell, the comparison with the expected schedule (recorded, not a label), and SHA-256 of every `last_gen` checkpoint
with the last and every 1,000th listed separately (review F11). Training-rollout rates are not evaluation results.

## What is compared later (evaluations need their own preregistration)

1. S2b-FT final vs CAM-S2a ep12000 run with a 1.0472 rad/s yaw clip (`navrl_yawclip`, the CAM-S2a-E arm): does
   adapting to the ceiling beat clipping a policy that believes it can turn at 3 rad/s? Limit: the comparison is
   confounded, because FT also has 3,000 more training epochs and is the only one of the two trained at 190 and 205
   bars. A control that isolates the yaw ceiling (CAM-S2a continued 3,000 epochs at 3.0 rad/s with the same schedule)
   is not part of this unit and would need the user's approval.
2. 45 deg/s for the first flights: first checked as an evaluation clip of S2b-FT at **0.785** rad/s (45 deg/s, below
   its 1.0472 ceiling, as `navrl_yawclip` requires). A dedicated 45 deg/s training run is proposed only if that clip
   loses materially against the unclipped S2b-FT (threshold fixed in the evaluation preregistration).
3. S2b-SCR final vs S2b-FT final (both at 60 deg/s), after SCR.
Density ranges: FT and SCR train up to 205 bars; CAM-S2a never trained above 175.

## Not claimed

No evaluation, no comparison, no flight readiness (the runtime must set `NAVRL_YAW_RATE_MAX=1.0472` for these
policies, and review F1/F2 remain open). One seed per run. S2b-FT is a continuation of CAM-S2a's lineage, not an
independent replicate.

## Review

Independent read-only review r1 (Claude Opus 5.5, fresh context, 2026-11-24;
`~/motar_tmp/cams2b_20261124/review_r1/REVIEW_r1.md`): **PASS_WITH_LIMITS**, no P0/P1. It reran the launcher chain
on CPU with a stub interpreter and confirmed that the receipt environment differs from CAM-S2a's only as tabled
above, and that the warm-start messages and checkpoint values quoted here are correct. Findings addressed before
this file was committed: F2 (watcher failure: recorded, stop rule re-applied after the run), F3 (comparison-1
confound stated, clip 1.0472), F4 (stale `AERIAL_RUN_NAME`/`_TITLE`/`_TASK_CONFIG_MODULE` stripped), F5 (stop window
ends at the check epoch), F6 (the 3-epoch figure is an average), F7 (consumer list), F8 (PX4 parameter not
re-verified), F10 (driver pinned to cores 6,7,12,13). Open: F1 (FT wall-cap margin, handled above), F9 (no code-SHA
comparison with CAM-S2a's receipt; the reviewer verified by `git diff 22ed16f 3a453e6` that the training path is
unchanged except inert opt-in hooks in `tools/realveh_entry.py`), F11 (test hygiene, minor).
