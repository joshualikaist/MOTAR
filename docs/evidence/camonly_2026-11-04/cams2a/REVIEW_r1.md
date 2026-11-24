# CAM-S2a code review r1 (aafcb21 + 22ed16f, base b05467a)

Reviewer: Claude Opus 5.5 (fresh context, read-only). Date 2026-11-24.
Scope: `aerial_gym/task/navrl_task/navrl_cams2.py`, the cams2 hook in `tools/realveh_entry.py`,
`aerial_gym/rl_training/rl_games/train_navrl_cams2a.sh`, `tools/cams2a_train.py`, `tests/test_cams2.py`, and the wrapped
methods of `navrl_task.py` (read at 22ed16f, which is the same file the receipt hashed). Run evidence:
`~/motar_tmp/cams2a_20261123/train_s1/{receipt.json, launcher.log, run_report.json, stage_summary.json, checkpoint_sha256.txt}`.
Criteria: `docs/plans/prereg_camonly_s2a_2026-11-23.md` + `_amendment1.md`, and rule sheet `~/motar_tmp/cams2_design_20261123/RULES.md` (R3, R7, R15, R16).

Nothing under `src/` was edited. `tests/test_cams2.py` was run once on CPU (cores 6,7,12,13, `-B`, no bytecode written): 11/11 OK.

Integrity checks that passed: `navrl_task.py` and `navrl_perception.py` are not in the diff b05467a..22ed16f. The receipt's
`sha256.navrl_task` (c5461656...) equals the 22ed16f blob. `cams2_sha256` (1da8c4ce...) equals the committed `navrl_cams2.py`.
`run_report.preflight.head` = 22ed16f.

## Verdict: PASS_WITH_LIMITS

The trained run implements the preregistered delay, contact-as-collision, capture rule (as worded), altitude, camera mount
and detection resolution (amended), and no ground-truth target quantity enters the actor observation through this unit.
No finding invalidates the run as a training-only result. Limits:
- the curriculum was not time-based as preregistered;
- the per-unit contact and capture counters were never recorded;
- the "previous action" the policy saw is a delayed/undelayed hybrid that the existing flight runtime does not reproduce;
- several evaluation-side recorders still treat a target contact as a capture.
Fix the P1 items before this policy is evaluated on hardware or deployed.

## Findings

| ID | Sev | Finding |
| --- | --- | --- |
| F1 | P1 | The observed previous action is a hybrid: xy is the **delayed** command after the governor and blind cap, a_{t-k}. z and yaw are the **undelayed** raw action a_t. The flight runtime feeds the undelayed governed xy instead. |
| F2 | P1 | The flight runtime/contract does not carry the cams2 config: altitude 1.5 (which also sets the height-observation normalisation), camera x 0.08 m, delay, or capture/contact rule. A sim evaluation that does not install cams2 would score the 0.40 m contact as a capture. |
| F3 | P2 | Curriculum deviation: promotion is gated by 16,384-episode windows on top of the 1,000-epoch dwell. Stages lasted 1,585–1,828 epochs, not 1,000. The density was 85 bars at epoch 3,000 (prereg: 100) and 175 at epoch 12,000 (prereg/R15: 205 by ~10,000). |
| F4 | P2 | Preregistered "CAM-S2a contact/capture counters" not recorded: `_cams2_stats` is filled but never printed or exported. Target contacts cannot be separated from obstacle crashes in the log. |
| F5 | P2 | Capture uses the previous observation's detection (`_visible_now` is one control step older than the distance and closing-speed test). R3 says "target detected that step". |
| F6 | P2 | Target contacts converted by the wrapper keep `_crash_cause_code = -1`. They are recorded as captures by `observe_capture`, the episode dump and the obs dump, and the crash diagnostics exclude them. This matters for any evaluation that uses those recorders. |
| F7 | P2 | The capture rule bounds only the radial closing speed, so any receding or tangential motion qualifies. A fast fly-by within 1 m (for example 2.5 m/s tangential at 0.8 m) counts as a capture. This matches the prereg wording but not the "safe capture" intent. The evaluation should report relative speed at capture. |
| F8 | P3 | The progress term is not zeroed at terminal for wrapper captures: up to 0.99 reward below the task's own terminal handling. The camsafe blind penalty is applied on the capture step. |
| F9 | P3 | Priority change: a same-step crash now beats a capture (the base task did the opposite). This is conservative; it is not stated in the prereg. |
| F10 | P3 | The integrated delay test draws k = [1, 1] (seed 4961, n=2), so the k=2 path through the wrapper is exercised only by the pure `delayed()` test. Further test gaps: no negative control that a non-reset env keeps its history; no prev_action, progress-term or counter-report test. |
| F11 | P3 | Recording/label gaps: checkpoint SHA-256 is listed for 1000/3000/6000/9000/11750/12000 only (prereg: every 1,000th). The driver emits `COMPLETED_PROCESS`, not the prereg's `TRAINED`, and has no `VOID` mapping (faults 1→1, so not VOID). There is an inherited same-density early-stop guard that is not in the prereg; it did not fire. |
| F12 | P3 | Inherited curricula not listed in the prereg table: the capture-gated distance curriculum (k_max 7→28, threshold 0.6), the FOV curriculum (3,000 epochs) and the target-speed ramp (0.3→1.5 m/s over 300 epochs). |
| F13 | P3 | The delay seed is fixed (4961) and independent of NAVRL_SEED, so later seeds would share the delay stream. There are 4 host syncs per step (`.any()`, `.item()`). Stale comments: the launcher says "T/W 4.0", "160 x 90" and "train_navrl_cams1.sh"; the driver usage string says `cams1_train.py`. |

## Q1. Does the run implement every preregistered setting as written?

| Prereg row | Evidence (receipt / code) | Status |
| --- | --- | --- |
| Random init, camera only from epoch 0 | `unset CKPT` (train_navrl_cams2a.sh:36), `NAVRL_SENSOR_ABLATION=no_lidar` (:106); receipt `ablation.condition = no_lidar` | match |
| D8 quadrotor target, mesh_flat | `NAVRL_DYNAMIC_MESH_TREATMENT=mesh_flat` (:110; receipt env) | match |
| Detection resolution (amendment 1: render = detect = 312x234) | receipt env CAMERA 312x234, DETECT 312x234 (:112-115) | match (amended) |
| P9 injector | NAVRL_P9_ERROR_MODEL + SHA check (:40-50, :141-147) | match. `NAVRL_DETECTION_DROPOUT=0.3` and `DEPTH_NOISE_STD=0.02` are set but inactive, because `PERTURB=0` → `training=False` (navrl_task.py:6477; navrl_perception.py:1640, 1595) |
| Camera 66.7 x 52.6, mount 0.08 fwd / 0.03 up | HFOV/VFOV env (:107-108); receipt `cams2.config_after.camera_translation = [0.08, 0, 0.03]`. Used by the renderer (navrl_detector.py:555, 821) and by perception's back-projection (navrl_perception.py:784) | match |
| Depth model | receipt `depth_model.settings` = 29.88 / 0.125 / 0.315 / 20 / 0.02; validity unset (:135) | match |
| Vehicle T/W 3.5, ±15 %, imbalance ±5 %, tau 0.02–0.04 | receipt `vehicle_rand.settings`; `thrust_to_weight_range [2.826, 4.226]` | match |
| Gain randomisation ±20 % | receipt env GAIN_RAND=1, SCALE 0.2 | match |
| Delay k ~ U{1,2} per episode, `NAVRL_ACTION_DELAY_STEPS_MAX=2` | navrl_cams2.py:126, 141 (`randint(dmin, dmax+1)`), drawn at every reset; receipt `delay_steps [1,2]`; env `NAVRL_ACTION_DELAY_STEPS_MAX=2`; settings refuse a mismatch (:65-67) | match. Zero command before the episode's first action (history zeroed at reset, :140) |
| Capture: dist ≤ 1.0 AND closing ≤ 1.0 AND detected; < 0.40 m swept = collision | navrl_cams2.py:94-100, 145-159; receipt `success_radius 0.4`, radius 1.0, closing 1.0 | match as worded. Detection is one step stale (F5); radial-only bound (F7) |
| Altitude 1.5 m (both drones) | receipt `flight_altitude 1.5`. The task reads it for the robot spawn (navrl_task.py:2417), the target spawn (:2775) and the target per step (:8216/:8400) | match |
| Rewards: collision −40, r_ss range 4 m, blind penalty 0.5, blind cap 1.0 m/s outside ±33.35°, FREE 2.5 | receipt `camsafe` (collision −40, rss_clip 4, blind 1.0/0.5); env `SPEED_GOVERNOR_FREE_MPS=2.5`; half-FOV = perception.hfov/2 = 33.35° (navrl_camsafe.py:165) | match |
| Curriculum: 70 for 1,000 epochs, then +15 every 1,000 (gates 0.001) | navrl_task.py:8880-8884 gates every check on `check_after_episodes` = `NAVRL_DENSITY_CHECK_EPS=16384` (log line 95: `check_eps=16384`), in addition to the dwell (:8915-8925). Log: promotions at dwell 1749, 1704, 1655, 1585, 1617, 1679, 1828 epochs, each "after 16384 eps", with a DWELL hold at ~800-915/1000 epochs in between. stage_summary: 70 (1–1749), 85 (1750–3453), 100 (3454–5107), 115, 130, 145, 160 (9989–11816), 175 (11817–12000) | **DEVIATION (F3)** |
| Checkpoints every 250 | env `NAVRL_SAVE_FREQUENCY=250`; `runs/.../nn` holds 48 `last_*` files at 250-epoch steps | match |
| Stop rule at 3,000 (last-200 capture < 0.05) | cams2a_train.py:48-78, 168-177; run_report `stop_check` at epoch 3001, capture 0.734 → continue | match |
| Recorded: cams2 contact/capture counters | `stats` dict (navrl_cams2.py:117, 160-162) has no report path; zero matches in launcher.log | **DEVIATION (F4)** |
| Recorded: checkpoint SHA of the last + every 1,000th | checkpoint_sha256.txt lists 1000, 3000, 6000, 9000, 11750, 12000 | partial (F11) |

The curriculum deviation (F3) does not make the run invalid as training. It does change what "trained" means: the final
density was 175, not 205. The time-based reading the prereg promised (100 bars at the stop check, 205 from ~10,000) did
not happen. It needs a recorded deviation note, and any evaluation density range should be chosen knowing that the policy
never trained above 175 bars.

## Q2. Correctness of navrl_cams2.py

**Delay indexing (OK).** `transform_action_to_command` rolls the history by one, writes a_t to slot 0 and applies slot k
(navrl_cams2.py:129-133). So the command executed during interval t is a_{t-k}, with k ∈ {1,2} and 3 slots (`dmax+1`, :122).
There is no off-by-one: the pure test `test_delayed_selects_per_env` checks slot 1 and slot 2. Together with the
one-interval observation age this gives 100/200 ms of additional actuation latency, as R7 intends.

**History reset (OK).** The wrapped `reset_idx` zeroes the history rows of the reset envs and redraws their k
(:135-143). This runs after `orig_reset`, which zeroes `prev_action` (navrl_task.py:4901). The full `reset()` calls
`reset_idx(arange)` (:4695) before the history exists, and `_ensure` then allocates zeros, which is equivalent.
`env_ids` from the env manager is a long tensor on the device; `torch.as_tensor(..., device=hist.device)` handles that.
The test also covers a Python list.

**Generator seeding (OK, P3).** There is a device-local `torch.Generator`, seeded once with 4961, and it draws only
through that generator, so the global or training RNG is not disturbed. The k sequence depends on reset order, which is
deterministic. It is independent of NAVRL_SEED (F13).

**Effects reach rewards, terminations, statistics and resets (OK).**
- `self.terminations` aliases `obs_dict["crashes"]` (navrl_task.py:1577). The wrapper writes `self.terminations[cap] = 1`
  in place (:158), so the env manager's `collision_tensor` sees it and resets those envs (env_manager.py:409-419).
- `step()` reads `self.captured_now` / `self.crashed_now` after the reward call (navrl_task.py:5063, 5084-5085). It passes
  them to `_episode_outcome_info`, `_record_general_result` (:5143), `_log_progress` (:5144), `_update_curriculum`
  (:5160) and `_record_epoch_dashboard` (:5161).
- The log's "captured (success)" therefore counts cams2 captures, and the density and distance curricula are driven by
  them.
- `add_static_safety_reward` (camsafe replacement) runs after the wrapper, with alive = `terminations <= 0`, so captured
  and contact envs get no r_ss (:5179; navrl_camsafe.py:196-198).

**Contact vs capture ordering (OK; one change of priority, F9).** The task's swept test at `success_radius = 0.40` sets
`captured_now`. The wrapper turns those envs into a collision: the reward is overwritten with `collision_penalty` (−40,
the same as the task's crash overwrite at navrl_task.py:6298), `captured_now` is cleared and `crashed_now` is set
(:147-151). The capture bonus that the task added for those envs is discarded by the overwrite, so nothing is counted
twice. Capture is then evaluated only for envs with `terminations <= 0` (:152). This excludes contact envs and envs that
crashed into obstacles or went out of bounds in the same step. The base task gives capture priority over a same-step
crash (:6124); the wrapper reverses that (F9, conservative).

**Closing-speed sign (OK).** rel = robot − target and v_rel = v_robot − v_target, so d|rel|/dt = rel·v_rel/|rel|.
closing = −that is positive when approaching (:98-99). The unit test's env 1 (rel +0.9 x, v −0.5 x → closing +0.5) and
env 2 (closing +2.0, refused) confirm the sign. Receding gives a negative closing speed, which always passes (F7).

**Freshness of positions (OK except visibility, F5).**
- `robot_position` / `robot_linvel` are post-physics (the reward is called after `sim_env.step`, navrl_task.py:5059-5063).
- `target_position` was advanced before physics (:5021), and `target_vel_w` is the realised velocity of the same
  interval. These are the same quantities the task uses for `reward_vel` (:6029).
- `_visible_now` was written by `_process_obs_perception` (:6482) during the previous `get_return_tuple`, that is, the
  observation at the start of the interval. The capture therefore uses a detection one control step (100 ms) older than
  the distance/speed test. The task's own comment acknowledges this (:6064-6066).
- `diagnostics["visible"]` is the camera detection after P9 (state ≠ 2) OR'ed with LiDAR association
  (navrl_perception.py:1675, 2286, 2394). Under `no_lidar`, the LiDAR half is structurally off (navrl_sensor_ablation.py
  docstring; the association requires a return), so it is the camera detection. That is the right quantity, one step
  stale.

**Interaction with the task's own terms (F8).** For envs the wrapper captures, the task had already added
`progress_weight·(prev_dist − γ·dist)` with `term = False` (navrl_task.py:6285-6295). A terminal capture inside the task
would have used φ_next = 0. The wrapper capture therefore earns up to γ·1.0 = 0.99 less than a task-native terminal
capture. This is a small, closer-is-better bias (against bonus 30). The camsafe blind penalty (navrl_camsafe.py:176-183)
was also applied to these envs because they were alive when camsafe ran. Both effects are minor and consistent across
the run.

**Recorders (F6).** `_crash_cause_code`, `_episode_forensics.observe_capture`, `_record_episode_dump` and
`_record_obs_dump_crash_outcomes` run inside the original method (navrl_task.py:6120-6160). They see the 0.40 m contact as
a capture and the wrapper's capture as nothing. Crash-cause fractions in the log sum to 1 over `n_crash`, which excludes
target contacts (log line 1784: `bar_contact=0.943 ... oob=0.057 (n_crash=619)`). None of these feed training. Any
evaluation that uses them must be corrected.

**Device/dtype (OK).** History in the action dtype/device; delays int64 on the device; masks bool; scalar adds as
Python floats. There are 4 host syncs per step (`bool(any())`, `.item()` ×2), which affects speed only.

## Q3. Ground truth in the actor observation

**None through this unit.**
- The wrapper reads `target_position`, `target_vel_w`, `robot_position` and `robot_linvel` only to decide
  reward/termination (navrl_cams2.py:153-159). It writes only `rewards`, `terminations`, `captured_now`, `crashed_now` and
  its private `_cams2_*` buffers.
- The actor observation in perception mode is `perception.observe(...)` (navrl_task.py:6463-6481). The only values
  this unit changes in it are:
  - `previous_action` (via the delayed command, Q4);
  - the height normalisation `z / (3·flight_altitude)` (navrl_perception.py:2337), which is a constant;
  - the camera mount (renderer and back-projection), which is a constant.
- k itself is not observed.
- The privileged critic `states` still carry the ground-truth extras (navrl_task.py:6519-6540). That is pre-existing and
  is train-time only.

## Q4. What is the "previous action" in the observation?

It is a **hybrid**, and it is neither purely the raw action nor purely the delayed command (F1, P1):
- `step()` sets `prev_action = clamp(actions)` from the policy's **raw, undelayed** a_t (navrl_task.py:5053).
- Because the governor is on (`NAVRL_SPEED_GOVERNOR=riskcap`, receipt env), it then overwrites xy with
  `_last_governed_action_xy` (:5055-5057).
- That value is computed inside the original `transform_action_to_command` (:4967-4970) from the **delayed** action
  a_{t-k} that the wrapper passed in (navrl_cams2.py:133). It includes the riskcap governor and the camsafe blind cap.
- Result:
  - `prev_action[0:2]` = governed, blind-capped a_{t-k} xy / v_max (the command actually executed);
  - `prev_action[2]` = raw a_t z (z is not actuated; altitude hold);
  - `prev_action[3]` = raw a_t yaw. The yaw actually executed was a_{t-k} yaw (`_yaw_cmd`, navrl_task.py:5006).

Why this matters for deployment:
- `tools/camera_policy_runtime.py:1309-1310` builds the previous action as the governed xy of the **current** output a_t
  plus a_t z/yaw. It applies no blind cap (the missing blind cap is a CAM-S1 / R14 gap, outside this unit).
- Under CAM-S2a training, the xy channel of the next observation lagged the policy's own output by k = 1–2 steps. The
  runtime gives the policy a previous-action xy that leads the trained distribution by k steps.
- On hardware the latency is physical, so the runtime would have to delay its xy previous-action record by a chosen
  k̂ ∈ {1,2} to match training.
- Alternatively, retrain with a consistent definition: either the raw a_t for all 4 channels, or the applied
  (delayed) command for all 4.
- The yaw/xy inconsistency should be fixed in CAM-S2b in any case.
- Decide and record the definition before evaluation on hardware or SITL. A sim evaluation with cams2 installed
  reproduces the training semantics exactly.

## Q5. Are the tests non-vacuous?

Mostly yes. Each refusal has a positive control (`test_refusals` ends with a valid parse;
`test_refuses_without_camsafe_and_twice` installs once successfully; `test_not_detected_means_no_capture` flips
visibility to True and gets a capture). Counts are asserted non-zero (contacts 1, captures 1). The capture-mask test
isolates each of the three conditions plus alive.

Gaps (F10):
- With seed 4961 and n=2 the integrated delay test draws k = [1, 1] (checked on CPU: `torch.randint(1,3,(2,))` → [1, 1]).
  The wrapper's k=2 path is therefore not exercised end to end; only the pure `delayed()` test covers slot 2.
- The reset test checks that env 0 was cleared, but not that env 1 was untouched or that k was redrawn.
- The fake task has no `prev_action`, governor, progress term or camsafe layer, so F1, F8 and the blind-penalty
  interaction cannot be detected.
- There is no test that the counters are emitted (F4) and no test of receding or tangential capture (F7).

## Recommended actions

1. **Before any hardware/SITL use (P1):**
   - Fix the previous-action definition (F1): pick the runtime's k̂ or retrain consistently.
   - Make the flight runtime/contract load the cams2 config: altitude 1.5 for the height normalisation and altitude
     hold, camera x 0.08, delay bound (F2).
   - A sim evaluation must install `navrl_cams2` with the receipt's settings; otherwise the 0.40 m contact is scored as
     a capture.
2. **Record in the WORKLOG/EXPERIMENTS entry (P2):**
   - the curriculum deviation (actual stage lengths, final 175 bars);
   - that cams2 counters were not recorded;
   - the one-step-stale detection in the capture rule;
   - the radial-only closing bound.
   For evaluation: export target-contact and capture counts and relative speed at capture, and fix the crash-cause and
   recorder attribution (F6).
3. **CAM-S2b (P3):**
   - zero the progress term and skip the blind penalty on wrapper captures;
   - add a report line for `_cams2_stats`;
   - tie the delay seed to the run seed;
   - make the delay test draw both k values;
   - complete the per-1,000 checkpoint hashes (the files exist in `runs/.../nn`);
   - map the driver verdict to the prereg labels.
