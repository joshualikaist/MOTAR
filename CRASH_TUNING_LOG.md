# NavRL perception PPO — crash-cause tuning log

Measured, not guessed. The sensor-only perception+Transformer PPO run was stuck at ~99% crash / no
learning. Instead of guessing, we instrumented the exact termination cause (`NAVRL_CRASH_DIAG=1`)
and fixed the top measured blockers one at a time, re-measuring each time.

## How to diagnose (crash-cause instrumentation)

Run any training/eval with `NAVRL_CRASH_DIAG=1`. Every ~2048 finished episodes it logs:

```
NavRL crashdiag | bar_contact=0.54 (mean_x=4.8m steps=91) below=0.00 above=0.00 oob=0.46 [W=417 E=0 S=55 N=201 steps=135] (n_crash=1473)
```

- `bar_contact` — hit an obstacle bar (`mean_x` = where, `steps` = how long it survived).
- `below` / `above` — left the [0.1, 4.0] m altitude envelope (floor / ceiling strike).
- `oob` — left the arena; `W/E/S/N` = which wall (W = west = behind spawn), `steps` = time-to-exit.

Off by default (zero overhead). This is the single most useful tool for un-sticking a run.

## Measured diagnosis (2026-07-22/23)

First run, before fixes — crash cause split at 25 bars, static goal, from scratch:

| cause | share | note |
|---|---|---|
| **floor strike** (`below`) | **39%** | altitude bled off during maneuvers — the largest single cause |
| **bar contact** | 35% | `mean_x ~4.5 m` = right at the 4 m LiDAR horizon (can't see the bar in time) |
| **out-of-arena** | 26% | mostly `W` (drift back past spawn) + lateral — body-frame action + tight OOB fence |

Two more structural failures found by inspection/measurement:
- **Goal invisibility cold-start**: the target token is 0 until the detector first *acquires* the
  target (KF activates only inside FOV+range). A goal spawned outside the ±43.5° camera FOV left the
  actor goal-blind — `captured` flat at 0, no gradient. (Root cause of the original 99%-crash stall.)
- **Time-based distance curriculum runaway**: the goal window ramped `5-8 m → 15-24 m` by *epoch*
  regardless of skill; at capture ~1% every episode ended in a crash and the value function collapsed.

## Fixes applied (commit c37ade7 — all env-gated / vision-only; LiDAR path unchanged when off)

1. **Altitude-hold closed loop** — `vz = clamp(2*(flight_altitude - z))` instead of open-loop `vz=0`.
   Action space unchanged (actor still cannot command vertical). → `below` 39% → **0%** (measured).
2. **Cold-start FOV goal curriculum** (`NAVRL_FOV_CURRICULUM_EPOCHS`, default 3000) — constrain the
   goal's initial bearing to the camera FOV, widening over training. Shapes only initial conditions,
   **not** a GT leak. → gives the actor a target bearing to act on from step 0.
3. **Competence-gated distance curriculum** (`NAVRL_K_COMPETENCE=1`) — the goal window deepens only
   when capture clears `NAVRL_K_THRESHOLD` (default 0.6), mirroring the density curriculum. Persisted
   across `--checkpoint` resume. Logs `distance curriculum promoted|held`. → difficulty never outruns skill.
4. **Crash-cause instrumentation** (`NAVRL_CRASH_DIAG=1`) — the diagnosis tool above.

Both new code paths adversarially verified (2 workflow panels, 8 lenses, 0 blockers).

## Validated result

Phase-A run (below), 25 bars, static target: **`captured` 0.008 → 0.17** (prior runs plateaued at
~0.02), `crash` 0.99 → 0.63, `below` → 0.000. The fixes work.

## Reproducible recipe

Do **one difficulty axis at a time** — running the distance and density competence curricula together
makes them compete for the same capture budget and stalls (verified). Keep every goal inside the 20 m
detector range: `NAVRL_K_FINAL <= 18`.

```bash
cd aerial_gym/rl_training/rl_games && conda activate aerialgym

# Phase A — master distance at fixed low density (what was validated above)
NAVRL_PERCEPTION=1 NAVRL_CRASH_DIAG=1 \
NAVRL_K_COMPETENCE=1 NAVRL_K_FINAL=16 \
NAVRL_FOV_CURRICULUM_EPOCHS=1000000 \
NAVRL_NUM_BARS=25 \
./train_navrl.sh --max_epochs 10000 --seed 1

# Phase B (later) — freeze distance, ramp density by competence
#   drop NAVRL_K_COMPETENCE, set NAVRL_NUM_BARS unset, add
#   NAVRL_DENSITY_CURRICULUM=1 NAVRL_DENSITY_START=25 NAVRL_DENSITY_FINAL=110 NAVRL_DENSITY_THRESHOLD=0.65
```

## Known next blocker (open)

After peaking at 17% capture, PPO **collapses to a risk-averse hover** (100% timeout): approach
through the bars is too crash-prone (`bar_contact` 54-76% at `mean_x ~4.5 m`), so not-moving beats
gambling on a crash. Root cause = the **4 m LiDAR horizon** (`navrl_lidar_config.max_range = 4.0`) —
the drone cannot see a bar until 2 s before impact at 2 m/s.

**Next fix to try:** extend the obstacle horizon (LiDAR `max_range` 4 → 8 m; the sensor config
comment already notes a 10 m option) so navigation becomes reliable enough that approach beats hover,
then re-validate with `NAVRL_CRASH_DIAG=1`. Secondary: OOB drift (26%) — body-frame action + tight
fence; and a small trim of `visibility_bonus` if the hover optimum persists.

---

# Session 2 (2026-07-23/24): sensor-only MOVING-target interception + the sudden-NaN root cause

## What now works end-to-end
Sensor-only perception+Transformer policy intercepts a MOVING target through a 25-bar field at a
FASTER drone speed. Enablers added this session (all committed):
- Env-overridable `NAVRL_YAW_RATE_MAX` (task + Lee controller, default 2.5). At 2.0 m/s the weave
  already needs ~2.4 of 2.5 rad/s -> yaw-rate (not thrust/tilt, T/W~3.3) is the binding
  maneuverability limit. Ran the faster regime at yaw 3.0 / max_velocity 2.5.
- Moving target via `NAVRL_TARGET_SPEED_FINAL` ramp (0->1.5 m/s). The warm-started policy tracked
  and intercepted; capture stayed ~0.8 up to target ~1.25 m/s (drone 2.5).

## THE root cause of the repeated mid-run deaths: policy log-std runaway (fixed)
Runs kept dying by a SUDDEN NaN (~epoch 5000 of healthy capture 0.8-0.9, then a_loss->NaN in ONE
step -> hover collapse). NOT gradual overtraining. At the NaN step c_loss / kl / explained_variance
were all healthy; only ppo/entropy was pinned at ~16 == policy std sigma ~ 13.
Mechanism: `fixed_sigma: True` + `entropy_coef>0` -> log-std has no upper bound; as difficulty rose
the policy-loss's downward pressure on sigma weakened while entropy_coef kept pushing up, so sigma
drifted 1 -> ~13 over ~5000 epochs until the PPO log-prob/gradient overflowed to NaN. Both
entropy_coef 0.005 and 0.003 died this way (rate only). FIX (commit bb0faa7): clamp log-std to
[-5, 0.4] (sigma <= 1.49, entropy <= ~7.3) in navrl_transformer_network.forward -> sigma=13 is
unreachable. VALIDATED: warm-started from the 98.6% peak it sailed PAST epoch 5178 (the exact old
death point) healthy (entropy flat ~4.4, capture 0.86-0.91, no NaN).

## Eval/play crash was NOT a bug in our code
`*** Can't create empty tensor` is a benign Isaac Gym build-time diagnostic (hidden in train by the
quiet wrapper, shown in play). The real crash at NUM_ENVS=512 was VRAM OOM (512 perception cameras
on the 8 GB 3070). NUM_ENVS=128 (= training) evals fine. Use 128 for eval, not 512.

## BASELINE (measuring stick) -- results/baseline_speed_axis_peak986.csv
98.6%-peak policy, 25 bars, 13-16 m goals, drone 2.5 m/s, deterministic, 2049 episodes/cell:
  target 0.0/0.5/0.75/1.0/1.25/1.5 -> capture 0.741/0.762/0.755/0.744/0.724/0.686
Key: capture is FLAT ~0.74 across target speed (drone is fast enough), timeout ~0 (no hover). The
bottleneck is CRASH ~25% (bar contacts), NOT target speed -- confirming the earlier "25 bars but it
still crashes" concern. The 0.986 training peak vs 0.74 eval is the honest train-vs-eval gap.

## Next: crash-reduction tuning, ONE parameter at a time (measure each vs the 0.74 baseline)
Ranked (reward tuning excluded -- prior 3 null results, crash is geometric):
1. Proper altitude control (Lee controller z-position feedback) -- ENABLER: lets look-ahead extend
   without the floor strikes that killed the 12 m attempt (floor was altitude sag under heavier
   weaving, not a LiDAR floor return -- create_ground_plane=False, no floor mesh).
2. Extend obstacle look-ahead (LiDAR 8 -> 10/12 m) -- only after #1.
3. Feed the 10 m camera obstacle-depth to the actor (forward-only, no floor issue).
4. More obstacle tokens (5 -> 8) or larger Transformer (dim 64 -> 128).
Note: each needs a retrain (~30-40 min) + a 128-env eval; there is no zero-cost crash win.

## Candidate #1 RESULT: altitude PI-hold -- CONFIRMED WIN (2026-07-24) -- results/altitude_pi_speed_axis.csv

Root cause: `lee_velocity_control.compute_acceleration` always passes `setpoint_position=self.robot_position`,
so the low-level position-error term is permanently zero -- the controller is pure velocity-tracking. The
task-level altitude hold (`vz = clamp(4*z_err, +-mv)`) is a bare P loop on top of that, and during sustained
lateral+yaw weaving the attitude-tracking transient (desired vs actual body-z axis) biases achieved vertical
accel low even though thrust has headroom (T/W ~= 3.3) -- P alone settles to a nonzero steady-state sag.
Fix (uncommitted, navrl_task.py): PI, not P -- `vz = clamp(4*z_err + Ki*integral(z_err dt), +-mv)`, Ki=1.0,
anti-windup clamp to +-mv/Ki, integral reset every episode in reset_idx.

Validated in two stages:
1. **Stability**: warm-started from the 98.6% peak, ran ~450 epochs (3344->3800), 18 crashdiag samples --
   `below` stayed in a 0-5.6% band throughout (vs the 39%/71% floor-strike blowups seen earlier this
   session under weaker altitude authority). Reward/capture/crash all stayed in normal healthy ranges, no
   NaN.
2. **Held-out eval** (128 envs, 2049 episodes/cell, deterministic, same grid as the baseline), checkpoint =
   this run's best-reward `gen_ppo.pth` (epoch ~3800, reward ~161):

   | target | baseline capture | PI capture | baseline crash | PI crash |
   |---|---|---|---|---|
   | 0.0  | 0.741 | **0.804** | 0.251 | **0.191** |
   | 0.5  | 0.762 | **0.822** | 0.238 | **0.178** |
   | 0.75 | 0.755 | **0.813** | 0.245 | **0.187** |
   | 1.0  | 0.744 | **0.808** | 0.256 | **0.192** |
   | 1.25 | 0.724 | **0.786** | 0.275 | **0.214** |
   | 1.5  | 0.686 | **0.778** | 0.314 | **0.222** |
   | **mean** | **0.735** | **0.802** | **0.263** | **0.197** |

   **capture +6.7pt / crash -6.6pt on average, uniform across every target speed** (biggest gain at the
   fastest target, 1.5 m/s: +9.2pt). `below` (floor strikes) is now 1.6-2.3% of crashes at every speed --
   effectively solved. `bar_contact` remains the largest single crash cause (54-64% of crashes) but its
   absolute rate fell along with the total. **New finding**: `oob` (arena exit, overwhelmingly the N wall)
   is now 34-44% of crashes -- floor strikes were masking how much drift-out was already happening.

Next candidates, in order:
2. Extend LiDAR look-ahead 8 -> 10/12 m -- now unlocked (altitude no longer sags under heavier weaving).
3. Investigate the N-wall `oob` drift specifically (new leading secondary cause, wasn't visible before #1).
4. Feed 10 m camera depth to actor.
5. More obstacle tokens (5 -> 8) or larger Transformer.

## N-wall oob investigation -- starting hypothesis (2026-07-24, not yet investigated)

oob detection: `navrl_task.py:1070-1111`, N = `pos[:,1] > b_max[:,1] + oob_margin` (lateral/y-axis,
positive direction). Crash breakdown consistently shows N >> S,E,W (e.g. speed 1.0: N=122 vs
S=26,E=1,W=0 -- N is ~80% of all oob exits, not just the plurality).

Hypotheses to check first (cheap, no retrain needed -- code/data inspection only):
1. **Goal-y sampling asymmetry**: goal y is "free across the arena minus wall margin" (reset_idx,
   ~line 774) -- check whether the sampling is actually symmetric around the arena y-centerline, or
   biased toward +y (N), which would pull the drone's whole trajectory (and evasive excursions)
   toward the N wall.
2. **Bar layout asymmetry**: random bar placement (asset_manager) could be denser near the S side by
   chance-of-seed, statistically forcing evasive maneuvers northward.
3. **oob_margin too tight relative to weave amplitude**: now that altitude is fixed and the drone
   flies more assertively (higher effective speed/aggression in dodges), lateral excursion amplitude
   during a dodge may routinely exceed the fence's margin on whichever side the dodge happens to go.
   If (1)/(2) rule out systematic bias, check whether just widening the arena y-bound or oob_margin
   (vision-only, cosmetic) removes most N exits without touching reward/behavior.

Do NOT touch reward shaping. This is a placement/geometry lens first (same discipline as the
corner-clip diagnosis that led to the yaw-rate fix earlier this project).

## N-wall/OOB investigation -- measured result (2026-07-24)

Added evaluation-only `NAVRL_OOB_PROBE=1` and an env-overridable `NAVRL_OOB_MARGIN`. The probe never
enters actor/critic observations. It records initial/current target pull, active-bar y bias, outward
velocity/command, target visibility, track age/covariance, and lateral excursion at each OOB event.

Code inspection ruled out the first two hypotheses:
- Goal y sampling is symmetric about the arena center.
- Active bars are sampled symmetrically in y with direction-free rejection spacing.
- The boundary is not physical geometry. It is only a coordinate termination, and the actor gets
  neither absolute xy nor distance-to-boundary, so LiDAR/camera cannot observe it.

Paired 2,049-episode, seed-42 evaluation at target speed 1.0 m/s:

| spawn regime | margin | capture | crash | OOB / crash | lateral OOB |
|---|---:|---:|---:|---:|---:|
| fixed/central (diagnostic only) | 0.5 m | 0.900 | 0.100 | 0.190 | 35 |
| fixed/central (diagnostic only) | 1.0 m | 0.903 | 0.097 | 0.075 | 15 |
| **generalized random drone+target** | 0.5 m | **0.541** | **0.459** | **0.693** | **379** |
| **generalized random drone+target** | 1.0 m | **0.584** | **0.416** | **0.618** | **297** |

The required generalized regime is the decisive result. At margin 0.5, lateral exits had
`goal_pull_side=-6.62 m`, `bar_bias_side=-0.002`, `outward_vy=2.51 m/s`, target visibility 0.003,
and tracker age 4.95 s. The target was on the opposite side and the bars were unbiased, yet the
policy flew outward at full speed after losing the target. This is an observability/generalization
failure, not an N-wall placement bias. N>S was not stable under paired reruns, so there is no evidence
for a fixed north-wall code bias.

Margin 1.0 is a justified tolerance for the artificial invisible boundary (+4.3 capture points in
the generalized regime), but it does not solve the behavior: residual crash is still 41.6%. Therefore
do not combine LiDAR 10 m with the first correction. First run `train_navrl_general_8m_finetune.sh`
to teach randomized spawn at the verified 8 m sensor setting. Only after held-out generalized
capture recovers should the next one-variable branch change LiDAR 8 -> 10 m.

## `below` root cause under general-spawn: tilt-induced thrust sag at spawn -- MEASURED + FIXED (2026-07-24)

New crashdiag forensics (below now logs steps-to-death + tilt-at-death). General-spawn eval, comp OFF:
`below=0.134 (steps=15 tilt=30deg)` vs bar_contact steps=39, oob steps=29. Translation: below deaths
happen ~1.5 s after spawn (earliest of all causes) while banked ~30 deg -- the random-spawn initial
sharp turn, exactly when the task-level altitude-PI integral is still zero. Mechanism, from
velocity_control.py: Lee thrust T = f.b3 delivers vertical force (f.b3)*b3_z, which equals the
commanded f_z ONLY when the desired-force direction and the CURRENT body axis agree; during
attitude-lag transients the deficit is deterministic (cos-of-mismatch), NOT a prediction problem --
both vectors are known at the line where thrust is computed.

FIX: altitude-priority thrust (PX4-style tilt compensation), NavRL-scoped opt-in:
`T = f_z / clamp(b3_z, min=0.5)` -- achieved vertical force equals f_z regardless of current tilt
(60-deg cap). velocity_control.py gated by cfg flag `tilt_thrust_compensation`; enabled only in
lee_controller_config_navrl via `NAVRL_TILT_COMP` (default ON; set 0 to A/B). Cost: during mismatch
some thrust leaks laterally along the stale body axis (slightly slower reversals) -- acceptable,
floor strikes are terminal.

Zero-shot A/B on the general-spawn ckpt (0209, speed 1.0, n=2048/leg):
  OFF: capture 0.850, below absolute 1.96% (share 0.134, tilt 30deg)
  ON : capture 0.843, below absolute 1.22% (share 0.078, tilt 36deg)  -> below -38%, capture unchanged
Fingerprint check: surviving below-deaths shifted to HIGHER tilt (30->36 deg) = the compensation
removed exactly the moderate-tilt deaths it targets. Residual ~1.2% = motor-RPM lag (thrust arrives
late regardless of geometry) + recovery limits once vertical speed has built up; expect further
reduction from fine-tuning WITH comp on (policy can stop self-limiting its bank angle).

Next: fine-tune from the 0209 general ckpt with NAVRL_TILT_COMP=1 (the new default), then re-run the
6-speed general eval vs results/general_8m_speed_axis.csv.

## Tilt-comp fine-tune RESULT: below fixed but capture FLAT -- "conservation of failure" (2026-07-24)

Fine-tuned the general-spawn policy WITH NAVRL_TILT_COMP=1 (0209 -> 1052, +~1180 epoch, healthy,
peak captured 96.9%). 6-speed deterministic general-spawn eval vs the pre-tiltcomp general policy
(results/general_8m_tiltcomp_speed_axis.csv vs general_8m_speed_axis.csv), absolute rates (% of all
episodes), mean over 6 speeds:

  capture       0.837 -> 0.837   (EXACTLY flat -- the user predicted this)
  below (floor)  1.82% -> 0.50%  (-1.31pp -- the tilt comp worked as designed & measured)
  oob   (wall)   0.89% -> 2.38%  (+1.49pp -- the failure MOVED here)
  bar_contact   ~12.2% -> ~13.1% (unchanged, still dominant)

Interpretation -- CONSERVATION OF FAILURE: the tilt comp removed the floor-strike sag exactly as
designed, but fixing below let the policy bank harder (it fine-tuned into the freed aggression), and
that harder banking + the tilt-comp's lateral thrust leak pushed it out of the arena instead. below
and oob traded ~1:1; capture did not move. The constraint relocated from floor to wall.

THE decisive finding: below (0.5%) and oob (2.4%) are BOTH small. bar_contact (~13%) is 4-5x larger
than both combined and was never touched by any altitude work. That is why the altitude PI and the
tilt comp -- both physically correct and both validated on their own metric -- produced ZERO net
capture gain. We have been polishing secondary failure modes. To move capture off ~0.84 we must
attack bar_contact, which is geometric/perceptual (drone hits bars deep in the field at mean_x
~12 m), NOT altitude.

Keep the tilt comp + PI: they are the correct altitude control and the foundation for higher-density
sweeps (weaving intensifies with density -> floor strikes return without them). The oob regression is
tracked separately (candidate 3). But the NEXT capture lever is bar_contact.

## PIVOT to bar_contact: candidate 2 (LiDAR look-ahead 8 -> 12 m)

Now unlocked -- altitude is solid (below ~0.5%), so extending look-ahead can no longer regress into
floor strikes (that was the entire reason altitude came first). Warm-start from the tilt-comp policy
(1052) with NAVRL_LIDAR_RANGE=12 and re-run the 6-speed general eval; the direct question is whether
bar_contact (~13%) drops. Caveat: extending the LiDAR range changes the static-scan normalization
(scan/range), so warm-start needs a re-adaptation window -- watch for a transient capture dip that
recovers. If bar_contact does NOT drop, the limiter is not look-ahead (8 m = 3.2 s at 2.5 m/s is
already generous for a single dodge) but obstacle-token capacity (MAX_OBSTACLES=5, candidate 4) or
path-planning through the field -- diagnose crowding-at-contact before spending another train there.
