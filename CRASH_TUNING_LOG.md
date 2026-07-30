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

## Candidate #2 RESULT: look-ahead is NOT the bar_contact lever (2026-07-24)

Trained the 12 m look-ahead run (1052 -> 1230). It PEAKED at capture 95.2% / reward 111 @ epoch 6145,
then COLLAPSED around epoch ~6700: crashdiag oob exploded 0.10 -> 0.60 spread uniformly over W/E/S/N
(drone lost heading entirely and flew out of the arena in every direction), n_crash 350 -> 900, final
capture 0.558 @ ep7000. The best-reward `gen_ppo.pth` predates the collapse and is the valid policy.

Likely collapse trigger: changing NAVRL_LIDAR_RANGE 8 -> 12 changes the static-scan normalization
(scan/range) while the warm-started checkpoint's `running_mean_std` still encodes 8 m statistics. As
those input-normalization stats slowly re-adapt mid-training, the network's effective input scale
drifts under it -> at some point the policy misreads the scan -> navigation lost -> on-policy PPO
spirals on the bad data. (Consistent with the ~200-epoch delay after peak; not proven.)

Eval of the surviving peak (results/general_12m_lookahead_speed_axis.csv), 6-speed general-spawn
deterministic, absolute rates (% of all episodes), mean over 6 speeds, vs the 8 m tilt-comp policy:

  capture       0.837 -> 0.856   (+1.8pp -- best so far)
  crash         0.160 -> 0.139
  oob            2.4% -> 1.2%    (-1.2pp  <- where the gain actually came from)
  bar_contact   13.1% -> 12.6%   (-0.5pp  <- the INTENDED target, essentially unmoved)

CONCLUSION: extending look-ahead did NOT reduce bar_contact. 8 m was already 3.2 s of warning at
2.5 m/s -- ample for a single dodge -- so more range only helped the drone stop wandering out of the
arena (oob), which is a small pool. Together with the altitude campaign this now gives three
independent negatives on the SAME number:

  altitude PI     -> bar_contact ~13%  (unmoved)
  tilt comp       -> bar_contact ~13%  (unmoved)
  look-ahead 12 m -> bar_contact ~13%  (unmoved)

bar_contact (~13% of ALL episodes, mean_x ~12.4 m = deep inside the bar field) is immune to both
altitude authority and sensing range. It is the entire remaining gap to a higher capture rate and it
must now be diagnosed DIRECTLY rather than attacked with another guessed lever.

Remaining hypotheses (mutually distinguishable by measurement, NOT by another training run):
  H1 OBSTACLE TOKEN CAPACITY -- MAX_OBSTACLES=5 (navrl_perception.py). Deep in the field more than 5
     bars are within range; bars 6+ are truncated out of the policy input entirely, so the drone
     cannot avoid what it is never shown.
  H2 PERCEPTION/TRACK QUALITY -- the KF track position for the hit bar is off by enough that the
     drone plans around a phantom and clips the real one.

DIAGNOSTIC (next step, no retraining): at each bar_contact instant record (a) how many bars are
within the perception radius (crowding), (b) whether the bar actually hit was among the MAX_OBSTACLES
tokens fed to the policy, (c) the track-vs-true position error of that bar. H1 predicts high crowding
+ hit bar frequently NOT in the token set; H2 predicts low crowding + hit bar present in tokens but
with large track error. This is the same measure-first discipline that correctly identified the
below/tilt mechanism.

## bar_contact DIAGNOSED: both H1 and H2 confirmed, plus a hidden design ceiling (2026-07-24)

New probe `NAVRL_BAR_PROBE=1` (evaluation-only; uses GT bar positions but nothing it computes
reaches actor/critic/reward/termination). At every bar_contact death it records scene crowding, and
whether the bar that actually hit was represented among the MAX_OBSTACLES obstacle tokens.

Run on the 12 m peak policy (general spawn, target 1.0 m/s, n=266 contact deaths):

  bars_in_range = 15.8    GT bars inside the LiDAR horizon at impact   (capacity = 5)
  occupied_bins = 19.5    of 36 scan bearings returning an obstacle
  hit_in_tokens = 0.647   fraction of struck bars that HAD a matching token
  token_err     = 0.57 m  position error of the matched token vs the true bar
  token_rank    = 0.9     matched hits sat in the nearest slots, not the far ones
  hit_dist      = 0.56 m  distance to bar center at contact

H1 (CAPACITY) CONFIRMED, strongly: 15.8 bars in range vs a capacity of 5, and **35% of all
bar_contact deaths are collisions with a bar that was never in the policy's input at all**. The
drone cannot avoid what it is never shown.

H2 (TRACK QUALITY) CONFIRMED too, and its root cause is NOT the tracker: for the 65% that WERE
represented, the token was off by 0.57 m -- comparable to the free gap width (drone collision box
0.28 m, bar radius 0.2-0.4 m). That magnitude is explained by ANGULAR QUANTIZATION alone: 36
horizontal bins = 10 deg each, so at ~5 m a half-bin error is ~0.44 m. The token positions are built
from range/angle geometry (`_fuse_static_and_extract_obstacles`), so their lateral accuracy is capped
by the beam count, not by filtering.

HIDDEN CEILING (found while designing the fix): each accepted token blanks +-2 bins around itself
(`for off in (-2,-1,0,1,2)`) = 50 deg of the 360 deg scan. 360/50 = 7.2, so **no more than ~7 tokens
can ever be populated regardless of MAX_OBSTACLES** -- raising the capacity to 8+ without narrowing
the suppression window silently wastes the extra slots.

=> The fix is a package of three, all of which change the observation layout and therefore require a
   FRESH policy (no warm-start):
     1. MAX_OBSTACLES 5 -> 8            (H1: represent more of the 15.8 bars in range)
     2. suppression +-2 -> +-1 bins     (H1: otherwise slots 8+ can never fill)
     3. horizontal beams 36 -> 72       (H2: halve the 10 deg quantization driving the 0.57 m error)
   This is the first bar_contact intervention in the whole campaign that is grounded in measurement
   rather than a guessed lever -- the altitude PI, the tilt compensation and the 12 m look-ahead all
   left bar_contact at ~13% because none of them touched the obstacle REPRESENTATION.

## 85-bar follow-up: duplicate-token bottleneck and PPO safety gap (2026-07-30)

The corrected bearing/chirality and bounded-action policy moved the density limit from 65 to 85
bars at effectively unchanged plateau capture (`0.678` at 65 versus `0.676±0.001` at 85). A held-out
density sweep also improved at every measured cell. The remaining 85-bar failures are dominated by
bar contact, not altitude or commanded stall.

Probe v2 resolves the next representation bottleneck more precisely than the original capacity
hypothesis. At 85 bars, roughly 35 bars fall inside the 240° selection FOV and all 8 token slots are
filled, but only about 3 slots correspond to unique bars; about 2.7–3.0 associated slots duplicate a
bar already represented. A near 0.8 m-wide bar at 2 m subtends about ±11°, wider than the baseline
±10° suppression half-window. One physical bar can therefore consume two selections.

The controlled intervention is suppression `±10°→±15°`. It keeps the 898-D observation shape and
can warm-start, but changes selection semantics, so the checkpoint preflight requires an explicit
override and records it. Success requires all three:

1. unique represented bars `3.0→4.5+`;
2. 85-bar capture `≥0.70` over the 16,384-episode promotion window;
3. PPO KL `≤0.04` with no latent-mean/action-edge explosion.

An earlier attempted run did **not** test this intervention: the density launcher overwrote the
external 15° request with 10°. That mistaken continuation stayed near 64–67% capture through epoch
10250, then crossed KL 0.04 at epoch 10276. Latent means exploded (`|mu_x|` eventually >2000,
`|mu_z|` >460), x/z/yaw actions saturated at the tanh boundary, and tail-500 capture/crash became
1.0%/86.0%. The minibatch KL gate could skip later updates but could not roll back the already
damaged weights; the global reward-collapse guard was disabled for density curricula. This run is
discarded as a PPO safety failure, not evidence against suppression 15°.

The real suppress-15 run starts explicitly from pre-collapse epoch 8350. Its first probe moved
unique `3.0→3.5` and duplicate `~3.0→1.9`; this is directionally correct but below the registered
coverage gate and is not yet a success. Before any further long density run, add a collapse guard
that compares reward only within a fixed density and a KL/latent fail-stop or last-known-good
checkpoint rollback.

---

# 부록 — Phase-1 GT-goal 시절 clearance 보상 튜닝 (B/C/D), 2026-07-14

> 원래 `aerial_gym/rl_training/rl_games/CRASH_TUNING_LOG.md`에 있던 내용.
> **`navrl_task_config.py:390`과 `navrl_task.py:2221`의 코드 주석이 이 음성 결과를 참조**하므로
> 파일 통합 시 여기로 옮겨 보존한다. 결론: 보상으로 충돌을 못 줄인다(기하 문제).

> `PERCEPTION_TRANSFORMER_PLAN.md`를 따른다.

목표: crash ~14%(먼 18m 목표로 가는 transit 중 막대 충돌)를 captured 손실 없이 줄이기.
고정 변수: safety_static_weight 1.5, 아레나 24×24, 기둥 48, 커리큘럼 18m, episode 250, num_envs 256, 6000 epoch.
지표는 마지막 500 epoch 평균(안정 구간). 기준 = captured↑ / crash↓ / timeout 0 유지.

## Baseline — `ppo_260714_0153` (clearance OFF)
- captured **0.863**, crash **0.137**, timeout 0.0
- 참고: `ppo_260713_2210`(safety 1.0)은 captured 0.86 / crash 0.14 → **safety 가중치 1.0→1.5는 crash에 무효**.

## Run B — `ppo_260714_0346` (clearance 거리 모드, weight 1.5, margin 0.6, speed-gate OFF)
- captured **0.861**, crash **0.139**, timeout 0.0, closest(no-crash) 0.406, best 0.222
- **판정: crash 저감 실패** (0.139 vs baseline 0.137, 사실상 동일).
- 원인: 페널티가 속도 유인 대비 약함 — 막대 0.3m 앞에서 `1.5×(0.6−0.3)=0.45/step` < 속도 리워드 +2. 스쳐도 순이득이라 회피 유인 부족.

## Run C — `ppo_260714_0555` (clearance speed-gated, weight 1.5, margin 0.6, speed-gate ON)
- captured **0.861**, crash **0.138**, timeout 0.002, closest(no-crash) 0.408, best 0.239
- **판정: crash 저감 실패** (0.138 vs baseline 0.137). 속도-게이트로 페널티가 2배(0.9/step) 세졌지만 여전히 속도 리워드 +2보다 작아 무효.

## 결론 (세 run 종합)

| run | 변경 | captured | crash | timeout |
|---|---|---|---|---|
| 2210 | safety 1.0 | 0.860 | 0.140 | 0.0 |
| 0153 (baseline) | safety 1.5 | 0.863 | 0.137 | 0.0 |
| B (0346) | + clearance 거리 1.5 | 0.861 | 0.139 | 0.0 |
| C (0555) | + clearance speed-gated 1.5 | 0.861 | 0.138 | 0.002 |

**모두 동일 (captured ~0.86 / crash ~0.14).** "장애물 근접 페널티"는 가중치 1.5로는 어떤 형태(soft log / hard 거리 / hard 속도게이트)든 crash를 못 움직임 — **페널티가 속도 유인(+2/step)보다 약한 게 근본 원인.**

**중요 관찰**: captured가 초기(커리큘럼 ~5m 근거리)엔 0.97(crash 3%), full 18m에선 0.86(crash 14%). → **crash는 먼 목표로 가는 긴 transit(48기둥 통과)에 집중**. 거리×밀도가 만드는 hard tail일 수 있음.

**다음 후보** (블라인드 리워드 튜닝은 3run으로 소진 — 이제 LOOK):
1. **뷰어로 14% 실패 재확인** (수분) — "돌진 충돌"인지 "좁은 틈 갇힘"인지. 처방이 갈림.
2. 돌진이면 → clearance_weight 4~5 + margin 1.0 (훨씬 강하게) 한 판.
3. 갇힘/hard-geometry면 → max_velocity↓(반응시간) 또는 LiDAR range↑, 또는 수용.
4. **86% 수용** — NavRL hybrid 80.96% 상회, Phase 1 검증 목적 달성. Phase 2로.

clearance는 무효로 판명 → config를 baseline(off)로 복원함.

---

## ⚠️ 위 3run(0153/0346/0555)은 옛 스킴 결과 — 새 스킴에선 전제가 달라짐 (2026-07-14)

위 실험들은 **옛 스폰 스킴**(목표가 드론 근처에 몰려 막대밭 관통이 8%뿐) + **0.05m 구 충돌체**에서 나온
결과다. 그래서 "clearance 무효"의 진짜 원인은 두 가지가 섞여 있었다: (a) 막대가 애초에 거의 무관했고,
(b) 페널티(cw 1.5)가 속도 유인(+2/step)보다 약했다. **(a)는 새 스킴(매 에피소드 48기둥 관통 + 0.28m 박스,
run 1904)에서 사라졌지만 (b)는 구조적 사실로 남는다.** 따라서 clearance를 재시도하되, (b)를 정면으로
깨는 강도로 건다.

## Run D (예정) — `ppo_2607??_????` : 새 스킴 + speed-gated clearance **cw=6.0, margin=0.5**

- **베이스라인 = 1904** (새 스킴, 축정렬 48기둥, 0.28m 박스): captured 0.65 / crash 0.35 / timeout 0.
- **바꾼 것(1레버, config 3값, 코드변경 없음)**: `clearance_speed_gated False→True`,
  `clearance_weight 0.0→6.0`, `clearance_margin 0.6→0.5`. 그 외(safety 1.5, 커리큘럼, episode 300,
  256 envs, 6000 epoch) 전부 동일 → 결과가 이 레버에 깔끔히 귀속.
- **설계 근거(멀티에이전트 워크플로우: 진단 3렌즈 → 설계 4 → 반박검증 3/4 생존 → 종합)**:
  - **실패모드 = "순항속도로 막대 스치기"(charging/shave-at-cruise-speed)** — 3개 진단 렌즈(리워드수학·
    기하·지각제어)가 모두 dominant로 지목. 드론은 막대를 **보고도** 스친다(closest_nocrash 0.43m가 증거):
    스쳐도 매 step 순이득(+1.2/step)이라 감속·중앙정렬 유인이 없음.
  - **왜 cw=6인가**: d=0.30m 스침·v=2에서 페널티 `6*relu(0.5-0.30)*2 = 2.40/step > 속도리워드 2.0`.
    즉 유효 속도계수 `(1 - 6*relu(0.5-d))`가 d<0.333m에서 **음수** → "빠르게 막대로" 가 처벌됨(단순 상쇄가
    아니라 부호반전). 실패한 cw=1.5의 약 4배(2.40 vs 0.75).
  - **왜 margin=0.5인가**: 최악 갭(0.8m 축정렬 기둥 2개 @1.8m) 자유폭 ~1.0m → 중앙통과 min_dist ~0.5m →
    `relu(0.5-0.5)=0` → **정상 통과는 비용 0(1904와 동일)**. 벗어난/스치는 통과(d<0.5)만 과세.
  - **왜 speed-gated인가**: 페널티가 |v|에 비례 → 좁은 갭도 **감속하면 통과 가능**(회피·정지 아님).
    거리모드였다면 좁은 갭에서 무조건 감점 → detour/timeout 위험.
- **예상**: crash 0.35 → **~0.15-0.22**(점추정 0.18), captured ~0.60-0.70 유지(정상통과 경제성 불변),
  timeout ~0.01-0.03(300 cap 훨씬 아래). **판정 = crash↓ AND captured 유지(둘 다)**.
- **LOOK-first 생략 근거**: 3렌즈가 실패모드에 수렴 + d≤0.30 위험구간에서 부호반전이 robust. 다만 기하
  렌즈는 corner-clip(축정렬 기하 바닥)도 co-dominant로 봄 → crash가 0.18까지 안 내려가고 ~0.25에서
  멎으면 그 잔여분은 reward가 아닌 기하/지각 한계 신호(다음 레버: 속도governor 또는 LiDAR 해상도↑).
- **폴백**: captured가 5pt↑ 하락 → margin 0.5→0.45(cw는 5.0 밑으로 내리지 말 것). timeout>0.05 →
  거리모드 1/d 배리어(cw=2.0, margin=0.45, speed-gate OFF)로 전환.

### Run D 실제 결과 — `ppo_260714_2207` (6000 epoch 완료, cw=6 확정: 커밋 71aa606 22:02 → run 22:05)

| 지표(last-500) | 1904(baseline) | **Run D(2207)** | Δ |
|---|---|---|---|
| captured | 0.653 (peak 0.802) | **0.663 (peak 0.833)** | +0.01 (flat) |
| crash | 0.347 (min 0.198) | **0.316 (min 0.162)** | −0.03 (미미) |
| timeout | 0.000 | **0.021** | +0.02 (등장) |
| closest_nocrash | 0.428 | 0.440 | flat |
| mean_ep_len | 88 | **116** | **+32% (감속)** |

- **판정: 예측 실패(crash 0.18 예상 → 실제 0.32).** speed-gate는 **작동함**(드론 32% 감속 + timeout 등장이 증거)
  이나 **crash를 거의 못 줄임.** captured도 flat.
- ⚠️ **교란**: Run D는 cw=0→6 **뿐 아니라 k_min_final 10→20**(더 깊은 목표)도 바뀜 → 순수 cw 효과 아님.
  더 어려운 커리큘럼에서 crash가 그래도 소폭↓ = cw가 어려워진 만큼을 상쇄했을 순 있음. 하지만 clean 아님.
- **결정적 해석**: **큰 감속(88→116)에도 crash 불변** ⇒ crash는 speed/reward-shaving이 아니라 **기하/지각
  (corner-clip: 10° 빔각으로 갭 중앙정렬 정밀도가 반클리어런스와 비슷 → 감속해도 못 고침).** 기하 렌즈의
  co-dominant 진단이 옳았고 리워드-수학 렌즈의 "순항 스침" 베팅은 빗나감.
- **결론: 리워드로 crash 줄이기 소진(B/C/D 모두 실패).** 다음은 리워드가 아님:
  1. **지각**: LiDAR 36→72빔(갭 중앙정렬 정밀도↑, corner-clip 직접 공격). 2. **요 제어**: 대각 footprint
     0.38m→0.28m. 3. **수용**: ~0.66-0.83은 NavRL 0.81 상회 → Phase1 종료하고 Phase2(밀도)로(critical path).
- **cw=6 유지 여부 미정**: 감속+timeout 부작용 vs 미미한 안전이득. Phase2는 clean baseline(cw=0) 권장 검토.
