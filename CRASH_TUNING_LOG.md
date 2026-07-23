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
