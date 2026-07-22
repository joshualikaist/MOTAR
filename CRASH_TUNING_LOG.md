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
