# Preregistration: control-gain randomization, GR-0 — Policy F under randomized controller gains

Fixed 2026-11-04 before any GPU run of this unit. Decision: D-2026-11-04-1 in [`docs/DECISIONS.md`](../DECISIONS.md)
(new PPO allowed again; the user approves the GPU runs of this unit). Branch `gainrand/dev`, worktree
`src/MOTAR-gainrand`, base `62e59c0` (`codex/mt-p1`).

**Question.** Every Policy F result so far comes from one airframe (`navrl_quad`) with fixed Lee-controller gains.
Those are the midpoints K_vel 2.5, K_rot (1.0, 1.0, 0.5) and K_angvel 0.15, with `randomize_params = False`.
How much do F's capture and crash rates change when the gains vary from episode to episode? The answer decides
whether training with gain randomization (GR-1, a separate preregistration) is motivated. Policy F, its checkpoint,
the riskcap governor and every other setting are unchanged.

## Opt-in switch (default OFF)

- `aerial_gym/config/controller_config/lee_controller_config_navrl.py` reads three variables.
  - `NAVRL_GAIN_RAND` unset or `0`: `GAIN_RAND = None`. The config class is the historical one, with no new
    attributes.
  - `1`: `randomize_params = True` plus `gain_rand_scale` from `NAVRL_GAIN_RAND_SCALE`, in [0, 1), and
    `gain_rand_seed` from `NAVRL_GAIN_RAND_SEED`, >= 0. Both are required; any other value refuses.
- `aerial_gym/control/controllers/base_lee_controller.py`, only when the config defines `gain_rand_scale`:
  - at every robot reset (`base_multirotor.reset_idx` already calls `randomize_params`), each element of the reset
    envs' K_vel, K_rot and K_angvel becomes nominal x (1 + scale x (2u - 1));
  - u ~ U[0, 1) comes from a dedicated `torch.Generator` seeded once, so the global RNG stream is untouched;
  - K_pos (unused by velocity control) is untouched.
- `aerial_gym/task/navrl_task/navrl_task.py` adds `cfg_gain_rand`, `cfg_gain_rand_scale` and `cfg_gain_rand_seed` to
  the env state and to the bulk-evaluation `condition`, only when ON (an empty dict is unpacked when OFF).
- `tests/test_gain_randomization.py` (CPU, in child processes) checks OFF = historical config, the parsing and
  refusals, bounded seeded draws, global RNG untouched, non-reset envs kept nominal, scale 0 bitwise nominal.

## Runner and settings

`tools/gr_run.py` runs the KROC2027 launcher byte for byte through `tools/kroc2027_eval.py` helpers. Every cell uses:
- a scrubbed environment (every inherited `NAVRL_*` removed), with `PYTHONPATH` set to the tree;
- arm `E2_obstacle_aware` and the motion recorder;
- the quota ledger: 128 envs x the first 16 episodes per env, i.e. 2,048 quota episodes per cell, as in KROC2027;
- `NAVRL_TARGET_DISTANCE_BACKEND = direct_l2_v1`, the TF32-safe target distance (D-2026-10-16-0), as in B1-R.

Fail-closed preflight:
- branch `gainrand/dev`, approved commit = HEAD, clean tree, this preregistration committed;
- launcher, player and checkpoint SHA-256 pinned (`f7022139...`);
- exactly one RTX 3070 and no other GPU compute process;
- `aerial_gym` and `motar_eval` resolve inside the tree.

After every cell:
- kernel fault record, boot id and online CPU list unchanged; otherwise VOID_HOST_FAULT and stop;
- receipt `status` OK, with this commit and 0 dirty files;
- the recorded `NAVRL_GAIN_RAND*` and `condition.cfg_gain_rand*` equal the level's values, and are absent when OFF;
- ledger complete, import origins inside the tree, recorder violations empty, tree still clean.

The runner stops at the first failing cell. No heavy CPU work runs beside it.

## Stage 1 — identity (OFF reproduces the native run; ON path exercised)

One cell, `E2_obstacle_aware`, 160 bars, seed 4501, quota 2 (256 episodes), run four times:

| Run | Tree | Gains |
| --- | --- | --- |
| `ref` | `src/MOTAR-mt-p1` at `62e59c0` (before this unit's change) | OFF |
| `off` | this tree | OFF |
| `s00` | this tree | `NAVRL_GAIN_RAND=1`, scale 0.0, seed 4501 |
| `s40` | this tree | `NAVRL_GAIN_RAND=1`, scale 0.4, seed 4501 |

Comparisons cover `result.json` (without the run nonce, the absolute checkpoint path and, for `s00` / `s40`, the
`cfg_gain_rand*` fields) and every row of `episodes.jsonl`.

**IDENTITY_PASS** iff:
- `off` equals `ref`;
- `s00` equals `off`;
- `s40`'s episodes differ from `off`'s (positive control: the switch acts).

Anything else is IDENTITY_FAIL, which stops the unit.

## Stage 2 — GR-0

Requires IDENTITY_PASS on the same commit. Policy F, `E2_obstacle_aware`, densities 70 / 115 / 160 / 205 bars, seeds
4501 / 4502 / 4503 (fresh), gain levels:

| Level | Setting | Range of every K_vel / K_rot / K_angvel element |
| --- | --- | --- |
| S00 | OFF | nominal |
| S20 | scale 0.2, seed = cell seed | nominal x [0.8, 1.2] (about the upstream Aerial Gym ranges) |
| S40 | scale 0.4, seed = cell seed | nominal x [0.6, 1.4] |

The run has 36 cells, in the order density, then seed, then level. Expected cost is about 2 GPU-h, capped at
`--max-hours 4`.

**Analysis** (`tools/gr_analyse.py`, fixed here):
- Blocks are the 12 (density, seed) pairs.
- For S20 and S40, delta_capture and delta_crash are the mean over blocks of (level - S00) quota rates, in
  percentage points. 95 % CIs are percentile bootstraps over blocks (10,000 resamples, seed 20,261,110).

Per-level reading:
- **GAIN_SENSITIVE**: (delta_capture <= -3 pp and its CI upper bound < 0) or (delta_crash >= +2 pp and its CI lower
  bound > 0).
- **GAIN_ROBUST**: CI(delta_capture) inside [-3, +3] pp and CI(delta_crash) inside [-2, +2] pp.
- **INCONCLUSIVE**: otherwise.

Also reported: pooled and per-density capture / crash / timeout rates per level.

**Next-step rule.** If S20 or S40 reads GAIN_SENSITIVE, training with gain randomization (GR-1) is motivated and
gets its own preregistration. If both read GAIN_ROBUST, gain variation within ±40 % alone does not motivate
retraining. Broader platform variation (mass, inertia, motor strength) would then be the next question for the user.
The label stays at most as computed until an independent review.

## Limitations fixed in advance

- Controller gains are a proxy for platform variation. Mass, inertia, motor strength and drag stay nominal. The motor
  thrust constant is already redrawn per reset, but a code comment says that has no effect.
- Gains are drawn uniformly per element and per reset, independently of each other. Real airframes correlate them.
- One target arm (E2), the `navrl_quad` airframe, simulation only; no real-flight claim.
- Cells are independent samples blocked by seed label (KROC2027 UNPAIRED_STREAM). Episodes are not paired across
  levels once trajectories diverge.

## Disclosed development (not evidence)

- `tests/test_gain_randomization.py`: 5/5 pass on the uncommitted tree.
- A child-process import probe (path-only package stubs): OFF has no new attributes; ON parses (0.4, 7).
- No GPU run before this preregistration.
