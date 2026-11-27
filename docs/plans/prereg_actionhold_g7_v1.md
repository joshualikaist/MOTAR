# Preregistration: G7 — does Policy F tolerate held or delayed actions? (v1, 2026-11-05)

Branch `camonly/dev`.
- **Hook:** `tools/actionhold_hook.py` and `tools/actionhold_site/sitecustomize.py`, opt-in, default OFF.
- **Runner and analysis:** `tools/actionhold_run.py`, `tools/actionhold_analyse.py`.
- **Approval:** the user's GPU session of 2026-11-05 ("이거 하고 gpu 세션 바로 시작하자"). Its queue (Beads `motar-g9a`)
  names this run as G7.
- **Literature:** `~/motar_tmp/action_chunking_20261105/ACTION_CHUNKING_literature.md`, candidate 1.

## 1. Questions

1. **Action chunking premise.** Can F, unchanged, fly with open-loop chunks of k = 2 or 4 steps (0.2 s / 0.4 s)?
   - If even k = 2 is harmful, open-loop chunking at MOTAR's 10 Hz is ruled out for this policy.
2. **Latency tolerance for PX4.** Does a one-step (100 ms) command delay (d = 1) hurt F?
   - This sizes the latency budget of the SITL/real bridge.

These are measurements of F's tolerance. They are not a test of a policy trained with chunks.

## 2. Design

- **Path:** the unchanged camonly cell path, condition `full` (native F), GR-0's grid condition (E2, A3, 128 envs,
  quota 16).
- **Hook:** the opt-in hook wraps the player's `get_action` and changes only the returned action.
  - The governor, controller, altitude PI and recorders are unchanged; the governor still acts every step.
  - **Hold k:** latch at the episode start and every k-th step.
  - **Delay d:** return the action from d steps earlier; the zero action (hover) for an episode's first d steps.
- **Cells, in order:**

  | Bars, seed | Arms |
  | --- | --- |
  | 205/4501 | k1d0 (identity control), k2d0, k4d0, k1d1 |
  | 70/4501 | k2d0, k4d0, k1d1 |

- **Identity gate:** k1d0 runs with the hook installed and returning the action unchanged. It must equal GR-0's S00
  205/4501 cell (`episodes.jsonl` byte-identical and `result_equal`); otherwise HOOK_IDENTITY_FAIL and the unit
  stops.
- **Sidecar check:** every other cell must carry the hook's sidecar with the patch applied, its config, and a
  non-zero count of held or delayed steps.

## 3. Analysis (fixed here)

- **Measure:** quota crash and capture per cell. The F reference is GR-0's S00 cell with the same density and seed.
- **Pooled crash difference per arm:** over its 70- and 205-bar cells, episode-weighted. 95 % percentile bootstrap,
  episodes resampled within each cell for the arm and F independently, 10,000 resamples, seed 20261105.
- **Reading per arm** (crash, pp):

  | Reading | Rule |
  | --- | --- |
  | **TOLERANT** | Δ ≤ +2 and CI upper ≤ +4 |
  | **HARMFUL** | Δ ≥ +5 and CI lower > 0 |
  | **INCONCLUSIVE** | otherwise |

- **What follows:**
  - k2d0 HARMFUL → no open-loop chunking at 10 Hz for F.
  - k2d0 TOLERANT → the chunking premise is not ruled out. Training a chunked policy would still be a new,
    separately approved unit.
  - k1d1 HARMFUL → the bridge must keep command latency well under 100 ms, or the policy needs latency-aware
    training.
- **Reported, not gated:** per-cell rates, capture differences, per-density deltas.

## 4. Checks before the run

- **CPU unit tests** (`tests/test_actionhold_hook.py`, 8 tests):
  - the identity returns the same tensor;
  - hold latches every k steps and at episode starts;
  - delay by d with a hover start, cleared at episode starts;
  - config validation;
  - the patch wraps and keeps the original.
- **Analysis smoke on stand-in cells:**
  - `no_lidar` cells in place of k2 read HARMFUL (+38.0 pp, the positive control);
  - F's own cells read TOLERANT (0.0 pp);
  - `no_lidar_in_fov` reads TOLERANT.
- **Integration:** the identity cell is the integration test on the GPU (the hook cannot load without Isaac).

## 5. Limits fixed in advance

- **Scope:** one seed, two densities, F unchanged.
- **Hold:** it is harsher than a trained chunk policy, so it gives an upper bound on harm.
- **Delay:** d = 1 is the only latency tested.

## 6. Cost

7 cells at 2-3 min each, about 20 min of GPU.
