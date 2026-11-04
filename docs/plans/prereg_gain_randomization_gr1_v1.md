# Preregistration: control-gain randomization, GR-1 — training Policy F's successor with randomized gains

Fixed 2026-11-04, before any GR-1 training or evaluation. Decision D-2026-11-04-1 (new PPO allowed again; the user
approves this unit's GPU runs). Branch `gainrand/dev`.

**Entry condition.** GR-0 ([`prereg_gain_randomization_gr0_v1.md`](prereg_gain_randomization_gr0_v1.md), amendment
1; evidence [`docs/evidence/gain_randomization_2026-11-04/`](../evidence/gain_randomization_2026-11-04/README.md))
read **GAIN_SENSITIVE** at ±40 %: crash +3.69 pp [+3.03, +4.37], capture -3.69 pp. Its preregistered next-step rule
makes GR-1 motivated. GR-1 trains only after GR-0's independent review finds no blocking issue.

**Standing prohibition.** `docs/discipline_review_2026-08-22.md` forbids extending fixed-density PPO training to paper
over a failure, unless a non-density bottleneck is identified and the remedy is preregistered. GR-0 identified one:
sensitivity to controller gains, measured at fixed settings. GR-1 is that preregistered remedy, with a matched
control arm that gets the same extra training without randomization.

## Training (two arms, `tools/gr1_train.py` → `aerial_gym/rl_training/rl_games/train_navrl_gr1.sh`)

Both arms continue Frozen Policy F (`last_gen_ppo_ep_25000_rew_39.742134.pth`, SHA-256 `f7022139…`) for 1,000 epochs,
from epoch 25001 to 26000 (4,096,000 samples). Each branches into a new `runs/` folder; F itself is never modified.

- **Contract:** the one that produced F (`train_navrl_v2_ep24000_speed_governor.sh`): fixed 205 bars, curriculum off,
  seed 1, LR 5e-6, main profile (128 envs), and the riskcap governor parameters re-read from the same validated
  screen summary.
- **Isolation:** `PYTHONPATH` is pinned to the tree, and the launcher refuses unless `aerial_gym` resolves inside it.

| Arm | Gains during training | Run tag |
| --- | --- | --- |
| `rand40` | `NAVRL_GAIN_RAND=1`, scale 0.4 (the S40 range), gain seed 1 | `v2-gr1-ep25000-205bars-main-riskcap-rand40-s1` |
| `ctrl` | OFF (nominal) | `v2-gr1-ep25000-205bars-main-riskcap-ctrl-s1` |

The wrapper's preflight requires:
- this preregistration committed, approved commit = HEAD, clean tree;
- one RTX 3070 with no other GPU compute or NavRL training process;
- the Policy F SHA.

Each arm has a 3-hour wall-clock cap. Afterwards:
- kernel fault record, boot id and online CPU list unchanged (otherwise VOID_HOST_FAULT);
- exactly one new run folder with the arm's tag, containing `last_gen_ppo_ep_26000_*.pth`;
- the checkpoint is copied, with its SHA-256 and the session log, into
  `aerial_gym_simulator/results/gain_randomization_gr1_v1/train_<arm>/`.

Any other outcome is TRAINING_FAIL. It is recorded, and the unit goes back to the user; there is no automatic
retraining. `rand40` trains first, then `ctrl`.

## Evaluation (`tools/gr_run.py`)

- **Source restored before GR-1.** GR-0's commit also wrote `cfg_gain_rand*` into result.json's `condition` through
  `navrl_task.py`. Three H-R3 tests pin that file's bytes, and they failed on the GR-0 commit. GR-1 therefore restores
  `navrl_task.py` to the base bytes (`62e59c0`), so the switch is recorded only in the receipt's `NAVRL_GAIN_RAND*`
  snapshot, which the runner checks. GR-1 cells must carry no `cfg_gain_rand*` condition field. The controller and
  config changes, which act on the run, are unchanged.
- **Reproduction check (`gr1repro`).** On the GR-1 commit, Policy F reruns the GR-0 cell S40 / 205 bars / seed 4501
  (quota 16). Its result.json, without the `cfg_gain_rand*` fields, and every episode row must equal the GR-0 cell:
  GR1_REPRO_PASS. This shows that GR-0's Policy F cells remain the F arm and that the restored source changes
  nothing but the record. If it fails, the unit stops.
- **GR-1 cells (`gr1eval`, per trained checkpoint).**
  - The cells, levels (S00 / S20 / S40), seeds (4501-4503), quota, A3 placement and all checks are exactly GR-0's.
  - The launcher is `eval_navrl_gr1_arms.sh`, the pinned evaluation launcher with only the checkpoint path and
    expected SHA taken from the caller.
  - Each receipt's `checkpoint_sha256` must equal the trained checkpoint's.
  - 36 cells per checkpoint, about 1.5 GPU-h each.

## Analysis (`tools/gr1_analyse.py`, fixed here)

Quota rates; blocks are the 12 (density, seed) pairs. 95 % CIs are percentile bootstraps over blocks (10,000
resamples, seed 20,261,111).

- **P1 (robustness):** the block mean of crash(rand40, S40) - crash(ctrl, S40).
- **P2 (nominal cost):** the block mean of capture(rand40, S00) - capture(ctrl, S00).

Reading (first match wins):

| Reading | Rule |
| --- | --- |
| **GR1_MATERIAL_LOSS** | P2 <= -2 pp and CI(P2) upper < 0 |
| **GR1_SUPPORTED** | P1 <= -1.5 pp and CI(P1) upper < 0 and CI(P2) lower > -2 pp |
| **GR1_NOT_SUPPORTED** | CI(P1) lower > -1.5 pp |
| **INCONCLUSIVE** | otherwise |

Reported, not gated:
- every policy's pooled and per-density rates at each level;
- rand40 - F and ctrl - F at each level (F from GR-0);
- each policy's S40 - S00 crash gap.

Labels stay as computed until an independent review.

## Limitations fixed in advance

- **One seed per arm.** Training variance is not measured, so a GR1_SUPPORTED reading is evidence for this
  pair of runs, not a population claim.
- **Narrow training setup.** Training uses fixed 205 bars and continues F for 1,000 epochs; evaluation covers 70-205
  bars.
- **Gains are a proxy for platform variation.** Mass, inertia and motor strength stay nominal, so this is not an
  airframe-transfer claim. The 2026-08 ref5in adaptation is not retested here.
- **No changes outside the gains.** The actor observation (898-D), the network, the governor and the reward are
  unchanged.
- **Status of the new checkpoints.** Frozen Policy F stays the baseline of every other study. A GR-1 checkpoint
  replaces nothing without a separate user decision.

## Disclosed development (not evidence)

- `NAVRL_PREFLIGHT_ONLY=1 train_navrl_gr1.sh rand40` (contract preflight, no training): PASS. The checks were
  continuation mode, 205 -> 205 bars, LR 5e-6, the riskcap governor, 128 envs and seed 1.
- No GR-1 GPU run before this preregistration.
