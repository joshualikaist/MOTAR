# Amendment 1 to the GR-1 preregistration: bindings, claim, controls and next-step rules (before any GR-1 evaluation)

Dated 2026-11-04. Amends [`prereg_gain_randomization_gr1_v1.md`](prereg_gain_randomization_gr1_v1.md); that file
is unchanged.

**Timing.** This amendment is committed after both trainings and before any GR-1 evaluation cell. It follows an
independent pre-measurement design review (Claude Opus 5.5, fresh context; verdict **AMEND_FIRST**;
[`gr1_design/REVIEW_gr1_design_r1.md`](../evidence/gain_randomization_2026-11-04/gr1_design/REVIEW_gr1_design_r1.md),
a copy of `~/motar_tmp/gainrand_20261104/review_gr1_design/audit/REVIEW.md`). Each change below names the review
finding it answers.

## Training (done; recorded here)

| Arm | Commit | Verdict | Checkpoint SHA-256 | Wall time |
| --- | --- | --- | --- | --- |
| `rand40` | `8f64631` | TRAINED | `3f122b47…a6c5` | 70 min |
| `ctrl` | `8f64631` | TRAINED | `0f6ca010…d137` | 65 min |

**H3 (same code and environment).**
- `ctrl` trained on the same commit as `rand40`.
- During the run, its runner process environment (`/proc/<pid>/environ`) was compared with the reviewer's saved
  `rand40` reference (128 keys). The verdict was **ENV_MATCH**: the only differences were the three
  `NAVRL_GAIN_RAND*` variables and the run tag. The record is [`gr1_design/ctrl_env_check.json`](../evidence/gain_randomization_2026-11-04/gr1_design/ctrl_env_check.json).
- This amendment's commit changes nothing under `aerial_gym/` or `motar_eval/`.

## Evaluation changes

- **H1 (labels bound to checkpoints).** `gr1eval` no longer takes a checkpoint or SHA from the caller.
  - It reads the arm, checkpoint copy and SHA-256 from `--train-report`, which must be a TRAINED `train_report.json`.
  - The analysis checks that each eval folder's arm label, SHA and every cell receipt match its training report.
  - It also checks that the two training reports share one commit. Any mismatch reads **EVAL_BINDING_FAIL**.
- **H2 (the evaluation used the checkpoint it was given).**
  - (a) No GR-1 cell's `episodes.jsonl` may equal the GR-0 Policy F cell with the same (level, density, seed), and
    no `rand40` cell may equal the matching `ctrl` cell. Any identical pair reads EVAL_BINDING_FAIL.
  - (b) `gr1repro` reruns the GR-0 cell S40 / 205 bars / seed 4501 twice: through the pinned launcher, and through
    the GR-1 launcher copy with `GR1_CKPT` = Policy F. Both must equal GR-0's cell.
- **L3.**
  - The GR-1 launcher copy is pinned by SHA-256 (`418433eb…`).
  - GR-1 reports name the GR-1 preregistration.
  - `gr1eval` and `gr1repro` default to a 3-hour cap.

## Claim, gates and qualifiers (M1, M3, L1, L2)

**Claim.** On the GR-0 grid (E2, A3, 70-205 bars x seeds 4501-4503, quota 16), this `rand40` continuation of Policy
F crashes at least 1.5 pp less than this `ctrl` continuation under ±40 % gains (S40). It does so without losing
capture under ±40 % gains, and loses at most 2 pp nominal capture (S00). The unit of inference is this pair of
training runs, one seed each.

**Quantities.**
- P1 and P2 are as preregistered.
- New, **P3**: capture(rand40, S40) − capture(ctrl, S40).
- New, **DiD**: the S40 − S00 crash gap of `rand40` minus that of `ctrl`.
- Every quantity gets the preregistered block bootstrap and a t-interval as a sensitivity check (L2).

**Reading** (first match wins):

| Reading | Condition |
| --- | --- |
| EVAL_BINDING_FAIL | any binding check fails |
| GR1_MATERIAL_LOSS | P2 ≤ −2 pp and CI(P2) upper < 0 |
| GR1_SUPPORTED | P1 ≤ −1.5 pp and CI(P1) upper < 0, CI(P2) lower > −2 pp, and **CI(P3) lower > 0** (M1b: crashes turned into timeouts do not count) |
| GR1_NOT_SUPPORTED | CI(P1) lower > −1.5 pp |
| INCONCLUSIVE | otherwise |

**Qualifiers**, reported with a SUPPORTED reading:
- **Wording (M1a).** "Reduced gain sensitivity" only if CI(DiD) upper < 0. Otherwise "lower crash under ±40 %
  gains", because a policy that is uniformly better is not more robust.
- **Drift (M3 minimum).** If |crash(ctrl, S40) − crash(F, S40)| ≥ |P1|, SUPPORTED is reported as "not separable
  from continuation drift".

**Power (L1, from the review's simulation on GR-0's 12 blocks).** P(SUPPORTED) is about 0.5 at a true P1 of −1.5 pp,
about 0.8 at −1.7 pp and at least 0.88 at −2.0 pp. At a true P1 of 0 the reading is NOT_SUPPORTED with
probability about 1.

**Crash causes (M2), reported and not gated.** The table gives bar contacts and floor strikes ("below") per 100
episodes for each policy and level, summed over cells from `result.json` (aggregate window). GR-0's S40 excess
splits into 2.00 bar contacts and 1.72 floor strikes per 100 episodes. The policy has no vertical action, because
the task's altitude PI sets it. So P1 may be capped near −2 pp even if randomization removes all bar-contact excess.

## Second training seed (user decision, 2026-11-04)

The review recommended a second `ctrl` seed as a negative control (M3, about 2 GPU-h). The user decided to evaluate
first and run more GPU work only if needed ("일단 한번 검사한다음에 gpu 또 돌려도 되는거아니야?").
The rule is fixed here, before the evaluation:
- If the reading is **GR1_SUPPORTED** (with or without qualifiers) or **INCONCLUSIVE**, a confirmation with a second
  training seed per arm is proposed as a new preregistration, for the user's decision. Until then a SUPPORTED
  reading is labelled "single training seed per arm".
- If the reading is **GR1_NOT_SUPPORTED**, no second seed is run.
- Neither the thresholds nor the readings change after the evaluation.

## Next-step rules per reading (M2)

| Reading | Next step |
| --- | --- |
| GR1_SUPPORTED | Confirmation seeds (above). If confirmed, include the `rand40` checkpoint in GR-2 (platform randomization). |
| INCONCLUSIVE | Confirmation-seed decision (above). Read the crash-cause table: if bar contacts fall but floor strikes persist, a controller-side remedy is the next design, not more PPO variants (discipline review 2026-08-22). |
| GR1_NOT_SUPPORTED | No further gain-randomization PPO variants. If floor strikes dominate the S40 excess, design a controller-side remedy. Otherwise GR-2 evaluates Policy F. |
| GR1_MATERIAL_LOSS | Stop and report; nothing is adopted. |
| EVAL_BINDING_FAIL | Fix the pipeline; rerun the evaluation into new folders; the failed folders stay recorded. |

## Known-bad and known-good controls for the analysis (disclosed development, not evidence)

[`gr1_design/test_gr1_analyse_synthetic.py`](../evidence/gain_randomization_2026-11-04/gr1_design/test_gr1_analyse_synthetic.py)
builds synthetic GR-1 folders from the real GR-0 Policy F cells and runs the amended `tools/gr1_analyse.py` (the same
bytes, sha256 `b230fb72…`). The result is
[`gr1_design/synthetic_controls.json`](../evidence/gain_randomization_2026-11-04/gr1_design/synthetic_controls.json):

| Case | Expected | Got |
| --- | --- | --- |
| crashes at S40 turned into captures (P1 −3.22 pp) | GR1_SUPPORTED | GR1_SUPPORTED, wording "reduced gain sensitivity" (DiD −3.22) |
| no effect | GR1_NOT_SUPPORTED | GR1_NOT_SUPPORTED |
| arm folders swapped | EVAL_BINDING_FAIL | EVAL_BINDING_FAIL (labels) |
| the same folder for both arms | EVAL_BINDING_FAIL | EVAL_BINDING_FAIL (labels, identical cells) |
| crashes at S40 turned into timeouts (P1 −3.22, P3 0) | not SUPPORTED | INCONCLUSIVE |

The first version of the test built two synthetic arms with identical cells outside S40, and the analysis correctly
read EVAL_BINDING_FAIL. The test was fixed by giving each arm its own perturbation.

## Not changed

Cells, levels, seeds, quota, placement, P1/P2 thresholds and the training contract.
