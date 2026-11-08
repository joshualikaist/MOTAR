# CAM-K1-E v1 — matched evaluation of the CAM-K1 camera-only lineage (2026-11-08)

**Status:** ran 2026-11-08 at `971ae67`. Stage 1 IDENTITY_PASS; stage 2 COMPLETE (36 cells); host faults 0 → 0.
**Gate R MATERIAL_RECOVERY** for both seeds and **gate P MATERIAL_LOSS** for both seeds, with all four controls as
required. The analysis prints `REPORTED` when no control failed; the labels are the gate labels. Results review r1
(Claude Opus 5.5): **PASS_WITH_LIMITS**, so the readings are no longer CONDITIONAL. Read the
[preregistration](../../../plans/prereg_camonly_k1_eval_v1_2026-11-08.md) first.

- [`REVIEW_design_r1.md`](REVIEW_design_r1.md): the design review (Claude Opus 5.5, fresh context) of the
  preregistration, launcher copy, runner, analysis and tests. Its probes stay in
  `~/motar_tmp/camk1e_20261108/review_design_r1/`.
- Code: `tools/camk1e_run.py`, `tools/camk1e_analyse.py`, `aerial_gym/rl_training/rl_games/eval_navrl_camk1e_arms.sh`,
  `tests/test_camk1e.py` (18 tests, CPU).
- Raw outputs will be in `~/motar_tmp/camk1e_20261108/raw/`.

## Results (2026-11-08)

**Records** in this folder:
- [`identity_report.json`](identity_report.json) and [`eval_run_report.json`](eval_run_report.json): byte copies of
  the runner's reports.
- [`analysis.json`](analysis.json): a byte copy of `tools/camk1e_analyse.py`'s output. Every number below comes from
  it.
- [`SHA256_MANIFEST.json`](SHA256_MANIFEST.json): SHA-256 of every raw cell file under
  `~/motar_tmp/camk1e_20261108/raw/`, plus the chain log and the approval file.
- [`RESULTS_REVIEW_r1.md`](RESULTS_REVIEW_r1.md): the independent results review.

**Execution:**
- Chain `~/motar_tmp/camk1e_20261108/run_camk1e.sh`, 13:08:54–14:52:13 KST.
- Identity: 3 cells, about 2.9 min each.
  - `F87_identity` equals C2's `no_lidar` cell exactly (798 captures, 1,250 crashes).
  - `F66_negative` differs from it.
  - `K1s1_smoke` differs from `F66_negative`.
- Eval: 36 cells, 13:17:23–14:52:11, median 157 s per cell.
- Free disk went from 2.64 to 2.18 GB.
- **Concurrency (review l2):** CPU-only work outside this tree ran between 13:08 and about 14:45 on cores 6–7, at
  `nice` 10–19, with an 8 GB memory cap and no CUDA, as the T-LOAD rule allows. It covered:
  - the implementation agents' focused tests in `src/MOTAR-realveh`;
  - the S4F-1 and CAM-K2 design reviews and their probes.

  This unit reads per-episode outcome files only. The identity stage's F66 and K1s1 cells equal the eval's, and no
  wall-clock number from this unit is used.
- **Tree:** clean during the run. Every file dirty afterwards changed after the chain had ended (review).
- **PUBLIC_CPU** at the preregistration commit: 3,575 run, 0 failures. There were 2 errors from `test_boia_planner`'s
  3 GB free-disk floor (review l3).

**Pooled quota rates** (%, mean over the 12 blocks):

| Arm | Capture | Crash |
| --- | --- | --- |
| A: F, camera + LiDAR, 87° | 95.54 | 4.46 |
| C87: camera-only F, 87° (C2) | 55.59 | 44.40 |
| F66: camera-only F, 66° | 46.11 | 53.89 |
| K1s1: CAM-K1 seed 1, 66° | 83.78 | 16.22 |
| K1s2: CAM-K1 seed 2, 66° | 82.85 | 17.15 |

**Gated contrasts** (crash, pp; stratified bootstrap 95 % and stratified t 95 %, 8 df):

| Contrast | Mean | Bootstrap | t | Label |
| --- | --- | --- | --- | --- |
| R, K1s1 − F66 | −37.67 | [−38.45, −36.78] | [−38.89, −36.45] | MATERIAL_RECOVERY |
| R, K1s2 − F66 | −36.74 | [−37.46, −35.98] | [−37.84, −35.65] | MATERIAL_RECOVERY |
| P, K1s1 − A | +11.77 | [+11.26, +12.30] | [+11.01, +12.53] | MATERIAL_LOSS |
| P, K1s2 − A | +12.69 | [+12.22, +13.21] | [+11.98, +13.40] | MATERIAL_LOSS |

- Timeouts did not rise (0.00 % in both K1 arms), so the recovery is not CRASH_TO_TIMEOUT.
- Capture moved by the same amounts with the opposite sign: R +37.67 / +36.75 pp, P −11.76 / −12.69 pp.

**Controls** (existing data; required labels): C87 − A +39.95 pp MATERIAL_LOSS; A − C87 MATERIAL_RECOVERY; S20 − A
+0.61 pp NO_MATERIAL_LOSS and NEGLIGIBLE. All OK.

**Reported, not gated:**
- **Narrowing camera-only F's view from 87° to 66°** (F66 − C87): crash +9.49 pp [+8.67, +10.29].
- **Loss at 66°** (F66 − A): crash +49.43 pp.
- **Seed spread** (K1s1 − K1s2): crash −0.92 pp [−1.53, −0.33].

**Per density** (crash %):

| Bars | A | C87 | F66 | K1s1 | K1s2 |
| --- | --- | --- | --- | --- | --- |
| 70 | 1.2 | 25.8 | 32.1 | 7.2 | 7.7 |
| 115 | 2.9 | 40.1 | 48.7 | 13.1 | 13.2 |
| 160 | 4.9 | 51.4 | 62.1 | 18.2 | 20.0 |
| 205 | 8.8 | 60.3 | 72.6 | 26.5 | 27.7 |

- **Crash causes** (exploratory, aggregate window): bar contact per 100 episodes was A 4.38, F66 54.44, K1s1 16.31 and
  K1s2 17.42. Floor strikes and out-of-bounds stay at or below 0.04 per 100 in the K1 arms.

**What may be said:**
- On this grid, in simulation, CAM-K1 seed 1 lowers the crash rate by 37.7 pp and seed 2 by 36.7 pp against
  camera-only F at the same 66° camera (gate R: MATERIAL_RECOVERY for both seeds).
- Both seeds stay 11.8 and 12.7 pp above F with camera + LiDAR at 87° (gate P: MATERIAL_LOSS for both seeds).
- K1 − F66 measures CAM-K1's whole recipe: `no_lidar` training, the 66° FOV, the two reward weights and 3,000 more
  epochs. Not camera-only training alone.

**What may not be said:**
- anything about the real vehicle, real images, the OAK-D Pro's depth or real flight;
- that the LiDAR is unnecessary (gate P reads MATERIAL_LOSS);
- that more training would or would not help;
- an effect from one seed alone.
- **Training rollouts against evaluation rates:** the training rollouts (about 26 % capture) ran under exploration
  noise at a fixed 205 bars. They are not comparable with these evaluation rates (K1 at 205 bars: about 73 %
  capture).
- The intervals describe these 12 blocks; they are not over training seeds.

## Results review r1 (Claude Opus 5.5, fresh context): PASS_WITH_LIMITS

The review is [`RESULTS_REVIEW_r1.md`](RESULTS_REVIEW_r1.md); its probes stay in
`~/motar_tmp/camk1e_20261108/review_results_r1/`. It found 0 MAJOR, 5 MINOR and 7 LOW findings.
- **Integrity:** it verified the approval, the identity stage and all 36 cells. It used its own code, not
  `tools/`, and checked receipts, checkpoint SHA-256 against the train reports, 66.0 × 54.0 in every recorder
  summary, conditions and snapshots against C87, the run order, the host, the reference manifests and the script.
- **Recomputation:** means agree to machine precision, the t-intervals to 7e-8 pp, and the bootstrap ends to
  0.05 pp, including against the exact bootstrap over all 27⁴ resamples. All labels are unchanged under every
  interval method.
- **Margins:** the nearest interval end is 25.6 pp from gate R's −10 pp threshold and 8.0 pp from gate P's +3 pp
  threshold.

**Disclosed, from the review:**
- **m1, the approval's condition.** The user approved today's runs "only after an Opus review and a preflight have
  passed". This unit's design review r1 read AMEND_FIRST. Its findings were repaired before the commit and were not
  reviewed again before launch; the preregistration says so. The results review confirmed on the real data that the
  four MAJOR repairs worked, but it did not re-review the whole design. Whether the condition was met is the user's
  call.
- **m2, the seeds.** The intervals exclude training-seed variation, and the seeds differ (K1s1 − K1s2 crash −0.92 pp,
  interval excluding 0). The seeds are not pooled into one interval, and a per-seed interval is not a recipe-level
  interval.
- **m3, training rollouts.** The rollouts ran with exploration noise and the training target condition. They are not
  comparable with these rates.
- **m4, ratios.** A "share of the loss recovered" and K1 − C87 are not preregistered and are not reported as
  results. F66 − A mixes removing the LiDAR with narrowing 87° to 66°. Gate P reads as a loss of about 12 pp, never
  as "close to F".
- **l5, arm identity.** `result.json` records neither the sensor condition nor the FOV. Arm identity rests on the
  receipts, the camtrain receipts and the recorder summaries, bound in [`SHA256_MANIFEST.json`](SHA256_MANIFEST.json).
- **l6, pixels.** The camera image stays 160 × 90 at 66° × 54°, so pixels are not square (fx ≈ 123.2, fy ≈ 88.3 px).
- **l7, crash causes and governor.** The crash causes come from each cell's aggregate (legacy) window, not from the
  quota rows. The governor telemetry differs strongly between arms: perceived clearance is about 6.6 m for K1 and
  0.44 m for F66. It is descriptive only, at most "consistent with".
- **l4, a control value.** The prereg's precomputed bound −39.65 for A − C87 reads −39.640 in the final run, from the
  shared random stream. The preregistration is not edited.
- **Other limits** (prereg): the vehicle is navrl_quad (0.25 kg, T/W 3.26); one camera serves RGB and depth with no
  stereo model; the detector is F's colour rule; CAM-K1 trained at a fixed 205 bars; A and C87 come from earlier
  commits, admitted through C1's identity and this unit's stage 1.
