# CAM-K2-E v1 — matched evaluation of CAM-K2 v2 (2026-11-08)

**Status:** preregistered; design review r1 AMEND_FIRST (in `../camk2/REVIEW_v2_design_r1.md`) and round 2
AMEND_FIRST ([`REVIEW_design_r2.md`](REVIEW_design_r2.md)) and round 3 PASS_WITH_LIMITS
([`REVIEW_design_r3.md`](REVIEW_design_r3.md)), every finding repaired before the commit. Approved by the
user on 2026-11-08 at about 23:30 KST. **RAN** 2026-11-08 23:42 – 2026-11-09 02:31 KST at `1237f9f`: gate A
SMALL_ADAPTATION (lower crash under the treatments), gate N NO_MATERIAL_LOSS, gate I SEEDS_DISAGREE,
`adaptation_supported` false. Results review r1 PASS. See **Results** below. Read the
[preregistration](../../../plans/prereg_camonly_k2_eval_v1_2026-11-08.md) first.

**Found before launch** (CPU, exploratory; not CAM-K2-E data):
- [`DEPTHPROBE_REPORT_codex.md`](DEPTHPROBE_REPORT_codex.md) and [`CONTRACT_AUDIT_codex.md`](CONTRACT_AUDIT_codex.md)
  are by a Codex agent. Their raw files are in `~/motar_tmp/camk2_depthprobe_20261108/`.
- [`REVIEW_depthprobe_r1.md`](REVIEW_depthprobe_r1.md) is the independent review (Claude Opus 5.5): PASS_WITH_LIMITS
  for both reports.
- **What they found:** v2's MinZ of 0.4 m and its invalid → no-return convention erase face-on bars in about the last
  0.26 m of approach.
- **What is not shown:** that this changes the crash rate (NOT_TESTED).

**Records:**
- [`controls.json`](controls.json): the five controls on CAM-K1-E's data, CONTROLS_OK.
- Code: `tools/camk2e_run.py`, `tools/camk2e_analyse.py`, `aerial_gym/rl_training/rl_games/eval_navrl_camk2e_arms.sh`,
  `tests/test_camk2e.py` (27 tests).
- Raw outputs are in `~/motar_tmp/camk2e_20261108/raw/`.

## Results (ran 2026-11-08 23:42 – 2026-11-09 02:31 KST at `1237f9f`)

**Reading:** gate A **SMALL_ADAPTATION** for both seeds, which here means only "lower crash under the treatments".
Gate N **NO_MATERIAL_LOSS** for both. Gate I **SEEDS_DISAGREE** (seed 1 INTERACTION_NOT_DETECTED, seed 2
INTERACTION_UNRESOLVED). `adaptation_supported` is **false** for both seeds, so the word "adaptation" is not used.
Results review r1 (Claude Opus 5.5): **PASS** ([`RESULTS_REVIEW_r1.md`](RESULTS_REVIEW_r1.md)). It recomputed every
mean and interval from the raw `episodes.jsonl` with its own code, to within 1e-7 pp, and re-derived the labels from
the preregistration's text.

**Run:**
- Chain: identity 23:42–23:54, **IDENTITY_PASS**. The K1s1 cell with the treatments off equals CAM-K1-E's cell
  exactly. With the treatments on, the cell differs, and its repeat equals it. The K2 smoke cell ran.
- Eval 23:54–02:31, **COMPLETE**, 60 of 60 cells. Then the analysis. Host faults 0 → 0. Only the RustDesk display process
  was on the GPU.
- Every binding check in `analysis.json` holds. This includes the K2s1 and K2s2 cells coming from different
  checkpoints. Their pooled crash rates are both 14.84 % by coincidence: each seed has 3,647 crashes in 24,576
  episodes, but the per-block counts differ in all 12 blocks, the episode files differ in every block, and the
  receipts and cell logs name `train_s1` (769a26f4…) and `train_s2` (e76eb048…) respectively (review r1).
- **Rate gate:** passed. The runner did not record its projection. Recomputed from the reports (review r1): identity
  mean 177.2 s / CAM-K1-E's 169.7 s = 1.044; 60 × 158 s × 1.044 × 1.10 ≈ 3.02 h, under the 3.5 h cap. The eval took
  2.62 h (157 s per cell against CAM-K1-E's 158 s).
- The raised limit was used. In the treated cells, 1.6–1.9 % of motor commands asked for more than navrl_quad's own
  limit, and 1.1–1.4 % were at the raised limit of 2.4525 N, read back live in every cell.

**Pooled rates** (quota rates, %; main arms over 12 blocks, `_nom` over 6 blocks at 70 and 205 bars):

| Arm | Treatments | Crash | Capture | Timeout |
| --- | --- | ---: | ---: | ---: |
| CAM-K1-E K1s1 / K1s2 (reference) | off | 16.22 / 17.15 | 83.78 / 82.85 | 0 / 0 |
| K1s1 / K1s2 | on | 18.64 / 19.58 | 81.36 / 80.42 | 0 / 0 |
| K2s1 / K2s2 | on | 14.84 / 14.84 | 85.16 / 85.16 | 0 / 0 |
| K2s1_nom / K2s2_nom | off | 13.83 / 13.52 | 86.17 / 86.48 | 0 / 0 |

**Gates** (crash, pp; bootstrap 95 % and t 95 %; df 8 over 12 blocks, df 4 over 6):

| Contrast | Seed 1 | Seed 2 |
| --- | --- | --- |
| A: K2 − K1, both treated | −3.80 [−4.63, −2.93; t −5.04, −2.57] SMALL_ADAPTATION | −4.74 [−5.26, −4.24; t −5.48, −4.01] SMALL_ADAPTATION |
| N: K2_nom − CAM-K1-E's K1, both on F's vehicle | −2.99 [−4.14, −1.85; t −5.06, −0.93] NO_MATERIAL_LOSS | −4.18 [−4.46, −3.95; t −4.64, −3.72] NO_MATERIAL_LOSS |
| I: interaction, 6 blocks | −0.15 [−1.57, +1.78; t −3.18, +2.88] INTERACTION_NOT_DETECTED | −0.69 [−1.22, −0.20; t −1.60, +0.22] INTERACTION_UNRESOLVED |
| Cost of the treatments to CAM-K1 (reported) | +2.42 [+1.66, +3.12; t +1.35, +3.49] | +2.44 [+2.03, +2.82; t +1.86, +3.02] |
| Seed spread K2s1 − K2s2 (reported) | 0.00 [−0.52, +0.55; t −0.78, +0.78] NEGLIGIBLE | |

- Seed 2's gate I t-interval reaches +0.22 pp, which is why it is UNRESOLVED and not INTERACTION_NEGATIVE.
- Seed 1's bootstrap ends move by about 0.2 pp with the bootstrap generator. All six labels stay the same (review r1).

Timeouts are 0 in every contrast, so CRASH_TO_TIMEOUT does not apply. All five controls read as required
(CONTROLS_OK).

**What may be said** (the preregistration's wording):
> On this grid, in simulation, under CAM-K2 v2's treatments (navrl_quad's per-motor thrust limit raised to T/W 4.0,
> the thrust-to-weight of a 1.1 kg vehicle at an INFERRED T/W 4.0 with navrl_quad's rotational dynamics and drag;
> ±15 % / ±5 % thrust randomization, τ 0.02–0.04 s; and the OAK-D Pro depth model as configured), CAM-K2 v2 changes
> the crash rate by −3.80 pp [−4.63, −2.93; t −5.04, −2.57] against CAM-K1 for seed 1 and by −4.74 pp [−5.26, −4.24;
> t −5.48, −4.01] for seed 2 (gate A: SMALL_ADAPTATION, both seeds; read as lower crash under the treatments). On F's vehicle
> (T/W 3.26, τ 0.04 s, exact depth), it changes the crash rate by −2.99 pp [−4.14, −1.85; t −5.06, −0.93] and
> −4.18 pp [−4.46, −3.95; t −4.64, −3.72] (gate N: NO_MATERIAL_LOSS, both seeds; within 3 pp of CAM-K1 on F's vehicle,
> at 70 and 205 bars on this grid). The interaction is −0.15 pp (gate I: INTERACTION_NOT_DETECTED) and −0.69 pp
> (INTERACTION_UNRESOLVED).

- CAM-K2's lower crash rate under the treatments is about as large as its lower crash rate on F's vehicle. The
  interaction does not separate it from the 1,000 extra training epochs, so it is **not** shown to be adaptation to
  the treatments.
- **May not be said:** adaptation; anything about the real vehicle, the real OAK-D Pro or real flight; robustness in
  general; which of the three treatments mattered; any comparison with F with camera + LiDAR, or with
  training-rollout rates.
- **Not tested here:** whether the near-bar depth gap found by the CPU probe changes the crash rate. Both K2 arms and
  the treated K1 arms ran with the same depth model.

**Records:**
- [`analysis.json`](analysis.json): a byte copy of `analysis_01.json`.
- [`eval_run_report.json`](eval_run_report.json) and [`identity_report.json`](identity_report.json): byte copies.
- [`SHA256_MANIFEST.json`](SHA256_MANIFEST.json): 572 raw cell, report, chain and analysis files in
  `~/motar_tmp/camk2e_20261108/`.
