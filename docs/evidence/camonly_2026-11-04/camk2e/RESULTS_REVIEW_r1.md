# CAM-K2-E v1 — independent results review r1

- Reviewer: Claude Code subagent, model Opus 5.5 (claude-opus-5-5), fresh context, read-only.
- Date: 2026-11-09 ~02:35 KST. Worktree `MOTAR-camonly` HEAD `1237f9f`; `git status --porcelain` empty before and
  after (no tracked file touched). Only this file was written (plus a scratch script in the session scratchpad).
- Inputs: prereg `docs/plans/prereg_camonly_k2_eval_v1_2026-11-08.md`; `tools/camk2e_analyse.py` (+ helpers in
  `camk1e_analyse.py`, `camonly_c2_analyse.py`, `gr_analyse.py`); `tools/camk2e_run.py`;
  `~/motar_tmp/camk2e_20261108/{chain.log, identity_01.log, eval_01.log, analysis_01.json, analysis_01.log,
  execution_approval.json, raw/identity_01, raw/eval_01}`; `~/motar_tmp/camk1e_20261108/raw`.
- All Python ran as `CUDA_VISIBLE_DEVICES="" nice -n 10 taskset -c 6,7 python3 -I`.

## 1. Independent recomputation (from raw `episodes.jsonl`, `in_quota` rows)

My own script (not importing the repo's analysis code) re-read every cell's quota rows (n = 2048 in all 60 eval cells
and the 24 CAM-K1-E K1 cells), rebuilt per-block rates, the stratified bootstrap (seeds resampled within density,
10,000 resamples, seed 20261108) and the stratified t-interval (df 8 for 12 blocks, t = 2.306004; df 4 for 6 blocks,
t = 2.776445). Result: every mean, bootstrap end and t end in `analysis_01.json` reproduced to < 1e-7 pp (same
generator order). Labels were re-derived with my own implementation of the prereg rules.

| Contrast (crash, pp) | mean | bootstrap 95 % | t 95 % | df | label (mine = script) |
| --- | --- | --- | --- | --- | --- |
| A: K2s1 − K1s1 (ON) | −3.80 | [−4.63, −2.93] | [−5.04, −2.57] | 8 | SMALL_ADAPTATION |
| A: K2s2 − K1s2 (ON) | −4.74 | [−5.26, −4.24] | [−5.48, −4.01] | 8 | SMALL_ADAPTATION |
| N: K2s1_nom − K1E_K1s1 | −2.99 | [−4.14, −1.85] | [−5.06, −0.93] | 4 | NO_MATERIAL_LOSS |
| N: K2s2_nom − K1E_K1s2 | −4.18 | [−4.46, −3.95] | [−4.64, −3.72] | 4 | NO_MATERIAL_LOSS |
| I: seed 1 | −0.15 | [−1.57, 1.78] | [−3.18, 2.88] | 4 | INTERACTION_NOT_DETECTED |
| I: seed 2 | −0.69 | [−1.22, −0.20] | [−1.60, 0.22] | 4 | INTERACTION_UNRESOLVED |
| seed spread K2s1 − K2s2 (ON) | 0.00 | [−0.52, 0.55] | [−0.78, 0.78] | 8 | (reported; C2 scheme NEGLIGIBLE) |
| seed spread K1s1 − K1s2 (ON) | −0.94 | [−1.60, −0.31] | [−1.89, −0.00] | 8 | (reported) |
| seed spread K2s1_nom − K2s2_nom | +0.31 | [−0.22, 0.86] | [−0.64, 1.26] | 4 | (reported) |
| cost to CAM-K1, s1 / s2 | +2.42 / +2.44 | [1.66, 3.12] / [2.03, 2.82] | [1.35, 3.49] / [1.86, 3.02] | 8 | (reported) |

- **Label logic vs prereg text: correct.** Gate A order UNRESOLVED → MATERIAL → SMALL (upper < 0 and mean ≤ −2) →
  WORSE → NEGLIGIBLE → NO_DETECTED matches `gate_a`. CRASH_TO_TIMEOUT substitutes for MATERIAL/SMALL when the timeout
  contrast's mean ≥ +2 pp; here every arm has 0 timeouts (timeout contrast 0.00), so it cannot fire. Gate N =
  CAM-K1-E's `gate_p` with band 3 (asserted). Gate I UNRESOLVED when the two intervals' sides disagree: seed 2's
  bootstrap excludes 0 (side −1), its t-interval does not (side 0) → INTERACTION_UNRESOLVED is the correct label.
  Gate I weights {K2s +1, K1s −1, K2s_nom −1, K1E_K1s +1} on the 6 nominal blocks, df 4, as preregistered.
- **Robustness of labels to the bootstrap draw** (my check, not a gate): with a fresh generator per contrast, and
  with an unrelated seed, all six labels are unchanged; the bootstrap ends move by ≤ 0.2 pp (I seed 1 upper end
  1.57–1.78, the coarseness the prereg discloses).
- Lineage: A SMALL_ADAPTATION (shared), N NO_MATERIAL_LOSS (shared), I SEEDS_DISAGREE; `adaptation_supported`
  false for both seeds (gate I is not INTERACTION_NEGATIVE for either). Controls: all five read as required
  (CONTROLS_OK), identical to `controls_r1_20261108.json`.

### Pooled K2s1 = K2s2 = 14.83968 %: genuine coincidence

- Both total exactly 3,647 crashes / 24,576 quota episodes, but the per-block counts differ in **all 12 blocks**
  (K2s1: 112, 133, 114, 204, 222, 208, 305, 366, 322, 558, 545, 558; K2s2: 125, 115, 115, 190, 224, 232, 360, 342,
  346, 546, 527, 525); per-density rates differ (e.g. 205 bars 27.03 % vs 26.01 %); `episodes.jsonl` SHA-256 differs
  in every block; receipts carry 769a26f4… vs e76eb048…; each cell log's `ckpt=` line names `train_s1/…ep_29000…`
  vs `train_s2/…ep_29000…`. The equal pooled total is a sum coincidence, not a reused checkpoint or cell.

## 2. Binding

- `sha256sum` of the checkpoint files now: K2 s1 `769a26f4…de04e8a`, K2 s2 `e76eb048…88321db6`, K1 s1
  `d7bf51a8…`, K1 s2 `5128bf2e…` — equal to the runner pins (`camk2e_run.py:86-87`, `camk1e_run.py:68-69`), the run
  report's `checkpoints`, and every cell receipt's `checkpoint_sha256` (one SHA per arm, as expected).
- Every eval and identity receipt: `git_commit` 1237f9f, `git_dirty_files` 0; approval file names HEAD 1237f9f and
  the scope; analysis script, runner and launcher bytes equal `git show HEAD:` (launcher e5923d8e…, as pinned).
- Identity: K1s1_off_identity result and episodes equal CAM-K1-E (crash 523 = CAM-K1-E block); ON negative differs;
  ON repeat equals negative; K2s1 smoke differs from negative → IDENTITY_PASS. Identity ON/smoke cells equal the eval
  K1s1 / K2s1 205/4501 cells (579 / 558 crashes).
- Grids complete (12 / 12 / 12 / 12 / 6 / 6), 60 records in the planned order (density → seed → arm), all
  verdict OK with no problems; module seeds 10e+1 / 10e+2 on ON cells, none on OFF cells; ON snapshots carry the
  v2 keys (TW 4.0, raise, 0.15, 0.05, τ 0.02–0.04, oakd MinZ 0.4); OFF snapshots carry only
  `NAVRL_DEPTH_NOISE_STD=0.02`, which is also in CAM-K1-E's reference snapshot and is applied only with
  `training=True` (not a deviation). All 22 binding flags in `analysis_01.json` are true.

## 3. Deviations from the prereg

None found that affects the result.
- Host: fault count 0 at chain start, after every cell, at end and in the kernel log now; boot id unchanged; online
  CPUs `0-3,6-23` throughout (cpu4/5 offline, the known state).
- Cap: identity 11.8 min; eval 23:53:59–02:31:13 = 2.62 h (sum of walls), < 3.5 h; max cell 191 s.
- Rate gate: passed (empty `preflight_problems`), but its projection is not written to any report. Recomputed from
  the reports: identity mean 177.2 s / CAM-K1-E identity mean 169.7 s = 1.044; 60 × 158 s × 1.044 × 1.10 ≈ 3.02 h
  ≤ 3.5 h. (The CAM-K1-E identity cells are F87_identity, F66_negative, K1s1_smoke — "the same 205-bar seed-4501
  cells", not the same policies; as implemented and reviewed.)
- Disk: 1.49 GB free at start (≥ 1 GB), 1.0 GB at end.
- Concurrency: no file under `~/motar_tmp` (outside this unit) or under `src/*` (depth 3) was modified during the eval
  window; mean eval cell 157 s vs CAM-K1-E's 158 s. No sign of concurrent CPU work; not provable without process
  accounting.

## 4. Allowed result sentence (prereg "May be said" template)

> On this grid, in simulation, under CAM-K2 v2's treatments (navrl_quad's per-motor thrust limit raised to T/W 4.0,
> the thrust-to-weight of a 1.1 kg vehicle at an INFERRED T/W 4.0 with navrl_quad's rotational dynamics and drag;
> ±15 % / ±5 % thrust randomization, τ 0.02–0.04 s; and the OAK-D Pro depth model as configured), CAM-K2 v2 seed 1
> changes the crash rate by −3.80 pp [bootstrap −4.63, −2.93; t −5.04, −2.57, 8 df] against CAM-K1 seed 1, and seed 2
> by −4.74 pp [−5.26, −4.24; t −5.48, −4.01, 8 df] against CAM-K1 seed 2 (gate A: SMALL_ADAPTATION for both seeds,
> shared; read as lower crash under the treatments). On F's vehicle (T/W 3.26, τ 0.04 s, exact depth), seed 1 changes
> the crash rate by −2.99 pp [−4.14, −1.85; t −5.06, −0.93, 4 df, 70 and 205 bars] and seed 2 by −4.18 pp
> [−4.46, −3.95; t −4.64, −3.72, 4 df, 70 and 205 bars] against CAM-K1 seed s (gate N: NO_MATERIAL_LOSS for both
> seeds, shared: within 3 pp of CAM-K1 on F's vehicle, at 70 and 205 bars on this grid). The interaction is −0.15 pp
> [−1.57, 1.78; t −3.18, 2.88, 4 df] for seed 1 (gate I: INTERACTION_NOT_DETECTED) and −0.69 pp [−1.22, −0.20;
> t −1.60, 0.22, 4 df] for seed 2 (gate I: INTERACTION_UNRESOLVED); the lineage reading for gate I is SEEDS_DISAGREE
> and `adaptation_supported` is false.

Optional, reported-not-gated: the treatments cost CAM-K1 +2.42 / +2.44 pp crash; K2s1 − K2s2 under the treatments
is 0.00 pp [−0.52, 0.55; t −0.78, 0.78].

**Must avoid:**
- "adaptation", "adapts", "adapted", "robust to the treatments" anywhere except quoting the label name
  SMALL_ADAPTATION, which must carry the gloss "lower crash under the treatments"; continued training alone can
  explain gate A (gate N shows a comparable reduction with the treatments off).
- "negative interaction", "partial/one-seed adaptation" or "seed 2 supports adaptation" (I_K2s2 is UNRESOLVED;
  its bootstrap alone excluding 0 is not a reading).
- "better/lower than CAM-K1 on F's vehicle" as a gated finding (gate N has no such label; give the signed numbers
  with "within 3 pp" as the prereg's phrase), and gate N beyond 70/205 bars.
- "fully recovers / more than recovers the treatments' cost" or any ratio of gate A to the cost (derived ratio).
- Real vehicle / real OAK-D / real flight; attributing the result to one treatment; pooled "CAM-K2 recipe"
  intervals; training-rollout comparisons; any comparison with F camera+LiDAR; "the two seeds are identical"
  (equal pooled totals are a coincidence).

## 5. VOID / correction check

Nothing makes the result VOID: bindings hold, controls pass, host faults 0, cap and disk respected, tree clean.
Things the results record should do (low severity, record completeness, not analysis errors):
1. (Low) Record the rate-gate projection (≈ 3.02 h, ratio 1.044), labelled as recomputed from the reports, since the
   runner did not write it.
2. (Low) State the pooled-equality coincidence explicitly, with the per-block counts or the `episodes.jsonl` SHA
   difference, so a reader does not suspect a reused checkpoint.
3. (Low) Mention that I_K2s2's t upper end is +0.22 and I_K2s1's bootstrap upper end is draw-sensitive by ~0.2 pp
   (labels unchanged), consistent with the prereg's disclosed coarseness.

Verdict: PASS
