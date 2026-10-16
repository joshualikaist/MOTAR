# Amendment 2 to the KROC2027 Experiment A preregistration (2026-10-16)

Amends [`prereg_kroc2027_A_v1.md`](prereg_kroc2027_A_v1.md) and
[amendment 1](prereg_kroc2027_A_v1_amendment_2026-10-15.md). Written after the MT-P2-C01 canary and
before any cell of the A3/A4 main run (KROC27-A-M02). The canary's capture, crash and timeout counts
were not inspected and play no part in it. Machine-readable contract:
[`kroc2027_A_amendment2_v1.json`](kroc2027_A_amendment2_v1.json); where the two differ, the JSON
governs.

**Authority.** On 2026-10-16 the user approved MT-P1, commit `4bba883` and one run of the MT-P2-C01
canary. The user then delegated the remaining steps to the agent: proceed with the recommended steps
and report at the end. This amendment and the run it governs follow that delegation, and the user
reviews them afterwards. Still not authorised: PPO training, push, changes to existing results, and
the user's WIP or stash.

## Why the moving arms can now run

- Amendment 1 recorded that E1_cv and E2_obstacle_aware failed swept clearance at 160 bars, so
  H-A2 was `NOT_TESTED`.
- The cause was TF32 matmul inside `torch.cdist` (commit `1c72c22`). rl_games turns TF32 on for the
  whole process. On the RTX 3070, `cdist` then picks a TF32 kernel when the bar count is a multiple of
  4, and the executor's distances err by up to about 2 m.
- MT-P1 (`c4880be`) adds the opt-in lineage `direct_l2_v1`, which computes the same distances as a
  plain difference norm. Its motion definitions, limits, candidates and ranking are unchanged, and so
  are TF32 and Policy F. The default `legacy_cdist` path is bitwise unchanged.
- Canary MT-P2-C01 (`4bba883`, seed 4190, 4 quota episodes per environment): `CANARY_PASS`, all 16 cells (E1 and E2 direct × A3/A4 × 70/115/160/205 bars). G1/G2, swept-clearance and wall violations: 0. The smallest swept clearance was 0.225 m (proxy limit 0.15 m), and the largest target step speed 1.49998 m/s. The runner's TF32 state was unchanged in every cell. The run took 0.349 GPU hours. Report SHA-256 `e0e4598a…`, RAW_MANIFEST `9b28af16…`.

## Changes

1. **Moving model.** Section 5's choice rule is applied to the new lineage, in the same order:
   E2_direct_l2_v1, then E1_direct_l2_v1. The model is the first one whose eight canary cells
   (A3/A4 × four densities) all pass. The runner applies this rule to the canary report itself.
   Result: **E2_direct_l2_v1**. The historical E1/E2 results keep their recorded status.
2. **Main run KROC27-A-M02.**
   - Cells: A3 (visible) and A4 (hidden) with that lineage, at 70, 115, 160 and 205 bars, seeds
     4201–4207, which gives 56 cells.
   - Each cell has 128 environments with the first 16 completed episodes of each as the quota, and a
     600-action timeout. The initial view of section 4 is unchanged, and the recorder is on.
   - Packets use full_v2 and are written only on a violation.
   - The run stops at the first motion, swept-clearance or wall violation (section 5, main-run
     rule) and at the first failed cell.
3. **Same simulation source as the canary.** The run's commit must hold `aerial_gym/` and
   `motar_eval/` byte-identical to the canary commit `4bba883`, and the runner checks this. The
   launcher and checkpoint are pinned by SHA-256. A different commit is allowed only for `tools/`,
   `tests/` and `docs/`.
4. **A1 and A2 are reused, not re-run.**
   - They are the KROC27-A-M01 cells: commit `bbf530b`, `results/kroc2027_A_v1/`, each file
     re-verified against that run's `RAW_MANIFEST.json`. They were analysed under amendment 1
     (`analysis.json`, SHA-256 `1d79b21f…`).
   - The arms were always independent samples (`UNPAIRED_STREAM`), so using two runs breaks no
     pairing. It does mean the static and moving arms come from different commits and different
     days.
   - A static target never calls the moving-target executor, and the legacy path is bitwise unchanged
     (tests). This is a code argument, not a re-measurement.
5. **Analysis: section 7 is restored as preregistered.**
   - Confirmatory family: Delta_A and I_A, two-sided paired t over the seven seeds, Holm over both,
     α = 0.05. Practical claims use the Bonferroni 97.5 % interval against 3 pp, with 1 and 5 pp
     also reported. Sensitivity uses BCa and the exact sign-flip test from the pinned helpers.
   - Secondary contrasts: static-only and moving-only. Code: `motar_eval.protocol.analyse_a`,
     unchanged, through `tools/kroc2027_eval.py analyse-a-moving`.
   - The static half of Delta_A (A1 − A2) was observed before this amendment, so Delta_A is not fully
     blind. I_A and A3 − A4 are blind.
   - The amendment-1 static result (H-A `SUPPORTED`, 3 pp not shown) stays as recorded and is not
     re-labelled. The paper reports both.
6. **GPU cap.** Projected from the canary: E2's eight cells took 633 s at 4 quota episodes. Scaled by 4 for 16 episodes and by 7 seeds, that gives 4.92 h. The cap is 7.5 h, about 1.5 × the projection. 8.10 of the 30 GPU hours were used before this run. At the cap the running cell is killed and kept as partial, and the run stops.

## Not changed

Hypotheses and predicted directions, seeds, quota, timeout, initial-view treatment, gates and
tolerances, event definitions, stop rules, Experiment B and its recorded result.

## Limits stated in advance

- The new lineage corrects distance arithmetic only. The longest-safe-prefix fallback can still
  execute an unsafe step from a trapped state, so the run stops on any violation rather than
  excusing it.
- The moving target is a virtual point with bounded planar dynamics, not a physical UAV.
- I_A is about this lineage, not the historical E1/E2 models.
- Simulation only.
