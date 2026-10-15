# Amendment 1 to the KROC2027 Experiment A preregistration (2026-10-15)

Amends [`prereg_kroc2027_A_v1.md`](prereg_kroc2027_A_v1.md). Written after the canary KROC27-A-C01
and before any main-run cell (KROC27-A-M01). The canary's capture, crash and timeout counts were
not inspected and play no part in it. Approved by the user on 2026-10-15.

## What the canary showed (integrity and motion only)

- The off-path cell reproduced the 2026-09-25 replication cell byte for byte (`rows_sha256`
  `0a1bf878…`); recorder on and off gave identical results, trajectory digest, ledger rows and
  1,024 sampled actor observations; recorder and task first-acquisition indices agreed in every
  episode; the static target passed every gate; B1 searched in 1,790 episodes with no invariant
  violation.
- **No moving model passed.** E2_obstacle_aware and E1_cv both failed the swept-clearance gate at
  160 bars, in A3 and A4 alike: the target centre came within 0.15 m of a bar footprint in 2–4 % of
  those episodes and reached a footprint (clearance 0) in 5–32 episodes per cell, spread over
  11–57 distinct scenes. No violation occurred at 70, 115 or 205 bars, and G1/G2 passed everywhere.
  The two models share the bounded executor, whose fallback can execute a step that is already
  unsafe; the synthetic layouts of `tools/audits/audit_e2_target_validity.py` did not expose it.
- Under section 5 of the preregistration, A3 and A4 are not run and **H-A2 is `NOT_TESTED`**.

## Changes

1. **Confirmatory test without a moving model.** The family is the single contrast
   A1 − A2 (static, visible minus hidden capture, per seed, densities weighted equally). Test:
   two-sided paired t over the seven seeds at α = 0.05, no multiplicity adjustment, predicted
   direction positive. Decision labels as in section 7, with the practical claim judged from the
   95 % t interval against 3 percentage points (1 and 5 also reported). Delta_A and I_A are
   `NOT_TESTED`. Sensitivity analyses are unchanged (BCa, exact sign-flip). Code:
   `motar_eval.protocol.analyse_a_static`.
2. **Main run.** 56 cells: A1 and A2 × four densities × seeds 4201–4207. The canary's timing rule
   projected 4.6 GPU hours for these cells, within the budget, so all four densities run.
3. **Canary verdict rule (code defect, no protocol change).** `tools/kroc2027_eval.py` counted
   swept-clearance violations as instrumentation failures as well, which made the report read
   `CANARY_FAIL`. Section 5 assigns them to the motion contract only. The fix keeps them in the
   motion gate (main runs still stop on them) and out of the canary's instrumentation check. Since
   the main run needs a `CANARY_PASS` on the same commit, the canary is re-run unchanged on the
   commit that adds this file.
4. **Experiment B.** The arm rule now has only the static hidden arm (A2), as `select_b_arm`
   already does; the gate and everything else in section 9 are unchanged.

## Not changed

Hypotheses' predicted directions, seeds, quota, timeout, initial-view treatment, gates and
tolerances, event definitions, stop rules. Existing E1/E2 code and past results are untouched; a
corrected motion lineage would be a separate, separately approved package.
