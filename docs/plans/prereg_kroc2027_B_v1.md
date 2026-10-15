# Preregistration — KROC2027 Experiment B: B1 supervisor on hidden static starts (v1)

- **Status:** approved by the user on 2026-10-15 and committed before any B cell; never edited after
  that (amendments are new, dated files).
- **Written after Experiment A, before any B data.** No B cell has run. The B1 integrity subruns of the
  A canary checked invariants only; their outcome counts were not inspected and are not used here.
- **Binds:** protocol `docs/plans/kroc2027_protocol_v1.json` (SHA-256 `ad706d71…`, experiment_B section),
  A preregistration and amendment 1 (`6e3dc2d8…`), A analysis `results/kroc2027_A_v1/analysis.json`
  (SHA-256 `1d79b21fa2c561e2688ce6033f0249ce6443ad13963a826aac8b2dcd7b116ab8`), and the source commit that
  adds this file. The B gate record (`results/kroc2027_B_v1/b_gate.json`) must carry this file's SHA-256,
  that commit and the user's approval; `tools/kroc2027_eval.py run --stage b_main` refuses otherwise.

## 1. Hypothesis

**H-B.** On starts where the target is out of view (A2 condition), wrapping the unchanged Policy F in
the B1 supervisor raises capture. Predicted direction: positive.

B1 is the whole supervisor: the initial search from 1.0 s, the re-search after track expiry, the
horizontal hold while searching, the 0.6 rad/s yaw, the LiDAR suppression, the hand-back on acquisition
and the executed-action feedback. A result is the effect of that wrapper, not of yaw scanning alone.

## 2. Exploratory motivation from Experiment A (already observed)

A2 lost 2.84 pp of capture against A1 (95.32 % vs 98.16 %). Never-acquired episodes were 2.84 % of A2
episodes, 2.78 % ended in failure, and they made up 59.3 % of A2's failures. The arm rule
(`select_b_arm`) selects A2 because no moving model exists. These figures motivated B and set its scale
(section 4); they are not evidence for H-B, and A's seeds are not reused.

## 3. Design

- Condition: the A2 start (E0_static target, 10 m, same altitude, clear line of sight, pursuer turned
  180°), Policy F and the frozen evaluation contract unchanged.
- Arms: **B0** (Policy F alone) and **B1** (Policy F + supervisor, `NAVRL_KROC2027_SEARCH=b1`).
- Seeds 4301–4307 (unused before; independent of A's 4201–4207 and the canaries' 4190/4104).
- Densities 70, 115, 160 and 205 bars (the gate's density rule; about 26 GPU hours remain).
- 2 arms × 4 densities × 7 seeds = 56 cells; 128 environments × 16 quota episodes; timeout 600 actions.
- Smoke run **KROC27-B-S01** before the main run: seed 4300 (unused), B1 only, 115 and 205 bars,
  4 quota episodes per environment. It checks the B1 task-path invariants below; its outcomes are not
  used.

## 4. Analysis (fixed now)

- Per seed, densities weighted equally; inference unit = seed (n = 7).
- **Primary:** capture(B1) − capture(B0), two-sided paired t over seeds, α = 0.05 (single test).
- **Practical threshold: 1 pp.** The whole hidden-start deficit in A was 2.84 pp, so the A-plan's 3 pp
  is out of reach by construction; 1 pp is about a third of the deficit. Fixed here, from A (exploratory),
  before any B data. Whether the 95 % interval clears 0.5, 1, 2 and 3 pp is also reported.
- **Decision labels:** `INVALID` (any integrity failure), `INCONCLUSIVE` (p ≥ 0.05),
  `OPPOSITE_DIRECTION` (significant decrease), `SUPPORTED` (significant increase),
  `PRACTICALLY_SUPPORTED` (and the 95 % interval's lower bound above 1 pp).
- **Safety criteria (fixed now):** crash(B1) − crash(B0) and timeout(B1) − timeout(B0), each with a
  two-sided 95 % paired-t interval. **Adoption** of B1 as an improvement needs `SUPPORTED` capture *and*
  both upper bounds at or below +0.5 pp (non-inferiority margin 0.5 pp). The protocol's stricter
  "upper bound at or below 0 pp" is reported as well. These are simulation criteria, not flight-safety
  claims.
- **Secondary:** never acquired, first-acquisition latency among acquired episodes (reported with the
  acquisition rate; never-acquired latency is null), recovery fraction
  (B1 − B0) / 2.84 pp (exploratory), search use (episodes searched, steps, suppression, re-searches),
  crashes and timeouts with the search active at the end (an attribution flag, not a causal claim).
- **Sensitivity:** BCa 95 % and exact sign-flip from the pinned replication helpers.

## 5. B1 task-path invariants (smoke and every B1 cell; any violation stops the run)

Per episode, from the recorder on the real task path:

1. the initial search starts at action 10 (1.0 s) and only if no fused measurement occurred before it;
2. while searching, the commanded horizontal speed is 0 and the yaw command is at most 0.6 rad/s;
3. the actor's previous-action feedback equals the executed action (horizontal 0, yaw equal to the
   supervisor's output);
4. while suppressed (LiDAR clearance at or below 0.45 m or invalid) the yaw command is 0;
5. no search step occurs on an observation with a fused measurement and an active track.

The body yaw rate during search is recorded as a diagnostic.

## 6. Integrity and stop rules

Every cell: receipt OK; result, ledger rows and recorder files match their recorded SHA-256 when
recomputed; the receipt's commit, view, search mode, arm, seed, density, run id, quota and protocol hash
equal the cell's contract; recorder records cover every ledger episode and agree on outcomes and first
acquisition; motion gates and swept clearance pass (static target). Before every cell: HEAD unchanged,
tracked tree clean, the gate still matches HEAD, the A analysis, this file and the approval; the RTX
3070 present. A completed cell is skipped only if it still passes all of this. The run stops at
`--max-gpu-hours` (a running cell is killed and kept as partial). No seed or cell is replaced.

## 7. GPU budget

From the A main run's receipts, B0 ≈ A2 = 1.66 h; B1 ran faster than A2 in the canary. Planned:
B smoke 0.25 h, B main 4.2 h (3.3 h × 1.25), run cap 5.0 h. GPU used so far about 3.8 h of 30 h.

## 8. Limitations

Static target only; "hidden" means out of the field of view, not occluded; the supervisor's
parameters were fixed in plan v2 and are not tuned on any data; an effect is the wrapper's, not yaw
scanning's alone; simulation only.
