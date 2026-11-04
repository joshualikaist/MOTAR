# Amendment 1 to the GR-0 preregistration: KROC2027 A3 placement for every cell

Dated 2026-11-04. Amends [`prereg_gain_randomization_gr0_v1.md`](prereg_gain_randomization_gr0_v1.md); that file is
unchanged. No GR-0 cell (stage 2) has been run.

## What happened

The identity stage ran once, from commit `d38b247`. Its output is
`aerial_gym_simulator/results/gain_randomization_gr0_v1/identity_01/`, and `identity_report.json` has SHA-256
`b7f45bf0…3b18`. The verdict is **IDENTITY_FAIL**, and it stays recorded.

- All four cells stopped after the first control step: the base-tree reference `ref` as well as `off`, `s00` and
  `s40`. Each wrote no result.
- In each cell the KROC2027 recorder raised "target-motion contract violated". Its swept-clearance check found the
  target at clearance 0 from a bar in 3 of 128 environments, in the first judged step.
- The runner had not set the KROC2027 initial-view placement. The task's own spawn sampling then let a target start
  overlapping a bar, and the recorder correctly stopped the cell.
- No comparison was made, so the failure says nothing about OFF against the native run.

## What changes

`tools/gr_run.py` now builds every cell with the KROC2027 **A3** placement: a moving target, the initial view
`visible` at 10 m, the protocol's spawn-clearance and line-of-sight rules, and arm label `A3`. This is the
condition B1-R used for Policy F (`tools/b1r_run.py`: `view='visible'`, `arm='A3'`). The stage-1 identity runs
(`ref`, `off`, `s00`, `s40`) and the stage-2 GR-0 cells all use it. The base-tree reference `ref` uses the same
placement, so the identity comparison stays like for like.

Nothing else changes: the cells, levels, seeds, quota, analysis, readings, thresholds and next-step rule are as
preregistered.

## Stop rule for this amendment

The identity stage runs once more, into `identity_02`. If it is not IDENTITY_PASS, the unit stops and goes back to
the user. The preregistration's rule that an IDENTITY_FAIL stops the unit refers to a completed comparison. This
amendment allows exactly one rerun, because the first attempt never reached a comparison.
