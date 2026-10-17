# S-L1C independent review and limit correction — 2026-10-18

This addendum is dated after the `343f0d5` result commit. [`REPORT.md`](REPORT.md) is hashed in
[`index.json`](index.json) and is left unedited. Where the two disagree, this addendum supersedes
the report.

## Independent review

The reviewer was a read-only Claude subagent running `claude-fable-5-1` (Fable 5.1).

- **First attempt.** It stopped early on an API safeguard error (`general_harms`) and produced no
  verdict. Before stopping it reported only that the four CPU test files passed. That attempt is
  not counted as a review.
- **Second attempt.** The prompt was reworded neutrally, and the same model completed the review.
  Its verdict was **GO-WITH-NOTES, with no blocker**.

The reviewer verified these points itself:

- the code matches the preregistered formulas, with no frame or sign error;
- the positive eligibility is defined by geometry, not by the S-L1C rule;
- the GPU negatives run the production function;
- the launcher's trace wrapper cannot change behaviour;
- ground truth is used only for fixture priming and evaluation;
- whole-file SHA-256 matches EL0-R1, SL1A-C01 and SL1B-C01, and P1 matches P2;
- every report number agrees with the analysis output, and the analysis re-run is byte-identical;
- manifests verify: SL1C-C01 176/176, SL1A-C01 and SL1B-C01 138/138, EL0-R1 99/99;
- the CPU tests pass;
- the S-L1B switch-test edit is acceptable, because it tightens the test rather than loosening it.

## Correction: undeclared recall-loss band

The preregistration and the report state two things that are too narrow:

- They claim that beyond 1.152 m a 0.20 m sphere cannot return on two rows. That holds only for a
  sphere centred on a row.
- They state the short-range limit as "refused below 1.152 m".

A sphere whose centre lies between two rows, which are 10° apart, can return on both up to
0.20/sin(5°) = **2.295 m**. Its two horizontal ranges then differ by only 2–3 cm, so the shape
test marks it as vertically extended and rejects a genuine return.

The reviewer found this band, and the reference reproduces it on CPU. At bearing 0°, with
elevations swept over −8.5° to +17.5° in 0.5° steps:

| Centre range | Elevations with a genuine LiDAR hit | Rejected by S-L1C |
| --- | --- | --- |
| 1.16 m | 53 | 47 |
| 1.50 m | 53 | 23 |
| 2.00 m | 53 | 8 |
| 2.28 m | 53 | 2 |
| 2.30 m | 50 | 0 |
| 3.00 m | 38 | 0 |

**Corrected limit.** S-L1C LiDAR association is refused below 1.152 m. For centres between rows it
is also refused up to 2.295 m.

- **Fixtures.** None of the preregistered fixtures fell in the band. S22 at 2.0 m is on a row, and
  S03 sits on the −10° row. C-POS therefore did not measure the loss.
- **Gates.** The preregistered gates are unaffected, and 117/117 positives are kept.
- **Closed loop.** Evaluation-only trajectories show about 2–3% of steps inside this band, which is
  consistent with the lower S-L1C LiDAR accept counts.
- **Proposed fix (not implemented).** Make the shape test elevation-aware: ignore an adjacent row
  that the predicted sphere itself can reach. Add exact fixtures at 1.2–2.3 m off-row. This needs
  a new preregistered version (`s_l1c_v2`); it is not a post-hoc change to v1.

## Other notes

- **C-POS short-range recall.** C-POS eligibility includes the 1.152 m cutoff by
  preregistration, so short-range recall is defined out of the gate rather than measured.
- **Fixture variety.** It is modest: 13 eligible scenes, 7 negatives, and only tall vertical bars.
  The reviewer swept 105 tall-bar negatives; 5 were accepted, all with a front face ≥ 11.38 m,
  inside the declared residual band. Spheres near bars (behind, beside, on a pedestal) were all
  accepted.
- **Report timing.** `REPORT.md` gives closed-loop wall times of "46.9–52.0 s"; the minimum is
  actually 46.76 s.
- **No-eligible-row case.** With no eligible row, the helper returns the far range instead of the
  column minimum. This is harmless, because it is never used when the association is invalid.

## Decision

**S-L1C: GO.**

- All preregistered gates pass, on independent CPU and GPU evidence.
- The lower-lineage regression passes, and the originals are intact.
- The fixed source SHA is `4d5911d`.
- There is no unresolved blocker.

The corrected recall band is an inherited limitation of `s_l1c_v1`. S-L1D's preregistration names
it, and no S-L1D gate depends on S-L1C recall in that band.
