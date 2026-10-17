# S-L1D independent review, corrections and status — 2026-10-18

This addendum is dated after the `aab7379` result commit. [`REPORT.md`](REPORT.md) is hashed in
[`index.json`](index.json), so it is not edited. Where the two disagree, this addendum supersedes
the report.

## Independent review

- **Reviewer:** a read-only Claude subagent on `claude-fable-5-1` (Fable 5.1), prompted neutrally.
- **Verdict:** **GO-WITH-NOTES, no blocker.**
- **What it confirmed:**
  - The production rule equals the E-L0 target-alone rule and the preregistered precedence.
  - The online function reads no evaluation label, renderer mask or true position.
  - GT priming happens only in fixture probes.
  - The closed-loop trace wrapper is inert: s_l1d is bitwise equal to s_l1c, and s_l1c ran
    without the wrapper.
- **What it reproduced:**
  - The analysis, byte for byte.
  - Raw and committed records: 131/131.
  - Earlier manifests.
  - The CPU tests.
- **Probes:** 18 additional cases (production equals oracle in all of them).

## Corrections

1. **Scale-loss and missing preregistered CPU fixtures.** The report says the scale-loss
   precedence is "evidenced by the CPU oracle fixtures". At `aab7379` that was false.
   - `test_scale_loss_band` asserted only inside `if alone_px < 2` branches, which never fired, so
     the test was vacuous.
   - The preregistered CPU cases were not all realized: FOV edge 40/44°, the S14-like rim gap, and
     the 0.2 m primed offset were missing.
   - These cases are now asserted in
     [`tests/test_s_l1d_fixtures.py`](../../../tests/test_s_l1d_fixtures.py). Each checks that
     production equals the oracle and applies the preregistered mapping:
     - clear targets at 2–24 m;
     - FOV edge 40/44/50°;
     - scale loss: a one-pixel target at 13.5 m gives PRED_SCALE_LOSS; the same target behind a
       bar is E-L0 FULL with alone 1, and still gives PRED_SCALE_LOSS (scale takes precedence);
       an on-axis 20.05 m target is E-L0 SCALE_LOSS and gives PRED_SCALE_LOSS;
     - full occlusion, centre-only and partial;
     - the S14-like rim gap: E-L0 CENTRE_ONLY, 56 alone / 16 visible pixels, giving a
       partial-type class;
     - the 0.2 m offset.
   - The production code is unchanged from `76f0f4c`, no gate changes, and the result is 5/5 OK.
     This completes a preregistered CPU validation after the GPU run. It changes neither the GPU
     result nor its criteria.
2. **The "predicted centre" is the current-step posterior.** It is taken after the camera update
   and the S-L1C LiDAR update, as `REPORT.md` says. The `.md` wording "predicted" is imprecise.
   The input remains sensor-only, and fixtures prime the state directly, so no gate is affected.
3. **The GPU-informative set is FULL only.** The 18 RANGE/FOV cases have support 0. There, the
   online rule reduces to the E-L0 geometry on the primed true centre, which D-ORACLE already
   implies. The depth/τ mechanism is exercised on real GPU depth only by the 4 FULL scenes: S11,
   S16, S17 and S19. The distinct online geometries number 7, not 10.
4. **Minor points.**
   - D-SEPARATION's AST check screens names only. The data flow was confirmed by reading the code.
   - The S-L1D JSON inherits two plan paths from the S-L1C plan.
   - The static closed-loop table omits PARTIAL → DETECTED 86.
   - Traces hold 601 observe calls (a reset plus 600 steps).

## Status decision: S-L1D INCOMPLETE (not GO)

Two of the user's S-L1D acceptance items are only partly evidenced on GPU:

- The comparison with *actual GPU RGB-D/LiDAR observations* holds for support, FOV exit, range exit
  and full occlusion.
- The preregistered fixtures for *partial, centre-only and scale* pass on CPU only. In SL1D-C01,
  every GPU CENTRE_ONLY (9) and PARTIAL (3) fixture pass was detected by the camera, and no GPU
  fixture was SCALE_LOSS. Those online classes were therefore never compared with real GPU depth in
  an undetected state.

Under the run's decision rules, required independent evidence is missing, so S-L1D is
**INCOMPLETE**. The work done so far is recorded at these commits:

| Item | Commit |
| --- | --- |
| Preregistration | `e0c4229` |
| Oracle | `02be6aa` |
| Runtime | `4e46c16` |
| Tools | `76f0f4c` |
| Results | `aab7379` |
| CPU fixture completion | this commit |

## Resume instructions

1. **Amendment.** Write a dated `s_l1d_v1_amendment1` that adds GPU sensor fixtures for the
   uncovered undetected states, with gates and mapping unchanged:
   - far in-frustum targets with one or zero pixel-centre support, at 13–20.1 m with lateral and
     vertical offsets (the oracle search in `tests/test_s_l1d_fixtures.py` finds positions);
   - far targets (8–12 m) partly covered by a bar edge, leaving fewer than `min_target_pixels`
     visible, with the centre ray both blocked and open.

   Bar sizes in the GPU fixture are randomised per slot. Design these scenes by an edge position
   (`edge_left_y` / `edge_right_y`) and accept whatever class E-L0 assigns, since the mapping is
   evaluator-relative.
2. **Canary.** Run a sensor-only canary `SL1D-C02` (2 processes, about 25 s) at a fixed SHA, with
   analysis, evidence, an independent review and a push.
3. **Decision.** Record S-L1D GO only if the new undetected partial, centre-only and scale cases
   meet the preregistered mapping on GPU.
