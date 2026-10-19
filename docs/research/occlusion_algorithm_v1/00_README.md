# Occlusion-handling navigation algorithm v1 — research and design record

**Status: G1-R COMPLETE (2026-10-19, Codex); G4 CPU U1–U7 IN_PROGRESS; no simulator validation.** Nothing is validated. Nothing
in this folder is a performance claim. Proposed equations and modules are research hypotheses until
CPU and simulator evidence exists.

## Scope and authority

- **Request.** On 2026-10-19 the user asked for this work: design a new occlusion-handling UAV
  navigation algorithm on the basis of a 25-paper set.
- **Stages approved:**
  - **G0:** state and PDF check.
  - **G1:** paper reading and official source audit.
  - **G2:** MOTAR perception, dynamics and occlusion-failure audit.
  - **G3:** algorithm selection and mathematical/software design.
- **Additional authority (2026-10-19):** G1-R original25+4 papers and CPU-only G4 U1–U7.
- **Not approved:**
  - G5 CPU integration;
  - G6/G7 GPU qualification and pilots;
  - G8 confirmation;
  - G9 learned extension.

  Each needs its own user approval, and the GPU stages need a preregistration as well.
- **Tracking.** Beads epic `motar-3p3`:
  - `.1` G0;
  - `.2` G1;
  - `.3` G2;
  - `.4` G3;
  - `.5` G4 CPU-only approved; `.7` G1-R.

## Files

| File | Content | Status |
| --- | --- | --- |
| `paper_manifest.json` | 25+4 originals plus duplicate: SHA-256, pages, read status, prior-copy identity | DONE |
| `01_paper_audit.md` | Per-paper read status, sections read, key equations with page and equation numbers | DONE |
| `02_methods_comparison.md` | Method and equation comparison across papers | DONE |
| `03_code_audit.md` | Official repositories: commit, licence, files/functions, portability | DONE |
| `04_motar_audit.md` | MOTAR sensor, tracker, dynamics and termination contracts; occlusion failure data | DONE |
| `05_candidates.md` | Three competing algorithm designs | DONE |
| `06_novelty_matrix.md` | Prior-art overlap, row by row | DONE |
| `07_recommended_spec.md` | Full mathematical specification of the recommended algorithm | DONE |
| `08_implementation_plan.md` | Files, functions, shapes and staged CPU tests | DONE |
| `09_experiment_plan.md` | Density × target motion × occlusion experiment design | DONE |
| `10_roadmap.md` | Domestic-conference MVP and international-paper extension | DONE |
| `11_assumptions_risks.md` | Unverified assumptions, risks, stop conditions | DONE |
| `12_next_approval.md` | Current G4 authority plus preserved historical request | DONE |
| `13_design_deltas.md` | Original-copy corrections and unapplied proposals | G1-R DONE |
| `notes/P6_extra_papers.md` | Four extra original-paper full reads | DONE |
| `reading_receipts.json` | Per-reader pages, checks, original SHA and design findings | DONE |

## G0 result and G1-R original-copy update

The prior public-copy audit had19 byte-identical papers, four different versions and two missing.
That history is preserved in `paper_manifest.v1.json` and the earlier dated notes. The supplied
originals now resolve availability/version identity:30PDF files represent25core+4extra papers and
one byte-identical EGO-Planner duplicate. All30sources are unchanged by the reading pass. Of the
original25,24match the available user-inventory SHA; YOPO's inventorySHA is null, not guessed.

All29papers have cumulativeFULL_READ status:15were fully read now and14earlier identical-copy
FULL_READ records were checked against the originals. Physical read pages and spot checks are in
[01](01_paper_audit.md) and [receipts](reading_receipts.json). The originals remain outside Git.

## Conclusions of G0–G3 (detail in the numbered files)

1. **Where the measurable failure is.** Policy F's measurable occlusion-related failure is not lost
   tracking. All of the following are associations, not causes (04 §3):
   - it re-observes FOV exits 96–99 % of the time, and geometric losses 77–82 % of the time within 2 s;
   - fused resumption after restoration is 100 %.

   The failure is **collision while the target is occluded or uncertain**: 3.23 % (160 bars) and
   3.82 % (205 bars) of geometric-occlusion onsets end in a crash within 2 s, and the governor is
   not a brake. This is the primary target.
2. **Oracle detector.** MOTAR's camera detector reproduces the GT silhouette through a colour code.
   Detector realism must therefore be an experimental factor.
3. **Recommended design.** BOIA, a belief-conditioned occlusion/interception arbitration over the
   frozen Policy F (05, 07). It combines:
   - a joint Monte Carlo detection forecast (BOIA's form of BPOD Eq.9);
   - BPOD's expected covariance (Eq.11d);
   - a governor-aware rollout of about 20 finite candidates, Policy F's own command included;
   - a FASTER-style stopping feasibility screen with two tiers;
   - capture-aware cost;
   - calibrated-advantage arbitration with hysteresis;
   - terminal-first handling.
4. **Novelty.** Elements(a),(b),(d) are prior art. PRIMER also covers generic learned-plan safety
   rejection/fallback, so that broad claim is NOT_NOVEL_ENOUGH. Narrow calibrated advantage over
   FrozenF remains CANDIDATE_CONTRIBUTION_UNVERIFIED; G1b `motar-3p3.6` is still open.
5. **Next step.** Implement the approved CPU-only U1–U7 sequentially; no G5/simulator/GPU work.
   G1-R conclusion is MINOR_ADJUSTMENTS_PROPOSED; proposals in13are not applied to07–09.

## How to resume

1. Run `bd show motar-3p3` and `bd ready --label occ-algo`.
2. The full G1/G2 working notes, with page/equation and file:line detail, are in
   [`notes/`](notes/), together with M3's analysis scripts and outputs in `notes/m3_scripts/`. The
   numbered files above are curated from them.
3. G1-R committedf103721; U5 focusedCPU16PASS; next unit pending localcommit/gate. Diskstop3GB; G5/GPU unapproved.
