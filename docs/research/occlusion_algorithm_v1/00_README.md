# Occlusion-handling navigation algorithm v1 — research and design record

**Status: RESEARCH DESIGN COMPLETE (G0–G3); G4 AWAITS USER APPROVAL.** Nothing is implemented, and nothing is validated. Nothing
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
- **Not approved:**
  - G4 code implementation;
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
  - `.5` G4, which needs approval.

## Files

| File | Content | Status |
| --- | --- | --- |
| `paper_manifest.json` | 25 papers: SHA-256 against the user's inventory, pages, local path | DONE |
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
| `12_next_approval.md` | Scope of the first code unit for which approval is requested | DONE |

## G0 result: paper availability

The user's inventory came from a different machine (a ChatGPT sandbox). On this machine:
- **5 papers** were found locally with identical SHA-256.
- **14 more** were downloaded from arXiv or CVF open access, and their SHA-256 is identical to the
  user's.

That gives **19 identical copies**. For the rest:
- **4 are different versions:**
  - EGO-Planner: arXiv v2;
  - ORTrack: arXiv v1;
  - Star-Convex: arXiv v1;
  - Lee SE(3): arXiv 1003.2005v1, 8 pages, against the user's 6-page conference version.

  Their content may differ from the user's copies.
- **2 are NOT_AVAILABLE:**
  - YOPO: only on IEEE Xplore (paywalled), with no arXiv version. Its official code is audited
    instead.
  - "Rethinking Occlusion Modeling for UAV Tracking" (OCTrack): not found on arXiv or CVF.

Neither missing paper is summarised from memory. The PDFs are stored outside every Git repository,
at `/home/fair/workspaces/aerial_gym_ws/reference/papers_occlusion_v1/`.

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
4. **Novelty.** BOIA elements (a), (b) and (d) are prior art. Element (c), the arbitration, is not
   in the 23 papers read, but is **unverified** against literature outside the 25 (06 §3; Beads
   `motar-3p3.6`).
5. **Next step.** G4-U1, the CPU geometry and visibility kernel (12). It needs the user's approval.

## How to resume

1. Run `bd show motar-3p3` and `bd ready --label occ-algo`.
2. The full G1/G2 working notes, with page/equation and file:line detail, are in
   [`notes/`](notes/), together with M3's analysis scripts and outputs in `notes/m3_scripts/`. The
   numbered files above are curated from them.
3. Do not start G4 without the user's approval of `12_next_approval.md`.
