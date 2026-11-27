# 10 — Roadmap: domestic-conference MVP and international-paper extension

**Scheduling constraint.** The KROC2027 study is P0. Its Simulation Freeze is 2026-10-28 and the
internal paper is due 2026-11-13 (`motar-d43`). This occlusion-algorithm lineage is a **separate,
later research line**. It must not displace the KROC paper, nor change any frozen KROC record or
Policy F result.

## Stage plan

| Stage | Content | Approval | Rough size |
| --- | --- | --- | --- |
| G0–G3 | Paper and code audit, MOTAR audit, design (this folder) | Approved, **done** | — |
| G1b | Read-only check of prior art outside the 25 (`motar-3p3.6`) | Approved scope (read-only) | 1–2 sessions |
| G4 | U1 → U7 opt-in CPU modules, one approval per unit (08) | **Each unit needs user approval** | U1 about ≤ 500 lines + tests; U1–U7 about 1.5–2 k lines |
| G5 | CPU integration + replay calibration (V2–V5) | User approval | Depends on what the raw traces hold |
| G6 | GPU qualification (one seed, one density, F0 + F5) | Preregistration + explicit GPU approval | Under 1 GPU-h expected; NOT_MEASURED |
| G7 | Exploratory pilot (4 seeds × 2 densities × 6 arms) | Preregistration + explicit GPU approval | Sized after G6 |
| G8 | Confirmatory study (new seeds, preregistered family) | Separate approval | Sized by simulation |
| G9 | Learned extension (candidate C) | Separate approval; new checkpoint lineage | Weeks |

## Domestic-conference MVP (first paper)

**Claim shape (only if the evidence supports it).** In a simulated forward RGB-D + LiDAR interceptor
with a frozen RL policy, a calibrated arbitration layer reduces crash risk after occlusion onset
without lowering capture. The comparison is against:
- the policy alone;
- a visibility-blind safety layer;
- a reactive visibility-recovery baseline.

**Required evidence.**
- V0–V7, with the primary endpoint and the guards.
- The detector-realism stress arm.
- The honest prior-art statement (06).

**Explicitly not claimed.**
- Physical safety (P-L1/G-L1 are not done);
- real flight;
- state of the art;
- novelty of (a), (b) or (d) in 06 §2.

## International-paper extension

1. **First acquisition / hidden start.** A UAST-style explored-region search, with a belief over the
   unseen target added to BOIA's mode NEVER_ACQUIRED. This targets the 32–62 % never-acquired failure
   share (M3 §2).
2. **Learned scorer (candidate C).** Trained offline on logged decisions; Policy F stays frozen.
3. **Detector realism.** Replace the oracle colour-mask detector with a learned or perturbed
   detector qualified separately. This is a production-perception change and needs its own lineage
   approval.
4. **Generalisation.** Independent layouts, the non-overlapping placement lineage (never pooled),
   other target behaviours and speed strata.
5. **External baseline.** A matched MOTAR re-implementation of Eva-Tracker and UAST ideas, labelled
   NOT_REPRODUCED until it matches.
