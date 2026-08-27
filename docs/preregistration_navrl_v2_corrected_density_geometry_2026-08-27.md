# Preregistration — corrected-v2 density geometry contract

Date frozen: 2026-08-27.

Geometry **gates** were written into `VERIFICATION.md` before the audit JSON
existed. This document freezes the **density curriculum contract** implied by
those gates and the canonical receipt. It does not authorize GPU work.

## Binding measurement

Receipt:
`results/navrl_v2_density_geometry_audit_2026-08-27/density_geometry_canonical_6_28.json`
SHA-256 `6b1f1b36cf73409d0c09483c3e1767b7ff196aecf0b95769f0e07d8dffa268d5`.

| item | frozen value |
|---|---|
| placement | `footprint_clearance`, surface clearance 0.45 m, merge fallback 0 |
| arena | 40 × 40 × 3 m |
| goal band for this cap | canonical 6–28 m |
| binding inflation | body circumradius + 0.45 m tracking reserve |
| connectivity gate | ≥ 95% |
| no-route gate | ≤ 5% |
| generation-failure gate | 0 |
| 205 bars | **PASS** (99.167% / 0.833% / 0) |
| highest passing density | 250 bars |
| 300 bars | **FAIL** (94.661% / 5.339% / 0) |

## Density contract

1. Fresh training target remains **70 → 205 bars**, step 15, minimum dwell 1,000
   epochs/level. 250 passing does **not** raise the training cap.
2. Connected evaluation / OOD cells are 64, 70, 100, 130, 160, 190, 205, 220, 250.
3. 300 bars is an **asset ceiling and disconnected-stress** cell, not a connected
   OOD claim. Do not write “300-bar connected free space” from this lineage.
4. `body_only` connectivity is not a training gate.
5. YOPOv2 4 m / 5 m count-density cells (100 / 64 bars) remain reference labels
   only. They are not a second training cap.

## Explicit non-authority

This freeze does **not** authorize:

- PPO or any other policy training
- 500-epoch smoke
- Track A detection Stage 2
- Track B recovery/route GPU reruns, 1.5 m/s claims, or env-count changes
- raising 205 because 220/250 also passed
- treating 300 as a connected evaluation density

The still-blocked next GPU step, if authority is later lifted, is a separate
engineering/smoke preregistration. Hardware Track A remains
`docs/SIM2REAL_3DAY_EXECUTION_PLAN.md`.

## Claim boundary

Supported: under non-overlap `footprint_clearance` layouts and canonical 6–28 m
pairs, 205-bar free space is connected after body+tracking inflation.

Unsupported: corrected-v2 PPO performance; hard 22.5–28 m connectivity; hardware
validation; carrying historical overlap-permitting capture curves into this
lineage.
