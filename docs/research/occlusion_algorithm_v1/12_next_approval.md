# Current authority — G1-R and CPU-only G4 approved (2026-10-19)

Original PDF provision is **COMPLETE**:30files,29unique papers. G1-R reading/corrections are complete
with **MINOR_ADJUSTMENTS_PROPOSED** ([13](13_design_deltas.md)). The user approved **U1–U7 CPU-only**
after that gate, not only U1. This supersedes the historical request below; it is retained as history.
Current disk stop is **3GB**, explicitly amended by the user after the initial8GB startup stop.
G5 simulator integration/raw calibration and G6+ GPU/pilots remain **NOT_APPROVED**. No production
change, PPO, push or main merge. Minor design proposals remain unapplied;07 is the implementation
basis. Required U7 work is synthetic replay plus raw schema inventory only, not calibration.

---

# 12 — Requested approval: first code unit (G4-U1)

**Request.** Approve G4-U1 only: an opt-in, CPU-only geometry and visibility kernel for BOIA.

## Scope

- **New files:**
  - `motar_eval/boia/__init__.py`;
  - `motar_eval/boia/types.py`;
  - `motar_eval/boia/geometry.py`;
  - `motar_eval/boia/visibility.py` (oracle p_det mode only);
  - `tests/test_boia_geometry_visibility.py`.
- **No edits** to any existing file, apart from an appended `WORKLOG.md` entry and the Beads issue
  status.
- **Functions** (signatures as in [08](08_implementation_plan.md)):
  - `lidar_points_xy`;
  - `exclude_target`;
  - `seg_point_dist`;
  - `known_free`;
  - `forecast` (oracle mode);
  - `p_any`;
  - `bpod_analytic`, for the ablation comparison.
- **Tests:** the seven U1 tests in 08, run on synthetic scenes only. No simulator import, no GPU,
  no raw data.
- **Size:** about ≤ 500 lines of source plus tests.
- **Checks:** focused tests, then the full PUBLIC_CPU once. The PUBLIC_CPU result is reported
  together with the pre-existing `motar-6nn.5` failure.
- **Commit:** one local commit after the tests pass. **No push.**

## Out of scope for this approval

- any change to production code, the 898-D actor, the governor, the controller or the tracker;
- any simulator run, CPU replay of raw data, or GPU work;
- U2–U7.

## Why this unit first

- **It is the core new computation.** The joint Monte Carlo detection forecast against sensed bars
  (07 §5) is where BOIA differs most from BPOD.
- **It can be tested completely on synthetic geometry.** That includes the bias test of BPOD's
  independence product for adjacent bars, which directly tests a design premise.
- **It touches nothing that could leak GT.** Its inputs are plain tensors.

## Decisions requested from the user

1. Approve G4-U1 as scoped above: yes or no.
2. Optionally provide the original PDFs:
   - YOPO and OCTrack (NOT_AVAILABLE);
   - the user's versions of EGO-Planner, ORTrack, Star-Convex and Lee SE(3).
3. Confirm the scheduling stance: this lineage stays secondary to the KROC2027 freeze (2026-10-28)
   and paper (`motar-d43`).
