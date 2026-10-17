# S-L1A pre-implementation audit — 2026-10-18

Compared `183a333bd01c0124a95ae379dba0a8509aebc999` with
`289fd80722daac52123bb1e74fa785b6dc0539a2` using `git diff --name-status` and `git diff --stat`.

| Status | Exact changed path | Scope |
| --- | --- | --- |
| M | `AGENTS.md` | Standing sensor and conditional publication rules |
| M | `ARCHITECTURE.md` | RGB-D/LiDAR roles and current contract link |
| M | `PROJECT.md` | Official sensor requirement and correction link |
| M | `WORKLOG.md` | Append-only correction record |
| A | `docs/SENSOR_CONTRACT.md` | Current requirements and provisional hypotheses |
| A | `docs/plans/correction_lineages_plan_v2.md` | S-L1A/B/C/D scope and fusion roadmap |
| A | `docs/plans/sensor_contract_amendment_2026-10-18.md` | Explicit historical supersession inventory |
| A | `docs/plans/sensor_contract_amendment_2026-10-18.validation.json` | Preservation and document checks |

Total: **8 documentation files, +523 / −5 lines**. Runtime Python, URDF, controller, detector,
perception, adapter and E-L0 code changed: **none**; runtime function changes: **none**.

Independent read-only preservation check at S-L1A baseline `289fd807`: 2,054 runtime/test source
files, 101 historical EQ-0/EQ-1/EQ-1D/E-L0 evidence files, 10 raw manifests and 3,479 members match
the previous records. Both Policy F copies retain SHA-256
`f702213936601860995cf61dcc570247e72543b1976e3716055cd8ec5593ad40`. Main remains
`cb644bbe46456b1900020177617a13040a3f1552`; its three WIP files and stash `23ddb33` are unchanged.

The [sensor amendment](../../plans/sensor_contract_amendment_2026-10-18.md) inventories the remaining
camera-only/LiDAR-free roadmap text in immutable EQ records, v1 plans and old WORKLOG entries.
Those are explicitly superseded historical statements. No additional active preregistration
requiring a camera-only final goal was identified. Channel-only diagnostic flags and historical
ablations are not current sensor requirements. Existing EQ-1D/E-L0 preregistrations use both sensors;
their original bytes and outcomes remain intact.

S-L1A changes the environment under a new opt-in lineage, so its data cannot be silently pooled
with historical OCC/EQ results. The [new preregistration](../../plans/s_l1a_v1.md) records its own
scope, independent oracle, gates and GPU execution limits. No historical experiment is amended.
External backup was not established by this read-only check. Original raw remain at their recorded
paths, independently of untracked worktree symlinks.
