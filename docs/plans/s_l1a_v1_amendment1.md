# S-L1A v1 amendment 1 — match historical non-interference seeds

Recorded before any S-L1A GPU process or measurement. The original
[plan](s_l1a_v1.md) and [JSON](s_l1a_v1.json) remain unchanged. The
[amendment JSON](s_l1a_v1_amendment1.json) governs this one override.

The plan's shared closed-loop seed 4401 was an incorrect assumption about the old E-L0 runs.
Inspection of the preserved E-L0 plan and run metadata shows **static seed 4501** and **moving
seed 4502** for the N8, 600-step, 160-bar P2 runs. To perform the preregistered strict historical
digest comparison, all S-L1A closed-loop arms now use 4501 for static and 4502 for moving.
Sensor fixture construction retains its seed 4401.

This repairs the comparison configuration before measurement. It selects no seed based on
S-L1A performance; no S-L1A measurement exists. All hypotheses, gates, geometry, correction rules,
sample sizes, limits and interpretation remain unchanged. The loader must bind both plan versions
and record the effective per-process seeds. No cross-seed bitwise-equality claim is permitted.
