# Q02 handoff — after Q02-A

2026-10-21 KST, `codex/mt-p1`, local commits only. CPU only; GPU NOT_RUN; G5 HOLD.

## Done in Q02-A

- Code audit with file:line evidence and the fixed contracts: [Q02A_SPEC.md](Q02A_SPEC.md).
- `motar_eval/q02/` (numpy-only, no simulator/torch/RNG imports):
  `contracts.py` (frames, camera/LiDAR models, immutable `EvidenceInput`, Q01 packet adapters),
  `geometry.py` (yaw frames, pinhole projection, LiDAR rays, sphere first hit),
  `validation.py` (missing/stale/invalid/frame-mismatch report),
  `evidence.py` (evidence record + deterministic geometric hypothesis, UNKNOWN allowed).
- `tests/test_q02_evidence.py` (16 CPU tests) and a read-only replay over the real Q01 frames:
  [docs/evidence/q02a_cpu_2026-10-21/](../../evidence/q02a_cpu_2026-10-21/).

## What is not established

- No detection or occlusion accuracy: no labels exist, and the camera detector is oracle-grade in
  the Frozen F profile. The Q01 frames (3 unique) all have the target visible.
- Thresholds in `EvidenceParams` are declared priors, not fitted.
- Modules B–E are contracts/designs only. No candidate, executor, timing rule or utility exists.

## Next: Q02-B (CPU first, then one approval for data)

1. **Labels design, evaluator-side.** Specify per-decision GT occlusion/FOV/LOS labels computed
   outside the estimator (renderer target mask before bar removal, LOS against obstacle meshes)
   and joined by record ID; extend the Q01 recorder in a new versioned observer, not in production.
2. **Module C v0** on the CPU: short-window sensor-only polar map from LiDAR rays and camera depth
   columns, unknown ≠ free, reusing `boia/geometry.known_free`; tests on synthetic scenes.
3. **Module B forecasts** with `boia.belief.predict` beyond the decision only; coverage checks need
   labels.
4. **Module D v0**: score `boia/candidates` commands with visibility gain, clearance and unknown
   exposure; `NO_FEASIBLE_PLAN` when the safety screen rejects all; no execution.
5. **Module E** stays feature logging + ABSTAIN until matched CONTINUE/REACQUIRE outcomes exist.
6. Then a preregistered data-collection run (needs separate GPU approval): hidden initial view,
   several seeds/densities, full packets every decision, evaluator labels.

Open Beads: `motar-3p3.13`, `motar-3p3.15` (BOIA P0 model/sensor blockers), `motar-3p3.24`
(Q01-V2 verify P3s), `motar-6nn.5` (PUBLIC_CPU 4 known errors).
