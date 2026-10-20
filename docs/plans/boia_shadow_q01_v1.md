# BOIA-SHADOW-Q01 v1 — passive N1 qualification preregistration

2026-10-21 KST. G4-R6 authorizes implementation and CPU verification, **not GPU**.
This document and [executable configuration](boia_shadow_q01_v1.json) must be in
the exact source commit explicitly approved for future execution. The launcher's
`--approved-commit` binds that commit without a self-referential hash in this file.
No source, config, tests, WORKLOG or tracked documentation may change during GPU work.

Purpose: qualify permitted-packet acquisition, phase/ID joins, passive logging and
exact OFF/ON noninterference. No mission-effect, H-R3 confirmation, physical safety,
identity calibration, full continuation model or real-time-planner claim is tested.
RGB-D + LiDAR / actor898 / Frozen F / riskcap / controller remain unchanged.

- N1, seed4901, moving E2_direct_l2_v1,70bars, initially visible at10m.
- OFF sensor correction lineage; native F sensing/physics/RNG retained. No reset/RNG
  replacement, new sensor correction, intervention, PPO or branch simulator.
- S0: passive ON, at most20actions /120s process wall.
- S1_OFF and S1_ON: fresh processes, at most200actions /300s each; stop at first
  completed episode. Native task cap600actions /0.1s remains unchanged.
- Cumulative GPU-process wall budget720s; no retry/automatic expansion. Supervisor
  starts shutdown two seconds before the wall cap. Process action/wall limits are
  censoring, never fabricated task timeouts. Failed runs/partial files remain.
- Diagnostics every5actions. Exact protocol/fingerprints/contacts everyaction.
  No actual mission model: utility ABSTAIN, safety INPUT_CERTIFICATE_UNAVAILABLE,
  selector NOT_EVALUATED, physical safety NOT_ASSESSED, actual ownerF/appliedFalse.
- Camera/LiDAR timing is DECLARED_SIM_SCHEDULE relative to first explicit reset,
  not hardware measured time. Pinned zero latency/no perturbation/frame/layout are
  runtime guards; missing clocks refuse evidence. Host monotonic time is separate.
- Preflight free bytes>=3.3GB; stop<3GB. Total new output<=256MB; at most8 full
  sensor snapshots across stages. Every sampled frame records byte hashes, shapes,
  permitted small sensor/state values and categorical post-Warp censoring.
- Device planning ceiling=min6656MiB,total−768MiB. Actual VRAM NOT_MEASURED.
  Device queries/OOM/build/runtime compatibility are future runtime gates.
- Per-sample observer watchdog2s /three consecutive sampled observations>500ms;
  complete pipeline host copy/schema/diagnostic/hash/log costs are recorded apart
  from native full-step time and tiny diagnostic-only time. No5ms/10Hz guarantee.

OFF/ON gate: exact equality of join IDs, deterministic sensor and interpretation
metadata, actor input, policy action, transformed/governed/PI command, RNG states,
consumed histories, state/outcomes/contact, and returned normal/reset observation,
actor/history/obstacle/target/RNG fingerprints. Host time/latency are excluded by
definition. No sensor tolerance, row exclusion or salvage. ON compares against
completed OFF records at each phase and stops at the first mismatch.

Ground truth remains exclusively evaluator-side: outcomes and target/obstacle
fingerprints never enter SensorPacket or BOIA diagnostics. Pre-mask contact and
height/OOB are captured before the original compute mutates its aliased crash
buffer. Substep contact uses the existing ContactChain /0.05N threshold. Prospective
collision-free capture requires original capture plus zero accumulated contact and
no action-boundary height/OOB violation. This qualified proxy does not certify
absence of subthreshold contact or between-boundary height/OOB excursion.

Any source/config/checkpoint, import, packet/phase/count, F-action, reset, RMS,
logger, resource or OFF/ON mismatch stops the campaign without another command or
fallback. B03/B04 remain unresolved; historical results and missing metrics stay
unchanged. G5 HOLD. Successful Q01 would qualify these mechanics only.

Exact commands and output paths:
[GPU_SHADOW_RUNBOOK](../evidence/boia_g4r6_shadow_preflight_2026-10-21/GPU_SHADOW_RUNBOOK.md).
