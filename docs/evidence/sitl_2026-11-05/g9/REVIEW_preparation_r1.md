# G9 attempt 3 preparation review

Date: 2026-11-05 KST. Reviewer: Codex child agent `/root/g9_resume_audit`; exact underlying model identifier is not exposed. This is not the requested Opus GR-1b results review.

Scope: read-only source audit of `MOTAR-camonly/tools/sitl_g9_lee_steps.py` at pause HEAD `5fd8d7d` and external `gpu_queue_20261105/apply_g9_bar_placement_fix.py`. No tests, GPU/CPU experiment, or repository writes were performed. Reading is CONDITIONAL until the preparation issues below are addressed and the intended runner is smoke checked.

## Findings

1. **MAJOR — the raw simulation does not implement the stated one-bar contract.** `tools/sitl_g9_lee_steps.py:33` sets `NAVRL_NUM_BARS=1`, but raw SimBuilder never reads that task-level active-density knob. `aerial_gym/config/asset_config/env_object_config.py:810` instead builds `NAVRL_MAX_BARS` assets, default 150. `aerial_gym/env_manager/env_manager.py:196-206,278` sets `num_obstacles_in_env` to the loaded asset count, and `asset_manager.py:121` deactivates only indices past that count. The proposed patch at lines 22-23 writes every asset's XY to a common location and its distance record at line 25 watches only asset index 0. Pin `NAVRL_MAX_BARS=1` before imports and reject any physical-target/distractor environment or asset count other than exactly one. Record the count in the new attempt. The previous attempt's claimed one-bar scene is unverified and must retain its original record.

2. **MEDIUM — environment values can override the intended dynamics/scene.** `sitl_g9_lee_steps.py:28-34` uses `setdefault`, allowing e.g. an inherited smaller arena or different bar pool. The proposed start (5,5) and fixed diagonal path assume the 40 m arena and a static bar-only scene. Sanitize/pin the launch environment or validate these contract values before construction; capture effective values in the output. This is a pre-existing risk that matters to the placement repair.

3. **MEDIUM — metrics alone do not establish a clear trajectory or healthy host.** The patch records minimum bar-center XY distance, but the runner only records dirty status (`:106`) and host fault counts (`:178`), without rejecting a dirty tree or increased count. It does not record contact force, altitude extrema, or whole-run tilt extrema. Before accepting the new response reference, use the external wrapper to require a clean, expected commit and unchanged faults and inspect all envs' response completeness/finite values. Prefer recording contact force and whole-run altitude/tilt along with actual initial/final pose, bar pose and clearance; do not invent an acceptance threshold after observing results.

4. **LOW — tensor relocation does not refit Warp.** `IGE_env.write_to_sim()` (`IGE_env_manager.py:620-627`) updates PhysX root state, not Warp meshes. `warp_env_manager.py:40-53` refits those meshes only during reset. If sensors are ever inspected in the new attempt, invoke the mesh reset/refit after relocation. No policy/sensor read enters G9's current control path, so stale Warp placement does not itself invalidate the velocity response.

## Confirmed source behavior

- Robot position/orientation/linear/angular velocity are views of the root state; the patch writes through them and then `IGE_env.write_to_sim()` submits the actor state. This is the correct basic relocation order (`IGE_env_manager.py:441-453,620-627`).
- The controller refreshes derived body/vehicle state at the start of every physics step (`base_multirotor.py:287-305`), so manually setting identity quaternion before the initial hover does not leave a stale yaw in the control calculation.
- With a 40 m arena and intended yaw-zero start, the nominal command sequence stays comfortably away from a bar at (38,2). The clear path still needs measured confirmation; initial random placement was not sufficient.
- `em.simulate()` does not auto-reset collision cases; `compute_observations()` only accumulates collision counts (`env_manager.py:381-407`). Therefore a tipped/crashed env may continue to produce numerical response values. All four envs need inspection.
- The prepared replacement script checks all three source substrings before its single final write. It will refuse a repeat application due missing replacement anchors; it does not partially write a source file if a later anchor assertion fails.

## Run ordering and evidence

Keep the active GR-1b run and its worktrees frozen. Prepare the corrected diff externally; after GR-1b finishes, validate both arm reports and faults, analyze/review GR-1b as planned, apply and locally commit the G9 source/dated correction, then run the new attempt with an exclusive output folder. Preserve all older attempts. The runner currently uses `OUT.exists()` followed much later by `write_text`, which is not an atomic exclusion; launch only one process against a freshly reserved output path, or use exclusive creation. Creating the output parent is the wrapper's responsibility.
