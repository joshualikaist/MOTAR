# S4R-1 v1, amendment 2: fresh attempt02 at the same latency bound (2026-11-10; NOT_RUN)

- **Amends:** `docs/plans/prereg_sitl_s4r1_v1_2026-11-10.md` and amendment 1
  `docs/plans/prereg_sitl_s4r1_v1_amendment1_2026-11-10.md`. Both remain immutable and in force except below.
- **Status:** NOT_RUN under this amendment. Preparation follows the user's 2026-11-10 continuation request. An
  independent review, CPU checks, local commit and new scoped approval at the final HEAD precede all live execution.

## Reason and preservation of the incomplete attempt

At `023998f`, smoke02_contact_on_env0 passed, then attempt01 stopped at index 7 (boot 8/12), env1 repeat1 contact ON,
for LATE 0.168 s > 0.15 s. Its wall time was 107.7 s, compared with 48.7–63.5 s for the other seven recorded boots.
A host scheduling stall is a hypothesis, not a measured cause; no kernel fault was recorded. The original eight
records and the `INCOMPLETE_INVALID` reading remain unchanged at `~/motar_tmp/s4r1_20261110/attempt01/` and in
`docs/evidence/sitl_2026-11-05/s4r1/attempt01_*`. They do not support per-scene readings or repeatability conclusions.

## Fresh single-use folders and unchanged rules

- **New smoke:** `~/motar_tmp/s4r1_20261110/smoke03_contact_on_env0/`, env0 contact ON, same fixed short smoke.
  It must be COMPLETE and valid, with HEAD/source bindings at the new committed version, before attempt02.
- **New attempt:** `~/motar_tmp/s4r1_20261110/attempt02/`, all twelve boots from index 0 in the same fixed order.
  Never resume attempt01 or append to its ledger. Old smoke folders and approvals are not reusable.
- **Latency:** keep the existing **0.15 s** operational validity bound. A LATE member invalidates the attempt and
  stops at that first invalid member, with no replacement boot, trimming, retry, or threshold relaxation.
- **Host:** keep the idle-host requirement. Close Cursor before the new run and start no other compute jobs during
  the smoke/attempt; record host snapshots and kernel fault counts as before. Closing an editor does not establish
  the cause of the previous delay and does not guarantee that a fresh attempt will meet the latency bound.
- Keep the 4 GiB launch floor, 1 GiB minimum free space, 3 GiB maximum output, 8 GiB owned RSS cap, no RLIMIT_AS,
  180 s per boot, 600 s outer smoke/1800 s outer attempt and 60 s kill grace. No raw-result deletion is authorized.
  Cursor removal is a separate user request; record actual free space before the launch preflight.
- Both dated amendments are in `OWN_SOURCES`. The new SCOPE requires a new final-HEAD approval; old approvals fail.

## SCOPE and execution

The new approval must quote exactly:

```
S4R-1 v1 amendment2: one fixed instrumentation smoke smoke03_contact_on_env0 (env0, contact subscriber ON, not a result, 600s outer, a short boot) then attempt02: 12 fresh Gazebo/PX4 boots, G8 r2 first two eligible 70-bar scenes env0/env1, 3 repeats per scene with world-contact subscriber OFF/ON, fixed alternating order; S4F1 lit_rendered ground arm, Frozen Policy F, navrl_quad_s4, body-fixed 87deg RGB/depth/gpu_ray, pinned historical bridge, explicit new Gazebo seed4501; MAVLink recv_match arrivals and sends recorded (wait_heartbeat NOT_RECORDED), all delivered raw frames and actor observations recorded; contact log: no missing, duplicate or off-grid simulator step in the reading window, arrivals at most 2 steps late and at most 1% late; no retries, stop on first operational/recording invalid member, 180s per boot, 1800s outer; owned-session RSS cap 8GiB, no RLIMIT_AS; DISPLAY :1 with a readable inherited XAUTHORITY; 3GiB maximum output, 4GiB launch free and 1GiB minimum free disk; descriptive only: first recorded divergence per stage and within/between-condition spread, no gate
```

The launch commands are identical to the original preregistration (§8); only source-bound output-folder and scope
constants change. Scenes, seed, policy/checkpoint, sensor and vehicle models, bridge, contact-continuity tolerance,
positive control, data contract, analysis, reading labels and reporting tolerances remain unchanged.
This amendment proposes one fresh bounded attempt, not a retry-until-valid procedure. No subsequent attempt is
implicitly authorized if this one is incomplete. Historical attempt01 is not reanalyzed under the new source contract.
