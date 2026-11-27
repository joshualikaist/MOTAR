# AGQ follow-up preparation — 2026-11-06

State: CPU diagnosis and additive preparation. New live/GPU measurements below are
**PROPOSED, NOT_RUN**; formal S3b/S3c/S4/S5 are not authorized or qualified by this
document. Existing dev08 outcomes, input tapes, launchers, Policy F and preregistrations
remain immutable. This is a separate development lineage, not a dev08 rerun amendment.

The user's current instruction authorizes diagnosing the four existing records,
repairing established environment defects and preparing formal-validation prerequisites.
The completed D-2026-11-05-3 measurement authorization stays consumed. The inherited
session still wraps at 2026-11-05 17:45:22 UTC and finishes by 18:35:22 UTC.

Read the [diagnosis and preparation record](../evidence/sitl_2026-11-05/dev08/agq/README.md)
first. Its JSON binds the existing four PX4 ULogs, commands, parameters, worlds and
independent audits. Geometric footprint intersection is an explicitly labelled proxy;
the existing logs do not record physical contacts or exact packet send/receive times.

## Correction prepared now

`tools/sitl_training_geometry.py` defaults to `native`, which exports the exact
recorded SDF bytes. Opt-in `training-box` deletes only the four rotor collision
elements. The remaining collision is the training URDF's base box. No mass, inertia,
motor/thrust model, sensor/plugin, actuator, gain, policy, observation or termination
setting is changed. Existing installed PX4 assets are read-only. The generated model
package lives under a fresh `~/motar_tmp` directory, with source/output hashes.

The pure `launch_plan()` adapter selects the package by the spawn-file argument and
keeps PX4's original airframe and original resource search paths. The original owned
session/runner remains intact. CPU preparation proves the model/runtime-file delta;
it does not establish the corrected model's live contact or control response.

## Minimum geometry intervention

The smallest test is **two fresh native-gain 205/env5 episodes**, one with each
collision model. A second pair on 205/env2 with candidate gains checks the scene
whose large position discrepancy followed earlier disturbances. A proposed single
unit contains exactly these four members, in this fixed order:

| Member | Scene | PX4 gain map | Collision model |
| --- | --- | --- | --- |
| 0 | 205/env5 | native dev08 | native |
| 1 | 205/env5 | native dev08 | training-box |
| 2 | 205/env2 | candidate dev08 | training-box |
| 3 | 205/env2 | candidate dev08 | native |

Keep each pair's Policy F, target prefix, start/layout, 10 Hz loop, ODE iterations,
rangefinder, stream/timing guards and parameters fixed. Change only the four rotor
collision elements. Do not tune gains or EKF during this unit. Each member uses a
fresh rootfs and effective parameter readback. Boot/noise repeatability is not assumed;
one pair is a mechanism diagnostic, not a treatment-effect estimate.

Prediction before new measurement: removing the extra collision shapes eliminates
their contact opportunity. If native env5's yaw impulse and altitude collapse depend
on these contacts, their onset should disappear or move with the contacts. This does
not predict env2 will succeed: its old candidate trace also has a later base-box
intersection proxy, and command tracking overshoot exists before some intersections.

Required telemetry in **both** variants: existing PX4 ULogs; complete selected
pose/attitude/truth snapshots and generated commands; decoded MAVLink arrivals and
send-call entry/return times on one monotonic clock; Gazebo contacts with collision
names, simulator timestamps and impulses if available. An owned contact subscriber
must demonstrate a known floor/body contact before takeoff; silence alone cannot
count as no contact. These recording additions need implementation/CPU review before
launch. They must not feed truth/contact data into Policy F or its controller.

Report every attempted member, including invalid/negative cases. Compare first
named bar contact, angular-rate departure, true altitude loss, estimate/truth
discrepancy and actual-versus-requested velocity, with publication-phase limits.
If a native member does not reproduce the old contact/disturbance pattern, report
mechanism **INCONCLUSIVE**; do not replace or repeat it automatically. A repeated
disturbance with no preceding contact keeps the controller/plant hypothesis open.
No capture gain ranking or formal acceptance follows from four development episodes.

Before execution: commit/review the final launcher, recording changes and fixed
analysis rules; bind a new scoped user approval to that exact clean commit. Keep the
host idle, hide CUDA, use cores6/7/12/13 with nice10 and one compute thread; retain
the 240s episode watchdog, sampled12GiB RSS guard and 16GiB address-space limit.
Use a 1200s outer timeout with kill20s and explicit unit budget. An increased kernel
fault count, source drift, invalid parameters/streams, recorder loss, contact
positive-control failure, cleanup leak or resource guard failure stops the unit.
No outcome-dependent retry. Freeze all tracked files while it runs.

## S3a numerical control, then full-horizon input collection

The existing S3a reports remain **NOT_EQUIVALENT**. CPU singleton/batch controls
do not explain the stored GPU action discrepancy. A target raster boundary can
change a pixel count and propagate into KF target velocity; original GPU pixel
measurements were not recorded. A partial float64 discriminant probe fixes one
boundary case, not both. No production detector/KF change is justified yet.

The smallest new GPU control reuses the **stored states/observations**; it need not
run the whole G8 simulator campaign. Freeze these two separate comparisons first:

1. Policy: identical stored observations and Frozen F, CPU versus GPU; change only
   TF32 ON/OFF on GPU. Set and read back backend flags **after** player construction,
   since the player enables TF32. Record per-action errors, weights/schema and
   backend settings. This tests a precision hypothesis without replacing F.
2. Sensing: both densities' first ten recorded calls for the original first eight
   envs, with exact recorded pose/quaternions, target, layout and perception history.
   Record CPU/GPU raw RGB-D, target/occlusion masks, pixel counts, depth reductions,
   measurements and KF priors/posteriors. Include the original one-degree-yaw
   negative control and an unchanged-input positive control. Compare the existing
   fixed observation/action tolerances; do not widen them after measurement.

Any stable raster or inference correction must then be separately implemented and
qualified on a new source-bound reference, with independent review. TF32-off
reference data would be a new numerical lineage, never a correction of old bytes.

`tools/sitl_formal_readiness.py` now requires finite, contiguous **states0..600**
for every fixed scene (600 actions plus the final check). It checks trace schema,
context/receipt identities and hash bindings. It never pads, clamps, drops env5,
substitutes another episode, or authorizes a formal gate. Its synthetic complete
control can pass structural checks while still requiring source-bound controls,
layout/sensor qualification and independent review.

The old raw targets cannot be extended retrospectively: hidden target/RNG/reset
state is unavailable and global-RNG coupling was observed in G8. Full-length input
collection therefore needs a new explicit input lineage, committed collector plan
and GPU approval. The collector must record state0 before action1 through state600
after action600 even when a target approaches the hovering pursuer; terminal/reset
handling, RNG isolation and matched Isaac/SITL replay comparator must be fixed before
launch. Complete native reset/continuation semantics must be reviewed rather than
silently disabling termination or relabelling this as the old episode.

Only after new inputs and S3a satisfy their reviewed contracts can a separate formal
S3b protocol/approval begin. Sensor realism and actual flight readiness remain later
stages. A structural readiness result is never formal GO.
