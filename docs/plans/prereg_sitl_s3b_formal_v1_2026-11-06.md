# S3b formal v1 — Policy F through PX4 SITL on the recorded G8 r2 scenes (preregistration, 2026-11-06)

**State:** preregistered, **NOT_RUN**. Fixed before any measurement of this unit. Runner `tools/sitl_s3b_formal.py`,
tests `tests/test_sitl_s3b_formal.py`. Plan stage S3b of [`sitl_validation_plan_v1.md`](sitl_validation_plan_v1.md).
Amendments go in a new dated file.

## Question

Does Frozen Policy F, unchanged, still avoid the bars when it flies a PX4 v1.14.3 vehicle in Gazebo Classic SITL
(PX4's estimator, controllers and offboard velocity interface), compared with Isaac Gym on the same scenes?

The plan's acceptance rule, fixed here: **"feasible in SITL" if SITL crash ≤ Isaac crash + 5 pp at 70 bars.**

## Entry into S3b, and what is disclosed

- S3a reads **NOT_EQUIVALENT**:
  - The policy part of its residual is GPU TF32 (S3a-P1, `TF32_EXPLAINS_POLICY_RESIDUAL`).
  - The target-velocity slice residual (up to 1.2e-2 at 205 bars) is not explained; a raster-boundary hypothesis is
    untested.
  - Entering S3b anyway is the user's decision. Approving this unit's execution records that decision.
- The development run dev07 (same runner loop, n = 20) had 20/20 outcomes equal to Isaac. It was not preregistered.
- AGQ geometry-v1 attempt01 showed outcomes at 205 bars can flip under a 2.5–3.1 cm start difference. This unit
  therefore compares **rates over many scenes**, not single paired episodes, and measures a small repeat block.

## Fixed treatment

The development runner's loop (`tools/sitl_s3b_closed_loop.py`, dev07), unchanged in what it computes:
- **Policy and bridge:**
  - Frozen Policy F `f7022139…`, deterministic, on the CPU through `tools/sitl_bridge.py`;
  - MOTAR's own sensors rendered from the simulator's true pose;
  - perception on PX4's estimated pose;
  - the riskcap governor.
- **Control:**
  - 10 Hz in PX4 simulation time;
  - governed vehicle-frame velocity and yaw rate sent as an offboard setpoint, with altitude held by PX4 at 1.0 m;
  - command yaw `truth`.
- **Vehicle and world:**
  - navrl_quad (airframe 10090, downward rangefinder as the height reference);
  - the 10 PX4 parameters dev07 set: `MC_ROLL/PITCHRATE_P 0.013, _I 0.017, _D 0.00026`, `MPC_XY_VEL_P_ACC 2.5`,
    `_I_ACC 0.1`, `_D_ACC 0`, `THR_MDL_FAC 1`. Every other parameter is airframe 10090's default from a fresh
    rootfs (for example `MC_YAWRATE_P 0.11`, `_I 0.055`). dev07 booted the shared rootfs, whose stored parameters
    it imported; its effective values for the parameters it did not set are NOT_RECORDED. This unit is therefore
    not a byte-for-byte repeat of dev07's configuration;
  - ODE quickstep iterations 100.
- **Termination:** MOTAR's rules on the true pose.
  - Capture: the swept 3-D distance falls below 0.5 m.
  - Crash: the 0.14 m footprint meets a bar box, or z < 0.1 m or z > 4 m, or the vehicle is more than 1 m outside
    the arena.
  - Timeout: 600 steps.

Added for a formal run, none of which changes a computed command:
- an owned PX4 + Gazebo session per episode with a **fresh rootfs** (`tools/sitl_owned_session.py`), so no parameter
  persists between episodes or from earlier runs;
- a **read-back** of every parameter set;
- each control step's **lateness** against its schedule;
- input pins, an approval bound to the commit, and kernel-fault checks.

The command-delay hook stays at 0 steps. A unit test asserts that 0 passes each command through unchanged, and that
the generated world is byte-identical to the development runner's.

## Inputs and schedule (fixed now)

- **Scenes:** G8 r2, seed 4501, condition `full`, first episode of each env.
  - The 24 files are pinned by SHA-256 in the runner (`INPUT_SHA256`).
  - The source is `results/sitl_ref_v1_r2/g8/` in the main checkout.
- **Target:** the trajectory recorded in the matching zero-action cell (60 s), replayed; it does not react to the
  SITL pursuer.
  - A scene is **eligible** when its zero-action first episode ran all 600 actions. This is decided from the
    zero-action ledger alone.
  - 122 of 128 scenes are eligible at 70 bars and 124 of 128 at 205 bars. All 162 scheduled scenes have 600 target
    frames.
- **Blocks,** run in this order with envs ascending:

| Block | Density | Scenes | Isaac on these scenes | Role |
| --- | --- | --- | --- | --- |
| A | 70 bars | all 122 eligible | 121 capture, 1 crash | **primary reading** |
| B | 205 bars | first 30 eligible | reported | secondary, reported |
| C | 70 bars | first 10 eligible, a second boot each | — | repeatability, reported |

## Validity and stop rules

- **An episode is VALID** when all of these hold:
  - it ends in capture, crash or timeout;
  - every parameter read back equals the value set, within 1e-6, relative above magnitude 1 and absolute below it.
    INT32 parameters are decoded from PX4's byte-cast float;
  - its largest control-step lateness is at most **0.15 s**. That is the bound the user fixed for dev08
    (D-2026-11-05-2), adopted here for this unit and named in its approval scope.
    - Lateness is PX4's simulation time at the step's state sample minus the step's scheduled time.
    - The bridge's compute time and the truth sample's wall-clock age at each step (`truth_age_max_s`) are recorded
      per episode, but are not part of this bound.

  Otherwise it is INVALID, with the reason recorded (HARNESS_INVALID, ERROR, PARAM_MISMATCH or LATE). INVALID
  episodes are excluded from both the SITL and the Isaac rate of that block.
- **The unit stops (ABORTED)** in any of these cases:
  - the kernel fault count rises;
  - the first episode is INVALID (the harness does not work);
  - more than 12 block-A episodes are INVALID;
  - a scheduled scene lacks a full-horizon target.

  Any exception or termination request in the unit loop is recorded as ABORTED with its reason in `final.json`
  before it propagates. The outer `timeout -k 30 14400` is the last resort.
- **A stalled episode ends:** PX4's simulation time must reach each step within 2 s of wall time, and the owned
  session's process and watchdog check runs every 0.1 s while waiting. Otherwise the episode is ERROR.
- **The stack under test is bound per episode.** The runtime binding is recorded per episode: the PX4 binary,
  installed and source rcS and airframe, `parameters.json`, the model SDF and its resources, every Gazebo plugin,
  and the simulator environment. An episode whose binding (world excluded) differs from the first VALID episode's is
  INVALID `BINDING_CHANGED`.
- **Host idleness is checked per episode.** The runner's cores are checked before every episode. If they are still
  busy after one 30 s wait, the episode is recorded INVALID `HOST_BUSY` without running. That decision comes before
  any outcome exists.
- **Tree changes stop the unit.** A change of HEAD or of tracked files after an episode stops the unit (ABORTED).
- **Repeated failures stop the unit.** Three consecutive INVALID episodes stop the unit. In block A this is ABORTED.
  In block B or C it is `STOPPED_AFTER_A`, which keeps the block-A reading when all 122 block-A rows exist.
- **No retries.** No episode is rerun, replaced or dropped after its outcome is known.

## Reading (fixed analyzer: `tools/sitl_s3b_formal.py --analyze`)

- **VOID** in any of these cases:
  - the attempt is neither COMPLETE nor `STOPPED_AFTER_A` with block A complete, or `final.json` is missing;
  - the ledger is not the schedule's prefix in order, or a COMPLETE attempt has fewer rows than scheduled;
  - the kernel fault count changed, or the tree's HEAD or tracked files changed during the run;
  - fewer than 110 block-A episodes are VALID.
- Otherwise, for block A, with Δcrash = SITL crash % − Isaac crash % on the same VALID scenes:
  - **FEASIBLE_IN_SITL** if Δcrash ≤ +5.0 pp;
  - **NOT_FEASIBLE_IN_SITL** otherwise.
- **Reported without a gate:**
  - for block A, the Clopper-Pearson 95 % interval of the SITL crash rate, and a worst case that counts every
    INVALID episode as a SITL crash (INVALID exclusion is not strictly outcome-independent, because a longer
    episode has more chances to be LATE);
  - per block: capture and timeout rates, Δcapture, and per-scene outcome agreement with Isaac;
  - block B in the same form;
  - block C: the number of scenes whose second boot repeats the block-A outcome;
  - timing: lateness maximum and p99, compute time, real-time factor;
  - EKF-versus-truth error and parameter read-backs.

## What follows, and what does not

- **FEASIBLE_IN_SITL** means F, unchanged, flies this PX4 stack at 70 bars within the plan's crash margin under the
  disclosed approximations. It is the S3b prerequisite the plan names for S5.
- **It does not follow:**
  - real-flight readiness;
  - S3a equivalence;
  - Gazebo-sensor realism (S4);
  - heavier-vehicle or latency robustness (S3c);
  - a claim at 205 bars;
  - target reactivity;
  - a gain-matched controller (G9 is not matched).

## Known limits (disclosed in the result)

- **Target:** the replayed target does not react to the pursuer. The zero-action target equals Isaac's own target only
  on common frames before the coupled global RNG diverges (G8 r2 `TARGET_DEPENDS_ON_PURSUER`). So the comparison is
  of rates on the same scenes, not of paired counterfactuals.
- **Sensors:** MOTAR's own sensors, from the true pose; not Gazebo's.
- **Scope:** one seed, first episodes only; the rangefinder is an addition to the training-spec vehicle.
- **Timing differences from dev07:**
  - The parameter read-back runs on the ground, disarmed, before the origin is captured. It adds 0.7–8.7 s there.
  - dev07's reported `compute_ms_*` fields read the true-y column, not compute time (review r1). This unit's compute
    time is column 21.
- **External code:** the LiDAR replay module comes from `MOTAR-cplanner`. The launch refuses if it has local
  changes, and `attempt.json` binds it by SHA-256 and that tree's HEAD.
- **Contact:** the plan's Gazebo-contact cross-check of crashes is not implemented. A crash is MOTAR's geometric
  rule only.
- **Scenes:** the plan names B1-R or GR-0 layouts; this unit uses the G8 r2 scenes, because their zero-action cells
  give full 60 s target trajectories. dev07 already flew 9 of the block-A scenes and 10 of the block-B scenes
  (envs 0–9, minus the excluded ones). Their dev07 outcomes are not part of this unit.

## Execution envelope

- **Process:** CPU only (`CUDA_VISIBLE_DEVICES=""`); cores 6,7,12,13; `nice -n 10`; `OMP_NUM_THREADS=1`,
  `MKL_NUM_THREADS=1`; `PYTHONNOUSERSITE=1`; `python -B`; `timeout -k 30 14400`.
- **Termination:** a SIGTERM outside an owned session is caught and recorded as ABORTED in `final.json`. Inside a
  session, the session's own handler records it.
- **Host and tree:** an idle host, since this is a real-time SITL acceptance run. The tree is frozen during the run.
- **Single use:** one attempt, `~/motar_tmp/s3b_formal_20261106/attempt01`. The runner refuses an existing attempt
  directory.
- **Launch refusals:** before anything is created, the runner refuses unless:
  - every source it runs, and every production module the bridge loads (the dev08 source list), is tracked and
    equal to HEAD;
  - Policy F's checkpoint has SHA-256 `f7022139…5ad40`;
  - the external replay module is clean, and the 24 inputs match their pins;
  - the process has CUDA hidden, one thread, nice ≥ 10 and affinity within cores 6,7,12,13;
  - no inherited `NAVRL_*`, `PX4_*`, `S3B_*` or `GAZEBO_*` variable is set, and the command yaw is `truth`. The
    environment's keys are recorded;
  - those cores are idle;
  - the kernel fault baseline is readable.
- **Memory:** the owned session's watchdog caps the session's resident memory at 12 GiB.
- **Approval:** `~/motar_tmp/s3b_formal_20261106/execution_approval.json`, with `approved_by`, `approved_at`, the
  runner's exact `SCOPE` and the commit.
- **Duration:** expected about 2.3 h (dev07: about 45 s per episode).
