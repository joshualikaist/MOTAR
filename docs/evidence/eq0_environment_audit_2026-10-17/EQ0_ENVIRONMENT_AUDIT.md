# EQ-0 — MOTAR environment qualification audit

Started 2026-10-17. **READ-ONLY source/record audit with independent CPU algebra and recorded-state
forensics. No simulator rollout, GPU evaluation, EQ-1 experiment, environment change or checkpoint
loading.** The four qualification domains are **LIMITED_VALIDITY overall**. Specific mathematical
subgates FAIL; physical/hardware properties without evidence remain NOT_TESTED. This is not a
certificate that the environment is physically validated.

## 1. Authority and exact implemented structure

- Actual OCC-P2 measurement source: `5ff285ad053f181c5a1c86dddb2462bb44e6327b`, tree
  `fe665151892de7e6d0260dcbb71d0a75432ebc1f`, as recorded in the linked launch receipt.
  Source SHA, source blobs and recorded instantiated conditions are separately checked in the
  supporting evidence.
- Audit base: `7cea3c52d8c8d7ab6dbeef5fae6e929acfe279a2`, branch `codex/mt-p1`. The two preceding
  commits `ceedc52` and `7cea3c5` are retained. Their changes from the measurement source are
  research records/instructions, not simulator implementations.
- Policy F SHA-256: `f702213936601860995cf61dcc570247e72543b1976e3716055cd8ec5593ad40`.
  The checkpoint was hashed, never deserialized in this audit.
- Run `OCC-P2`, preregistration v1: [MD](../../plans/prereg_occlusion_confirmatory_v1.md),
  [JSON](../../plans/prereg_occlusion_confirmatory_v1.json). See the byte-identical preserved
  [launch receipt](../occ_p2_confirmatory_2026-10-17/launch_receipt.json),
  [run report](../occ_p2_confirmatory_2026-10-17/run_report.json),
  [manifest](../occ_p2_confirmatory_2026-10-17/RAW_MANIFEST.json) and
  [original analysis](../occ_p2_confirmatory_2026-10-17/analysis.json).
- Raw root is
  `/home/fair/workspaces/aerial_gym_ws/src/aerial_gym_simulator/results/occlusion_confirmatory_v1/OCC-P2`.
  SHA bindings and exhaustive observed setting values: [run inventory](supporting/g4/run_inventory.json).

| Stage | Actual implementation and authority | Qualification boundary |
| --- | --- | --- |
| Environment | 40 × 40 m arena; `bars_h3` upright boxes spanning z=0..3 m; `navrl_band` placement; 70/115/160/205 bars | Synthetic compound-bar scene distribution, not a forest reconstruction or certified connected free space |
| Pursuer | `NavRLQuadCfg` / `quad_navrl_collide.urdf`, nominal mass 0.25 kg, one 0.28 × 0.28 × 0.08 m collision box; force/torque-driven GPU PhysX | Real 3D rigid-body state; no independent motor/airframe/hardware identification |
| Control | Frozen 4-output actor; XY body/yaw-frame velocity request, yaw rate; actor Z overwritten by altitude PI toward 1 m; Lee velocity/attitude controller and motor allocator | Policy cannot directly choose climb/descent; altitude is an objective, not a locked state |
| Limits | XY request ±2.5 m/s **per axis**, diagonal norm up to 3.5355 m/s; yaw request ±3 rad/s; desired tilt 45° | Request limits do not bound actual speed/tilt; no observed-state safety guarantee |
| Timing | `BaseSimConfig`: physics dt=0.01 s, substeps=1; 10 physics/controller iterations per RL action; action/altitude loop and normally consumed sensors at 0.1 s | 100 Hz physics/Lee versus 10 Hz policy/observation; reset refreshes differ; true capture timestamps absent |
| Target | E0 static or E2 `bounded_planar_drone_v1_rollout_direct_l2_v1`; virtual target at z=1 m; E2 nominal sampled speed 0.3–1.5 m/s, actual motion may brake to zero; acceleration ≤4 m/s² and heading slew ≤150°/s | Kinematic executor writes pose; no PhysX target UAV, attitude, rotor or physical target contact |
| Camera | Source-resolved synthetic target RGB-D at 160×90, HFOV 87° / VFOV 58°, translation [0.10,0,0.03] m; analytic flat-color radius-0.15 m sphere; yaw-only camera; obstacle depth 40×24/10 m upsampled | Ideal stabilization, metric simulated depth and appearance; not an RGB-only hardware pipeline |
| LiDAR | 72×4 ideal rays, azimuth spacing 5°, elevations 20/10/0/−10°, range 0.2–12 m; body-centre/yaw-only; analytic target radius 0.20 m | Sparse, noiseless model; target association and range calibration unqualified |
| Perception | Fixed RGB bootstrap score and ≥2 pixels, camera depth-derived 3D estimate; LiDAR association when camera absent; tracker/past-state tokens | Redetection almost exactly follows rendered pixel availability; fused validity is not identity proof |
| Actor/critic | 898-D actor = 288 scan + 480 obstacle history + 50 ego history + 80 target history; critic adds 8 privileged values, 906-D; 17 tokens | Source separates privileged critic/evaluator fields; dynamic intervention/normalizer invariance NOT_TESTED |
| Execution | Riskcap preserves XY direction and floors its cap at 2 m/s; altitude PI and Lee controller then execute the request | No certified stopping law, abort state or dynamic collision-free tube |
| Outcomes | Swept relative **straight segment** over 0.1 s, capture radius <0.5 m; capture wins same-step crash; height/OOB are task terminations | Capture is close approach, not physical docking or collision-free arrival |

Primary source anchors and function-level detail are in the independent
[G1 review](supporting/g1/findings.md), [G2 review](supporting/g2/audit.md) and
[G3 review](supporting/g3/REPORT.md). The actual launcher is
[`eval_navrl_target_behavior_arms.sh`](../../../aerial_gym/rl_training/rl_games/eval_navrl_target_behavior_arms.sh),
called by [`occlusion_confirmatory.py`](../../../tools/occlusion_confirmatory.py).
Do not substitute the different density-sweep launcher's defaults for this run.

## 2. What was verified, and what was not

| Domain | Overall | Evidence already available | Not established |
| --- | --- | --- | --- |
| EQ-G1 geometry/physics/dynamics | LIMITED_VALIDITY | Source/conditions and independent 10 Hz pose/kinematic checks; bounded-target motion gates | Physical identification, timestep convergence, contact fidelity, globally reachable scenes; active motor routine fails classical RK4 algebra |
| EQ-G2 rendering/sensors/occlusion | LIMITED_VALIDITY | Recorded source/poses/LOS, G0 mesh parity and localized boundary audit | Calibrated fusion FAIL; full silhouette, sensor hardware and capture-time fidelity NOT_TESTED |
| EQ-G3 perception/tracker/control | LIMITED_VALIDITY | Flag algebra, widths, expiry definition, source dataflow separation | Real redetection, identity-correct LiDAR, covariance calibration, dynamic GT firewall and stopping guarantee |
| EQ-G4 experimental validity/reproducibility/safety | LIMITED_VALIDITY | Precommitted protocol, source/manifest/ledger integrity, seed-block analysis | Complete runtime/state replay, all-case measurement noninterference, safe abort and collision-free capture |

The [qualification matrix](QUALIFICATION_MATRIX.md) defines each PASS narrowly. Mathematical FAIL
does not mean a historical result folder is rewritten or a new environment was installed.

Independent record scans cover **112 cells, 57,344 quota episodes and 2,320,985 consumed frames**.
There are 29,042 retained nonprimary tail episodes. No direct production detector, geometry,
tracker, motor, controller or statistical metric function is called as an oracle by those scans.

- Actual UAV altitude varies: Static 0.1078..2.1944 m, Moving 0.1014..2.7928 m. Maximum observed
  tilt is 99.72°/110.97° and XY speed 4.7853/4.6723 m/s. These may include collision transients;
  their cause and 100 Hz extrema are NOT_RECORDED. They refute hard actual-state cap wording,
  not the declared **command** caps. [Independent state evidence](supporting/g1/record_forensics.json).
- `camera_valid == (rendered_target_pixels >= 2)` on every quota frame; fused validity equals
  camera OR accepted LiDAR on every frame. LiDAR-only valid frames total 187,114. Neither proves
  correct physical identity. [Independent flag evidence](supporting/g3/independent_record_algebra.json).
- Nonzero configured dropout 0.3 / RGB noise 0.015 / depth noise 0.02 are **inactive** on the clean
  source path: `NAVRL_PERCEPTION_PERTURB` is absent, its default is false, and perturbations are
  gated by that switch. Do not describe this run as a 30% dropout evaluation.
- Primary in-view centre-LOS intervals have no supported 5 s tail. Other loss modes did exercise
  tracker expiry: 41 Moving episodes have ≥51 consecutive fused misses and 41 track deactivations;
  Static has none. Expiry is strict age >5 s since last accepted camera **or LiDAR** correction,
  typically the 51st 0.1 s miss; old history tokens can remain after current track deactivation.
- The calibrated camera→LiDAR mapping fails an independent pinhole/common-origin fixture:
  at a 20° LiDAR direction, the current linear map selects a ray at about 23.4109°, a 5.8164-pixel
  error; camera offset alone produces a −0.10 m front-plane range error. These are fixture
  counterexamples, not measured errors on every OCC association.
- Motor `rk4_integration` uses the wrong intermediate error signs for a decaying first-order
  tracking ODE. At actual dt=0.01, tau=0.04, its composite gain is 0.2214, close to exact nominal
  lag gain 0.2211992 because the coefficient is also modified. This limits immediate severity;
  it does not make the routine classical RK4 or qualify another timestep. No per-motor time series
  is recorded in the audited OCC artifacts.

## 3. Experimental validity and safety

OCC-P1 stays exploratory. OCC-P2 stays the independent confirmation **relative to OCC-P1** of the
registered clean frozen-loop endpoint. RQ1/RQ3 remain SUPPORTED; RQ2 remains
INCONCLUSIVE_EQUIVALENCE_NOT_TESTED. This audit does not change the preregistration or p values.

The primary endpoint counts **centre-LOS-blocked intervals while the target centre is in view**,
including blocked FOV re-entry, per total observed post-acquisition time. It is not full silhouette
loss, a clear-time hazard or every physical occlusion crossing. Actual clear→blocked counts are
supplemental. FOV exit and terminal events can censor durations informatively. The 100% eventual
fused resumption fraction conditions on 14,257 geometry-restored intervals, excluding 3,654 FOV
exits and 574 crash endings. It is not unconditional recovery or a learned Recovery algorithm result.

Primary outcomes from quota ledgers are **55,441 capture / 1,898 crash / 5 timeout**.
Raw `result.json` uses the earlier legacy aggregate snapshot: **55,478 / 1,997 / 0** over 57,475
batch-window completions. Outcome distributions differ in 96 cells. Its `crash_causes` and governor
diagnostics must not be reported as quota-conditioned metrics. This is a population boundary, not
a newly discovered defect in the OCC-P2 analysis, which explicitly joins quota episodes.

Seed numbers are disjoint from P1; 4301–4307 were previously used in B, disclosed before P2.
Stored scene hashes show 93,092 unique P2 geometries and no cross-seed repeats; P1 intersection
was zero. Rounded hashes do not prove statistical independence. All groups/densities remain in
their seed cluster in the fixed six-test Holm analysis.

G0 measured noninterference PASS on **eight paired conditions at seed 4201** with sampled actor
observations, episode/recorder comparisons and trajectory digests. The same runtime source is
retained. OCC-P2 per-cell metadata still says NOT_TESTED for noninterference; do not call this
an exhaustive 112-cell on/off replay. See the preserved
[G0 report](../occ_p2_pre_execution_2026-10-17/OCC-G0_rerun1/g0_report.json).

Runtime evidence is partial: logs identify PyTorch 2.4.1+cu121, Warp 1.0.0 and GPU PhysX; Moving
records TF32 enabled and float32 precision `high`. Static lacks the matching matmul record.
No complete historical runtime fingerprint, sensor capture timestamps, normalization-state digest,
per-motor force history, 100 Hz contact chronology or full simulator/RNG replay snapshot exists.
Current installed dependency code is not proof of its historical bytes.

## 4. Issue priority and required action before future claims

The [issue register](ISSUE_REGISTER.md) gives exact files/functions, counterexamples, impact,
independent reproductions, acceptance criteria and proposed remedies. There is **no blocker to
preserving or reporting the bounded OCC result, or to this read-only audit**. There are blockers
to treating it as qualification for realistic long-duration Recovery, calibrated multisensor
mapping, independent vertical planning or a physical target UAV.

Before such a future hypothesis: fix/requalify fusion under a new sensor lineage; choose and
qualify the intended sensing contract; establish actual measurement-loss exposure,
identity and track/history availability; certify admitted scene feasibility and safety semantics.
The user's future online Recovery supervisor must use inputs available in its declared real sensor
contract, without unavailable synthetic depth or GT. Current Policy F uses LiDAR and synthetic
RGB-D; the long-term camera-only candidate is a different, unresolved contract. An explicitly ideal
RGB-D baseline analysis may continue, but does not establish camera-only or hardware compatibility.
Resolve the motor integrator before timestep/model-transfer claims. If 3D manoeuvres or physical
target flight are required, that requires a separately approved interface/plant lineage.
No frozen Policy F, tracker 5 s contract or actor width is changed by this recommendation.

The user supplied **PROVISIONAL — NOT PREREGISTERED** definitions during EQ-0:

| Candidate | Environment qualification consequence, not a hypothesis test |
| --- | --- |
| H-R1 Target Tracking Continuity | Distinguish geometric obstruction, FOV/scale loss, camera/fused loss, valid identity and expiry; qualify latency, prediction/covariance and remaining history. Conditional redetection is not end-to-end recovery. |
| H-R2 Safe Target Approach | Qualify pursuer/target dynamics, collision support and scene passage; measure collision-free capture, crash, timeout, abort and approach time. Stopping or avoidance alone does not establish improvement. |
| H-R3 Contribution of Active Visibility Control | Prediction and all sensor information must be equal; certify the same pre-intervention state/prefix and sensor-only visibility scoring against prediction-only/yaw-only baselines. No GT viewpoint advantage. |

Exact provisional statements, modality constraints and the issue-by-hypothesis mapping are in
[PROVISIONAL_HYPOTHESES.md](PROVISIONAL_HYPOTHESES.md) and the [issue register](ISSUE_REGISTER.md).
None of the three is preregistered or tested. Earlier supporting reviewer notes that say the
definitions were unavailable describe their earlier audit stage; this user clarification governs
the assembled report. OCC RQ1/RQ2/RQ3 are separate. OCC-P2 may inform exploratory design of these
new algorithm hypotheses, **never serve as their independent confirmatory test data**.

## 5. Next approved scope and traceability

[EQ-1 plan](EQ1_VALIDATION_PLAN.md) is a **proposal only**, with independent oracles, gates,
resource estimates and explicit GPU approval boundaries. EQ-0 did not execute it. Recovery
Scenario/Belief/Planner, PPO, additional GPU, main merge and push remain unapproved.

Without an environment change, research may preserve/report the existing frozen-loop rates,
audit source/raw provenance and derive independent analytic qualification fixtures. It cannot
claim new Recovery effectiveness, full visual loss, realistic 3D flight or collision-free capture.

This new directory and an appended worktree WORKLOG are locally committed as one EQ-0 unit after
diff/link/evidence validation. The commit records audit evidence, not an environment repair.
See its Git history for the resulting SHA; base is above. Main WIP, stash, Policy F and historical
raw/manifests remain unchanged. External backup is still **NOT_COMPLETED**; this did not block
read-only qualification. Local commits are not visible through GitHub until an authorized push.

## Read-only reproduction and evidence checks

[source_bindings.json](source_bindings.json) binds 45 source/config/asset/protocol files to the
measurement commit and resolves exact function names by AST. [replay_validation.json](replay_validation.json)
records identical replays of the independent readers; [math results](math_reproduced.json) reproduce
the counterexamples. These integrity checks do not promote a physical qualification gate to PASS.

From the research worktree, with preserved raw available:

```bash
CUDA_VISIBLE_DEVICES='' PYTHONNOUSERSITE=1 /home/fair/miniconda3/envs/aerialgym/bin/python -B docs/evidence/eq0_environment_audit_2026-10-17/independent_math_repro.py
CUDA_VISIBLE_DEVICES='' PYTHONNOUSERSITE=1 /home/fair/miniconda3/envs/aerialgym/bin/python -B docs/evidence/eq0_environment_audit_2026-10-17/validate_audit.py --full-raw
```

Standalone readers under `supporting/g1`, `g2`, `g3` and `g4` reproduce state, geometric fixture,
flag/expiry and population inventory checks. They import no simulator or production math modules.
G1/G2/G4 allocate fresh scratch outputs; G3 prints JSON by default and allows only exclusive
scratch output. They do not overwrite committed evidence. No EQ-1 rollout is part of these commands.

## Git Traceability

- Branch: `codex/mt-p1`.
- Base SHA: `7cea3c52d8c8d7ab6dbeef5fae6e929acfe279a2`.
- New Commit SHA: the EQ-0 commit containing this file; resolve with `git log -1 --format=%H -- docs/evidence/eq0_environment_audit_2026-10-17/EQ0_ENVIRONMENT_AUDIT.md`. The user completion message supplies its actual SHA.
- Changed Files: this new audit directory and appended worktree WORKLOG; [exact list](CHANGED_FILES.txt), [SHA index](index.json). No runtime file was edited.
- Tests: independent reader/math replays, 45 measured-source blob comparisons, 560 bound metadata files, 3,347 raw member hashes; document/link/diff checks. [Preservation check](preservation_check.json) is the pre-seal validation-stage record.
- Working Tree: own audit/docs changes are committed as one unit; the pre-existing untracked raw symlinks remain deliberately excluded. Main WIP/stash untouched.
- Remote Push: NO.
- Main Merge: NO.
- Raw/Checkpoint Integrity: PASS for hash preservation; checkpoint not loaded. External backup NOT_COMPLETED.
