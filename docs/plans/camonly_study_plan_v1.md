# Camera-only vs camera + LiDAR: what the LiDAR contributes (plan v1)

Status: plan, 2026-11-04, decision [D-2026-11-04-3](../DECISIONS.md). Branch `camonly/dev`, worktree
`src/MOTAR-camonly`, base `ab6531b` (`gainrand/dev`).
- The user approved the direction and the recommendations K1-K4 and K6 ("일단 진행해. 그렇게.").
- They judged K5's native-resolution arm hard and asked for another way; that way is in "K5 revised".
- This plan fixes the C0 gate (CPU). Each GPU stage needs its own preregistration and the user's approval.
- The design draft it comes from is `~/motar_tmp/camonly_design_20261104/CAMONLY_study_DRAFT.md`. The first draft,
  which wrongly kept the LiDAR in every option, is
  `~/motar_tmp/gainrand_20261104/design/DEPTH_camera_training_DRAFT.md`.

## Question and authority

- **The user's question (2026-11-04):** "기존의 우리 방식은 카메라와 라이다를 둘 다 쓰는건데, 라이다를 쓰지 않았을때
  어떠한 차이가 나타나는지를 보고 싶어서" — what difference appears when the LiDAR is not used?
- **Authority.** AGENTS.md and `docs/SENSOR_CONTRACT.md` forbid removing or zeroing the LiDAR "without a new explicit
  user instruction". This message is that instruction, scoped to this comparison.
  - Frozen Policy F and the official RGB-D + LiDAR contract stay unchanged.
  - The camera-only arm is an opt-in research arm, default OFF.
  - The unit records the instruction as a `docs/DECISIONS.md` entry in its first commit.

## Facts this design rests on

The source is a read-only survey of `MOTAR-mt-p1`, 2026-11-04
(`~/motar_tmp/camonly_design_20261104/lidar_survey_2026-11-04_from_transcript.md`). The lines marked (re-checked)
were checked again on `MOTAR-gainrand` `ab6531b`, the same task file.

| Where the LiDAR enters | What it does now | Without the LiDAR |
| --- | --- | --- |
| Actor, static scan (288) | min-fusion of LiDAR and camera depth; the camera covers only \|bearing\| ≤ 43.5° | 17 of 72 bins have a source; **220 of 288 values have none**; in-FOV 10-12 m has none |
| Actor, obstacle tokens (480) | from the fused scan, 8 sectors of 30° over ±120° | the camera fully covers 2 sectors, partly covers 2, and covers **4 not at all** |
| Actor, target history (80) | Kalman update from the camera; LiDAR backup correction; fused visibility = camera OR LiDAR; one LiDAR-confidence field | camera only; `NAVRL_LIDAR_TARGET_ASSOC=0` already exists |
| Actor, robot history (50) | ego state, previous action | unchanged |
| Riskcap speed governor | **raw LiDAR** `depth_range_pixels`, corridor 0.45 m half-width (T:5259-5261, re-checked) | needs a new clearance source |
| Reward r_ss (training) | mean log of **raw LiDAR** ranges (T:6307-6308, re-checked) | train-time signal, not a sensor of the deployed policy |
| Critic (training) | actor observation + 8 privileged values | follows the actor observation |
| Crash detection | physics contacts | unchanged |

Further facts from the survey:
- **No switch and no result.** No switch removes, zeroes or masks the LiDAR, and no camera-only or LiDAR-zeroed run
  was ever executed. `RESEARCH_PLAN.md`:102 and :285 planned such comparisons; none ran.
- **Empty reads as free.** Policy F has no "unknown" encoding. 1.0 in the static scan means "no return within
  12 m", and empty token slots are all zeros. Filling the missing slots with 1.0 or 0 reads as free space; filling
  the static scan with 0 reads as a wall all around. A zero-shot camera-only F is therefore out of distribution.
- **Shape stays.** Keeping 898-D, HBEAMS, MAX_OBSTACLES and the 12 m normaliser lets F's weights load, so a
  warm-started camera-only continuation is possible.
- **Camera depth.** 40 x 24 out to 10 m, bilinearly upsampled, which creates phantom obstacles (EQ-1D). Without the
  LiDAR this is the only obstacle source.
- **Existing hints.** 19.4 % of frames are camera losses, and LiDAR-only fused visibility covers 42.7 % of them
  (WORKLOG:20657). Earlier LiDAR target-path ablations moved capture by at most about 3 pp, but they kept the
  LiDAR obstacle path.

## Definition of the camera-only arm

The deployed controller uses **no LiDAR information online**:
- actor observation: LiDAR values replaced as fixed below;
- target association and its LiDAR correction off; LiDAR-confidence field 0; fused visibility = camera only;
- governor clearance from the camera only.

Allowed, and identical in both arms:
- **Evaluation recorders and metrics** may still read the LiDAR and ground truth (bar-contact probe, contact
  geometry). They do not feed control.
- **Training-only signals.** The reward r_ss and the critic's privileged values are the same in both arms. The
  observation contract allows privileged information there. Changing the reward would add a second difference.
  Consequence: the claim is about the sensors of the deployed policy, not about a LiDAR-free training pipeline.

## Design decisions (user, 2026-11-04: K1-K4 and K6 as recommended; K5 revised)

| Id | Decision | Recommendation | Why |
| --- | --- | --- | --- |
| K1 | Fill value for the slots with no source | static scan 1.0 ("no return", the same value the camera's own no-return maps to); empty obstacle slots 0, as now | it is the existing encoding of "nothing seen". For a trained camera-only policy a constant carries no information, so it can learn to discount it. Zero-shot it reads as free space; that is a property of F, reported, not hidden |
| K2 | Governor without LiDAR | clearance from the camera-fused scan inside the FOV; clearance 0 outside, so the cap is 2.0 m/s | it is the riskcap law with an honest sensor. It reduces to F's `fixed` 2.0 mode for lateral and rear commands |
| K3 | Separate the two roles of the LiDAR | a 2 x 2 zero-shot factorial: observation {fused, camera} x governor {LiDAR, camera} | it attributes the change to perception or to the safety filter. Evaluation only, so cheap |
| K4 | Control arm for training | reuse the GR-1 `ctrl` continuation (F + 1,000 epochs, nominal gains, seed 1), provided the identity stage shows the switch OFF reproduces it | it saves one training run. Otherwise train a new matched control |
| K5 | Depth-fidelity confound | **revised:** split the LiDAR by bearing instead of changing the camera: a 2 x 2 of LiDAR {inside the camera FOV, outside it}, plus a CPU sensor-level measurement of the camera's misses and phantoms (see "K5 revised") | a native-resolution camera changes the observation statistics, so F would need retraining; it would also change GPU memory and was judged hard |
| K6 | Training length | 1,000 epochs, as GR-1. If the camera-only reward is still rising at the end, extend both arms equally under a new preregistration | equal budgets keep the comparison fair |

## K5 revised: separate camera quality from field of view without changing the camera

Changing the camera's resolution is hard here:
- it changes the in-FOV statistics F was trained on, so the result would need retraining;
- it changes GPU memory at 128 environments;
- the 898-D observation collapses the camera to 17 bins of 5 deg in any case.

But the code shows exactly how the camera reaches the policy (`navrl_perception.py:1910-1921`, read 2026-11-04):
- each in-FOV LiDAR bin takes the **minimum** of the LiDAR range and one camera column, the column at the bin's own
  bearing, minimum over all image rows;
- the camera image is the 40 x 24 obstacle render, bilinearly upsampled.

So the camera's resolution can affect the observation in only two ways:
- **Phantoms**, where the camera reads too near: bilinear blending between a bar and the background. Min-fusion
  keeps the nearer value, so **the LiDAR can never remove a phantom**. Phantoms are present with and without the
  LiDAR, so the comparison never credits the LiDAR with removing them. They can still matter through interactions,
  for example when out-of-FOV information is also missing.
- **Misses**, where the camera reads too far: a bar between the 40 render rays, or a bar at 10-12 m beyond the
  camera's range. The LiDAR corrects these only where one of its beams, 5 deg apart at the same bearings, hits the
  bar.

Therefore the camera-quality part of the LiDAR's value lives entirely inside the FOV, and the field-of-view part
entirely outside it. Removing the LiDAR by bearing separates them:

| Condition | LiDAR inside FOV | LiDAR outside FOV | Reads as |
| --- | --- | --- | --- |
| A `full` | yes | yes | F as deployed (GR-0 S00 cells) |
| B_in `no_lidar_in_fov` | no | yes | the camera's own in-FOV quality, with LiDAR coverage elsewhere; close to F's training distribution |
| B_out `no_lidar_out_fov` | yes | no | a sensor with the camera's FOV but LiDAR-corrected quality inside it |
| C `no_lidar` | no | no | camera only |
| C_gL `no_lidar_obs_only` | no (observation) | no (observation) | camera-only observation, governor still on the raw LiDAR (K3) |

Each condition applies to every online consumer: observation, target association and governor. The exception is
C_gL, where the governor keeps the LiDAR.

- A − B_in: value of the in-FOV LiDAR. This is the cost of the camera's misses, i.e. resolution and range.
- A − B_out: value of the coverage outside the FOV.
- A − B_in − B_out + C: their interaction.
- C_gL − C: the governor's share once the observation is camera-only.

**Sensor-level check (CPU, no policy).** Render the 40 x 24 camera and the 72 x 4 LiDAR with MOTAR's own Warp
kernels on the CPU at poses in the recorded bar layouts, as M2's replay did for the LiDAR. Compare each in-FOV bin
with exact geometry and count misses and phantoms by range and by bar position between rays. This says how often
each kind of error happens, independent of the policy.

What this cannot do: say what a better camera would give a policy trained for it. Only training could answer that,
so it is out of scope (the user's judgement).

## C0 implementation (no pinned file changes)

`navrl_task.py` and `navrl_perception.py` are byte-pinned by the H-R3 tests (`tests/test_h_r3_p02_runner.py:14-19`,
`tests/test_h_r3_env_isolation_run.py:11-13`). They stay unchanged.

- **New module** `aerial_gym/task/navrl_task/navrl_sensor_ablation.py`. Nothing imports it unless the new entry
  point does. `install(condition)` wraps three methods at run time:
  1. `NavRLPerceptionModule.observe`: `lidar_m` is masked before use. Removed bins become 12 m, "no return". So:
     - in the FOV the fused value is the camera's;
     - outside the FOV it is the K1 fill 1.0;
     - target association finds no LiDAR return, and the LiDAR-confidence field and the LiDAR part of fused
       visibility follow without further changes.
  2. `NavRLPerceptionModule._fuse_static_and_extract_obstacles`: records the fused scan in metres for the governor,
     and the target mask computed on the raw LiDAR for C_gL. Its output is unchanged.
  3. `_apply_sensor_speed_governor` together with `directional_lidar_clearance`, only inside that call:
     - the clearance is computed on the governor scan: LiDAR where allowed, the camera-fused value in removed
       in-FOV bins, 12 m in removed out-of-FOV bins;
     - when out-of-FOV LiDAR is removed and the command bearing is outside the FOV, the clearance is 0, so the cap
       is 2.0 m/s (K2).

     Any other governor feature (omni, arc, open-width, lateral or yaw channel) raises in an ablation condition, so
     it fails closed. F uses none of them.
- **Entry point:** a launcher copy sets `PYTHON` to a wrapper. The wrapper imports `isaacgym`, calls `install()`
  from `NAVRL_SENSOR_ABLATION` and runs `runner.py` unchanged. `play_navrl.sh` and the pinned launcher are not
  touched.
- **CPU tests (C0 gate):**
  - Leak: for each condition, random LiDAR in the removed bins leaves the actor observation, the association state
    and the governor command bit-identical. Positive control: with `full`, the same perturbation changes them.
  - Transparency: `full` gives bit-identical outputs to the unwrapped module.
  - Replica: the raw-LiDAR target mask computed by the wrapper equals perception's own mask under `full`.
  - Governor: the K2 rule caps exactly the commands outside the FOV, and the in-FOV clearance equals a direct
    computation on the camera bins.

## Stages

| Stage | Hardware | What | Pass / reading |
| --- | --- | --- | --- |
| C0 switch | CPU | opt-in `NAVRL_SENSOR_ABLATION=<condition>` through a separate entry point; see "C0 implementation" | OFF: the pinned files and launchers are unchanged; `full` through the new entry is transparent |
| C0 leak test | CPU (checked 2026-11-04: `NavRLPerceptionModule` runs on `"cpu"` in `tests/test_navrl_latency_compensate.py`, 40 tests OK in 1 s; `speed_governor.py` imports only torch and loads on the CPU) | randomise the LiDAR tensor and require actor observation, association state and governor command to be unchanged in camera-only mode | positive control: in fused mode the same perturbation must change them (non-vacuous) |
| C1 identity | GPU, about 10 min | condition `full` through the new entry reproduces a GR-0 Policy F cell exactly, as gr1repro did | exact equality |
| C2 zero-shot conditions | GPU, about 2 h | four new conditions on GR-0's S00 grid (12 blocks, quota 16, E2, A3); `full` is GR-0's S00 cells, after C1 | a diagnostic; conditions without out-of-FOV LiDAR are labelled out of distribution; no adoption decision |
| C3 training | GPU, about 70 min | camera-only continuation of F, the same contract as GR-1 `ctrl` | TRAINED |
| C4 matched evaluation | GPU, about 90 min | camera-only continuation vs `ctrl` on the same grid | capture and crash differences with block-bootstrap CIs; thresholds fixed in the preregistration |
| reported, not gated | CPU | crash causes (bar contact vs floor), crashes with the nearest bar inside vs outside the FOV, time to capture | |

Reading of C4, to fix in the preregistration: the LiDAR contribution is capture(camera + LiDAR) − capture(camera
only) and crash(camera only) − crash(camera + LiDAR), with "no material loss" and "material loss" thresholds set
before the run. The preregistration also needs the gate-design checklist: a known-bad control at the scale of the
claim, measured coverage, and an independent design review before measurement.

## Feasibility on the CPU

- **Not feasible:** the closed-loop evaluation and training. The full stack uses the GPU pipeline. Isaac Gym can
  create a CPU simulation, but the task, sensors and rl_games are wired to `cuda:0`.
- **Feasible:**
  - C0 (the switch, unit tests and the leak test; perception and governor run on CPU tensors, checked 2026-11-04);
  - all analysis of recorded cells;
  - the Warp camera kernel itself (the 2026-11-04 probe returned the correct 3.750 m depth on CPU).

## C0 gate (CPU), fixed before its gate run

C0 is **PASS** only if all of the following hold on one commit of `camonly/dev`:
1. `tests/test_navrl_sensor_ablation.py`: every test passes. The tests cover leak with positive controls, kept bins,
   transparency of `full`, equality with a LiDAR that returns nothing (K1), the raw target-mask replica, the
   governor's K2 rule and in-FOV clearance, obs_only, refusals and isolation, and the entry point.
2. Known-bad controls (`tests/camonly_c0_mutants.py`): the unmodified module passes, and each of the five mutants
   makes at least one test fail. The mutants are:
   - no masking in perception;
   - a governor on the raw LiDAR;
   - no K2 cap;
   - a fill of 0 m instead of no return;
   - the removal mask shifted by one bin.
3. The full PUBLIC_CPU suite (`tools/run_tests.py`), run in one process in this worktree:
   - it adds no failure or error to the set the same suite gives on the base `ab6531b` in the same worktree and
     environment (the new files moved out);
   - the new tests pass inside it.

   A clean "0 failures" bar is not available here. The base tree already fails in this environment because of
   untracked raw files that no worktree has (`results/...stopcap_seed49_screen`,
   `results/...braking_lower1p25_headingrest_seed827`) and a temporary directory that one test assumes. The
   2026-11-04 GR worktree run fails on the same tests. This item was corrected before its gate run; the first
   development run is disclosed in the WORKLOG.
4. `git diff ab6531b -- aerial_gym/task/navrl_task/navrl_task.py aerial_gym/task/navrl_task/navrl_perception.py`
   is empty.
5. An independent review (Opus 5.5, fresh context) of the C0 code and tests finds no blocking issue before C1.

Until item 5, C0 is reported as CONDITIONAL at most.

Limits of C0:
- It shows the switch does what it says on CPU tensors with the real perception and governor code.
- It does not show the simulator path: the entry point under Isaac Gym, or the launcher copy. That is C1's
  identity.

## Order with the other work

1. GR-1 evaluation (running) and its review. Commits on `camonly/dev` happen in another worktree. They do not
   touch the running tree (`MOTAR-gainrand`), which stays frozen.
2. C0 on the CPU, then its independent review.
3. C1-C2 need a preregistration and the user's GPU approval; C3-C4 need another.

## Open decisions for the user

- GPU approval for C1 and C2, after C0 and its review.
- Whether C2's zero-shot result alone answers the question, or C3-C4 (new PPO) is wanted. The user has approved
  new PPO in principle (D-2026-11-04-1, GR unit); each run still needs its own approval.

## After review r1 (2026-11-04): repairs and corrected readings

The C0 gate run gate_01 on `4d03f35` read C0_CONDITIONAL_PASS. The independent review r1 (Opus 5.5, fresh
context; `docs/evidence/camonly_2026-11-04/c0/REVIEW_r1.md`) read **FAIL** on one blocking, test-only finding.
This section supersedes the earlier text where they differ. The earlier text stays as written.

**B1 (blocking): the association leak check was vacuous.**
- In the original test scene the camera never lost the target, so the LiDAR target association never ran:
  0 of 24 env-steps.
- A mutant whose association reads the raw LiDAR passed every test.
- Repair: two camera-loss scenes, inside the FOV and outside it (the vehicle is yawed by 60 deg).
  - Under `full` the association runs on 12 env-steps in each scene, and it changes under perturbation.
  - In each condition that removes those bearings it runs on 0 env-steps, and every output is bit-identical.
  - In each condition that keeps them it still runs on 12.
- The mutant `assoc_raw` is now caught.

**M1: the governor shrank camera clearances by cos 20 deg.** The governor projects each of the 4 rows by its
elevation; perception copies one camera value into all of them. The governor scan now divides the camera value
by the cosine of each row's elevation, so the projection returns the camera value.
- Test: one bar 4 m ahead gives the same governed speed seen by the LiDAR (`full`) and by the camera
  (`no_lidar_in_fov`, `no_lidar`): riskcap's 2.768 m/s.
- The mutant `no_elevation_fix` is caught.
- Remaining and disclosed: camera values are depths along the principal axis. F's perception uses them as
  ranges, and the governor inherits that, so off-axis camera clearances are under-estimated by about cos(bearing):
  0.73 at the FOV edge. This is part of "the camera as F's perception uses it", not removed.

**M2: the K5 claim holds for the observation only.** In A the governor reads only the raw LiDAR, so camera
phantoms never slow it. The in-FOV LiDAR also holds the target track when the camera loses it inside the FOV.
So A − B_in is not only the camera's misses. Corrected reading, with the new condition
`no_lidar_in_fov_obs_only` (B_in_gL: in-FOV LiDAR removed from perception, governor on the raw LiDAR):
- A − B_in_gL: the observation's in-FOV use of the LiDAR, i.e. the camera's misses plus the in-FOV target
  association.
- B_in_gL − B_in: the governor's in-FOV use of the LiDAR, including its immunity to camera phantoms.
- The association share inside A − B_in_gL is not separated here. The existing switch
  `NAVRL_LIDAR_TARGET_ASSOC=0` can separate it, as an optional condition in the C2 preregistration.

**M3: PATH.** `play_navrl.sh` puts the `PYTHON` wrapper's directory first on `PATH`, so ninja (for the gymtorch
build) was not found. `tools/camonly_python` now puts the interpreter's bin directory first. Tested.

**Disclosed, not changed:**
- L1: in the `*_obs_only` conditions the observation's previous-action field is the command after the
  raw-LiDAR governor. Those observations are not strictly LiDAR-free.
- L2: in the `*_obs_only` conditions the governor's target mask uses the camera-only visibility. A target the
  camera has lost is therefore an obstacle to the raw-LiDAR governor.
- L4: under the entry point, `__file__` and `argv[0]` of `runner.py` are absolute paths. `__name__`,
  `sys.path[0]` and `__main__` match a native run.
- L9: under ablation, joint-speed telemetry mixes the ablated requested clearance with raw-LiDAR actual
  clearances. This affects a recorder only.

**Requirements for the C1/C2 launcher (L5):**
- each cell gets its own receipt path;
- the runner refuses a cell whose receipt is missing or names another condition, because `NAVRL_SENSOR_ABLATION`
  without the entry point is silently ignored;
- `PYTHON=tools/camonly_python` is set only for the `play_navrl.sh` call.

**Tests added for the low findings:**
- L3: quiet startup.
- L6: the governor-mode, lineage and "governor before perception" refusals, and the K2 threshold at 43 vs 44 deg.
- L7: the test camera uses the production 10 m far plane.

**C0 gate after r1:** the same five items, with 14 tests and 7 mutants. It is run again on the repair commit
(gate_02) and reviewed again (r2).
