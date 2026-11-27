# EQ-1D — GPU environment qualification of the frozen Policy F simulator (run EQ1D-R1)

Status: **completed; environment qualification only.** No Recovery Planner, PPO, H-R confirmation
or correction lineage was run. No simulator, sensor, controller, task, Policy F, OCC result or
EQ-0/EQ-1 record was modified. Known EQ-1A/B/C FAILs were not fixed: they were observed again on
the GPU and recorded separately from the CPU mathematics. A PASS here does not certify hardware
physics or real-camera redetection.

## Identity

| Item | Value |
| --- | --- |
| Approval | user, 2026-10-18 ("MOTAR — EQ-1D GPU Qualification 승인 및 후속 환경 수정 우선순위") |
| Preregistration | `docs/plans/eq1d_qualification_v1.{json,md}`, commit `0a92e76` (before any EQ-1D GPU process); plan SHA-256 `ddb349a1…81b0` |
| Approved source | `d5877703038737fc0878b7487369d0ab46dfaab0` (HEAD; tracked tree clean and frozen during the run) |
| Run | `EQ1D-R1`, 2026-10-18 01:36:07–01:38:52 KST, `../aerial_gym_simulator/results/eq1d_qualification_v1/EQ1D-R1` |
| Raw manifest | 33 members, `RAW_MANIFEST.json` SHA-256 `e28dfc68531b559161bbbc217e541917e696b0960837c3418e71f6115efb4b22`; all re-verified after the run |
| Policy F | `f702213936601860995cf61dcc570247e72543b1976e3716055cd8ec5593ad40`; RMS digest `642fc9f7…387f` |
| Environment | OCC-P2 receipt (`…/E0_static__115bars__seed4301/receipt.json`, SHA-256 `85c13b08…963c`) minus recorder variables; `navrl_quad`, `base_sim` (D1 dt overridden per process), RTX 3070 |
| Analysis | `tools/eq1d_analysis.py` → [`eq1d_analysis.json`](eq1d_analysis.json) (byte-reproducible); run records copied in [`run_records/`](run_records/) |

**Resources (measured).** Official run: 8/8 processes completed, 164.9 s of exclusive GPU wall
time (0.046 GPU-h), 12.97 MB raw output, free disk 19.27 → 19.26 GB. The GPU memory cap was set at
launch from the actual state: 8,192 MiB total − 430 MiB in use (no other compute process) −
512 MiB OOM reserve = 7,250 MiB. Peak per-process GPU memory was 4,076–5,446 MiB (N = 128); torch
itself allocated at most 448 MiB. Development smoke runs (session scratchpad, not evidence)
took 231.3 s (0.064 GPU-h) and 15 MB.

**Reproducibility.** All eight raw output files of the official run are byte-identical to the full
smoke run of the same fixtures, including Policy F in closed loop at N = 128. The simulator is
bitwise deterministic run to run under this contract.

## Results by gate

### D1 — dynamics and timing (24 fixtures × dt 10 / 5 / 2.5 ms, scripted controller commands)

| Gate | Result | Measurement against the independent reference |
| --- | --- | --- |
| D1-T1 control/physics periods | **PASS** | every action = 10/20/40 substeps, command constant within it, 0.1 s per action |
| D1-M1 motor fidelity | **PASS** | logged thrust equals the closed-form production update; max error 0.0032 × (1e-4 \|T\| + 1e-6 N) |
| D1-M2 intended lag | **PASS** | effective gain within 0.096 % (10 ms), 0.023 % (5 ms), 0.020 % (2.5 ms) of 1 − exp(−dt/τ). The EQ-1B named-method FAILs (not classical RK4, order 3) stand. |
| D1-C1 dt convergence | **FAIL** (2/20) | first-order self-convergence: 10-vs-5 / 5-vs-2.5 ratio ≈ 2.08. 5-vs-2.5 position difference 0.0098–0.0265 m (gate 0.01 m), attitude up to 0.64° (fixture 10). Hover and climb pass. |
| D1-H1 hover thrust | **PASS** | total thrust 2.452504 N against m·g = 2.4525 N (relative 1.7e-6) |
| D1-V1 velocity tracking | **FAIL** | axis steps within 0.4 %. The diagonal 3.5355 m/s command reaches the right speed but in a direction off by −18° (rel. error 31 %): the vehicle yaws −17.8° with zero yaw-rate command |
| D1-B1 braking | **PASS** | stopping distance 0.82–0.83 m (2.0 m/s), 1.05–1.06 m (2.5), 1.56–1.58 m (3.52) ≤ riskcap model 0.87 / 1.30 / 2.45 m; mean deceleration 2.1 / 2.5 / 3.4 m/s² |
| D1-A1 tilt ≤ 45° | **FAIL** | the saturated diagonal stays at 45.0°; the oblique contact flips to 90.8° (10 ms), 27.4° (5 ms), 66.8° (2.5 ms) after impact |
| D1-K1 contact onset | **PASS** (fixtures 20, 21, 23 × 3 dt); fixture 22 **NOT_TESTED** | the first contact force (> 0.05 N) falls one substep after SAT separation ≤ 0.004 m at every dt. Fixture 22 (30° corner) did not touch: open-loop yaw drift (−11°) moved the path 0.04 m clear |
| D1-K2 characterisation | — | head-on contact stops the drone within one substep (1.0 → 0.03–0.04 m/s, 2.0 → 0.07–0.09 m/s); the 60° oblique contact keeps 1.3–1.7 m/s. Peak force scales with 1/dt (24 / 48 / 97 N at 1 m/s), an impulse resolution. Crash-detection delay to the end of the action is 0.05–0.09 s. First-contact time spread across dt is ≤ 7.5 ms |

Pre-contact, contact and post-contact states are separate in the substep log. Post-contact
attitude is dt-dependent, so it is not converged.

### D2 — sensors and rendering (32 scenes × 3 attitudes, with and without bars)

| Gate | Result | Measurement |
| --- | --- | --- |
| D2-R1 target render | **FAIL** (precision) | 460,620 robust pixels, **0 mask mismatches**. Depth error > 1e-4 m on 7 pixels, max 3.3e-4 m (centre pixel, target at 15.9 m): float32 cancellation in the analytic sphere kernel |
| D2-R2 obstacle depth, native 40 × 24 | **PASS** | 30,720 robust rays, max 4.7e-5 m |
| D2-R3 delivered 160 × 90 depth | **FAIL** | only 92.7 % of robust pixels within 1e-4 m; 10,673 (2.3 %) off by > 0.05 m, max 7.3 m. The depth frame perception receives is the 40 × 24 render bilinearly upsampled, so bar edges blend into phantom intermediate depths |
| D2-L1 LiDAR | **FAIL** (precision) | 9,216 robust beams; one is off by 1.25e-4 m at 9.9 m; all others ≤ 1e-4 m. Ray table, slant range, 0.20 m target sphere and yaw-only attach confirmed |
| D2-Y1 yaw-only mount | **PASS** | roll/pitch 15° renders bitwise equal to level. Synthetic idealisation: a body-fixed camera would shift 21.8 px at 15° pitch; hardware **NOT_TESTED** |
| D2-F1 fusion on rendered inputs | **FAIL** | 306 in-FOV beams. 33 read nearer than the bars: about −0.1 m from the camera-origin bias (EQ-1A A2), and phantom obstacles at −4.6 m and −7.2 m from upsampled bar edges. 2 beams read farther: a **2.8 m false clear** (below) |
| D2-A1 camera measurement | direction **PASS**, range **FAIL** | direction 0.05–0.13 px; centre range +0.0445 … +0.0508 m (A7 again) |
| D2-O1 occlusion classes | **FAIL** (3/23) | S08: range exit at 24 m reads as FOV exit. S12: centre-visible partial occlusion reads VISIBLE. S14: the rim-gap scene reads FULL while 16 pixels remain detectable. Full (S11, S16, S17, S19) and centre-only (S13) agree |
| D2-S1 scale | **PASS** | rendered pixel counts equal the analytic pixel-centre counts exactly (56, 17, 35, 56, 8, 4, 2, 2, 0, 0, 0, 11, 136); detector visibility equals count ≥ 2 |
| D2-N1 camera negatives | **PASS** | 9/9 bar-only scenes: no detection |
| D2-N2 LiDAR negatives | **FAIL** | **7/7** target-absent bar returns accepted as the target (bearings 0, 20, 90, 180°; 2–10 m) |
| D2-T1 capture/delivery | **PASS** | capture step = delivery step for all 6 passes |

### D3 — task-level GT firewall (Policy F closed loop, N = 4 / 1 / 2 / 128, 320 steps)

| Gate | Result | Measurement |
| --- | --- | --- |
| D3-F1 privileged mutation, all envs | **PASS** | 898-D actor rows and deterministic actions bitwise equal at steps 50/150/300. The mutation covered target position/velocity/orientation, critic states, goal direction, the LiDAR target-injection buffer, the LiDAR semantic buffer (`segmentation_pixels`), obstacle positions and visibility flags, with the rendered sensors replayed |
| D3-F2 permutation | **PASS** | privileged tensors of env i replaced by env i+1 |
| D3-F3 subset | **PASS** | even envs mutated; all envs bitwise equal |
| D3-F4 reset boundary | **PASS** | 2 post-reset branch steps per process |
| D3-P1 positive control | **PASS** | moving the target 0.6 m with re-rendered sensors changed the actor input in every branch, so the comparison is sensitive |
| D3-R1 RMS | **PASS** | digest `642fc9f7…` at restore and after 320 steps; model and RMS in eval mode |
| D3-T1 delivery latency | **PASS** | 320 steps per process: camera render, LiDAR render and observation share the step counter (0-step latency) |
| D3-B1 batch sizes | **PASS** | N = 1, 2, 4, 128 |
| D3-S1 identity | **PASS** | actor width 898, Policy F SHA verified in-process, imports from the worktree; TF32 matmul on (the historical Runner setting) |

Sensor-changing interventions (P_control) and actor-leak tests (B branches with replayed
sensors) were separate branches. No mismatch occurred, so no tensor or source edge needs recording.

## Consistency with EQ-1A/B/C

| EQ-1 finding | EQ-1D on the GPU |
| --- | --- |
| A1/A2 fusion geometry FAIL | reproduced on rendered inputs (−0.1 m bias; D2-F1), and enlarged by depth upsampling |
| A4/A7 radius and centre range | camera +0.045–0.051 m (D2-A1); LiDAR proxy 0.20 m confirmed in the renderer |
| A5 occlusion labels | the same three disagreement types on real renders (D2-O1) |
| A6/T3 false association | 7/7 on rendered LiDAR scans (D2-N2) |
| B1 motor (named-method FAIL, composite ≈ lag) | the composite gain agrees with the exact lag within 0.1 % on the GPU (D1-M2); the CPU FAIL stays |
| B2 tilt > 45° | reproduced in contact (90.8°); saturation sits at the 45° command limit |
| B5 riskcap floor | the plant brakes within the model (D1-B1); the floor counterexample (EQ-1B) is unchanged |
| C1/C2 module-level firewall and RMS | confirmed at task level, all batch sizes (D3) |

## New problems found by EQ-1D

1. **Delivered depth is upsampled 40 × 24.** 2.3 % of robust pixels are off by more than 5 cm
   (max 7.3 m). Through fusion this creates phantom obstacles at intermediate ranges at bar
   edges (D2-R3, D2-F1).
2. **Target-like carve-out deletes real obstacles.** In S14 a bar 0.53 m in front of the target lies
   inside the 15° / 0.55 m target-like window. It is removed from the obstacle scan, and the fused
   scan reports 5.14 m where the LiDAR measured 2.32 m: a 2.8 m false clear on 2 beams (D2-F1).
   EQ-0 had only listed this as a possible hazard.
3. **Open-loop yaw is not held.** With zero yaw-rate command, combined lateral motion yaws the
   vehicle by −17.8° (diagonal held), −32.8° (diagonal brake) and −131° (saturated diagonal). The
   vehicle-frame command then rotates with it (D1-V1). In closed loop the policy commands yaw.
4. **First-order dt dependence.** Position diverges at first order, about 2.08× per halving. The
   altitude drift with zero vertical command is −0.039 / −0.020 / −0.010 m over 6 s, so the 0.01 m
   convergence gate fails narrowly for most fixtures (D1-C1). Post-contact motion is not converged.
5. **Range exit is conflated with FOV exit** by the centre-based frustum indicator (S08 at 24 m).
6. **float32 limits** of the analytic target and LiDAR kernels reach 3.3e-4 m and 1.25e-4 m. Below
   any task tolerance, but above the 1e-4 m exact-fixture gate.

Development smoke runs also exposed one launcher bug. The 30° corner fixture was placed 0.39 m
clear of the bar; it was fixed to match the plan before the official run and disclosed in the
WORKLOG. The gates and fixtures did not change.

## Correction to an earlier record

EQ-1 (`1257156`, REPORT B5 and WORKLOG) said the production `stopcap` mode was "never trained or
evaluated with Policy F". The training half is right; the evaluation half is wrong. A
preregistered screen evaluated frozen ep25000 (`f7022139…`) under stopcap on 2026-09-02
(`aerial_gym/rl_training/rl_games/eval_navrl_v2_ep25000_stopcap_screen.sh`,
`docs/prereg_2026-09-02_speed_governor_stopcap_screen.md`, results
`results/navrl_v2_ep25000_stopcap_seed49_screen`). The EQ-1 record is kept as committed. That
screen was not re-analysed here.

## Impact on the provisional H-R hypotheses

- **H-R1 (tracking continuity).** The target-absent association (7/7) and the range-only
  covariance FAIL mean an "active" or "fused" track is not evidence of valid identity. Continuity
  metrics must use E-L0 provenance labels and silhouette-based loss types: FOV, range and scale
  are now distinguishable in the renderer. Positive for H-R1's design: the firewall holds at task
  level, so prediction or visibility gains cannot come from GT leakage into the actor.
- **H-R2 (safe approach).** The carve-out false clear and the phantom obstacles affect exactly the
  near-target geometry where approach and capture happen. Riskcap's floor remains unsafe as a
  guarantee, although the plant itself brakes within the model. Contact is detected 0.05–0.09 s
  late and capture wins a same-step contact, so collision-free capture needs E-L0 contact labels.
  H-R2 comparisons against Frozen F must disclose these sensor-side effects.
- **H-R3 (active visibility).** The camera is an ideal yaw-only gimbal (D2-Y1). Viewpoint gains
  measured here would not transfer to a body-fixed camera without qualification. Open-loop yaw
  drift means scripted viewpoint baselines must close the yaw loop. Matched arms are supported by
  the bitwise determinism and the snapshot/restore branch mechanism shown in D3.

## Re-judgement input (for the user's decision)

| Domain | EQ-1A/B/C | After EQ-1D |
| --- | --- | --- |
| Dynamics and timing | LIMITED_VALIDITY | **LIMITED_VALIDITY.** Timing, motor fidelity, hover, braking and contact onset PASS. dt convergence first order (fails the 0.01 m gate), yaw/tilt not bounded, post-contact not converged |
| Sensors and rendering | LIMITED_VALIDITY | **LIMITED_VALIDITY.** Target rendering, LiDAR geometry, scale and camera negatives PASS (float32 aside). Delivered depth, fusion, occlusion labels and association FAIL. New: carve-out false clear, upsampling phantoms |
| Information flow | LIMITED_VALIDITY | **PASS for the declared firewall and RMS claims** (task level, all batch sizes); identity/provenance still not recorded |
| Reproducibility | LIMITED_VALIDITY | **PASS for bitwise run-to-run determinism** of these fixtures; full historical-state replay is still NOT_TESTED |

The follow-up lineages are proposed in [LINEAGE_DIFFS.md](LINEAGE_DIFFS.md), in the user's order:
E-L0, S-L1, P-L1, G-L1, then camera-based. None is implemented.
