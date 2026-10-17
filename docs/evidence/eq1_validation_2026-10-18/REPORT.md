# EQ-1A/B/C — CPU independent validation of the frozen Policy F environment

Status: **completed, validation only.** No sensor, physics, controller, Policy F, OCC-P1/P2 source or
raw result was modified. Every production verdict below comes from executing the frozen production
code (or reading the frozen raw records) against an independent reference. A reproduced
counterexample is recorded as a FAIL of the current implementation; a proposed correction is
reference code only, recorded separately, and is not wired into any lineage.

- Approval: user message "MOTAR — EQ-0 검토 완료 및 EQ-1 CPU 독립 검증 승인" (2026-10-17), after EQ-0
  (`197b794`, overall `LIMITED_VALIDITY`).
- Source: worktree `codex/mt-p1`, base `197b79414dddbd3bd3fcecb51f46ddc7677873ce`. Every exercised
  production path is byte-identical to the OCC measurement source `5ff285ad053f181c5a1c86dddb2462bb44e6327b`
  (`git diff 5ff285a HEAD -- aerial_gym motar_eval` is empty; each record repeats the check and the
  file SHA-256s under `source`).
- Environment: Python 3.8.20, torch 2.4.1+cu121 (CPU, `CUDA_VISIBLE_DEVICES=''`), NumPy 1.24.4,
  SciPy 1.10.1, rl_games 1.6.5; 24 cores. Production perception is built under the OCC-P2 receipt
  environment (`static/A1/E0_static__115bars__seed4301/receipt.json`: LiDAR 72×4 / 12 m,
  `MAX_OBSTACLES=8`, `cluster_sector`, riskcap settings, perturbations off at evaluation).
- Raw inputs (read only): `../aerial_gym_simulator/results/occlusion_confirmatory_v1/OCC-P2`
  (112 cells, 57,344 quota episodes, 2,320,985 quota frames); Policy F
  `f702213936601860995cf61dcc570247e72543b1976e3716055cd8ec5593ad40`.
- Records: seven files in this folder, each regenerated twice from the committed code with
  byte-identical SHA-256 (see [`index.json`](index.json)). Total CPU wall ≈ 3 min with 12 workers.

Companion documents: [sensor candidates](SENSOR_CANDIDATES.md),
[evaluation contract design](EVALUATION_CONTRACT.md), [EQ-1D GPU contract](EQ1D_GPU_CONTRACT.md).

## Independent references used

| Package | Reference (never imports production) |
| --- | --- |
| EQ-1A geometry | pinhole projection `u = cx − fx·tanθ` (`fx = 84.3024`, `cx = 79.5`); explicit SE(3) of the camera mount (0.10, 0, 0.03) m; analytic ray/sphere and ray/box; 8×8 sub-pixel analytic silhouettes of the r = 0.15 m target, with and without occluders |
| EQ-1B motor | closed-form exact lag `1 − exp(−dt/τ)`, independent classical RK4, exact zero-order-hold step |
| EQ-1B scene | URDF parsed with ElementTree and transformed by joint/visual origins; exact box-distance cell certificates (conservative 4-connected / optimistic 8-connected) with SciPy labelling |
| EQ-1B envelope | exact ray cast against Minkowski box+disc obstacles (checked against brute-force marching); stopping inequality `c − m ≥ v·t_r + v²/2a` |
| EQ-1C tracker | float64 constant-velocity Kalman filter from the equations; EKF range-only Jacobian `H = uᵀ` |
| EQ-1C flow | AST of the task call site; held-fixed-input invariance; independent recomputation of the quota and legacy-window rules from ledger rows |

## Results

Gate values were fixed in the EQ-1 plan or in each tool's docstring before execution; none was relaxed.

### EQ-1A — sensor geometry ([`eq1a_geometry.json`](eq1a_geometry.json))

| ID | Test | Current implementation | Proposed correction (reference only) |
| --- | --- | --- | --- |
| A1 | Camera column → LiDAR bearing (gate ≤ 0.5 px), 17 beams in the FOV | **FAIL** — 20° beam: pinhole u = 48.8164, production column 43, **5.8164 px / 3.4109°**; worst 25°: 6.1891 px / 3.3568° | **PASS** — `u = round(cx − fx·tanθ)`, worst 0.49999 px (column quantisation) |
| A2 | Camera range reused as body-origin range (front plane x = 5 m; gate ≤ 1e-4 m) | **FAIL** — on axis **−0.0998 m** (the 0.10 m mount); off axis −0.0998 … +0.0614 m (mount plus A1) | **PASS** — SE(3) reprojection of each hit plus chord intersection with the beam, worst 2.1e-7 m (float32 depth) |
| A3 | Vertical aggregation | **LIMITED_VALIDITY** — conservative 2-D envelope PASS (no false clear); per-elevation metric FAIL (one camera minimum copied to all 4 LiDAR rows) | — |
| A4 | Radius / centre-range conventions | **FAIL** — LiDAR return (0.20 m proxy) + association radius 0.15 → **−0.050 m**; camera mean surface depth + 0.15 → +0.016 … +0.044 m (limit +r/3) | **PASS** for the LiDAR term (add the proxy's own radius or unify radii); camera: invert the sphere per pixel or use the minimum depth |
| A5 | Full / partial / centre-only / FOV / scale labels (11 fixtures) | **FAIL** — 3 disagreements: clear target at 20 m reads FULL (0 pixels = scale loss); centre-visible one-side occlusion reads VISIBLE; two bars blocking all 9 rays read FULL while 22 pixels remain visible and detectable. Possible scale loss from **12.75 m**, certain from **23.0 m** (clear on-axis target, min_target_pixels = 2) | **PASS_BY_CONSTRUCTION** (design) — label from target-alone support and visible support |
| A6 | Identity: target-absent bar at the predicted range | **FAIL** — accepted as the target and the track age reset to 0; a bar 1.5 m beyond the prediction is rejected | **NOT_TESTED** — identity needs appearance/re-ID evidence; otherwise label it "range association" |
| A7 | Production camera target measurement on analytic frames (4 fixtures) | Direction **PASS** — ≤ 0.10 px (intrinsics, 0.10/0.03 m extrinsic and vertical sign consistent; ignoring the offset would show 0.52–1.27 px). Centre range **FAIL** — +0.0445 … +0.0585 m | (A4 camera correction) |

### EQ-1B — physics and scene feasibility

Motor ([`eq1b_motor.json`](eq1b_motor.json)). Policy F: `navrl_quad`, dt 0.01 s, RPM-domain RK4,
τ = 0.04 s, discrete mixing factor `1/(dt + τ)`; the 1e5 rate limit is never reached (largest rate
9,293).

| ID | Test | Result |
| --- | --- | --- |
| M1 | Routine called rk4 equals classical RK4 for its own ODE (≤ 1e-12) | **FAIL** — gain 0.2214 versus 0.1812667 (stages use `e + dt·k/2`) |
| M2 | Convergence order ≥ 3.5 against the intended lag, dt 10/5/2.5/1.25 ms | **FAIL** — observed **2.84 / 2.91 / 2.95** (global order 3). With x = dt/τ the production gain is x − x²/2 + x³/6 + x⁴/24, so the sign error and the `1/(dt+τ)` coefficient cancel up to the x³ term; local error x⁴/12. Classical RK4 on the intended ODE: 4.15 / 4.08 / 4.04 |
| M3 | Preserve the fixed-step response (≤ 0.5 % of gain 0.2214; τ_eff 0.039959 s) | ZOH exact τ = 0.04: **PASS** (−0.091 %); classical RK4 with `1/τ`: **PASS** (−0.094 %); naive sign flip keeping `1/(dt+τ)`: **FAIL** (−18.1 %, a 0.050 s motor); backward Euler: **FAIL** (−9.7 %) |
| M4 | Rate-limited regime | **PASS** (dt·max_rate exactly); not exercised by Policy F |

Speed and tilt ([`eq1b_envelope.json`](eq1b_envelope.json)). OCC-P2 quota frames at 10 Hz; post-contact states are not recorded.

| Class | Frames | Tilt median / P99.9 / max | > 45° | Horizontal speed median / P99 / max | Governor-brake stopping margin < 0 |
| --- | ---: | --- | ---: | --- | ---: |
| normal flight | 2,302,287 | 6–9° / 40–41° / 91.0° | 951 (0.041 %) | 2.9–3.1 / 3.43 / 4.47 m/s | 0.42 % |
| last 1 s before a crash | 18,698 | 10–12° / 92–94° / 111° | 1.48 % | 1.9–2.0 / 3.0–3.3 / 4.79 m/s | 47.0 % |
| last recorded frame before a crash | 1,898 | 6.5–6.9° / 84–88° / 89.9° | 1.26 % | **1.96–1.98** / 3.2 / 4.79 m/s | **97.4 %** (median free travel 0.02–0.03 m) |
| during / after contact | — | NOT_RECORDED | | | |

Tilt above 45° (`NAVRL_MAX_TILT_DEG`) occurs in normal flight, so 45° is a command limit, not a
state bound (**FAIL** for a state-bound claim). 171 normal frames exceed the 3.5355 m/s free cap by
more than 1 %. No surviving frame is closer to a bar than the optimistic body radius (contact-model
consistency **PASS**). Crashes happen at the riskcap floor speed with almost no free travel left.

| ID | Test | Result |
| --- | --- | --- |
| B3 | URDF visual inside collision | 3-D **FAIL** — base_link visual sphere r 0.05 m exceeds the 0.08 m-thick box by 0.010 m vertically; horizontal **PASS** (motor cylinders reach the ±0.14 m faces exactly). No propellers modelled: rotor envelope **NOT_TESTED**. Bars: 40/40 assets visual = collision **PASS**; all recorded layout half extents match an asset |
| B4 | Start → capture-disc passability on all 57,344 recorded quota layouts ([`eq1b_scene.json`](eq1b_scene.json)) | Body tier (conservative r = 0.2020 m / optimistic 0.0990 m): **57,344 / 57,344 certified CONNECTED** (all at the 0.1 m grid); 0 disconnected. Reserve tier (+0.45 m): 57,340 certified, **4 UNRESOLVED** at 0.025 m (listed), 0 disconnected. Initial target position only. Dynamic feasibility **NOT_TESTED** |
| B5 | Riskcap braking law (production `apply_speed_governor`, receipt settings) | Fidelity to its own inequality **PASS**; non-negative margin **FAIL** — at 0.1 m clearance and 2 m/s request the executed speed is 2.0 m/s and the margin **−0.8755 m**; 1.3255 m is needed at the 2 m/s floor. The existing production mode `stopcap` keeps the margin ≥ 0 on the same grid (**PASS**), but Policy F was never trained or evaluated with it |

### EQ-1C — perception and experimental contract

| ID | Test | Result |
| --- | --- | --- |
| C1a | Static firewall (AST of `_process_obs_perception`) | **PASS** — one `observe` call, 13 keyword inputs, none reads target/critic state; the actor tensor is written only from its result; the target pose reaches only `render_raw_rgbd` |
| C1b | Dynamic firewall, production `NavRLPerceptionModule` on CPU, 40 steps × 4 envs | **PASS** — 898-D actor rows bit-identical across fresh instances, changed torch/NumPy/random RNG, undone permutation and a mid-sequence reset; env-0 rows bit-identical for batch sizes 1/2/4/128; positive control with perturbations on diverges. Task-level privileged-state mutation with fixed rendered sensors → EQ-1D |
| C2 | Checkpoint and normalisation state | **PASS** — SHA-256 `f7022139…` matches; 898-D RunningMeanStd digest `642fc9f7…`, count 409,600,001; eval-mode forward × 1000 leaves it unchanged, train mode changes it; rl_games player calls `model.eval()` and never `train()`. The actual Runner was not executed (Isaac Gym) |
| T1 | Kalman predict/correct against the hand-derived filter | **PASS** — worst relative error 5.4e-7 over 40 steps |
| T2 | 5 s expiry after 0/49/50/51/60 misses | **PASS** — active, active, active (float32 age 4.9999976), expired, expired: expiry on the 51st miss |
| T3 | Identity-bound clock (camera lost, compatible bar on the LiDAR) | Literal contract (time since last accepted fused update) **PASS**; identity-bound **FAIL** — 100/100 accepts, track alive after 10 s, age 0 |
| T4 | Range-only information versus `H = uᵀ` | Policy F **FAIL** — one return cuts tangential variance 1.0 → 0.022 and vertical 1.0 → 0.038 (world-axis diagonal R on a point whose bearing and height are the track's own prediction; at 90° bearing the range axis gets 0.15² instead of 0.08²). Existing opt-in `NAVRL_LIDAR_RANGE_ONLY_UPDATE=1`: **PASS** (0.9996) |
| C4 | Loss-type separation (2,261,590 post-acquisition quota frames) | Camera lost in 438,089 (19.4 %): FOV exit 304,721 (69.6 %), geometric centre occlusion in view 133,305 (30.4 %), in-view clear camera loss 63 (60 within, 3 beyond the scale band). Sensor dropout NOT_ACTIVE. LiDAR-only fused frames 187,114 = 42.7 % of camera-lost frames (range association; identity loss **NOT_RECORDED**). Camera valid while centre-occluded 33,109; camera valid while the centre is outside the frustum 279,596 (edge views with ≥ 2 pixels): the centre-based FOV flag also needs a silhouette label |
| C5 | Quota versus legacy populations | Both rules reproduced independently in 112/112 cells (cutoff steps equal). Membership differs in **112/112** cells; outcome counts differ in **96/112** (EQ-0's number). Same-population claim **FAIL**; OCC-P2 analysis uses the quota population and is unaffected |
| C6 | Evaluation contract for collision-free capture / crash / timeout / safe abort | Design written ([EVALUATION_CONTRACT.md](EVALUATION_CONTRACT.md)); measurement NOT_TESTED |

## Qualification matrix (EQ-0 rows updated by EQ-1)

| EQ-0 row | Property | EQ-0 | After EQ-1 | Evidence |
| --- | --- | --- | --- | --- |
| G1.4 | Actor has independent Z authority | FAIL | FAIL (unchanged; not retested) | EQ-0 |
| G1.5 | Active RK4 routine is classical RK4 | FAIL | **FAIL** (M1/M2); fixed-step composite within 0.1 % of the intended lag; two corrections preserve it | M1–M3 |
| G1.7 | Admitted scenes globally connected, reserve-passable, dynamically feasible | NOT_TESTED | Kinematic body tier **PASS** (57,344/57,344); reserve tier **LIMITED_VALIDITY** (4 unresolved); dynamic **NOT_TESTED** | B4 |
| G1.8 | Mass/inertia/motor/drag/contact calibrated | NOT_TESTED | NOT_TESTED; URDF: collision box bounds motors, no rotor geometry | B3 |
| G1.9 | Timestep/contact/capture convergence | NOT_TESTED | Motor sub-model order 3 (M2); full plant **NOT_TESTED** → EQ-1D | M2 |
| — | Speed/tilt state bounds | — | **FAIL** as hard bounds (tilt > 45° in 0.041 % of normal frames, max 91°) | B2 |
| G2.3 | Camera depth → LiDAR fusion pinhole-consistent, common origin | FAIL | **FAIL** (A1 5.8164 px / 3.4109°, A2 −0.0998 m); correction PASS | A1, A2 |
| — | Vertical aggregation | — | **LIMITED_VALIDITY** (conservative only) | A3 |
| G2.4 | Pixel count proves full silhouette obstruction | NOT_TESTED | **FAIL** (A5 counterexamples; scale band 12.75–23 m) | A5 |
| G2.6/2.7 | Radius conventions / identity | PASS / NOT_TESTED | **FAIL** −0.050 m LiDAR, +0.0445 … +0.0585 m camera (A7); identity FAIL | A4, A6, A7 |
| — | Camera target direction (intrinsics, extrinsic, vertical sign) | — | **PASS** | A7 |
| G3.4 | Expiry contract | PASS | **PASS** (51st miss; float32 checked) | T2 |
| — | Identity-bound expiry clock | — | **FAIL** | T3 |
| — | Range-only covariance | — | **FAIL** (Policy F); opt-in flag PASS | T4 |
| G3.7/3.8 | GT firewall / dynamic GT + RMS invariance | PASS / NOT_TESTED | Module-level dynamic **PASS**, RMS **PASS**; task-level **NOT_TESTED** → EQ-1D | C1, C2 |
| G3.11 | Riskcap enforces stopping margin | FAIL | **FAIL** (−0.8755 m); `stopcap` PASS | B5 |
| G3.12 | Safe-abort state and endpoint | NOT_TESTED | NOT_TESTED; contract designed | C6 |
| G4.7 | Legacy diagnostics and quota share one population | FAIL | **FAIL** (membership 112/112, counts 96/112) | C5 |
| — | Loss-type separation | — | **LIMITED_VALIDITY** — FOV / geometric / sensor separated; identity NOT_RECORDED | C4 |

Overall EQ-1 verdict per domain stays **LIMITED_VALIDITY**: the frozen loop is internally consistent
and its historical results stand as whole-system results, but calibrated metric fusion, identity,
silhouette-level occlusion, hard safety bounds and plant convergence are not qualified.

## Parts of the existing environment that must be fixed before the corresponding claims

Each fix belongs in a separate opt-in lineage (default off, Policy F and OCC unchanged).

1. **Camera → LiDAR fusion** (A1, A2): pinhole column mapping and SE(3) reprojection into the body
   frame. This changes the 288-D fused scan and the 480-D obstacle tokens the actor reads.
2. **Radius conventions** (A4, A7): one target radius across camera, LiDAR proxy and association, or
   an explicit transform; a camera centre-range estimator without the mean-depth bias.
3. **Occlusion labels** (A5, C4): target-alone support and a silhouette-based FOV label in the
   evaluator; zero pixels never stand for full occlusion.
4. **Identity** (A6, T3): association provenance in evaluator-only output; a clock bound to
   identity-bearing observations if persistent identity is claimed; otherwise call it range association.
5. **Range-only covariance** (T4): the existing opt-in `NAVRL_LIDAR_RANGE_ONLY_UPDATE=1` (it changes the
   80-D target token).
6. **Motor integrator** (M1–M3): an exact ZOH lag at τ = 0.04 s (−0.091 % gain change at dt 0.01 s).
   Do not just flip the sign: that is a 0.050 s motor.
7. **Braking law** (B5): a law with no speed floor inside the stopping distance (`stopcap` exists) if a
   safety guarantee is claimed.
8. **Evaluation contract** (C5, C6): one joined quota population for every metric; separate
   collision-free capture, contact-overlapping capture, crash, timeout and safe abort.

## Research scope possible without changing the environment

With the frozen Policy F stack and its current sensors, still stated as whole-system results:

- Frequency, duration and reacquisition of **centre-ray geometric occlusion**, FOV exit and
  LiDAR-only continuation by density and target motion (OCC-P1/P2 type), with loss types kept apart.
- Kinematic scene feasibility claims: every recorded scene is passable for the body (B4).
- Crash/timeout/capture comparisons on the quota population, with crashes described as occurring
  at the riskcap floor speed.
- Evaluation-only additions that are proven non-interfering by a G0-type test (target-alone support
  counts, association provenance, inner-step contact flags, safe-abort labels). They add
  observability without changing the actor input or the plant.

Not possible without a change: calibrated metric camera/LiDAR geometry, persistent target identity,
realistic redetection, silhouette-level occlusion ground truth, hard speed/tilt or stopping
guarantees, plant convergence/real-UAV transfer, and constructed 5 s geometric-loss benchmarks.

## Minimal new sensor/plant lineage proposal (not implemented)

| Order | Lineage (opt-in flag, default off) | Changes | Actor input | Policy F reuse | Qualification |
| --- | --- | --- | --- | --- | --- |
| 0 | `E-L0` evaluation telemetry | target-alone support, silhouette FOV, association provenance, inner-step contact, safe abort | none | yes | G0-type ON/OFF bitwise non-interference + EQ-1A labels |
| 1 | `P-L1` motor ZOH (τ = 0.04 s) | motor update only | distribution only (−0.091 % gain) | evaluate as-is, report the difference | M1–M3 gates; EQ-1D dt convergence |
| 2 | `S-L1` calibrated fusion | A1/A2 pinhole + SE(3), A4 radius, T4 range-only covariance | 288-D scan, 480-D tokens, 80-D target token change | out-of-distribution: measure, then decide on retraining | EQ-1A gates on the corrected code; EQ-1D sensor scenes |
| 3 | `G-L1` stopping-safe governor | `stopcap` or equivalent | none (closed loop changes) | new closed-loop baseline | B5 grid; EQ-1D braking/contact fixtures |

Each lineage keeps the frozen baseline side by side. A combined lineage is defined only after each
part has passed its own gates.

## Limitations

- B2 uses 10 Hz consumed states: the terminating step, inner-step contact and post-contact motion
  are not recorded. B4 uses the target's first recorded position (moving routes not checked) and
  holonomic motion. C1b is module-level; the task-level intervention needs the renderer (EQ-1D).
  C2 used rl_games' own RunningMeanStd and the player source, not a Runner execution.
- Proposed corrections were run only as reference code on the fixtures above; they are not
  implementations and have not been tested in the simulator.
- During development the first full PUBLIC_CPU run showed 10 failures / 21 errors caused by this
  package's own helper leaking stub `aerial_gym` modules and receipt variables into later tests. The
  helper now restores `sys.modules` and `os.environ` (tested); the final run passed 2,392/2,392.
