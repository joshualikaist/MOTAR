# 07 — BOIA v1: full mathematical specification (recommended candidate B)

**Status: RESEARCH_DESIGN, NOT_IMPLEMENTED, NOT_VALIDATED.**

Every formula carries a tag:
- **[PAPER]:** as published, with the page and equation.
- **[ADAPTED]:** changed for MOTAR.
- **[NEW]:** proposed here.

The defaults in §14 are starting values only. They must be fixed in a preregistration before any
data that could tune them is seen. None is tuned on pilot outcomes.

## 0. Notation

**Indices.**
- k: decision index (Δt = 0.1 s).
- j = 1…H: rollout step (H = 10, so 1.0 s), with substeps s = 1…S (S = 10, δ = 0.01 s).
- e: environment (N parallel); vectors are per env unless shown.
- m = 1…M: target-belief samples.
- γ ∈ Γ: candidate.

**Ego and target.**
- Ego state: position p (3), velocity v (3), yaw ψ.
- Target belief: μ = [μ_p; μ_v] (6) and Σ (6×6), read from `perception.tracker.state`/`.cov`.

**Frames.**
- W: world, z up.
- V: yaw-only vehicle frame (camera and LiDAR).
- R_V(ψ): rotation from V to W.

## 1. Information boundary (hard rule)

**Allowed online inputs** (M1 §10.1; read-only):

| Symbol | Source attribute | Shape |
| --- | --- | --- |
| p, v | `obs_dict["robot_position"]`, `["robot_linvel"]` | (N,3) |
| q | `obs_dict["robot_vehicle_orientation"]` | (N,4) |
| ρ | `task._lidar_distance_m()` | (N,288) = 4 rows × 72 bins, metres |
| μ, Σ, active, age | `perception.tracker.{state,cov,active,age}` | (N,6), (N,6,6), (N,), (N,) |
| c | `task._camera_visible_now` | (N,) bool |
| a_F | Policy F action | (N,4) |
| — | Calibration constants (FOV, intrinsics, `lidar_max_range` = 12, `max_camera_range` = 20, `target_radius` = 0.15, elevations [20, 10, 0, −10]°) | — |

**Never read.**
- GT target state or its aliases;
- semantics, masks, `DETECT_CHANNEL`;
- obstacle positions;
- E-L0, KROC or forensics outputs;
- `task_obs["states"]`.

**Enforcement.** A whitelist accessor (08, module `access`) plus a D3-style mutation firewall test:
mutating every forbidden tensor must leave the BOIA output bitwise unchanged.

## 2. Mode machine [NEW]

Per env, the mode is evaluated **after** the terminal check (§11).

| Mode | Entry condition (online only) | BOIA behaviour |
| --- | --- | --- |
| TERMINAL | The simulator flagged terminated or truncated at the last step, i.e. the current obs is post-reset | No action. Reset the internal state; Policy F acts |
| NEVER_ACQUIRED | No camera detection since reset | Safety screen only (§7). v1 does not search |
| TRACK_VISIBLE | c = 1 and ‖μ_p − p‖ ≥ d_int | Evaluate. Intervene only on safety override |
| PRE_OCCLUSION | c = 1 and P_det forecast under γ_F over the next H_pre = 5 steps is below p_pre | Evaluate; arbitration enabled |
| LOST_PREDICTABLE | c = 0, active, tr(Σ_pp) ≤ σ²_lost, camera-miss streak ≤ n_lost | Evaluate; arbitration enabled |
| LOST_UNLOCALIZED | c = 0 and (not active, or tr(Σ_pp) > σ²_lost, or streak > n_lost) | Safety screen only. v1 does not search; that is the hidden-start extension |
| INTERCEPT | c = 1 and ‖μ_p − p‖ < d_int | Safety override only; Policy F flies the endgame |
| SAFE_HOLD | No feasible candidate exists (including γ_F) | Maximum-deceleration command (§6.4) |

The camera-miss streak is counted by BOIA from c. It uses camera detections only, because LiDAR
resets `age` (M1 §5).

## 3. Target belief

### 3.1 Read [as built]

μ and Σ come from MOTAR's CV tracker (`navrl_perception.py:640-723`):
- F = [[I, Δt I], [0, I]];
- diagonal Q (q_pos = 2.5e-4 m², q_vel = 0.025 (m/s)² per step);
- Joseph-form update;
- no camera gate.

BOIA never mutates the tracker.

### 3.2 Shadow prediction for the horizon [PAPER: UAST Eq.4–5, BPOD Eq.4]

μ_{j+1} = F μ_j and Σ⁻_{j+1} = F Σ_j Fᵀ + Q_b, where Q_b = s_Q · Q_tracker and s_Q is calibrated
(§13, V3).

### 3.3 Expected update under forecast detection [PAPER: BPOD p.3 Eq.11d; ADAPTED]

Σ_{j+1} = Σ⁻_{j+1} − P̄_det,j+1 · K̃ H Σ⁻_{j+1},

with:
- H = [I₃ 0];
- K̃ = Σ⁻Hᵀ(HΣ⁻Hᵀ + R)⁻¹;
- R the camera measurement covariance **as MOTAR builds it** (diagonal in world axes; M1 finding);
- P̄_det from §5.

The mean is unchanged: μ_{j+1} = Fμ_j. This expected-covariance form assumes the gain is
independent of the realised sample (BPOD p.3). It is an approximation and is checked in V3.

### 3.4 Belief samples [ADAPTED]

For m = 1…M:
- x^{(m)}_0 ∼ N(μ, Σ);
- x^{(m)}_{j+1} = F x^{(m)}_j + w^{(m)}_j, with w ∼ N(0, Q_b).

**Validity.**
- **Gaussian, constant velocity.** The E2 target follows piecewise waypoint motion that is obstacle
  aware, so CV is a local approximation. V3 measures, on recorded traces, the evaluator-only NEES of
  the shadow prediction at horizons 0.5 s and 1.0 s.
- **Fallback.** If NEES at 1.0 s exceeds the χ² 95 % bound in more than 20 % of cases, Q_b is
  inflated by s_Q. If that is still insufficient, the horizon is cut to 0.5 s and the shortening is
  recorded.

## 4. Sensed obstacle model [ADAPTED from H-R3 arms, FASTER known/unknown split]

### 4.1 Points

Each LiDAR return becomes a horizontal point. For row r, bin b, range ρ_rb < 0.995·12 m:

o_rb = p + R_V(ψ)·[ρ_rb cos ε_r cos α_b, ρ_rb cos ε_r sin α_b]ᵀ.

Only the horizontal (x, y) components are kept, because the bars are vertical (M2 §10).

### 4.2 Target exclusion

A return is excluded from the obstacle set when its horizontal Mahalanobis distance to the belief
satisfies

(o − μ_p,xy)ᵀ(Σ_pp,xy + r²_T I)⁻¹(o − μ_p,xy) ≤ χ²₂(0.99) = 9.21.

This makes the target not occlude itself (M1 risk 6). The exclusion uses the belief, never semantics.

### 4.3 Known-free and unknown

For each bin b, the known-free set is the sector wedge up to d_b = min_r ρ_rb − m_free. Beyond d_b,
and everywhere beyond 12 m, is **unknown**. Unknown is never free.

The angular gap between beams is 5°, giving 0.44 m between adjacent rays at 5 m. Points between two
known-free rays are counted free only within d_free,b = min(d_b, d_{b±1}). This is conservative.

### 4.4 Occluder inflation

A point blocks a line of sight when its distance to the segment is below r_occ = r_bar_est + r_T:
- r_bar_est is half the bar width, from the M2 §10 geometry, declared as a prior;
- r_T = 0.15 m.

## 5. Visibility forecast [ADAPTED from BPOD Eq.9/13–15; NEW joint form]

For candidate γ, rollout step j and sample m:
- **FOV indicator.** f^{(m)}_j = 1 if the vector x^{(m)}_j − p_γ,j, expressed in V at the rolled-out
  yaw ψ_γ,j, lies inside the declared horizontal and vertical FOV, and its range is ≤ 20 m.
- **Line-of-sight indicator.** l^{(m)}_j = 1 if every non-excluded obstacle point o satisfies
  dist(o, segment[p_γ,j, x^{(m)}_j]) ≥ r_occ.
  - The test is horizontal (2D).
  - The segment's pass through **unknown** space is scored conservatively as blocked (default
    `unknown_blocks = 1`); the opposite setting is an ablation.
- **Detector reliability.** p^{(m)}_j = p_det(n_px(x^{(m)}_j, p_γ,j); θ_det) · f · l, with apparent
  size n_px = π (f_x r_T / d)² pixels.
  - **Oracle setting** (MOTAR as built, M1 §2.2): p_det = 1{n_px ≥ 2}.
  - **Stressed setting:** p_det = (1 − p_miss) · σ(β(n_px − n_0)).

  Partial occlusion is handled by l only, because v1 tests the centre ray. A visible-fraction factor
  is a v2 option, calibrated against E-L0 labels **offline only**.

**Joint horizon quantities, computed from the same samples** (no product over time across different
samples; BPOD's Eq.15 independence is avoided):
- P̄_det,j(γ) = (1/M) Σ_m p^{(m)}_j;
- P_any(γ) = (1/M) Σ_m [1 − Π_j (1 − p^{(m)}_j)];
- P_loss(γ) = 1 − P_any(γ).

The within-path product assumes detector errors are conditionally independent across time given the
geometry. That holds exactly for the oracle detector, whose p is 0 or 1. Under the stressed detector
it is an assumption, and V2 records it.

**Fast-path ablation [PAPER: BPOD Eq.15, 19].** γ_BPOD,j = γ_FOV · Π_i γ_LOS,i is computed with erf
on linearised signed distances to point-discs. It is compared with the Monte Carlo value in V5,
measuring the bias of the independence assumption for adjacent bars.

## 6. Response model and candidate rollout [ADAPTED from M2 assumptions A1–A8; to be calibrated]

### 6.1 Command interpretation (code facts)

- Policy F action a = [a_x, a_y, a_z, a_ψ] gives a vehicle-frame velocity command
  u_V = 2.5·clip(a_xy, −1, 1).
- The yaw-rate command is ω = 3.0·clip(a_ψ, −1, 1).
- a_z is overwritten by the altitude PI, so z is assumed held.

### 6.2 Governor

The exact riskcap law (`speed_governor.py:466-475`) is reproduced as a pure function of the decision
scan and the command. In the rollout it is re-evaluated at each step j against the point model from
the predicted pose [ADAPTED]. The governor only scales speed (M2 §3).

### 6.3 Plant

Within each substep:
- v ← v + (δ/τ_v)·sat_a(R_V(ψ) ū − v), with ‖accel‖ ≤ a_max = g·tan 45° and ū the governed command;
- ψ ← ψ + δ·ω_eff, with ω_eff ← ω_eff + (δ/τ_ψ)(ω − ω_eff);
- p ← p + δ·v.

The command is **re-rotated by the current ψ every substep** (M2 A2).

### 6.4 Parameters and safe hold

- θ_dyn = (τ_v, τ_ψ, a_max). Prior: τ_v = 0.4 s is derived, τ_ψ is unknown. All are calibrated in V4
  on recorded pre-contact traces.
- **SAFE_HOLD command:** ū = 0 with yaw toward the belief bearing. The modelled stop follows §7.2.

**Validity.** The model is valid pre-contact only. The V4 tolerance is that the 95th-percentile
position error at 1.0 s is ≤ 0.15 m and the speed error is ≤ 0.2 m/s. If either is exceeded, H is
cut or the design returns to G3.

## 7. Safety screen [ADAPTED from FASTER p.4; NEW coupling]

A candidate is feasible only if both tests pass.

### 7.1 Swept clearance (two-tier)

**Why two tiers.** The Policy F layouts (`navrl_band`) intentionally contain passable gaps of only
0.469–0.592 m, against a 0.396 m body diagonal (M2 §10). A hard margin of 0.2 m on each side would
make those gaps infeasible. Policy F would then be overridden almost everywhere, and capture would
collapse. The screen is therefore split.

- **Hard (reject).** For all substeps, min_o ‖p_γ − o‖ ≥ r_hard + ‖v_γ‖·t_det, and p_γ lies in
  known-free space (§4.3).
  - r_hard = 0.14 m, the body half-width, from the 0.28 × 0.28 m collision box.
  - t_det = 0.09 s, the measured late contact flag.

  This rejects predicted contact only.
- **Soft (cost).** The clearance below r_body + m_s, with r_body = 0.198 m (circumradius) and
  m_s = 0.20 m, enters the cost through C(γ) (§9).

**Calibration gate.** On CPU replay of Policy F traces (V5), the hard screen must flag γ_F as
infeasible in **≤ 2 % of non-crash decisions**, and it must flag the pre-crash decisions at a recorded
rate.

If the false-alarm rate is exceeded, r_hard and t_det are revisited before the preregistration and
the change is recorded. They are never revisited after.

The bar width (r_bar_est in §4.4) is **not stated in the audited records**. It must be read from
`env_object_config.py` (`bars_h3`) in G4 and declared.

### 7.2 Terminal stopping feasibility

From the rollout end state (p_H, v_H), the stopping distance is

d_stop = ‖v_H‖·t_react + D_brake(‖v_H‖),

where D_brake interpolates the measured braking table (0.82, 1.05, 1.57 m at 2.0, 2.5, 3.52 m/s;
extrapolated as v²/(2â) below 2 m/s, with â fitted to the table) and t_react = 0.1 s. The straight
stop along v_H must remain in known-free space with the hard clearance r_hard.

This is a FASTER-style condition, the guarantee that "the committed plan ends at rest in known-free
space". It is **empirical only**, because unknown space, sensor errors and model error all violate
the formal premises.

**Safety override.** If γ_F is infeasible, BOIA intervenes regardless of hysteresis. If no candidate
is feasible, the mode becomes SAFE_HOLD.

## 8. Candidate set Γ (K = 20) [ADAPTED from YOPO lattice, Eva arc, H-R3 offsets]

| Family | Count | Commands, held for the horizon |
| --- | ---: | --- |
| γ_F | 1 | Policy F's current action |
| γ_brake | 1 | ū = 0, ω toward the belief bearing |
| Lattice | 15 | Heading offsets {−40°, −20°, 0, +20°, +40°} from the belief bearing × speed {0.6, 1.2, 2.0} m/s, ω toward the belief bearing (clamped) |
| Arc viewpoints | 2 | Left and right lateral offsets δ_L, δ_R: the smallest offsets with a line of sight clear against points (Eva p.3 idea; H-R3 geometry), with pursuit at 1.0 m/s |
| Yaw-only | 1 | ū = 0.6 m/s along the current velocity heading, ω toward the belief bearing |

All commands are clipped to the action box before the rollout.

## 9. Cost [NEW]

For a feasible γ:

J(γ) = w_c Φ(γ) + w_l P_loss(γ) + w_u U(γ) + w_s S(γ) + w_r C(γ).

The terms:
- **Φ, capture progress:** Φ = [E_m‖x^{(m)}_H − p_γ,H‖ − ‖μ_p − p‖]/(v_ref H Δt), with
  v_ref = 2.0 m/s. It is negative when closing.
- **U, uncertainty:** U = ½ log det(Σ_pp,H(γ)) − ½ log det(Σ_pp,0), using §3.3.
- **S, smoothness:** S = ‖a_γ − a_prev‖² / 4.
- **C, clearance risk:** C = (1/H) Σ_j relu(d_soft − clr_γ,j)/d_soft, with d_soft = 1.0 m. It is a
  soft term beyond the hard screen.

Mode-dependent weights are in §14.

**Double counting.** P_loss and U are correlated: both fall when detection is likely. The V5 ablations
remove each in turn. If one adds nothing, it is dropped before the preregistration.

## 10. Arbitration [NEW]

**Expected advantage.** Δ_k = J(γ_F) − min_{γ∈Γ_feas} J(γ).

**Engage, with the minimum-cost candidate γ*,** when either holds:
- (safety override) γ_F ∉ Γ_feas; or
- mode ∈ {PRE_OCCLUSION, LOST_PREDICTABLE} and Δ_k > τ_on.

**Stay engaged** for at least n_min steps.

**Disengage** when one of these holds:
- Δ < τ_off < τ_on for n_off consecutive steps;
- the mode becomes TRACK_VISIBLE or INTERCEPT, and γ_F is feasible;
- TERMINAL.

**Re-planning.** It is receding-horizon: only the first 0.1 s of γ* is executed, and everything is
re-evaluated at k + 1.

**Calibration of τ_on.**
- On CPU replay of recorded Policy F traces (V5), predicted Δ is compared with the realised
  difference in the cost terms that can be evaluated from data.
- τ_on is set at the smallest value whose reliability curve shows non-negative realised advantage
  for 90 % of engagements.
- It is then **frozen in the preregistration**. It is not tuned on GPU pilot outcomes.

**Hand-back.** On disengage, Policy F resumes. Its `prev_action` history contains the BOIA actions.
This out-of-distribution exposure is measured: actor-input drift and Policy F action change after
hand-back are logged.

## 11. Terminal-first handling [NEW integration rule]

At every decision, before anything else, for every env e:
1. **Check the terminal flags.** If the simulator reported terminated or truncated for e at the
   previous step (crash, capture, timeout, OOB), the current obs belongs to the next episode.
   - Clear BOIA state for e: dwell counters, previous command, miss streak, engaged flag.
   - Set the mode to TERMINAL (or NEVER_ACQUIRED for the new episode).
   - Do not intervene this step.
2. **Never declare an outcome.** BOIA never declares capture or crash, and never uses predicted
   closeness as an outcome. Outcomes come only from the simulator flags.
3. **Log only within the episode.** Logging and evaluation use pre-terminal observations only. A
   terminal-step observation is post-reset; the H-R3 P01 defect is the precedent.

## 12. Pseudocode

```text
for each decision k (all envs batched):
  T = terminal_flags_from_previous_step()            # (N,) bool
  state[T] = reset_state(); mode[T] = TERMINAL
  inp = access.read_whitelisted(task)                # firewall-checked
  mode = update_modes(mode, inp, state)              # §2; uses forecast under γ_F if needed
  a_F = inp.policy_action
  act = a_F.clone()
  live = ~T & (mode != NEVER_ACQUIRED | screen_needed)
  pts, free = obstacles.build(inp.lidar, inp.pose, inp.belief)        # §4
  Γ = candidates.generate(a_F, inp.pose, inp.belief, pts)             # §8  (N,K,4)
  roll = dynamics.rollout(Γ, inp.pose, inp.vel, pts, θ_dyn)           # §6  (N,K,H,·)
  feas = safety.screen(roll, pts, free)                               # §7  (N,K) bool
  samp = belief.sample(inp.belief, M, H, Q_b)                         # §3  (N,M,H,6)
  pdet = visibility.forecast(roll, samp, pts, free, θ_det)            # §5  (N,K,M,H)
  Σ̃   = belief.expected_cov(inp.belief, pdet.mean(M), R)              # §3.3
  J    = cost.evaluate(roll, samp, pdet, Σ̃, a_prev, mode, weights)    # §9  (N,K)
  act, engaged, reason = arbitration.decide(J, feas, a_F, mode, state, τ)   # §10
  act[no feasible] = fallback.safe_hold(inp)                          # §6.4
  trace.log(online inputs digest, Γ, feas, J, Δ, act, mode, reason, post-step realised)
  return act      # → existing governor → altitude hold → Lee controller → PhysX
```

## 13. Compute budget

Per env and decision:
- rollout: K × H × S = 2,000 substeps (trivial);
- line-of-sight tests: K × M × H × P = 20 × 32 × 10 × 288 ≈ 1.84 × 10⁶ point–segment distances.

For N = 128 that is about 2.4 × 10⁸ distance evaluations per decision. They must run as batched GPU
tensors with candidate culling (points outside the segment's bounding box are skipped). The target
is ≤ 5 ms per decision on the RTX 3070, and the budget is measured in V5/V6.

If it is exceeded, these are cut in order, and each cut is recorded:
1. M from 32 to 16;
2. K from 20 to 12;
3. switch to the BPOD analytic fast path.

## 14. Parameters (defaults, to be fixed in the preregistration)

| Parameter | Default | Basis |
| --- | --- | --- |
| H, Δt, S | 10, 0.1 s, 10 | Decision rate (M2 §1); about 1 s, matching the measured stopping time |
| M | 32 | Compute budget |
| d_int | 3.0 m | Endgame hand-off; Policy F captures well when the target is visible |
| p_pre, H_pre | 0.5, 5 | PRE_OCCLUSION trigger |
| σ²_lost, n_lost | 1.0 m², 20 steps | LOST_PREDICTABLE bounds |
| r_hard, r_body, m_s, t_det, t_react | 0.14 m (hard), 0.198 m and 0.20 m (soft), 0.09 s, 0.1 s | Collision box 0.28 × 0.28 m, measured detection lag (M2 A8), gaps of 0.47–0.59 m (M2 §10); hard false-alarm gate in §7.1 |
| m_free | 0.1 m | Free-space margin |
| r_occ | r_bar_est + 0.15 m | §4.4 |
| s_Q | 1.0, calibrated in V3 | §3 |
| θ_dyn | τ_v 0.4 s (prior), τ_ψ TBD, a_max 9.8 m/s² | §6, calibrated in V4 |
| Weights: TRACK_VISIBLE | w_c 1.0, w_l 0.2, w_u 0.1, w_s 0.05, w_r 0.5 | Capture-dominant when the target is seen |
| Weights: PRE_OCCLUSION and LOST_PREDICTABLE | w_c 1.0, w_l 1.0, w_u 0.3, w_s 0.05, w_r 1.0 | Loss and clearance matter more |
| τ_on, τ_off, n_min, n_off | Calibrated in V5; τ_off = 0.5 τ_on; 5; 3 | Hysteresis |
| p_det oracle setting | 1{n_px ≥ 2} | MOTAR detector (M1 §2.1) |
| p_det stressed setting | p_miss ∈ {0.1, 0.3}, β = 1, n_0 = 4 | NavRL++ dropout of 0.3 as a reference point (p.6–7) |

## 15. Assumptions and their checks

| Assumption | Holds when | Checked by |
| --- | --- | --- |
| CV Gaussian target belief over 1 s | E2 waypoint legs are long compared with 1 s | V3: NEES and NIS on recorded traces (NEES is evaluator-only) |
| Expected-covariance approximation (BPOD Eq.11) | The gain is weakly dependent on the sample | V3: compared with a Monte Carlo posterior covariance |
| 2D line of sight against vertical bars | Altitude about 1 m, target near the same height | V2: compared with E-L0 3D silhouettes offline |
| Conditional independence of detector errors across time | Exact for the oracle detector | V2 for the stressed setting |
| First-order velocity response with yaw re-rotation | Pre-contact, tilt ≤ 45° | V4 |
| Governor determinism | Exact; it is a pure function of the scan | Unit test against `speed_governor.py` |
| LiDAR known-free set is free | No obstacle between beams that is closer than the returns | V2: compared with the bar layout offline (evaluator) |
| Perfect odometry | A simulation idealisation; not removed | Declared limit |
| Oracle detector | MOTAR as built | Detector-realism factor in 09 |
