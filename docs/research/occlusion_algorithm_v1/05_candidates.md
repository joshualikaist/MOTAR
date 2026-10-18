# 05 — Three competing algorithm designs

## Shared contract

These rules apply to all three candidates (from [`04_motar_audit.md`](04_motar_audit.md)):
- **Policy F stays frozen.** The 898-D actor and its observation schema are untouched.
- **Opt-in layer.** Each candidate is a module under `motar_eval/` that may replace Policy F's
  4-D action, a = [vx/2.5, vy/2.5, (overwritten), ψ̇/3] in the yaw-only vehicle frame.
- **Execution path.** The replacement passes through the unchanged riskcap governor, altitude hold
  and Lee velocity controller.
- **Decision rate.** 10 Hz (0.1 s ZOH).
- **Inputs.** Only the M1 §10 whitelist. No GT, E-L0 output, semantics or renderer mask.

**Frames.**
- **W:** world, z up.
- **V:** yaw-only vehicle frame, the camera and LiDAR frame (x forward, y left).
- **B:** body, unused by the planner.

**Shapes are per decision for N parallel envs.** Raw LiDAR is (N, 4, 72), with elevations
[20, 10, 0, −10]°.

## A — Reactive visibility recovery (reference baseline; prior art)

**Origin.** H-R3 arm B3 (`motar_eval/h_r3_arms.py`), plus Eva-Tracker's arc viewpoint (p.3) and
UAST's "temporarily invisible" mode (p.4).

**Trigger.** In the first episode after acquisition, the camera flag goes true → false. The trigger
uses the online flag only.

**Window.** At most W_A = 20 steps, ending early when the camera detects again or the env terminates.

**Action in the window.**
1. Compute the belief-relative vector r = R_V^T(μ_T − p_D), where μ_T is `tracker.state[:, :3]` (N, 3).
2. Build the pursuit direction u = r/‖r‖ and the lateral normal n.
3. Choose the candidate lateral offsets δ ∈ {−2.0, −1.75, …, 2.0} m, 17 values.
4. Keep only the offsets whose line of sight from p_D + δn to μ_T clears every LiDAR point that lies
   farther than 0.5 m from the belief.
5. Pick δ* = argmin |δ|.
6. Command v = v_p u + clamp(1.5 δ*, ±1.5) n, then apply the shared safety filter.
7. Set the yaw rate to clamp(2 atan2(r_y, r_x), ±3).

**Cost.** About 1.7 ms per decision for 128 envs, measured in H-R3 P01.

**Fallback.** The governor and the LiDAR approach filter.

**Stop condition.** End of the window, or terminal.

**Known evidence (H-R3).** It raises 2-second geometric re-observation over the scripted yaw-only
arm. It is not attributable to viewpoint, it lowers capture within 5 s, and it is exploratory only.

**Role.** Arm F2 in [`09_experiment_plan.md`](09_experiment_plan.md). It is a baseline, **not**
proposed as novel.

## B — BOIA: Belief-conditioned Occlusion/Interception Arbitration (recommended; spec in 07)

**Idea.** At every decision the module does six things:
1. It rolls out a finite set Γ of K ≈ 26 constant-command candidates for a horizon H = 10 steps
   (1.0 s). **Policy F's own current command** is one of them.
2. It uses a calibrated response and governor model for that rollout.
3. It screens each rollout for swept clearance and for stopping feasibility in known-free space.
4. For each feasible candidate it forecasts:
   - the joint probability of an identity-consistent camera detection, by Monte Carlo over the
     target belief and the sensed obstacle points;
   - the expected target covariance;
   - capture progress.
5. It computes the expected advantage of the best candidate over Policy F's command.
6. It **overrides Policy F only when that advantage exceeds a preregistered threshold**, with
   hysteresis and a minimum dwell.

A safety override (Policy F's command is infeasible) bypasses the hysteresis.

**Modes.**
- NEVER_ACQUIRED
- TRACK_VISIBLE
- PRE_OCCLUSION
- LOST_PREDICTABLE
- LOST_UNLOCALIZED
- INTERCEPT
- SAFE_HOLD
- TERMINAL

**Inputs and shapes.**
- Belief: μ (N, 6) and Σ (N, 6, 6).
- Scan: (N, 288) metres.
- Ego: p, v (N, 3), quaternion (N, 4).
- Policy F action (N, 4).
- Camera flag (N,).

**Output.** The action (N, 4) and a trace record.

**Rate and cost.**
- 10 Hz.
- K × H × M = 26 × 10 × 32 samples per env per decision.
- Target budget ≤ 5 ms for N = 128 on the RTX 3070. To be measured; not yet measured.

**Fallback.** SAFE_HOLD: the maximum-deceleration command toward v = 0, with yaw toward the belief.

**Stop conditions.**
- Terminal flag (terminal-first).
- No feasible candidate → SAFE_HOLD.
- A model-validity alarm, when the prediction error of the realised motion exceeds its tolerance →
  disengage and log.

**What it targets.** Crash within 2 s after a geometric-occlusion onset, with capture as a guard.
This is the measured failure. Re-observation is a secondary effect.

## C — YOPO-style learned candidate scoring (extension candidate)

**Idea.** Keep B's candidate set Γ, the hard safety screen and the arbitration, but replace the
hand-built cost J with a learned scorer.
- **Input:** per-candidate rollout features (clearances, P_det samples, Σ̃ trace, progress) plus a
  scan embedding.
- **Output:** a predicted cost (YOPO `yopo_trainer.py:174`; YOPOv2 p.7–9; UAST p.6 Eq.17).
- **Training target:** the realised outcome over the next 2 s (crash, capture progress, camera
  re-observation), from logged closed-loop data. This is offline supervised learning, not PPO.
  Policy F stays frozen.

**Shapes.** Features (N, K, F ≈ 40) → an MLP of 2 × 128 → (N, K) scores.

**Cost.** Below 1 ms on the GPU, but it needs a dataset (≥ 10⁵ decisions), a new checkpoint lineage
and its own preregistration.

**Fallback and stop conditions.** As in B. The learned score must never override the hard screen.

**Risks.**
- Distribution shift between logged data and closed loop.
- The scorer could learn oracle-detector artefacts.
- An 8 GB GPU shared with Isaac Gym limits training.
- Overlap with YOPO and UAST learned scoring reduces novelty.

**Role.** International-paper extension (G9), after B is validated.

## Selection

Scores run 1 (worst) to 5 (best). The reasons cite the evidence.

| Criterion | A reactive | B BOIA | C learned | Evidence |
| --- | :---: | :---: | :---: | --- |
| Targets the measured failure (crash under occlusion, capture guard) | 2 | 5 | 5 | M3 §5: crash 3.2–3.8 % within 2 s of onset; FOV re-observation 96–99 % (no headroom); A lowers capture within 5 s (H-R3) |
| Novelty against the 23 read papers | 1 | 3 | 2 | 06: A ≈ Eva/UAST/H-R3. B's combination of (a)–(d) is absent. C overlaps YOPO/UAST scoring |
| Feasibility for a domestic-conference MVP (CPU-first, no training) | 5 | 4 | 1 | B needs about 1.5 k lines of Python plus calibration. C needs data, training and a new lineage |
| Risk to Policy F's capture (intervention frequency) | 2 | 4 | 4 | A overrides on every loss. B and C override only on calibrated advantage |
| Testability on CPU before any GPU | 4 | 5 | 2 | B's modules are pure functions over recorded tensors |
| Dependence on the oracle detector | 3 | 3 | 1 | All three see it. C may learn it, while B's p_det can be stress-tested explicitly |
| **Total** | **17** | **24** | **15** | |

**Decision for G3: B (BOIA) is recommended, provisionally.**
- A is a **mandatory baseline**.
- C is the learned extension.

B must be redesigned or dropped if any of these hold:
- the outside-25 prior-art check (06 §3) finds an equivalent arbitration method;
- the response model cannot be calibrated within tolerance (gate V4);
- the CPU replay shows that P_det forecasts carry no information beyond the current camera flag
  (gate V5).
