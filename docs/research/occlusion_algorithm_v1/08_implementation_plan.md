# 08 — Implementation plan (opt-in, CPU-first)

**Status: PLAN.** Nothing here is approved for coding. G4 needs the user's approval of one unit at a
time; see [`12_next_approval.md`](12_next_approval.md).

## Rules for every unit

- **New files only,** under `motar_eval/boia/`, `tests/test_boia_*.py` and, later,
  `tools/boia_*.py`. No edit to production files:
  - `aerial_gym/**`;
  - `navrl_task.py` and `navrl_perception.py`;
  - `speed_governor.py`;
  - the frozen KROC adapter.
- **Pure functions on tensors.** No `isaacgym` import in `motar_eval/boia/`. Tests run under
  PUBLIC_CPU with `CUDA_VISIBLE_DEVICES=''`.
- **Small diffs.** One unit, about ≤ 500 lines plus tests, per approval.
- **Firewall.** Static import checks forbid GT attribute names. A mutation test is added in U2.
- **Governor.** The riskcap law is reused by calling the existing pure function in
  `aerial_gym/task/navrl_task/speed_governor.py`, read-only, or by mirroring it with a bitwise
  equality test. Its configuration is never changed.
- **Pollution.** Respect the shared PUBLIC_CPU process hazard: no process-wide environment variables
  or module-global switches.

## Modules

N = envs, K = candidates (20), H = horizon steps (10), S = substeps (10), M = samples (32),
P = LiDAR points (288).

| Module | Role | Key signatures (all `torch.Tensor`) | Output shapes |
| --- | --- | --- | --- |
| `types.py` | Frozen dataclasses and parameters | `BoiaParams` (all §14 defaults), `OnlineInputs`, `Rollout`, `Decision` | — |
| `access.py` | Whitelist reader + firewall | `read_online(task, perception, policy_action) -> OnlineInputs`; `FORBIDDEN_ATTRS` | `OnlineInputs` holding p, v (N,3); q (N,4); ρ (N,288); μ (N,6); Σ (N,6,6); active, age, c (N,); a_F (N,4) |
| `geometry.py` | Frames, LiDAR → points, segment distances | `lidar_points_xy(rho, angles, elev, pose) -> (N,P,2), valid (N,P)`; `exclude_target(pts, mu_xy, Sigma_xy, r_T, chi2)`; `seg_point_dist(a (N,…,2), b (N,…,2), pts (N,P,2)) -> (N,…,P)`; `known_free(rho, angles, pose, m_free)` | — |
| `belief.py` | Shadow prediction, samples, expected covariance | `predict(mu, Sigma, F, Qb, H) -> mus (N,H,6), Sigmas (N,H,6,6)`; `sample(mu, Sigma, Qb, M, H, generator) -> (N,M,H,6)`; `expected_cov(Sigma_pred, pdet_mean (N,K,H), R) -> (N,K,H,6,6)` | — |
| `dynamics.py` | Governor-aware first-order rollout | `rollout(cmd (N,K,4), p, v, psi, pts, theta_dyn, gov_cfg, H, S) -> Rollout` with positions (N,K,H,S,3), velocities, ψ; `braking_distance(speed, table)` | — |
| `candidates.py` | Γ generator | `generate(a_F, pose, mu, pts, params) -> cmd (N,K,4), family_id (K,)` | (N,20,4) |
| `safety.py` | Two-tier screen | `screen(rollout, pts, free, params) -> hard_ok (N,K), clr_min (N,K,H)`; `stopping_ok(rollout, free, table, params) -> (N,K)` | — |
| `visibility.py` | Forecasting P_det | `forecast(rollout, samples, pts, free, cam, params, mode="oracle"\|"stressed") -> p (N,K,M,H)`; `p_any(p) -> (N,K)`; `bpod_analytic(...)` (ablation) | — |
| `cost.py` | J terms | `evaluate(rollout, samples, p, Sigma_exp, a_prev, mode, weights) -> J (N,K), terms dict` | — |
| `arbitration.py` | Mode machine + hysteresis | `update_mode(state, inp, forecast_F) -> mode (N,)`; `decide(J, hard_ok, a_F, cmd, mode, state, params) -> action (N,4), engaged (N,), reason (N,)` | — |
| `fallback.py` | SAFE_HOLD | `safe_hold(inp) -> (N,4)` | — |
| `terminal.py` | Terminal-first guard | `apply(terminal_prev (N,), state) -> state` | — |
| `trace.py` | Online/evaluator separation in logs | `record(decision, inputs_digest)` for online-only fields; evaluator labels are joined offline by `tools/boia_replay.py` | — |
| `planner.py` | Orchestration | `BoiaPlanner(params).step(task, perception, a_F, terminal_prev) -> action, decision` | — |

## Staged units and CPU tests

Each unit's tests must pass before the next unit starts. Every failure condition listed is a test
assertion.

| Unit | Contents | CPU tests (pass criteria) | Failure conditions |
| --- | --- | --- | --- |
| **U1** | `types`, `geometry`, `visibility` (oracle mode only), with synthetic scenes | (1) `seg_point_dist` matches a brute-force reference to 1e-6. (2) A bar between ego and target gives P_det = 0, and a clear line of sight inside the FOV gives P_det = 1. (3) Target exclusion removes the self-return and never removes a bar farther than the gate. (4) An outside-FOV belief gives 0. (5) Monte Carlo P_any converges (M = 1024 against analytic, within 0.02 for a single-occluder case). (6) Two adjacent bars: Monte Carlo against the BPOD independence product shows the predicted bias direction. (7) Static import test: no forbidden attributes | Any GT name in the source; non-determinism under a fixed generator; NaN |
| **U2** | `access` + firewall | (1) The reader returns exactly the whitelisted shapes from a fake task object. (2) **Mutation firewall:** mutating `target_position`, `navrl_target_position`, `segmentation_pixels`, `target_mask`, `_targets` and `obstacle_position` on the fake task leaves `OnlineInputs` bitwise equal. (3) Reading never writes (all tensors compared before and after) | Any forbidden read; any mutation |
| **U3** | `belief` | (1) Prediction equals MOTAR's tracker `predict` for the same F and Q. (2) Expected covariance with P_det = 0 equals pure prediction, and with P_det = 1 equals the Joseph update to 1e-9. (3) The covariance never shrinks when P_det = 0. (4) Sample mean and covariance converge | Covariance shrink without forecast detection; not PSD |
| **U4** | `dynamics` + governor reuse | (1) The governor mirror is bitwise equal to `speed_governor` on random inputs. (2) The rollout reproduces the measured braking table within 10 % under the τ_v prior (a sanity check, not a calibration). (3) Yaw re-rotation: a constant vehicle-frame command under constant ω curves as expected. (4) No altitude authority | Mismatch with the governor; braking off by more than 25 % (a model-form error) |
| **U5** | `candidates`, `safety` | (1) K = 20 with γ_F always at index 0. (2) The hard screen rejects a rollout through a bar and accepts a 0.55 m gap at low speed. (3) Stopping feasibility rejects ending at 2 m/s within 0.8 m of a bar. (4) Unknown is never free | γ_F missing; unknown counted as free |
| **U6** | `cost`, `arbitration`, `fallback`, `terminal` | (1) With Δ below τ_on there is no override. (2) Safety override when γ_F is infeasible, regardless of dwell. (3) Hysteresis: no flapping on an alternating Δ sequence. (4) Terminal reset clears the state, and the post-reset step never acts. (5) INTERCEPT allows a safety override only | Flapping; acting on a terminal step |
| **U7** | `planner`, `trace`, `tools/boia_replay.py` | CPU replay on **recorded** Policy F traces (OCC-P2 / H-R3 raw, read-only): (1) the online-input digest matches the recorded one; (2) calibration reports for V3 (NEES, NIS), V5 (P_det reliability, Brier/ECE against E-L0 labels offline) and the hard-screen false-alarm rate; (3) timing per decision | Calibration gates in 09 §6 not met; then stop and return to G3 |

**Recorded traces for U7.** Existing raw traces may not hold everything the full scan needs. The
first U7 task is an inventory: if the raw lacks LiDAR scans or tracker covariance per step, U7 is
limited to what exists. A new recording run is a GPU step (G6) and needs approval.

## Integration (G5, after U1–U7)

An opt-in hook in a **new** launcher (`tools/boia_launcher.py`), following the `tools/h_r3_*`
pattern. It substitutes the action after `player.get_action` and before `env_step`, inside the
qualified isolation runner, with no change to the production task. PUBLIC_CPU must stay green, apart
from the pre-existing `motar-6nn.5` pollution failure, which is tracked separately.
