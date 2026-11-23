# CAM-S1 reward review r1 — mathematical analysis (read-only)

Reviewer: Claude Fable 5.1 (independent, fresh context). Date 2026-11-23. Scope: reward mathematics only.
Inputs read (worktree `/home/fair/workspaces/aerial_gym_ws/src/MOTAR-camonly`, unchanged):
`aerial_gym/task/navrl_task/navrl_task.py` (compute_state_reward_and_terminations 6019-6304, add_static_safety_reward
6306-6325, transform_action_to_command 4953-5008, step ordering 5063-5181), `navrl_camsafe.py` (draft),
`speed_governor.py` (riskcap law 465-472, corridor clearance), `navrl_sensor_ablation.py` (out-of-FOV clearance 0),
`navrl_task_config.py` 759-788, `ppo_navrl_perception_transformer.yaml`, `train_navrl_v2_search.sh`,
`navrl_action_models.py` (mu scale), `docs/evidence/camonly_2026-11-04/camk2/train_report_v3_s1.json` (lineage env),
`resources/models/environment_assets/bars_h3/*.urdf`. Calculations: `calc.py` / `calc_output.txt` in this folder
(CPU, `CUDA_VISIBLE_DEVICES=""`).

## Verdict: SOUND_WITH_CHANGES

The three additions are internally consistent and open no standing income, but two of the six claims rest on wrong
arithmetic. The dominant per-step term in a bar field is not the −0.05 time cost but the static-safety term r_ss,
which in the 12 m-range configuration is a standing **cost** of about −0.3/step (70 bars) to −0.9/step (205 bars).
That term, not the alive cost, sets the crash-versus-survive ordering, and −40 does not reverse it at 205 bars.

Facts that differ from the briefing: (i) the bars are boxes of 0.41-0.79 m side (mean 0.60 m), not ~0.2 m;
(ii) in the camera-only lineage the riskcap governor is ON in training (CAM-K2 v3 receipt: riskcap, fixed 2.0,
free 3.5355, slow 3, release 5, half-width 0.45) and out-of-FOV commands already get clearance 0, so the 45°
diagonal is capped at 2.0 m/s before any CAM-S1 change; (iii) the vy mu-scale 0.4 is a multiplier on an unbounded
network output (`mu = raw_mu * scale`), a soft bias, not a limit.

## Claims

### C1 — PARTLY
Only one term is potential-based shaping in the Ng-Harada-Russell sense, and only for a static target:
`progress = w·(||prev_pos − target|| − γ·||pos − target||)` with Φ(s)=dist, progress_gamma 0.99 = PPO γ, and
Φ zeroed at true terminals (correct). Its discounted sum telescopes to Φ(s_0) − γ^k Φ(s_k), i.e. +D over an episode
that ends in capture; at rest it pays a standing (1−γ)·D = +0.01·D per step (0.15 at 15 m), harmless because PBRS.
For a moving target the code re-anchors both distances to target_{t+1}; the non-telescoping residual per step is
bounded by ||target_{t+2} − target_{t+1}|| ≤ v_target·dt ≤ 0.15, so it is "almost PBRS" but, as the code says,
not policy-invariant.
Every other term defines the objective: vel (closing speed), alive, smooth, height, crab, yaw-rate², visibility,
r_ss, collision, capture. The vel term is 10×(Φ_t − Φ_{t+1}) with Φ=dist, i.e. PBRS at γ=1; relative to exact PBRS
at γ=0.99 it carries an extra per-step cost of 10·(1−γ)·dist = 0.1·dist (5 m: 0.5, 15 m: 1.5, 28 m: 2.8), i.e. a
strong, distance-proportional time pressure — not shaping.
Changing collision_penalty (terminal reward) and adding the blind penalty are objective changes; no invariance
theorem applies and "low risk" is an empirical statement about F's lineage (LiDAR in the observation, 87° FOV),
not a mathematical one for a from-scratch camera-only run.

### C2 — REFUTED (arithmetic and remedy), but a suicide incentive does exist for another reason
"−0.05 × 600 = −30" is undiscounted and ignores value bootstrapping. Under γ=0.99 the alive cost of surviving is
Σ_{t<600} 0.99^t·0.05 = 4.99 (infinite horizon: 5.00). −5 > −20, so the alive cost alone never favours crashing,
with either penalty. Time-outs are bootstrapped (`time_outs`, value_bootstrap True), so surviving to step 600 has no
terminal value at all: V(survive) is the infinite-horizon value of the behaviour, r_step/(1−γ) = 100·r_step.
The real driver is r_ss (table A1): V_hover = 100·(−0.05 + 1.5·r_ss + 0.01·D) ≈ −24 (70 bars) and −77 (205 bars);
V_wander ≈ −45 / −97. Against crash-now −20: at 205 bars crashing beats hovering by 57 and −40 still beats hovering
by 37. −40 flips the ordering only at 70 bars (−40 < −24.5). The break-even target-acquisition probability below
which "crash now" is optimal falls from 0.0063/step (158 expected search steps) to 0.0040/step (247 steps) at 205
bars — an improvement of a factor 1.6, not a removal. Remedy with numbers in E.

### C3 — PARTLY
Arithmetic of the base reward in isolation is right: per-axis ±2.5 gives 3.54 m/s on the diagonal, +1.04/step of
closing-speed reward, versus a crab penalty 0.6·0.5·(1−cos45°) = 0.088. But the premise is false in the study's own
configuration: with riskcap on and `no_lidar`, any command with |bearing| > 33° has clearance 0 and is capped at the
2.0 m/s floor, so the 45° diagonal yields 2.0 < 2.5 (forward). The surviving incentive is the FOV **edge**: at 33°
the per-axis box allows 2.5/cos33° = 2.98 m/s, +0.48/step over forward against a crab cost of 0.048, provided the
riskcap corridor clearance is ≥ 5 m (release 1.0); at 3.65-5 m the cap is 2.5-2.98, below 3.65 m it is 2.0 in every
direction and the box no longer matters. So the base reward pays the policy to ride the FOV edge, not to leave it.

### C4 — VERIFIED (with a condition)
2.5/cos 33° = 2.98 m/s; the camsafe cap leaves in-FOV commands untouched. The 2.98 needs clearance ≥ 5 m in the
0.45 m corridor; forward 2.5 needs ≥ 3.65 m; otherwise riskcap already holds 2.0. In clutter, "keeps its speed"
means 2.0 m/s in every in-FOV direction.

### C5 — VERIFIED
excess = (|θ|−33°)/147°; penalty = 0.5·excess·min(v/1,1): 45° → 0.041, 90° → 0.194, 180° → 0.500 per step
(at v ≥ 1 m/s). Scaled by 0.1 these are 0.004 / 0.019 / 0.05 against a closing-speed term of 0.25 and an r_ss
fluctuation of ±0.03-0.09 per step; with per-minibatch advantage normalisation only ratios matter, so the 45° signal
is ≈2 % of the vel term — an auxiliary signal. The cap removes the speed (2.0 → 1.0), which is the operative change.

### C6 — PARTLY
Hovering is strongly dominated by capture (+103 at 205 bars, +127 at 70) and, in hopeless states, even by crashing
(table A1); the additions do not change the hover value (gate = 0 at rest, cap irrelevant). They lower only the value
of blind motion, so relative to blind motion hovering becomes more attractive, and −40 raises the price of the
exploratory contacts a from-scratch policy must make. Both push toward the documented early hover trap (yaml comment;
entropy_coef history), so the risk is real but it is a training-dynamics risk. The reward-ordering risk that the
changes leave open is the opposite one: crash-seeking once the critic judges a state hopeless (C2).

## A. Discounted returns from a mid-episode state (D = 15 m, static target, yaw_align 0.6, vis 0.03)

Synthetic r_ss (assumptions): 40×40 m, bars uniform, discs of radius 0.38 m (= mean projected width 4s/π of the
0.60 m boxes; 0.10 m shown for the briefing's 0.2 m), 72 azimuths × 4 rows at +20/+10/0/−10°, slant range clipped
at 12 m, bars z∈[0,3], drone z=1, no ground plane, 400 free positions (≥0.5 m from a surface).

| field | r_ss mean (sd) | ×1.5 per step | 4 m clip ×1.5 |
|---|---|---|---|
| 70 bars, r 0.38 | −0.230 (0.156) | −0.34 | −0.11 |
| 205 bars, r 0.38 | −0.581 (0.241) | −0.87 | −0.27 |
| 70 bars, r 0.10 | −0.063 | −0.09 | — |
| 205 bars, r 0.10 | −0.193 | −0.29 | — |
| 205 bars, gradient: nearest bar 0.3-0.7 m vs 2-3 m | −0.975 vs −0.341 | 0.95/step | — |

Table A1 (values; capture at 2.5 m/s = 60 steps, progress telescopes to +D, bonus discounted):

| bars | P | V_hover | V_wander | crash now | crash @10 | crash @50 | capture @1.0 | capture @2.5 | p* (search beats crash) |
|---|---|---|---|---|---|---|---|---|---|
| 70 | −20 | −24.5 | −44.5 | −20 | −20.4 | −21.8 | 68.8 | 126.8 | 0.0017/step (594 steps) |
| 70 | −40 | −24.5 | −44.5 | −40 | −38.5 | −33.9 | 68.8 | 126.8 | 0.0003/step |
| 205 | −20 | −77.2 | −97.2 | −20 | −25.5 | −42.6 | 27.7 | 102.9 | 0.0063/step (158 steps) |
| 205 | −40 | −77.2 | −97.2 | −40 | −43.6 | −54.7 | 27.7 | 102.9 | 0.0040/step (247 steps) |

Readings. (1) Capture dominates everything whenever it is reachable; the ordering problem exists only in states
the critic judges hopeless. (2) "Crash after k steps" is worse than "crash now" at 205 bars for both penalties
(per-step reward is negative), so the pathological optimum is crash-as-soon-as-possible, which is what a policy that
flies blind into the nearest bar does. (3) −40 changes a preference only where |V_hover| < 40, i.e. sparse fields;
at 205 bars neither penalty makes survival preferable. (4) With the 4 m clip (NavRL's own default range, table
above) V_hover at 205 bars becomes 100·(−0.05 − 0.27 + 0.15) = −17 > −20: no suicide ordering with the existing
penalty. (5) Time-out: bootstrapped, so there is no "−30 then stop"; a policy that survives is valued as if it
continued, which is exactly why the per-step standing cost matters.

## B. Per-step terms at typical states (r_ss column at the 205-bar mean; see calc_output.txt for the full table)

| state | vel | alive | crab | vis | progress | blind | sum (no r_ss) | sum + r_ss(205) |
|---|---|---|---|---|---|---|---|---|
| forward, target ahead, 2.5 m/s | 2.50 | −0.05 | 0 | 0.03 | 0.40 | 0 | 2.88 | 2.01 |
| FOV edge 33°, 2.98 m/s toward target | 2.98 | −0.05 | −0.048 | 0.03 | 0.45 | 0 | 3.36 | 2.48 |
| 45° toward target, 2.0 m/s (riskcap floor) | 2.00 | −0.05 | −0.088 | 0 | 0.35 | −0.041 | 2.17 | 1.30 |
| 45° toward target, 1.0 m/s (new cap) | 1.00 | −0.05 | −0.088 | 0 | 0.25 | −0.041 | 1.07 | 0.20 |
| sideways 90°, 2.0 m/s | 2.00 | −0.05 | −0.300 | 0 | 0.35 | −0.194 | 1.80 | 0.93 |
| sideways 90°, 1.0 m/s (new cap) | 1.00 | −0.05 | −0.300 | 0 | 0.25 | −0.194 | 0.71 | −0.17 |
| backward 1.0 m/s | 1.00 | −0.05 | −0.600 | 0 | 0.25 | −0.500 | 0.10 | −0.77 |
| hover 15 m, target visible | 0 | −0.05 | 0 | 0.03 | 0.15 | 0 | 0.13 | −0.74 |
| turn in place, full yaw rate (yaw-rate² −0.02) | 0 | −0.05 | 0 | 0 | 0.15 | 0 | 0.08 | −0.79 |

Dominance order while closing: vel (2-3) ≫ r_ss (−0.3..−0.9, up to −1.5 beside a bar) > progress (0.25-0.45)
> crab/blind (≤0.3 unless backward) > alive, visibility, yaw-rate (≤0.05). Smooth: ~−0.08/step during a 3-step
acceleration, 0 at steady state. Height: 0 at the 1 m hold. The crash step is overwritten (−20/−40, no r_ss, no
blind); the capture step adds +30 to the shaped reward.

## C. Conflicts

- Riskcap governor: camsafe wraps the governor outermost and takes min(governor, 1.0) outside the FOV; inside the
  FOV it is the identity. No conflict; the 2.0 floor simply becomes unreachable out of view. Caveat: the governor's
  telemetry (`executed_speed_mps`, `scale`, contact_* sums) is recorded inside the inner call, before the cap, so
  those diagnostics overstate executed speed on capped frames.
- Closing-speed term with the target out of view: the vel term is paid on ground truth regardless of visibility.
  Target at 90°, D = 10 m, 205 bars: crab at 2.0 m/s (old) V = 51.2; crab at 1.0 m/s (new) V = −5.4; turn 90° in 6
  steps (3 rad/s) then 2.5 m/s forward V = 72.6. Turning was already optimal and the cap widens the margin. Target at
  60°: forward 2.5 m/s gives 2.5·cos60° = 1.25/step closing with no penalty, versus 1.0 − 0.15 − 0.09 = 0.76 flying
  blind at the cap. No trap.
- Search: turning in place costs only 0.02·a² ≤ 0.02/step; moving forward while scanning is penalty-free; a full
  360° sweep takes 2.1 s at 3 rad/s. The additions do not penalise search, only blind translation.
- Yaw dynamics: the penalty uses measured vehicle-frame velocity, the cap the command. During a 3 rad/s turn
  (17°/step) at 2.5 m/s the measured bearing leaves the FOV within 1-2 steps while the command is compliant; the
  penalty is 0.05-0.2 for the 2-3 steps the velocity controller needs to realign — small against the turn's value.
- One-step staleness: inside the wrapper `self.truncations` is the previous step's value (step() sets it after
  compute_state_reward_and_terminations), so the blind penalty skips the first step after a time-out reset and is
  applied on the time-out step itself (bootstrapped, harmless). Negligible, but not the mask r_ss uses.

## D. Reward-hacking routes and scaling

- Slow blind motion: penalty per metre = 0.5·excess·min(v,1)/(0.1·v) = 5·excess per metre for v ≤ 1 (0.41/m at
  45°, 1.94/m sideways, 5/m backward); constant in v below the cap and the cap forbids v > 1 out of view, so there is
  no "go faster to pay less per metre" exploit (the pre-existing crab term has the same gate and the same property).
  Backward 0.5 m/s toward the target nets 0.5 − 0.30 − 0.25 − 0.05 = −0.10/step: not profitable.
- Standing income: at rest the only positive terms are (1−γ)·D ≤ 0.28 (PBRS) and visibility 0.03; r_ss outweighs
  them everywhere in a bar field. No loiter income is opened.
- Yaw oscillation: it rotates the measured velocity in the vehicle frame and therefore triggers the penalty rather
  than avoiding it; yaw-rate² adds ≤0.02. Not a route.
- Scaling: scale_value 0.1 gives the learner crash −2/−4, capture +3, vel ≤0.3, r_ss −0.03..−0.15, blind −0.004 (45°)
  to −0.05 (backward). normalize_value removes the absolute scale from the critic; normalize_advantage makes the
  policy gradient depend on ratios within a 4096-sample batch, so a −40 penalty does not change the gradient scale
  but raises the batch advantage std slightly (≈2 crash samples per batch, spread over the 16.8-step GAE horizon),
  which marginally down-weights the dense terms. Nothing here creates a new optimum.

## E. Recommendations (each with the number that justifies it) and what to monitor

1. Remove the r_ss standing cost (preferred) or size the penalty to it. Compute r_ss with NavRL's own 4 m clip,
   `mean(log(min(d, 4)/4))`, instead of the 12 m sensor range: at 205 bars the per-step term goes −0.87 → −0.27,
   V_hover −77 → −17 > −20, so survival beats crashing with the existing −20 and p* → 0; the near-bar gradient (0.95/
   step between 0.5 m and 2.5 m) is unchanged inside 4 m. If the base r_ss must stay, the penalty that makes
   crash-now lose to wandering at 205 bars is P < V_wander ≈ −97, i.e. about −100 (scaled −10); −40 is a half
   measure. Either choice should be stated in the preregistration with this calculation.
2. Make no direction faster than forward: set `NAVRL_SPEED_GOVERNOR_FREE_MPS=2.5` (riskcap then caps the norm at
   2.5 in every direction; FREE ≥ FIXED 2.0 still holds), or clamp the command norm instead of the axes. This deletes
   the +0.48/step FOV-edge optimum (2.98 vs 2.5) and the diagonal bonus at its root without touching the reward; the
   camsafe cap then only decides 1.0 vs 2.0 out of view.
3. Keep blind weight 0.5 and cap 1.0 as drafted (0.041/0.194/0.5 per step, 5·excess per metre); do not raise the
   weight expecting it to steer — at 45° it is 2 % of the vel term under advantage normalisation. If a steering
   reward is wanted, the cap is the right lever (already in place).
Monitor (training logs / eval records):
- crash-seeking: crash rate stratified by "target detected at least once this episode" and by step of contact
  (`_diag_steps["contact"]` mean); early contacts with no detection = the C2 signature;
- hover trap: time-out rate, mean speed, fraction of steps with speed < 0.2 m/s, entropy/sigma drift;
- blind motion: `[camsafe] capped_frac / penalised_frac / mean_penalty` (every 3200 governor calls) and an eval
  histogram of |velocity bearing| (watch 25-33° edge-riding and the share > 33°);
- the actual standing cost: mean per-step reward of time-out episodes (should be ≈ −0.05..−0.3 after rec. 1, not
  −0.8); compare with the critic's value at episode start;
- governor telemetry: treat executed-speed statistics as pre-cap until the cap is recorded.
