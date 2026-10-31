# B1-R — Classical sensor-based baselines vs frozen Policy F (preregistration DRAFT)

**Status: DRAFT, not a preregistration.** This file does not bind anything. It becomes binding only
when a completed copy is committed as `docs/plans/prereg_b1r_classical_baseline_v1.md` **before** any
GPU run. GPU work needs separate user approval. G5 HOLD is unaffected. Branch `codex/mt-p1`, drafted
at HEAD `fde21cd`, 2026-10-23.

## 1. Purpose and claim boundary

MOTAR has no matched classical baseline that controls whole episodes under Policy F's contract
(`docs/EVIDENCE.md`: Class A = 0). The H-R3 scripted arms only controlled 2-s windows after the first
loss, at 160 bars and one seed, so they cannot serve as one. This study adds two sensor-based
classical controllers, evaluated under the same task path as Policy F.

- **Baseline value:** it tells us whether F's learned behaviour beats simple classical guidance with
  local LiDAR avoidance under an identical simulator, sensor and governor contract.
- **No novelty claim.** Learned vs PN or pure-pursuit comparisons exist (e.g., Gavin et al.
  arXiv 2603.16279 and 2607.05939; Pliska et al. RA-L 2024). It is already known that PN alone does
  not avoid obstacles. Nothing here is a real-flight, physical-interception or SOTA claim.

## 2. Arms (3)

| Arm | Controller | Online inputs | Output |
| --- | --- | --- | --- |
| **F** | Frozen Policy F, SHA-256 `f7022139…3ad40`, deterministic | the 898-D sensor-only actor observation | the policy action |
| **B-PN** | velocity-direction proportional navigation plus the shared LiDAR filter | tracker belief (position and velocity), own position, velocity and attitude, raw LiDAR ranges | normalised vehicle-frame `[vx, vy, 0, yaw_rate]` |
| **B-PP** | pure pursuit (heading = line of sight) plus the same LiDAR filter | same as B-PN | same as B-PN |

**Never an online input to B-PN or B-PP:** target ground truth, renderer masks, semantic IDs, the
obstacle layout, the critic `states`.

### 2.1 Guidance laws (planar; pursuer and target are both held at the 1.0 m flight altitude)

Let `r = p̂_T − p` and `ṙ = v̂_T − v` in the world XY plane. Here `p̂_T` and `v̂_T` are the online
tracker mean, and `p` and `v` are the pursuer's own state.

- LOS angle `λ = atan2(r_y, r_x)`. LOS rate `λ̇ = (r_x ṙ_y − r_y ṙ_x) / max(|r|², ε)`.
- **B-PN:** commanded course `ψ_c(k+1) = ψ_v(k) + N · λ̇ · Δt`, with Δt = 0.1 s and `ψ_v` = course of
  the pursuer's velocity. If `|v| < 0.2 m/s`, use `ψ_c = λ`.
- **B-PP:** `ψ_c = λ`.
- Nominal velocity: `v_nom · [cos ψ_c, sin ψ_c]`, rotated into the vehicle frame.

### 2.2 Shared components (identical in B-PN and B-PP)

- **LiDAR filter.** For the 5 nearest horizontal LiDAR points within 1.5 m, the approach component is
  removed with weight `clamp((1.5 − r)/1.0, 0, 1)`. Parameters are those of
  `motar_eval/h_r3_arms.py:16-29, 85-95`.
- **Speed bounds.** Per-axis normalisation by `NAVRL_MAX_VELOCITY = 2.5`, clamped to [−1, 1]. This is
  the same action bound as F.
- **Yaw law.** Yaw toward the belief with gain 2.0 /s, clamped to `YAW_RATE_MAX = 3.0`
  (`h_r3_arms.py:98-100`). This keeps the camera on the target.
- **No belief** (tracker inactive): zero XY command and a yaw scan at 0.6 rad/s until the camera
  reacquires. The 0.6 rad/s value is the KROC B1 search-supervisor rate.
- **Stale belief** (camera lost, tracker still active): use the tracker prediction until tracker
  expiry (5 s).

## 3. Execution path

The baseline action replaces F's action at every step of every episode, in the B arms only. It then
passes through the unchanged task path:
1. the riskcap governor, with `executed = min(requested, cap)` and cap ∈ [2.0, 3.5355] m/s
   (`speed_governor.py:465-475, 497`);
2. the altitude PI;
3. the Lee velocity controller;
4. `prev_action` feedback.

Requests below 2.0 m/s are never changed by riskcap. This is why B arms with `v_nom ≤ 2.0` are
governed only by their own filter.

**Implementation constraint.** No production file may change. That rules out `navrl_task.py`,
`speed_governor.py`, `kroc_eval_adapter.py` (byte-pinned by tests), checkpoints and `results/`.

The injection point must keep the KROC recorder's per-environment quota ledger intact. The candidates
are:
- an opt-in player-loop replacement following the `tools/h_r3_launcher.py:103-107` pattern; or
- a runtime attach in the style of E-L0.

The choice is fixed in this file before code freeze (open item O2).

## 4. Evaluation contract (copied from OCC-P2 moving A3)

The OCC-P2 A3 receipt is the contract source. Example:
`results/occlusion_confirmatory_v1/OCC-P2/moving/A3/E2_obstacle_aware__160bars__seed4301/receipt.json`.

Key items, which must be identical across arms:

- **Episodes:** `NAVRL_EPISODE_LEN_STEPS=600`, `NAVRL_EPISODE_QUOTA_PER_ENV=4`, 128 environments,
  so 512 quota episodes per cell.
- **Initial view and placement:** visible, 10.0 m, `PLACEMENT_MODE navrl_band`.
- **Target:** `e2_obstacle_aware`, waypoint pattern, bounded dynamics, `direct_l2_v1` backend, speed
  U[0.3, 1.5] m/s.
- **Limits:** `MAX_VELOCITY 2.5`, `YAW_RATE_MAX 3.0`, `ALT_HOLD_VMAX 2.5`, `MAX_TILT_DEG 45`.
- **Sensing:** LiDAR 72×4 at 12 m, perception and vision on.
- **Governor:** riskcap, FIXED 2.0 / FREE 3.53553390593 / SLOW 3.0 / RELEASE 5.0 /
  HALF_WIDTH 0.45 / MARGIN 0.45.
- **Termination:** capture = swept closest approach under 0.5 m; crash = contact above 0.05 N, height
  out of bounds or arena exit; timeout = 600 actions.

Because any contact terminates the episode as a crash, **capture implies collision-free capture**
under this contract.

- **Densities:** 70 / 115 / 160 / 205 bars.
- **Evaluation seeds:** 4301–4307. These are the OCC-P2 seeds, so the F arm doubles as a reproduction
  canary. F's values on these seeds are therefore known in advance (unblinded). The baseline arms are
  evaluated blind.
- **Cells:** 3 arms × 4 densities × 7 seeds = **84**.

## 5. Baseline parameter selection (before evaluation, separate seed)

The grids are:
- **B-PN:** `N ∈ {3, 4, 5}` × `v_nom ∈ {2.0, 2.5, 3.0}` m/s (9 configurations).
- **B-PP:** `v_nom ∈ {2.0, 2.5, 3.0}` (3 configurations).

Tuning is run on:
- **Seed:** 4390. The preflight must verify that it appears in no existing receipt.
- **Densities:** 115 and 205.
- **Quota:** 2, giving 24 tuning cells.

Selection rule:
- **Metric:** the mean capture rate over the two densities.
- **Tie-breaks:** lower crash rate, then lower `v_nom`.

The selected values are written to a frozen JSON with SHA-256 before any evaluation cell starts.
There is no re-tuning after evaluation.

## 6. Integrity and stop gates

| Gate | Rule | On failure |
| --- | --- | --- |
| G0 reproduction | All 28 F cells reproduce the OCC-P2 per-cell capture, crash and timeout counts exactly | `VOID_CONTRACT_MISMATCH`; stop |
| G1 baseline viability | The selected B-PN tuning capture at 115 bars is ≥ 0.80. OCC-P2 F at 115 bars is 6987/7168 = 97.5 % | `STOP_BASELINE_DEFECT`; fix only by a dated amendment, then re-tune |
| G2 runtime | No non-finite action; actions within bounds; z command 0; source and config SHAs unchanged | stop the cell, record, do not salvage |
| G3 resources | Disk ≥ 3 GB; wall cap 15 min per cell, 10 h total | stop |

## 7. Endpoints

**Primary**
- Capture rate per arm. The unit is the evaluation seed, using the unweighted mean of its four
  density cells. Under §4, capture is collision-free capture.

**Secondary**
- Crash and timeout rates; per-density capture and crash.
- Time to capture: on captured episodes, and as RMST over all episodes truncated at 60 s.
- Requested, executed and actual speed (governor diagnostics in `result.json`).
- Stopping margin at the crash step.
- Fraction of steps with a camera detection.

**Exploratory**
- Crash hazard by tracking state (lost, after reacquisition, steady), using the R01 method from the
  OCC-style frame trace.

## 8. Analysis

**Contrasts on the primary endpoint**
- C1: F − B-PN
- C2: F − B-PP
- C3: B-PN − B-PP

**Inference**
- Seed-paired, n = 7.
- Exact two-sided sign-flip test (minimum attainable p = 0.015625).
- BCa 95 % CI over seeds.
- Holm correction over C1–C3.
- Per-density contrasts are exploratory and uncorrected.

**Pre-declared reading**

| Outcome | Reading (baseline statement only) |
| --- | --- |
| C1 and C2 > 0 with CIs above 0 | F outperforms both classical sensor-based baselines under this contract |
| C1 CI includes 0, or favours B-PN | F's learned behaviour does not beat a classical sensor-based controller. This is reported as a primary limitation of the system paper |
| C3 | PN vs pure pursuit, descriptive only |

No result is described as a new method, a safety guarantee or a physical-interception capability.

## 9. Implementation scope (after approval only)

**New files**
- `motar_eval/classical_baseline.py`: pure functions taking tracker belief, own state and LiDAR, and
  returning actions. No simulator imports.
- A launcher or opt-in runner hook (open item O2).
- `tests/test_b1r_classical_baseline.py`.

**CPU tests**
- action bounds and z = 0;
- input firewall: no ground-truth or layout arguments, enforced by an import and signature check;
- LOS-rate sign and magnitude on synthetic geometry;
- B-PP equals B-PN when `λ̇` is 0 and the course is on the LOS;
- the no-belief yaw scan;
- determinism;
- that the LiDAR filter matches the `h_r3_arms` reference outputs.

**Unchanged:** all production and task files, governor, checkpoints, results and preregistrations.

## 10. Cost estimate

| Item | Basis | Estimate |
| --- | --- | --- |
| F canary, 28 cells | OCC-P2 F 160-bar cell: 110 s (receipt above) | about 1 GPU-h |
| B-PN and B-PP, 56 cells | slower controllers mean longer episodes; assume 1–2× F | 2–4 GPU-h |
| Tuning, 24 cells at quota 2 | about half a cell each | 0.5–1 GPU-h |
| **Total** | | **about 3.5–6 GPU-h**, under a 10 h cap |

Engineering time (implementation, CPU tests, review) is separate: about 1–2 working days.

## 11. Open items before freeze

- O1: verify tuning seed 4390 is unused.
- O2: choose the action-injection point compatible with the KROC recorder quota ledger and occlusion
  trace.
- O3: confirm the governor telemetry fields recorded per cell.
- O4: independent review of the completed draft.
- O5: decide whether the result must land before the 2026-10-28 KROC Simulation Freeze.
