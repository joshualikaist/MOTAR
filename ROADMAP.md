<!-- MOTAR long-term implementation roadmap. Auto-generated 2026-07-14 by the roadmap
workflow (5 design agents + synthesis, grounded in the codebase + NavRL). The high-level
phase plan lives in RESEARCH_PLAN.md; this file holds the per-phase EXACT code changes. -->

# MOTAR — Integrated Long-Term Roadmap (Phase 1 → paper)

Synthesis of Findings A–E. Where the five sub-analyses disagreed on ordering (each author front-loaded its own feature), I adjudicate below and commit to a single plan. All paths absolute under `…/src/aerial_gym_simulator/`.

---

## 0. TL;DR

- **Recommended next phase (immediately after the current Phase-1 run): Phase 2 = static DENSITY sweep (feature 3).** Cheapest, zero obs/reward change (current checkpoint transfers), and it nails the 8 GB VRAM envelope early — which every later feature depends on knowing.
- **Committed feature order: (3) density → (2) moving target → (4) dynamic obstacles → (1) 3D.** Feature-node terms; renumbered as Phases 2→3→4→5.
- **Why this beats the sub-findings' orders:** the paper's headline is the **density × target-speed performance map**, which needs only (3)+(2). Securing that on the critical path — before the two most expensive/optional features (dynamic branch, 3D exploration) — is the right bet on a single RTX 3070. See §2 for the adjudication.

---

## 1. Cross-cutting decisions (RESOLVED)

1. **2D-first; 3D last and isolated.** The paper core (density×speed map, RQ1–RQ2) is fully 2D-expressible. 8 GB cannot afford taller-arena + doubled-exploration of 3D *simultaneously* with dense/dynamic fields. 3D enters only as a terminal capability check/ablation at **reduced density**. (Overrides Finding B's "3D-first" — Finding B's own VRAM/exploration analysis actually supports last.)
2. **Moving-goal frame stays FIXED per episode.** Set `target_dir_2d` once at reset (start→initial-target, z-zeroed, `navrl_task.py:245-247`); S_int direction/dist/vel already recompute each step vs current `target_position` (`process_obs_for_task:457-468`). Do **not** recompute the frame each step (collapses `rpos_unit_g→(1,0,0)`, deletes bearing). This directly corrects the overstated requirement at `RESEARCH_PLAN.md:133`. `goal_frame_refresh=True` is an ablation only.
3. **Paper core = density × target-speed performance map (RQ2).** This is why (2) precedes (4): the headline is reachable at the end of Phase 3 without dynamic obstacles ever being built.
4. **VRAM is the master constraint, and feature (3) is its ONLY real lever.** Measured baseline 48 bars × 256 env ≈ 6.8 GB. Moving target (+0 mesh, pure coordinate), dynamic obstacles (+0 warp mesh — GT-state tensors, NavRL-faithful), and 3D (taller boxes = identical triangle count) are all **VRAM-neutral**. So: fix the envelope in Phase 2, and every subsequent feature fits inside it. Never stack high-density + 3D exploration.
5. **Target speed near `max_velocity=2.0` is degenerate.** Pure tail-chase is uncatchable at v_target = v_chaser. Cap **trainable** target speed at ~1.0–1.5 m/s; report **2.0 as an eval-only stress cell**. This is both a real result and a training-stability guard.

---

## 2. Dependency graph + ordering adjudication

**Feature nodes and edges** (hard = blocking; soft = reuse-amortizing):

```
        (3) DENSITY ──soft(count-curriculum machinery)──▶ (4) DYNAMIC
   [no deps; sets VRAM envelope]                              ▲
        │                                                     │ soft (shares _advance_* motion code;
        │ x-axis of headline heatmap                          │ target is GOAL via S_int, NOT via S_dyn)
        ▼                                                     │
        (2) MOVING TARGET ────────────────────────────────────┘
   [no hard deps: moving goal = S_int implicit, Finding C]
        │
        ├─ (3)+(2) ⇒ HEADLINE density×speed map  ◀── critical path ends here
        │
   (1) 3D  ── orthogonal to all; introduces typed short/tall obstacles
   [do LAST]     reused by (4)'s 2D/3D-type split and (2)'s 3D-interception
```

- **Hard prerequisites between features: none.** Every post-Phase-1 feature only needs the working static policy.
- **Soft (reuse) edges:** (2)→(4) motion machinery; (3)→(4) `num_obstacles_in_env` count machinery; (1)→{(4) 3D-type obstacles, (2) 3D interception} via obstacle typing + vz unlock.
- **Critical path to the paper's headline figure:** Phase 1 → (3) → (2). Everything else (4, 1, perception) is an additive extension.

**The adjudicated disagreement — (2)-before-(4) vs (4)-before-(2):**
- Finding A argued **(4) before (2)** ("build S_dyn once; the target is just another dynamic object; feeds the detection story").
- Finding C argued **(2) before (4)** and is architecturally decisive: **the moving target is the GOAL, perceived through the S_int channel** (per-step `rpos`/`dist`/`vel` already recomputed), **not through S_dyn**, which is the *obstacle*-avoidance channel. So (4) is **not** a true prerequisite for (2).
- **Resolution: (2) before (4).** Reasons: (i) the headline (density×speed) needs only (3)+(2) — reach it fast; (ii) (2) is cheaper and lower-risk (no 202-dim obs, no second network branch, no new reward term) — do the cheap high-value thing first; (iii) (2) builds the single-entity motion machinery that (4) then generalizes to `(N,M)` vectorized goal-seeking patrol; (iv) on a single 3070 with ~2-week training budgets, securing the headline before the most complex feature is the correct risk posture.
- **Kept as documented alternative:** if the "target-as-dynamic-entity / perception-detection substrate" story is later prioritized over speed-to-headline, (3)→(4)→(2) is defensible — but Phase 6 (perception) builds (4) regardless, so the detection story is not lost by deferring (4).

**3D last (near-unanimous except Finding B):** orthogonal to dynamics; VRAM-neutral but multiplies exploration cost on every other axis; worst thing to combine with dense/dynamic on 8 GB. Isolated terminal ablation at reduced density.

---

## 3. Phase-by-phase plan

Existing plan Phases 0–6 renumber as: **P0→folded into P1 preamble; P1 unchanged; P2 unchanged (density); old P3 (dynamic) → new P4; old P4 (target) → new P3 (SWAP); NEW P5 = 3D; old P5 (perception) → P6; old P6 (paper) → P7.**

---

### Phase 1 — Static 2D navigation *(current, ~done; M1 tuning)*
- **Objective:** cross the 48-bar field at 1 m, capture-terminate. Current result **86% capture (peak 94%), 14% crash**.
- **Code/config:** as-is. No changes; this phase is the baseline checkpoint every later phase warm-starts from.
- **Milestone/metric:** stable ≥85% capture at full 24 m crossing over ≥3 seeds. Freeze this checkpoint as `static_baseline`.
- **Carryover:** obs 152 (S_int 8 + LiDAR 36×4=144), action 3-D goal-frame velocity (z zeroed), reward = vel-toward-goal + PBRS + static-safety(1.5, re-baselined) − smooth(0.1) − height(8, inert) + capture(+30)/collision(−20) − alive(0.05).

---

### Phase 2 — Static DENSITY sweep (feature 3) — **NEXT** — RQ1 x-axis
*(spec: Finding D)*
- **Objective:** density–performance curve + curriculum-vs-fixed + cross-density generalization matrix (NavRL Table-I in spirit). This is the paper's headline **x-axis** and it de-risks VRAM.
- **Key mechanism:** active bar count is set live by writing `self.obs_dict["num_obstacles_in_env"]=n` (read every reset at `env_manager.py:290`); `AssetManager.reset_idx` (`asset_manager.py:55-78`) places `n` on the jittered grid and teleports bars `n..N_max` to `-1000` (invisible to 4 m LiDAR + contacts). **Blocker fix:** `bar_asset_params.keep_in_env=True` (`env_object_config.py:719`) currently floors the count at `num_assets`; flip to **`False`**.
- **Exact code changes:**
  - `config/asset_config/env_object_config.py` — `bar_asset_params`: `keep_in_env=False`; `num_assets=int(os.environ.get("NAVRL_MAX_BARS",150))` (build ceiling; env-var lets fixed-density runs build exactly N to save VRAM); `import os`; (risk §) raise `min_state_ratio[0]→0.13` to widen the spawn corridor at high density.
  - `config/env_config/navrl_bars_env.py`: `min_obstacle_xy_spacing 1.8→1.5` (grid guarantees ≥1.5 m up to N≈160).
  - `config/task_config/navrl_task_config.py`: add `class density` (`num_bars_active`, `use_density_curriculum`, `n_start=25`, `n_final=150`, `mode="success_gated"`, `success_threshold=0.8`, `promote_step=15`, `warmup_epochs=2500`) and `class eval` (`densities=[25,50,75,110,150]`, `episodes_per_condition=500`, `seeds=[1..5]`).
  - `task/navrl_task/navrl_task.py`: in `__init__` set `n_bars_active` (env-var `NAVRL_NUM_BARS` or config/curriculum floor) and write `obs_dict["num_obstacles_in_env"]`; add `_update_density(successes,finished)` called after `_update_curriculum` (`:318`) — success-gated promotion (+15 bars when rolling capture>0.8) or epoch-ramp; persist `n_bars_active` in `get_env_state`/`set_env_state` (`:169-177`, `num_task_steps` already saved).
  - `task/navrl_task/train_dashboard.py`: add `n_bars_active` field to `record_navrl_epoch_episodes`.
  - **New** `rl_training/rl_games/eval_navrl_density.py` (cross-density matrix; Wilson-score CIs on rates, 10k bootstrap on time/SPL/min-sep; long-form CSV `train_density,eval_density,seed,metric,value,ci_low,ci_high`); **new** `task/navrl_task/plot_density.py` (Fig A curve + Fig B heatmap; build with `dataviz` skill).
- **Density levels** (band denominator A_band = x∈[2.16,23.04]×y∈[0,24] ≈ 501 m²): 25/50/75/110/150 bars → **5.0 / 10.0 / 15.0 / 22.0 / 29.9 per 100 m²** (L4=22 = NavRL density; L5=150 = ceiling for 1.5 m spacing).
- **Obs/action/reward/term:** **UNCHANGED** (density is a pure environmental IV; the re-baselined static-safety log term already scales with density).
- **Curriculum:** keep the existing goal-x curriculum. Two regimes to compare: **fixed-density** (5 runs, build exactly N) and **density-curriculum** (1 run, success-gated 25→150). Sequence: goal-x plateaus first, then density ramps (no dual-curriculum entanglement).
- **Milestone/metric:** monotone success-vs-density curve with 95% CI over ≥3 seeds; 6×5 generalization matrix (train A→eval B), ≥500 eps/cell.
- **VRAM:** the pressured phase — the ONLY mesh-mover. Fixed-density training builds exactly N (L1–L4 shrink VRAM); **only L5=150 and the single-build eval are at risk**. Protocol: build L5 headless, watch `nvidia-smi`; if >~7 GB, drop **L5 and the max-build eval only** to `--num_envs 128` (runner rewrites env_config, `runner.py:444-449`); L1–L4 stay 256. (Note: `nvidia-smi` currently shows an active process — measure from a fresh process.)
- **Risks/mitigations:** spawn-adjacent bar at L5 → raise `min_state_ratio[0]→0.13` + optional spawn-clearance resample; jittered grid removes clustering (document as deliberate; optional clustered robustness variant later); rising L5 timeouts at `episode_len_steps=300` (real reachability vs horizon artifact) → run one longer-horizon L5 control.
- **Effort:** ~3 days code. **Training dominates: 5 fixed + 1 curriculum × 3 seeds ≈ 18 runs × 8–12 h ≈ 1.5–2 weeks** on one 3070 (exceeds the plan's 1-week estimate — flag it). Mitigation: 1-seed screening pass (6 runs, ~3–4 days) to lock curve shape, then add seeds only on anchors L2/L4/L5.

---

### Phase 3 — MOVING TARGET / 2-drone, scripted (feature 2) — RQ2 headline
*(spec: Finding C)* **SWAPPED ahead of dynamic obstacles.**
- **Objective:** turn static navigation into pursuit-of-a-moving-goal amid obstacles; produce the **density × target-speed heatmap** (with Phase-2 density) — the unique contribution — plus zero-shot vs retrained curves.
- **Architectural key:** the goal is a **pure coordinate** advanced by the task (capture is distance-based, no contact). Reuse the *motion math* from `shooting_moving_target_task` but **not** its actor/`env_asset_state_tensor` plumbing → **0 extra VRAM**, warp budget untouched.
- **Exact code changes** (`task/navrl_task/navrl_task.py`):
  - `__init__`: buffers `target_velocity`, `target_origin/wp/center`, `target_radius/phase`; compute **`rl_dt = num_physics_steps_per_env_step_mean × obs_dict["dt"] = 10×0.01 = 0.1 s`** (do NOT reuse raw `obs_dict["dt"]`); `_resample_every = round(target_resample_seconds/rl_dt)`.
  - New `_sample_target_velocity(env_ids)` — port `shooting_moving_target_task.py:99-110` (speed~U[lo,hi] from a `_target_speed_bounds` curriculum helper, random heading, vz=0).
  - New `_advance_target()` — called first in `step()` (`:279`): integrate `target_position[:, :2] += target_velocity[:, :2]*rl_dt`; z=`flight_altitude`; dispatch on `target_motion ∈ {static, constant_reflect (port `_reflect_target_xy_at_env_bounds:120-142`), waypoint (NavRL patrol `env.py:256-282`), circular, evade (stretch)}`; **push goal out of bars** each step by reusing the `reset_idx` clearance loop (`:228-239`) so the goal never sits inside a pillar.
  - `reset_idx`: after goal placement call `_sample_target_velocity`; seed motion anchors; keep `target_dir_2d` fixed (decision §1.2).
  - `compute_state_reward_and_terminations` (`:350`): change velocity reward to **range-rate on the moving goal** — `reward_vel = ((vel_w − target_velocity)·vel_dir)` (reduces to current term at speed 0). PBRS `prev_dist` already recomputes vs current goal — unchanged.
  - Enable the already-scaffolded **segment capture** (option F, `:384-388` + `prev_pos` buffer) — anti-tunneling vs fast crossings (~0.4 m relative displacement/step vs 0.5 m sphere).
  - `process_obs_for_task`: unchanged for 8-dim; for the optional 11-dim variant append `vec_to_goal_frame(target_velocity − vel_w, target_dir_2d)`.
- **New config params** (`navrl_task_config.py`): `target_motion="static"`, `target_speed_min/max=0.0`, `reflect_target_at_bounds=True`, `target_resample_seconds=2.0`, `target_local_range=[5,5]`, `target_circle_radius=3.0`, `target_circle_omega=0.5`, `target_evade_radius=3.0`, `target_evade_gain=1.0`, `goal_frame_refresh=False`, `obs_target_velocity=False`, `capture_segment_test=True`, plus target-speed-curriculum block (`start_epoch=500`, `end_epoch=2000`).
- **Obs/action/reward/term:** **action unchanged**; **obs 152 unchanged** for zero-shot/retrain-same-shape (moving info implicit in per-step `rpos`); optional 11-dim → obs 155 (retrain only). Reward: one-line range-rate change. Termination: instantaneous `dist<0.5` + segment test; **no `t_hold`** (a dwell makes evading capture near-impossible; NavRL uses instantaneous, `env.py:583`).
- **Curriculum:** stage axes — Phase-2 goal-distance reaches full scale first, then **freeze distance and ramp `target_speed_max 0→1.5`**; speed=0 warm-starts from the static checkpoint.
- **Milestone/metric:** M0 static-speed regression (½ d); **M1 zero-shot capture-vs-speed curve** {0,0.5,1,1.5,2} × {constant,circle,waypoint}, no training, the primary free figure (expected monotone decline, ~0 near 2.0); M2 retrained 8-dim to 1.0 m/s ≥ ~2× zero-shot at that speed; M3 11-dim + waypoint/evade comparison. **Headline heatmap = Phase-2 density × this speed axis** (secured here).
- **VRAM:** +0 mesh; a handful of (N,3) tensors (KBs). Keep `--num_envs 256`.
- **Risks/mitigations:** speed→v_lim degeneracy → cap trainable ≤1.5, report 2.0 eval-only; tunneling → segment capture; goal-in-bar → per-step push-out; two-axis instability → stage distance then speed; wrong dt → use `rl_dt=0.1`.
- **Effort:** ~3–5 days, heavy reuse from `shooting_moving_target_task`. Evading/self-play target = **Phase 3b stretch** (+5–10 days).
- **Optional (eval only):** visible target actor in an IGE-only env copy driven by `env_asset_state_tensor[:,0,0:3]=target_position` — cosmetic; keep it OUT of the warp env (avoids per-step `refit()`).

---

### Phase 4 — DYNAMIC obstacles, 2D (feature 4) — RQ1 (dynamic half)
*(spec: Finding E)*
- **Objective:** reach the goal while avoiding a **patrolling** obstacle field, requiring relative-velocity/intercept reasoning. Enables the S_dyn-branch ablation and density×dynamic-count grid.
- **Architectural key (NavRL-faithful):** NavRL LiDAR raycasts **only static terrain** (`env.py:57`); dynamic obstacles are perceived **only** via a GT-state block `S_dyn` (`env.py:479-518`) with **analytic** collision (`env.py:520-528`), no physics contact. In our stack, making moving meshes LiDAR-visible would force a per-step warp `refit()` of every env (`warp_env_manager.py:40-54`) and multiply mesh VRAM. **So dynamic obstacles are pure per-env GPU tensors in `navrl_task` — no actors, no meshes, no refit → essentially free on 8 GB.** Reuses Phase-3's `_advance_target` motion machinery, generalized to `(N,M)`.
- **Exact code changes:**
  - `task/navrl_task/navrl_task.py`: buffers `dyn_pos/vel/origin/goal/size (N,M,3)`, `dyn_active (N,M)`, `dyn_is_2d (N,M)`. New `_spawn_dynamic_obstacles(env_ids)` (port `env.py:172-247`: interior-band origins with rejection-spacing reusing the goal-clearance loop, avoid spawn corridor + goal; **Phase-4-2D: all `dyn_is_2d=True`, height=5.0**, avoid-only). New `_advance_dynamic_obstacles()` called first in `step()` (port `env.py:251-291`, vectorized: goal-reached→resample local-range goal, every ~2 s resample speed `U[0.5,1.5]`, integrate at `rl_dt`, mask inactive). New `_dynamic_obs_features()` (port `env.py:479-532`: topk-5 nearest by 2D dist, goal-frame rpos/vel via `vec_to_goal_frame`, **10-dim/obstacle** `[rpos_unit_g(3),dist2d,dist_z,vel_g(3),width_cat,height_cat]`, zero-pad out-of-range; analytic collision `dist2d≤w/2+0.3 & dist_z≤h/2+0.3`; **re-baselined** `r_ds=(log(clamp(surf,1e-6,range))−log(range)).mean` — the `−log(range)` mirrors the static fix at `:434` to kill loiter income). `process_obs_for_task`: append S_dyn at `[152:202]`. `compute_state_reward_and_terminations`: `crashed_out |= dyn_collision_now`. `add_static_safety_reward`: `+= safety_dynamic_weight * r_ds`.
  - `rl_training/rl_games/navrl_network.py`: add NavRL dynamic branch (`ppo.py:30-40`) — `dyn_mlp = Linear(50,128)→ELU→Linear(128,64)→ELU`; fuse dim `= 128(CNN)+8(S_int)+64`; guard on obs dim (202→dynamic, 152→static) for backward-compat so Phase-1/3 checkpoints load.
  - `train_dashboard.py`: add `dyn_collision_rate`.
- **New config** (`navrl_task_config.py` `class dynamic`): `n_d=5`, `count_start=60`, `count_final=120`, `count_warmup_epochs=2000`, `vel_min=0.5`, `vel_max=1.5`, `vel_resample_period_s=2.0`, `local_range=[5,5,0]`, `width_bins=[0.25,0.5,0.75,1.0]`, `height_2d=5.0`, `height_3d=1.0` (deferred to P5), `frac_2d=1.0`, `min_dyn_spacing=2.0`; `safety_dynamic_weight=1.0`. **`observation_space_dim 152→202`.**
- **Obs/action/reward/term:** obs `152→202` (layout `[S_int 8 | LiDAR 144 | S_dyn 50]`); action unchanged; reward `+ 1.0·r_ds`; termination `crashed_out |= analytic dyn_collision` (existing −20).
- **Curriculum:** count 60→120 (our addition — NavRL ships fixed 80) driven by `num_task_steps`/`ppo_horizon`; optional speed ramp 0.5→1.5. Start only after static goal-x plateaus; resume Phase-1/3 checkpoint (S_int/LiDAR weights transfer; only dyn branch + widened fuse reinit).
- **Milestone/metric:** M1 plumbing (obs=202 flows, unit-test motion + S_dyn zero-pad); M2 slow/few (count=60, vel≤0.8): capture within ~10–15 pts of static, dyn-collision <15%; M3 full 60→120, vel 0.5–1.5: capture ≥~65%, dyn-collision ≤10%. Ablations: no-S_dyn (LiDAR-only), no-r_ds, GT-vs-noised S_dyn.
- **VRAM:** **+0 warp mesh**; ~9 buffers `(512,120,3)` ≈ 2–3 MB + ~10k net params — **8 GB ceiling untouched**. Keep 256 (128 only if combined with Phase-2 high density). Do NOT make dynamic obstacles LiDAR-visible for training.
- **Risks/mitigations:** loiter income → `−log(range)` re-baseline (specified); port sign/frame bugs → unit-test single-env slice vs NumPy reference; topk over inactive slots → mask inactive to `+inf`; per-RL-step integration → fine at ≤1.5 m/s, 10 Hz (≤0.15 m/step).
- **Effort:** ~3–5 days (task port ~150 lines transcribing `env.py:251-554`; network ~15 lines; config declarative) + training. No new URDF/env/robot assets.

---

### Phase 5 — 3D / z-axis flight (feature 1) — NEW, capability check
*(spec: Finding B)* Was unmapped in the plan.
- **Objective:** unlock vertical DOF so the drone chooses **per obstacle** to fly over (short bars) or around (tall bars). **The feature is NOT "un-zero vz" — that trivializes to overflying everything; it is "un-zero vz + make the field vertically heterogeneous."**
- **Exact code changes:**
  - `task/navrl_task/navrl_task.py`: gate the altitude lock — replace `command[:,2]=0.0` (`:275`) with `if getattr(task_config,"lock_altitude",False): command[:,2]=0.0`. Replace goal-z pin (`:204`) with sampled `goal[:,2]=z_lo+(z_hi−z_lo)*rand` from a new `_goal_z_range()` (mirror `_goal_x_max:476-483`, ramp over `z_warmup_epochs` using `num_task_steps`/`ppo_horizon`, checkpoint-safe). Add verticality diagnostics to the dashboard (mean peak `|z−flight_altitude|`, "overfly fraction", current `(z_lo,z_hi)`). **`height_range` (`:249-254`) and `penalty_height` (weight 8.0, `:355-361`) are already coded and become active automatically.**
  - `navrl_task_config.py`: `lock_altitude=False`; `curriculum.goal_z_min=0.5`, `goal_z_max=2.5`, `z_start_epoch=3000` (after horizontal ramp), `z_warmup_epochs=2000`; `upper_height_bound 4.0` (now bites).
  - **Mixed-height assets:** new `tools/generate_bars_3d.py` → `bars_short/` (1.2 m: blocks at 1.0 m, cleared ≥1.5 m) + `bars_tall/` (4.0 m = ceiling, un-overflyable); new `short_bar_asset_params`/`tall_bar_asset_params` in `env_object_config.py` (clone `bar_asset_params:658-724`, z-ratio 0.20 / 0.667, 24 each); new `config/env_config/navrl_bars3d_env.py`; register in `env_manager/__init__.py` (`:18`); set `env_name="navrl_bars3d_env"`.
  - LiDAR: default keep 36×4 VFOV[−10,20] (obs 152, no net change). Ablation A (free): `navrl_lidar_wide_config.py` VFOV[−30,40]. Ablation B: vbeams 4→6 (obs 152→224, needs net width change).
- **Obs/action/reward/term:** action dim stays 3 (vertical now live); obs unchanged (dist_z, 3D vel_g already computed); **height penalty (8.0) becomes the primary 3D tuning knob** (over-a-short-bar excursion ~0.08/step); band = start↔goal so necessary climbs are free.
- **Curriculum:** Phase-0 goal-z band [1,1] (2D over the already-mixed field, concurrent with goal-x ramp) → Phase-1 band expands 1.0→[0.5,2.5] (verticality emerges) → optional Phase-2 randomize start-z. Note: warp bakes meshes at build → verticality is curricula'd via the goal-z band, not by adding bars (both bar types present from step 0).
- **Milestone/metric:** M1 `lock_altitude=True` on 3D env ≈ 2D capture (no regression); M2 full band capture within ~10 pts of 2D; M3 verticality causally helps (z-free strictly beats z-locked on high/low goals; overfly-fraction rising; mixed-height-vs-uniform ablation).
- **VRAM:** **neutral** — splitting 48 bars into 24+24 taller boxes = identical triangle count; ray buffer unchanged. Cost is exploration/wall-clock, not memory. Keep 256, but **reduce density** and don't stack with Phases 2-4 hard settings.
- **Risks/mitigations:** height penalty over-suppresses overflight → lower to ~4.0 / widen margin 0.2→0.4 in Phase-1, don't remove; 4 vbeams too coarse → VFOV widen (free) before adding beams; trivial overflight leaks → tall=4.0=ceiling + goal_z_max=2.5<3.0; dual curriculum → sequence (z after k_max).
- **Effort:** ~2–2.5 days code, mostly asset/config plumbing (very high reuse — machinery is dormant, not absent) + ~1 week training/ablation wall-clock.

---

### Phase 6 — Perception integration (was Phase 5) — RQ3
- **Objective:** move from GT state to a degradation model (track A: noise/range-limit/dropout/latency on S_dyn + LiDAR) or a depth-detector (track B). Phase-4's GT S_dyn is the clean baseline this ablates on top of (config flag `dynamic.gt_state=True` default).
- **Dependency:** requires Phase 4 (dynamic obstacles built) — satisfied regardless of the 2-vs-4 ordering choice.
- **Effort/metrics:** as in existing plan; report degradation curves vs GT.

### Phase 7 — Failure-mode analysis, ablations, stats, paper (was Phase 6)
- ≥3–5 seeds, ≥500 eval eps/condition; failure-mode clustering; the full ablation suite (z-locked vs z-free, no-S_dyn, no-r_ds, curriculum-vs-fixed, density×speed, density×dyn-count). Assemble Fig A (density curve), Fig B (generalization heatmap), headline density×speed map, zero-shot-vs-retrained gap.

---

## 4. VRAM budget shared across features (8 GB / RTX 3070)

| Phase (feature) | Mesh Δ | VRAM impact | Env-count guidance |
|---|---|---|---|
| P1 static (baseline) | 48 bars | ~6.8 GB @ 256 | 256 |
| **P2 density (3)** | up to **150 bars** | **only mesh-mover** | 256 for L1–L4; **128 for L5=150 / max-build eval only** (or shrink arena) |
| P3 target (2) | +0 (pure coord) | KB tensors | 256 |
| P4 dynamic (4) | +0 warp (GT-state) | ~2–3 MB tensors | 256 (128 only if stacked w/ P2 high density) |
| P5 3D (1) | +0 (taller box = same tris) | neutral; cost = exploration | 256, **reduced density**; expect fewer envs for stable *learning* (wall-clock, not VRAM) |

**Master rule:** VRAM = feature-(3) density × `num_envs`. Nail the envelope in P2; every later feature fits inside it; **never co-schedule high-density + 3D exploration.**

---

## 5. What changes in RESEARCH_PLAN.md (concrete edits)

**Mark DONE:**
- `:105-107` Phase-0 boxes → all complete (per WORKLOG branch `research/navrl-env`, 36×4 LiDAR, TB logging, NavRL env.py read).
- `:203-210` "이번 주 액션 아이템" 1-6 → done; **Section 9 is entirely stale — archive it.**

**Fix STALE:**
- `:111, :116` "커리큘럼 18m" → **24 m** (`k_final=24`), floor rising to **20 m** (`k_min_final=20`). Purge every "18 m".
- `:174` "512–1024 envs", `:177` "20×20m 축소판" → **24×24 arena; 48 bars caps 256 env @ ~6.8 GB**; env count is density-dependent (see §4).
- `:159` density "0.5–3.0개/100m²" (typo) → **{5,10,15,22,30}/100 m²** (band denominator).
- Episode length: reconcile to **`episode_len_steps=300`** (config, newest); WORKLOG's 250 is stale.

**Rewrite WRONG:**
- `navrl_bars_env.py:13-14, 46-47` docstring "radial, 41 m diagonal" → **cross-field, goal at x=k, k∈[k_min,24]** (`navrl_task.py:208-240`). Not radial, no 41 m.
- `navrl_task_config.py:17` "16 static bars" → **48** (`bar_asset_params.num_assets`, `env_object_config.py:678`).
- `navrl_task_config.py:5-13` reward docstring → the interception redesign: **alive is a −0.05 time cost, + PBRS progress, + capture +30, − collision −20, re-baselined static-safety** (not "NavRL weights").
- **Fix one density denominator** — recommend the **covered band (~501 m²)** since the spawn corridor has no bars; make `env_object_config.py:675` ("~11 bars/100 m²") and `RESEARCH_PLAN.md:120` ("8.3/100 m²") consistent to the band figure.
- `:133` "goal frame 정의가 이동 표적에서 매 스텝 갱신 필요" → **soften**: frame is fixed per episode; S_int recomputes each step against the current `target_position` (more stable). Not a requirement.

**ADD:**
- **New Phase 5 = 3D flight** (was unmapped): un-zero vertical, mixed-height obstacles, activate height reward.
- **Reorder**: swap old Phase 3 (dynamic) ↔ old Phase 4 (target) → **new Phase 3 = moving target, Phase 4 = dynamic**; renumber perception→6, paper→7. State the rationale (headline density×speed reachable after new Phase 3).
- **RQ2 headline** = density × target-speed performance map, produced at end of new Phase 3.
- **VRAM-budget section** (§4 here): feature-(3)-is-the-only-mesh-mover principle.
- **Target-speed degeneracy note**: train ≤1.5 m/s, report 2.0 as eval-only.

---

## 6. Phase summary table

| Phase | One-line | Feature | RQ | VRAM | Status |
|---|---|---|---|---|---|
| 1 | Static 2D nav, cross 48-bar field @1 m, capture-terminate — 86% (peak 94%), crash 14% | — | — | 6.8 GB @256 | ~done |
| **2** | **Static density sweep {5,10,15,22,30}/100 m² + curriculum-vs-fixed + cross-density matrix** | (3) | RQ1 | **pressured; 128 @ L5** | **NEXT** |
| 3 | **Moving target (2D, scripted); density×speed heatmap; zero-shot vs retrain** | (2) | RQ2 | neutral | planned |
| 3b | Evading target (heuristic flee / self-play) — stretch | (2)+ | RQ2 | neutral | stretch |
| 4 | **Dynamic obstacles (2D): +motion +S_dyn(5×10) +r_ds +dyn-branch +count curriculum; ≥65%** | (4) | RQ1 | neutral (GT-state) | planned |
| 5 | **3D flight (isolated, reduced density): un-zero vz, mixed short/tall bars, activate height reward** | (1) | capability | neutral | **new** |
| 6 | Perception: GT→degradation model (A) / depth-detector (B) | — | RQ3 | — | planned |
| 7 | Failure clustering, ablations, stats (≥3–5 seeds, ≥500 eps), paper | — | all | — | planned |

**Single recommended next phase after the current run: Phase 2 — static density sweep (feature 3).** Start with the 1-seed screening pass (6 fixed-density runs) to lock the curve and confirm the 150-bar VRAM ceiling before committing to the full multi-seed matrix.
