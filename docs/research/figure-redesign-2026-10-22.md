# MOTAR publication figures — source verification and full-page redesign

The user requested a complete page and figure rebuild after inspecting the first October
update. This supersedes the earlier visual scope that required retaining the old design.
Historical measurements, preregistrations and figure-package bytes are unchanged. This is
a documentation/visualization change; no experiment or algorithm was run or changed.

## Source authority

- Historical replication measurement source: `b1c0bc612339d2f8dfb86a0ca16de11bdffe09b5`,
  from `results/target_motion_replication_7seed/replication_record.json`.
- Current BOIA snapshot: `fde21cdf969a7df380aa1c4deddc1824cb524506`.
- Policy F checkpoint: `f702213936601860995cf61dcc570247e72543b1976e3716055cd8ec5593ad40`.
  Its training-source commit is UNKNOWN. Measurement and research source commits must
  never be presented as the checkpoint training source.
- No local BOIA source is copied into the public page. Its reviewed facts and hashes are
  already documented in `docs/status/data/boia-progress.json`.

Source inspection was read-only in a separate research worktree. The architecture was
checked both at the historical measurement commit and at the current research snapshot.
No simulator, GPU, PPO or checkpoint execution is part of this work.

## Architecture corrections

Paths in this table are under `aerial_gym/` at the historical measurement source above.
Line numbers refer to that commit, not the evolving checkout.

| Previous presentation problem | Verified source and mathematical contract | New figure |
| --- | --- | --- |
| Tokens shown before the flat actor observation | `task/navrl_task/navrl_perception.py::observe` (2201–2242) assembles flat data; `rl_training/rl_games/navrl_transformer_network.py::forward` (155–205) slices and embeds it. | Figures 1/2: **flat 898 → embedded 17 × 64** |
| Actor `vz` treated as an applied policy command | `navrl_task.py::transform_action_to_command` (4885–4941); z overwritten by PI altitude hold at 4934. | Figure 1: planar/yaw request; task altitude hold |
| Arc cap drawn as the baseline or a second serial filter | `speed_governor.py::apply_speed_governor`; task `_apply_sensor_speed_governor`. Policy F baseline is riskcap; arc-clearance is an alternate arm. | Figure 1: single XY cap; alternate mode labelled separately |
| Evaluation/arm grid used as the end of the control path | `control/controllers/velocity_control.py::LeeVelocityController.update` (20–97), then task `sim_env.step`. | Figure 1: Lee thrust/torques → UAV PhysX → next sensor delivery |
| Camera exclusively target / LiDAR exclusively obstacles | `_process_obs_perception` (6366–6397), perception `_detect_rgbd` (1535–1716), tracker (640–723). LiDAR assists camera-initialized target tracking; depth and LiDAR support obstacle features. | Figure 1: cross-modal perception with own state |
| Two-second history confused with tracker lifetime | `_update_histories` (2042–2050) and `navrl_task_config.py` (173, 266): five slots, nominal 0.5 s spacing; separate 5 s tracker expiry. | Figure 2: both clocks explicitly named |
| BOIA proposal drawn as connected to the vehicle | Current `motar_eval/q02/pipeline.py::ShadowRecorder` and `q02/shadow.py`: no action sink/executor; BOIA utility not estimated / ABSTAIN. | Figure 6: passive records; Frozen F keeps command ownership |

The five obstacle tokens each embed all eight obstacle slots for one history time. They
are not five individual obstacles. The own-state channel is simulator state; it does not
establish a hardware state estimator. Track active/age and camera detection do not confirm
persistent identity.

### Actor record

| Group | Shape | Flat scalars | Embedded tokens |
| --- | --- | ---: | ---: |
| Fused static scan (RGB-D + LiDAR) | 4 × 72 | 288 | 1 |
| Obstacle history | 5 × 8 × 12 | 480 | 5 |
| Own-state history | 5 × 10 | 50 | 5 |
| Target history | 5 × 16 | 80 | 5 |
| Total | concatenated | 898 | 16 + CLS = 17 |

The target features are relative position (3), relative velocity (3), position variance
(3), velocity variance (3), camera confidence, LiDAR confidence, normalized age and radius.
The own-state features are vehicle-frame velocity (3), yaw rate, previous action (4),
altitude and validity. The previous action is the actor action with governed XY recorded;
its z field is not the executed PI altitude command. Camera depth is fused into the 288-D
static scan (`navrl_perception.py:1818–1831`). Tracker state is a 6-D mean with a 6 × 6 covariance.

GT supplies scene rendering, rewards, evaluation, termination and the privileged critic
(898 + 8 = 906). It is not a direct actor target-state channel. The renderer/detector
provenance is synthetic; the Q02-A profile is `ORACLE_TRUE_POSE_RASTER`. A sensor-only
actor schema does not turn this into a validated real-image flight stack.

## Numerical corrections and plotting choices

| Issue | Correct treatment |
| --- | --- |
| Safety pooled point estimate combined with the seed-t interval | Figure 5 uses `seed_t.mean` with `seed_t.lo/hi`. The distinct pooled estimate and pooled CI remain in the evidence ledger. |
| Repeated frozen evaluations interpreted as independent training replications | Perception cost retains its episode-level interval from two evaluation seeds. Only readaptation uses the three training-seed inference. |
| High-density E2 gain implicitly uniform | Figure 4 exposes each seed. At 205 bars, E2−H is positive in four seeds, negative in two, zero in one. |
| Potentially invented density-specific intervals | No such intervals are recorded in the source; Figure 4 displays existing seed trajectories and means only. It is exploratory. |
| E3-S range-span midpoint labelled block mean range | The rebuilt page does not plot that mislabeled x variable. Its one-flight result stays in the evidence ledger. |
| P8 stacked segment widths altered by visual padding | Those stacks are not reused in this figure family. No measured probability is represented by an adjusted width. |
| Minimum-distance series with incompatible event semantics | Kept WITHHELD_SEMANTIC_MISMATCH; not plotted. |

The old immutable packages are retained for historical traceability, not used as current
page illustrations. No old raw, source record or preregistration is corrected in place.

## New figure family

1. Verified sensor-to-action architecture — actual baseline data/control topology.
2. Observation data structure — flat dimensions, network tokens and temporal contracts.
3. Independent target-motion replication — seed values and recorded paired BCa contrasts.
4. Density response — exploratory display of immutable per-cell primary counts.
5. Separate sensitivities — perception, readaptation, arc geometry and policy-R rendering;
   inference units and estimands stay separate.
6. BOIA passive architecture — allowed evidence, belief, local map, candidate screens and
   shadow timing; no command execution is implied.

Rebuild using the existing CPU plotting environment:

```bash
PYTHONNOUSERSITE=1 python tools/build_publication_figures_20261022.py
python tools/build_research_page_20261022.py
```

The first command requires matplotlib and NumPy, without simulator imports. The second
uses only the Python standard library. Output is under
`docs/assets/paper/redesign-2026-10-22/`: SVG text, embedded-font PDF, 300 dpi PNG, source/
output SHA-256 manifest and a ZIP bundle. New sources are separate from frozen figure
directories. The pages are static and readable without JavaScript; the pre-existing
browser GT preview is an independently labelled appendix.

## Research status boundary

Q02-A is reviewed with explicit identity limitations. Q02-B/Q03 independent review FAIL/HOLD
and the two unfixed P1 findings are retained. Q04 is design-only. No occlusion-accuracy,
identity, capture-benefit, online flight-safety or real-flight claim follows from these
figures. H-R remains provisional; no historical A/B/P01/P02 numbers are promoted to
long-occlusion recovery or confirmatory algorithm evidence.
