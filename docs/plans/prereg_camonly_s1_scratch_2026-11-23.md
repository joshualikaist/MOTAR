# Preregistration: CAM-S1 v1 — a camera-only policy trained from scratch (training only)

Written 2026-11-23, before any CAM-S1 run. Branch `camonly/dev`. Family `camera-only` (docs/EXPERIMENTS.md).
User decision D-2026-11-23-1 (docs/DECISIONS.md). Design review r1 (Claude Opus 5.5) AMEND_FIRST and reward review r1
(Claude Fable 5.1) SOUND_WITH_CHANGES were answered in this version (section "Reviews").

## Question

Can a policy that never receives a LiDAR input — trained from epoch 0 with the 66 x 54 deg camera, the real vehicle's
treatments and a "fly where you look" rule — learn the moving-object approach task within a reduced budget of 12,000
epochs? This unit trains and records the training. It makes no performance claim; an evaluation is a separate
preregistration (CAM-S1-E) written before it runs.

Why from scratch (user, 2026-11-23): every earlier camera-only policy (CAM-K1, CAM-K2 v2/v3) continues Policy F, whose
weights and habits were learned with a 360-degree LiDAR. Nothing in the records shows that the continuations' remaining
crashes are inherited LiDAR habits (design review r1, A); this run is the lineage that does not inherit them.

## Approval and scope

The user approved on 2026-11-23: "처음부터 학습 시작해", with a reduced budget ("학습량을 조금 줄여서"), the 898-D
layout kept, lowered curriculum gates with a stop rule, and about 12,000 epochs. Scope: one development smoke
(2 epochs, not a result), then ONE training run of seed 1, at most 12,000 epochs and 17 h of wall time. No evaluation,
no second seed, no continuation and no flight use are in this scope.

## Settings

Policy F's fresh-training launcher contract (`aerial_gym/rl_training/rl_games/train_navrl_v2_search.sh`, fresh mode:
PPO `ppo_navrl_perception_transformer.yaml`, LR 3e-5, 128 environments, 600-step episodes, legacy target dynamics,
mixed pattern, distance and FOV curricula, 898-D actor / 906-D critic), run through
`aerial_gym/rl_training/rl_games/train_navrl_cams1.sh` and `tools/cams1_train.py`, with these differences:

| Item | Policy F (fresh) | CAM-S1 |
| --- | --- | --- |
| Start | random initialisation | random initialisation (no checkpoint) |
| Sensors | camera 87 x 58 deg + LiDAR 72 x 4 | camera only from epoch 0 (`NAVRL_SENSOR_ABLATION=no_lidar`), 66 x 54 deg |
| Reward: yaw_align_weight, visibility_bonus | 0.3, 0.02 | 0.6, 0.03 (CAM-K1's) |
| Reward: collision penalty | -20 | -40 |
| Reward: static-safety range | 12 m | 4 m (`NAVRL_CAMSAFE_RSS_CLIP_M`) |
| Reward: blind-motion penalty | none | 0.5 x excess x gate (below) |
| Rule: blind-direction cap | none | horizontal command outside +-33 deg scaled to <= 1.0 m/s |
| Speed governor | off until ep24000, then riskcap | riskcap from epoch 0, screened parameters except FREE 3.5355 -> 2.5 m/s |
| Vehicle | navrl_quad | navrl_quad, per-motor limit at T/W 4.0 (INFERRED), thrust +-15 %, imbalance +-5 %, tau 0.02-0.04 s |
| Depth | exact | OAK-D Pro model (MinZ 0.4 m, MaxZ 20 m, sigma_d 0.15 px, holes 0.02) + validity rule `blob_near` |
| Detection | colour rule, no error | colour rule + P9 empirical error injector (detection-loss placeholder, below) |
| Density curriculum | 70 -> 205, gates `70:0.82,85:0.77,100:0.72,115:0.70`, stage minimum 1000, warm-up 1000 | 70 -> 205, gates `70:0.284,85:0.266,100:0.249,115:0.242` (start 0.277, end 0.242), stage minimum 500, warm-up 500 |
| Epochs | 25,000 | at most 12,000 |
| Checkpoints | every 50 epochs | every 250 epochs |
| Training seed | 1 | 1 (vehicle 4931, depth 4941, P9 4951) |

The blind-motion penalty (alive environments only): `0.5 * max(0, |bearing(v)| - 33 deg) / (180 deg - 33 deg) *
clamp(|v| / 1.0 m/s, 0, 1)`, v the measured vehicle-frame horizontal velocity. It is zero inside the camera's view.

All additions are opt-in code: `aerial_gym/task/navrl_task/navrl_camsafe.py` (rule, penalty, collision penalty,
static-safety range), installed by `tools/realveh_entry.py` only when `NAVRL_CAMSAFE=1`, after the sensor ablation.
`navrl_task.py` and `navrl_perception.py` are unchanged (byte-pinned). With `NAVRL_CAMSAFE` unset, `realveh_entry`
behaves as before (CAM-K2 receipts unchanged; tests/test_camsafe.py, tests/test_realveh.py).

## Why each change (with its evidence and limits)

- **Blind-direction cap (a flight-safety rule).** Policy F flies outside its FOV in 51.8 % of moving frames; without
  LiDAR its contacts are "consistent with" bars outside the view (C2 out_fov +39.25 pp, G3 BLIND_SIDE_SUPPORTED). For
  the trained CAM-K2 v3, CAM-T1 found most contacts in view by the footprint profile (67.7 %), the camera inside MinZ
  at every valid contact, and no crash change from lower global caps. So the cap is "do not move fast where the camera
  cannot see"; it is not claimed to lower crashes.
- **Governor FREE 2.5 m/s.** The per-axis 2.5 m/s action box makes off-axis directions faster (2.98 m/s at the 33 deg
  FOV edge); with FREE 2.5 no direction is faster than straight ahead (reward review r1, recommendation 2).
- **Static-safety range 4 m.** With the 12 m range the static-safety term is a standing cost of about -0.34 / -0.87
  per step at 70 / 205 bars (synthetic estimate, reward review r1 table A), which makes crashing at once (-20) worth more
  than hovering (about -24 / -77 discounted, gamma 0.99). With NavRL's own 4 m range hovering at 205 bars is about -17
  and the near-bar gradient is unchanged. The earlier argument "-0.05 x 600 = -30 > -20" was undiscounted and is
  withdrawn (review r1 C2 REFUTED).
- **Collision penalty -40.** Kept as a margin on top of the 4 m range (-17 versus -40 at 205 bars instead of -17 versus
  -20). Risk: exploratory contacts cost more, which pushes towards the documented early hover trap; monitored (below).
- **Blind-motion penalty 0.5.** Auxiliary (about 2 % of the closing-speed term at 45 deg after scaling); the cap does
  the main work. No standing income (zero at rest).
- **P9 from epoch 0.** It turns perfect red-sphere detection into intermittent detection. On this 160 x 90 image it
  is a **detection-loss placeholder**, not a drone-recognition model: frames beyond 2.31 m fall into its smallest
  supported size bin, more than 99 % of its latency samples round to zero steps at 10 Hz, and it produces no false
  detections when the object is out of view (design review r1 P1-3). In flight a real image detector would replace the
  colour rule; that detector does not exist in the runtime yet.
- **Gate calibration.** F's gates sit 2 pp below F's own LiDAR training ceiling at 70 bars (0.843); camera-only
  training would very likely never pass them and would stay at 70 bars silently (design review r1 P1-1). The camera-only
  gates are F's schedule x RATIO, RATIO = mean(26.73, 28.51) / 79.84 = 0.3459: CAM-K2 v3's last-window training-rollout
  capture at 205 bars (docs/evidence/camonly_2026-11-04/camk2/README.md line 96) over the F-lineage LiDAR reference in
  the same setting (GR-1 ctrl, docs/evidence/camonly_2026-11-04/camk1/README.md line 85). Limit: one ratio from 205 bars
  is applied at every density; no camera-only rollout record exists at 70-190 bars.
- **Budget.** About 1.3 h per 1,000 epochs (CAM-K1 14,499 s / 3,000; CAM-K2 v2 4,816 s / 1,000), so 12,000 epochs is
  about 16 h against the 17 h wall cap. F needed about 19,000 epochs to reach 205 bars with LiDAR (design review r1 B),
  so this run may end below 205 bars; that is a recorded outcome, not a failure.

## Stop rule (preregistered, automatic)

`tools/cams1_train.py` parses the launcher log. Once, at epoch 3,000: if the density is still 70 bars and the
episode-weighted training-rollout capture of the last 200 epochs is below 0.10, the run is stopped (SIGINT, then
SIGTERM after 120 s) and labelled STOPPED_NO_LEARNING. The existing NaN fail-fast and same-density guard of the
launcher stay on.

## Labels (training record only)

- **TRAINED:** the process ended at 12,000 epochs with checkpoints written, host faults unchanged.
- **STOPPED_NO_LEARNING:** the stop rule fired.
- **WALL_CAP:** the 17 h cap ended the run before 12,000 epochs; the last checkpoint is recorded.
- **TRAINING_FAIL:** any other non-zero exit.
- **VOID:** host fault count rose during the run (AGENTS.md "Host faults void a run").

Recorded with the label: final epoch and density stage, the epochs at which each density stage was entered, the
last-100-epoch training-rollout capture / crash / timeout, the `[camsafe]` counters (capped fraction, penalised
fraction, mean penalty, mean clipped r_ss), wall time and s/epoch, checkpoint SHA-256 of the last and of every
1,000th epoch. Training-rollout rates are under exploration noise and are not evaluation results.

Monitoring during the run (reported, no gate): time-out rate and mean speed (hover trap), crash rate (crash seeking),
`[camsafe]` capped_frac rising towards 1 (edge riding or blind motion), density stage progress.

## Not claimed

No evaluation number, no comparison with F or CAM-K2 v3, no drone recognition (P9 is a placeholder), no latency
robustness (trained at zero latency; CAM-T1 read one-step delay WORSE), no seed generality (one seed), no flight
readiness. The flight runtime cannot load a CAM-S1 checkpoint (contract pins CAM-K2 v3), has no blind cap, and would
switch P9 on from the copied training environment (design review r1 P0-F); those are a separate unit. The user decided
that the flight of 2026-11-24 is manual with recording, not autonomous (D-2026-11-23-1).

## Reviews

- Design review r1, Claude Opus 5.5, fresh context, read-only: AMEND_FIRST. P1-1 gates (answered: calibrated gates +
  stop rule), P1-2 no mid-run evaluation (answered: evaluation after the run, separate preregistration), P1-3 P9
  wording (answered), P1-4 full-stack smoke (answered: smoke before the run, counters must be non-zero), P2-1 rationale
  (answered), P2-2 cap cliff (open, runtime must match), P2-4 stop check (answered), P2-5 latency (open, not claimed),
  P2-6 one seed (not claimed), P3 counters on device (answered), install-order test with the real ablation (answered).
  Copy: docs/evidence/camonly_2026-11-04/cams1/REVIEW_design_r1.md.
- Reward review r1, Claude Fable 5.1, fresh context, read-only: SOUND_WITH_CHANGES. Recommendations 1 (4 m static-safety
  range), 2 (FREE 2.5) and 3 (keep 0.5 and 1.0, monitor) adopted. Copy:
  docs/evidence/camonly_2026-11-04/cams1/REVIEW_reward_r1.md.
