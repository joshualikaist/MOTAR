# MT-P1: opt-in target distance precision, CPU implementation and future canary

Status: **CPU implementation only. No GPU run is authorized.** This is a new lineage,
not an amendment to a historical result or a claim that moving-target simulation is now valid.
The machine-readable companion is `mt_p1_precision_v1.json`. Review and commit the final
source and plan before requesting the separate run below. Bind the resulting commit, source
manifest and plan hashes in an external launch receipt; never edit this plan to insert its own
commit hash after committing it.

## Change and boundaries

`NAVRL_TARGET_DISTANCE_BACKEND` defaults to `legacy_cdist`. With explicit `direct_l2_v1`,
bounded E1/E2 candidate-to-bar distances use a subtraction followed by a norm, avoiding the
matmul formulation of `torch.cdist`. This does not change the process-wide TF32 flags, actor,
checkpoint, motion bounds, candidate set, ranking, governor, rewards or termination. The old
H/E0/E1/E2 labels and recorded results remain historical. New evaluation labels are
`E1_direct_l2_v1` and `E2_direct_l2_v1`; their recorded motion model is
`bounded_planar_drone_v1_rollout_direct_l2_v1`. The frozen launcher's old E1/E2 arm names are
execution aliases only and must never be presented as evidence under the old lineage.

The opt-in does **not** fix an unsafe initial state or make the executor's longest-safe-prefix
fallback a safety guarantee. It does not certify swept collision avoidance: the executor's
virtual-target center-distance checks are complemented by an independent swept-surface monitor.
An infeasible first step or any collision counterexample remains a failure to investigate.

## CPU verification and historical evidence

The new tool imports `target_motion.py` by file path; it never imports `aerial_gym`, queries
CUDA availability, initializes CUDA, calls the simulator or launches another process. Default
output is JSON on stdout. `--out` accepts a new file outside this checkout's immutable results
tree and never overwrites an existing file. Run with the pinned CPU-capable Python:

```bash
CUDA_VISIBLE_DEVICES='' PYTHONNOUSERSITE=1 /home/fair/miniconda3/envs/aerialgym/bin/python -B \
  tools/kroc2027_motion_verify.py --device cpu --suite all \
  --packets-root /home/fair/workspaces/aerial_gym_ws/src/aerial_gym_simulator/results/kroc2027_moving_target_packets_v1 \
  --out docs/plans/mt_p1_cpu_validation_v1.json
```

The recorded invocation used the CLI arguments above; the reproduction command additionally
hides CUDA devices as an environment guard. The new machine-readable CPU record is
`docs/plans/mt_p1_cpu_validation_v1.json` (SHA-256
`9ffd0bb2e8c2d2723071c3d0824d1a1d3d432b223d8d3e8467e5b4de3f8087d6`).
The record embeds tool SHA-256
`471a93c2a1aebf5f4555059f88a3656bb8ba75ac704667a755b5db995dd4242f`,
motion/gate source hashes, command arguments, and actual gate tolerances. The existing report
will be refused on repetition: omit `--out` for stdout, or choose a different new report path.
Measured CPU findings: 1,280 synthetic environment-steps with zero reported violations and
zero candidate-ranking or position disagreements against the independent distance oracle;
606/606 historical starts already inside their center-clearance envelope and 606/606 recorded
executed sweeps violating the surface proxy; no recorded kinematic violation. Both CPU sign
hypotheses have zero feasible first steps; neither reconstructs missing historical inputs.

CPU synthetic fixtures cover the actual experiment densities 70/115/160/205 and stress counts
0/40/80/120/160, two turn signs and 80 steps each. The tool checks speed, acceleration, heading,
position correction, displacement speed, walls, nonfinite state, and continuous segment-to-AABB
surface clearance; endpoint-only geometry is insufficient. It compares candidate ranking and
selected positions with a float64 subtraction-distance oracle. These are small safe-start
fixtures, **not** simulator rollouts, production scene coverage, achieved GPU performance or a
replacement for the future canary. CPU pass cannot establish identical GPU trajectories.

The frozen packet set has 606 violations. The CPU report hashes every original packet file,
recomputes initial center clearance, actual swept clearance and kinematic bounds, and evaluates
both turn-sign hypotheses using local bars plus explicitly labeled filler slots. It reports
unsafe starts separately rather than expecting the precision change to rescue them.

`tf32_replay.json`'s 598/606 CUDA-TF32 batch-2 executed-position matches are **imported historical
measurements**, not newly recomputed GPU results. The remaining eight stay `UNRESOLVED_INPUTS`.
Old packets do not contain complete ordered bars, original batch/tensor layout, turn sign and
all candidate scores. Neither the 598 matches nor a CPU local-bar reconstruction proves a full
historical replay. The frozen replay stores aggregate tallies only, so the eight unresolved
packet identities are themselves `NOT_RECORDED`; no particular eight rows are invented.
A packet diagnostic exits successfully when these limitations are recorded;
its status is `INCOMPLETE_INPUTS`, never a motion-validity PASS.

## Full input packets for future counterexamples

Future runs enable `NAVRL_KROC2027_PACKETS=1` and `NAVRL_KROC2027_PACKET_SCHEMA=full_v2`.
The content-addressed executor-call bundle must include full ordered obstacle tensors and slots,
all environment rows and row IDs, actual input dtype/device/shape/stride/storage offset, turn
sign, velocity and desired velocity, bounds, backend and motion-model identity, actual executor
kwargs, and actual output plus selected candidate/certificate. The violation row points to its
bundle SHA and executor row. Preserve it even when a motion gate aborts the cell. This makes
future reproduction auditable; it cannot fill missing inputs in the old packets.

## Separate future approval: MT-P2-C01

Purpose: test the new precision lineage in the actual RTX 3070 runner with its normal TF32
state, using the existing G1/G2 and swept-clearance gates. No policy training or performance
effect estimation. Seed 4190 is deliberately reused for a diagnostic canary, not called fresh.

Design: 2 lineages × visible/hidden A3/A4 × densities 70/115/160/205 = **16 cells**;
128 environments × first four completed episodes = 512 primary episodes/cell, 8,192 total;
600 actions at 0.1 s. Keep all tail episodes but exclude them from the quota estimand. E1 and
E2 must be reported separately. A passing precision canary does not automatically restore H-A2
or authorize an A main run, R0–R3, a speed treatment or new seeds.

Checkpoint SHA-256:
`f702213936601860995cf61dcc570247e72543b1976e3716055cd8ec5593ad40`.
Read-only primary checkpoint:
`/home/fair/workspaces/aerial_gym_ws/src/aerial_gym_simulator/aerial_gym/rl_training/rl_games/runs/ppo_260805_0413_navrl_v2-speedgov-ep24000-205bars-main-riskcap-s1/nn/last_gen_ppo_ep_25000_rew_39.742134.pth`.
Preserve the original bytes and never overwrite or edit either checkpoint. An exclusive-create,
byte-identical copy (8,869,844 bytes, same SHA above) now exists at the same relative path in
`/home/fair/workspaces/aerial_gym_ws/src/MOTAR-mt-p1`, so the frozen launcher can resolve it from
the reviewed worktree. No checkpoint was loaded with torch and no original asset was modified.

Static preparation is recorded in `docs/plans/mt_p1_asset_preflight_v1.json` (SHA-256
`182d60ad1126060fb10fa21ae4d3d42c6b1fa105a1245d629aff51a162ca1ad1`). Its 45 hashed files cover
the checkpoint, frozen launcher/player, Transformer YAML, `quad_navrl_collide.urdf` and all 40
`bars_h3` URDFs; all match primary bytes. These URDFs use primitive geometry, with no external
mesh dependency. The default NavRL perception path uses code-initialized appearance weights;
learned detector/profile files are optional overrides and must be absent in the future clean
run environment. VAE weights belong to `NavigationTask`, not this NavRL task, and are not needed.
`PathFinder.find_spec` resolves `aerial_gym` and `motar_eval` into this worktree, and locates
Isaac Gym, torch, numpy, Warp, rl_games, trimesh and urdfpy without importing those packages.
This is a static path/asset check, **not** a CUDA probe, runtime import test or simulator canary.

Expected cost is **0.75–1.5 RTX 3070 hours**, hard cumulative cap **2.0 hours**. This is a
prediction from prior diagnostic cells, not measured direct-backend throughput. Full input
bundles and direct-distance tensor arithmetic may add cost. The operator must meter cumulative
time before every cell and stop on the first failed/incomplete cell or budget exhaustion.

Before GPU execution, fill and review the external receipt's source commit, source manifest SHA,
plan SHA, clean tracked
tree, Python/import origins, GPU identity, checkpoint SHA and the exact output path. A future
approved execution root must contain that reviewed source and the existing checkpoint layout;
the frozen shell computes paths from its own location. Running the primary shell while importing
a different worktree would require a separate two-root source receipt contract and is not approved
by this plan. Asset/import-origin preflight must pass before the future GPU approval; preserve
all original checkpoint and result bytes while preparing the execution layout.

Concrete **future single-cell** command (repeat only the predeclared 16-cell matrix after approval,
with an operator that checks the preceding cell and cumulative budget; this is not a new runner):

```bash
# Future only. Run from the approved source root after its source/asset preflight passes.
env PYTHONNOUSERSITE=1 PYTHONPATH="$PWD" \
  NAVRL_TARGET_DISTANCE_BACKEND=direct_l2_v1 \
  NAVRL_KROC2027_RECORDER=1 NAVRL_KROC2027_PACKETS=1 \
  NAVRL_KROC2027_PACKET_SCHEMA=full_v2 NAVRL_KROC2027_ON_VIOLATION=stop \
  NAVRL_KROC2027_INITIAL_VIEW=visible NAVRL_KROC2027_INITIAL_DISTANCE_M=10 \
  NAVRL_KROC2027_RUN_ID=MT-P2-C01 NAVRL_KROC2027_ARM=A3 \
  NAVRL_EPISODE_LEDGER=1 NAVRL_EPISODE_QUOTA_PER_ENV=4 \
  NAVRL_PLAYER_GAMES_CAP=1000000000 \
  ARMS=E1_cv DENSITIES=70 SEEDS=4190 GAMES=512 NUM_ENVS=128 \
  OUT_ROOT="$PWD/results/mt_p1_precision_v1/MT-P2-C01/E1_direct_l2_v1/A3" \
  bash aerial_gym/rl_training/rl_games/eval_navrl_target_behavior_arms.sh
```

The frozen launcher SHA remains
`4077e2718c73c3177cdf1f775cc11a6b50cb18d591e352382e175c45fb14f7ec`.
It neither sets nor unsets `NAVRL_TARGET_DISTANCE_BACKEND`, so it preserves the explicit opt-in.
Its internal failed-cell `continue` is why each invocation here contains one cell; an operator
must inspect its receipt, because shell exit status alone does not certify cell success.

For E2 use `ARMS=E2_obstacle_aware`, label/output `E2_direct_l2_v1`; for A4 use hidden/A4;
substitute each of the four declared densities. The future receipt must bind lineage aliases,
actual backend, source and hashes. A repeated attempt gets a new run/output identity, preserving
the original incomplete result.

Acceptance: all 16 cells and exact quotas complete, actual backend/model identity correct,
zero G1/G2/nonfinite/wall/swept-surface violations, and all source, packet, sidecar, ledger,
import-origin and receipt checks pass. Stop immediately on any failure and retain the first
counterexample plus full inputs. Never replace a seed or relax a bound after inspecting results.
If no safe first candidate exists despite a valid start, record that independent fallback problem;
do not conceal it behind the numerical fix. GPU validity is currently **NOT_TESTED**.
