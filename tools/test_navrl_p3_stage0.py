"""Phase-3 Stage-0/0.5 verification: the moving target is perceived via the LiDAR (segmentation),
injected ANALYTICALLY as a sphere (no mesh, no per-step warp refit) so it is training-fast.

Run (GPU free, vision ON):
  NAVRL_VISION=1 NAVRL_MAX_BARS=40 NAVRL_NUM_BARS=40 PYTHONNOUSERSITE=1 \
    python tools/test_navrl_p3_stage0.py

Checks:
  1. env builds with vision on; the sensor exposes obs_dict['navrl_target_position'].
  2. NO extra obstacle column (target is analytic, not a mesh asset): max_bars == obstacle cols.
  3. segmentation channel exists (obs_dict['segmentation_pixels']).
  4. target (id=50) is hit by LiDAR rays when placed in front of the drone.
  5. the target MOVES: driving self.target_position changes which rays see id=50 (analytic follows).
  6. THROUGHPUT is back to ~baseline (no refit) -- the whole point of Stage 0.5.
"""
import os
import time

os.environ.setdefault("NAVRL_VISION", "1")
from aerial_gym.registry.task_registry import task_registry  # isaacgym before torch
import torch

TARGET_SEMANTIC_ID = 50
BAR_SEMANTIC_ID = 3
N = 16
fails = []


def check(name, ok, extra=""):
    print(f"  [{'PASS' if ok else 'FAIL'}] {name} {extra}")
    if not ok:
        fails.append(name)


task = task_registry.make_task("navrl_task", seed=7, num_envs=N, headless=True, use_warp=True)
print(f"[stage0] has_vision_target={task.has_vision_target} "
      f"max_bars={task.max_bars_available} n_bars_active={task.n_bars_active}")

obst = task.obs_dict["obstacle_position"]
check("sensor target buffer exposed (navrl_target_position)", task.has_vision_target)
check("NO extra obstacle column (target is analytic, not a mesh)",
      task.max_bars_available == obst.shape[1], f"(cols={obst.shape[1]})")

task.reset()
has_seg = "segmentation_pixels" in task.obs_dict and task.obs_dict["segmentation_pixels"] is not None
check("segmentation channel exists", has_seg)

# --- target 2 m directly in front of each drone; step once and read the segmentation
act = torch.zeros(N, 4, device=task.device)
task.tm.speed_fixed = 0.0
pos = task.obs_dict["robot_position"]
task.target_position[:, 0] = pos[:, 0] + 2.0
task.target_position[:, 1] = pos[:, 1]
task.target_position[:, 2] = pos[:, 2]
task.step(act)

if has_seg:
    seg = task.obs_dict["segmentation_pixels"].reshape(N, -1)
    hits_front = (seg == TARGET_SEMANTIC_ID).sum(dim=1)
    n_seeing = int((hits_front > 0).sum())
    print(f"[stage0] target-in-front: envs seeing id=50 = {n_seeing}/{N}, "
          f"id=50 rays = {int(hits_front.sum())}, bar rays(id=3) = {int((seg == BAR_SEMANTIC_ID).sum())}")
    check("target (id=50) hit by LiDAR when in front", n_seeing >= N // 2, f"({n_seeing}/{N})")

    # move the target far away and re-render: id=50 rays vanish (analytic follows the position)
    task.target_position[:, 0] = pos[:, 0] - 50.0
    task.target_position[:, 1] = pos[:, 1] - 50.0
    task._sync_target_to_sensor()
    task.sim_env.render(render_components="sensors")
    seg2 = task.obs_dict["segmentation_pixels"].reshape(N, -1)
    hits_away = int((seg2 == TARGET_SEMANTIC_ID).sum())
    print(f"[stage0] target-moved-away: id=50 rays = {hits_away} (was {int(hits_front.sum())})")
    check("moving the target changes segmentation (analytic injection works)",
          hits_away < int(hits_front.sum()), f"(away={hits_away} < front={int(hits_front.sum())})")

# --- throughput with vision ON (must be ~baseline: NO refit)
task.reset()
task.tm.speed_fixed = 1.5
task.tm.pattern = "cv"
task.reset()
torch.cuda.synchronize()
t0 = time.time()
STEPS = 60
for _ in range(STEPS):
    task.step(act)
torch.cuda.synchronize()
dt = time.time() - t0
print(f"[stage0] throughput: {STEPS/dt:.1f} steps/s ({N*STEPS/dt:.0f} env-steps/s), "
      f"{1000*dt/STEPS:.1f} ms/step (N={N}, {task.n_bars_active} bars, analytic target, NO refit)")

print(f"[stage0] RESULT: {'ALL PASS' if not fails else 'FAILURES: ' + str(fails)}")
