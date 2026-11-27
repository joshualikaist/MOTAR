"""CPU-only synthetic observation replay; no simulator, raw replay or calibration."""
import argparse
import math
from pathlib import Path
import shutil
import sys
from types import SimpleNamespace

import numpy as np
import torch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from motar_eval.boia.types import BoiaParams  # noqa: E402


def fixture_params():
    """Explicit synthetic values, not fitted sensor/dynamics parameters."""
    return BoiaParams(H=2, S=2, M=8, tau_on=.1, r_bar_est=.2,
                      R_camera_diag=(.04, .04, .04))


def make_scene(name, n=1, dtype=torch.float32):
    """Return (task, perception, aF), with only declared synthetic online inputs.

    ``clear`` deliberately has no target return: it does not validate the known
    target-first-hit/known-free endpoint problem (motar-3p3.13). ``narrow_gap``
    is a 0.55 m point-aperture fixture, not a traversability or real-bar claim.
    The blocked ``bar`` has camera loss; prior camera acquisition is not asserted.
    """
    if name not in ("clear", "bar", "narrow_gap") or not isinstance(n, int) or n <= 0:
        raise ValueError("choose clear/bar/narrow_gap and positive n")
    if dtype not in (torch.float32, torch.float64):
        raise TypeError("fixture dtype must be float32/float64")
    quat = torch.zeros(n, 4, dtype=dtype)
    quat[:, 3] = 1
    scan = torch.full((n, 288), 12., dtype=dtype)
    angles = math.pi - torch.arange(72, dtype=dtype) * (2 * math.pi / 72)
    obs = {"robot_position": torch.zeros(n, 3, dtype=dtype),
           "robot_linvel": torch.zeros(n, 3, dtype=dtype),
           "robot_vehicle_orientation": quat, "robot_body_angvel": torch.zeros(n, 3, dtype=dtype)}
    action = torch.zeros(n, 4, dtype=dtype)
    action[:, 0] = .2
    if name == "bar":
        obs["robot_linvel"][:, 0] = 2.
        action[:, 0] = .8
        for row, elevation in enumerate((20., 10., 0., -10.)):
            scan[:, row * 72 + 36] = .95 / math.cos(math.radians(elevation))
    elif name == "narrow_gap":
        distance = .275 / math.sin(math.radians(5))
        scan[:, 2 * 72 + 35] = scan[:, 2 * 72 + 37] = distance
    task = SimpleNamespace(obs_dict=obs, _camera_visible_now=torch.ones(n, dtype=torch.bool),
                           _fixture_scan=scan, task_obs={"states": torch.zeros(n, 906, dtype=dtype)})
    if name == "bar":
        task._camera_visible_now.zero_()
    task._lidar_distance_m = lambda: task._fixture_scan
    task.target_position = torch.full((n, 3), 999., dtype=dtype)
    obs["navrl_target_position"] = task.target_position.clone()
    obs["segmentation_pixels"] = torch.ones(n, 2, 3, dtype=dtype)
    obs["obstacle_position"] = torch.full((n, 4, 3), 999., dtype=dtype)
    task.detector = SimpleNamespace(target_mask=torch.ones(n, 2, 3, dtype=dtype),
                                    target_depth=torch.ones(n, 2, 3, dtype=dtype),
                                    _targets=torch.full((n, 3), 999., dtype=dtype))
    mu = torch.zeros(n, 6, dtype=dtype)
    mu[:, 0] = 6.
    tracker = SimpleNamespace(state=mu, cov=torch.eye(6, dtype=dtype)[None].repeat(n, 1, 1) * .01,
                              active=torch.ones(n, dtype=torch.bool), age=torch.zeros(n, dtype=dtype))
    hfov, vfov = math.radians(87.), math.radians(58.)
    perception = SimpleNamespace(tracker=tracker, _lidar_angles=angles, hfov=hfov, vfov=vfov,
        fx=160 / (2 * math.tan(hfov / 2)), fy=90 / (2 * math.tan(vfov / 2)), cx=79.5, cy=44.5,
        camera_offset=torch.tensor([[.1, 0., .03]], dtype=dtype), target_radius=.15,
        max_camera_range=20., lidar_max_range=12., min_pixels=2)
    return task, perception, action


def run_replay(steps=128, *, scene="clear", n=1, seed=20261019, terminal_at=None, params=None):
    """Advance BOIA state over repeated synthetic observations, not plant dynamics."""
    from motar_eval.boia.planner import BoiaPlanner
    if not isinstance(steps, int) or steps <= 0:
        raise ValueError("steps must be positive")
    task, perception, aF = make_scene(scene, n)
    planner = BoiaPlanner(fixture_params() if params is None else params)
    generator = torch.Generator(device="cpu")
    generator.manual_seed(seed)
    actions, traces = [], []
    for k in range(steps):
        if shutil.disk_usage(ROOT).free < 3_000_000_000:
            raise RuntimeError("free disk is below the user-approved 3 GB floor")
        terminal = torch.full((n,), k == terminal_at, dtype=torch.bool)
        action, decision = planner.step(task, perception, aF, terminal, generator)
        if not bool(torch.isfinite(action).all()):
            raise ValueError("nonfinite synthetic action")
        actions.append(action.detach().cpu().numpy().copy())
        traces.append(decision.trace)
        planner = planner.with_state(decision.new_state)
    return {"schema": "motar.boia.synthetic-replay.v1", "scene": scene,
            "synthetic_static_observation_replay": True, "calibrated": False,
            "known_target_first_hit_limitation": "motar-3p3.13; NOT_FIXED",
            "actions": np.stack(actions), "traces": traces}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", choices=("clear", "bar", "narrow_gap"), default="clear")
    parser.add_argument("--steps", type=int, default=128)
    parser.add_argument("--n", type=int, default=1)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    output = args.output.resolve()
    if Path("/tmp/codex_occ") not in output.parents or output.suffix != ".npz":
        parser.error("output must be a new .npz under /tmp/codex_occ/")
    if shutil.disk_usage(ROOT).free < 3_000_000_000:
        raise RuntimeError("free disk is below the user-approved 3 GB floor")
    result = run_replay(args.steps, scene=args.scene, n=args.n)
    arrays = {"actions": result["actions"], "scope": np.array("SYNTHETIC_ONLY_NOT_CALIBRATED")}
    for k, trace in enumerate(result["traces"]):
        arrays.update({"step%04d__%s" % (k, key): value for key, value in trace.items()})
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("xb") as handle:
        np.savez_compressed(handle, **arrays)


if __name__ == "__main__":
    main()
