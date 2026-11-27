"""Export a checkpoint's observation normaliser for a camera-runtime contract (RT-S3W, 2026-11-28).

Writes the npz a CAM-S2 contract binds (camera_policy_contract_cams2: checkpoint.obs_norm_path): running_mean and
running_var (898 slots, float64), count, epsilon 1e-5 and clip 5.0, copied from the checkpoint's
model['_orig_mod.running_mean_std.*'] (or the uncompiled 'running_mean_std.*'), the same layout as the CAM-S2a handoff
file. The runtime re-checks at start that the npz equals the checkpoint's own running_mean_std
(CameraPolicyRuntime._check_cams2_build). CPU only; refuses to overwrite an existing file.

    python tools/camera_policy_export_obs_norm.py CHECKPOINT.pth OUT.npz
"""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

EPSILON = 1e-5          # rl_games RunningMeanStd default (the CAM-S2a handoff npz)
CLIP = 5.0              # rl_games default normalize_input clip (the CAM-S2a handoff npz)
PREFIXES = ("_orig_mod.running_mean_std.", "running_mean_std.")


def normaliser(state):
    """(running_mean, running_var, count) as float64 numpy arrays from a loaded checkpoint dict."""
    model = state.get("model") if isinstance(state, dict) else None
    if not isinstance(model, dict):
        raise ValueError("checkpoint has no model state")
    for prefix in PREFIXES:
        keys = [prefix + k for k in ("running_mean", "running_var", "count")]
        if all(k in model for k in keys):
            mean, var, count = (model[k].detach().double().cpu().numpy() for k in keys)
            if mean.shape != (898,) or var.shape != (898,) or count.shape != ():
                raise ValueError("observation normaliser must hold 898 slots")
            return mean, var, count
    raise ValueError("checkpoint has no running_mean_std")


def export(checkpoint, out):
    import torch
    out = Path(out)
    if out.exists():
        raise FileExistsError(out)
    state = torch.load(str(checkpoint), map_location="cpu", weights_only=False)
    mean, var, count = normaliser(state)
    with out.open("xb") as stream:
        np.savez(stream, running_mean=mean, running_var=var, count=count, epsilon=np.float64(EPSILON),
                 clip=np.float64(CLIP))
    digest = hashlib.sha256(out.read_bytes()).hexdigest()
    return {"checkpoint": str(checkpoint), "checkpoint_sha256": hashlib.sha256(Path(checkpoint).read_bytes()).hexdigest(),
            "obs_norm": str(out), "obs_norm_sha256": digest, "count": float(count)}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("checkpoint", type=Path)
    parser.add_argument("out", type=Path)
    args = parser.parse_args(argv)
    print(json.dumps(export(args.checkpoint, args.out), sort_keys=True))


if __name__ == "__main__":
    main()
