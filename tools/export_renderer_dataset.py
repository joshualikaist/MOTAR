#!/usr/bin/env python3
"""Deterministic static generic graphics export; no target/UAV asset or detector integration."""
import argparse
import hashlib
from pathlib import Path


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--device", choices=("cpu", "cuda:0"), default="cpu")
    p.add_argument("--geometry", choices=("boxes", "background"), default="boxes")
    p.add_argument("--width", type=int, default=160)
    p.add_argument("--height", type=int, default=90)
    p.add_argument("--num-scenes", type=int, default=1)
    p.add_argument("--frames", type=int, default=1)
    p.add_argument("--material-seed", type=int, default=0)
    p.add_argument("--light-seed", type=int, default=0)
    p.add_argument("--position", type=float, nargs=3, default=[0, 0, 0])
    p.add_argument("--quaternion", type=float, nargs=4, default=[0, 0, 0, 1])
    p.add_argument("--instance-offset", type=int, default=0, help="Debug label renumbering only")
    args = p.parse_args(argv)
    if args.output.exists():
        raise FileExistsError("Output must not exist")
    from renderer_validation.public_pipeline import (Pipeline, budget, provenance, verify_source,
                                                     write_json, array_record, isolated)
    from runtime_fingerprint import runtime_fingerprint
    import numpy as np
    budget(args.width, args.height, args.num_scenes, args.frames)
    if min(args.material_seed, args.light_seed) < 0:
        raise ValueError("Seeds must be nonnegative")
    source = provenance()
    config = vars(args).copy()
    config["output"] = str(config["output"])
    record = {"schema": "generic_renderer_export_v1", "source": source, "config": config,
              "status": "INCOMPLETE", "experiment_verdict": "NOT_EVALUATED",
              "training": "NOT_SUPPORTED", "files": []}
    args.output.mkdir(parents=True, exist_ok=False)
    write_json(args.output / "request.json", record)
    try:
        pipeline = Pipeline(args.geometry, args.width, args.height, args.num_scenes, args.device,
                            args.material_seed, args.light_seed, args.position, args.quaternion,
                            args.instance_offset)
        record["description"] = pipeline.description()
        record["runtime"] = runtime_fingerprint(include_device=args.device != "cpu")
        record["runtime"]["warp"] = pipeline.renderer.wp.__version__
        for i in range(args.frames):
            arrays = pipeline.frame()
            path = args.output / ("frame_%04d.npz" % i)
            with path.open("xb") as stream:
                np.savez_compressed(stream, **arrays)
            record["files"].append({"file": path.name, "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
                                    "arrays": array_record(arrays)})
        isolated()
        verify_source(source)
        record["status"] = "EXPORTED_UNASSESSED"
        write_json(args.output / "receipt.json", record)
    except BaseException as exc:
        record.update(status="FAILED_INCOMPLETE", error_type=type(exc).__name__)
        write_json(args.output / "failure.json", record)
        raise
    print("EXPORTED_UNASSESSED " + str(args.output))


if __name__ == "__main__":
    main()
